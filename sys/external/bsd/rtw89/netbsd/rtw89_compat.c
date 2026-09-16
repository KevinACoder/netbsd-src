/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by the RK3568 lab (NetBSD/RTL8821CU bring-up).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Runtime side of the rtw89 compatibility layer: sk_buff, workqueue,
 * completion and firmware(9) plumbing, plus the mac80211 entry points the
 * chip code calls.
 *
 * Everything the chip code runs here is invoked from thread context (the
 * rtw89 workqueue or the driver's own task), never from an interrupt
 * handler: the register accessors in the transport sleep.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/kthread.h>
#include <sys/queue.h>
#include <sys/callout.h>
#include <sys/workqueue.h>
#include <sys/device.h>
#include <sys/errno.h>

#include <dev/firmload.h>

#include "rtw89_compat.h"
#include <linux/firmware.h>

#include "rtw89_mac80211.h"
#include "rtw89_glue.h"

/* ------------------------------------------------------------------ */
/* sk_buff                                                             */
/* ------------------------------------------------------------------ */

void *
rtw89_skb_alloc(unsigned int size, gfp_t gfp)
{
	struct sk_buff *skb;

	/* one allocation for the header and the data area */
	skb = kmem_zalloc(sizeof(*skb) + size, (int)gfp);
	skb->head = (unsigned char *)(skb + 1);
	skb->data = skb->head;
	skb->tail = 0;
	skb->end = size;
	skb->len = 0;
	skb->truesize = sizeof(*skb) + size;
	return skb;
}

void
rtw89_skb_free(struct sk_buff *skb)
{

	if (skb == NULL)
		return;
	kmem_free(skb, sizeof(*skb) + skb->end);
}

void
rtw89_skb_queue_init(struct sk_buff_head *q)
{

	q->next = (struct sk_buff *)q;
	q->prev = (struct sk_buff *)q;
	q->qlen = 0;
}

void
rtw89_skb_queue_tail(struct sk_buff_head *q, struct sk_buff *skb)
{
	struct sk_buff *head = (struct sk_buff *)q;
	struct sk_buff *tail = head->prev;

	skb->next = head;
	skb->prev = tail;
	skb->list = q;
	tail->next = skb;
	head->prev = skb;
	q->qlen++;
}

void
rtw89_skb_queue_head(struct sk_buff_head *q, struct sk_buff *skb)
{
	struct sk_buff *head = (struct sk_buff *)q;
	struct sk_buff *first = head->next;

	skb->next = first;
	skb->prev = head;
	skb->list = q;
	first->prev = skb;
	head->next = skb;
	q->qlen++;
}

struct sk_buff *
rtw89_skb_dequeue(struct sk_buff_head *q)
{
	struct sk_buff *head = (struct sk_buff *)q;
	struct sk_buff *skb = head->next;

	if (skb == head)
		return NULL;
	rtw89_skb_unlink(skb, q);
	return skb;
}

void
rtw89_skb_unlink(struct sk_buff *skb, struct sk_buff_head *q)
{

	skb->prev->next = skb->next;
	skb->next->prev = skb->prev;
	skb->next = NULL;
	skb->prev = NULL;
	skb->list = NULL;
	if (q != NULL && q->qlen > 0)
		q->qlen--;
}

void
rtw89_skb_queue_purge(struct sk_buff_head *q)
{
	struct sk_buff *skb;

	while ((skb = rtw89_skb_dequeue(q)) != NULL)
		rtw89_skb_free(skb);
}

/* ------------------------------------------------------------------ */
/* workqueue                                                           */
/* ------------------------------------------------------------------ */

/*
 * Deferred work runs on a private thread rather than workqueue(9): work items
 * are queued from USB completion callbacks, which run in softint context and
 * must not take a sleeping mutex.  The pending list is guarded by a spin
 * mutex and the worker is poked with wakeup(), both of which are safe from
 * any context.
 */
struct rtw89_work_item {
	SIMPLEQ_ENTRY(rtw89_work_item) wi_entry;
	struct work_struct *wi_work;
};

static SIMPLEQ_HEAD(, rtw89_work_item) rtw89_pending =
    SIMPLEQ_HEAD_INITIALIZER(rtw89_pending);
static kmutex_t rtw89_pending_mtx;
static bool rtw89_worker_started;
static bool rtw89_ready;

static void
rtw89_workqueue_worker(void *arg)
{
	struct rtw89_work_item *item;
	struct work_struct *w;

	for (;;) {
		mutex_enter(&rtw89_pending_mtx);
		item = SIMPLEQ_FIRST(&rtw89_pending);
		if (item != NULL)
			SIMPLEQ_REMOVE_HEAD(&rtw89_pending, wi_entry);
		mutex_exit(&rtw89_pending_mtx);

		if (item == NULL) {
			/* wakeup() from the producer cuts this short */
			kpause("rtw89wq", false, 1, NULL);
			continue;
		}

		w = item->wi_work;
		kmem_free(item, sizeof(*item));

		/* the item may requeue itself from inside wk_func() */
		w->wk_queued = 0;
		atomic_store_relaxed(&w->wk_running, 1);
		w->wk_func(w);
		atomic_store_relaxed(&w->wk_running, 0);
	}
}

void
rtw89_workqueue_ready(void)
{
	lwp_t *lwp;
	int error;

	if (rtw89_ready)
		return;
	rtw89_ready = true;
	netbsd_spin_mutex_init(&rtw89_pending_mtx);

	error = kthread_create(PRI_NONE, 0, NULL, rtw89_workqueue_worker,
	    NULL, &lwp, "rtw89wq");
	if (error != 0) {
		printf("rtw89: cannot start the work thread (%d)\n", error);
		return;
	}
	rtw89_worker_started = true;
}

struct workqueue *
rtw89_workqueue_alloc(const char *name)
{

	rtw89_workqueue_ready();
	/* the queue is a singleton; the handle is only a token */
	return rtw89_worker_started ? (struct workqueue *)&rtw89_pending : NULL;
}

/*
 * The chip code calls destroy_workqueue() from failure paths that may run on
 * the rtw89 worker itself (rtw_core_init() bailing out, for example), so the
 * thread is kept for the lifetime of the kernel.
 */
void
rtw89_workqueue_free(struct workqueue *wq)
{
	(void)wq;
}

/*
 * Wait for a work item that is queued or currently running.  There is no way
 * to cancel an item that has not run yet, so this is a flush, which is what
 * the callers (rtw_core_stop and friends) actually need before tearing state
 * down.
 */
void
rtw89_work_flush(struct work_struct *w)
{

	while (w->wk_queued != 0 || w->wk_running)
		kpause("rtw89fl", false, 1, NULL);
}

/*
 * Queue an item for the worker: safe from softint and callout context.
 */
void
rtw89_work_enqueue_safe(struct work_struct *w)
{
	struct rtw89_work_item *item;

	if (!rtw89_worker_started)
		return;
	if (atomic_cas_uint(&w->wk_queued, 0, 1) != 0)
		return;

	item = kmem_alloc(sizeof(*item), KM_NOSLEEP);
	if (item == NULL) {
		w->wk_queued = 0;
		return;
	}
	item->wi_work = w;
	mutex_enter(&rtw89_pending_mtx);
	SIMPLEQ_INSERT_TAIL(&rtw89_pending, item, wi_entry);
	mutex_exit(&rtw89_pending_mtx);
	wakeup(&rtw89_pending);
}

void
rtw89_delayed_work_callout(void *arg)
{
	struct delayed_work *dw = arg;

	dw->dw_scheduled = false;
	if (dw->dw_cancel)
		return;
	rtw89_work_enqueue(&dw->work);
}

void
rtw89_timer_callout(void *arg)
{
	struct timer_list *tl = arg;

	tl->tl_pending = false;
	tl->tl_func(tl);
}

/* ------------------------------------------------------------------ */
/* deferred calls                                                      */
/* ------------------------------------------------------------------ */

/*
 * net80211 calls into the driver at splnet, but the chip code below sleeps,
 * so those paths are deferred here and run on the rtw89 workqueue.
 */
/*
 * The work item cannot be freed from inside its own callback (the workqueue
 * still walks it afterwards), so deferred calls are taken from a small pool
 * whose entries are recycled.
 */
#define	RTW89_ASYNC_CALLS	16

struct rtw89_async_call {
	struct work_struct	work;
	void			(*fn)(void *);
	void			*arg;
	volatile unsigned int	in_use;
	bool			initialised;
};

static struct rtw89_async_call rtw89_async_calls[RTW89_ASYNC_CALLS];
static unsigned int rtw89_async_call_next;

static void
rtw89_async_call_cb(struct work_struct *work)
{
	struct rtw89_async_call *call =
	    container_of(work, struct rtw89_async_call, work);

	call->fn(call->arg);
	call->in_use = 0;
}

int
rtw89_call_async(void (*fn)(void *), void *arg)
{
	struct rtw89_async_call *call;
	unsigned int i, slot;

	for (i = 0; i < RTW89_ASYNC_CALLS; i++) {
		slot = (rtw89_async_call_next + i) % RTW89_ASYNC_CALLS;
		call = &rtw89_async_calls[slot];
		if (atomic_cas_uint(&call->in_use, 0, 1) != 0)
			continue;
		call->fn = fn;
		call->arg = arg;
		if (!call->initialised) {
			call->initialised = true;
			INIT_WORK(&call->work, rtw89_async_call_cb);
		}
		rtw89_async_call_next = slot + 1;
		rtw89_work_enqueue(&call->work);
		return 0;
	}

	return EAGAIN;
}

/* ------------------------------------------------------------------ */
/* firmware(9) bridge                                                  */
/* ------------------------------------------------------------------ */

/*
 * The chip code passes Linux paths like "rtw89/rtw8921c_fw.bin"; firmware(9)
 * looks them up as <prefix>/<driver>/<image>, so only the base name is used.
 */
int
rtw89_request_firmware_nowait(const char *name, struct device *dev,
    struct firmware **fw_out, void (*cont)(const struct firmware *, void *),
    void *context)
{
	firmware_handle_t fh;
	struct firmware *fw;
	const char *base;
	u8 *buf;
	off_t size;
	int error;

	base = strrchr(name, '/');
	base = base != NULL ? base + 1 : name;

	error = firmware_open("if_rtw89", base, &fh);
	if (error != 0) {
		printf("%s: failed to open firmware %s (error %d)\n",
		    dev != NULL ? dev->name : "rtw89", base, error);
		if (cont != NULL)
			cont(NULL, context);
		return error;
	}

	size = firmware_get_size(fh);
	buf = firmware_malloc(size);
	if (buf == NULL) {
		firmware_close(fh);
		if (cont != NULL)
			cont(NULL, context);
		return ENOMEM;
	}
	error = firmware_read(fh, 0, buf, size);
	firmware_close(fh);
	if (error != 0) {
		firmware_free(buf, size);
		if (cont != NULL)
			cont(NULL, context);
		return error;
	}

	fw = kmem_zalloc(sizeof(*fw), KM_SLEEP);
	fw->size = size;
	fw->data = buf;
	fw->priv = buf;		/* the allocation to free */
	if (fw_out != NULL)
		*fw_out = fw;
	if (cont != NULL)
		cont(fw, context);
	return 0;
}

int
rtw89_request_firmware(const struct firmware **fw_out, const char *name,
    struct device *dev)
{
	firmware_handle_t fh;
	struct firmware *fw;
	const char *base;
	u8 *buf;
	off_t size;
	int error;

	base = strrchr(name, '/');
	base = base != NULL ? base + 1 : name;

	error = firmware_open("if_rtw89", base, &fh);
	if (error != 0)
		return -ENOENT;

	size = firmware_get_size(fh);
	buf = firmware_malloc(size);
	if (buf == NULL) {
		firmware_close(fh);
		return -ENOMEM;
	}
	error = firmware_read(fh, 0, buf, size);
	firmware_close(fh);
	if (error != 0) {
		firmware_free(buf, size);
		return -EIO;
	}

	fw = kmem_zalloc(sizeof(*fw), KM_SLEEP);
	fw->size = size;
	fw->data = buf;
	fw->priv = buf;
	if (fw_out != NULL)
		*fw_out = fw;
	(void)dev;
	return 0;
}

void
rtw89_release_firmware(const struct firmware *fw)
{

	if (fw == NULL)
		return;
	if (fw->priv != NULL)
		firmware_free(fw->priv, fw->size);
	kmem_free(__UNCONST(fw), sizeof(*fw));
}

/* ------------------------------------------------------------------ */
/* mac80211: the little of it the chip code really reaches             */
/* ------------------------------------------------------------------ */

/*
 * A single-instance mac80211 stand-in: one hw (with an embedded wiphy), one
 * STA vif and one peer sta.  if_rtw89.c owns these through the helpers below
 * and keeps them in sync with net80211 state; the chip code only reads them
 * (rates, AID, association) and iterates over them in its watchdog.
 */
struct rtw89_mac80211_ctx {
	struct ieee80211_hw	hw;
	struct wiphy		wiphy;
	struct ieee80211_vif	vif;
	struct ieee80211_sta	sta;
	size_t			priv_size;
	bool			vif_valid;
	bool			sta_valid;
};

struct ieee80211_hw *
rtw89_mac80211_alloc(void *priv)
{
	struct rtw89_mac80211_ctx *ctx;
	struct ieee80211_hw *hw;

	ctx = kmem_zalloc(sizeof(*ctx), KM_SLEEP);
	hw = &ctx->hw;
	hw->priv = priv;
	hw->wiphy = &ctx->wiphy;
	netbsd_mutex_init(&ctx->wiphy.wiphy_mtx);
	ctx->wiphy.rts_threshold = 2347;
	ctx->wiphy.max_scan_ssids = 1;
	ctx->wiphy.max_scan_ie_len = 512;
	hw->conf.chandef.chan = NULL;
	return hw;
}

struct ieee80211_hw *
wiphy_to_ieee80211_hw(struct wiphy *wiphy)
{
	struct rtw89_mac80211_ctx *ctx =
	    container_of(wiphy, struct rtw89_mac80211_ctx, wiphy);

	return &ctx->hw;
}

void
rtw89_mac80211_free(struct ieee80211_hw *hw)
{
	struct rtw89_mac80211_ctx *ctx;

	if (hw == NULL)
		return;
	ctx = container_of(hw, struct rtw89_mac80211_ctx, hw);
	kmem_free(ctx, sizeof(*ctx));
}

static struct rtw89_mac80211_ctx *
rtw89_mac80211_ctx(struct ieee80211_hw *hw)
{
	return container_of(hw, struct rtw89_mac80211_ctx, hw);
}

struct ieee80211_vif *
rtw89_mac80211_vif(struct ieee80211_hw *hw)
{
	struct rtw89_mac80211_ctx *ctx = rtw89_mac80211_ctx(hw);

	ctx->vif_valid = true;
	ctx->vif.type = NL80211_IFTYPE_STATION;
	/*
	 * Non-MLD single-link convention: link_conf[0] is the embedded
	 * bss_conf.  Without it the chip code's link_conf lookups miss
	 * (rtw89_mac_port_cfg_bcn_psr_rpt et al).
	 */
	ctx->vif.link_conf[0] = &ctx->vif.bss_conf;
	return &ctx->vif;
}

struct ieee80211_sta *
rtw89_mac80211_sta(struct ieee80211_hw *hw)
{
	struct rtw89_mac80211_ctx *ctx = rtw89_mac80211_ctx(hw);

	ctx->sta_valid = true;
	return &ctx->sta;
}

void
rtw89_mac80211_set_sta(struct ieee80211_hw *hw, const u8 *addr, bool valid)
{
	struct rtw89_mac80211_ctx *ctx = rtw89_mac80211_ctx(hw);

	ctx->sta_valid = valid;
	ether_addr_copy(ctx->sta.addr, addr);
}

void
rtw89_mac80211_set_mac(struct ieee80211_hw *hw, const u8 *addr)
{
	struct rtw89_mac80211_ctx *ctx = rtw89_mac80211_ctx(hw);

	ether_addr_copy(ctx->vif.addr, addr);
	ether_addr_copy(ctx->vif.bss_conf.addr, addr);
}

void
rtw89_mac80211_set_assoc(struct ieee80211_hw *hw, const u8 *bssid, bool assoc)
{
	struct rtw89_mac80211_ctx *ctx = rtw89_mac80211_ctx(hw);

	/*
	 * STA semantics: vif->addr stays our own MAC (set with
	 * rtw89_mac80211_set_mac()); only the BSSID slots move.
	 */
	ctx->vif_valid = true;
	ctx->vif.cfg.assoc = assoc;
	ctx->vif.bss_conf.assoc = assoc;
	if (bssid) {
		ether_addr_copy(ctx->vif.cfg.ap_addr, bssid);
		ether_addr_copy(ctx->vif.bss_conf.bssid, bssid);
	}
}

int
ieee80211_register_hw(struct ieee80211_hw *hw)
{

	/*
	 * The net80211 registration is done by if_rtw89.c.  What the glue
	 * must replicate here is cfg80211's behaviour at wiphy
	 * registration: the current core regulatory domain ("00", world)
	 * is applied synchronously, which invokes the driver's
	 * reg_notifier.  Without it rtwdev->regulatory.regd stays NULL
	 * and the driver's later regulatory paths fault.
	 */
	if (hw != NULL && hw->wiphy != NULL && hw->wiphy->reg_notifier != NULL) {
		struct regulatory_request req;

		memset(&req, 0, sizeof(req));
		req.alpha2[0] = '0';
		req.alpha2[1] = '0';
		req.initiator = NL80211_REGDOM_SET_BY_CORE;
		hw->wiphy->reg_notifier(hw->wiphy, &req);
	}
	return 0;
}

void
ieee80211_unregister_hw(struct ieee80211_hw *hw)
{
}

void
ieee80211_restart_hw(struct ieee80211_hw *hw)
{
}

void
ieee80211_queue_work(struct ieee80211_hw *hw, struct work_struct *work)
{

	rtw89_work_enqueue(work);
}

void
ieee80211_queue_delayed_work(struct ieee80211_hw *hw,
    struct delayed_work *dwork, unsigned long delay)
{

	schedule_delayed_work(dwork, delay);
}

void
ieee80211_wake_queues(struct ieee80211_hw *hw)
{
}

void
ieee80211_stop_queues(struct ieee80211_hw *hw)
{
}

void
ieee80211_tx_status_irqsafe(struct ieee80211_hw *hw, struct sk_buff *skb)
{

	rtw89_skb_free(skb);
}

void
ieee80211_free_txskb(struct ieee80211_hw *hw, struct sk_buff *skb)
{

	rtw89_skb_free(skb);
}

void
ieee80211_tx_info_clear_status(struct ieee80211_tx_info *info)
{

	memset(&info->status, 0, sizeof(info->status));
}

struct sk_buff *
ieee80211_tx_dequeue(struct ieee80211_hw *hw, struct ieee80211_txq *txq)
{

	return NULL;
}

int
ieee80211_txq_get_depth(struct ieee80211_txq *txq, unsigned long *frame_cnt,
    unsigned long *byte_cnt)
{

	if (frame_cnt != NULL)
		*frame_cnt = 0;
	if (byte_cnt != NULL)
		*byte_cnt = 0;
	return 0;
}

struct ieee80211_sta *
ieee80211_find_sta(struct ieee80211_vif *vif, const u8 *addr)
{
	struct rtw89_mac80211_ctx *ctx =
	    container_of(vif, struct rtw89_mac80211_ctx, vif);

	if (!ctx->sta_valid)
		return NULL;
	return &ctx->sta;
}

struct ieee80211_sta *
ieee80211_find_sta_by_ifaddr(struct ieee80211_hw *hw, const u8 *addr,
    const u8 *localaddr)
{
	struct rtw89_mac80211_ctx *ctx = rtw89_mac80211_ctx(hw);

	if (!ctx->sta_valid)
		return NULL;
	return &ctx->sta;
}

void
ieee80211_iterate_active_interfaces_atomic(struct ieee80211_hw *hw,
    u32 iter_flags, void (*iterator)(void *data, u8 *mac,
    struct ieee80211_vif *vif), void *data)
{
	struct rtw89_mac80211_ctx *ctx = rtw89_mac80211_ctx(hw);

	if (!ctx->vif_valid)
		return;
	iterator(data, ctx->vif.addr, &ctx->vif);
}

void
ieee80211_iterate_active_interfaces(struct ieee80211_hw *hw, u32 iter_flags,
    void (*iterator)(void *data, u8 *mac, struct ieee80211_vif *vif),
    void *data)
{

	ieee80211_iterate_active_interfaces_atomic(hw, iter_flags, iterator,
	    data);
}

void
ieee80211_iterate_stations_atomic(struct ieee80211_hw *hw,
    void (*iterator)(void *data, struct ieee80211_sta *sta), void *data)
{
	struct rtw89_mac80211_ctx *ctx = rtw89_mac80211_ctx(hw);

	if (!ctx->sta_valid)
		return;
	/*
	 * The shadow mac80211 never issues a station-add callback, so the chip
	 * side's per-station state (rtw_sta_info in drv_priv[]) is unbound
	 * until here.  Without it the iterator dereferences si->sta == NULL.
	 */
	rtw89_sta_init(&ctx->sta, &ctx->vif, hw->priv);
	iterator(data, &ctx->sta);
}

void
ieee80211_iterate_stations(struct ieee80211_hw *hw,
    void (*iterator)(void *data, struct ieee80211_sta *sta), void *data)
{

	ieee80211_iterate_stations_atomic(hw, iterator, data);
}

void
ieee80211_iter_keys(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
    void (*iter)(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
    struct ieee80211_sta *sta, struct ieee80211_key_conf *key, void *data),
    void *data)
{
}

void
ieee80211_iter_keys_rcu(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
    void (*iter)(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
    struct ieee80211_sta *sta, struct ieee80211_key_conf *key, void *data),
    void *data)
{
}

void
ieee80211_request_smps(struct ieee80211_vif *vif, int link_id,
    enum ieee80211_smps_mode smps_mode)
{
}

int
ieee80211_start_tx_ba_session(struct ieee80211_sta *sta, u16 tid,
    u16 timeout)
{

	return -EOPNOTSUPP;
}

/*
 * Randomised MAC address with the given mask (mac80211's
 * get_random_mask_addr): only the bits the caller wants to randomise move.
 */
void
get_random_mask_addr(u8 *buf, const u8 *addr, const u8 *mask)
{
	u8 rnd[ETH_ALEN];
	unsigned int r;
	int i;

	r = (unsigned int)random();
	memcpy(rnd, &r, sizeof(r));
	r = (unsigned int)random();
	memcpy(rnd + sizeof(r), &r, ETH_ALEN - sizeof(r));

	for (i = 0; i < ETH_ALEN; i++)
		buf[i] = (rnd[i] & ~mask[i]) | (addr[i] & mask[i]);
	buf[0] &= ~0x01;	/* never a group address */
}

/*
 * cfg80211 regulatory hint: the chip code reads its own efuse regulatory
 * data and only uses wiphy->reg_notifier for notifications, so the hint
 * itself has nothing to do here.
 */
int
regulatory_hint(struct wiphy *wiphy, const char *alpha2)
{

	return 0;
}

void
ieee80211_scan_completed(struct ieee80211_hw *hw,
    struct cfg80211_scan_info *info)
{
}

void
ieee80211_connection_loss(struct ieee80211_vif *vif)
{
}

void
ieee80211_cqm_rssi_notify(struct ieee80211_vif *vif,
    enum nl80211_cqm_rssi_threshold_event rssi_event, s32 rssi_level,
    gfp_t gfp)
{
}

struct sk_buff *
ieee80211_nullfunc_get(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
    int link_id, bool qos)
{

	return NULL;
}

struct sk_buff *
ieee80211_probereq_get(struct ieee80211_hw *hw, const u8 *src_addr,
    const u8 *ssid, size_t ssid_len, size_t tailroom)
{

	return NULL;
}

struct sk_buff *
ieee80211_proberesp_get(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{

	return NULL;
}

struct sk_buff *
ieee80211_pspoll_get(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{

	return NULL;
}

struct sk_buff *
ieee80211_beacon_get_tim(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
    u16 *tim_offset, u16 *tim_length, int link_id)
{

	return NULL;
}

/* RX delivery hook, installed by the driver glue (see rtw89_compat.h) */
void (*rtw89_rx_deliver)(struct ieee80211_hw *, struct sk_buff *);

/* ------------------------------------------------------------------ */
/* hex dump (print_hex_dump_bytes)                                     */
/* ------------------------------------------------------------------ */

void
print_hex_dump_bytes(const char *prefix_str, int prefix_type,
    const void *buf, size_t len)
{
	const u8 *p = buf;
	size_t i, j;

	for (i = 0; i < len; i += 16) {
		printf("%s%s%04zx: ", prefix_str != NULL ? prefix_str : "",
		    prefix_str != NULL ? ": " : "",
		    prefix_type == DUMP_PREFIX_OFFSET ? i : 0);
		for (j = i; j < i + 16 && j < len; j++)
			printf("%02x ", p[j]);
		printf("\n");
	}
}

/* ------------------------------------------------------------------ */
/* wiphy_work / wiphy_delayed_work                                     */
/* ------------------------------------------------------------------ */

static void
rtw89_wiphy_work_trampoline(struct work_struct *w)
{
	struct wiphy_work *ww = container_of(w, struct wiphy_work, work);

	if (ww->cancelled || ww->func == NULL)
		return;
	ww->func(ww->wiphy, ww);
}

void
wiphy_work_init(struct wiphy_work *ww, wiphy_work_func_t func)
{

	INIT_WORK(&ww->work, rtw89_wiphy_work_trampoline);
	ww->func = func;
	ww->wiphy = NULL;
	ww->cancelled = false;
}

void
wiphy_work_queue(struct wiphy *wiphy, struct wiphy_work *ww)
{

	ww->wiphy = wiphy;
	ww->cancelled = false;
	rtw89_work_enqueue(&ww->work);
}

bool
wiphy_work_cancel(struct wiphy *wiphy, struct wiphy_work *ww)
{
	bool was = !ww->cancelled;

	(void)wiphy;
	/*
	 * Mark-only: a queued item still on the pending list runs and
	 * returns immediately.  Bringing-up paths never free a work that
	 * might still be queued (if_rtw89 keeps the interface for the
	 * lifetime of the device), so no grace wait is taken here.
	 */
	ww->cancelled = true;
	return was;
}

static void
rtw89_wiphy_delayed_work_callout(void *arg)
{
	struct wiphy_delayed_work *dw = arg;

	dw->scheduled = false;
	if (dw->cancelled)
		return;
	/* the inner wiphy_work carries func; the generic trampoline
	 * dispatches it (the dist's callbacks container_of() back). */
	wiphy_work_queue(dw->work.wiphy, &dw->work);
}

void
wiphy_delayed_work_init(struct wiphy_delayed_work *dw, wiphy_work_func_t func)
{

	INIT_WORK(&dw->work.work, rtw89_wiphy_work_trampoline);
	dw->work.func = func;
	dw->work.wiphy = NULL;
	dw->work.cancelled = false;
	dw->cancelled = false;
	dw->scheduled = false;
	callout_init(&dw->dl_callout, 0);
	(void)dw->work.wiphy;
}

void
wiphy_delayed_work_queue(struct wiphy *wiphy, struct wiphy_delayed_work *dw,
    unsigned long ticks)
{

	dw->work.wiphy = wiphy;
	dw->cancelled = false;
	dw->scheduled = true;
	callout_reset(&dw->dl_callout, MAX(1, (int)ticks),
	    rtw89_wiphy_delayed_work_callout, dw);
}

bool
wiphy_delayed_work_cancel(struct wiphy *wiphy, struct wiphy_delayed_work *dw)
{
	bool was;

	(void)wiphy;
	was = dw->scheduled;
	dw->cancelled = true;
	callout_stop(&dw->dl_callout);
	dw->scheduled = false;
	rtw89_work_flush(&dw->work.work);
	return was;
}

void
wiphy_delayed_work_flush(struct wiphy *wiphy, struct wiphy_delayed_work *dw)
{

	(void)wiphy;
	callout_stop(&dw->dl_callout);
	rtw89_work_flush(&dw->work.work);
}

int
wiphy_locked_debugfs_read(struct wiphy *wiphy, struct file *file, char *buf,
    size_t bufsz, const void *ops,
    void (*fn)(struct wiphy *, struct wiphy_work *, void *), char *priv)
{

	return 0;
}

int
wiphy_locked_debugfs_write(struct wiphy *wiphy, struct file *file, char *buf,
    size_t bufsz, const void *ops,
    void (*fn)(struct wiphy *, struct wiphy_work *, void *), char *priv)
{

	return 0;
}

/* ------------------------------------------------------------------ */
/* ieee80211_alloc_hw / ieee80211_free_hw                              */
/* ------------------------------------------------------------------ */

/*
 * The rtw89 core allocates its own rtw89_dev as hw->priv (plus the bus
 * data trailing it), so unlike the rtw88 lane the driver does not hand a
 * pre-allocated priv in: alloc it here and remember its size.
 */
struct ieee80211_hw *
ieee80211_alloc_hw(unsigned int priv_data_len,
    const struct ieee80211_ops *ops)
{
	struct rtw89_mac80211_ctx *ctx;
	struct ieee80211_hw *hw;

	ctx = kmem_zalloc(sizeof(*ctx), KM_SLEEP);
	hw = &ctx->hw;
	hw->priv = kmem_zalloc(priv_data_len, KM_SLEEP);
	ctx->priv_size = priv_data_len;
	hw->ops = ops;
	hw->wiphy = &ctx->wiphy;
	ctx->wiphy.rts_threshold = 2347;
	ctx->wiphy.max_scan_ssids = 1;
	ctx->wiphy.max_scan_ie_len = 512;
	hw->conf.chandef.chan = NULL;
	netbsd_mutex_init(&ctx->wiphy.wiphy_mtx);
	return hw;
}

void
ieee80211_free_hw(struct ieee80211_hw *hw)
{
	struct rtw89_mac80211_ctx *ctx;

	if (hw == NULL)
		return;
	ctx = container_of(hw, struct rtw89_mac80211_ctx, hw);
	if (hw->priv != NULL)
		kmem_free(hw->priv, ctx->priv_size);
	netbsd_mutex_destroy(&ctx->wiphy.wiphy_mtx);
	kmem_free(ctx, sizeof(*ctx));
}

/* ------------------------------------------------------------------ */
/* cfg80211 helpers                                                    */
/* ------------------------------------------------------------------ */

const struct element *
cfg80211_find_elem(u8 eid, const u8 *ies, size_t len)
{
	const struct element *elem;

	while (len >= 2) {
		elem = (const struct element *)ies;
		if ((size_t)elem->datalen + 2 > len)
			return NULL;
		if (elem->id == eid)
			return elem;
		len -= elem->datalen + 2;
		ies += elem->datalen + 2;
	}
	return NULL;
}

const u8 *
cfg80211_find_ie(u8 eid, const u8 *ies, size_t len)
{
	const struct element *elem = cfg80211_find_elem(eid, ies, len);

	if (elem == NULL)
		return NULL;
	return (const u8 *)elem;
}

void
cfg80211_chandef_create(struct cfg80211_chan_def *chandef,
    struct ieee80211_channel *chan, enum nl80211_chan_width width)
{

	if (chandef == NULL)
		return;
	memset(chandef, 0, sizeof(*chandef));
	if (chan == NULL)
		return;
	chandef->chan = chan;
	chandef->width = width;
	chandef->center_freq1 = chan->center_freq;
	chandef->center_freq2 = 0;
}

bool
cfg80211_channel_is_psc(struct ieee80211_channel *chan)
{

	return false;
}

void
cfg80211_bss_iter(struct wiphy *wiphy, struct cfg80211_chan_def *chandef,
    void (*iter)(struct wiphy *, struct cfg80211_bss *, void *), void *data)
{

	/* no shadow BSS table: OBSS narrow-BW RU probing is not needed */
}

/* ------------------------------------------------------------------ */
/* mac80211 entry points that only have to link                        */
/* ------------------------------------------------------------------ */

void
ieee80211_txq_schedule_start(struct ieee80211_hw *hw, u8 ac)
{
}

void
ieee80211_txq_schedule_end(struct ieee80211_hw *hw, u8 ac)
{
}

struct sk_buff *
ieee80211_tx_dequeue_ni(struct ieee80211_hw *hw, struct ieee80211_txq *txq)
{

	return NULL;
}

void
ieee80211_schedule_txq(struct ieee80211_hw *hw, struct ieee80211_txq *txq)
{
}

struct ieee80211_txq *
ieee80211_next_txq(struct ieee80211_hw *hw, u8 ac)
{

	return NULL;
}

/* ------------------------------------------------------------------ */
/* dummy net_device                                                    */
/* ------------------------------------------------------------------ */

struct net_device {
	char			name[16];
};

struct net_device *
alloc_netdev_dummy(int sizeof_priv)
{

	return kmem_zalloc(sizeof(struct net_device) + sizeof_priv, KM_SLEEP);
}

void
free_netdev(struct net_device *nd)
{

	kmem_free(nd, sizeof(struct net_device));
}

void
ieee80211_return_txq(struct ieee80211_hw *hw, struct ieee80211_txq *txq,
    bool boost)
{
}

u32
ieee80211_rx_napi(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
    struct sk_buff *skb, struct napi_struct *napi)
{

	/*
	 * The RX delivery hook is installed by the driver glue; without it
	 * (pure probe/bring-up) the frame is dropped here.
	 */
	if (rtw89_rx_deliver != NULL)
		rtw89_rx_deliver(hw, skb);
	else
		rtw89_skb_free(skb);
	return 0;
}

int
ieee80211_stop_tx_ba_session(struct ieee80211_sta *sta, u16 tid)
{

	return 0;
}

void
ieee80211_stop_tx_ba_cb_irqsafe(struct ieee80211_vif *vif, const u8 *ra,
    u16 tid)
{
}

int
ieee80211_sta_ps_transition(struct ieee80211_sta *sta, bool start)
{

	return 0;
}

void
ieee80211_sta_pspoll(struct ieee80211_sta *sta)
{
}

void
ieee80211_sta_uapsd_trigger(struct ieee80211_sta *sta, u8 tid)
{
}

void
ieee80211_ready_on_channel(struct ieee80211_hw *hw)
{
}

void
ieee80211_remain_on_channel_expired(struct ieee80211_hw *hw)
{
}

bool
ieee80211_beacon_cntdwn_is_complete(struct ieee80211_vif *vif, u8 link_id)
{

	return false;
}

void
ieee80211_csa_finish(struct ieee80211_vif *vif, u8 link_id)
{
}

void
ieee80211_beacon_loss(struct ieee80211_vif *vif)
{
}

void
ieee80211_stop(struct ieee80211_hw *hw)
{
}

/* ------------------------------------------------------------------ */
/* mac80211 chanctx emulation (core.c assigns these for chanctx-less   */
/* chips; returning EOPNOTSUPP is what Linux mac80211 does)            */
/* ------------------------------------------------------------------ */

int
ieee80211_emulate_add_chanctx(struct ieee80211_hw *hw,
    struct ieee80211_chanctx_conf *ctx)
{
	return -EOPNOTSUPP;
}

void
ieee80211_emulate_remove_chanctx(struct ieee80211_hw *hw,
    struct ieee80211_chanctx_conf *ctx)
{
}

void
ieee80211_emulate_change_chanctx(struct ieee80211_hw *hw,
    struct ieee80211_chanctx_conf *ctx, u32 changed)
{
}

int
ieee80211_emulate_switch_vif_chanctx(struct ieee80211_hw *hw,
    struct ieee80211_vif_chanctx_switch *vifs, int n_vifs,
    enum ieee80211_chanctx_switch_mode mode)
{
	return -EOPNOTSUPP;
}

int
ieee80211_set_active_links(void *vif, u16 active_links)
{

	return -EOPNOTSUPP;
}
