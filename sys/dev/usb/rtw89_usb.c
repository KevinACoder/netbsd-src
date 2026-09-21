/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by the RK3568 lab (NetBSD/RTL8851BU bring-up).
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
 * usbdi(9) transport for the RTL8851BU, implementing the rtw89
 * struct rtw89_hci_ops on NetBSD -- translated from the lab's FreeBSD
 * native transport (os/freebsd/src/sys/dev/rtw89/rtw89_usb.c), which is
 * the protocol logic of contrib usb.c over a usbdi pipeline.
 *
 * Model difference from FreeBSD: NetBSD usbd transfers complete through a
 * single callback (usbd_create_xfer binds the pipe, usbd_setup_xfer +
 * usbd_transfer run it), no USB_ST_SETUP phase.  Everything else carries
 * over, in particular the disciplines the FreeBSD lane paid for:
 *
 *  1. The DMA channel array has RTW89_DMA_CH_NUM (13) entries: CH12 is
 *     the H2C/fwcmd channel, and an off-by-one here leaves it queueless.
 *  2. Completed TX slots are returned to the free pool by tx_submit()
 *     under sc->tx_mtx -- never in the completion callback.  A slot
 *     leaves the free pool when handed to USB; completion marks it done
 *     and wakes the worker (the rtw88 fix c36086f9cc3c, and the FreeBSD
 *     fix that un-stuck the firmware download at 8 H2C frames).
 *  3. R_AX_RXAGG_0 = 0x80002005 in mac_post_init, or the firmware
 *     withholds small C2H events in greedy aggregations.
 *  4. One bulk IN transfer carries every packet the RX aggregation
 *     gathered, back to back; the demux walks the chain with
 *     roundup2(offset, rx_agg_alignment = 8).  Parsing only the first
 *     packet silently dropped the C2H replies.
 *  5. A bulk OUT frame whose (txdesc + payload) is an exact multiple of
 *     512 needs the 4-byte MOD512 padding appended (short-packet
 *     delimiting) -- tx_write_fwcmd handles it.
 *  6. tx_kick_off submits synchronously in the caller's context: the
 *     firmware download reads status registers right after each H2C.
 *  7. The 8821CU reg_sec (0x4e0 write-back) does NOT apply: neither the
 *     Linux rtw89 usb.c nor the FreeBSD transport has it.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/device.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/sysctl.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>

#include "rtw89_compat.h"
#include <linux/usb.h>

#include "core.h"
#include "mac.h"
#include "reg.h"
#include "txrx.h"
#include "rtw89_dist_usb.h"
#include "rtw89_chipvar.h"
#include "rtw89_usbvar.h"

static int	rtw89_usb_tx_write(struct rtw89_dev *,
			    struct rtw89_core_tx_request *);
static void	rtw89_usb_tx_submit(struct rtw89_usb_softc *);
static void	rtw89_usb_tx_complete_skb(struct rtw89_dev *, uint8_t,
			    struct sk_buff *, usbd_status);
static void	rtw89_usb_tx_work_cb(struct work_struct *);
static void	rtw89_usb_rx_work_cb(struct work_struct *);
static void	rtw89_usb_txeof(struct usbd_xfer *, void *, usbd_status);
static void	rtw89_usb_rxeof(struct usbd_xfer *, void *, usbd_status);
static int	rtw89_usb_xfers_init(struct rtw89_usb_softc *);
static void	rtw89_usb_xfers_fini(struct rtw89_usb_softc *);
static void	rtw89_usb_sysctl_init(struct rtw89_usb_softc *);
/* the single instance, for the hw.<xname>.stats handler */
static struct rtw89_usb_softc *rtw89_usb_dbg_sc;

/*
 * Per-chip USB data: the register addresses and the DMA channel to bulk
 * OUT endpoint number map.  Mirrors rtw8851b_usb_info from the contrib
 * 8851BU front-end (endpoint 8 does not exist on this device).
 */
static const struct rtw89_usb_info rtw89_usb_info_8851b = {
	.usb_host_request_2		= R_AX_USB_HOST_REQUEST_2,
	.usb_wlan0_1			= R_AX_USB_WLAN0_1,
	.hci_func_en			= R_AX_HCI_FUNC_EN,
	.usb3_mac_npi_config_intf_0	= R_AX_USB3_MAC_NPI_CONFIG_INTF_0,
	.usb_endpoint_0			= R_AX_USB_ENDPOINT_0,
	.usb_endpoint_2			= R_AX_USB_ENDPOINT_2,
	.rx_agg_alignment		= 8,
	.bulkout_id = {
		[RTW89_DMA_ACH0] = 3,
		[RTW89_DMA_ACH1] = 4,
		[RTW89_DMA_ACH2] = 5,
		[RTW89_DMA_ACH3] = 6,
		[RTW89_DMA_B0MG] = 0,
		[RTW89_DMA_B0HI] = 1,
		[RTW89_DMA_H2C] = 2,
	},
};

/* ------------------------------------------------------------------ */
/* register access (vendor control transfers)                          */
/* ------------------------------------------------------------------ */

/*
 * The scratch buffer rides on the softc: a request that big from the
 * stack crosses a DMA boundary on some hosts, and calls must be
 * serialised so one request does not overwrite another's data.
 */
static void
rtw89_usb_vendorreq(struct rtw89_dev *rtwdev, u32 addr, void *data, u16 len,
    bool write)
{
	struct rtw89_usb_softc *sc = (struct rtw89_usb_softc *)rtwdev->priv;
	usb_device_request_t req;
	__le32 *buf;
	unsigned int idx, attempt;
	usbd_status err = USBD_NORMAL_COMPLETION;

	if (sc->detaching)
		return;

	mutex_enter(&sc->io_mtx);
	idx = sc->usb_data_idx;
	sc->usb_data_idx = (idx + 1) & (RTW89_USB_VENQT_MAX_BUF - 1);
	buf = &sc->usb_data[idx];

	req.bmRequestType = write ? UT_WRITE_VENDOR_DEVICE :
	    UT_READ_VENDOR_DEVICE;
	req.bRequest = RTW89_USB_VENQT;
	USETW(req.wValue, (u16)(addr & 0xffff));
	USETW(req.wIndex, (u16)((addr >> 16) & 0xff));
	USETW(req.wLength, len);

	for (attempt = 0; attempt < RTW89_USB_VENQT_MAX_ATTEMPTS; attempt++) {
		*buf = 0;
		if (write)
			memcpy(buf, data, len);

		err = usbd_do_request_flags(sc->udev, &req, buf, 0, NULL,
		    USBD_DEFAULT_TIMEOUT);
		if (err == USBD_NORMAL_COMPLETION) {
			sc->io_errors = 0;
			if (!write)
				memcpy(data, buf, len);
			mutex_exit(&sc->io_mtx);
			return;
		}

		if (++sc->io_errors >= RTW89_USB_IO_ERR_LIMIT &&
		    err == USBD_IOERROR) {
			/*
			 * Once dead, every later register access returns
			 * silently and the caller reports its own timeout
			 * far up the stack.  Say so here.
			 */
			sc->detaching = true;
			rtw89_warn(rtwdev, "usb %s%u 0x%x: %u consecutive "
			    "failures (%d), marking the device dead\n",
			    write ? "write" : "read", len * 8, addr,
			    sc->io_errors, err);
			break;
		}
	}

	mutex_exit(&sc->io_mtx);
	if (!sc->detaching)
		rtw89_warn(rtwdev, "usb %s%u 0x%x fail: %d\n",
		    write ? "write" : "read", len * 8, addr, err);
}

/*
 * The CMAC register window needs an aligned 4-byte read, and the chip can
 * answer with RTW89_R32_DEAD until its CMAC clock is running again: kick
 * the clock and retry (contrib usb.c rtw89_usb_read_cmac()).
 */
static u32
rtw89_usb_read_cmac(struct rtw89_dev *rtwdev, u32 addr)
{
	u32 addr32, val32, shift;
	int count;

	addr32 = addr & ~0x3u;
	shift = (addr & 0x3) * 8;

	for (count = 0; ; count++) {
		val32 = 0;
		rtw89_usb_vendorreq(rtwdev, addr32, &val32, 4, false);
		if (val32 != RTW89_R32_DEAD)
			break;

		if (count >= MAC_REG_POOL_COUNT) {
			rtw89_warn(rtwdev, "%s: addr %#x = %#x\n", __func__,
			    addr32, val32);
			break;
		}

		rtw89_write32(rtwdev, R_AX_CK_EN, B_AX_CMAC_ALLCKEN);
	}

	return val32 >> shift;
}

static u8
rtw89_usb_read8(struct rtw89_dev *rtwdev, u32 addr)
{
	u8 data = 0;

	if (ACCESS_CMAC(addr))
		return (u8)rtw89_usb_read_cmac(rtwdev, addr);

	rtw89_usb_vendorreq(rtwdev, addr, &data, 1, false);
	return data;
}

static u16
rtw89_usb_read16(struct rtw89_dev *rtwdev, u32 addr)
{
	__le16 data = 0;

	if (ACCESS_CMAC(addr))
		return (u16)rtw89_usb_read_cmac(rtwdev, addr);

	rtw89_usb_vendorreq(rtwdev, addr, &data, 2, false);
	return le16_to_cpu(data);
}

static u32
rtw89_usb_read32(struct rtw89_dev *rtwdev, u32 addr)
{
	__le32 data = 0;

	if (ACCESS_CMAC(addr))
		return rtw89_usb_read_cmac(rtwdev, addr);

	rtw89_usb_vendorreq(rtwdev, addr, &data, 4, false);
	return le32_to_cpu(data);
}

static void
rtw89_usb_write8(struct rtw89_dev *rtwdev, u32 addr, u8 val)
{
	rtw89_usb_vendorreq(rtwdev, addr, &val, 1, true);
}

static void
rtw89_usb_write16(struct rtw89_dev *rtwdev, u32 addr, u16 val)
{
	__le16 data = cpu_to_le16(val);

	rtw89_usb_vendorreq(rtwdev, addr, &data, 2, true);
}

static void
rtw89_usb_write32(struct rtw89_dev *rtwdev, u32 addr, u32 val)
{
	__le32 data = cpu_to_le32(val);

	rtw89_usb_vendorreq(rtwdev, addr, &data, 4, true);
}

/* ------------------------------------------------------------------ */
/* TX path                                                             */
/* ------------------------------------------------------------------ */

static int
rtw89_usb_tx_write_fwcmd(struct rtw89_dev *rtwdev,
    struct rtw89_core_tx_request *tx_req)
{
	struct rtw89_usb_softc *sc = (struct rtw89_usb_softc *)rtwdev->priv;
	struct rtw89_tx_desc_info *desc_info = &tx_req->desc_info;
	struct sk_buff *skb = tx_req->skb;
	struct sk_buff *skb512;
	u32 txdesc_size = rtwdev->chip->h2c_desc_size;
	void *txdesc;

	/*
	 * A bulk OUT frame whose length is an exact multiple of 512 must
	 * be avoided; when the descriptor plus payload would be, append
	 * padding (short-packet delimiting).
	 */
	if (((desc_info->pkt_size + txdesc_size) % 512) == 0) {
		skb512 = dev_alloc_skb(txdesc_size + desc_info->pkt_size +
		    RTW89_USB_MOD512_PADDING);
		if (skb512 == NULL)
			return -ENOMEM;

		skb_put_data(skb512, skb->data, skb->len);
		skb_put_zero(skb512, RTW89_USB_MOD512_PADDING);

		dev_kfree_skb_any(skb);
		skb = skb512;
		tx_req->skb = skb512;

		desc_info->pkt_size += RTW89_USB_MOD512_PADDING;
	}

	txdesc = skb_push(skb, txdesc_size);
	memset(txdesc, 0, txdesc_size);
	rtw89_chip_fill_txdesc_fwcmd(rtwdev, desc_info, txdesc);

	skb_queue_tail(&sc->ch[desc_info->ch_dma].queue, skb);
	return 0;
}

static int
rtw89_usb_tx_write(struct rtw89_dev *rtwdev,
    struct rtw89_core_tx_request *tx_req)
{
	struct rtw89_usb_softc *sc = (struct rtw89_usb_softc *)rtwdev->priv;
	struct rtw89_tx_desc_info *desc_info = &tx_req->desc_info;
	struct rtw89_tx_skb_data *skb_data;
	struct sk_buff *skb = tx_req->skb;
	struct rtw89_txwd_body *txdesc;
	u32 txdesc_size;

	if (desc_info->ch_dma >= RTW89_USB_CH_MAX ||
	    sc->ch[desc_info->ch_dma].nslots == 0) {
		rtw89_err(rtwdev, "no tx queue for dma channel %u\n",
		    desc_info->ch_dma);
		return -EINVAL;
	}

	if ((desc_info->ch_dma == RTW89_TXCH_CH12 ||
	     tx_req->tx_type == RTW89_CORE_TX_TYPE_FWCMD) &&
	    (desc_info->ch_dma != RTW89_TXCH_CH12 ||
	     tx_req->tx_type != RTW89_CORE_TX_TYPE_FWCMD)) {
		rtw89_err(rtwdev, "dma channel %d/TX type %d mismatch\n",
		    desc_info->ch_dma, tx_req->tx_type);
		return -EINVAL;
	}

	if (desc_info->ch_dma == RTW89_TXCH_CH12)
		return rtw89_usb_tx_write_fwcmd(rtwdev, tx_req);

	txdesc_size = rtwdev->chip->txwd_body_size;
	if (desc_info->en_wd_info)
		txdesc_size += rtwdev->chip->txwd_info_size;

	txdesc = skb_push(skb, txdesc_size);
	memset(txdesc, 0, txdesc_size);
	rtw89_chip_fill_txdesc(rtwdev, desc_info, txdesc);

	le32p_replace_bits(&txdesc->dword0, 1, RTW89_TXWD_BODY0_STF_MODE);

	skb_data = RTW89_TX_SKB_CB(skb);
	if (tx_req->desc_info.sn)
		skb_data->tx_rpt_sn = tx_req->desc_info.sn;
	if (tx_req->desc_info.tx_cnt_lmt)
		skb_data->tx_pkt_cnt_lmt = tx_req->desc_info.tx_cnt_lmt;

	skb_queue_tail(&sc->ch[desc_info->ch_dma].queue, skb);
	return 0;
}

/*
 * Complete one transmitted frame the way contrib usb.c does: the stack
 * waits for tx status (or a firmware tx report) before releasing the
 * frame, and H2C frames on CH12 have no status path at all.
 */
static void
rtw89_usb_tx_complete_skb(struct rtw89_dev *rtwdev, uint8_t ch_dma,
    struct sk_buff *skb, usbd_status status)
{
	struct ieee80211_tx_info *info;
	struct rtw89_txwd_body *txdesc;
	u32 txdesc_size;

	if (ch_dma == RTW89_TXCH_CH12) {
		dev_kfree_skb_any(skb);
		return;
	}

	txdesc = (struct rtw89_txwd_body *)skb->data;
	txdesc_size = rtwdev->chip->txwd_body_size;
	if (le32_get_bits(txdesc->dword0, RTW89_TXWD_BODY0_WD_INFO_EN))
		txdesc_size += rtwdev->chip->txwd_info_size;
	skb_pull(skb, txdesc_size);

	if (rtw89_is_tx_rpt_skb(rtwdev, skb)) {
		if (status == USBD_NORMAL_COMPLETION)
			rtw89_tx_rpt_skb_add(rtwdev, skb);
		else
			rtw89_tx_rpt_tx_status(rtwdev, skb,
			    RTW89_TX_MACID_DROP);
		return;
	}

	info = IEEE80211_SKB_CB(skb);
	ieee80211_tx_info_clear_status(info);

	if (status == USBD_NORMAL_COMPLETION) {
		if (info->flags & IEEE80211_TX_CTL_NO_ACK)
			info->flags |= IEEE80211_TX_STAT_NOACK_TRANSMITTED;
		else
			info->flags |= IEEE80211_TX_STAT_ACK;
	}

	ieee80211_tx_status_irqsafe(rtwdev->hw, skb);
}

/*
 * Retire completed slots, then feed every channel's pending frames into
 * its slots.  Runs in the kick-off context and on the compat worker (the
 * completion callback only marks the slot): the free lists are owned by
 * sc->tx_mtx and the USB callback never takes it.
 */
static void
rtw89_usb_tx_submit(struct rtw89_usb_softc *sc)
{
	struct rtw89_usb_ch *ch;
	struct rtw89_usb_tx_slot *slot;
	struct sk_buff *skb;
	uint8_t dma;

	mutex_enter(&sc->tx_mtx);

	for (dma = 0; dma < RTW89_USB_CH_MAX; dma++) {
		ch = &sc->ch[dma];
		if (ch->nslots == 0)
			continue;

		/*
		 * Completed slots are on ch->inflight, not on ch->free.
		 * Retiring from free never finds a completion and the
		 * pool drains after nslots frames (FreeBSD lane
		 * evidence).
		 */
		slot = TAILQ_FIRST(&ch->inflight);
		while (slot != NULL) {
			struct rtw89_usb_tx_slot *next =
			    TAILQ_NEXT(slot, next);

			if (slot->done) {
				slot->done = false;
				skb = slot->skb;
				slot->skb = NULL;
				if (ch->busy > 0)
					ch->busy--;
				TAILQ_REMOVE(&ch->inflight, slot, next);
				TAILQ_INSERT_TAIL(&ch->free, slot, next);
				if (skb != NULL && sc->rtwdev != NULL) {
					if (slot->done_status ==
					    USBD_CANCELLED) {
						/*
						 * device is going away:
						 * no tx status
						 */
						dev_kfree_skb_any(skb);
					} else {
						rtw89_usb_tx_complete_skb(
						    sc->rtwdev, dma, skb,
						    slot->done_status);
					}
				}
			}
			slot = next;
		}

		while ((skb = skb_dequeue(&ch->queue)) != NULL) {
			slot = TAILQ_FIRST(&ch->free);
			if (slot == NULL) {
				/* all buffers in flight: retry on completion */
				skb_queue_head(&ch->queue, skb);
				break;
			}
			TAILQ_REMOVE(&ch->free, slot, next);

			if (skb->len > RTW89_USB_TX_BUFSZ) {
				aprint_error_dev(sc->dev,
				    "%s: ch%u frame %u too large\n", __func__,
				    dma, skb->len);
				rtw89_usb_tx_complete_skb(sc->rtwdev, dma, skb,
				    USBD_INVAL);
				TAILQ_INSERT_TAIL(&ch->free, slot, next);
				continue;
			}

			TAILQ_INSERT_TAIL(&ch->inflight, slot, next);
			memcpy(slot->buf, skb->data, skb->len);
			slot->skb = skb;
			ch->busy++;
			sc->tx_frames++;
			if (dma == RTW89_TXCH_CH12 && sc->tx_frames == 1) {
				/* GT (FreeBSD/Linux): dword0 = 0x000c0000 */
				const uint8_t *p8 = slot->buf;
				printf("rtw89usb: CH12 frame0 %u bytes:"
				    " %02x %02x %02x %02x %02x %02x %02x %02x"
				    " %02x %02x %02x %02x %02x %02x %02x %02x"
				    " %02x %02x %02x %02x %02x %02x %02x %02x"
				    " %02x %02x %02x %02x %02x %02x %02x %02x\n",
				    (unsigned)skb->len,
				    p8[0], p8[1], p8[2], p8[3],
				    p8[4], p8[5], p8[6], p8[7],
				    p8[8], p8[9], p8[10], p8[11],
				    p8[12], p8[13], p8[14], p8[15],
				    p8[16], p8[17], p8[18], p8[19],
				    p8[20], p8[21], p8[22], p8[23],
				    p8[24], p8[25], p8[26], p8[27],
				    p8[28], p8[29], p8[30], p8[31]);
			}
			usbd_setup_xfer(slot->xfer, slot, slot->buf, skb->len,
			    0, USBD_NO_TIMEOUT, rtw89_usb_txeof);
			usbd_transfer(slot->xfer);
		}
	}

	mutex_exit(&sc->tx_mtx);
}

static void
rtw89_usb_tx_work_cb(struct work_struct *work)
{
	struct rtw89_usb_softc *sc =
	    container_of(work, struct rtw89_usb_softc, tx_work);

	rtw89_usb_tx_submit(sc);
}

static void
rtw89_usb_txeof(struct usbd_xfer *xfer, void *priv, usbd_status status)
{
	struct rtw89_usb_tx_slot *slot = priv;
	struct rtw89_usb_softc *sc = slot->sc;

	/*
	 * The submit path owns the free lists, so hand the slot back
	 * there.  sc->tx_mtx is never taken from this callback.
	 */
	if (status != USBD_NORMAL_COMPLETION && sc->tx_errprints++ < 20) {
		uint32_t alen = 0;

		usbd_get_xfer_status(xfer, NULL, NULL, &alen, NULL);
		printf("rtw89usb: BULK OUT err ch=%u status=%d len=%u\n",
		    slot->ch_dma, status, alen);
	}
	sc->tx_completes++;
	slot->done_status = status;
	slot->done = true;
	if (sc->rtwdev != NULL)
		rtw89_work_enqueue(&sc->tx_work);
}

static void
rtw89_usb_tx_kick_off(struct rtw89_dev *rtwdev, u8 txch)
{
	struct rtw89_usb_softc *sc = (struct rtw89_usb_softc *)rtwdev->priv;

	if (txch == RTW89_TXCH_CH12 && sc->tx_kicks++ % 32 == 0)
		printf("rtw89usb: kick#%u sent=%u done=%u\n",
		    sc->tx_kicks - 1, sc->tx_frames, sc->tx_completes);

	/*
	 * Submit in the caller's context: the firmware download queues
	 * hundreds of frames back to back and the chip code reads status
	 * registers right after an H2C frame, so the frames must be on
	 * their way before the caller continues.
	 */
	if (txch == RTW89_TXCH_CH12) {
		/*
		 * Read the completion target BEFORE the submit: a fast
		 * completion (a live firmware acks H2Cs in a few ms) can
		 * bump tx_completes between submit and the read, and the
		 * wait below would then block for the next frame's
		 * completion -- one full second per H2C.
		 */
		unsigned int target = sc->tx_completes + 1;

		rtw89_usb_tx_submit(sc);

		/*
		 * The H2C channel is single-slotted (serialised): with a
		 * deep inflight window the WCPU accepted only the first
		 * frame and the rest of the pipeline stalled.  The
		 * completion callback still only marks the slot; the
		 * worker returns it under the lock.
		 */
		if ((int)(sc->tx_completes - target) < 0) {
			int i;

			for (i = 0; i < 1000 &&
			    (int)(sc->tx_completes - target) < 0; i++)
				kpause("rtw89h2c", false, 1, NULL);
			if ((int)(sc->tx_completes - target) < 0 &&
			    sc->tx_errprints++ < 5)
				printf("rtw89usb: H2C frame completion "
				    "timeout\n");
		}
	} else {
		rtw89_usb_tx_submit(sc);
	}
}

static u32
rtw89_usb_check_and_reclaim_tx_resource(struct rtw89_dev *rtwdev, u8 txch)
{
	struct rtw89_usb_softc *sc = (struct rtw89_usb_softc *)rtwdev->priv;
	struct rtw89_usb_ch *ch;
	u32 nfree;

	/*
	 * The fwcmd/H2C channel has no per-channel resource accounting:
	 * answering "busy" here aborts the firmware download instead of
	 * merely pacing it.  The fixed slot pool queues whatever does not
	 * fit and drains on completion.
	 */
	if (txch == RTW89_TXCH_CH12)
		return 1;

	if (txch >= RTW89_USB_CH_MAX)
		return 0;

	ch = &sc->ch[txch];

	mutex_enter(&sc->tx_mtx);
	nfree = ch->nslots - ch->busy;
	mutex_exit(&sc->tx_mtx);

	return nfree;
}

/* ------------------------------------------------------------------ */
/* RX path                                                             */
/* ------------------------------------------------------------------ */

/*
 * The demultiplexer: the Linux handler from contrib usb.c, fed whole
 * bulk transfers.  Runs on the compat worker so the core's receive path
 * may sleep.
 */
static unsigned int rtw89_usb_rx_demux_dbg;
static unsigned int rtw89_usb_rx_work_dbg;
static unsigned int rtw89_usb_rx_overrun_dbg;

static void
rtw89_usb_rx_work_cb(struct work_struct *work)
{
	struct rtw89_usb_softc *sc =
	    container_of(work, struct rtw89_usb_softc, rx_work);
	struct rtw89_dev *rtwdev = sc->rtwdev;
	const struct rtw89_usb_info *info = sc->info;
	struct rtw89_rx_desc_info desc_info;
	struct sk_buff *rx_skb, *skb;
	u8 *pkt_ptr;
	u32 pkt_offset, aligned;
	int remaining, limit;

	if (rtw89_usb_rx_work_dbg < 10)
		printf("rtw89usb: rx work cb sc=%p rtwdev=%p info=%p\n",
		    (void *)sc, (void *)rtwdev, (void *)sc->info),
		    rtw89_usb_rx_work_dbg++;

	if (rtwdev == NULL)
		return;

	for (limit = 0; limit < 200; limit++) {
		mutex_enter(&sc->tx_mtx);
		rx_skb = skb_dequeue(&sc->rx_queue);
		mutex_exit(&sc->tx_mtx);
		if (rx_skb == NULL)
			break;

		/*
		 * One bulk transfer carries every packet the RX
		 * aggregation gathered, back to back, each with its own
		 * descriptor.  Walk the chain like Linux's rx worker
		 * does: parsing only the first packet silently dropped
		 * the C2H replies.
		 */
		pkt_ptr = rx_skb->data;
		remaining = rx_skb->len;

		do {
			memset(&desc_info, 0, sizeof(desc_info));
			rtw89_chip_query_rxdesc(rtwdev, &desc_info,
			    pkt_ptr, 0);

			pkt_offset = desc_info.offset + desc_info.rxd_len;
			if (remaining <
			    (int)(pkt_offset + desc_info.pkt_size)) {
				sc->rx_drop_overrun++;
				if (rtw89_usb_rx_overrun_dbg < 10) {
					printf("rtw89usb: rx overrun #%u: "
					    "off=%u rxd=%u size=%u rem=%d "
					    "d0=%08x\n",
					    rtw89_usb_rx_overrun_dbg,
					    desc_info.offset, desc_info.rxd_len,
					    desc_info.pkt_size, remaining,
					    le32dec(pkt_ptr));
					rtw89_usb_rx_overrun_dbg++;
				}
				rtw89_debug(rtwdev, RTW89_DBG_HCI,
				    "rx packet overruns the transfer "
				    "(%u + %u > %u)\n", pkt_offset,
				    desc_info.pkt_size, remaining);
				break;
			}

			if (rtw89_usb_rx_demux_dbg < 20) {
				printf("rtw89usb: rx pkt #%u type=%u size=%u "
				    "off=%u rem=%d: %02x %02x %02x %02x\n",
				    rtw89_usb_rx_demux_dbg,
				    desc_info.pkt_type,
				    desc_info.pkt_size, pkt_offset, remaining,
				    pkt_ptr[pkt_offset], pkt_ptr[pkt_offset + 1],
				    pkt_ptr[pkt_offset + 2],
				    pkt_ptr[pkt_offset + 3]);
				rtw89_usb_rx_demux_dbg++;
			}

			skb = rtw89_alloc_skb_for_rx(rtwdev,
			    desc_info.pkt_size);
			if (skb == NULL)
				break;

			skb_put_data(skb, pkt_ptr + pkt_offset,
			    desc_info.pkt_size);
			rtw89_core_rx(rtwdev, &desc_info, skb);

			pkt_offset += desc_info.pkt_size;
			aligned = roundup2(pkt_offset,
			    info->rx_agg_alignment);
			if (aligned == 0)
				break;
			pkt_ptr += aligned;
			remaining -= aligned;
		} while (remaining > 0);

		dev_kfree_skb_any(rx_skb);
	}

	if (limit == 200)
		rtw89_work_enqueue(&sc->rx_work);
}

static void
rtw89_usb_rxeof(struct usbd_xfer *xfer, void *priv, usbd_status status)
{
	struct rtw89_usb_softc *sc = priv;
	struct sk_buff *skb;
	u_int32_t actlen = 0;
	int i;

	if (status == USBD_CANCELLED)
		return;

	/* find our transfer index so the buffer matches on resubmit */
	for (i = 0; i < RTW89_USB_RX_XFERS; i++)
		if (sc->rx_xfer[i] == xfer)
			break;
	if (i == RTW89_USB_RX_XFERS)
		return;

	if (status != USBD_NORMAL_COMPLETION) {
		/* keep the pipe alive; the chip recovers on re-arming */
		if (sc->rx_errprints++ < 20)
			printf("rtw89usb: BULK IN status=%d\n", status);
		if (status == USBD_STALLED)
			/*
			 * Fire the async CLEAR_FEATURE(ENDPOINT_HALT) so a
			 * later re-submit can succeed; until then resubmits
			 * just re-STALL (softint context, so use the async
			 * variant).
			 */
			usbd_clear_endpoint_stall_async(sc->rx_pipe);
		goto resubmit;
	}

	usbd_get_xfer_status(xfer, NULL, NULL, &actlen, NULL);

	/*
	 * Linux rejects transfers shorter than a receive descriptor.  A
	 * transfer that OVERFLOWS the buffer means the device split a
	 * packet across transfers, which the demux would misread -- drop
	 * it.  A transfer that exactly fills the buffer is a legitimately
	 * full hardware aggregate (the RXAGG limit is 5 x 4K = this buffer
	 * size); rejecting >= that dropped whole aggregates under load, so
	 * only > is an error here, as in Linux.
	 */
	if (actlen < RTW89_USB_RX_MIN_LEN) {
		sc->rx_drop_short++;
		goto resubmit;
	}
	if (actlen > RTW89_USB_RX_BUFSZ) {
		sc->rx_drop_full++;
		goto resubmit;
	}

	if (sc->rx_dbg < 30) {
		printf("rtw89usb: rx xfer #%u actlen=%u "
		    "(full=%u short=%u)\n", sc->rx_dbg, actlen,
		    sc->rx_drop_full, sc->rx_drop_short);
		sc->rx_dbg++;
	}

	skb = alloc_skb(actlen, GFP_ATOMIC);
	if (skb != NULL) {
		skb_put_data(skb, sc->rx_buf[i], actlen);
		if (skb_queue_len(&sc->rx_queue) >= RTW89_USB_RXQ_MAX) {
			dev_kfree_skb_any(skb);
		} else {
			mutex_enter(&sc->tx_mtx);
			skb_queue_tail(&sc->rx_queue, skb);
			mutex_exit(&sc->tx_mtx);
			rtw89_work_enqueue(&sc->rx_work);
		}
	} else {
		aprint_error_dev(sc->dev, "rx buffer allocation failed\n");
	}

resubmit:
	usbd_setup_xfer(xfer, sc, sc->rx_buf[i], RTW89_USB_RX_BUFSZ,
	    USBD_SHORT_XFER_OK, USBD_NO_TIMEOUT, rtw89_usb_rxeof);
	usbd_transfer(xfer);
}

/* ------------------------------------------------------------------ */
/* chip-facing HCI ops                                                 */
/* ------------------------------------------------------------------ */

static void
rtw89_usb_ops_reset(struct rtw89_dev *rtwdev)
{
	struct rtw89_usb_softc *sc = (struct rtw89_usb_softc *)rtwdev->priv;
	struct rtw89_usb_ch *ch;
	uint8_t dma;

	/* Drop every queued frame; in-flight transfers complete alone. */
	mutex_enter(&sc->tx_mtx);
	for (dma = 0; dma < RTW89_USB_CH_MAX; dma++) {
		ch = &sc->ch[dma];
		if (ch->nslots != 0)
			skb_queue_purge(&ch->queue);
	}
	mutex_exit(&sc->tx_mtx);

	rtw89_tx_rpt_skbs_purge(rtwdev);
}

static int
rtw89_usb_ops_start(struct rtw89_dev *rtwdev)
{
	struct rtw89_usb_softc *sc = (struct rtw89_usb_softc *)rtwdev->priv;
	int i;

	if (!sc->xfers_inited)
		return 0;

	/*
	 * Arm the RX xfers once per firmware session.  NetBSD usbdi queues
	 * a re-submitted xfer again without any duplicate protection, so
	 * ops_stop/pipes_reset clear rx_armed together with the pipe state
	 * and every core_start arms freshly created xfers.
	 */
	if (sc->rx_armed)
		return 0;
	sc->rx_armed = true;

	/* Arm the bulk IN pipe: firmware C2H replies arrive from here on. */
	for (i = 0; i < RTW89_USB_RX_XFERS; i++) {
		usbd_setup_xfer(sc->rx_xfer[i], sc, sc->rx_buf[i],
		    RTW89_USB_RX_BUFSZ, USBD_SHORT_XFER_OK, USBD_NO_TIMEOUT,
		    rtw89_usb_rxeof);
		usbd_transfer(sc->rx_xfer[i]);
	}

	return 0;
}

static void
rtw89_usb_ops_stop(struct rtw89_dev *rtwdev)
{
	struct rtw89_usb_softc *sc = (struct rtw89_usb_softc *)rtwdev->priv;

	/*
	 * core_stop() calls hci_stop: quiesce the bulk IN pipe so no C2H
	 * from the dying firmware session reaches the core.  Cancelled
	 * completions return from rxeof without re-arming.  The bulk OUT
	 * pipes are torn down by the next chip_start()'s pipes_reset()
	 * (the firmware restart reinitialises its endpoints, so the
	 * host-side data toggles must not carry across).
	 */
	if (sc->rx_armed) {
		usbd_abort_pipe(sc->rx_pipe);
		sc->rx_armed = false;
	}
}

static void
rtw89_usb_ops_pause(struct rtw89_dev *rtwdev, bool pause)
{
}

static void
rtw89_usb_ops_switch_mode(struct rtw89_dev *rtwdev, bool low_power)
{
}

static int
rtw89_usb_ops_deinit(struct rtw89_dev *rtwdev)
{
	return 0;
}

/*
 * Realtek's own USB IO/TRX hang workaround: enable USBIO mode, drop the
 * RX/TX reset bits, then toggle the HCI DMA enables so the endpoint
 * state machines restart clean.
 */
static int
rtw89_usb_ops_mac_pre_init(struct rtw89_dev *rtwdev)
{
	struct rtw89_usb_softc *sc = (struct rtw89_usb_softc *)rtwdev->priv;
	const struct rtw89_usb_info *info = sc->info;
	u32 val32;

	rtw89_write32_set(rtwdev, info->usb_host_request_2,
	    B_AX_R_USBIO_MODE);
	rtw89_write32_clr(rtwdev, info->usb_wlan0_1,
	    B_AX_USBRX_RST | B_AX_USBTX_RST);

	val32 = rtw89_read32(rtwdev, info->hci_func_en);
	val32 &= ~(B_AX_HCI_RXDMA_EN | B_AX_HCI_TXDMA_EN);
	rtw89_write32(rtwdev, info->hci_func_en, val32);

	val32 |= B_AX_HCI_RXDMA_EN | B_AX_HCI_TXDMA_EN;
	rtw89_write32(rtwdev, info->hci_func_en, val32);

	return 0;
}

static int
rtw89_usb_ops_mac_pre_deinit(struct rtw89_dev *rtwdev)
{
	return 0;
}

/*
 * Program the RXDMA bulk size for the negotiated USB speed and tell the
 * MAC which endpoint number each DMA channel lives on, then set the RX
 * aggregation shape (R_AX_RXAGG_0 = 0x80002005): without it the C2H
 * stream dies here.
 */
static int
rtw89_usb_ops_mac_post_init(struct rtw89_dev *rtwdev)
{
	struct rtw89_usb_softc *sc = (struct rtw89_usb_softc *)rtwdev->priv;
	const struct rtw89_usb_info *info = sc->info;
	u32 ep;

	rtw89_write32_clr(rtwdev, info->usb3_mac_npi_config_intf_0,
	    B_AX_SSPHY_LFPS_FILTER);

	switch (sc->udev->ud_speed) {
	case USB_SPEED_SUPER:
		rtw89_write8(rtwdev, R_AX_RXDMA_SETTING,
		    RTW89_USB3_BULKSIZE);
		break;
	case USB_SPEED_HIGH:
		rtw89_write8(rtwdev, R_AX_RXDMA_SETTING,
		    RTW89_USB2_BULKSIZE);
		break;
	default:
		rtw89_write8(rtwdev, R_AX_RXDMA_SETTING,
		    RTW89_USB11_BULKSIZE);
		break;
	}

	for (ep = 5; ep <= 12; ep++) {
		if (ep == 8)
			continue;

		rtw89_write8_mask(rtwdev, info->usb_endpoint_0, B_AX_EP_IDX,
		    ep);
		rtw89_write8(rtwdev, info->usb_endpoint_2 + 1,
		    RTW89_USB_NUMP);
	}

	rtw89_write32(rtwdev, R_AX_RXAGG_0, RTW89_USB_RXAGG_0_8851B);

	return 0;
}

static void
rtw89_usb_ops_recalc_int_mit(struct rtw89_dev *rtwdev)
{
	/* interrupt moderation is a PCIe concept */
}

static int
rtw89_usb_ops_mac_lv1_rcvy(struct rtw89_dev *rtwdev,
    enum rtw89_lv1_rcvy_step step)
{
	/* 8851B is a BE chip: the V1 register pair is 8852C only */
	rtw89_write32_set(rtwdev, R_AX_USB_WLAN0_1,
	    B_AX_USBRX_RST | B_AX_USBTX_RST);
	msleep(30);
	rtw89_write32_clr(rtwdev, R_AX_USB_WLAN0_1,
	    B_AX_USBRX_RST | B_AX_USBTX_RST);
	(void)step;
	return 0;
}

static const struct rtw89_hci_ops rtw89_usb_ops = {
	.tx_write	= rtw89_usb_tx_write,
	.tx_kick_off	= rtw89_usb_tx_kick_off,
	.flush_queues	= NULL,		/* every channel has its own queue */
	.reset		= rtw89_usb_ops_reset,
	.start		= rtw89_usb_ops_start,
	.stop		= rtw89_usb_ops_stop,
	.pause		= rtw89_usb_ops_pause,
	.switch_mode	= rtw89_usb_ops_switch_mode,
	.recalc_int_mit	= rtw89_usb_ops_recalc_int_mit,

	.read8		= rtw89_usb_read8,
	.read16		= rtw89_usb_read16,
	.read32		= rtw89_usb_read32,
	.write8		= rtw89_usb_write8,
	.write16	= rtw89_usb_write16,
	.write32	= rtw89_usb_write32,

	.mac_pre_init	= rtw89_usb_ops_mac_pre_init,
	.mac_pre_deinit	= rtw89_usb_ops_mac_pre_deinit,
	.mac_post_init	= rtw89_usb_ops_mac_post_init,
	.deinit		= rtw89_usb_ops_deinit,

	.check_and_reclaim_tx_resource =
	    rtw89_usb_check_and_reclaim_tx_resource,
	.mac_lv1_rcvy	= rtw89_usb_ops_mac_lv1_rcvy,
	.dump_err_status = NULL,
	.napi_poll	= NULL,

	.recovery_start	= NULL,
	.recovery_complete = NULL,

	.ctrl_txdma_ch	= NULL,
	.ctrl_txdma_fw_ch = NULL,
	.ctrl_trxhci	= NULL,
	.poll_txdma_ch_idle = NULL,

	.clr_idx_all	= NULL,
	.clear		= NULL,
	.disable_intr	= NULL,
	.enable_intr	= NULL,
	.rst_bdram	= NULL,
};

/*
 * The imported rtw8851bu.c's inert driver struct references these names
 * at link time; on NetBSD binding happens through usbdevs in if_rtw89.c.
 */
int	rtw89_usb_probe(struct usb_interface *, const struct usb_device_id *);
void	rtw89_usb_disconnect(struct usb_interface *);

int
rtw89_usb_probe(struct usb_interface *intf, const struct usb_device_id *id)
{

	(void)intf;
	(void)id;
	return -ENODEV;
}

void
rtw89_usb_disconnect(struct usb_interface *intf)
{

	(void)intf;
}

const struct rtw89_hci_ops *
rtw89_usb_get_ops(void)
{

	return &rtw89_usb_ops;
}

/* ------------------------------------------------------------------ */
/* endpoint discovery + xfer setup                                     */
/* ------------------------------------------------------------------ */

/* Slot pool size per DMA channel (0 = channel not carried here). */
static uint8_t
rtw89_usb_ch_slots(uint8_t dma)
{
	size_t i;

	for (i = 0; i < __arraycount(rtw89_usb_ch_map); i++)
		if (rtw89_usb_ch_map[i].ch_dma == dma)
			return rtw89_usb_ch_map[i].nslots;

	return 0;
}

static int
rtw89_usb_parse_endpoints(struct rtw89_usb_softc *sc)
{
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	uint8_t num, nout = 0;
	int j;

	id = usbd_get_interface_descriptor(sc->iface);
	if (id == NULL) {
		aprint_error_dev(sc->dev, "%s: no interface descriptor\n",
		    __func__);
		return ENXIO;
	}
	if (id->bNumEndpoints > RTW89_MAX_ENDPOINT_NUM) {
		aprint_error_dev(sc->dev,
		    "%s: found %d endpoints, expected %d max\n", __func__,
		    id->bNumEndpoints, RTW89_MAX_ENDPOINT_NUM);
		return ENXIO;
	}

	for (j = 0; j < id->bNumEndpoints; j++) {
		ed = usbd_interface2endpoint_descriptor(sc->iface, j);
		if (ed == NULL)
			continue;

		num = UE_GET_ADDR(ed->bEndpointAddress);

		if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN &&
		    UE_GET_XFERTYPE(ed->bmAttributes) == UE_BULK) {
			if (sc->pipe_in != 0) {
				aprint_error_dev(sc->dev,
				    "%s: more than one bulk IN endpoint\n",
				    __func__);
				return ENXIO;
			}
			sc->pipe_in = num;
		} else if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_OUT &&
		    UE_GET_XFERTYPE(ed->bmAttributes) == UE_BULK) {
			if (nout >= RTW89_MAX_BULKOUT_NUM) {
				aprint_error_dev(sc->dev,
				    "%s: more than %d bulk OUT endpoints\n",
				    __func__, RTW89_MAX_BULKOUT_NUM);
				return ENXIO;
			}
			sc->bulkout_ep[nout++] = num;
		}
	}

	if (sc->pipe_in == 0 || nout < 1) {
		aprint_error_dev(sc->dev, "%s: no bulk %s endpoint found\n",
		    __func__, sc->pipe_in == 0 ? "IN" : "OUT");
		return ENXIO;
	}

	sc->n_out_ep = nout;
	return 0;
}

/*
 * Map the DMA channels onto the discovered bulk OUT endpoints (endpoint
 * numbers, not interface endpoint indices), and open one pipe per
 * channel.  The bulkout_id table is the contrib rtw8851b_usb_info map.
 */
static int
rtw89_usb_open_bulkout_pipes(struct rtw89_usb_softc *sc)
{
	int j, n;

	n = 0;
	for (j = 0; j < (int)__arraycount(rtw89_usb_ch_map); j++) {
		struct rtw89_usb_ch *ch =
		    &sc->ch[rtw89_usb_ch_map[j].ch_dma];
		uint8_t bulkout =
		    sc->info->bulkout_id[rtw89_usb_ch_map[j].ch_dma];
		uint8_t epnum;
		int error;

		if (bulkout >= sc->n_out_ep)
			continue;
		epnum = sc->bulkout_ep[bulkout];

		error = usbd_open_pipe(sc->iface,
		    epnum | UE_DIR_OUT, USBD_EXCLUSIVE_USE, &ch->pipe);
		if (error != 0) {
			aprint_error_dev(sc->dev,
			    "%s: cannot open bulk OUT ep %u (%d)\n",
			    __func__, epnum, error);
			return error;
		}
		ch->nslots = rtw89_usb_ch_map[j].nslots;
		n++;
	}
	if (n == 0) {
		aprint_error_dev(sc->dev, "%s: no usable bulkout mapping\n",
		    __func__);
		return ENXIO;
	}

	return 0;
}

/*
 * Tear down every pipe and transfer.  Also the first half of
 * pipes_reset(): aborting completes the in-flight callbacks (CANCELLED
 * paths in txeof/rxeof), then the xfers are safe to destroy.
 */
static void
rtw89_usb_xfers_fini(struct rtw89_usb_softc *sc)
{
	struct rtw89_usb_ch *ch;
	struct rtw89_usb_tx_slot *slot;
	uint8_t dma;
	int i;

	sc->rx_armed = false;

	if (sc->rx_pipe != NULL)
		usbd_abort_pipe(sc->rx_pipe);
	for (dma = 0; dma < RTW89_USB_CH_MAX; dma++) {
		ch = &sc->ch[dma];
		if (ch->pipe != NULL)
			usbd_abort_pipe(ch->pipe);
	}

	mutex_enter(&sc->tx_mtx);
	for (dma = 0; dma < RTW89_USB_CH_MAX; dma++) {
		ch = &sc->ch[dma];
		if (ch->nslots != 0)
			skb_queue_purge(&ch->queue);
	}
	skb_queue_purge(&sc->rx_queue);

	for (dma = 0; dma < RTW89_USB_CH_MAX; dma++) {
		ch = &sc->ch[dma];
		while ((slot = TAILQ_FIRST(&ch->inflight)) != NULL) {
			TAILQ_REMOVE(&ch->inflight, slot, next);
			if (slot->skb != NULL) {
				/* frames of the dead firmware session */
				dev_kfree_skb_any(slot->skb);
				slot->skb = NULL;
			}
			usbd_destroy_xfer(slot->xfer);
			kmem_free(slot, sizeof(*slot));
		}
		while ((slot = TAILQ_FIRST(&ch->free)) != NULL) {
			TAILQ_REMOVE(&ch->free, slot, next);
			usbd_destroy_xfer(slot->xfer);
			kmem_free(slot, sizeof(*slot));
		}
		if (ch->pipe != NULL) {
			usbd_close_pipe(ch->pipe);
			ch->pipe = NULL;
		}
		ch->nslots = 0;
		ch->busy = 0;
	}
	sc->xfers_inited = false;
	mutex_exit(&sc->tx_mtx);

	for (i = 0; i < RTW89_USB_RX_XFERS; i++) {
		if (sc->rx_xfer[i] != NULL) {
			usbd_destroy_xfer(sc->rx_xfer[i]);
			sc->rx_xfer[i] = NULL;
		}
	}
	if (sc->rx_pipe != NULL) {
		usbd_close_pipe(sc->rx_pipe);
		sc->rx_pipe = NULL;
	}
}

static int
rtw89_usb_xfers_init(struct rtw89_usb_softc *sc)
{
	struct rtw89_usb_ch *ch;
	struct rtw89_usb_tx_slot *slot;
	uint8_t dma;
	int i, error;

	/* bulk IN pipe + RX transfer ring */
	error = usbd_open_pipe(sc->iface, sc->pipe_in | UE_DIR_IN,
	    USBD_EXCLUSIVE_USE, &sc->rx_pipe);
	if (error != 0) {
		aprint_error_dev(sc->dev, "%s: cannot open bulk IN pipe (%d)\n",
		    __func__, error);
		return error;
	}

	for (i = 0; i < RTW89_USB_RX_XFERS; i++) {
		error = usbd_create_xfer(sc->rx_pipe, RTW89_USB_RX_BUFSZ,
		    USBD_SHORT_XFER_OK, 0, &sc->rx_xfer[i]);
		if (error != 0)
			goto out_fini;
		sc->rx_buf[i] = usbd_get_buffer(sc->rx_xfer[i]);
	}

	/* one TX slot pool per DMA channel, xfers bound to the channel pipe */
	error = rtw89_usb_open_bulkout_pipes(sc);
	if (error != 0)
		goto out_fini;
	for (dma = 0; dma < RTW89_USB_CH_MAX; dma++) {
		ch = &sc->ch[dma];
		for (i = 0; i < ch->nslots; i++) {
			slot = kmem_zalloc(sizeof(*slot), KM_SLEEP);
			error = usbd_create_xfer(ch->pipe, RTW89_USB_TX_BUFSZ,
			    0, 0, &slot->xfer);
			if (error != 0) {
				kmem_free(slot, sizeof(*slot));
				goto out_fini;
			}
			slot->buf = usbd_get_buffer(slot->xfer);
			slot->sc = sc;
			slot->ch_dma = dma;
			TAILQ_INSERT_TAIL(&ch->free, slot, next);
		}
	}

	sc->xfers_inited = true;
	return 0;

out_fini:
	rtw89_usb_xfers_fini(sc);
	return error;
}

/*
 * Bring every bulk pipe back to a pristine host-side state.  mac_pwr_off
 * restarts the device firmware, which reinitialises its bulk endpoints
 * at DATA0; a host-side pipe that carried traffic across that boundary
 * stays mid-toggle-stream and every transfer crawls -- the second
 * firmware download ran 0.2-2 s/frame this way, while the attach
 * download on freshly opened pipes ran ~1 ms/frame.  xhci(4) allocates a
 * fresh transfer ring and endpoint context in usbd_open_pipe
 * (xhci_open -> xhci_ring_init + xhci_configure_endpoint), so
 * close+open re-synchronises both sides of every endpoint.
 */
int
rtw89_usb_pipes_reset(struct rtw89_usb_softc *sc)
{

	if (!sc->xfers_inited || sc->detaching)
		return ENXIO;

	rtw89_usb_xfers_fini(sc);
	return rtw89_usb_xfers_init(sc);
}

int
rtw89_usb_attach(struct rtw89_usb_softc *sc, device_t dev,
    struct usbd_device *udev, struct usbd_interface *iface)
{
	struct rtw89_usb_ch *ch;
	uint8_t dma;
	int error;

	memset(sc, 0, sizeof(*sc));
	sc->dev = dev;
	sc->udev = udev;
	sc->iface = iface;
	sc->info = &rtw89_usb_info_8851b;

	netbsd_spin_mutex_init(&sc->tx_mtx);
	netbsd_mutex_init(&sc->io_mtx);

	skb_queue_head_init(&sc->rx_queue);
	INIT_WORK(&sc->rx_work, rtw89_usb_rx_work_cb);
	INIT_WORK(&sc->tx_work, rtw89_usb_tx_work_cb);

	for (dma = 0; dma < RTW89_USB_CH_MAX; dma++) {
		ch = &sc->ch[dma];
		TAILQ_INIT(&ch->free);
		TAILQ_INIT(&ch->inflight);
		skb_queue_head_init(&ch->queue);
	}

	error = rtw89_usb_parse_endpoints(sc);
	if (error != 0)
		return error;

	rtw89_usb_sysctl_init(sc);

	return rtw89_usb_xfers_init(sc);
}

/* ------------------------------------------------------------------ */
/* Diagnostics: hw.<xname>.stats read-only summary                     */
/* ------------------------------------------------------------------ */

static int
rtw89_usb_stats_sysctl(SYSCTLFN_ARGS)
{
	struct sysctlnode node = *rnode;
	struct rtw89_usb_softc *sc = rtw89_usb_dbg_sc;
	char ch[160];
	char buf[512];
	unsigned int i;
	int error;

	if (sc == NULL)
		return ENXIO;

	ch[0] = '\0';
	mutex_enter(&sc->tx_mtx);
	for (i = 0; i < RTW89_USB_CH_MAX; i++) {
		if (sc->ch[i].nslots == 0)
			continue;
		snprintf(ch + strlen(ch), sizeof(ch) - strlen(ch),
		    "%sch%u=%u/%u", (i != 0) ? " " : "", i,
		    sc->ch[i].busy, sc->ch[i].nslots);
	}
	mutex_exit(&sc->tx_mtx);

	snprintf(buf, sizeof(buf),
	    "tx: frames=%u completes=%u kicks=%u errprints=%u\n"
	    "rx: drop_full=%u drop_short=%u drop_overrun=%u errprints=%u\n"
	    "io: errors=%u slots: %s\n",
	    sc->tx_frames, sc->tx_completes, sc->tx_kicks,
	    sc->tx_errprints,
	    sc->rx_drop_full, sc->rx_drop_short, sc->rx_drop_overrun,
	    sc->rx_errprints,
	    sc->io_errors, ch);

	node.sysctl_data = buf;
	node.sysctl_size = strlen(buf) + 1;
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error != 0 || newp == NULL)
		return error;

	return 0;
}

static void
rtw89_usb_sysctl_init(struct rtw89_usb_softc *sc)
{
	const struct sysctlnode *rnode, *cnode;
	static int done;
	int error;

	if (done)
		return;
	done = 1;
	rtw89_usb_dbg_sc = sc;

	error = sysctl_createv(NULL, 0, NULL, &rnode,
	    0, CTLTYPE_NODE, device_xname(sc->dev),
	    SYSCTL_DESCR("rtw89u transport diagnostics"),
	    NULL, 0, NULL, 0, CTL_HW, CTL_CREATE, CTL_EOL);
	if (error)
		goto fail;
	error = sysctl_createv(NULL, 0, &rnode, &cnode,
	    CTLFLAG_READONLY, CTLTYPE_STRING,
	    "stats", SYSCTL_DESCR("transport counters and slot occupancy"),
	    rtw89_usb_stats_sysctl, 0, NULL, 512, CTL_CREATE, CTL_EOL);
	if (error)
		goto fail;
	return;
fail:
	aprint_error_dev(sc->dev, "sysctl_createv failed (%d)\n", error);
}

void
rtw89_usb_detach(struct rtw89_usb_softc *sc)
{
	sc->detaching = true;

	rtw89_usb_xfers_fini(sc);

	rtw89_work_flush(&sc->tx_work);
	rtw89_work_flush(&sc->rx_work);

	netbsd_mutex_destroy(&sc->io_mtx);
	netbsd_mutex_destroy(&sc->tx_mtx);
}
