/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by the RK3568 lab (NetBSD/AIC8800D80 bring-up).
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

#ifndef _DEV_USB_AIC8800VAR_H_
#define _DEV_USB_AIC8800VAR_H_

#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/callout.h>
#include <sys/queue.h>
#include <sys/mbuf.h>

#include <net/if.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <net80211/ieee80211_var.h>

#include <dev/usb/usbdi.h>

#include "aic8800_msg.h"

/*
 * The AIC8800D80 reaches this driver in one of two personalities after
 * umodeswitch(4) has flipped the fake CD-ROM (1111:1111):
 *
 *  AIC8800U_BROM  a69c:8d80  boot ROM.  Only one bulk pair exists; the
 *                            lmac_msg command channel runs on it and the
 *                            firmware download happens here.
 *  AIC8800U_APP   a69c:8d81  app.  The full-mac firmware runs the 802.11
 *                            state machine; the WiFi interface (vendor
 *                            class ff/ff/ff) carries the data bulk pair
 *                            plus dedicated message bulk endpoints, and
 *                            two further interfaces are Bluetooth.
 *
 * Ground truth: doc/rk3568-itx-wiki/runs/20260913-linux-aic8800-usb/
 * (AIC8800D80-GROUND-TRUTH.md).
 */
enum aic8800u_personality {
	AIC8800U_BROM,
	AIC8800U_APP,
};

/*
 * Bulk endpoint layout, filled by aic8800u_parse_endpoints() from the
 * interface descriptors in the order the vendor driver uses: the first
 * bulk IN/OUT of the WiFi interface are the data pipes, the second pair
 * (app personality only) carries lmac_msg command frames.
 */
struct aic8800u_ep {
	uint8_t		addr;
	uint16_t	maxpkt;
	bool		present;
};

struct aic8800u_endpoints {
	struct aic8800u_ep	data_in;
	struct aic8800u_ep	msg_in;
	struct aic8800u_ep	data_out;
	struct aic8800u_ep	msg_out;
};

/*
 * Firmware patch table entry list (parsed fw_patch_table_*.bin).  The
 * INF-table payload (struct aic8800u_patch_info) lives in aic8800_msg.h.
 */
struct aic8800u_patch_table {
	struct aic8800u_patch_table *next;
	uint32_t		type;
	uint32_t		len;		/* {addr, val} pair count */
	uint32_t		*data;		/* kmem_alloc'ed, len * 2 words */
};

/* worker flags (sc_flags, guarded by sc_work_mtx) */
#define AIC8800U_F_NEWSTATE	0x01
#define AIC8800U_F_TX		0x02
#define AIC8800U_F_EVENT	0x04
#define AIC8800U_F_EXIT		0x08
#define AIC8800U_F_SCANTIMO	0x10	/* scan watchdog fired */
#define AIC8800U_F_KEYSYNC	0x20	/* mirror net80211 keys to firmware */

/* pseudo event id for TX confirmations delivered through the event queue */
#define AIC8800U_EVT_TXCFM	0xffff

/* event queue entry: an unsolicited lmac_msg (or a TX CFM) for the worker */
struct aic8800u_event {
	TAILQ_ENTRY(aic8800u_event) ev_next;
	uint16_t		ev_id;
	uint8_t			*ev_data;	/* kmem'ed param copy */
	size_t			ev_len;
};

#define AIC8800U_TXCFM_SLOTS	64	/* vendor USB_TXDESC_CNT */
#define AIC8800U_EVTQ_MAX	128	/* bound scan-result bursts */

MBUFQ_HEAD(aic8800u_txq);

struct aic8800u_softc {
	device_t		 sc_dev;
	struct usbd_device	*sc_udev;
	struct usbd_interface	*sc_iface;	/* WiFi function */
	enum aic8800u_personality sc_personality;
	struct aic8800u_endpoints sc_ep;

	/*
	 * Synchronous lmac_msg command channel.  BROM: the loader issues
	 * one command at a time and reads CFM frames itself.  APP: the
	 * command is posted here and the evt thread matches the CFM.
	 */
	struct usbd_pipe	*sc_cmd_pipe;	/* command OUT */
	struct usbd_pipe	*sc_evt_pipe;	/* event/CFM IN */
	struct usbd_xfer	*sc_cmd_xfer;
	struct usbd_xfer	*sc_evt_xfer;
	uint8_t			*sc_cmd_buf;	/* AIC8800_TX_FRAME_MAX */
	uint8_t			*sc_evt_buf;	/* AIC8800_RX_BUF_MAX */
	bool			 sc_transport_ready;

	/*
	 * APP command-wait state (sc_cmd_mtx).  Exactly one command may
	 * be in flight; the evt thread fills sc_cmd_cfm_buf and signals
	 * sc_cmd_cv when the CFM with sc_cmd_cfm_id arrives.
	 */
	kmutex_t		 sc_cmd_mtx;
	kcondvar_t		 sc_cmd_cv;
	bool			 sc_cmd_active;
	uint16_t		 sc_cmd_cfm_id;
	void			*sc_cmd_cfm_buf;
	size_t			 sc_cmd_cfm_len;
	int			 sc_cmd_error;

	/*
	 * Data endpoints (APP only).  Data OUT carries data/mgmt frames,
	 * data IN delivers received MPDUs behind a 60-byte hardware
	 * header (aic8800_msg.h for the framing).
	 */
	struct usbd_pipe	*sc_data_out_pipe;
	struct usbd_pipe	*sc_data_in_pipe;
	struct usbd_xfer	*sc_tx_xfer;
	struct usbd_xfer	*sc_rx_xfer;
	uint8_t			*sc_tx_buf;	/* AIC8800_DATA_TX_BUF_MAX */
	uint8_t			*sc_rx_buf;	/* AIC8800_RX_BUF_MAX */
	bool			 sc_data_ready;

	/* transport threads (APP only; self-clear their lwp pointers) */
	lwp_t			*sc_evt_lwp;
	lwp_t			*sc_rx_lwp;
	bool			 sc_app_started;
	bool			 sc_if_attached;	/* rx gate, see below */

	/*
	 * net80211 front end (APP).  The worker follows the rtw8189f
	 * pattern: ic_newstate only snapshots and the worker drives the
	 * chip; everything sleeps on USB, the state machine runs at
	 * splnet from the worker.
	 *
	 * The ifnet MUST live inside a struct ethercom: ether_ioctl()
	 * recovers it as (struct ethercom *)ifp, and with a bare ifnet
	 * the ethercom fields (ec_multiaddrs et al) silently overlay
	 * sc_ic -- net80211 then stomps them and the first multicast
	 * join at ifconfig up writes through a garbage list head.
	 */
	struct ethercom		 sc_ec;
#define sc_if			sc_ec.ec_if
	struct ieee80211com	 sc_ic;
	int			(*sc_newstate)(struct ieee80211com *,
				    enum ieee80211_state, int);
	uint8_t			 sc_mac_addr[IEEE80211_ADDR_LEN];

	kmutex_t		 sc_work_mtx;	/* IPL_NET */
	kcondvar_t		 sc_cv;
	uint32_t		 sc_flags;
	/*
	 * Pending state transitions, FIFO (sc_work_mtx).  A single slot
	 * loses hops: connect_ind queues S_ASSOC then S_RUN back to back,
	 * and the overwritten S_ASSOC made the stack see an illegal
	 * S_AUTH->S_RUN transition that skipped the association-complete
	 * notification wpa_supplicant waits for.
	 */
#define AIC8800U_NSTATEQ_MAX	8
	enum ieee80211_state	 sc_nstateq[AIC8800U_NSTATEQ_MAX];
	int			 sc_nargq[AIC8800U_NSTATEQ_MAX];
	unsigned		 sc_nstateq_head;
	unsigned		 sc_nstateq_tail;
	lwp_t			*sc_worker;
	struct callout		 sc_scan_to;	/* firmware scan watchdog */
	struct aic8800u_txq	 sc_txq;

	/* firmware events from the evt thread to the worker */
	kmutex_t		 sc_evtq_mtx;
	TAILQ_HEAD(, aic8800u_event) sc_evtq;
	unsigned		 sc_evtq_count;

	/* firmware runtime state */
	uint8_t			 sc_vif_idx;	/* MM_ADD_IF result */
	int			 sc_ap_idx;	/* -1 = not connected */
	uint16_t		 sc_aid;
	bool			 sc_connected;
	bool			 sc_scanning;	/* firmware scan in flight */

	/*
	 * The firmware decrypts CCMP, so frames handed to net80211 carry
	 * no host-visible key and the stack's F_DROPUNENC policy (set by
	 * wpa_supplicant on every WPA association) would discard every
	 * received data frame.  The driver takes that policy over:
	 * sc_dropunenc records that it was requested, the flag itself is
	 * cleared on the ieee80211com, and rx_frame() enforces it against
	 * the firmware's per-frame decryption status instead.
	 */
	bool			 sc_dropunenc;

	/* key / control-port bookkeeping (hw.aic8800.stats) */
	uint32_t		 sc_key_ptk;
	uint32_t		 sc_key_gtk;
	uint32_t		 sc_key_fail;
	uint32_t		 sc_cp_open;
	uint32_t		 sc_cp_fail;
	bool			 sc_cp_state;

	/* need_cfm TX bookkeeping (EAPOL / management frames) */
	struct mbuf		*sc_txcfm_m[AIC8800U_TXCFM_SLOTS];
	uint16_t		 sc_txcfm_plen[AIC8800U_TXCFM_SLOTS];
	unsigned		 sc_txcfm_free;
	unsigned		 sc_txcfm_used;
	uint32_t		 sc_txcfm_acked;
	uint32_t		 sc_txcfm_retried;
	uint32_t		 sc_txcfm_lost;
	uint32_t		 sc_txcfm_submitted;
	uint32_t		 sc_txcfm_last_submit;
	uint32_t		 sc_txcfm_last_used;

	/* diagnostics */
	uint32_t		 sc_rx_frames;
	uint32_t		 sc_rx_fcserr;
	uint32_t		 sc_rx_decrerr;
	uint32_t		 sc_rx_amsdu;
	uint32_t		 sc_rx_decrypted;	/* fw-decrypted, delivered */
	uint32_t		 sc_rx_unenc_drop;	/* policy drop, see above */
	uint32_t		 sc_rx_dbg_logged;	/* rx_debug prints done */
	uint32_t		 sc_tx_dbg_logged;	/* tx_debug prints done */
	uint32_t		 sc_tx_frames;		/* data frames submitted */
	uint32_t		 sc_tx_errors;
	uint32_t		 sc_mgmt_dropped;
	uint32_t		 sc_evtq_dropped;
	uint32_t		 sc_evt_trunc;	/* frame claimed past the URB */
	uint32_t		 sc_scan_clamped;	/* TLVs clamped/dropped */

	/* loader progress */
	uint32_t		 sc_chip_id;
	uint32_t		 sc_fw_version;

	/*
	 * Bring-up thread lifecycle.  Threads self-clear their lwp
	 * pointer before kthread_exit(); whoever observes the other side
	 * gone (detached / thread exited) tears the transport down
	 * (rtw8189f stop() deadlock lesson).
	 */
	kmutex_t		 sc_load_mtx;
	lwp_t			*sc_bringup_lwp;
	bool			 sc_dying;
	bool			 sc_detached;
};

/* aic8800_usb.c */
int	aic8800u_parse_endpoints(struct aic8800u_softc *,
	    struct usbd_interface *);
int	aic8800u_transport_init(struct aic8800u_softc *);
void	aic8800u_transport_fini(struct aic8800u_softc *);
int	aic8800u_cmd(struct aic8800u_softc *, uint16_t id, uint16_t dest_id,
	    uint16_t src_id, const void *param, size_t param_len,
	    void *cfm, size_t cfm_len);
int	aic8800u_cmd_cfm(struct aic8800u_softc *, uint16_t id, uint16_t dest_id,
	    uint16_t src_id, const void *param, size_t param_len,
	    uint16_t cfm_id, void *cfm, size_t cfm_len);
int	aic8800u_cmd_send(struct aic8800u_softc *, uint16_t id, uint16_t dest_id,
	    uint16_t src_id, const void *param, size_t param_len);
int	aic8800u_data_write(struct aic8800u_softc *, size_t len);
int	aic8800u_threads_start(struct aic8800u_softc *);
size_t	aic8800u_evt_dequeue(struct aic8800u_softc *,
	    struct aic8800u_event **);
void	aic8800u_evt_release(struct aic8800u_softc *,
	    struct aic8800u_event *);
int	aic8800u_dbg_read32(struct aic8800u_softc *, uint32_t addr,
	    uint32_t *val);
int	aic8800u_dbg_write32(struct aic8800u_softc *, uint32_t addr,
	    uint32_t val);
int	aic8800u_dbg_mask_write32(struct aic8800u_softc *, uint32_t addr,
	    uint32_t mask, uint32_t val);
int	aic8800u_start_app(struct aic8800u_softc *, uint32_t boot_addr);

/* aic8800_chip.c */
const char *aic8800u_personality_name(enum aic8800u_personality);
bool	aic8800u_firmware_available(struct aic8800u_softc *);
void	aic8800u_fw_download(struct aic8800u_softc *);
int	aic8800u_fw_init(struct aic8800u_softc *);
int	aic8800u_scan_start(struct aic8800u_softc *, const uint8_t *ssid,
	    size_t ssid_len);
int	aic8800u_connect(struct aic8800u_softc *, struct ieee80211_node *);
void	aic8800u_disconnect(struct aic8800u_softc *);
int	aic8800u_key_add(struct aic8800u_softc *, const uint8_t *key,
	    size_t key_len, unsigned key_idx, bool pairwise);
void	aic8800u_control_port(struct aic8800u_softc *, bool open);
void	aic8800u_dbg_sysctl_init(struct aic8800u_softc *);
extern int	aic8800u_dbg_payload_mode;
extern int	aic8800u_dbg_min_tx;
extern int	aic8800u_dbg_driver_dropunenc;
extern int	aic8800u_dbg_rx_debug;
extern int	aic8800u_dbg_tx_debug;
extern int	aic8800u_dbg_cfm_all;

/* upper bound for the hw.aic8800.stats string */
#define AIC8800_STATS_LEN	640

/*
 * Short data frames (EAPOL-Key M4, ARP) are silently swallowed by the
 * firmware unless the descriptor advertises at least roughly this many
 * bytes; measured on the board: 113 dropped, 135 passed, 120 passed.
 */
#define AIC8800U_TX_MIN_PAYLOAD	140

#endif	/* _DEV_USB_AIC8800VAR_H_ */
