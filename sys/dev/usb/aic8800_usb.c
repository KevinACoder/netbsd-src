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

/*
 * AIC8800D80 usbdi(9) transport -- endpoint discovery, the lmac_msg
 * command channel and the app-mode data paths.
 *
 * The bulk layout follows the vendor driver's order-based rule: the
 * first bulk IN and first bulk OUT of the WiFi interface are the data
 * pipes; in the app personality a second bulk pair carries lmac_msg
 * command frames.  The boot ROM has no dedicated message endpoints and
 * sends its command channel over the first bulk pair.
 *
 * The command channel has two modes.  The boot ROM speaks to the same
 * thread that sends: write the frame, then read event frames until the
 * CFM arrives.  In the app personality unsolicited events (scan
 * results, connection indications, TX confirmations) interleave with
 * command CFMs, so a dedicated evt thread owns the message IN pipe:
 * it completes the one in-flight command (matched by exact CFM id) and
 * queues everything else for the net80211 worker.  Every wait has a
 * timeout -- the KI-036 lesson from the Linux lane.
 *
 * Frame framing is documented in aic8800_msg.h.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/kmem.h>
#include <sys/mbuf.h>
#include <sys/device.h>
#include <sys/kthread.h>
#include <sys/proc.h>		/* kpause */

#include <net/if.h>
#include <net/if_media.h>
#include <net80211/ieee80211_var.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>

#include "aic8800var.h"

/* TX timeout per frame; the RX budget is enforced across the CFM loop. */
#define AIC8800_TX_TIMEOUT_MS	1000
#define AIC8800_RX_TIMEOUT_MS	2000

static void
aic8800u_ep_record(struct aic8800u_ep *ep, uint8_t addr, uint16_t maxpkt)
{
	ep->addr = addr;
	ep->maxpkt = maxpkt;
	ep->present = true;
}

int
aic8800u_parse_endpoints(struct aic8800u_softc *sc, struct usbd_interface *iface)
{
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	int j;

	memset(&sc->sc_ep, 0, sizeof(sc->sc_ep));

	id = usbd_get_interface_descriptor(iface);
	if (id == NULL)
		return EINVAL;

	for (j = 0; j < id->bNumEndpoints; j++) {
		ed = usbd_interface2endpoint_descriptor(iface, j);
		if (ed == NULL)
			continue;
		if (UE_GET_XFERTYPE(ed->bmAttributes) != UE_BULK)
			continue;

		if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN) {
			if (!sc->sc_ep.data_in.present)
				aic8800u_ep_record(&sc->sc_ep.data_in,
				    ed->bEndpointAddress,
				    UGETW(ed->wMaxPacketSize));
			else if (!sc->sc_ep.msg_in.present)
				aic8800u_ep_record(&sc->sc_ep.msg_in,
				    ed->bEndpointAddress,
				    UGETW(ed->wMaxPacketSize));
		} else {
			if (!sc->sc_ep.data_out.present)
				aic8800u_ep_record(&sc->sc_ep.data_out,
				    ed->bEndpointAddress,
				    UGETW(ed->wMaxPacketSize));
			else if (!sc->sc_ep.msg_out.present)
				aic8800u_ep_record(&sc->sc_ep.msg_out,
				    ed->bEndpointAddress,
				    UGETW(ed->wMaxPacketSize));
		}
	}

	if (!sc->sc_ep.data_in.present || !sc->sc_ep.data_out.present)
		return ENODEV;

	aprint_normal_dev(sc->sc_dev, "bulk in %#x maxpkt %d, out %#x maxpkt %d\n",
	    sc->sc_ep.data_in.addr, sc->sc_ep.data_in.maxpkt,
	    sc->sc_ep.data_out.addr, sc->sc_ep.data_out.maxpkt);
	if (sc->sc_ep.msg_in.present && sc->sc_ep.msg_out.present)
		aprint_normal_dev(sc->sc_dev,
		    "msg in %#x maxpkt %d, out %#x maxpkt %d\n",
		    sc->sc_ep.msg_in.addr, sc->sc_ep.msg_in.maxpkt,
		    sc->sc_ep.msg_out.addr, sc->sc_ep.msg_out.maxpkt);

	return 0;
}

/*
 * Pick the command-channel endpoints and open the pipes.  The boot ROM
 * speaks on the data pair; the app personality uses the dedicated msg
 * pair for commands and opens the data pair for frames.
 */
int
aic8800u_transport_init(struct aic8800u_softc *sc)
{
	uint8_t cmd_addr, evt_addr;
	bool app_data = false;
	int error;

	KASSERT(sc->sc_transport_ready == false);

	if (sc->sc_personality == AIC8800U_BROM ||
	    !sc->sc_ep.msg_in.present || !sc->sc_ep.msg_out.present) {
		cmd_addr = sc->sc_ep.data_out.addr;
		evt_addr = sc->sc_ep.data_in.addr;
	} else {
		cmd_addr = sc->sc_ep.msg_out.addr;
		evt_addr = sc->sc_ep.msg_in.addr;
		app_data = true;
	}

	error = usbd_open_pipe(sc->sc_iface, cmd_addr, USBD_EXCLUSIVE_USE,
	    &sc->sc_cmd_pipe);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "cannot open cmd pipe %#x: %s\n",
		    cmd_addr, usbd_errstr(error));
		return EIO;
	}
	error = usbd_open_pipe(sc->sc_iface, evt_addr, USBD_EXCLUSIVE_USE,
	    &sc->sc_evt_pipe);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "cannot open evt pipe %#x: %s\n",
		    evt_addr, usbd_errstr(error));
		usbd_close_pipe(sc->sc_cmd_pipe);
		sc->sc_cmd_pipe = NULL;
		return EIO;
	}

	sc->sc_cmd_buf = kmem_alloc(AIC8800_TX_FRAME_MAX, KM_SLEEP);
	sc->sc_evt_buf = kmem_alloc(AIC8800_RX_BUF_MAX, KM_SLEEP);
	error = usbd_create_xfer(sc->sc_cmd_pipe, AIC8800_TX_FRAME_MAX, 0, 0,
	    &sc->sc_cmd_xfer);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "cannot create cmd xfer\n");
		goto fail;
	}
	error = usbd_create_xfer(sc->sc_evt_pipe, AIC8800_RX_BUF_MAX, 0, 0,
	    &sc->sc_evt_xfer);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "cannot create evt xfer\n");
		usbd_destroy_xfer(sc->sc_cmd_xfer);
		sc->sc_cmd_xfer = NULL;
		goto fail;
	}

	if (app_data) {
		error = usbd_open_pipe(sc->sc_iface, sc->sc_ep.data_out.addr,
		    USBD_EXCLUSIVE_USE, &sc->sc_data_out_pipe);
		if (error != 0) {
			aprint_error_dev(sc->sc_dev,
			    "cannot open data out pipe %#x: %s\n",
			    sc->sc_ep.data_out.addr, usbd_errstr(error));
			goto fail2;
		}
		error = usbd_open_pipe(sc->sc_iface, sc->sc_ep.data_in.addr,
		    USBD_EXCLUSIVE_USE, &sc->sc_data_in_pipe);
		if (error != 0) {
			aprint_error_dev(sc->sc_dev,
			    "cannot open data in pipe %#x: %s\n",
			    sc->sc_ep.data_in.addr, usbd_errstr(error));
			usbd_close_pipe(sc->sc_data_out_pipe);
			sc->sc_data_out_pipe = NULL;
			goto fail2;
		}
		sc->sc_tx_buf = kmem_alloc(AIC8800_DATA_TX_BUF_MAX, KM_SLEEP);
		sc->sc_rx_buf = kmem_alloc(AIC8800_RX_BUF_MAX, KM_SLEEP);
		error = usbd_create_xfer(sc->sc_data_out_pipe,
		    AIC8800_DATA_TX_BUF_MAX, 0, 0, &sc->sc_tx_xfer);
		if (error != 0) {
			aprint_error_dev(sc->sc_dev,
			    "cannot create tx xfer\n");
			goto fail2;
		}
		error = usbd_create_xfer(sc->sc_data_in_pipe,
		    AIC8800_RX_BUF_MAX, 0, 0, &sc->sc_rx_xfer);
		if (error != 0) {
			aprint_error_dev(sc->sc_dev,
			    "cannot create rx xfer\n");
			usbd_destroy_xfer(sc->sc_tx_xfer);
			sc->sc_tx_xfer = NULL;
			goto fail2;
		}
		sc->sc_data_ready = true;
	}

	sc->sc_transport_ready = true;
	return 0;

fail2:
	kmem_free(sc->sc_cmd_buf, AIC8800_TX_FRAME_MAX);
	kmem_free(sc->sc_evt_buf, AIC8800_RX_BUF_MAX);
	sc->sc_cmd_buf = sc->sc_evt_buf = NULL;
	usbd_destroy_xfer(sc->sc_evt_xfer);
	sc->sc_evt_xfer = NULL;
	usbd_destroy_xfer(sc->sc_cmd_xfer);
	sc->sc_cmd_xfer = NULL;
	usbd_close_pipe(sc->sc_evt_pipe);
	sc->sc_evt_pipe = NULL;
	usbd_close_pipe(sc->sc_cmd_pipe);
	sc->sc_cmd_pipe = NULL;
	return EIO;

fail:
	kmem_free(sc->sc_cmd_buf, AIC8800_TX_FRAME_MAX);
	kmem_free(sc->sc_evt_buf, AIC8800_RX_BUF_MAX);
	sc->sc_cmd_buf = sc->sc_evt_buf = NULL;
	usbd_close_pipe(sc->sc_evt_pipe);
	sc->sc_evt_pipe = NULL;
	usbd_close_pipe(sc->sc_cmd_pipe);
	sc->sc_cmd_pipe = NULL;
	return EIO;
}

void
aic8800u_transport_fini(struct aic8800u_softc *sc)
{
	if (!sc->sc_transport_ready)
		return;

	if (sc->sc_data_ready) {
		if (sc->sc_rx_xfer != NULL) {
			usbd_destroy_xfer(sc->sc_rx_xfer);
			sc->sc_rx_xfer = NULL;
		}
		if (sc->sc_tx_xfer != NULL) {
			usbd_destroy_xfer(sc->sc_tx_xfer);
			sc->sc_tx_xfer = NULL;
		}
		kmem_free(sc->sc_tx_buf, AIC8800_DATA_TX_BUF_MAX);
		kmem_free(sc->sc_rx_buf, AIC8800_RX_BUF_MAX);
		sc->sc_tx_buf = sc->sc_rx_buf = NULL;
		usbd_close_pipe(sc->sc_data_in_pipe);
		sc->sc_data_in_pipe = NULL;
		usbd_close_pipe(sc->sc_data_out_pipe);
		sc->sc_data_out_pipe = NULL;
		sc->sc_data_ready = false;
	}

	if (sc->sc_evt_xfer != NULL) {
		usbd_destroy_xfer(sc->sc_evt_xfer);
		sc->sc_evt_xfer = NULL;
	}
	if (sc->sc_cmd_xfer != NULL) {
		usbd_destroy_xfer(sc->sc_cmd_xfer);
		sc->sc_cmd_xfer = NULL;
	}
	kmem_free(sc->sc_cmd_buf, AIC8800_TX_FRAME_MAX);
	kmem_free(sc->sc_evt_buf, AIC8800_RX_BUF_MAX);
	sc->sc_cmd_buf = sc->sc_evt_buf = NULL;
	usbd_close_pipe(sc->sc_evt_pipe);
	sc->sc_evt_pipe = NULL;
	usbd_close_pipe(sc->sc_cmd_pipe);
	sc->sc_cmd_pipe = NULL;

	sc->sc_transport_ready = false;
}

static int
aic8800u_bulk_write(struct aic8800u_softc *sc, const void *buf, size_t len)
{
	uint32_t count;
	usbd_status status;

	usbd_setup_xfer(sc->sc_cmd_xfer, NULL, __UNCONST(buf), len,
	    USBD_SYNCHRONOUS, AIC8800_TX_TIMEOUT_MS, NULL);
	status = usbd_transfer(sc->sc_cmd_xfer);
	usbd_get_xfer_status(sc->sc_cmd_xfer, NULL, NULL, &count, &status);
	if (status != USBD_NORMAL_COMPLETION || count != len)
		return EIO;

	return 0;
}

static int
aic8800u_bulk_read_evt(struct aic8800u_softc *sc, uint32_t *count)
{
	usbd_status status;

	usbd_setup_xfer(sc->sc_evt_xfer, NULL, sc->sc_evt_buf,
	    AIC8800_RX_BUF_MAX, USBD_SYNCHRONOUS | USBD_SHORT_XFER_OK,
	    AIC8800_RX_TIMEOUT_MS, NULL);
	status = usbd_transfer(sc->sc_evt_xfer);
	usbd_get_xfer_status(sc->sc_evt_xfer, NULL, NULL, count, &status);
	if (status == USBD_TIMEOUT)
		return ETIMEDOUT;
	if (status != USBD_NORMAL_COMPLETION)
		return EIO;
	if (*count == 0)
		return EIO;

	return 0;
}

static int
aic8800u_bulk_read_data(struct aic8800u_softc *sc, uint32_t *count)
{
	usbd_status status;

	usbd_setup_xfer(sc->sc_rx_xfer, NULL, sc->sc_rx_buf,
	    AIC8800_RX_BUF_MAX, USBD_SYNCHRONOUS | USBD_SHORT_XFER_OK,
	    AIC8800_RX_TIMEOUT_MS, NULL);
	status = usbd_transfer(sc->sc_rx_xfer);
	usbd_get_xfer_status(sc->sc_rx_xfer, NULL, NULL, count, &status);
	if (status == USBD_TIMEOUT)
		return ETIMEDOUT;
	if (status != USBD_NORMAL_COMPLETION)
		return EIO;
	if (*count == 0)
		return EIO;

	return 0;
}

/*
 * Build one command frame in sc_cmd_buf and push it out the command
 * pipe.  Frame layout in aic8800_msg.h.
 */
static int
aic8800u_cmd_frame_and_write(struct aic8800u_softc *sc, uint16_t id,
    uint16_t dest_id, uint16_t src_id, const void *param, size_t param_len)
{
	uint8_t *buf = sc->sc_cmd_buf;
	size_t len = 8 + param_len;	/* lmac_msg header + param */

	KASSERT(param_len <= AIC8800_MEM_BLOCK_WRITE_REQ_LEN);

	memset(buf, 0, AIC8800_TX_FRAME_MAX);
	buf[0] = (len + 4) & 0xff;
	buf[1] = ((len + 4) >> 8) & 0x0f;
	buf[2] = AIC8800_USB_TYPE_CFG_CMD_RSP;
	buf[3] = 0x00;
	/* [4..7] dummy word stays zero */
	buf[8] = id & 0xff;
	buf[9] = (id >> 8) & 0xff;
	buf[10] = dest_id & 0xff;
	buf[11] = (dest_id >> 8) & 0xff;
	buf[12] = src_id & 0xff;
	buf[13] = (src_id >> 8) & 0xff;
	buf[14] = param_len & 0xff;
	buf[15] = (param_len >> 8) & 0xff;
	if (param_len > 0)
		memcpy(&buf[16], param, param_len);

	return aic8800u_bulk_write(sc, buf, len + 8);
}

/*
 * Walk the event frames in one bulk-IN buffer looking for the CFM with
 * the expected id.  Boot-ROM mode only; the app personality has its own
 * thread for that (unsolicited events interleave with CFMs there).
 */
static int
aic8800u_scan_evt_buf(struct aic8800u_softc *sc, uint32_t count,
    uint16_t cfm_id, void *cfm, size_t cfm_len, bool *found)
{
	uint8_t *buf = sc->sc_evt_buf;
	size_t off = 0;

	*found = false;

	while (off + 4 <= count) {
		uint16_t pkt_len, msg_id, msg_param_len;
		uint8_t type;

		pkt_len = buf[off] | (buf[off + 1] << 8);
		type = buf[off + 2] & 0x7f;

		if (pkt_len == 0)
			break;

		/* a frame past the end of the transfer is a truncation */
		if (off + 4 + pkt_len > count)
			break;

		if ((buf[off + 2] & AIC8800_USB_TYPE_CFG) == AIC8800_USB_TYPE_CFG &&
		    type == AIC8800_USB_TYPE_CFG_CMD_RSP) {
			if (pkt_len < 12)
				break;
			msg_id = buf[off + 4] | (buf[off + 5] << 8);
			msg_param_len = buf[off + 10] | (buf[off + 11] << 8);
			if (msg_param_len > pkt_len - 12)
				msg_param_len = pkt_len - 12;

			if (msg_id == cfm_id) {
				size_t copy = msg_param_len;

				if (copy > cfm_len)
					copy = cfm_len;
				if (copy > 0 && cfm != NULL)
					memcpy(cfm, &buf[off + 16], copy);
				*found = true;
				return 0;
			}
		} else if (type == AIC8800_USB_TYPE_CFG_PRINT) {
			/* firmware console output: keep the log clean */
		} else {
			aprint_error_dev(sc->sc_dev,
			    "unexpected event frame type %#x len %u\n",
			    type, pkt_len);
		}

		/* frame header (4) + payload rounded to 4 */
		off += 4 + ((pkt_len + 3) & ~3u);
	}

	return 0;
}

/*
 * Queue one unsolicited event for the worker.  ev_data is a kmem copy
 * of the lmac_msg param block.
 */
static void
aic8800u_evt_enqueue(struct aic8800u_softc *sc, uint16_t id,
    const uint8_t *data, size_t len)
{
	struct aic8800u_event *ev;

	ev = kmem_alloc(sizeof(*ev) + len, KM_SLEEP);
	ev->ev_id = id;
	ev->ev_len = len;
	ev->ev_data = (uint8_t *)(ev + 1);
	memcpy(ev->ev_data, data, len);

	mutex_enter(&sc->sc_evtq_mtx);
	if (sc->sc_evtq_count >= AIC8800U_EVTQ_MAX) {
		sc->sc_evtq_dropped++;
		mutex_exit(&sc->sc_evtq_mtx);
		kmem_free(ev, sizeof(*ev) + len);
		return;
	}
	TAILQ_INSERT_TAIL(&sc->sc_evtq, ev, ev_next);
	sc->sc_evtq_count++;
	mutex_exit(&sc->sc_evtq_mtx);

	mutex_enter(&sc->sc_work_mtx);
	sc->sc_flags |= AIC8800U_F_EVENT;
	cv_broadcast(&sc->sc_cv);
	mutex_exit(&sc->sc_work_mtx);
}

size_t
aic8800u_evt_dequeue(struct aic8800u_softc *sc, struct aic8800u_event **evp)
{
	struct aic8800u_event *ev;

	mutex_enter(&sc->sc_evtq_mtx);
	ev = TAILQ_FIRST(&sc->sc_evtq);
	if (ev != NULL) {
		TAILQ_REMOVE(&sc->sc_evtq, ev, ev_next);
		sc->sc_evtq_count--;
		mutex_exit(&sc->sc_evtq_mtx);
		*evp = ev;
		return ev->ev_len;
	}
	mutex_exit(&sc->sc_evtq_mtx);
	*evp = NULL;
	return 0;
}

void
aic8800u_evt_release(struct aic8800u_softc *sc, struct aic8800u_event *ev)
{

	kmem_free(ev, sizeof(*ev) + ev->ev_len);
}

/*
 * Event-thread buffer dispatch: complete the in-flight command when the
 * CFM id matches, queue everything else for the worker.
 */
static void
aic8800u_app_dispatch(struct aic8800u_softc *sc, uint32_t count)
{
	uint8_t *buf = sc->sc_evt_buf;
	size_t off = 0;

	while (off + 4 <= count) {
		uint16_t pkt_len, msg_id, msg_param_len;
		uint8_t type;

		pkt_len = buf[off] | (buf[off + 1] << 8);
		type = buf[off + 2] & 0x7f;

		if (pkt_len == 0)
			break;

		/*
		 * A frame claiming more than this transfer holds is a
		 * truncation: everything from here on is stale content of
		 * an earlier transfer.  Never parse or queue it -- stale
		 * TLVs with garbage lengths are how the heap gets smashed
		 * downstream (ieee80211_add_scan trusts its caller).
		 */
		if (off + 4 + pkt_len > count) {
			sc->sc_evt_trunc++;
			break;
		}

		if ((buf[off + 2] & AIC8800_USB_TYPE_CFG) == AIC8800_USB_TYPE_CFG &&
		    type == AIC8800_USB_TYPE_CFG_CMD_RSP) {
			bool taken = false;

			if (pkt_len >= 12) {
				msg_id = buf[off + 4] | (buf[off + 5] << 8);
				msg_param_len = buf[off + 10] |
				    (buf[off + 11] << 8);
				/* the param lives at frame offset 12; the
				 * claim must fit inside the received frame */
				if (msg_param_len > pkt_len - 12)
					msg_param_len = pkt_len - 12;

				mutex_enter(&sc->sc_cmd_mtx);
				if (sc->sc_cmd_active &&
				    msg_id == sc->sc_cmd_cfm_id) {
					size_t copy = uimin(msg_param_len,
					    sc->sc_cmd_cfm_len);

					if (copy > 0 && sc->sc_cmd_cfm_buf != NULL)
						memcpy(sc->sc_cmd_cfm_buf,
						    &buf[off + 16], copy);
					sc->sc_cmd_active = false;
					cv_broadcast(&sc->sc_cmd_cv);
					taken = true;
				}
				mutex_exit(&sc->sc_cmd_mtx);

				if (!taken)
					aic8800u_evt_enqueue(sc, msg_id,
					    &buf[off + 16], msg_param_len);
			}
		} else if (type == AIC8800_USB_TYPE_CFG_DATA_CFM) {
			/* TX confirmation: 8-byte {status, used_idx} */
			if (pkt_len >= 8)
				aic8800u_evt_enqueue(sc, AIC8800U_EVT_TXCFM,
				    &buf[off + 4], 8);
		}
		/* CFG_PRINT: firmware console output, dropped */

		/* frame header (4) + payload rounded to 4 */
		off += 4 + ((pkt_len + 3) & ~3u);
	}
}

static void
aic8800u_evt_thread(void *arg)
{
	struct aic8800u_softc *sc = arg;
	uint32_t count;
	int error;

	while (!sc->sc_dying) {
		error = aic8800u_bulk_read_evt(sc, &count);
		if (error == ETIMEDOUT)
			continue;
		if (error != 0)
			break;		/* pipe aborted or device gone */
		aic8800u_app_dispatch(sc, count);
	}

	mutex_enter(&sc->sc_load_mtx);
	sc->sc_evt_lwp = NULL;
	mutex_exit(&sc->sc_load_mtx);
	kthread_exit(0);
}

/*
 * Received data frame: one packet per transfer, 60-byte hardware header
 * then the 802.11 MPDU (aic8800_msg.h for the layout).  Firmware-decrypted
 * frames still carry the 8-byte CCMP header behind the 802.11 header and
 * the Protected bit, but their MIC is gone; strip both and deliver the
 * plaintext to net80211 (the old stack would otherwise hand the frame to
 * its software CCMP, which cannot parse it).
 */
static void
aic8800u_rx_frame(struct aic8800u_softc *sc, uint32_t count)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	struct mbuf *m;
	uint8_t *buf = sc->sc_rx_buf;
	uint8_t *mpdu = &buf[AIC8800_RX_MPDU_OFF];
	uint16_t mpdu_len = buf[0] | (buf[1] << 8);
	uint32_t status;
	size_t hdr_len;
	int8_t rssi1, rssileg;
	int rssi, s;

	/* the rx thread runs from before ieee80211_ifattach (the fw_init
	 * command chain needs the evt thread earlier still); frames that
	 * sneak in before the interface exists have nowhere to go */
	if (!sc->sc_if_attached)
		return;

	if (buf[2] & 0x10)		/* msg frame on the wrong endpoint */
		return;
	if (mpdu_len < sizeof(*wh) ||
	    AIC8800_RX_MPDU_OFF + mpdu_len > count)
		return;

	status = le32dec(&buf[AIC8800_RX_STATUS_OFF]);
	if (AIC8800_RX_FCS_ERR(status)) {
		sc->sc_rx_fcserr++;
		return;
	}

	wh = (struct ieee80211_frame *)mpdu;
	hdr_len = sizeof(*wh);
	if ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS) ==
	    IEEE80211_FC0_SUBTYPE_QOS)
		hdr_len += 2;

	m = m_gethdr(M_DONTWAIT, MT_DATA);
	if (m == NULL)
		return;
	MCLGET(m, M_DONTWAIT);
	if ((m->m_flags & M_EXT) == 0) {
		m_freem(m);
		return;
	}

	rssi1 = (int8_t)buf[AIC8800_RX_RSSI1_OFF];
	rssileg = (int8_t)buf[AIC8800_RX_RSSI_LEG_OFF];
	rssi = rssi1 != 0 ? -rssi1 : -rssileg;
	if (rssi < 0)
		rssi = 0;

	if ((wh->i_fc[1] & IEEE80211_FC1_WEP) != 0 &&
	    AIC8800_RX_DECR_STATUS(status) == AIC8800_DECR_CCMP128) {
		/* firmware already decrypted: drop the CCMP header */
		if (hdr_len + 8 > mpdu_len) {
			m_freem(m);
			return;
		}
		memcpy(mtod(m, void *), mpdu, hdr_len);
		memcpy(mtod(m, uint8_t *) + hdr_len, mpdu + hdr_len + 8,
		    mpdu_len - hdr_len - 8);
		wh = mtod(m, struct ieee80211_frame *);
		wh->i_fc[1] &= ~IEEE80211_FC1_WEP;
		m->m_len = m->m_pkthdr.len = mpdu_len - 8;
	} else if ((wh->i_fc[1] & IEEE80211_FC1_WEP) != 0) {
		/* encrypted but not decrypted for us */
		m_freem(m);
		sc->sc_rx_decrerr++;
		return;
	} else {
		memcpy(mtod(m, void *), mpdu, mpdu_len);
		m->m_len = m->m_pkthdr.len = mpdu_len;
	}

	s = splnet();
	ni = ieee80211_find_rxnode(ic,
		    (const struct ieee80211_frame_min *)mtod(m, void *));
	ieee80211_input(ic, m, ni, rssi, 0);
	ieee80211_free_node(ni);
	splx(s);

	sc->sc_rx_frames++;
}

static void
aic8800u_rx_thread(void *arg)
{
	struct aic8800u_softc *sc = arg;
	uint32_t count;
	int error;

	while (!sc->sc_dying) {
		error = aic8800u_bulk_read_data(sc, &count);
		if (error == ETIMEDOUT)
			continue;
		if (error != 0)
			break;
		aic8800u_rx_frame(sc, count);
	}

	mutex_enter(&sc->sc_load_mtx);
	sc->sc_rx_lwp = NULL;
	mutex_exit(&sc->sc_load_mtx);
	kthread_exit(0);
}

int
aic8800u_threads_start(struct aic8800u_softc *sc)
{
	int error;

	KASSERT(sc->sc_personality == AIC8800U_APP);
	KASSERT(sc->sc_data_ready);

	error = kthread_create(PRI_NONE, 0, NULL, aic8800u_evt_thread, sc,
	    &sc->sc_evt_lwp, "%s-evt", device_xname(sc->sc_dev));
	if (error != 0)
		return error;

	error = kthread_create(PRI_NONE, 0, NULL, aic8800u_rx_thread, sc,
	    &sc->sc_rx_lwp, "%s-rx", device_xname(sc->sc_dev));
	if (error != 0) {
		/* wake the evt thread out of its read loop; it exits on
		 * the aborted transfer and self-clears sc_evt_lwp */
		usbd_abort_pipe(sc->sc_evt_pipe);
		return error;
	}

	return 0;
}

/*
 * Send one lmac_msg command and wait for its CFM.  cfm_id is explicit
 * (SCANU_START_REQ answers with SCANU_START_CFM_ADDTIONAL, not id + 1).
 * Returns 0 with *cfm filled on success, ETIMEDOUT when the device never
 * answers (KI-036: never wait unbounded), EIO on a broken transfer.
 */
int
aic8800u_cmd_cfm(struct aic8800u_softc *sc, uint16_t id, uint16_t dest_id,
    uint16_t src_id, const void *param, size_t param_len,
    uint16_t cfm_id, void *cfm, size_t cfm_len)
{
	int error;

	KASSERT(sc->sc_transport_ready);

	if (sc->sc_personality == AIC8800U_BROM) {
		error = aic8800u_cmd_frame_and_write(sc, id, dest_id, src_id,
		    param, param_len);
		if (error != 0) {
			aprint_error_dev(sc->sc_dev,
			    "cmd %#x: bulk write failed (%d)\n", id, error);
			return error;
		}
		for (;;) {
			uint32_t count;
			bool found;

			error = aic8800u_bulk_read_evt(sc, &count);
			if (error != 0)
				return error;

			error = aic8800u_scan_evt_buf(sc, count, cfm_id,
			    cfm, cfm_len, &found);
			if (error != 0)
				return error;
			if (found)
				return 0;
		}
	}

	/*
	 * App mode: the evt thread reads the pipe and completes the
	 * command here.  One command in flight, bounded wait.  The
	 * expected CFM id is armed BEFORE the frame goes out: firmware
	 * that answers within the write-completion latency must find
	 * the waiter already posted, or the CFM gets misfiled as an
	 * unsolicited event.
	 */
	KASSERT(sc->sc_evt_lwp != NULL);

	mutex_enter(&sc->sc_cmd_mtx);
	KASSERT(sc->sc_cmd_active == false);
	sc->sc_cmd_active = true;
	sc->sc_cmd_cfm_id = cfm_id;
	sc->sc_cmd_cfm_buf = cfm;
	sc->sc_cmd_cfm_len = cfm_len;
	sc->sc_cmd_error = 0;
	mutex_exit(&sc->sc_cmd_mtx);

	error = aic8800u_cmd_frame_and_write(sc, id, dest_id, src_id,
	    param, param_len);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "cmd %#x: bulk write failed (%d)\n", id, error);
		mutex_enter(&sc->sc_cmd_mtx);
		sc->sc_cmd_active = false;
		sc->sc_cmd_cfm_buf = NULL;
		mutex_exit(&sc->sc_cmd_mtx);
		return error;
	}

	mutex_enter(&sc->sc_cmd_mtx);
	while (sc->sc_cmd_active && sc->sc_cmd_error == 0 && !sc->sc_dying) {
		if (cv_timedwait(&sc->sc_cmd_cv, &sc->sc_cmd_mtx,
		    mstohz(AIC8800_CMD_TIMEOUT_MS)) == EWOULDBLOCK)
			sc->sc_cmd_error = ETIMEDOUT;
	}
	error = sc->sc_cmd_error;
	if (error == 0 && sc->sc_dying)
		error = EIO;
	sc->sc_cmd_active = false;
	sc->sc_cmd_cfm_buf = NULL;
	mutex_exit(&sc->sc_cmd_mtx);

	return error;
}

int
aic8800u_cmd(struct aic8800u_softc *sc, uint16_t id, uint16_t dest_id,
    uint16_t src_id, const void *param, size_t param_len,
    void *cfm, size_t cfm_len)
{

	return aic8800u_cmd_cfm(sc, id, dest_id, src_id, param, param_len,
	    id + 1, cfm, cfm_len);
}

/*
 * Push one fully framed data/mgmt frame from sc_tx_buf (the worker
 * builds it; layout in aic8800_msg.h).
 */
int
aic8800u_data_write(struct aic8800u_softc *sc, size_t len)
{
	uint32_t count;
	usbd_status status;

	KASSERT(sc->sc_data_ready);

	usbd_setup_xfer(sc->sc_tx_xfer, NULL, sc->sc_tx_buf, len,
	    USBD_SYNCHRONOUS, AIC8800_TX_TIMEOUT_MS, NULL);
	status = usbd_transfer(sc->sc_tx_xfer);
	usbd_get_xfer_status(sc->sc_tx_xfer, NULL, NULL, &count, &status);
	if (status != USBD_NORMAL_COMPLETION || count != len)
		return EIO;

	return 0;
}

/*
 * Fire-and-forget command: frame it and push it out, no CFM wait.  Used
 * for SCANU_START_REQ, whose completion is reported by the async
 * SCANU_START_CFM (0x1001) event -- this firmware build never sends the
 * ADDTIONAL CFM (0x1009) the vendor driver waits for.
 */
int
aic8800u_cmd_send(struct aic8800u_softc *sc, uint16_t id, uint16_t dest_id,
    uint16_t src_id, const void *param, size_t param_len)
{

	KASSERT(sc->sc_transport_ready);

	return aic8800u_cmd_frame_and_write(sc, id, dest_id, src_id,
	    param, param_len);
}

int
aic8800u_dbg_read32(struct aic8800u_softc *sc, uint32_t addr, uint32_t *val)
{
	uint8_t req[AIC8800_MEM_READ_REQ_LEN];
	uint8_t cfm[AIC8800_MEM_READ_CFM_LEN];
	int error;

	req[0] = addr & 0xff;
	req[1] = (addr >> 8) & 0xff;
	req[2] = (addr >> 16) & 0xff;
	req[3] = (addr >> 24) & 0xff;

	error = aic8800u_cmd(sc, AIC8800_DBG_MEM_READ_REQ, AIC8800_TASK_DBG,
	    AIC8800_DRV_TASK_ID, req, sizeof(req), cfm, sizeof(cfm));
	if (error != 0)
		return error;

	*val = cfm[4] | (cfm[5] << 8) | (cfm[6] << 16) | ((uint32_t)cfm[7] << 24);
	return 0;
}

int
aic8800u_dbg_write32(struct aic8800u_softc *sc, uint32_t addr, uint32_t val)
{
	uint8_t req[AIC8800_MEM_WRITE_REQ_LEN];

	req[0] = addr & 0xff;
	req[1] = (addr >> 8) & 0xff;
	req[2] = (addr >> 16) & 0xff;
	req[3] = (addr >> 24) & 0xff;
	req[4] = val & 0xff;
	req[5] = (val >> 8) & 0xff;
	req[6] = (val >> 16) & 0xff;
	req[7] = (val >> 24) & 0xff;

	return aic8800u_cmd(sc, AIC8800_DBG_MEM_WRITE_REQ, AIC8800_TASK_DBG,
	    AIC8800_DRV_TASK_ID, req, sizeof(req), NULL, 0);
}

int
aic8800u_dbg_mask_write32(struct aic8800u_softc *sc, uint32_t addr,
    uint32_t mask, uint32_t val)
{
	uint8_t req[AIC8800_MEM_MASK_WRITE_REQ_LEN];

	req[0] = addr & 0xff;
	req[1] = (addr >> 8) & 0xff;
	req[2] = (addr >> 16) & 0xff;
	req[3] = (addr >> 24) & 0xff;
	req[4] = mask & 0xff;
	req[5] = (mask >> 8) & 0xff;
	req[6] = (mask >> 16) & 0xff;
	req[7] = (mask >> 24) & 0xff;
	req[8] = val & 0xff;
	req[9] = (val >> 8) & 0xff;
	req[10] = (val >> 16) & 0xff;
	req[11] = (val >> 24) & 0xff;

	return aic8800u_cmd(sc, AIC8800_DBG_MEM_MASK_WRITE_REQ, AIC8800_TASK_DBG,
	    AIC8800_DRV_TASK_ID, req, sizeof(req), NULL, 0);
}

int
aic8800u_start_app(struct aic8800u_softc *sc, uint32_t boot_addr)
{
	uint8_t req[AIC8800_START_APP_REQ_LEN];
	uint8_t cfm[AIC8800_START_APP_CFM_LEN];
	uint32_t bootstatus;
	int error;

	req[0] = boot_addr & 0xff;
	req[1] = (boot_addr >> 8) & 0xff;
	req[2] = (boot_addr >> 16) & 0xff;
	req[3] = (boot_addr >> 24) & 0xff;
	req[4] = AIC8800_HOST_START_APP_AUTO;
	req[5] = 0;
	req[6] = 0;
	req[7] = 0;

	error = aic8800u_cmd(sc, AIC8800_DBG_START_APP_REQ, AIC8800_TASK_DBG,
	    AIC8800_DRV_TASK_ID, req, sizeof(req), cfm, sizeof(cfm));
	if (error != 0)
		return error;

	bootstatus = cfm[0] | (cfm[1] << 8) | (cfm[2] << 16) |
	    ((uint32_t)cfm[3] << 24);
	aprint_normal_dev(sc->sc_dev, "start_app @%#x bootstatus %#x\n",
	    boot_addr, bootstatus);

	return 0;
}
