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
 * AIC8800D80 usbdi(9) transport -- endpoint discovery and the
 * synchronous lmac_msg command channel.
 *
 * The bulk layout follows the vendor driver's order-based rule: the
 * first bulk IN and first bulk OUT of the WiFi interface are the data
 * pipes; in the app personality a second bulk pair carries lmac_msg
 * command frames.  The boot ROM has no dedicated message endpoints and
 * sends its command channel over the first bulk pair.
 *
 * The command channel is synchronous: one command in flight, the same
 * thread writes the frame and then reads event frames until the CFM
 * with id + 1 arrives (or the budget expires -- every wait has a
 * timeout, the KI-036 lesson from the Linux lane).  Unsolicited
 * CFG_PRINT frames are consumed and discarded on the way.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kmem.h>
#include <sys/device.h>

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
 * pair when it has one.
 */
int
aic8800u_transport_init(struct aic8800u_softc *sc)
{
	uint8_t cmd_addr, evt_addr;
	int error;

	KASSERT(sc->sc_transport_ready == false);

	if (sc->sc_personality == AIC8800U_BROM ||
	    !sc->sc_ep.msg_in.present || !sc->sc_ep.msg_out.present) {
		cmd_addr = sc->sc_ep.data_out.addr;
		evt_addr = sc->sc_ep.data_in.addr;
	} else {
		cmd_addr = sc->sc_ep.msg_out.addr;
		evt_addr = sc->sc_ep.msg_in.addr;
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

	sc->sc_transport_ready = true;
	return 0;

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
aic8800u_bulk_read(struct aic8800u_softc *sc, uint32_t *count)
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

/*
 * Walk the event frames in one bulk-IN buffer and look for the CFM with
 * the expected id.  The wire layout of an event frame is documented in
 * aic8800_msg.h; a buffer can carry several packets, each padded to a
 * multiple of 4.
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

		if ((buf[off + 2] & AIC8800_USB_TYPE_CFG) == AIC8800_USB_TYPE_CFG &&
		    type == AIC8800_USB_TYPE_CFG_CMD_RSP) {
			msg_id = buf[off + 4] | (buf[off + 5] << 8);
			msg_param_len = buf[off + 10] | (buf[off + 11] << 8);

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
		} else if (sc->sc_personality == AIC8800U_BROM) {
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
 * Send one lmac_msg command and wait for its CFM (id + 1).  Returns 0
 * with *cfm filled on success, ETIMEDOUT when the device never answers
 * (KI-036: never wait unbounded), EIO on a broken transfer.
 */
int
aic8800u_cmd(struct aic8800u_softc *sc, uint16_t id, uint16_t dest_id,
    uint16_t src_id, const void *param, size_t param_len,
    void *cfm, size_t cfm_len)
{
	uint8_t *buf = sc->sc_cmd_buf;
	size_t len = 8 + param_len;	/* lmac_msg header + param */
	uint16_t cfm_id = id + 1;
	int error;

	KASSERT(sc->sc_transport_ready);
	KASSERT(param_len <= AIC8800_MEM_BLOCK_WRITE_REQ_LEN);

	/*
	 * Frame it.  The 12-bit length field counts the lmac_msg bytes
	 * plus four; the dummy word sits between the frame header and
	 * the lmac_msg header.
	 */
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

	error = aic8800u_bulk_write(sc, buf, len + 8);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "cmd %#x: bulk write failed (%d)\n", id, error);
		return error;
	}

	for (;;) {
		uint32_t count;
		bool found;

		error = aic8800u_bulk_read(sc, &count);
		if (error != 0)
			return error;

		error = aic8800u_scan_evt_buf(sc, count, cfm_id, cfm, cfm_len,
		    &found);
		if (error != 0)
			return error;
		if (found)
			return 0;
	}
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
