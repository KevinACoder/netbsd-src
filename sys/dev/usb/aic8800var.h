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

struct aic8800u_softc {
	device_t		 sc_dev;
	struct usbd_device	*sc_udev;
	struct usbd_interface	*sc_iface;	/* WiFi function */
	enum aic8800u_personality sc_personality;
	struct aic8800u_endpoints sc_ep;

	/*
	 * Synchronous lmac_msg command channel (the loader issues one
	 * command at a time and waits for its CFM, exactly like the
	 * vendor command manager in firmware-download phase).
	 */
	struct usbd_pipe	*sc_cmd_pipe;	/* command OUT */
	struct usbd_pipe	*sc_evt_pipe;	/* event/CFM IN */
	struct usbd_xfer	*sc_cmd_xfer;
	struct usbd_xfer	*sc_evt_xfer;
	uint8_t			*sc_cmd_buf;	/* AIC8800_TX_FRAME_MAX */
	uint8_t			*sc_evt_buf;	/* AIC8800_RX_BUF_MAX */
	bool			 sc_transport_ready;

	/* loader progress */
	uint32_t		 sc_chip_id;
	uint32_t		 sc_fw_version;

	/*
	 * Bring-up thread lifecycle.  The thread self-clears
	 * sc_bringup_lwp before kthread_exit(); whoever observes the
	 * other side gone (detached / thread exited) tears the
	 * transport down (rtw8189f stop() deadlock lesson).
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

#endif	/* _DEV_USB_AIC8800VAR_H_ */
