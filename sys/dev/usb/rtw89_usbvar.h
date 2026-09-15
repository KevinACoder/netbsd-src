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
 * usbdi(9) transport state for the RTL8851BU, shared between the
 * transport (rtw89_usb.c) and the chip glue (rtw89_chip.c).
 */

#ifndef _RTW89_USBVAR_H_
#define _RTW89_USBVAR_H_

#include <sys/mutex.h>
#include <sys/workqueue.h>

#include <dev/usb/usbdi.h>

#include "rtw89_compat.h"

/* vendor control transfer */
#define	RTW89_USB_VENQT			0x05
#define	RTW89_USB_VENQT_MAX_BUF		4
#define	RTW89_USB_VENQT_MAX_ATTEMPTS	10
#define	RTW89_USB_IO_ERR_LIMIT		4

/* RX geometry */
#define	RTW89_USB_RX_XFERS		8
#define	RTW89_USB_RX_BUFSZ		20480
#define	RTW89_USB_RXQ_MAX		512
/* one rx descriptor: a transfer shorter than this carries no packet */
#define	RTW89_USB_RX_MIN_LEN		24

/* TX geometry */
#define	RTW89_USB_TX_BUFSZ		16384
#define	RTW89_USB_MOD512_PADDING	4

/* R_AX_RXAGG_0: enable | 32 x 32us timeout | 5 x 4KB threshold */
#define	R_AX_RXAGG_0			0x8900
#define	RTW89_USB_RXAGG_0_8851B		(BIT(31) | (0 << 16) | (32 << 8) | 5)

/* bulksize register values (contrib usb.c) */
#define	RTW89_USB3_BULKSIZE		0x6
#define	RTW89_USB2_BULKSIZE		0x8
#define	RTW89_USB11_BULKSIZE		0x0
#define	RTW89_USB_NUMP			0x1

struct rtw89_usb_tx_slot {
	TAILQ_ENTRY(rtw89_usb_tx_slot) next;
	struct usbd_xfer	*xfer;
	void			*buf;
	struct rtw89_usb_softc	*sc;
	struct sk_buff		*skb;
	uint8_t			ch_dma;
	bool			done;
	usbd_status		done_status;
};

struct rtw89_usb_ch {
	TAILQ_HEAD(, rtw89_usb_tx_slot) free;
	TAILQ_HEAD(, rtw89_usb_tx_slot) inflight;
	struct sk_buff_head	queue;
	struct usbd_pipe	*pipe;
	uint8_t			nslots;
	uint8_t			busy;
};

/*
 * Slot pool layout per DMA channel; every other channel is not carried on
 * USB.  CH12 (H2C) gets eight slots: the firmware download streams frames
 * back to back and each needs somewhere to wait for completion.
 */
static const struct {
	uint8_t	ch_dma;
	uint8_t	nslots;
} rtw89_usb_ch_map[] = {
	{ RTW89_DMA_ACH0, 5 },
	{ RTW89_DMA_ACH1, 5 },
	{ RTW89_DMA_ACH2, 5 },
	{ RTW89_DMA_ACH3, 5 },
	{ RTW89_DMA_B0MG, 2 },
	{ RTW89_DMA_B0HI, 2 },
	{ RTW89_DMA_H2C, 1 }, /* experiment: serialise H2C */
};

#define	RTW89_USB_CH_MAX	RTW89_DMA_CH_NUM

struct rtw89_usb_softc {
	struct rtw89_dev	*rtwdev;	/* set by the chip glue */
	device_t		dev;
	struct usbd_device	*udev;
	struct usbd_interface	*iface;

	kmutex_t		tx_mtx;		/* spin: poked from callbacks */
	kmutex_t		io_mtx;		/* vendor request scratch */

	__le32			usb_data[RTW89_USB_VENQT_MAX_BUF];
	unsigned int		usb_data_idx;
	unsigned int		io_errors;
	unsigned int		tx_errprints;
	unsigned int		tx_frames;	/* handed to USB */
	unsigned int		tx_completes;	/* completed callbacks */
	unsigned int		tx_kicks;	/* kick_off calls */
	bool			detaching;
	bool			xfers_inited;

	struct usbd_pipe	*rx_pipe;
	struct usbd_xfer	*rx_xfer[RTW89_USB_RX_XFERS];
	void			*rx_buf[RTW89_USB_RX_XFERS];
	struct sk_buff_head	rx_queue;
	struct work_struct	rx_work;

	uint8_t			pipe_in;
	uint8_t			bulkout_ep[RTW89_MAX_BULKOUT_NUM];
	uint8_t			n_out_ep;

	struct rtw89_usb_ch	ch[RTW89_USB_CH_MAX];
	struct work_struct	tx_work;

	const struct rtw89_usb_info *info;
};


int	rtw89_usb_attach(struct rtw89_usb_softc *, device_t,
	    struct usbd_device *, struct usbd_interface *);
void	rtw89_usb_detach(struct rtw89_usb_softc *);

#endif /* _RTW89_USBVAR_H_ */
