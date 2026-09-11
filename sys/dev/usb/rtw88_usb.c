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
 * usbdi(9) transport for the imported rtw88 chip code: implements
 * struct rtw_hci_ops on top of the RTL8821CU's bulk endpoints.
 *
 * Ported from the Linux usb.c of the same import; the shape is the same
 * (register access through vendor control requests, firmware pages written
 * 196 bytes at a time, an aggregating bulk IN pipe and up to four bulk OUT
 * pipes), with these NetBSD specifics:
 *
 *   - usbdi owns the DMA buffers, so every transfer whose length is only
 *     known at callback time (RX) is copied into an sk_buff, and TX copies
 *     the sk_buff into the xfer buffer;
 *   - the RX completion callback only moves data and resubmits: the chip
 *     code it feeds (rtw_rx_query_rx_desc(), C2H handling) takes mutexes and
 *     sleeps, so the actual demultiplexing runs on the rtw88 workqueue;
 *   - the 8821C needs one extra vendor write after every register read
 *     (rtw88_usb_reg_sec), exactly like Linux.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
#include <sys/device.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>

#include "rtw88var.h"

static void	rtw88_usb_rxeof(struct usbd_xfer *, void *, usbd_status);
static void	rtw88_usb_rx_submit(struct rtw88_rx_xfer *);

#define	RTW_USB_CMD_READ	0xc0
#define	RTW_USB_CMD_WRITE	0x40
#define	RTW_USB_CMD_REQ		0x05
#define	RTW_USB_VENQT_CMD_IDX	0x00
#define	RTW_USB_VENQT_TIMEOUT	1000	/* ms */
#define	RTW_USB_REG_SEC_TIMEOUT	500

#define	FW_START_ADDR_LEGACY	0x1000
#define	RTW_USB_FW_BLOCK	196

/* ------------------------------------------------------------------ */
/* register access                                                     */
/* ------------------------------------------------------------------ */

/*
 * Bring-up instrumentation: a rate-limited trace of every register access
 * (address, width, value).  It is armed by the chip-start glue so the efuse
 * probe does not burn the budget, and goes silent after the limit to keep
 * the console usable.  Remove once the bring-up is closed.
 */
static unsigned int rtw88_trace_left;
/* Bring-up instrumentation counters (rx bulk completions / tx ring full). */
static unsigned int rtw88_rx_dbg;
static unsigned int rtw88_tx_dbg;
static unsigned int rtw88_tx_nobuf;
/* Per-pipe submit/complete pairing and a hex dump of the first transmitted
 * descriptors, to be compared byte-for-byte with the Linux lane. */
static unsigned int rtw88_tx_sub[RTW88_TX_EP_MAX];
static unsigned int rtw88_tx_comp[RTW88_TX_EP_MAX];
static unsigned int rtw88_tx_dump;
static unsigned int rtw88_rsvd_probe;

static void
rtw88_trace(const char *fmt, ...)
{
	va_list ap;

	if (rtw88_trace_left == 0)
		return;
	if (rtw88_trace_left == 1) {
		rtw88_trace_left = 0;
		printf("rtw88t: trace limit hit, muting\n");
		return;
	}
	printf("rtw88t %u: ", 1500 - rtw88_trace_left);
	rtw88_trace_left--;
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
}

void
rtw88_trace_arm(unsigned int n)
{

	rtw88_trace_left = n;
	if (n != 0)
		printf("rtw88t: armed for %u lines\n", n);
}

/*
 * Register accesses go through vendor control requests; on the 8821C the
 * always-powered sections additionally need one byte written to 0x4e0 after
 * every access, reads and writes alike (the "register security" workaround
 * of the vendor driver; missing it after writes leaves the power-on sequence
 * and the firmware download with silently dropped write steps).
 */
static void
rtw88_usb_reg_sec(struct rtw_dev *rtwdev, u32 addr, __le32 *data)
{
	struct rtw88_usb *usb = rtw88_usb_from_dev(rtwdev);
	usb_device_request_t req;
	bool reg_on_section = false;
	usbd_status err;

	if (addr <= 0xff || (addr >= 0x1000 && addr <= 0x10ff))
		reg_on_section = true;
	if (!reg_on_section)
		return;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = RTW_USB_CMD_REQ;
	USETW(req.wValue, 0x4e0);
	USETW(req.wIndex, RTW_USB_VENQT_CMD_IDX);
	USETW(req.wLength, 1);

	err = usbd_do_request_flags(usb->udev, &req, data, 0, NULL,
	    RTW_USB_REG_SEC_TIMEOUT);
	if (err != USBD_NORMAL_COMPLETION)
		rtw_warn(rtwdev, "%s: reg 0x%x write failed: %s\n", __func__,
		    0x4e0, usbd_errstr(err));
}

static u32
rtw88_usb_read(struct rtw_dev *rtwdev, u32 addr, u16 len)
{
	struct rtw88_usb *usb = rtw88_usb_from_dev(rtwdev);
	usb_device_request_t req;
	u_int32_t actlen = 0;
	__le32 *data;
	unsigned int idx;
	usbd_status err;

	mutex_enter(&usb->reg_mtx);
	idx = usb->usb_data_index;
	usb->usb_data_index = (idx + 1) & (__arraycount(usb->usb_data) - 1);
	mutex_exit(&usb->reg_mtx);

	data = &usb->usb_data[idx];
	*data = 0;

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = RTW_USB_CMD_REQ;
	USETW(req.wValue, addr);
	USETW(req.wIndex, RTW_USB_VENQT_CMD_IDX);
	USETW(req.wLength, len);

	err = usbd_do_request_flags(usb->udev, &req, data, 0, &actlen,
	    RTW_USB_VENQT_TIMEOUT);
	if (err != USBD_NORMAL_COMPLETION) {
		rtw_dbg(rtwdev, RTW_DBG_USB,
		    "%s: read 0x%x len %u failed: %s\n", __func__, addr, len,
		    usbd_errstr(err));
		rtw88_trace("rd%u 0x%x FAILED %s\n", len, addr,
		    usbd_errstr(err));
		return 0;
	}

	rtw88_usb_reg_sec(rtwdev, addr, data);

	rtw88_trace("rd%u 0x%x = 0x%08x\n", len, addr, le32_to_cpu(*data));

	return le32_to_cpu(*data);
}

static u8
rtw88_usb_read8(struct rtw_dev *rtwdev, u32 addr)
{
	return (u8)rtw88_usb_read(rtwdev, addr, 1);
}

static u16
rtw88_usb_read16(struct rtw_dev *rtwdev, u32 addr)
{
	return (u16)rtw88_usb_read(rtwdev, addr, 2);
}

static u32
rtw88_usb_read32(struct rtw_dev *rtwdev, u32 addr)
{
	return rtw88_usb_read(rtwdev, addr, 4);
}

static void
rtw88_usb_write(struct rtw_dev *rtwdev, u32 addr, u32 val, int len)
{
	struct rtw88_usb *usb = rtw88_usb_from_dev(rtwdev);
	usb_device_request_t req;
	__le32 *data;
	unsigned int idx;
	usbd_status err;

	mutex_enter(&usb->reg_mtx);
	idx = usb->usb_data_index;
	usb->usb_data_index = (idx + 1) & (__arraycount(usb->usb_data) - 1);
	mutex_exit(&usb->reg_mtx);

	data = &usb->usb_data[idx];
	*data = cpu_to_le32(val);

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = RTW_USB_CMD_REQ;
	USETW(req.wValue, addr);
	USETW(req.wIndex, RTW_USB_VENQT_CMD_IDX);
	USETW(req.wLength, len);

	err = usbd_do_request_flags(usb->udev, &req, data, 0, NULL,
	    RTW_USB_VENQT_TIMEOUT);
	if (err != USBD_NORMAL_COMPLETION)
		rtw_dbg(rtwdev, RTW_DBG_USB,
		    "%s: write 0x%x len %d failed: %s\n", __func__, addr, len,
		    usbd_errstr(err));
	else
		rtw88_trace("wr%u 0x%x <- 0x%08x\n", len, addr, val);

	rtw88_usb_reg_sec(rtwdev, addr, data);
}

static void
rtw88_usb_write8(struct rtw_dev *rtwdev, u32 addr, u8 val)
{
	rtw88_usb_write(rtwdev, addr, val, 1);
}

static void
rtw88_usb_write16(struct rtw_dev *rtwdev, u32 addr, u16 val)
{
	rtw88_usb_write(rtwdev, addr, val, 2);
}

static void
rtw88_usb_write32(struct rtw_dev *rtwdev, u32 addr, u32 val)
{
	rtw88_usb_write(rtwdev, addr, val, 4);
}

/* ------------------------------------------------------------------ */
/* firmware download                                                   */
/* ------------------------------------------------------------------ */

/*
 * Firmware is written straight into the chip's memory window: each page is
 * selected in REG_MCUFW_CTRL and then written 196 bytes at a time (8 for the
 * tail, 1 for the last odd byte), all as vendor control transfers.
 */
static void
rtw88_usb_write_firmware_page(struct rtw_dev *rtwdev, u32 page,
    const u8 *data, u32 size)
{
	struct rtw88_usb *usb = rtw88_usb_from_dev(rtwdev);
	usb_device_request_t req;
	const u8 *p = data;
	u32 addr = FW_START_ADDR_LEGACY;
	u32 n;
	usbd_status err;

	rtw_write32_mask(rtwdev, REG_MCUFW_CTRL, BIT_ROM_PGE, page);

	while (size > 0) {
		if (size >= RTW_USB_FW_BLOCK)
			n = RTW_USB_FW_BLOCK;
		else if (size >= 8)
			n = 8;
		else
			n = 1;

		req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
		req.bRequest = RTW_USB_CMD_REQ;
		USETW(req.wValue, addr);
		USETW(req.wIndex, RTW_USB_VENQT_CMD_IDX);
		USETW(req.wLength, n);

		err = usbd_do_request_flags(usb->udev, &req, __UNCONST(p), 0,
		    NULL, 500);
		if (err != USBD_NORMAL_COMPLETION) {
			rtw_err(rtwdev,
			    "%s: write 0x%x len %u failed: %s\n", __func__,
			    addr, n, usbd_errstr(err));
			break;
		}

		addr += n;
		p += n;
		size -= n;
	}
}

/* ------------------------------------------------------------------ */
/* TX path                                                             */
/* ------------------------------------------------------------------ */

static int
rtw88_dma_mapping_to_ep(enum rtw_dma_mapping dma_mapping)
{
	switch (dma_mapping) {
	case RTW_DMA_MAPPING_HIGH:
		return 0;
	case RTW_DMA_MAPPING_NORMAL:
		return 1;
	case RTW_DMA_MAPPING_LOW:
		return 2;
	case RTW_DMA_MAPPING_EXTRA:
		return 3;
	default:
		return -EINVAL;
	}
}

static void
rtw88_usb_txeof(struct usbd_xfer *xfer, void *priv, usbd_status status)
{
	struct rtw88_tx_xfer *tx = priv;
	struct rtw88_usb *usb = tx->usb;

	if (rtw88_tx_dbg < 10)
		printf("rtw88dbg tx done #%u: ep %d status %d\n",
		    rtw88_tx_dbg, tx->ep, status);
	rtw88_tx_dbg++;
	if (tx->ep >= 0 && tx->ep < RTW88_TX_EP_MAX)
		rtw88_tx_comp[tx->ep]++;

	if (tx->skb != NULL) {
		rtw88_skb_free(tx->skb);
		tx->skb = NULL;
	}

	if (status == USBD_STALLED)
		usbd_clear_endpoint_stall_async(tx->pipe);

	TAILQ_INSERT_TAIL(&usb->tx_free[tx->ep], tx, next);
	rtw88_work_enqueue(&usb->tx_work);
}

static void
rtw88_usb_tx_submit(struct rtw88_usb *usb)
{
	int i;

	mutex_enter(&usb->tx_mtx);

	for (i = 0; i < usb->n_tx_pipe; i++) {
		struct sk_buff *skb;
		struct rtw88_tx_xfer *tx;
		usbd_status err;

		while ((skb = rtw88_skb_dequeue(&usb->tx_queue[i])) != NULL) {
			tx = TAILQ_FIRST(&usb->tx_free[i]);
			if (tx == NULL) {
				if (rtw88_tx_nobuf < 3) {
					rtw_err(usb->rtwdev,
					    "%s: no free tx buffer on pipe %d\n",
					    __func__, i);
					rtw88_tx_nobuf++;
				}
				rtw88_skb_free(skb);
				continue;
			}
			TAILQ_REMOVE(&usb->tx_free[i], tx, next);

			if (skb->len > RTW88_TX_BUFSZ) {
				rtw_err(usb->rtwdev,
				    "%s: tx packet %u too large\n", __func__,
				    skb->len);
				rtw88_skb_free(skb);
				TAILQ_INSERT_TAIL(&usb->tx_free[i], tx, next);
				continue;
			}

			memcpy(tx->buf, skb->data, skb->len);
			tx->skb = skb;

			rtw88_tx_sub[i]++;
			/*
			 * Dump BEFORE usbd_transfer(): the completion callback
			 * can run synchronously inside usbd_transfer and free
			 * the skb, so nothing owned by the skb may be touched
			 * afterwards (the usbd buffer itself stays valid).
			 */
			if (rtw88_tx_dump < 3) {
				const uint8_t *p = tx->buf;
				unsigned int len = skb->len;
				unsigned int n;

				printf("rtw88dbg tx dump #%u: pipe %d len %u\n",
				    rtw88_tx_dump, i, len);
				for (n = 0; n < 48; n++)
					printf("%02x%s", p[n],
					    (n % 16 == 15) ? "\n" : " ");
				printf("rtw88dbg tx dump #%u data:",
				    rtw88_tx_dump);
				for (n = 48; n < len && n < 80; n++)
					printf(" %02x", p[n]);
				printf("\n");
				rtw88_tx_dump++;
			}

			usbd_setup_xfer(tx->xfer, tx, tx->buf, skb->len,
			    USBD_FORCE_SHORT_XFER, RTW88_TX_TIMEOUT,
			    rtw88_usb_txeof);
			err = usbd_transfer(tx->xfer);
			if (err != USBD_NORMAL_COMPLETION &&
			    err != USBD_IN_PROGRESS) {
				rtw88_skb_free(skb);
				tx->skb = NULL;
				TAILQ_INSERT_TAIL(&usb->tx_free[i], tx, next);
				rtw_err(usb->rtwdev,
				    "%s: tx transfer failed: %s\n", __func__,
				    usbd_errstr(err));
			}
		}
	}
	mutex_exit(&usb->tx_mtx);
}

static void
rtw88_usb_tx_work(struct work_struct *w)
{
	struct rtw88_usb *usb = container_of(w, struct rtw88_usb, tx_work);

	rtw88_usb_tx_submit(usb);
}

static int
rtw88_usb_tx_write(struct rtw_dev *rtwdev, struct rtw_tx_pkt_info *pkt_info,
    struct sk_buff *skb)
{
	struct rtw88_usb *usb = rtw88_usb_from_dev(rtwdev);
	const struct rtw_chip_info *chip = rtwdev->chip;
	struct rtw_tx_desc *tx_desc;
	int ep;

	if (pkt_info->qsel >= TX_DESC_QSEL_MAX) {
		rtw_err(rtwdev, "%s: invalid qsel %u\n", __func__,
		    pkt_info->qsel);
		return -EINVAL;
	}
	ep = usb->qsel_to_ep[pkt_info->qsel];
	if (ep < 0 || ep >= usb->n_tx_pipe) {
		rtw_err(rtwdev, "%s: qsel %u has no endpoint\n", __func__,
		    pkt_info->qsel);
		return -EINVAL;
	}

	tx_desc = (struct rtw_tx_desc *)skb_push(skb, chip->tx_pkt_desc_sz);
	memset(tx_desc, 0, chip->tx_pkt_desc_sz);
	rtw_tx_fill_tx_desc(rtwdev, pkt_info, tx_desc);
	rtw_tx_fill_txdesc_checksum(rtwdev, pkt_info, tx_desc);

	rtw88_skb_queue_tail(&usb->tx_queue[ep], skb);
	return 0;
}

static void
rtw88_usb_tx_kick_off(struct rtw_dev *rtwdev)
{
	struct rtw88_usb *usb = rtw88_usb_from_dev(rtwdev);

	/*
	 * Submit right away rather than through the workqueue: the chip code
	 * polls the hardware for packets it has just queued (firmware and
	 * reserved page download) and would otherwise wait for a workqueue
	 * that it is itself occupying.
	 */
	rtw88_usb_tx_submit(usb);
}

/*
 * Reserved page (beacon/keep-alive) and H2C packets are plain TX frames with
 * a fixed queue selection and the packet info filled in by the caller.
 */
static int
rtw88_usb_write_data(struct rtw_dev *rtwdev,
    struct rtw_tx_pkt_info *pkt_info, u8 *buf)
{
	const struct rtw_chip_info *chip = rtwdev->chip;
	struct sk_buff *skb;
	unsigned int size = pkt_info->tx_pkt_size;
	int ret;

	skb = alloc_skb(chip->tx_pkt_desc_sz + size, GFP_KERNEL);
	if (skb == NULL)
		return -ENOMEM;

	/* room for the descriptor that rtw88_usb_tx_write() pushes later */
	skb_reserve(skb, chip->tx_pkt_desc_sz);
	skb_put_data(skb, buf, size);

	ret = rtw88_usb_tx_write(rtwdev, pkt_info, skb);
	if (ret == 0)
		rtw88_usb_tx_kick_off(rtwdev);
	else
		rtw88_skb_free(skb);

	return ret;
}

static int
rtw88_usb_write_data_rsvd_page(struct rtw_dev *rtwdev, u8 *buf, u32 size)
{
	const struct rtw_chip_info *chip = rtwdev->chip;
	struct rtw_tx_pkt_info pkt_info = {0};

	/*
	 * Bring-up probe: 0x290 lives in the switchable register domain.
	 * Reads of 0xea mean the domain went dark; sample it periodically
	 * during the firmware download to find the step that kills it.
	 * From page 57 on, arm the register trace so the download tail,
	 * the download end flow and mac_init land in the serial ring.
	 */
	if ((rtw88_rsvd_probe++ & 0x0f) == 0 &&
	    rtw_read8(rtwdev, REG_RXDMA_MODE) == 0xea)
		printf("rtw88dbg domain dark at rsvd page #%u\n",
		    rtw88_rsvd_probe - 1);
	if (rtw88_rsvd_probe == 57)
		rtw88_trace_arm(500);

	pkt_info.tx_pkt_size = size;
	pkt_info.qsel = TX_DESC_QSEL_BEACON;
	pkt_info.offset = chip->tx_pkt_desc_sz;
	pkt_info.ls = true;

	return rtw88_usb_write_data(rtwdev, &pkt_info, buf);
}

static int
rtw88_usb_write_data_h2c(struct rtw_dev *rtwdev, u8 *buf, u32 size)
{
	struct rtw_tx_pkt_info pkt_info = {0};

	pkt_info.tx_pkt_size = size;
	pkt_info.qsel = TX_DESC_QSEL_H2C;

	rtw_info(rtwdev, "h2c packet %u bytes, 0x290=0x%02x\n", size,
	    rtw_read8(rtwdev, REG_RXDMA_MODE));

	return rtw88_usb_write_data(rtwdev, &pkt_info, buf);
}

/* ------------------------------------------------------------------ */
/* RX path                                                             */
/* ------------------------------------------------------------------ */

/*
 * The bulk IN pipe carries a concatenation of packets, each with a receive
 * descriptor and a driver info area in front and padded to 8 bytes; the
 * demultiplexing is done on the workqueue (see rtw88_chip.c).
 */
static void
rtw88_usb_rx_submit(struct rtw88_rx_xfer *rx)
{
	usbd_status err;

	if (rx->xfer == NULL)
		return;
	usbd_setup_xfer(rx->xfer, rx, rx->buf, RTW88_RX_BUFSZ,
	    USBD_SHORT_XFER_OK, USBD_NO_TIMEOUT, rtw88_usb_rxeof);
	err = usbd_transfer(rx->xfer);
	if (err != USBD_NORMAL_COMPLETION && err != USBD_IN_PROGRESS)
		printf("rtw88: rx submit failed: %s\n", usbd_errstr(err));
}

static void
rtw88_usb_rxeof(struct usbd_xfer *xfer, void *priv, usbd_status status)
{
	struct rtw88_rx_xfer *rx = priv;
	struct rtw88_usb *usb = rx->usb;
	u_int32_t len = 0;
	struct sk_buff *skb;

	if (status == USBD_CANCELLED || status == USBD_NOT_STARTED)
		return;

	if (status != USBD_NORMAL_COMPLETION) {
		if (status == USBD_STALLED)
			usbd_clear_endpoint_stall_async(usb->rx_pipe);
		rtw88_usb_rx_submit(rx);
		return;
	}

	usbd_get_xfer_status(xfer, NULL, NULL, &len, NULL);

	/* Bring-up instrumentation: are bulk IN transfers completing? */
	if (len > 0 && (rtw88_rx_dbg < 30 || (rtw88_rx_dbg % 500) == 0))
		printf("rtw88dbg rx #%u: %u bytes\n", rtw88_rx_dbg, len);
	if (len > 0)
		rtw88_rx_dbg++;

	if (len > 0) {
		skb = alloc_skb(len, GFP_ATOMIC);
		if (skb != NULL) {
			skb_put_data(skb, rx->buf, len);
			rtw88_skb_queue_tail(&usb->rx_queue, skb);
			rtw88_work_enqueue(&usb->rx_work);
		} else {
			printf("rtw88: rx buffer allocation failed\n");
		}
	}

	rtw88_usb_rx_submit(rx);
}

/* ------------------------------------------------------------------ */
/* remaining HCI ops                                                   */
/* ------------------------------------------------------------------ */

static int
rtw88_usb_setup(struct rtw_dev *rtwdev)
{
	return 0;
}

static int
rtw88_usb_start(struct rtw_dev *rtwdev)
{
	struct rtw88_usb *usb = rtw88_usb_from_dev(rtwdev);
	int i;

	for (i = 0; i < RTW88_RX_XFER_NUM; i++)
		rtw88_usb_rx_submit(&usb->rx[i]);

	/* H2C queue pointers right after RX arming, before general_info and
	 * phydm_info are pushed over bulk OUT; the chip-start dump repeats
	 * them so a delta tells whether the device took the packets. */
	rtw_info(rtwdev,
	    "h2cq pre-gi: head 0x%08x tail 0x%08x read 0x%08x info 0x%02x pktw 0x%08x pktr 0x%08x\n",
	    rtw_read32(rtwdev, REG_H2C_HEAD), rtw_read32(rtwdev, REG_H2C_TAIL),
	    rtw_read32(rtwdev, REG_H2C_READ_ADDR),
	    rtw_read8(rtwdev, REG_H2C_INFO),
	    rtw_read32(rtwdev, REG_H2C_PKT_WRITEADDR),
	    rtw_read32(rtwdev, REG_H2C_PKT_READADDR));

	return 0;
}

static void
rtw88_usb_stop(struct rtw_dev *rtwdev)
{
}

static void
rtw88_usb_deep_ps(struct rtw_dev *rtwdev, bool enter)
{
}

static void
rtw88_usb_link_ps(struct rtw_dev *rtwdev, bool enter)
{
}

/*
 * RXDMA burst size and the "drop" threshold have to follow the negotiated USB
 * speed (the dongle is a high speed device on this board).
 */
static void
rtw88_usb_interface_cfg(struct rtw_dev *rtwdev)
{
	struct rtw88_usb *usb = rtw88_usb_from_dev(rtwdev);
	u8 rxdma, burst_size;

	/* bring-up probe: domain health right before the mac-init tail */
	rtw_info(rtwdev, "interface_cfg entry: 0x290=0x%02x\n",
	    rtw_read8(rtwdev, REG_RXDMA_MODE));

	rxdma = BIT_DMA_BURST_CNT | BIT_DMA_MODE;

	if (usb->udev->ud_speed == USB_SPEED_HIGH)
		burst_size = BIT_DMA_BURST_SIZE_512;
	else
		burst_size = BIT_DMA_BURST_SIZE_64;

	u8p_replace_bits(&rxdma, burst_size, BIT_DMA_BURST_SIZE);

	rtw_write8(rtwdev, REG_RXDMA_MODE, rxdma);
	rtw_write16_set(rtwdev, REG_TXDMA_OFFSET_CHK, BIT_DROP_DATA_EN);

	rtw_info(rtwdev, "interface_cfg exit: 0x290=0x%02x\n",
	    rtw_read8(rtwdev, REG_RXDMA_MODE));
}

static void
rtw88_usb_dynamic_rx_agg(struct rtw_dev *rtwdev, bool enable)
{
	u8 size, timeout;
	u16 val16;

	switch (rtwdev->chip->id) {
	case RTW_CHIP_TYPE_8822C:
	case RTW_CHIP_TYPE_8822B:
	case RTW_CHIP_TYPE_8821C:
	case RTW_CHIP_TYPE_8814A:
		rtw_write8_set(rtwdev, REG_TXDMA_PQ_MAP, BIT_RXDMA_AGG_EN);
		rtw_write8_clr(rtwdev, REG_RXDMA_AGG_PG_TH + 3, BIT(7));

		if (enable) {
			size = 0x5;
			timeout = 0x20;
		} else {
			size = 0x0;
			timeout = 0x1;
		}
		val16 = u16_encode_bits(size, BIT_RXDMA_AGG_PG_TH) |
		    u16_encode_bits(timeout, BIT_DMA_AGG_TO_V1);
		rtw_write16(rtwdev, REG_RXDMA_AGG_PG_TH, val16);
		break;
	default:
		break;
	}
}

static const struct rtw_hci_ops rtw88_usb_ops = {
	.tx_write = rtw88_usb_tx_write,
	.tx_kick_off = rtw88_usb_tx_kick_off,
	.setup = rtw88_usb_setup,
	.start = rtw88_usb_start,
	.stop = rtw88_usb_stop,
	.deep_ps = rtw88_usb_deep_ps,
	.link_ps = rtw88_usb_link_ps,
	.interface_cfg = rtw88_usb_interface_cfg,
	.dynamic_rx_agg = rtw88_usb_dynamic_rx_agg,
	.write_firmware_page = rtw88_usb_write_firmware_page,

	.write8 = rtw88_usb_write8,
	.write16 = rtw88_usb_write16,
	.write32 = rtw88_usb_write32,
	.read8 = rtw88_usb_read8,
	.read16 = rtw88_usb_read16,
	.read32 = rtw88_usb_read32,

	.write_data_rsvd_page = rtw88_usb_write_data_rsvd_page,
	.write_data_h2c = rtw88_usb_write_data_h2c,
};

const struct rtw_hci_ops *
rtw88_usb_get_ops(void)
{

	return &rtw88_usb_ops;
}

/*
 * Bring-up forensics, called by the chip-start glue right after the firmware
 * handshake: per-pipe TX pairing counters plus every register the mac-init
 * chain programs, to be diffed against the Linux lane's debugfs read_reg.
 */
void
rtw88_usb_dbg_dump(struct rtw_dev *rtwdev)
{
	struct rtw_fifo_conf *fifo = &rtwdev->fifo;
	unsigned int i;

	for (i = 0; i < RTW88_TX_EP_MAX; i++)
		printf("rtw88dbg pipe %u: submitted %u completed %u\n",
		    i, rtw88_tx_sub[i], rtw88_tx_comp[i]);

	rtw_info(rtwdev,
	    "fifo host side: boundary %u pg %u acq %u h2cq 0x%x fwtx 0x%x\n",
	    fifo->rsvd_boundary, fifo->rsvd_pg_num, fifo->acq_pg_num,
	    fifo->rsvd_h2cq_addr, fifo->rsvd_fw_txbuf_addr);

	rtw_info(rtwdev,
	    "h2cq: head 0x%08x tail 0x%08x read 0x%08x info 0x%02x pktw 0x%08x pktr 0x%08x csr 0x%08x\n",
	    rtw_read32(rtwdev, REG_H2C_HEAD), rtw_read32(rtwdev, REG_H2C_TAIL),
	    rtw_read32(rtwdev, REG_H2C_READ_ADDR),
	    rtw_read8(rtwdev, REG_H2C_INFO),
	    rtw_read32(rtwdev, REG_H2C_PKT_WRITEADDR),
	    rtw_read32(rtwdev, REG_H2C_PKT_READADDR),
	    rtw_read32(rtwdev, REG_H2CQ_CSR));

	rtw_info(rtwdev,
	    "regs: CR 0x%08x pqmap 0x%04x rxff_bndy 0x%08x llt 0x%08x offset_chk 0x%08x\n",
	    rtw_read32(rtwdev, REG_CR), rtw_read16(rtwdev, REG_TXDMA_PQ_MAP),
	    rtw_read32(rtwdev, REG_RXFF_BNDY),
	    rtw_read32(rtwdev, REG_AUTO_LLT_V1),
	    rtw_read32(rtwdev, REG_TXDMA_OFFSET_CHK));

	rtw_info(rtwdev,
	    "regs: fpage_ctrl2 0x%08x txdma_status 0x%08x rxdma_status 0x%08x rxdma_mode 0x%02x\n",
	    rtw_read32(rtwdev, REG_FIFOPAGE_CTRL_2),
	    rtw_read32(rtwdev, REG_TXDMA_STATUS),
	    rtw_read32(rtwdev, REG_RXDMA_STATUS),
	    rtw_read8(rtwdev, REG_RXDMA_MODE));

	rtw_info(rtwdev,
	    "regs: fpage_info %04x %04x %04x %04x %04x\n",
	    rtw_read16(rtwdev, REG_FIFOPAGE_INFO_1),
	    rtw_read16(rtwdev, REG_FIFOPAGE_INFO_2),
	    rtw_read16(rtwdev, REG_FIFOPAGE_INFO_3),
	    rtw_read16(rtwdev, REG_FIFOPAGE_INFO_4),
	    rtw_read16(rtwdev, REG_FIFOPAGE_INFO_5));

	rtw_info(rtwdev,
	    "regs: fwhw_txq 0x%08x bcnq_bdyny 0x%04x bcnq1_bdyny 0x%04x rxagg 0x%04x drvinfo_sz 0x%02x\n",
	    rtw_read32(rtwdev, REG_FWHW_TXQ_CTRL),
	    rtw_read16(rtwdev, REG_BCNQ_BDNY_V1),
	    rtw_read16(rtwdev, REG_BCNQ1_BDNY_V1),
	    rtw_read16(rtwdev, REG_RXDMA_AGG_PG_TH),
	    rtw_read8(rtwdev, REG_RX_DRVINFO_SZ));

	rtw_info(rtwdev,
	    "regs: rcr 0x%08x sys_status1+1 0x%02x 0x10c3 0x%02x mcufw 0x%08x hmetfr 0x%02x\n",
	    rtw_read32(rtwdev, REG_RCR),
	    rtw_read8(rtwdev, REG_SYS_STATUS1 + 1),
	    rtw_read8(rtwdev, 0x10c3),
	    rtw_read32(rtwdev, REG_MCUFW_CTRL),
	    rtw_read8(rtwdev, REG_HMETFR));

	rtw_info(rtwdev,
	    "regs: dmem_con 0x%08x rsv_ctrl 0x%02x sys_func_en 0x%02x cr_ext3 0x%02x\n",
	    rtw_read32(rtwdev, REG_CPU_DMEM_CON),
	    rtw_read8(rtwdev, REG_RSV_CTRL),
	    rtw_read8(rtwdev, REG_SYS_FUNC_EN),
	    rtw_read8(rtwdev, REG_CR_EXT + 3));

	/*
	 * Write/readback probe on an off-section register: separates "domain
	 * dark (reads filler, writes void)" from "alive but unprogrammed".
	 */
	{
		uint8_t saved, before, after;

		saved = rtw_read8(rtwdev, REG_RXDMA_MODE);
		rtw_write8(rtwdev, REG_RXDMA_MODE, 0x55);
		before = rtw_read8(rtwdev, REG_RXDMA_MODE);
		rtw_write8(rtwdev, REG_RXDMA_MODE, saved);
		after = rtw_read8(rtwdev, REG_RXDMA_MODE);
		rtw_info(rtwdev,
		    "probe 0x290: saved 0x%02x w0x55->r 0x%02x restore->r 0x%02x\n",
		    saved, before, after);
	}

	/* latched by the phy table loaders (dist/phy.c, bring-up only) */
	{
		extern unsigned int rtw88_dark_kind, rtw88_dark_n;
		extern unsigned int rtw88_dark_addr, rtw88_dark_data;

		if (rtw88_dark_kind != 0)
			rtw_info(rtwdev,
			    "domain dark: kind %u entry %u addr 0x%x data 0x%x\n",
			    rtw88_dark_kind, rtw88_dark_n, rtw88_dark_addr,
			    rtw88_dark_data);
		else
			rtw_info(rtwdev, "domain dark: no\n");
	}
}

/* ------------------------------------------------------------------ */
/* attach / detach                                                     */
/* ------------------------------------------------------------------ */

static int
rtw88_usb_parse(struct rtw88_chip *chip, struct usbd_interface *iface)
{
	struct rtw_dev *rtwdev = &chip->rtwdev;
	struct rtw88_usb *usb = &chip->usb;
	const struct rtw_chip_info *chipinfo = rtwdev->chip;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	const struct rtw_rqpn *rqpn;
	uint8_t rx_ep = 0, int_ep = 0;
	int i, nout = 0, ret;

	id = usbd_get_interface_descriptor(iface);
	if (id == NULL) {
		rtw_err(rtwdev, "%s: no interface descriptor\n", __func__);
		return EINVAL;
	}

	for (i = 0; i < id->bNumEndpoints; i++) {
		ed = usbd_interface2endpoint_descriptor(iface, i);
		if (ed == NULL)
			continue;

		if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN &&
		    UE_GET_XFERTYPE(ed->bmAttributes) == UE_BULK) {
			if (rx_ep != 0) {
				rtw_err(rtwdev, "%s: too many bulk IN\n",
				    __func__);
				return EINVAL;
			}
			rx_ep = ed->bEndpointAddress;
		} else if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN &&
		    UE_GET_XFERTYPE(ed->bmAttributes) == UE_INTERRUPT) {
			int_ep = ed->bEndpointAddress;
		} else if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_OUT &&
		    UE_GET_XFERTYPE(ed->bmAttributes) == UE_BULK) {
			if (nout >= RTW88_TX_EP_MAX) {
				rtw_err(rtwdev, "%s: too many bulk OUT\n",
				    __func__);
				return EINVAL;
			}
			ret = usbd_open_pipe(iface, ed->bEndpointAddress,
			    USBD_EXCLUSIVE_USE, &usb->tx_pipe[nout]);
			if (ret != 0) {
				rtw_err(rtwdev,
				    "%s: cannot open tx pipe 0x%02x: %s\n",
				    __func__, ed->bEndpointAddress,
				    usbd_errstr(ret));
				return ret;
			}
			nout++;
		}
	}

	if (rx_ep == 0 || nout < 1) {
		rtw_err(rtwdev, "%s: unexpected endpoint layout (%d out)\n",
		    __func__, nout);
		return EINVAL;
	}

	ret = usbd_open_pipe(iface, rx_ep, USBD_EXCLUSIVE_USE, &usb->rx_pipe);
	if (ret != 0) {
		rtw_err(rtwdev, "%s: cannot open rx pipe 0x%02x: %s\n",
		    __func__, rx_ep, usbd_errstr(ret));
		return ret;
	}
	if (int_ep != 0) {
		ret = usbd_open_pipe(iface, int_ep, USBD_EXCLUSIVE_USE,
		    &usb->int_pipe);
		if (ret != 0)
			usb->int_pipe = NULL;	/* optional for now */
	}

	usb->n_tx_pipe = nout;
	rtwdev->hci.bulkout_num = nout;

	rqpn = &chipinfo->rqpn_table[nout];
	usb->qsel_to_ep[TX_DESC_QSEL_TID0] =
	    rtw88_dma_mapping_to_ep(rqpn->dma_map_be);
	usb->qsel_to_ep[TX_DESC_QSEL_TID1] =
	    rtw88_dma_mapping_to_ep(rqpn->dma_map_bk);
	usb->qsel_to_ep[TX_DESC_QSEL_TID2] =
	    rtw88_dma_mapping_to_ep(rqpn->dma_map_bk);
	usb->qsel_to_ep[TX_DESC_QSEL_TID3] =
	    rtw88_dma_mapping_to_ep(rqpn->dma_map_be);
	usb->qsel_to_ep[TX_DESC_QSEL_TID4] =
	    rtw88_dma_mapping_to_ep(rqpn->dma_map_vi);
	usb->qsel_to_ep[TX_DESC_QSEL_TID5] =
	    rtw88_dma_mapping_to_ep(rqpn->dma_map_vi);
	usb->qsel_to_ep[TX_DESC_QSEL_TID6] =
	    rtw88_dma_mapping_to_ep(rqpn->dma_map_vo);
	usb->qsel_to_ep[TX_DESC_QSEL_TID7] =
	    rtw88_dma_mapping_to_ep(rqpn->dma_map_vo);
	for (i = TX_DESC_QSEL_TID8; i <= TX_DESC_QSEL_TID15; i++)
		usb->qsel_to_ep[i] = -EINVAL;
	usb->qsel_to_ep[TX_DESC_QSEL_BEACON] =
	    rtw88_dma_mapping_to_ep(rqpn->dma_map_hi);
	usb->qsel_to_ep[TX_DESC_QSEL_HIGH] =
	    rtw88_dma_mapping_to_ep(rqpn->dma_map_hi);
	usb->qsel_to_ep[TX_DESC_QSEL_MGMT] =
	    rtw88_dma_mapping_to_ep(rqpn->dma_map_mg);
	usb->qsel_to_ep[TX_DESC_QSEL_H2C] =
	    rtw88_dma_mapping_to_ep(rqpn->dma_map_hi);

	rtw_info(rtwdev, "%d tx pipe(s), rx 0x%02x, interrupt 0x%02x\n",
	    nout, rx_ep, int_ep);
	return 0;
}

int
rtw88_usb_attach(struct rtw88_chip *chip, struct usbd_interface *iface)
{
	struct rtw88_usb *usb = &chip->usb;
	struct rtw_dev *rtwdev = &chip->rtwdev;
	int i, ret;

	/* Bring-up instrumentation: trace writes, firmware and state changes. */
	rtw_debug_mask |= RTW_DBG_USB;

	usb->rtwdev = rtwdev;
	netbsd_mutex_init(&usb->reg_mtx);
	netbsd_mutex_init(&usb->tx_mtx);
	rtw88_skb_queue_init(&usb->rx_queue);
	for (i = 0; i < RTW88_TX_EP_MAX; i++) {
		rtw88_skb_queue_init(&usb->tx_queue[i]);
		TAILQ_INIT(&usb->tx_free[i]);
	}
	INIT_WORK(&usb->rx_work, rtw88_chip_rx_work);
	INIT_WORK(&usb->tx_work, rtw88_usb_tx_work);

	ret = rtw88_usb_parse(chip, iface);
	if (ret != 0)
		return ret;

	/*
	 * TX buffers are bound to one bulk OUT pipe each, spread as evenly as
	 * possible over the pipes the device exposes.
	 */
	for (i = 0; i < RTW88_TX_XFER_NUM; i++) {
		int ep = i % usb->n_tx_pipe;

		ret = usbd_create_xfer(usb->tx_pipe[ep], RTW88_TX_BUFSZ,
		    USBD_FORCE_SHORT_XFER, 0, &usb->tx[i].xfer);
		if (ret != 0) {
			rtw_err(rtwdev, "%s: tx xfer %d failed: %s\n", __func__,
			    i, usbd_errstr(ret));
			return ret;
		}
		usb->tx[i].buf = usbd_get_buffer(usb->tx[i].xfer);
		usb->tx[i].usb = usb;
		usb->tx[i].pipe = usb->tx_pipe[ep];
		usb->tx[i].ep = ep;
		TAILQ_INSERT_TAIL(&usb->tx_free[ep], &usb->tx[i], next);
	}

	for (i = 0; i < RTW88_RX_XFER_NUM; i++) {
		ret = usbd_create_xfer(usb->rx_pipe, RTW88_RX_BUFSZ, 0, 0,
		    &usb->rx[i].xfer);
		if (ret != 0) {
			rtw_err(rtwdev, "%s: rx xfer %d failed: %s\n", __func__,
			    i, usbd_errstr(ret));
			return ret;
		}
		usb->rx[i].buf = usbd_get_buffer(usb->rx[i].xfer);
		usb->rx[i].usb = usb;
	}

	return 0;
}

void
rtw88_usb_detach(struct rtw88_chip *chip)
{
	struct rtw88_usb *usb = &chip->usb;
	int i;

	if (usb->rx_pipe != NULL)
		usbd_abort_pipe(usb->rx_pipe);
	for (i = 0; i < usb->n_tx_pipe; i++)
		usbd_abort_pipe(usb->tx_pipe[i]);

	for (i = 0; i < RTW88_RX_XFER_NUM; i++) {
		if (usb->rx[i].xfer != NULL)
			usbd_destroy_xfer(usb->rx[i].xfer);
	}
	for (i = 0; i < RTW88_TX_XFER_NUM; i++) {
		if (usb->tx[i].skb != NULL)
			rtw88_skb_free(usb->tx[i].skb);
		if (usb->tx[i].xfer != NULL)
			usbd_destroy_xfer(usb->tx[i].xfer);
	}
	for (i = 0; i < usb->n_tx_pipe; i++)
		rtw88_skb_queue_purge(&usb->tx_queue[i]);
	rtw88_skb_queue_purge(&usb->rx_queue);

	if (usb->int_pipe != NULL)
		usbd_close_pipe(usb->int_pipe);
	if (usb->rx_pipe != NULL)
		usbd_close_pipe(usb->rx_pipe);
	for (i = 0; i < usb->n_tx_pipe; i++)
		usbd_close_pipe(usb->tx_pipe[i]);
	netbsd_mutex_destroy(&usb->reg_mtx);
	netbsd_mutex_destroy(&usb->tx_mtx);
}
