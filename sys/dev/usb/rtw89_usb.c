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
 * struct rtw89_hci_ops on NetBSD.  M0 skeleton: the translation of the
 * FreeBSD native transport (os/freebsd/src/sys/dev/rtw89/rtw89_usb.c)
 * lands with M1.  Known pitfalls that the M1 translation must carry
 * over (FreeBSD lane evidence, doc/rk3568-itx-wiki/
 * runs/20260914-freebsd-rtw89-8851bu/README.md):
 *
 *  1. H2C/fwcmd channel is CH12; the DMA channel array is RTW89_DMA_CH_NUM
 *     (13) entries -- an off-by-one here leaves CH12 queueless.
 *  2. Completed TX slots are returned to the free pool by the worker under
 *     the lock, never in the completion callback (softint).
 *  3. R_AX_RXAGG_0 = 0x80002005 must be programmed in mac_post_init, or
 *     the firmware withholds small C2H events in greedy aggregations.
 *  4. RX aggregates hold back-to-back packets: walk desc chains with
 *     ALIGN(offset + rxd_len + pkt_size, rx_agg_alignment = 8).
 *  5. Bulk OUT frames whose (txdesc + payload) is a multiple of 512 bytes
 *     need the 4-byte MOD512 padding appended (short-packet delimiting).
 *  6. Firmware download: 2020-byte frames, WCPU_FW_CTRL == 0xe2 on done;
 *     the GET_FEATURE register handshake polls with udelay-only budget
 *     (>= 10 ms), never wall-clock (Linux lab_c2h_wall A/B evidence).
 *
 * 8821CU's reg_sec (0x4e0 write-back) does NOT apply: neither the Linux
 * rtw89 usb.c nor the FreeBSD transport has it.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include "rtw89_compat.h"
#include <linux/usb.h>

/*
 * M0: the imported rtw8851bu.c's inert driver struct references these
 * names at link time; the real usbdi transport replaces them in M1.
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
