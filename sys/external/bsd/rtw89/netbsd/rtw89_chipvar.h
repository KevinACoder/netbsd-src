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
 * Interface between the net80211 driver (if_rtw89.c) and the imported chip
 * code plus its usbdi(9) transport.  Nothing from the dist/ tree leaks
 * through here, so if_rtw89.c can include sys/net80211 without collisions.
 */

#ifndef _RTW89_CHIPVAR_H_
#define _RTW89_CHIPVAR_H_

#include <sys/types.h>
#include <sys/stdint.h>
#include <sys/device.h>
#include <dev/usb/usbdi.h>

struct rtw89_chip;
struct mbuf;

struct rtw89_hw_info {
	uint8_t		mac_addr[6];
	uint8_t		fw_version[32];
	bool		efuse_valid;
};

/* radio -> net80211: a received 802.11 frame (without FCS) */
typedef void (*rtw89_rx_cb_t)(void *, const uint8_t *, size_t, int);
/* reserved-page/beacon download finished after an association change */
typedef void (*rtw89_scan_cb_t)(void *);

struct rtw89_chip *rtw89_chip_attach(struct usbd_device *,
    struct usbd_interface *, device_t, struct rtw89_hw_info *);
void	rtw89_chip_detach(struct rtw89_chip *);
int	rtw89_chip_start(struct rtw89_chip *);
void	rtw89_chip_stop(struct rtw89_chip *);
int	rtw89_chip_set_channel(struct rtw89_chip *, unsigned int);
int	rtw89_chip_tx(struct rtw89_chip *, struct mbuf *, bool);
void	rtw89_chip_set_callbacks(struct rtw89_chip *, void *, rtw89_rx_cb_t,
	    rtw89_scan_cb_t);
void	rtw89_chip_set_assoc(struct rtw89_chip *, const uint8_t *, bool);
void	rtw89_chip_set_bssid(struct rtw89_chip *, const uint8_t *);
bool	rtw89_chip_ready(const struct rtw89_chip *);
const uint8_t *rtw89_chip_mac_addr(const struct rtw89_chip *);

#endif /* _RTW89_CHIPVAR_H_ */
