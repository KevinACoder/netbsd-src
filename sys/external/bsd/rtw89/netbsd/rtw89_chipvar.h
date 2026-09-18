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
struct rtw89_usb_softc;
struct rtw89_hci_ops;
struct mbuf;

struct rtw89_hw_info {
	uint8_t		mac_addr[6];
	bool		efuse_valid;
	uint8_t		fw_format;
};

/* Value snapshot: no net80211 node pointers cross the sleeping boundary. */
struct rtw89_peer_info {
	uint8_t		bssid[6];
	uint8_t		rates[16];	/* 500 kbps units, bit 7 = basic */
	uint8_t		nrates;
	uint8_t		dtim_period;
	uint16_t	aid;		/* without the two on-air reserved bits */
	uint16_t	beacon_int;
	bool		short_slot;
};

enum rtw89_peer_state {
	RTW89_PEER_NONE,
	RTW89_PEER_AUTHENTICATING,
	RTW89_PEER_AUTHENTICATED,
	RTW89_PEER_ASSOCIATED
};

int	rtw89_chip_set_peer(struct rtw89_chip *, enum rtw89_peer_state,
	    const struct rtw89_peer_info *);

typedef void (*rtw89_rx_cb_t)(void *, const uint8_t *, size_t, int);

/* usbdi transport (dev/usb/rtw89_usb.c) */
const struct rtw89_hci_ops *rtw89_usb_get_ops(void);
int	rtw89_usb_attach(struct rtw89_usb_softc *, device_t,
	    struct usbd_device *, struct usbd_interface *);
void	rtw89_usb_detach(struct rtw89_usb_softc *);

/* chip glue (dev/usb/rtw89_chip.c); runs the imported core.  Attach
 * allocates the chip handle (NULL on failure); detach consumes it. */
struct rtw89_chip *rtw89_chip_attach(device_t, struct usbd_device *,
	    struct usbd_interface *);
void	rtw89_chip_detach(struct rtw89_chip *);
int	rtw89_chip_start(struct rtw89_chip *);
void	rtw89_chip_stop(struct rtw89_chip *);
int	rtw89_chip_set_channel(struct rtw89_chip *, unsigned int);
int	rtw89_chip_tx(struct rtw89_chip *, struct mbuf *, bool, bool);
int	rtw89_chip_scan(struct rtw89_chip *, bool);
void	rtw89_chip_set_callbacks(struct rtw89_chip *, void *,
	    rtw89_rx_cb_t);
bool	rtw89_chip_ready(const struct rtw89_chip *);
const uint8_t *rtw89_chip_mac_addr(const struct rtw89_chip *,
	    struct rtw89_hw_info *);
/* The wiphy mutex serialises chip start/stop against the state machine and
 * the C2H full-handlers (mac80211 contract).  The shadow mac80211 types are
 * invisible to the front end, hence these helpers. */
void	rtw89_chip_wiphy_lock(struct rtw89_chip *);
void	rtw89_chip_wiphy_unlock(struct rtw89_chip *);

#endif /* _RTW89_CHIPVAR_H_ */
