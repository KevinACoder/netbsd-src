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
 * Driver-side state shared by if_rtw89.c (net80211 front end) and
 * rtw89_chip.c (chip glue).  Neither net80211 nor dist headers may be
 * included here: the front end cannot see the shadow mac80211 types and
 * the chip glue cannot see net80211's (they collide by name).
 */

#ifndef _RTW89VAR_H_
#define _RTW89VAR_H_

#include <sys/types.h>
#include <sys/device.h>

#include <dev/usb/usbdi.h>

/*
 * The compat `struct device' (rtw89_compat.h): a name carrier the core
 * keeps in rtwdev->dev.  Including the compat header here is safe for
 * both consumers -- the shadow mac80211 types live in rtw89_mac80211.h,
 * which neither if_rtw89.c nor rtw89_chip.c may mix with net80211.
 */
#include "rtw89_compat.h"

struct rtw89_dev;		/* dist core (core.h), opaque here */
struct rtw89_usb_softc;		/* transport, opaque here */
struct ieee80211_hw;		/* compat shadow */


/*
 * The chip handle: the rtw89_dev the core allocated, the stable compat
 * `struct device' the core keeps in rtwdev->dev (it must outlive the hw
 * private allocation, which happens later and requests firmware through
 * the device), and the identity the efuse read produced.
 */
struct rtw89_chip {
	struct rtw89_dev	*rtwdev;
	struct device		hostdev;	/* compat device carrier */

	uint8_t			mac_addr[6];
	bool			efuse_valid;
	uint8_t			fw_format;

	/* the attach sequence (pwr_on, firmware download, chip info)
	 * completed: the WCPU is running and must not be re-initialised */
	bool			fw_ready;

	/* the shadow vif is bound to the core (ops->add_interface done) */
	bool			vif_added;

	/* RX delivery (set by if_rtw89 via rtw89_chip_set_callbacks) */
	void			*rx_arg;
	void			(*rx_cb)(void *, const uint8_t *, size_t,
				    int);
};

#endif /* _RTW89VAR_H_ */
