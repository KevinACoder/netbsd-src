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
 * Realtek RTL8851BU (rtw89 family) -- net80211 front end.
 *
 * The chip logic lives in external/bsd/rtw89 (imported, unmodified);
 * rtw89_usb.c is the usbdi(9) transport and rtw89_chip.c the glue.
 * The scan/association state machine follows if_rtw88.c: net80211 soft
 * scan (newstate + callout), never the firmware scan offload, which on
 * this silicon is gated behind class-9 (FW_OFLD) H2C commands whose
 * completion the FreeBSD lane proved the firmware never delivers.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/device.h>
#include <sys/mutex.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>

#include "usbdevs.h"

#include "rtw89_glue.h"
#include "rtw89_chipvar.h"

struct rtw89_softc {
	device_t		sc_dev;
	struct usbd_device	*sc_udev;
	struct usbd_interface	*sc_iface;
	struct rtw89_chip	*sc_chip;
	struct rtw89_hw_info	sc_info;
	bool			sc_chip_started;
	int			sc_dying;
};

static const struct usb_devno rtw89_devs[] = {
	{ USB_VENDOR_REALTEK, USB_PRODUCT_REALTEK_RTL8851BU },
};

static int	rtw89_match(device_t, cfdata_t, void *);
static void	rtw89_attach(device_t, device_t, void *);
static int	rtw89_detach(device_t, int);
static int	rtw89_activate(device_t, enum devact);

CFATTACH_DECL_NEW(rtw89u, sizeof(struct rtw89_softc), rtw89_match,
    rtw89_attach, rtw89_detach, rtw89_activate);

static void	rtw89_bringup_task(void *);

static int
rtw89_match(device_t parent, cfdata_t match, void *aux)
{
	struct usb_attach_arg *uaa = aux;

	if (usb_lookup(rtw89_devs, uaa->uaa_vendor, uaa->uaa_product) != NULL)
		return UMATCH_VENDOR_PRODUCT;

	return UMATCH_NONE;
}

/*
 * The WiFi function is the vendor-specific interface carrying one bulk IN
 * and at least one bulk OUT endpoint (the RTL8851BU dongle has no combo
 * Bluetooth half, but keep the same discovery as the RTL8821CU front end).
 */
static struct usbd_interface *
rtw89_find_wifi_iface(struct rtw89_softc *sc)
{
	usb_config_descriptor_t *cdesc;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	struct usbd_interface *iface;
	int i, j, error;
	int nrx, ntx;

	cdesc = usbd_get_config_descriptor(sc->sc_udev);
	if (cdesc == NULL)
		return NULL;

	for (i = 0; i < cdesc->bNumInterface; i++) {
		error = usbd_device2interface_handle(sc->sc_udev, i, &iface);
		if (error != 0)
			continue;
		id = usbd_get_interface_descriptor(iface);
		if (id == NULL || id->bInterfaceClass != 0xff)
			continue;

		nrx = ntx = 0;
		for (j = 0; j < id->bNumEndpoints; j++) {
			ed = usbd_interface2endpoint_descriptor(iface, j);
			if (ed == NULL)
				continue;
			if (UE_GET_XFERTYPE(ed->bmAttributes) != UE_BULK)
				continue;
			if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN)
				nrx++;
			else
				ntx++;
		}
		if (nrx < 1 || ntx < 1)
			continue;
		return iface;
	}

	return NULL;
}

static void
rtw89_attach(device_t parent, device_t self, void *aux)
{
	struct rtw89_softc *sc = device_private(self);
	struct usb_attach_arg *uaa = aux;

	sc->sc_dev = self;
	sc->sc_udev = uaa->uaa_device;

	aprint_naive(": Realtek RTL8851BU\n");
	aprint_normal(": Realtek RTL8851BU 802.11ax\n");

	sc->sc_iface = rtw89_find_wifi_iface(sc);
	if (sc->sc_iface == NULL) {
		aprint_error_dev(self, "no WiFi interface found\n");
		return;
	}

	/*
	 * firmware(9) cannot read images before the root is mounted, so
	 * the chip bring-up runs on the compat worker like on if_rtw88.
	 */
	if (rtw89_call_async(rtw89_bringup_task, sc) != 0)
		aprint_error_dev(self, "cannot defer bring-up\n");
}

static void
rtw89_bringup_task(void *arg)
{
	struct rtw89_softc *sc = arg;

	rtw89_workqueue_ready();

	/* Chip bring-up (firmware, efuse, net80211) lands with M1. */
	aprint_normal_dev(sc->sc_dev, "bring-up stub (M1 pending)\n");
}

static int
rtw89_detach(device_t self, int flags)
{
	struct rtw89_softc *sc = device_private(self);

	sc->sc_dying = 1;
	return 0;
}

static int
rtw89_activate(device_t self, enum devact act)
{
	struct rtw89_softc *sc = device_private(self);

	switch (act) {
	case DVACT_DEACTIVATE:
		sc->sc_dying = 1;
		return 0;
	default:
		return EOPNOTSUPP;
	}
}
