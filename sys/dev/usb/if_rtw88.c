/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by the RK3568 bring-up lab.
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
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.  IN NO EVENT
 * SHALL THE FOUNDATION OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Realtek RTL8811CU/RTL8821CU USB 802.11ac driver (rtw88 family).
 *
 * The rtw88 chips are unrelated to urtwn(4) (RTL8188/8192 family): they
 * need the rtw88 register map, PHY tables, firmware download and H2C/C2H
 * protocols.  The chip logic is imported (dual GPL-2.0/BSD-3-Clause) under
 * sys/external/bsd/rtw88; this file and its siblings are the NetBSD glue.
 *
 * Bring-up is staged.  This first revision only claims the interface and
 * dumps the configuration's interfaces and endpoints, so the descriptor
 * layout can be confirmed against the Linux ground truth recorded in
 * doc/rk3568-itx-wiki/runs/20260911-linux-rtl8821au/.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/module.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>

#include "usbdevs.h"

#ifdef RTW88_DEBUG
int rtw88_debug = 1;
#define DPRINTF(x)	do { if (rtw88_debug) printf x; } while (/*CONSTCOND*/0)
#define DPRINTFN(n, x)	do { if (rtw88_debug > (n)) printf x; } while (/*CONSTCOND*/0)
#else
#define DPRINTF(x)
#define DPRINTFN(n, x)
#endif

struct rtw88_softc {
	device_t		sc_dev;
	struct usbd_device	*sc_udev;
	struct usbd_interface	*sc_iface;
};

static const struct usb_devno rtw88_devs[] = {
	{ USB_VENDOR_REALTEK, USB_PRODUCT_REALTEK_RTL8821CU },
};

static int	rtw88_match(device_t, cfdata_t, void *);
static void	rtw88_attach(device_t, device_t, void *);
static int	rtw88_detach(device_t, int);
static int	rtw88_activate(device_t, enum devact);

CFATTACH_DECL_NEW(rtw88u, sizeof(struct rtw88_softc), rtw88_match,
    rtw88_attach, rtw88_detach, rtw88_activate);

static int
rtw88_match(device_t parent, cfdata_t match, void *aux)
{
	struct usb_attach_arg *uaa = aux;

	if (usb_lookup(rtw88_devs, uaa->uaa_vendor, uaa->uaa_product) != NULL)
		return UMATCH_VENDOR_PRODUCT;

	return UMATCH_NONE;
}

static void
rtw88_attach(device_t parent, device_t self, void *aux)
{
	struct rtw88_softc *sc = device_private(self);
	struct usb_attach_arg *uaa = aux;
	usb_config_descriptor_t *cdesc;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	struct usbd_interface *iface;
	char *devinfop;
	int error, i, j;

	sc->sc_dev = self;
	sc->sc_udev = uaa->uaa_device;

	aprint_naive("\n");
	aprint_normal("\n");

	devinfop = usbd_devinfo_alloc(sc->sc_udev, 0);
	aprint_normal_dev(self, "%s\n", devinfop);
	usbd_devinfo_free(devinfop);

	/* Select the (single) configuration so interfaces are addressable. */
	error = usbd_set_config_no(sc->sc_udev, 1, 0);
	if (error != 0) {
		aprint_error_dev(self, "failed to set configuration, err=%s\n",
		    usbd_errstr(error));
		return;
	}

	cdesc = usbd_get_config_descriptor(sc->sc_udev);
	if (cdesc == NULL) {
		aprint_error_dev(self, "could not read configuration\n");
		return;
	}

	aprint_normal_dev(self, "config %d, %d interface(s), %s\n",
	    cdesc->bConfigurationValue, cdesc->bNumInterface,
	    cdesc->bNumInterface > 1 ? "combo device" : "single function");

	for (i = 0; i < cdesc->bNumInterface; i++) {
		error = usbd_device2interface_handle(sc->sc_udev, i, &iface);
		if (error != 0) {
			aprint_error_dev(self, "interface %d: no handle, err=%s\n",
			    i, usbd_errstr(error));
			continue;
		}
		id = usbd_get_interface_descriptor(iface);
		if (id == NULL)
			continue;
		aprint_normal_dev(self, " interface %d: class %#x/%#x/%#x,"
		    " %d endpoint(s), alt %d\n", id->bInterfaceNumber,
		    id->bInterfaceClass, id->bInterfaceSubClass,
		    id->bInterfaceProtocol, id->bNumEndpoints,
		    id->bAlternateSetting);
		for (j = 0; j < id->bNumEndpoints; j++) {
			ed = usbd_interface2endpoint_descriptor(iface, j);
			if (ed == NULL)
				continue;
			aprint_normal_dev(self, "  endpoint %#x attr %#x"
			    " maxpkt %d interval %d\n", ed->bEndpointAddress,
			    ed->bmAttributes, UGETW(ed->wMaxPacketSize),
			    ed->bInterval);
		}
		if (sc->sc_iface == NULL)
			sc->sc_iface = iface;
	}

	aprint_normal_dev(self, "stub: chip bring-up not implemented yet\n");
}

static int
rtw88_detach(device_t self, int flags)
{

	return 0;
}

static int
rtw88_activate(device_t self, enum devact act)
{

	return 0;
}
