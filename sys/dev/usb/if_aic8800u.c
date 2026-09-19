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
 * AICSemi AIC8800D80 (aic8800u) -- net80211 front end.
 *
 * The device claims itself as a fake USB stick first; umodeswitch(4)
 * flips it to the AIC boot ROM, this driver downloads the firmware
 * there, and the re-enumerated app personality carries the data plane.
 * The chip logic is a clean-room implementation of the vendor SDK's
 * lmac_msg protocol (os/aic8800, GPL); aic8800_chip.c holds the command
 * layer and aic8800_usb.c the usbdi(9) transport.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/module.h>
#include <sys/pmf.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>

#include "usbdevs.h"

#include "aic8800var.h"

static const struct usb_devno aic8800u_devs[] = {
	{ USB_VENDOR_AICSEMI, USB_PRODUCT_AICSEMI_AIC8800D80_BROM },
	{ USB_VENDOR_AICSEMI, USB_PRODUCT_AICSEMI_AIC8800D80 },
};

static int	aic8800u_match(device_t, cfdata_t, void *);
static void	aic8800u_attach(device_t, device_t, void *);
static int	aic8800u_detach(device_t, int);
static int	aic8800u_activate(device_t, enum devact);

CFATTACH_DECL_NEW(aic8800u, sizeof(struct aic8800u_softc), aic8800u_match,
    aic8800u_attach, aic8800u_detach, aic8800u_activate);

static void	aic8800u_dump_layout(struct aic8800u_softc *);
static struct usbd_interface *aic8800u_find_wifi_iface(struct aic8800u_softc *);

static int
aic8800u_match(device_t parent, cfdata_t match, void *aux)
{
	struct usb_attach_arg *uaa = aux;

	if (usb_lookup(aic8800u_devs, uaa->uaa_vendor, uaa->uaa_product) != NULL)
		return UMATCH_VENDOR_PRODUCT;

	return UMATCH_NONE;
}

/*
 * The app personality is a composite: interface 1.0 is the WiFi function
 * (vendor class ff/ff/ff), 1.1/1.2 are Bluetooth.  The boot ROM exposes
 * a single vendor-class interface.  Pick the vendor-class interface that
 * carries bulk endpoints, exactly like the Linux driver's
 * aicwf_parse_usb() does.
 */
static struct usbd_interface *
aic8800u_find_wifi_iface(struct aic8800u_softc *sc)
{
	usb_config_descriptor_t *cdesc;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	struct usbd_interface *iface;
	int i, j, error, nin, nout;

	cdesc = usbd_get_config_descriptor(sc->sc_udev);
	if (cdesc == NULL)
		return NULL;

	for (i = 0; i < cdesc->bNumInterface; i++) {
		error = usbd_device2interface_handle(sc->sc_udev, i, &iface);
		if (error != 0)
			continue;
		id = usbd_get_interface_descriptor(iface);
		if (id == NULL || id->bInterfaceClass != 0xff ||
		    id->bInterfaceSubClass != 0xff ||
		    id->bInterfaceProtocol != 0xff)
			continue;

		nin = nout = 0;
		for (j = 0; j < id->bNumEndpoints; j++) {
			ed = usbd_interface2endpoint_descriptor(iface, j);
			if (ed == NULL)
				continue;
			if (UE_GET_XFERTYPE(ed->bmAttributes) != UE_BULK)
				continue;
			if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN)
				nin++;
			else
				nout++;
		}
		if (nin < 1 || nout < 1)
			continue;

		aprint_normal_dev(sc->sc_dev,
		    "using interface %d (class %#x/%#x/%#x, %d endpoints)\n",
		    id->bInterfaceNumber, id->bInterfaceClass,
		    id->bInterfaceSubClass, id->bInterfaceProtocol,
		    id->bNumEndpoints);
		return iface;
	}

	return NULL;
}

/*
 * Record the full interface/endpoint layout of the current personality.
 * The boot-ROM and app descriptors are a gap in the Linux ground truth
 * ([待补] there); this dump is the NetBSD-side record.
 */
static void
aic8800u_dump_layout(struct aic8800u_softc *sc)
{
	usb_config_descriptor_t *cdesc;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	struct usbd_interface *iface;
	int i, j;

	cdesc = usbd_get_config_descriptor(sc->sc_udev);
	if (cdesc == NULL)
		return;

	for (i = 0; i < cdesc->bNumInterface; i++) {
		if (usbd_device2interface_handle(sc->sc_udev, i, &iface) != 0)
			continue;
		id = usbd_get_interface_descriptor(iface);
		if (id == NULL)
			continue;
		aprint_normal_dev(sc->sc_dev,
		    "if %d class %#x/%#x/%#x, %d endpoints\n",
		    id->bInterfaceNumber, id->bInterfaceClass,
		    id->bInterfaceSubClass, id->bInterfaceProtocol,
		    id->bNumEndpoints);
		for (j = 0; j < id->bNumEndpoints; j++) {
			ed = usbd_interface2endpoint_descriptor(iface, j);
			if (ed != NULL)
				aprint_normal_dev(sc->sc_dev,
				    "  ep %#x attr %#x maxpkt %d\n",
				    ed->bEndpointAddress, ed->bmAttributes,
				    UGETW(ed->wMaxPacketSize));
		}
	}
}

static void
aic8800u_attach(device_t parent, device_t self, void *aux)
{
	struct aic8800u_softc *sc = device_private(self);
	struct usb_attach_arg *uaa = aux;
	char *devinfop;
	int error;

	sc->sc_dev = self;
	sc->sc_udev = uaa->uaa_device;

	aprint_naive(": AICSemi AIC8800D80\n");
	aprint_normal("\n");

	devinfop = usbd_devinfo_alloc(sc->sc_udev, 0);
	aprint_normal_dev(self, "%s\n", devinfop);
	usbd_devinfo_free(devinfop);

	error = usbd_set_config_no(sc->sc_udev, 1, 0);
	if (error != 0) {
		aprint_error_dev(self, "failed to set configuration, err=%s\n",
		    usbd_errstr(error));
		return;
	}

	sc->sc_personality =
	    uaa->uaa_product == USB_PRODUCT_AICSEMI_AIC8800D80_BROM ?
	    AIC8800U_BROM : AIC8800U_APP;
	aprint_normal_dev(self, "%s personality (0x%04x:0x%04x)\n",
	    aic8800u_personality_name(sc->sc_personality),
	    uaa->uaa_vendor, uaa->uaa_product);

	aic8800u_dump_layout(sc);

	sc->sc_iface = aic8800u_find_wifi_iface(sc);
	if (sc->sc_iface == NULL) {
		aprint_error_dev(self, "no WiFi interface found\n");
		return;
	}

	if (aic8800u_parse_endpoints(sc, sc->sc_iface) != 0) {
		aprint_error_dev(self, "bulk endpoint layout not usable\n");
		return;
	}

	/*
	 * M1 brings the firmware up from a dedicated thread here (the
	 * boot-ROM personality) and registers with net80211 (the app
	 * personality); the interface does not exist before that.
	 */

	pmf_device_register(self, NULL, NULL);
	usbd_add_drv_event(USB_EVENT_DRIVER_ATTACH, sc->sc_udev, sc->sc_dev);
}

static int
aic8800u_activate(device_t self, enum devact act)
{
	struct aic8800u_softc *sc = device_private(self);

	switch (act) {
	case DVACT_DEACTIVATE:
		sc->sc_dying = true;
		break;
	}

	return 0;
}

static int
aic8800u_detach(device_t self, int flags)
{
	struct aic8800u_softc *sc = device_private(self);

	sc->sc_dying = true;
	pmf_device_deregister(self);

	return 0;
}
