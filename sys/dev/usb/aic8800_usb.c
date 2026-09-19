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
 * AIC8800D80 usbdi(9) transport -- endpoint discovery.
 *
 * The bulk layout follows the vendor driver's order-based rule: the
 * first bulk IN and first bulk OUT of the WiFi interface are the data
 * pipes; in the app personality a second bulk pair carries lmac_msg
 * command frames.  The boot ROM has no dedicated message endpoints and
 * sends its command channel over the first bulk pair.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>

#include "aic8800var.h"

static void
aic8800u_ep_record(struct aic8800u_ep *ep, uint8_t addr, uint16_t maxpkt)
{
	ep->addr = addr;
	ep->maxpkt = maxpkt;
	ep->present = true;
}

int
aic8800u_parse_endpoints(struct aic8800u_softc *sc, struct usbd_interface *iface)
{
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	int j;

	memset(&sc->sc_ep, 0, sizeof(sc->sc_ep));

	id = usbd_get_interface_descriptor(iface);
	if (id == NULL)
		return EINVAL;

	for (j = 0; j < id->bNumEndpoints; j++) {
		ed = usbd_interface2endpoint_descriptor(iface, j);
		if (ed == NULL)
			continue;
		if (UE_GET_XFERTYPE(ed->bmAttributes) != UE_BULK)
			continue;

		if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN) {
			if (!sc->sc_ep.data_in.present)
				aic8800u_ep_record(&sc->sc_ep.data_in,
				    ed->bEndpointAddress,
				    UGETW(ed->wMaxPacketSize));
			else if (!sc->sc_ep.msg_in.present)
				aic8800u_ep_record(&sc->sc_ep.msg_in,
				    ed->bEndpointAddress,
				    UGETW(ed->wMaxPacketSize));
		} else {
			if (!sc->sc_ep.data_out.present)
				aic8800u_ep_record(&sc->sc_ep.data_out,
				    ed->bEndpointAddress,
				    UGETW(ed->wMaxPacketSize));
			else if (!sc->sc_ep.msg_out.present)
				aic8800u_ep_record(&sc->sc_ep.msg_out,
				    ed->bEndpointAddress,
				    UGETW(ed->wMaxPacketSize));
		}
	}

	if (!sc->sc_ep.data_in.present || !sc->sc_ep.data_out.present)
		return ENODEV;

	aprint_normal_dev(sc->sc_dev, "bulk in %#x maxpkt %d, out %#x maxpkt %d\n",
	    sc->sc_ep.data_in.addr, sc->sc_ep.data_in.maxpkt,
	    sc->sc_ep.data_out.addr, sc->sc_ep.data_out.maxpkt);
	if (sc->sc_ep.msg_in.present && sc->sc_ep.msg_out.present)
		aprint_normal_dev(sc->sc_dev,
		    "msg in %#x maxpkt %d, out %#x maxpkt %d\n",
		    sc->sc_ep.msg_in.addr, sc->sc_ep.msg_in.maxpkt,
		    sc->sc_ep.msg_out.addr, sc->sc_ep.msg_out.maxpkt);

	return 0;
}
