/*	$NetBSD: umodeswitch.c,v 1.6 2023/08/04 13:25:17 manu Exp $	*/

/*-
 * Copyright (c) 2009, 2017 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation.
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


#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: umodeswitch.c,v 1.6 2023/08/04 13:25:17 manu Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/tty.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usbdi_util.h>

#include "usbdevs.h"

/*
 * This device driver handles devices that have two personalities.
 * The first uses the 'usbdevif'
 * interface attribute so that a match will claim the entire USB device
 * for itself. This is used for when a device needs to be mode-switched
 * and ensures any other interfaces present cannot be claimed by other
 * drivers while the mode-switch is in progress.
 */
static int umodeswitch_match(device_t, cfdata_t, void *);
static void umodeswitch_attach(device_t, device_t, void *);
static int umodeswitch_detach(device_t, int);

CFATTACH_DECL2_NEW(umodeswitch, 0, umodeswitch_match,
    umodeswitch_attach, umodeswitch_detach, NULL, NULL, NULL);

/*
 * Lab: result of the last send_bulkmsg() transfer, for the mode-switch
 * wrappers that need to report why a device did (not) flip personality.
 * The CSW pair reports the status stage of a Bulk-Only transaction.
 */
static usbd_status umodeswitch_last_status;
static uint32_t umodeswitch_last_count;
static usbd_status umodeswitch_last_csw_status;
static uint32_t umodeswitch_last_csw_count;

/* Time to let the device settle before the interface is released. */
#define	UMODESWITCH_RELEASE_DELAY	100000	/* us */

/*
 * Lab: read the 13 byte Command Status Wrapper that ends a Bulk-Only
 * transaction.  usb_modeswitch does this whenever the message content starts
 * with the CBW signature (the "55534243" test in its sources) and only then
 * waits and releases the interface; several Realtek sticks acknowledge the
 * eject command but do not flip personality until the status stage of the
 * command has been read.  A failure here is expected when the device leaves
 * the bus mid-transaction, so it is reported through the
 * umodeswitch_last_csw_* variables instead of aborting the mode switch.
 */
static void
read_csw(struct usbd_interface *iface, usb_interface_descriptor_t *id)
{
	usb_endpoint_descriptor_t *eed;
	struct usbd_pipe *pipe;
	struct usbd_xfer *xfer;
	uint8_t buf[13];
	int err, j;

	umodeswitch_last_csw_status = USBD_NOT_STARTED;
	umodeswitch_last_csw_count = 0;

	for (j = 0; j < id->bNumEndpoints; j++) {
		eed = usbd_interface2endpoint_descriptor(iface, j);
		if (eed == NULL)
			continue;
		if (UE_GET_DIR(eed->bEndpointAddress) != UE_DIR_IN)
			continue;
		if ((eed->bmAttributes & UE_XFERTYPE) != UE_BULK)
			continue;
		break;
	}
	if (j == id->bNumEndpoints)
		return;

	err = usbd_open_pipe(iface, eed->bEndpointAddress,
	    USBD_EXCLUSIVE_USE, &pipe);
	if (err != 0) {
		umodeswitch_last_csw_status = (usbd_status)err;
		return;
	}

	if (usbd_create_xfer(pipe, sizeof(buf), 0, 0, &xfer) == 0) {
		memset(buf, 0, sizeof(buf));
		usbd_setup_xfer(xfer, NULL, buf, sizeof(buf),
		    USBD_SYNCHRONOUS | USBD_SHORT_XFER_OK,
		    USBD_DEFAULT_TIMEOUT, NULL);
		err = usbd_transfer(xfer);
		usbd_get_xfer_status(xfer, NULL, NULL,
		    &umodeswitch_last_csw_count,
		    &umodeswitch_last_csw_status);
		usbd_destroy_xfer(xfer);
	} else {
		umodeswitch_last_csw_status = USBD_NOMEM;
	}

	usbd_abort_pipe(pipe);
	usbd_close_pipe(pipe);
}

static int
send_bulkmsg(struct usbd_device *dev, void *cmd, size_t cmdlen)
{
	struct usbd_interface *iface;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	struct usbd_pipe *pipe;
	struct usbd_xfer *xfer;
	usb_config_descriptor_t *cdesc;
	int err, i, j, pass;

	/* Move the device into the configured state. */
	err = usbd_set_config_index(dev, 0, 0);
	if (err) {
		aprint_error("%s: failed to set config index\n", __func__);
		return UMATCH_NONE;
	}

	/*
	 * Prefer the mass-storage interface (like usb_modeswitch does), and
	 * fall back to interface 0 when no interface advertises it.
	 */
	cdesc = usbd_get_config_descriptor(dev);
	iface = NULL;
	id = NULL;
	ed = NULL;
	for (pass = 0; pass < 2 && ed == NULL; pass++) {
		unsigned nif = cdesc != NULL ? cdesc->bNumInterface : 1;
		for (i = 0; i < nif; i++) {
			if (usbd_device2interface_handle(dev, i, &iface) != 0)
				continue;
			id = usbd_get_interface_descriptor(iface);
			if (id == NULL)
				continue;
			if (pass == 0 && id->bInterfaceClass != 8)
				continue;
			ed = NULL;
			for (j = 0; j < id->bNumEndpoints; j++) {
				usb_endpoint_descriptor_t *eed;

				eed = usbd_interface2endpoint_descriptor(iface, j);
				if (eed == NULL)
					continue;
				if (UE_GET_DIR(eed->bEndpointAddress) != UE_DIR_OUT)
					continue;
				if ((eed->bmAttributes & UE_XFERTYPE) != UE_BULK)
					continue;
				ed = eed;
				break;
			}
			if (ed != NULL)
				break;
		}
		if (pass == 1 && ed == NULL)
			return UMATCH_NONE;
	}
	if (ed == NULL)
		return UMATCH_NONE;

	err = usbd_open_pipe(iface, ed->bEndpointAddress,
	    USBD_EXCLUSIVE_USE, &pipe);
	if (err != 0) {
		aprint_error("%s: failed to open bulk transfer pipe %d\n",
		    __func__, ed->bEndpointAddress);
		return UMATCH_NONE;
	}

	int error = usbd_create_xfer(pipe, cmdlen, 0, 0, &xfer);
	if (!error) {

		usbd_setup_xfer(xfer, NULL, cmd, cmdlen,
		    USBD_SYNCHRONOUS, USBD_DEFAULT_TIMEOUT, NULL);

		err = usbd_transfer(xfer);

		usbd_get_xfer_status(xfer, NULL, NULL, &umodeswitch_last_count,
		    &umodeswitch_last_status);
#if 0 /* XXXpooka: at least my huawei "fails" this always, but still detaches */
		if (err)
			aprint_error("%s: transfer failed\n", __func__);
#else
		err = 0;
#endif
		usbd_destroy_xfer(xfer);

		/*
		 * Lab: a message that starts with the CBW signature carries a
		 * SCSI command, so the device expects the host to fetch the
		 * status.  Only a completed Bulk-Only transaction makes some
		 * of these sticks act on the command (usb_modeswitch does the
		 * same for its "55534243" messages).
		 */
		if (((const uint8_t *)cmd)[0] == 0x55 &&
		    ((const uint8_t *)cmd)[1] == 0x53 &&
		    ((const uint8_t *)cmd)[2] == 0x42 &&
		    ((const uint8_t *)cmd)[3] == 0x43)
			read_csw(iface, id);

		/* Give the device a moment before the interface is released. */
		delay(UMODESWITCH_RELEASE_DELAY);
	} else {
		aprint_error("%s: failed to allocate xfer\n", __func__);
		err = USBD_NOMEM;
	}

	usbd_abort_pipe(pipe);
	usbd_close_pipe(pipe);

	return err == USBD_NORMAL_COMPLETION ? UMATCH_HIGHEST : UMATCH_NONE;
}

/* Byte 0..3: Command Block Wrapper (CBW) signature */
static void
set_cbw(unsigned char *cmd)
{
	cmd[0] = 0x55;
	cmd[1] = 0x53;
	cmd[2] = 0x42;
	cmd[3] = 0x43;
}

static int
u3g_bulk_scsi_eject(struct usbd_device *dev)
{
	unsigned char cmd[31];

	memset(cmd, 0, sizeof(cmd));
	/* Byte 0..3: Command Block Wrapper (CBW) signature */
	set_cbw(cmd);
	/* 4..7: CBW Tag, has to unique, but only a single transfer used. */
	cmd[4] = 0x01;
	/* 8..11: CBW Transfer Length, no data here */
	/* 12: CBW Flag: output, so 0 */
	/* 13: CBW Lun: 0 */
	/* 14: CBW Length */
	cmd[14] = 0x06;

	/* Rest is the SCSI payload */

	/* 0: SCSI START/STOP opcode */
	cmd[15] = 0x1b;
	/* 1..3 unused */
	/* 4 Load/Eject command */
	cmd[19] = 0x02;
	/* 5: unused */

	return send_bulkmsg(dev, cmd, sizeof(cmd));
}

static int
u3g_bulk_ata_eject(struct usbd_device *dev)
{
	unsigned char cmd[31];

	memset(cmd, 0, sizeof(cmd));
	/* Byte 0..3: Command Block Wrapper (CBW) signature */
	set_cbw(cmd);
	/* 4..7: CBW Tag, has to unique, but only a single transfer used. */
	cmd[4] = 0x01;
	/* 8..11: CBW Transfer Length, no data here */
	/* 12: CBW Flag: output, so 0 */
	/* 13: CBW Lun: 0 */
	/* 14: CBW Length */
	cmd[14] = 0x06;

	/* Rest is the SCSI payload */

	/* 0: ATA pass-through */
	cmd[15] = 0x85;
	/* 1..3 unused */
	/* 4 XXX What is this command? */
	cmd[19] = 0x24;
	/* 5: unused */

	return send_bulkmsg(dev, cmd, sizeof(cmd));
}

static int
u3g_huawei_reinit(struct usbd_device *dev)
{
	/*
	 * The Huawei device presents itself as a umass device with Windows
	 * drivers on it. After installation of the driver, it reinits into a
	 * 3G serial device.
	 */
	usb_device_request_t req;
	usb_config_descriptor_t *cdesc;

	/* Get the config descriptor */
	cdesc = usbd_get_config_descriptor(dev);
	if (cdesc == NULL) {
		usb_device_descriptor_t dd;

		if (usbd_get_device_desc(dev, &dd) != 0)
			return UMATCH_NONE;

		if (dd.bNumConfigurations != 1)
			return UMATCH_NONE;

		if (usbd_set_config_index(dev, 0, 1) != 0)
			return UMATCH_NONE;

		cdesc = usbd_get_config_descriptor(dev);

		if (cdesc == NULL)
			return UMATCH_NONE;
	}

	/*
	 * One iface means umass mode, more than 1 (4 usually) means 3G mode.
	 *
	 * XXX: We should check the first interface's device class just to be
	 * sure. If it's a mass storage device, then we can be fairly certain
	 * it needs a mode-switch.
	 */
	if (cdesc->bNumInterface > 1)
		return UMATCH_NONE;

	req.bmRequestType = UT_WRITE_DEVICE;
	req.bRequest = UR_SET_FEATURE;
	USETW(req.wValue, UF_DEVICE_REMOTE_WAKEUP);
	USETW(req.wIndex, UHF_PORT_SUSPEND);
	USETW(req.wLength, 0);

	(void) usbd_do_request(dev, &req, 0);

	return UMATCH_HIGHEST; /* Prevent umass from attaching */
}

static int
u3g_huawei_k3765_reinit(struct usbd_device *dev)
{
	unsigned char cmd[31];

	/* magic string adapted from some webpage */
	memset(cmd, 0, sizeof(cmd));
	/* Byte 0..3: Command Block Wrapper (CBW) signature */
	set_cbw(cmd);

	cmd[15]= 0x11;
	cmd[16]= 0x06;

	return send_bulkmsg(dev, cmd, sizeof(cmd));
}
static int
u3g_huawei_e171_reinit(struct usbd_device *dev)
{
	unsigned char cmd[31];

	/* magic string adapted from some webpage */
	memset(cmd, 0, sizeof(cmd));
	/* Byte 0..3: Command Block Wrapper (CBW) signature */
	set_cbw(cmd);

	cmd[15]= 0x11;
	cmd[16]= 0x06;
	cmd[17]= 0x20;
	cmd[20]= 0x01;

	return send_bulkmsg(dev, cmd, sizeof(cmd));
}

static int
u3g_huawei_e353_reinit(struct usbd_device *dev)
{
	unsigned char cmd[31];

	/* magic string adapted from some webpage */
	memset(cmd, 0, sizeof(cmd));
	/* Byte 0..3: Command Block Wrapper (CBW) signature */
	set_cbw(cmd);

	cmd[4] = 0x7f;
	cmd[9] = 0x02;
	cmd[12] = 0x80;
	cmd[14] = 0x0a;
	cmd[15] = 0x11;
	cmd[16] = 0x06;
	cmd[17] = 0x20;
	cmd[23] = 0x01;

	return send_bulkmsg(dev, cmd, sizeof(cmd));
}

static int
u3g_sierra_reinit(struct usbd_device *dev)
{
	/* Some Sierra devices presents themselves as a umass device with
	 * Windows drivers on it. After installation of the driver, it
	 * reinits into a * 3G serial device.
	 */
	usb_device_request_t req;

	req.bmRequestType = UT_VENDOR;
	req.bRequest = UR_SET_INTERFACE;
	USETW(req.wValue, UF_DEVICE_REMOTE_WAKEUP);
	USETW(req.wIndex, UHF_PORT_CONNECTION);
	USETW(req.wLength, 0);

	(void) usbd_do_request(dev, &req, 0);

	return UMATCH_HIGHEST; /* Match to prevent umass from attaching */
}

static int
u3g_4gsystems_reinit(struct usbd_device *dev)
{
	/* magic string adapted from usb_modeswitch database */
	unsigned char cmd[31];

	memset(cmd, 0, sizeof(cmd));
	/* Byte 0..3: Command Block Wrapper (CBW) signature */
	set_cbw(cmd);

	cmd[4] = 0x12;
	cmd[5] = 0x34;
	cmd[6] = 0x56;
	cmd[7] = 0x78;
	cmd[8] = 0x80;
	cmd[12] = 0x80;
	cmd[14] = 0x06;
	cmd[15] = 0x06;
	cmd[16] = 0xf5;
	cmd[17] = 0x04;
	cmd[18] = 0x02;
	cmd[19] = 0x52;
	cmd[20] = 0x70;

	return send_bulkmsg(dev, cmd, sizeof(cmd));
}

/*
 * Realtek USB wireless adapters (RTL8811CU/RTL8821CU family) come up as a
 * fake CD-ROM drive ("Realtek Driver Storage") holding a Windows driver.
 * A SCSI START/STOP UNIT with LoEj set makes them re-enumerate as an
 * 802.11ac NIC.  The message is the one usb_modeswitch sends for 0bda:1a2b;
 * only the CBW tag differs from u3g_bulk_scsi_eject().
 */
static int
realtek_rtl8821cu_reinit(struct usbd_device *dev)
{
	unsigned char cmd[31];
	usb_config_descriptor_t *cdesc;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	struct usbd_interface *iface;
	int attempt, i, j, rv = UMATCH_HIGHEST;

	memset(cmd, 0, sizeof(cmd));
	/* Byte 0..3: Command Block Wrapper (CBW) signature */
	set_cbw(cmd);
	/* 4..7: CBW Tag, has to be unique, but only a single transfer is used. */
	cmd[4] = 0x12;
	cmd[5] = 0x34;
	cmd[6] = 0x56;
	cmd[7] = 0x78;
	/* 8..11: CBW Transfer Length, no data here */
	/* 12: CBW Flag: output, so 0 */
	/* 13: CBW Lun: 0 */
	/* 14: CBW Length */
	cmd[14] = 0x06;

	/* Rest is the SCSI payload */

	/* 0: SCSI START/STOP opcode */
	cmd[15] = 0x1b;
	/* 1..3 unused */
	/* 4: LoEj */
	cmd[19] = 0x02;
	/* 5: unused */

	/*
	 * Lab: dump the fake-CD personality's layout so the endpoint the
	 * message has to go to can be verified against the Linux record.
	 */
	(void)usbd_set_config_index(dev, 0, 0);
	cdesc = usbd_get_config_descriptor(dev);
	if (cdesc != NULL) {
		for (i = 0; i < cdesc->bNumInterface; i++) {
			if (usbd_device2interface_handle(dev, i, &iface) != 0)
				continue;
			id = usbd_get_interface_descriptor(iface);
			if (id == NULL)
				continue;
			aprint_normal("umodeswitch: if %d class %#x/%#x/%#x,"
			    " %d endpoints\n", id->bInterfaceNumber,
			    id->bInterfaceClass, id->bInterfaceSubClass,
			    id->bInterfaceProtocol, id->bNumEndpoints);
			for (j = 0; j < id->bNumEndpoints; j++) {
				ed = usbd_interface2endpoint_descriptor(iface, j);
				if (ed != NULL)
					aprint_normal("umodeswitch:   ep %#x attr"
					    " %#x maxpkt %d\n",
					    ed->bEndpointAddress, ed->bmAttributes,
					    UGETW(ed->wMaxPacketSize));
			}
		}
	}

	/*
	 * Lab: send the eject up to three times.  The transfer result and
	 * byte count are reported (usb_modeswitch only warns on failure, so
	 * the in-kernel path has to be equally tolerant here).
	 */
	for (attempt = 0; attempt < 3; attempt++) {
		rv = send_bulkmsg(dev, cmd, sizeof(cmd));
		aprint_normal("umodeswitch: Realtek eject attempt %d: status %d"
		    " (%s), count %u, csw %d (%s), csw count %u\n",
		    attempt + 1,
		    (int)umodeswitch_last_status,
		    usbd_errstr(umodeswitch_last_status),
		    (unsigned)umodeswitch_last_count,
		    (int)umodeswitch_last_csw_status,
		    usbd_errstr(umodeswitch_last_csw_status),
		    (unsigned)umodeswitch_last_csw_count);
		if (umodeswitch_last_status == USBD_NORMAL_COMPLETION)
			break;
		delay(100000);
	}

	return rv;
}

/*
 * First personality:
 *
 * Claim the entire device if a mode-switch is required.
 */

static int
umodeswitch_match(device_t parent, cfdata_t match, void *aux)
{
	struct usb_attach_arg *uaa = aux;

	/*
	 * Huawei changes product when it is configured as a modem.
	 */
	switch (uaa->uaa_vendor) {
	case USB_VENDOR_HUAWEI:
		if (uaa->uaa_product == USB_PRODUCT_HUAWEI_K3765)
			return UMATCH_NONE;

		switch (uaa->uaa_product) {
		case USB_PRODUCT_HUAWEI_E1750INIT:
		case USB_PRODUCT_HUAWEI_K3765INIT:
			return u3g_huawei_k3765_reinit(uaa->uaa_device);
			break;
		case USB_PRODUCT_HUAWEI_E171INIT:
			return u3g_huawei_e171_reinit(uaa->uaa_device);
			break;
		case USB_PRODUCT_HUAWEI_E353INIT:
			return u3g_huawei_e353_reinit(uaa->uaa_device);
			break;
		default:
			return u3g_huawei_reinit(uaa->uaa_device);
			break;
		}
		break;

	case USB_VENDOR_NOVATEL2:
		switch (uaa->uaa_product){
		case USB_PRODUCT_NOVATEL2_MC950D_DRIVER:
		case USB_PRODUCT_NOVATEL2_U760_DRIVER:
			return u3g_bulk_scsi_eject(uaa->uaa_device);
			break;
		default:
			break;
		}
		break;

	case USB_VENDOR_LG:
		if (uaa->uaa_product == USB_PRODUCT_LG_NTT_DOCOMO_L02C_STORAGE)
			return u3g_bulk_scsi_eject(uaa->uaa_device);
		break;

	case USB_VENDOR_RALINK:
		switch (uaa->uaa_product){
		case USB_PRODUCT_RALINK_RT73:
			return u3g_bulk_scsi_eject(uaa->uaa_device);
			break;
		}
		break;

	case USB_VENDOR_SIERRA:
		if (uaa->uaa_product == USB_PRODUCT_SIERRA_INSTALLER)
			return u3g_sierra_reinit(uaa->uaa_device);
		break;

	case USB_VENDOR_ZTE:
		switch (uaa->uaa_product){
		case USB_PRODUCT_ZTE_INSTALLER:
		case USB_PRODUCT_ZTE_MF820D_INSTALLER:
			(void)u3g_bulk_ata_eject(uaa->uaa_device);
			(void)u3g_bulk_scsi_eject(uaa->uaa_device);
			return UMATCH_HIGHEST;
		default:
			break;
		}
		break;

	case USB_VENDOR_LONGCHEER:
		if (uaa->uaa_product == USB_PRODUCT_LONGCHEER_XSSTICK_P14_INSTALLER)
			return u3g_4gsystems_reinit(uaa->uaa_device);
		break;

	case USB_VENDOR_DLINK:
		switch (uaa->uaa_product) {
		case USB_PRODUCT_DLINK_DWM157E_CD:
		case USB_PRODUCT_DLINK_DWM157_CD:
		case USB_PRODUCT_DLINK_DWM222_CD:
			(void)u3g_bulk_ata_eject(uaa->uaa_device);
			(void)u3g_bulk_scsi_eject(uaa->uaa_device);
			return UMATCH_HIGHEST;
		default:
			break;
		}

	case USB_VENDOR_REALTEK:
		if (uaa->uaa_product == USB_PRODUCT_REALTEK_RTL8821CU_CD)
			return realtek_rtl8821cu_reinit(uaa->uaa_device);
		break;

	default:
		break;
	}

	return UMATCH_NONE;
}

static void
umodeswitch_attach(device_t parent, device_t self, void *aux)
{
	struct usb_attach_arg *uaa = aux;

	aprint_naive("\n");
	aprint_normal(": Switching off umass mode\n");

	if (uaa->uaa_vendor == USB_VENDOR_NOVATEL2) {
		switch (uaa->uaa_product) {
	    	case USB_PRODUCT_NOVATEL2_MC950D_DRIVER:
	    	case USB_PRODUCT_NOVATEL2_U760_DRIVER:
			/* About to disappear... */
			return;
			break;
		default:
			break;
		}
	}

	/* Move the device into the configured state. */
	(void) usbd_set_config_index(uaa->uaa_device, 0, 1);
}

static int
umodeswitch_detach(device_t self, int flags)
{

	return 0;
}
