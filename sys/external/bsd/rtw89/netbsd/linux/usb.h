#ifndef _RTW89_LINUX_USB_H_
#define _RTW89_LINUX_USB_H_
#include "rtw89_compat.h"

/*
 * The only Linux USB shapes the imported rtw8851bu.c needs: the device id
 * table and driver struct it declares at file scope.  The registration is
 * inert -- NetBSD binding happens in dev/usb/if_rtw89.c via usbdevs.
 */
typedef unsigned long		kernel_ulong_t;

#define	USB_DEVICE_ID_MATCH_INT_INFO	0x8000

struct usb_device_id {
	u16			match_flags;
	u16			idVendor;
	u16			idProduct;
	u16			bcdDevice_lo;
	u8			bDeviceClass;
	u8			bDeviceSubClass;
	u8			bDeviceProtocol;
	u8			bInterfaceClass;
	u8			bInterfaceSubClass;
	u8			bInterfaceProtocol;
	kernel_ulong_t		driver_info;
};

#define	USB_DEVICE_AND_INTERFACE_INFO(vend, prod, cl, sc, pr)		\
	.match_flags = USB_DEVICE_ID_MATCH_INT_INFO,			\
	.idVendor = (vend),						\
	.idProduct = (prod),						\
	.bInterfaceClass = (cl),					\
	.bInterfaceSubClass = (sc),					\
	.bInterfaceProtocol = (pr)

struct usb_interface;
struct usb_anchor {
	bool			dummy;
};

struct usb_driver {
	const char		*name;
	const struct usb_device_id *id_table;
	int			(*probe)(struct usb_interface *,
	    const struct usb_device_id *);
	void			(*disconnect)(struct usb_interface *);
};

/* USB_SPEED_* come from NetBSD's dev/usb/usb.h (same names, same values). */

#define	module_usb_driver(drv)						\
	static void *rtw89_drv_ref __used = (void *)&(drv)

#endif
