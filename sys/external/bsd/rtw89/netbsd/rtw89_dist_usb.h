#ifndef _RTW89_DIST_USB_FWD_H_
#define _RTW89_DIST_USB_FWD_H_
/* "usb.h" collides with NetBSD dev/usb/usb.h (quoted) and the
 * config-generated compile-dir usb.h (angled); reach the dist header
 * by its path relative to this directory. */
#include <linux/usb.h>		/* usb_interface/usb_device_id/usb_anchor */
#include "../dist/usb.h"
#endif
