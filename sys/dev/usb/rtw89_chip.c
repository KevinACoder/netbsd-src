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
 * Chip glue between if_rtw89.c, the compat shadow mac80211 and the
 * imported rtw89 core.  This file must not include sys/net80211: the
 * shadow mac80211 headers collide with it (same type names).  Frames
 * cross the boundary through the rx_cb/tx contracts in rtw89_chipvar.h.
 *
 * The attach sequence mirrors the FreeBSD lane: alloc_ieee80211_hw
 * (which recognises the firmware format), install the usbdi transport
 * behind rtwdev->priv, core_init, chip_info_setup (MAC power-on, efuse,
 * PHY caps) and core_register.  core_start -- the firmware download and
 * MAC/RF init -- runs from the net80211 init path.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/mbuf.h>
#include <sys/device.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include "rtw89_compat.h"

#include <dev/usb/usbdivar.h>

#include "rtw89_compat.h"
#include "rtw89_glue.h"

#include "core.h"
#include "chan.h"
#include "mac.h"
#include "reg.h"
#include "txrx.h"
#include "rtw8851b.h"
#include "rtw89_dist_usb.h"

#include "rtw89var.h"
#include "rtw89_chipvar.h"
#include "rtw89_usbvar.h"

/* ------------------------------------------------------------------ */
/* RX delivery: compat ieee80211_rx_napi() hands frames over here      */
/* ------------------------------------------------------------------ */

/*
 * The core hands frames WITH the 4-byte FCS (it sets
 * IEEE80211_HW_RX_INCLUDES_FCS); the net80211 front end strips it.
 * The dongle is unique on this board, so the back pointer is a
 * single instance.
 */
static struct rtw89_chip *rtw89_chip_instance;

static void
rtw89_chip_rx_hook(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	struct rtw89_dev *rtwdev = hw->priv;
	struct rtw89_chip *chip = rtw89_chip_instance;
	struct ieee80211_rx_status *rxs = IEEE80211_SKB_RXCB(skb);
	int rssi;

	(void)rtwdev;
	if (chip == NULL || chip->rx_cb == NULL) {
		dev_kfree_skb_any(skb);
		return;
	}

	rssi = rxs != NULL ? rxs->signal : 0;
	chip->rx_cb(chip->rx_arg, skb->data, skb->len, rssi);
	dev_kfree_skb_any(skb);
}

/*
 * The shadow mac80211 has no station-add callback: the per-station chip
 * state (struct rtw89_sta behind drv_priv) must be bound before any
 * iterator runs (rtw88 fix d5062cf55632).
 */
void
rtw89_sta_init(struct ieee80211_sta *sta, struct ieee80211_vif *vif,
    struct rtw_dev *rtwdev)
{
	(void)sta;
	(void)vif;
	(void)rtwdev;
}

/* ------------------------------------------------------------------ */
/* attach                                                              */
/* ------------------------------------------------------------------ */

struct rtw89_chip *
rtw89_chip_attach(device_t dev, struct usbd_device *udev,
    struct usbd_interface *iface)
{
	struct rtw89_chip *chip;
	struct rtw89_usb_softc *usb;
	struct rtw89_dev *rtwdev;
	int error;

	chip = kmem_zalloc(sizeof(*chip), KM_SLEEP);

	rtw89_workqueue_ready();

	memset(&chip->hostdev, 0, sizeof(chip->hostdev));
	snprintf(chip->hostdev.name, sizeof(chip->hostdev.name), "%s",
	    "rtw89usb");

	/*
	 * One allocation: sizeof(struct rtw89_dev) plus the transport
	 * state, whose address is rtwdev->priv (the flexible array at the
	 * end of the core struct).  This loads the firmware image header
	 * for format recognition (rtw8851b_fw-1.bin first).
	 */
	rtwdev = rtw89_alloc_ieee80211_hw(&chip->hostdev,
	    sizeof(struct rtw89_usb_softc), &rtw8851b_chip_info, NULL);
	if (rtwdev == NULL) {
		aprint_error_dev(dev,
		    "failed to allocate the chip/RF data\n");
		kmem_free(chip, sizeof(*chip));
		return NULL;
	}
	chip->rtwdev = rtwdev;

	if (rtwdev->fw.req.firmware == NULL) {
		/*
		 * Early format recognition failed: firmware(9) is not
		 * reachable yet (no root).  Bail so the frontend retry loop
		 * tries again after mountroot -- downloading with a NULL
		 * image cannot work.
		 */
		aprint_debug_dev(dev, "attach: firmware not reachable yet\n");
		rtw89_free_ieee80211_hw(rtwdev);
		chip->rtwdev = NULL;
		kmem_free(chip, sizeof(*chip));
		return NULL;
	}

	usb = (struct rtw89_usb_softc *)rtwdev->priv;
	memset(usb, 0, sizeof(*usb));
	usb->rtwdev = rtwdev;
	usb->udev = udev;
	usb->iface = iface;

	/* the hci vtable must be installed on the rtwdev we keep */
	rtwdev->hci.ops = rtw89_usb_get_ops();
	rtwdev->hci.type = RTW89_HCI_TYPE_USB;
	rtwdev->hci.tx_rpt_enabled = true;
	rtwdev->hci.dle_type = udev->ud_speed == USB_SPEED_SUPER ?
	    RTW89_HCI_DLE_TYPE_USB3 : RTW89_HCI_DLE_TYPE_USB2;

	aprint_normal_dev(dev, "attach: transport\n");
	error = rtw89_usb_attach(usb, dev, udev, iface);
	if (error != 0)
		goto out_fail;

	/*
	 * core_init brings the core up; chip_info_setup powers the MAC,
	 * reads the efuse and the PHY capability table -- the GET_FEATURE
	 * register handshake in there polls with an udelay-only budget
	 * (>= 10 ms) per the Linux lab_c2h_wall A/B evidence, which the
	 * compat register helpers implement.
	 */
	aprint_normal_dev(dev, "attach: core_init\n");
	error = rtw89_core_init(rtwdev);
	if (error != 0) {
		aprint_error_dev(dev, "core init failed: %d\n", error);
		goto out_fail;
	}
	aprint_normal_dev(dev, "attach: core_init done\n");

	aprint_normal_dev(dev, "attach: chip_info_setup\n");
	error = rtw89_chip_info_setup(rtwdev);
	if (error != 0) {
		aprint_error_dev(dev, "chip info setup failed: %d\n", error);
		goto out_fail;
	}

	aprint_normal_dev(dev, "attach: core_register\n");
	error = rtw89_core_register(rtwdev);
	if (error != 0) {
		aprint_error_dev(dev, "core register failed: %d\n", error);
		goto out_fail;
	}

	/* arm the bulk IN pipe: C2H replies arrive from here on */
	error = rtwdev->hci.ops->start(rtwdev);
	if (error != 0)
		goto out_fail;

	/* identity for the net80211 front end */
	ether_addr_copy(chip->mac_addr, rtwdev->efuse.addr);
	chip->efuse_valid = is_valid_ether_addr(chip->mac_addr);
	chip->fw_format = rtwdev->fw.fw_format;

	/* RX delivery: compat ieee80211_rx_napi() lands on our handler */
	rtw89_chip_instance = chip;
	rtw89_rx_deliver = rtw89_chip_rx_hook;

	aprint_normal_dev(dev,
	    "fw format %d, MAC %02x:%02x:%02x:%02x:%02x:%02x%s\n",
	    chip->fw_format,
	    chip->mac_addr[0], chip->mac_addr[1], chip->mac_addr[2],
	    chip->mac_addr[3], chip->mac_addr[4], chip->mac_addr[5],
	    chip->efuse_valid ? "" : " (invalid)");

	return chip;

out_fail:
	rtw89_usb_detach(usb);
	rtw89_free_ieee80211_hw(rtwdev);
	chip->rtwdev = NULL;
	kmem_free(chip, sizeof(*chip));
	return NULL;
}

void
rtw89_chip_detach(struct rtw89_chip *chip)
{
	struct rtw89_dev *rtwdev = chip->rtwdev;

	rtw89_rx_deliver = NULL;
	rtw89_chip_instance = NULL;

	if (rtwdev == NULL)
		return;

	if (rtwdev->hci.ops != NULL)
		rtwdev->hci.ops->stop(rtwdev);
	rtw89_core_unregister(rtwdev);
	rtw89_core_deinit(rtwdev);

	rtw89_usb_detach((struct rtw89_usb_softc *)rtwdev->priv);
	rtw89_free_ieee80211_hw(rtwdev);
	chip->rtwdev = NULL;
	kmem_free(chip, sizeof(*chip));
}

/* ------------------------------------------------------------------ */
/* start/stop (firmware download lives in core_start)                  */
/* ------------------------------------------------------------------ */

int
rtw89_chip_start(struct rtw89_chip *chip)
{

	return rtw89_core_start(chip->rtwdev);
}

void
rtw89_chip_stop(struct rtw89_chip *chip)
{

	rtw89_core_stop(chip->rtwdev);
}

bool
rtw89_chip_ready(const struct rtw89_chip *chip)
{

	return chip->rtwdev != NULL &&
	    test_bit(RTW89_FLAG_POWERON, chip->rtwdev->flags);
}

const uint8_t *
rtw89_chip_mac_addr(const struct rtw89_chip *chip,
    struct rtw89_hw_info *info)
{

	if (info != NULL) {
		memcpy(info->mac_addr, chip->mac_addr, 6);
		info->efuse_valid = chip->efuse_valid;
		info->fw_format = chip->fw_format;
	}
	return chip->mac_addr;
}

/* ------------------------------------------------------------------ */
/* channel                                                             */
/* ------------------------------------------------------------------ */

/*
 * The same two calls rtw89_ops_config() makes for
 * IEEE80211_CONF_CHANGE_CHANNEL, minus the powersave plumbing (this
 * port keeps the firmware awake).  The channel number maps into the
 * shadow wiphy's band tables, which the core filled at register time.
 */
int
rtw89_chip_set_channel(struct rtw89_chip *chip, unsigned int chan)
{
	struct rtw89_dev *rtwdev = chip->rtwdev;
	struct wiphy *wiphy = rtwdev->hw->wiphy;
	struct ieee80211_channel *c = NULL;
	unsigned int band, i;

	for (band = 0; band < NUM_NL80211_BANDS && c == NULL; band++) {
		struct ieee80211_supported_band *sband = wiphy->bands[band];

		if (sband == NULL)
			continue;
		for (i = 0; i < (unsigned int)sband->n_channels; i++) {
			if (sband->channels[i].hw_value == chan) {
				c = &sband->channels[i];
				break;
			}
		}
	}
	if (c == NULL) {
		rtw89_err(rtwdev, "%s: no shadow channel %u\n", __func__,
		    chan);
		return EINVAL;
	}

	rtwdev->hw->conf.chandef.chan = c;
	rtwdev->hw->conf.chandef.width = NL80211_CHAN_WIDTH_20;
	rtwdev->hw->conf.chandef.center_freq1 = c->center_freq;
	rtwdev->hw->conf.chandef.center_freq2 = 0;
	rtwdev->hw->conf.chandef.punctured = 0;

	rtw89_config_entity_chandef(rtwdev, RTW89_CHANCTX_0,
	    &rtwdev->hw->conf.chandef);
	return rtw89_set_channel(rtwdev);
}

/* ------------------------------------------------------------------ */
/* TX (mbuf -> core; full station wiring lands with M2/M3)             */
/* ------------------------------------------------------------------ */

int
rtw89_chip_tx(struct rtw89_chip *chip, struct mbuf *m, bool is_mgmt)
{
	struct rtw89_dev *rtwdev = chip->rtwdev;

	/*
	 * core_tx_write() needs bound rtwvif/rtwsta state (drv_priv),
	 * which only exists once the add-interface/add-station paths of
	 * the shadow mac80211 are wired (M2 soft scan, M3 association).
	 * Until then there is no legitimate transmitter.
	 */
	(void)rtwdev;
	(void)is_mgmt;
	m_freem(m);
	return 0;
}

void
rtw89_chip_set_callbacks(struct rtw89_chip *chip, void *arg,
    rtw89_rx_cb_t cb)
{

	chip->rx_arg = arg;
	chip->rx_cb = cb;
}
