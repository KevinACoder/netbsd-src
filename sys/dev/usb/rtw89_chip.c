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

/*
 * The shadow mac80211 drv_priv[] areas back the imported rtw89 structures
 * (vif_to_rtwvif()/sta_to_rtwsta() cast them, including one links_inst[]
 * entry of the MLO-reworked v7.0 shapes).  Keep the sizes honest at
 * compile time -- only this file sees both the dist and the shadow types.
 */
CTASSERT(sizeof(struct rtw89_vif) + sizeof(struct rtw89_vif_link) <=
    sizeof(((struct ieee80211_vif *)0)->drv_priv));
CTASSERT(sizeof(struct rtw89_sta) + sizeof(struct rtw89_sta_link) <=
    sizeof(((struct ieee80211_sta *)0)->drv_priv));
CTASSERT(sizeof(struct rtw89_txq) <=
    sizeof(((struct ieee80211_txq *)0)->drv_priv));
CTASSERT(sizeof(struct rtw89_chanctx_cfg) <=
    sizeof(((struct ieee80211_chanctx_ctx *)0)->drv_priv));

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

	/*
	 * Do NOT touch the softc before rtw89_usb_attach(): it memsets the
	 * whole struct (which used to wipe the rtwdev backpointer written
	 * here, leaving every rx_work callback to bail on rtwdev == NULL --
	 * the reason no C2H packet ever reached the core).
	 */
	usb = (struct rtw89_usb_softc *)rtwdev->priv;

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

	usb->rtwdev = rtwdev;
	usb->udev = udev;
	usb->iface = iface;

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
	chip->fw_ready = true;

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

/*
 * The shadow mac80211 never dispatches ops->add_interface on its own;
 * the call below is what mac80211 would do after ops->start when the
 * interface comes up.  It builds the rtwvif in the vif drv_priv[]
 * (mac_id, port, link instance) and sends the class 8/6/5 H2Cs -- with
 * zero class-9 (FW_OFLD) traffic on this path.  Without the binding,
 * every rtw89_core_tx_write() fails with -ENOLINK and the channel and
 * scan paths see no vif.
 */
static int
rtw89_vif_add(struct rtw89_chip *chip)
{
	struct rtw89_dev *rtwdev = chip->rtwdev;
	struct ieee80211_hw *hw = rtwdev->hw;
	struct ieee80211_vif *vif;
	int error;

	vif = rtw89_mac80211_vif(hw);
	rtw89_mac80211_set_mac(hw, chip->mac_addr);

	error = rtwdev->ops->add_interface(hw, vif);
	if (error != 0) {
		rtw89_err(rtwdev, "add_interface failed: %d\n", error);
		return error;
	}

	chip->vif_added = true;
	return 0;
}

static void
rtw89_vif_remove(struct rtw89_chip *chip)
{
	struct rtw89_dev *rtwdev = chip->rtwdev;
	struct ieee80211_hw *hw = rtwdev->hw;

	if (!chip->vif_added)
		return;
	chip->vif_added = false;
	rtwdev->ops->remove_interface(hw, rtw89_mac80211_vif(hw));
}

int
rtw89_chip_start(struct rtw89_chip *chip)
{
	struct rtw89_dev *rtwdev = chip->rtwdev;
	int error;

	if (chip->fw_ready) {
		/*
		 * Enter with the chip powered off, as the Linux ops->start()
		 * contract requires: chip_info_setup() ends with mac_pwr_off
		 * (dist core.c) and core_stop() powers off on the down path.
		 * The defensive pwr_off covers boots where a previous
		 * core_start left the WCPU running -- a firmware download
		 * pushed onto the live firmware stalls.
		 */
		rtw89_mac_pwr_off(rtwdev);
		chip->fw_ready = false;
	}

	/*
	 * The firmware restart above reinitialises the device's bulk
	 * endpoints at DATA0, while the host-side pipes still carry the
	 * previous session's data-toggle state (the attach download ran on
	 * these very pipes).  Close and reopen every pipe -- xhci(4)
	 * allocates a fresh transfer ring and endpoint context on
	 * usbd_open_pipe -- so the download below starts from matched
	 * toggles.  With stale pipes the re-download crawled at
	 * 0.2-2 s/frame and the RF table walk behind it ran on timeouts
	 * (M2 evidence, rounds D/E).
	 */
	error = rtw89_usb_pipes_reset((struct rtw89_usb_softc *)rtwdev->priv);
	if (error != 0)
		return error;

	error = rtw89_core_start(rtwdev);
	if (error != 0)
		return error;

	/* mac80211's ifup order: ops->start(), then add_interface */
	if (!chip->vif_added) {
		error = rtw89_vif_add(chip);
		if (error != 0)
			return error;
	}
	return 0;
}

void
rtw89_chip_stop(struct rtw89_chip *chip)
{

	/* mac80211's ifdown order: remove_interface, then ops->stop */
	rtw89_vif_remove(chip);
	rtw89_core_stop(chip->rtwdev);
}

/* ------------------------------------------------------------------ */
/* soft scan enter/leave                                               */
/* ------------------------------------------------------------------ */

/*
 * The Linux mac80211 soft-scan brackets: scanning=true enables the
 * beacon-IE-based RX frequency correction on the chip, LPS/EDCCA are
 * parked, and the addr cam is refreshed.  mac_addr must be non-NULL
 * (upstream ether_copy()s it): use the vif address, as mac80211 does.
 *
 * mac80211 would normally widen the MAC receive filter for the scan
 * window via configure_filter(FIF_BCN_PRBRESP_PROMISC) -- dead code
 * behind this port's net80211 bridge.  Do it here instead, mirroring
 * the exact bit set upstream clears (hw scan start does the same):
 * DEFAULT_AX_RX_FLTR keeps A_A1_MATCH|A_BC|A_BCN_CHK_EN set, which
 * makes the MAC engine drop every beacon from an unknown BSSID before
 * it reaches USB (C2H rides a separate path, which is why the radio
 * looks alive while nothing is heard).
 */
static unsigned int rtw89_chip_rxfltr_dbg;

static void
rtw89_chip_scan_rx_fltr(struct rtw89_dev *rtwdev, bool widen)
{
	const u32 scan_bits = B_AX_A_A1_MATCH | B_AX_A_BC | B_AX_A_BCN_CHK_EN;
	u32 reg, rx_fltr = rtwdev->hal.rx_fltr;

	if (widen)
		rx_fltr &= ~scan_bits;
	if (rtw89_chip_rxfltr_dbg < 4) {
		reg = rtw89_mac_reg_by_idx(rtwdev,
		    rtwdev->chip->mac_def->rx_fltr, RTW89_MAC_0);
		printf("rtw89usb: scan rx_fltr %s ce20 %08x -> %08x\n",
		    widen ? "widen" : "restore", rtw89_read32(rtwdev, reg),
		    rx_fltr);
		rtw89_chip_rxfltr_dbg++;
	}
	rtw89_mac_set_rx_fltr(rtwdev, RTW89_MAC_0, rx_fltr);
}

int
rtw89_chip_scan(struct rtw89_chip *chip, bool on)
{
	struct rtw89_dev *rtwdev = chip->rtwdev;
	struct ieee80211_hw *hw = rtwdev->hw;
	struct ieee80211_vif *vif;

	if (!chip->vif_added) {
		rtw89_err(rtwdev, "scan: vif not bound\n");
		return ENXIO;
	}

	vif = rtw89_mac80211_vif(hw);
	if (on) {
		rtwdev->ops->sw_scan_start(hw, vif, vif->addr);
		rtw89_chip_scan_rx_fltr(rtwdev, true);
	} else {
		rtw89_chip_scan_rx_fltr(rtwdev, false);
		rtwdev->ops->sw_scan_complete(hw, vif);
	}
	return 0;
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
static unsigned int rtw89_chip_ch_dbg;

int
rtw89_chip_set_channel(struct rtw89_chip *chip, unsigned int chan)
{
	struct rtw89_dev *rtwdev = chip->rtwdev;
	struct wiphy *wiphy = rtwdev->hw->wiphy;
	struct ieee80211_channel *c = NULL;
	unsigned int band, i;
	int ret;

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
	ret = rtw89_set_channel(rtwdev);

	/*
	 * Diagnostics: RF18 (CFGCH) low byte must track `chan` after each
	 * tune.  INV_RF_DATA or a stuck value means the RF bus or the
	 * entity chandef failed silently.
	 */
	if (rtw89_chip_ch_dbg < 32) {
		u32 rf18 = rtw89_read_rf(rtwdev, RF_PATH_A, 0x18, RFREG_MASK);

		printf("rtw89usb: set_channel %u ret=%d rf18=%08x ch=%u\n",
		    chan, ret, rf18, rf18 & 0xff);
		rtw89_chip_ch_dbg++;
	}
	return ret;
}

/* ------------------------------------------------------------------ */
/* TX (mbuf -> skb -> rtw89_core_tx_write)                             */
/* ------------------------------------------------------------------ */

/*
 * The net80211 front end hands over complete 802.11 frames (no FCS,
 * same convention mac80211 uses).  During soft scan only management
 * frames arrive here (probe requests); data frames stay gated until
 * the station binding lands (M4).  chip_tx() always consumes the mbuf.
 */
static unsigned int rtw89_chip_tx_dbg;

int
rtw89_chip_tx(struct rtw89_chip *chip, struct mbuf *m, bool is_mgmt)
{
	struct rtw89_dev *rtwdev = chip->rtwdev;
	struct ieee80211_hw *hw = rtwdev->hw;
	struct ieee80211_tx_info *info;
	struct ieee80211_vif *vif;
	struct sk_buff *skb;
	size_t len = m->m_pkthdr.len;
	int qsel;
	int ret;

	if (is_mgmt && rtw89_chip_tx_dbg < 10) {
		uint8_t fc[2] = { 0, 0 };

		if (len >= 2)
			m_copydata(m, 0, 2, fc);
		printf("rtw89usb: mgmt tx %zu bytes fc %02x %02x\n",
		    len, fc[0], fc[1]);
		rtw89_chip_tx_dbg++;
	}

	if (!is_mgmt || !chip->vif_added || len == 0 ||
	    len > RTW89_USB_TX_BUFSZ) {
		m_freem(m);
		return 0;
	}

	/* the HCI tx_write skb_push()es the txdesc: headroom is required */
	skb = alloc_skb(len + hw->extra_tx_headroom, GFP_ATOMIC);
	if (skb == NULL) {
		m_freem(m);
		return 0;
	}
	skb_reserve(skb, hw->extra_tx_headroom);
	m_copydata(m, 0, len, skb_put(skb, len));

	/*
	 * No REQ_TX_STATUS for now: the firmware TX-report path is a
	 * known grey zone on USB and net80211 scanning needs no
	 * per-frame status.
	 */
	vif = rtw89_mac80211_vif(hw);
	info = IEEE80211_SKB_CB(skb);
	info->flags = 0;
	info->control.vif = vif;
	info->control.sta = NULL;

	ret = rtw89_core_tx_write(rtwdev, vif, NULL, skb, &qsel);
	if (ret != 0) {
		ieee80211_free_txskb(hw, skb);
		m_freem(m);
		return 0;
	}

	rtw89_core_tx_kick_off(rtwdev, (u8)qsel);
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
