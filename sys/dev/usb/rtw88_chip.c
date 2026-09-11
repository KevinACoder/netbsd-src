/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by the RK3568 lab (NetBSD/RTL8821CU bring-up).
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
 * Chip bring-up glue for the RTL8821CU: runs the imported rtw88 core
 * (firmware download, MAC/BB/RF tables, H2C/C2H) and bridges it to
 * net80211 through the small API in rtw88_chipvar.h.
 *
 * This is where the driver deliberately does not look like the Linux one:
 * there is no mac80211 here, so the RX demultiplexer hands received frames
 * to the net80211 driver directly, and TX fills in struct rtw_tx_pkt_info
 * itself instead of taking mac80211's rate control output.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
#include <sys/device.h>
#include <sys/mbuf.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include "rtw88var.h"

/*
 * Longest frame net80211 can ever want (no HT: 802.11's 2346-byte MPDU).
 * Kept local: chip.c must not include sys/net80211, the shadow mac80211
 * headers collide with it.
 */
#define	RTW88_RX_FRAME_MAX	2346

/* Bring-up instrumentation: demuxed rx packet counter. */
static unsigned int rtw88_rx_pkt_dbg;
static unsigned int rtw88_bad_pkt_dbg;
static unsigned int rtw88_tx_mgmt_dbg;
static unsigned int rtw88_tx_probe_dbg;
static unsigned int rtw88_rx_bcn_dbg;
static unsigned int rtw88_rx_mgmt_dbg;
static unsigned int rtw88_txpwr_dbg;

static void
rtw88_chip_setup_device(struct rtw88_chip *chip, device_t dev)
{

	snprintf(chip->hostdev.name, sizeof(chip->hostdev.name), "%s",
	    device_xname(dev));
	chip->hostdev.dv_dev = dev;
	chip->rtwdev.dev = &chip->hostdev;
}

/*
 * Bring the chip up to the point where net80211 can be attached: transport,
 * firmware, efuse and the band/channel tables.
 */
struct rtw88_chip *
rtw88_chip_attach(struct usbd_device *udev, struct usbd_interface *iface,
    device_t dev, struct rtw88_hw_info *info)
{
	struct rtw88_chip *chip;
	struct rtw_dev *rtwdev;
	int error;

	chip = kmem_zalloc(sizeof(*chip), KM_SLEEP);
	rtwdev = &chip->rtwdev;

	rtw88_workqueue_ready();

	chip->hw = rtw88_mac80211_alloc(rtwdev);
	rtwdev->hw = chip->hw;
	rtw88_chip_setup_device(chip, dev);

	rtwdev->chip = &rtw8821c_hw_spec;
	rtwdev->hci.type = RTW_HCI_TYPE_USB;
	rtwdev->hci.ops = rtw88_usb_get_ops();

	chip->usb.udev = udev;

	error = rtw88_usb_attach(chip, iface);
	if (error != 0) {
		rtw_err(rtwdev, "failed to set up the USB transport\n");
		goto err_usb;
	}

	error = rtw_core_init(rtwdev);
	if (error != 0) {
		rtw_err(rtwdev, "failed to initialise the core\n");
		goto err_usb;
	}

	error = rtw_chip_info_setup(rtwdev);
	if (error != 0) {
		rtw_err(rtwdev, "failed to set up chip parameters\n");
		goto err_core;
	}

	rtw_info(rtwdev, "chip id %u, cut %u, sys_cfg 0x%08x\n",
	    rtwdev->chip->id, rtwdev->hal.cut_version,
	    rtwdev->hal.chip_version);
	rtw_info(rtwdev,
	    "phy cond: rfe %u (full 0x%04x) pkg %u, intf usb\n",
	    rtwdev->efuse.rfe_option, rtwdev->efuse.rfe_option_full,
	    rtwdev->hal.pkg_type);

	error = rtw_register_hw(rtwdev, chip->hw);
	if (error != 0) {
		rtw_err(rtwdev, "failed to initialise the device\n");
		goto err_core;
	}

	ether_addr_copy(info->mac_addr, rtwdev->efuse.addr);
	memcpy(info->fw_version, chip->hw->wiphy->fw_version,
	    sizeof(info->fw_version));
	info->efuse_valid = is_valid_ether_addr(rtwdev->efuse.addr);

	rtw_info(rtwdev, "RTL8821CU ready (firmware %s)\n",
	    chip->hw->wiphy->fw_version);
	return chip;

err_core:
	rtw_core_deinit(rtwdev);
err_usb:
	rtw88_usb_detach(chip);
	rtw88_mac80211_free(chip->hw);
	kmem_free(chip, sizeof(*chip));
	return NULL;
}

void
rtw88_chip_detach(struct rtw88_chip *chip)
{
	struct rtw_dev *rtwdev = &chip->rtwdev;

	/* rtw_core_deinit() destroys the core mutexes itself */
	rtw_core_deinit(rtwdev);
	rtw88_usb_detach(chip);
	rtw88_mac80211_free(chip->hw);
	kmem_free(chip, sizeof(*chip));
}

int
rtw88_chip_start(struct rtw88_chip *chip)
{
	struct rtw_dev *rtwdev = &chip->rtwdev;
	int error;

	rtw_info(rtwdev, "chip start begin\n");
	rtw88_trace_arm(0);
	mutex_lock(&rtwdev->mutex);
	error = rtw_core_start(rtwdev);
	mutex_unlock(&rtwdev->mutex);
	if (error != 0) {
		rtw_err(rtwdev, "failed to start the chip (%d)\n", error);
		return error;
	}
	rtw88_trace_arm(0);
	rtw_info(rtwdev, "chip start done\n");
	/*
	 * Bring-up probes: the firmware asserts BIT_WINTINI_RDY and friends
	 * in REG_MCUFW_CTRL while booting; REG_HMETFR shows whether the H2C
	 * mailboxes are free (bits clear) or still owned by the firmware.
	 */
	rtw_info(rtwdev, "fw state: MCUFW_CTRL 0x%08x HMETFR 0x%02x\n",
	    rtw_read32(rtwdev, REG_MCUFW_CTRL), rtw_read8(rtwdev, REG_HMETFR));
	rtw88_usb_dbg_dump(rtwdev);
	rtw88_trace_arm(150);
	chip->started = true;

	/*
	 * Program the interface address into MACID0 (port 0, like Linux's
	 * rtw_vif_port_config with PORT_SET_MAC_ADDR).  The RX "accept
	 * unicast to me" filter matches against this register; left at the
	 * power-on default it discards every unicast response (probe/auth)
	 * while broadcast frames still pass.
	 */
	{
		const uint8_t *lladdr = rtw88_chip_mac_addr(chip);
		int i;

		for (i = 0; i < 6; i++)		/* ETHER_ADDR_LEN */
			rtw_write8(rtwdev, 0x0610 + i, lladdr[i]);
	}

	/*
	 * Tune to channel 1 right away: net80211 only asks for a channel
	 * once it starts scanning, leaving the radio on the firmware's
	 * power-on default until then.  This also exercises/prints the
	 * TX power programming path at a predictable time.
	 */
	rtw88_chip_set_channel(chip, 1);
	return 0;
}

void
rtw88_chip_stop(struct rtw88_chip *chip)
{
	struct rtw_dev *rtwdev = &chip->rtwdev;

	if (!chip->started)
		return;
	rtw_info(rtwdev, "chip stop\n");
	mutex_lock(&rtwdev->mutex);
	rtw_core_stop(rtwdev);
	mutex_unlock(&rtwdev->mutex);
	chip->started = false;
}

/*
 * net80211 hands us a channel number; the chip core wants the matching entry
 * of the band table it built in rtw_register_hw().
 */
int
rtw88_chip_set_channel(struct rtw88_chip *chip, unsigned int chan)
{
	struct rtw_dev *rtwdev = &chip->rtwdev;
	struct ieee80211_hw *hw = chip->hw;
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *c = NULL;
	int i, band;

	for (band = 0; band < IEEE80211_NUM_BANDS; band++) {
		sband = hw->wiphy->bands[band];
		if (sband == NULL)
			continue;
		for (i = 0; i < sband->n_channels; i++) {
			if (sband->channels[i].hw_value == chan) {
				c = &sband->channels[i];
				break;
			}
		}
		if (c != NULL)
			break;
	}
	if (c == NULL) {
		rtw_err(rtwdev, "unknown channel %u\n", chan);
		return EINVAL;
	}

	hw->conf.chandef.chan = c;
	hw->conf.chandef.width = NL80211_CHAN_WIDTH_20_NOHT;
	hw->conf.chandef.center_freq1 = c->center_freq;

	mutex_lock(&rtwdev->mutex);
	rtw_set_channel(rtwdev);
	mutex_unlock(&rtwdev->mutex);

	/*
	 * Bring-up: the computed TX power indices.  If the efuse power
	 * tables were mis-parsed these are garbage and the radio is
	 * effectively mute even though every USB transaction succeeds.
	 */
	if (rtw88_txpwr_dbg < 40) {
		printf("rtw88dbg chan %u txpwr A: 1M 0x%02x 2M 0x%02x "
		    "5.5M 0x%02x 11M 0x%02x 6M 0x%02x 54M 0x%02x "
		    "MCS7 0x%02x\n",
		    chan,
		    rtwdev->hal.tx_pwr_tbl[0][DESC_RATE1M],
		    rtwdev->hal.tx_pwr_tbl[0][DESC_RATE2M],
		    rtwdev->hal.tx_pwr_tbl[0][DESC_RATE5_5M],
		    rtwdev->hal.tx_pwr_tbl[0][DESC_RATE11M],
		    rtwdev->hal.tx_pwr_tbl[0][DESC_RATE6M],
		    rtwdev->hal.tx_pwr_tbl[0][DESC_RATE54M],
		    rtwdev->hal.tx_pwr_tbl[0][DESC_RATEMCS7]);
		rtw88_txpwr_dbg++;
	}

	rtw_dbg(rtwdev, RTW_DBG_STATE, "channel %u (%u MHz)\n", chan,
	    c->center_freq);
	return 0;
}

/*
 * TX: the packet info is filled in here rather than by mac80211, so the rate
 * is a fixed legacy OFDM/CCK rate.  Management frames use the lowest basic
 * rate, like the Linux driver does when it has no rate control to consult.
 */
static int
rtw88_chip_tx_frame(struct rtw88_chip *chip, struct mbuf *m, bool is_mgmt)
{
	struct rtw_dev *rtwdev = &chip->rtwdev;
	struct rtw_tx_pkt_info pkt_info = {0};
	struct sk_buff *skb;
	struct ieee80211_hdr *hdr;
	size_t len = m->m_pkthdr.len;
	int error;

	if (len < IEEE80211_HDRLEN) {
		m_freem(m);
		return EINVAL;
	}

	/*
	 * The transport pushes the TX descriptor in front of the frame, so the
	 * buffer needs the headroom mac80211 would otherwise reserve
	 * (hw->extra_tx_headroom = chip->tx_pkt_desc_sz).
	 */
	skb = alloc_skb(rtwdev->chip->tx_pkt_desc_sz + len, GFP_ATOMIC);
	if (skb == NULL) {
		m_freem(m);
		return ENOMEM;
	}
	skb_reserve(skb, rtwdev->chip->tx_pkt_desc_sz);
	m_copydata(m, 0, len, skb_put(skb, len));
	m_freem(m);

	hdr = (struct ieee80211_hdr *)skb->data;

	/*
	 * NetBSD net80211 has no mgd_prepare_tx(); run the Linux equivalent
	 * (RF calibration before the first management TX after tuning) here,
	 * or the AUTH/ASSOC frames go out on an uncalibrated radio.
	 */
	if (is_mgmt)
		rtw_chip_prepare_tx(rtwdev);

	/* Bring-up: observe the AUTH/ASSOC handshake going out.  Probe
	 * requests are printed for the first few only, they arrive in
	 * bursts that would spend the budget of the interesting frames. */
	if (is_mgmt) {
		unsigned int sub = (le16_to_cpu(hdr->frame_control) >> 4) & 0xf;

		if (sub == 4) {			/* probe-req */
			if (rtw88_tx_probe_dbg < 3) {
				printf("rtw88dbg tx probe-req len %zu\n",
				    len);
				rtw88_tx_probe_dbg++;
			}
		} else if (rtw88_tx_mgmt_dbg < 100) {
			printf("rtw88dbg tx mgmt sub %u fc 0x%04x -> "
			    "%02x:%02x:%02x:%02x:%02x:%02x len %zu\n",
			    sub, le16_to_cpu(hdr->frame_control),
			    hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
			    hdr->addr1[3], hdr->addr1[4], hdr->addr1[5], len);
			rtw88_tx_mgmt_dbg++;
		}
	}

	pkt_info.tx_pkt_size = len;
	pkt_info.offset = rtwdev->chip->tx_pkt_desc_sz;
	pkt_info.ls = true;
	pkt_info.mac_id = 0;
	pkt_info.bw = RTW_CHANNEL_WIDTH_20;
	pkt_info.sec_type = 0;			/* software crypto only */
	pkt_info.bmc = is_broadcast_ether_addr(hdr->addr1) ||
	    is_multicast_ether_addr(hdr->addr1);
	pkt_info.dis_rate_fallback = true;
	pkt_info.use_rate = true;
	pkt_info.en_hwseq = true;
	pkt_info.dis_qselseq = true;
	pkt_info.hw_ssn_sel = 0;
	pkt_info.rate_id = RTW_RATEID_B_20M;
	pkt_info.seq = (le16_to_cpu(hdr->seq_ctrl) & IEEE80211_SCTL_SEQ) >> 4;

	if (is_mgmt) {
		pkt_info.qsel = TX_DESC_QSEL_MGMT;
		pkt_info.rate = DESC_RATE1M;
	} else {
		pkt_info.qsel = TX_DESC_QSEL_TID0;	/* best effort */
		pkt_info.rate = DESC_RATE54M;
	}

	error = rtw_hci_tx_write(rtwdev, &pkt_info, skb);
	if (error != 0) {
		rtw88_skb_free(skb);
		rtw_err(rtwdev, "failed to queue TX frame (%d)\n", error);
		return error;
	}
	rtw_hci_tx_kick_off(rtwdev);
	return 0;
}

int
rtw88_chip_tx(struct rtw88_chip *chip, struct mbuf *m, bool is_mgmt)
{

	return rtw88_chip_tx_frame(chip, m, is_mgmt);
}

void
rtw88_chip_set_callbacks(struct rtw88_chip *chip, void *ctx,
    rtw88_rx_cb_t rx_cb, rtw88_scan_cb_t scan_cb)
{

	chip->rx_ctx = ctx;
	chip->rx_cb = rx_cb;
	chip->scan_cb = scan_cb;
}

void
rtw88_chip_set_assoc(struct rtw88_chip *chip, const uint8_t *bssid, bool assoc)
{

	chip->assoc = assoc;
	rtw88_mac80211_set_assoc(chip->hw, bssid, assoc);
	rtw88_mac80211_set_sta(chip->hw, bssid, assoc);
}

void
rtw88_chip_set_bssid(struct rtw88_chip *chip, const uint8_t *bssid)
{
	struct rtw_dev *rtwdev = &chip->rtwdev;
	int i;

	/*
	 * Program the port-0 BSSID filter (like Linux's PORT_SET_BSSID)
	 * before the AUTH exchange: the firmware matches received frames
	 * against this register once management state starts moving, and
	 * leaves it at the power-on default otherwise.
	 */
	for (i = 0; i < 6; i++)		/* ETHER_ADDR_LEN */
		rtw_write8(rtwdev, 0x0618 + i, bssid[i]);

	/* PORT_SET_NET_TYPE: net type = managed/station (REG_CR 16-17). */
	rtw_write32_mask(rtwdev, 0x0100, 0x30000, RTW_NET_MGD_LINKED);
}

bool
rtw88_chip_ready(const struct rtw88_chip *chip)
{

	return chip->started;
}

const uint8_t *
rtw88_chip_mac_addr(const struct rtw88_chip *chip)
{

	return chip->info.mac_addr;
}

/* ------------------------------------------------------------------ */
/* RX demultiplexer                                                    */
/* ------------------------------------------------------------------ */

/*
 * The bulk IN pipe carries a run of packets: each one is a receive
 * descriptor, the driver info area, the PHY status (for the 8821C it is
 * parsed out of the descriptor) and the 802.11 frame, padded to 8 bytes.
 * C2H reports go to the firmware layer, data frames to net80211.
 */
void
rtw88_chip_rx_work(struct work_struct *w)
{
	struct rtw88_usb *usb = container_of(w, struct rtw88_usb, rx_work);
	struct rtw_dev *rtwdev = usb->rtwdev;
	struct rtw88_chip *chip = rtw88_chip_from_dev(rtwdev);
	struct ieee80211_rx_status rx_status;
	struct rtw_rx_pkt_stat pkt_stat;
	struct ieee80211_hw *hw = chip->hw;
	u32 pkt_desc_sz = rtwdev->chip->rx_pkt_desc_sz;
	u32 max_skb_len = pkt_desc_sz + 2048 + IEEE80211_MAX_MPDU_LEN_VHT_11454;
	struct sk_buff *rx_skb;
	struct sk_buff *skb;
	u32 pkt_offset, next_pkt, skb_len;
	u8 *rx_desc, *rx_buf;
	int limit;

	for (limit = 0; limit < 200; limit++) {
		rx_skb = rtw88_skb_dequeue(&usb->rx_queue);
		if (rx_skb == NULL)
			break;

		rx_desc = rx_skb->data;
		do {
			rx_buf = rx_desc + pkt_desc_sz;
			rtw_rx_query_rx_desc(rtwdev, rx_desc, rx_buf, &pkt_stat,
			    &rx_status);

			/* Bring-up instrumentation: what does the demux see? */
			if (rtw88_rx_pkt_dbg < 30)
				printf("rtw88dbg rxpkt #%u: len %u drvinfo %u "
				    "shift %u c2h %d rssi %d\n", rtw88_rx_pkt_dbg,
				    pkt_stat.pkt_len, pkt_stat.drv_info_sz,
				    pkt_stat.shift, pkt_stat.is_c2h,
				    pkt_stat.rssi);
			rtw88_rx_pkt_dbg++;

			pkt_offset = pkt_desc_sz + pkt_stat.drv_info_sz +
			    pkt_stat.shift;
			skb_len = pkt_stat.pkt_len + pkt_offset;
			if (skb_len > max_skb_len ||
			    (u32)(rx_desc - rx_skb->data) + skb_len >
			    rx_skb->len) {
				if (rtw88_bad_pkt_dbg < 30) {
					const uint8_t *p = rx_desc;
					unsigned int n;

					rtw_warn(rtwdev,
					    "bad packet: skb_len %u len %u "
					    "drvinfo %u shift %u c2h %d "
					    "xfer %u off %u\n",
					    skb_len, pkt_stat.pkt_len,
					    pkt_stat.drv_info_sz,
					    pkt_stat.shift, pkt_stat.is_c2h,
					    rx_skb->len,
					    (u32)(rx_desc - rx_skb->data));
					for (n = 0; n < 16; n++)
						printf("%02x%s", p[n],
						    (n % 16 == 15) ?
						    "\n" : " ");
				}
				rtw88_bad_pkt_dbg++;
				break;
			}
			if (pkt_stat.pkt_len <= FCS_LEN && !pkt_stat.is_c2h) {
				rtw_dbg(rtwdev, RTW_DBG_USB,
				    "skipping short packet (%u)\n",
				    pkt_stat.pkt_len);
				goto next;
			}

			skb = alloc_skb(skb_len, GFP_ATOMIC);
			if (skb == NULL) {
				rtw_dbg(rtwdev, RTW_DBG_USB,
				    "no buffer for a %u byte packet\n",
				    skb_len);
				goto next;
			}
			skb_put_data(skb, rx_desc, skb_len);

			if (pkt_stat.is_c2h) {
				rtw_fw_c2h_cmd_rx_irqsafe(rtwdev, pkt_offset,
				    skb);
			} else {
				size_t flen;
				uint16_t fc;

				skb_pull(skb, pkt_offset);

				/*
				 * Sanity gate: the device can emit runs of
				 * garbage descriptors, so never hand a run
				 * of random bytes to net80211 -- a bogus
				 * length corrupts kernel memory downstream.
				 */
				if (skb->len < 16 ||
				    skb->len > RTW88_RX_FRAME_MAX) {
					if (rtw88_bad_pkt_dbg < 30)
						rtw_dbg(rtwdev, RTW_DBG_USB,
						    "dropping garbage frame "
						    "(len %u)\n", skb->len);
					rtw88_bad_pkt_dbg++;
					rtw88_skb_free(skb);
					goto next;
				}
				fc = le16toh(*(uint16_t *)skb->data);
				if ((fc & 0x0003) != 0) {
					if (rtw88_bad_pkt_dbg < 30)
						rtw_dbg(rtwdev, RTW_DBG_USB,
						    "dropping frame with fc "
						    "0x%04x (len %u)\n", fc,
						    skb->len);
					rtw88_bad_pkt_dbg++;
					rtw88_skb_free(skb);
					goto next;
				}

				/*
				 * Bring-up: print management frames so the
				 * AUTH/ASSOC handshake is observable.  The
				 * per-class caps keep beacon noise from
				 * spending the budget of the interesting
				 * subtypes.
				 */
				if ((fc & 0x0c) == 0) { /* mgmt */
					static const char *st[] =
					    {"assoc-req", "assoc-resp",
					     "reassoc-req",
					     "reassoc-resp",
					     "probe-req",
					     "probe-resp",
					     "?6", "?7", "beacon",
					     "?9", "disassoc", "auth",
					     "deauth", "action",
					     "?14", "?15"};
					unsigned int sub = (fc >> 4) & 0xf;
					unsigned int *ctr = sub == 8 ?
					    &rtw88_rx_bcn_dbg :
					    &rtw88_rx_mgmt_dbg;

					if (*ctr < (sub == 8 ? 30 : 100)) {
						printf("rtw88dbg rx mgmt %s "
						    "len %u\n",
						    st[sub], skb->len);
						(*ctr)++;
					}
				}

				rtw_rx_stats(rtwdev,
				    rtw88_mac80211_vif(hw), skb);

				/* net80211 wants the frame without the FCS */
				flen = skb->len > FCS_LEN ?
				    skb->len - FCS_LEN : skb->len;
				if (chip->rx_cb != NULL && flen > 0)
					chip->rx_cb(chip->rx_ctx, skb->data,
					    flen, pkt_stat.rssi);
				rtw88_skb_free(skb);
			}
next:
			next_pkt = round_up(skb_len, 8);
			rx_desc += next_pkt;
		} while ((u32)(rx_desc - rx_skb->data) + pkt_desc_sz <
		    rx_skb->len);

		rtw88_skb_free(rx_skb);
	}
}
