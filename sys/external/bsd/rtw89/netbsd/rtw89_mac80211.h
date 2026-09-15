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
 * mac80211/cfg80211 shapes for the imported rtw89 chip code.
 *
 * These types exist so that 34k lines of Linux driver sources compile; the
 * driver itself never runs the mac80211 paths (no ieee80211_register_hw, no
 * rate control, no TXQ).  The members are only as complete as the chip code
 * actually touches, and the drv_priv[] areas are sized so that the chip
 * structures (rtw_vif, rtw_sta_info, rtw_txq) fit.
 *
 * net/mac80211.h and net/cfg80211.h both just include this file.
 */

#ifndef _RTW89_MAC80211_H_
#define _RTW89_MAC80211_H_

#include "rtw89_compat.h"

/*
 * The rtw88 compat layer is built into the same kernel and exports the
 * same mac80211 shadow API names; route every symbol the rtw89 sources
 * define or call to this driver's own implementations.
 */
#define	wiphy_to_ieee80211_hw		rtw89_wiphy_to_ieee80211_hw
#define	ieee80211_register_hw		rtw89_ieee80211_register_hw
#define	ieee80211_unregister_hw		rtw89_ieee80211_unregister_hw
#define	ieee80211_restart_hw		rtw89_ieee80211_restart_hw
#define	ieee80211_queue_work		rtw89_ieee80211_queue_work
#define	ieee80211_queue_delayed_work	rtw89_ieee80211_queue_delayed_work
#define	ieee80211_wake_queues		rtw89_ieee80211_wake_queues
#define	ieee80211_stop_queues		rtw89_ieee80211_stop_queues
#define	ieee80211_tx_status_irqsafe	rtw89_ieee80211_tx_status_irqsafe
#define	ieee80211_free_txskb		rtw89_ieee80211_free_txskb
#define	ieee80211_tx_info_clear_status	rtw89_ieee80211_tx_info_clear_status
#define	ieee80211_tx_dequeue		rtw89_ieee80211_tx_dequeue
#define	ieee80211_txq_get_depth		rtw89_ieee80211_txq_get_depth
#define	ieee80211_find_sta		rtw89_ieee80211_find_sta
#define	ieee80211_find_sta_by_ifaddr	rtw89_ieee80211_find_sta_by_ifaddr
#define	ieee80211_iterate_active_interfaces_atomic \
					rtw89_ieee80211_iterate_active_interfaces_atomic
#define	ieee80211_iterate_active_interfaces \
					rtw89_ieee80211_iterate_active_interfaces
#define	ieee80211_iterate_stations_atomic \
					rtw89_ieee80211_iterate_stations_atomic
#define	ieee80211_iterate_stations	rtw89_ieee80211_iterate_stations
#define	ieee80211_iter_keys		rtw89_ieee80211_iter_keys
#define	ieee80211_iter_keys_rcu		rtw89_ieee80211_iter_keys_rcu
#define	ieee80211_request_smps		rtw89_ieee80211_request_smps
#define	ieee80211_start_tx_ba_session	rtw89_ieee80211_start_tx_ba_session
#define	ieee80211_scan_completed	rtw89_ieee80211_scan_completed
#define	ieee80211_connection_loss	rtw89_ieee80211_connection_loss
#define	ieee80211_cqm_rssi_notify	rtw89_ieee80211_cqm_rssi_notify
#define	ieee80211_nullfunc_get		rtw89_ieee80211_nullfunc_get
#define	ieee80211_probereq_get		rtw89_ieee80211_probereq_get
#define	ieee80211_proberesp_get		rtw89_ieee80211_proberesp_get
#define	ieee80211_pspoll_get		rtw89_ieee80211_pspoll_get
#define	ieee80211_beacon_get_tim	rtw89_ieee80211_beacon_get_tim
#define	get_random_mask_addr		rtw89_get_random_mask_addr
#define	regulatory_hint			rtw89_regulatory_hint

/* v7.0 annotations and limits (needed before any struct below) */
#define	__rcu
#define	IEEE80211_MLD_MAX_NUM_LINKS	15
#define	IEEE80211_MAX_AMPDU_BUF		0x40
enum ieee80211_agg_state {
	IEEE80211_AMPDU_RX_START = 0,
	IEEE80211_AMPDU_RX_STOP,
	IEEE80211_AMPDU_TX_START,
	IEEE80211_AMPDU_TX_STOP_CONT,
	IEEE80211_AMPDU_TX_STOP_FLUSH,
	IEEE80211_AMPDU_TX_STOP_FLUSH_CONT,
	IEEE80211_AMPDU_TX_OPERATIONAL,
};
struct wiphy_work;
struct wiphy_delayed_work;
struct ieee80211_ops;
struct element {
	u8			id;
	u8			datalen;
	u8			data[];
} __packed;

/* ------------------------------------------------------------------ */
/* constants                                                           */
/* ------------------------------------------------------------------ */

#define	IEEE80211_NUM_ACS	4
#define	IEEE80211_NUM_TIDS	16
#define	IEEE80211_NUM_BANDS	4
#define	NUM_NL80211_BANDS	IEEE80211_NUM_BANDS
#define	IEEE80211_MAX_SSID_LEN	32
#define	IEEE80211_MAX_DATA_LEN	2304
#define	IEEE80211_MAX_CHAINS	4
#define	IEEE80211_MAX_MPDU_LEN_VHT_11454	11454
#define	FCS_LEN					4
#define	IEEE80211_MAX_AMPDU_BUF_HT	256

enum nl80211_band {
	NL80211_BAND_2GHZ = 0,
	NL80211_BAND_5GHZ = 1,
	NL80211_BAND_60GHZ = 2,
	NL80211_BAND_6GHZ = 3,
};

enum nl80211_sar_type {
	NL80211_SAR_TYPE_POWER,
	NL80211_SAR_TYPE_COUNT
};

enum nl80211_iftype {
	NL80211_IFTYPE_UNSPECIFIED = 0,
	NL80211_IFTYPE_ADHOC = 1,
	NL80211_IFTYPE_STATION = 2,
	NL80211_IFTYPE_AP = 3,
	NL80211_IFTYPE_AP_VLAN = 4,
	NL80211_IFTYPE_WDS = 5,
	NL80211_IFTYPE_MONITOR = 6,
	NL80211_IFTYPE_MESH_POINT = 7,
	NL80211_IFTYPE_P2P_CLIENT = 8,
	NL80211_IFTYPE_P2P_GO = 9,
	NL80211_IFTYPE_P2P_DEVICE = 10,
};

enum nl80211_chan_width {
	NL80211_CHAN_WIDTH_20_NOHT = 0,
	NL80211_CHAN_WIDTH_20 = 1,
	NL80211_CHAN_WIDTH_40 = 2,
	NL80211_CHAN_WIDTH_80 = 3,
	NL80211_CHAN_WIDTH_80P80 = 4,
	NL80211_CHAN_WIDTH_160 = 5,
};

enum ieee80211_ac_numbers {
	IEEE80211_AC_VO = 0,
	IEEE80211_AC_VI = 1,
	IEEE80211_AC_BE = 2,
	IEEE80211_AC_BK = 3,
};

enum ieee80211_smps_mode {
	IEEE80211_SMPS_OFF = 0,
	IEEE80211_SMPS_STATIC = 1,
	IEEE80211_SMPS_DYNAMIC = 2,
	IEEE80211_SMPS_AUTOMATIC = 3,
};

enum ieee80211_sta_rx_bandwidth {
	IEEE80211_STA_RX_BW_20 = 0,
	IEEE80211_STA_RX_BW_40 = 1,
	IEEE80211_STA_RX_BW_80 = 2,
	IEEE80211_STA_RX_BW_160 = 3,
	IEEE80211_STA_RX_BW_320 = 4,
};

/* channel flags */
#define	IEEE80211_CHAN_NO_IR		BIT(4)
#define	IEEE80211_CHAN_RADAR		BIT(3)
/* IEEE80211_CHAN_NO_IBSS and _PASSIVE_SCAN are aliases of _NO_IR, defined
 * by the imported regd.h exactly as Linux does. */
#define	IEEE80211_CHAN_NO_HT40MINUS	BIT(7)
#define	IEEE80211_CHAN_NO_80MHZ		BIT(11)
#define	IEEE80211_CHAN_DISABLED		0

/* HT capability bits */
#define	IEEE80211_HT_CAP_LDPC_CODING			BIT(0)
#define	IEEE80211_HT_CAP_SUP_WIDTH_20_40		BIT(1)
#define	IEEE80211_HT_CAP_SGI_20				BIT(5)
#define	IEEE80211_HT_CAP_SGI_40				BIT(6)
#define	IEEE80211_HT_CAP_TX_STBC			BIT(7)
#define	IEEE80211_HT_CAP_RX_STBC			(BIT(8) | BIT(9))
#define	IEEE80211_HT_CAP_RX_STBC_SHIFT			8
#define	IEEE80211_HT_CAP_DSSSCCK40			BIT(10)
#define	IEEE80211_HT_CAP_MAX_AMSDU			BIT(11)
#define	IEEE80211_HT_MCS_TX_DEFINED			BIT(0)
#define	IEEE80211_HT_MAX_AMPDU_64K			3
#define	IEEE80211_HT_MPDU_DENSITY_2			2

/* VHT capability bits */
#define	IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_11454		BIT(1)
#define	IEEE80211_VHT_CAP_RXLDPC			BIT(4)
#define	IEEE80211_VHT_CAP_SHORT_GI_80			BIT(5)
#define	IEEE80211_VHT_CAP_SHORT_GI_160			BIT(6)
#define	IEEE80211_VHT_CAP_RXSTBC_MASK			(BIT(8) | BIT(9) | BIT(10))
#define	IEEE80211_VHT_CAP_RXSTBC_1			BIT(8)
#define	IEEE80211_VHT_CAP_SU_BEAMFORMER_CAPABLE		BIT(11)
#define	IEEE80211_VHT_CAP_SU_BEAMFORMEE_CAPABLE		BIT(12)
#define	IEEE80211_VHT_CAP_BEAMFORMEE_STS_SHIFT		13
#define	IEEE80211_VHT_CAP_BEAMFORMEE_STS_MASK		(BIT(13) | BIT(14) | BIT(15))
#define	IEEE80211_VHT_CAP_SOUNDING_DIMENSIONS_SHIFT	16
#define	IEEE80211_VHT_CAP_SOUNDING_DIMENSIONS_MASK	(BIT(16) | BIT(17) | BIT(18))
#define	IEEE80211_VHT_CAP_MU_BEAMFORMER_CAPABLE		BIT(19)
#define	IEEE80211_VHT_CAP_MU_BEAMFORMEE_CAPABLE		BIT(20)
#define	IEEE80211_VHT_CAP_TXSTBC			BIT(26)
#define	IEEE80211_VHT_CAP_HTC_VHT			BIT(29)
#define	IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK BIT(31)
#define	IEEE80211_VHT_MCS_SUPPORT_0_7			0
#define	IEEE80211_VHT_MCS_SUPPORT_0_8			1
#define	IEEE80211_VHT_MCS_SUPPORT_0_9			2
#define	IEEE80211_VHT_MCS_NOT_SUPPORTED			3

/* RX status flags and rate encoding (rx.c fills struct ieee80211_rx_status) */
#define	RX_FLAG_MMIC_ERROR			BIT(0)
#define	RX_FLAG_DECRYPTED			BIT(1)
#define	RX_FLAG_FAILED_FCS_CRC			BIT(3)
#define	RX_FLAG_FAILED_PLCP_CRC			BIT(4)
#define	RX_FLAG_HT				BIT(6)
#define	RX_FLAG_40MHZ				BIT(7)
#define	RX_FLAG_VHT				BIT(11)
#define	RX_FLAG_MACTIME_START			BIT(12)
#define	RX_FLAG_NO_SIGNAL_VAL			BIT(15)
#define	RX_FLAG_NO_PSDU				BIT(29)

#define	RX_ENC_LEGACY				0
#define	RX_ENC_FLAG_SHORTPRE		BIT(0)
#define	RX_ENC_FLAG_SHORT_GI		BIT(1)
#define	RX_ENC_FLAG_HT_GF		BIT(2)
#define	RX_ENC_FLAG_LDPC		BIT(3)
#define	RX_ENC_FLAG_STBC_MASK		(BIT(4) | BIT(5))
#define	RX_ENC_FLAG_STBC_SHIFT		4
#define	RX_ENC_FLAG_BF			BIT(6)
#define	RX_ENC_HT				1
#define	RX_ENC_VHT				2

/* ieee80211_hw flags: the chip code names them without the prefix */
enum ieee80211_hw_flags {
	IEEE80211_HW_HAS_RATE_CONTROL,
	IEEE80211_HW_RX_INCLUDES_FCS,
	IEEE80211_HW_HOST_BROADCAST_PS_BUFFERING,
	IEEE80211_HW_SIGNAL_UNSPEC,
	IEEE80211_HW_SIGNAL_DBM,
	IEEE80211_HW_NEED_DTIM_BEFORE_ASSOC,
	IEEE80211_HW_SPECTRUM_MGMT,
	IEEE80211_HW_AMPDU_AGGREGATION,
	IEEE80211_HW_SUPPORTS_PS,
	IEEE80211_HW_PS_NULLFUNC_STACK,
	IEEE80211_HW_SUPPORTS_DYNAMIC_PS,
	IEEE80211_HW_MFP_CAPABLE,
	IEEE80211_HW_WANT_MONITOR_VIF,
	IEEE80211_HW_NO_AUTO_VIF,
	IEEE80211_HW_SW_CRYPTO_CONTROL,
	IEEE80211_HW_SUPPORT_FAST_XMIT,
	IEEE80211_HW_REPORTS_TX_ACK_STATUS,
	IEEE80211_HW_CONNECTION_MONITOR,
	IEEE80211_HW_QUEUE_CONTROL,
	IEEE80211_HW_SUPPORTS_PER_STA_GTK,
	IEEE80211_HW_AP_LINK_PS,
	IEEE80211_HW_TX_AMSDU,
	IEEE80211_HW_TX_FRAG_LIST,
	IEEE80211_HW_REPORTS_LOW_ACK,
	IEEE80211_HW_SUPPORTS_TX_FRAG,
	IEEE80211_HW_SUPPORTS_AMSDU_IN_AMPDU,
	IEEE80211_HW_SINGLE_SCAN_ON_ALL_BANDS,
	IEEE80211_HW_CHANCTX_STA_CSA,
	IEEE80211_HW_SUPPORTS_MULTI_BSSID,
	IEEE80211_HW_SUPPORTS_VHT_EXT_NSS_BW,
	NUM_IEEE80211_HW_FLAGS,
};

/* TX/RX status flags */
#define	IEEE80211_TX_CTL_SEND_AFTER_DTIM	BIT(1)
#define	IEEE80211_TX_CTL_INJECTED		BIT(15)
#define	IEEE80211_TX_CTL_NO_CCK_RATE		BIT(6)
#define	IEEE80211_TX_CTL_NO_ACK			BIT(2)
#define	IEEE80211_TX_CTL_TX_OFFCHAN		BIT(12)
#define	IEEE80211_TX_CTL_AMPDU			BIT(5)
#define	IEEE80211_TX_CTL_REQ_TX_STATUS		BIT(10)
#define	IEEE80211_TX_CTL_NO_PS_BUFFER		BIT(3)
#define	IEEE80211_TX_STAT_ACK			BIT(13)
#define	IEEE80211_TX_STAT_NOACK_TRANSMITTED	BIT(14)

#define	IEEE80211_CONF_IDLE			BIT(3)
#define	IEEE80211_CONF_CHANGE_SMPS		BIT(0)
#define	IEEE80211_CONF_CHANGE_LISTEN_INTERVAL	BIT(1)
#define	IEEE80211_CONF_CHANGE_MONITOR		BIT(2)
#define	IEEE80211_CONF_CHANGE_IDLE		BIT(3)
#define	IEEE80211_CONF_CHANGE_CHANNEL		BIT(4)
#define	IEEE80211_CONF_CHANGE_RETRY_LIMITS	BIT(5)
#define	IEEE80211_CONF_CHANGE_POWER_TYPE	BIT(6)
#define	IEEE80211_CONF_MONITOR			BIT(4)
#define	FIF_ALLMULTI				BIT(1)
#define	FIF_FCSFAIL				BIT(2)
#define	FIF_PLCPFAIL				BIT(3)
#define	FIF_BCN_PRBRESP_PROMISC			BIT(4)
#define	FIF_CONTROL				BIT(5)
#define	FIF_OTHER_BSS				BIT(6)
#define	FIF_PSPOLL				BIT(7)
#define	FIF_PROBE_REQ				BIT(8)
#define	IEEE80211_VIF_BEACON_FILTER		BIT(0)
#define	IEEE80211_VIF_SUPPORTS_CQM_RSSI		BIT(1)
#define	IEEE80211_VIF_SUPPORTS_UAPSD		BIT(2)
#define	NL80211_CHAN_NO_HT			NL80211_CHAN_WIDTH_20_NOHT
#define	IEEE80211_FCTL_PM			0x1000
#define	IEEE80211_CHANCTX_CHANGE_WIDTH		BIT(0)
#define	IEEE80211_CHANCTX_CHANGE_RX_CHAINS	BIT(1)
#define	IEEE80211_CHANCTX_CHANGE_TX_CHAINS	BIT(2)
#define	IEEE80211_CHANCTX_CHANGE_RADAR		BIT(3)
#define	IEEE80211_CHANCTX_CHANGE_MIN_WIDTH	BIT(4)
#define	IEEE80211_CHANCTX_CHANGE_PUNCTURING	BIT(5)
#define	NUM_NL80211_IFTYPES			(NL80211_IFTYPE_P2P_DEVICE + 1)
#define	IEEE80211_P2P_OPPPS_ENABLE_BIT		BIT(8)
#define	IEEE80211_P2P_OPPPS_CTWINDOW_MASK	0x00ff

#define	IEEE80211_IFACE_ITER_NORMAL		0

#define	IEEE80211_KEY_FLAG_PAIRWISE		BIT(0)
#define	IEEE80211_KEY_FLAG_GENERATE_IV		BIT(1)
#define	IEEE80211_KEY_FLAG_GENERATE_MMIC	BIT(2)
#define	IEEE80211_KEY_FLAG_PUT_IV_SPACE		BIT(3)
#define	IEEE80211_KEY_FLAG_RX_MGMT		BIT(4)
#define	IEEE80211_KEY_FLAG_SW_MGMT_TX		BIT(5)
#define	IEEE80211_KEY_FLAG_MFP_CAPABLE		BIT(6)

#define	WLAN_CIPHER_SUITE_WEP40			0x000fac01
#define	WLAN_CIPHER_SUITE_TKIP			0x000fac02
#define	WLAN_CIPHER_SUITE_CCMP			0x000fac04
#define	WLAN_CIPHER_SUITE_WEP104		0x000fac05
#define	WLAN_CIPHER_SUITE_AES_CMAC		0x000fac06
#define	WLAN_CIPHER_SUITE_GCMP			0x000fac08
#define	WLAN_CIPHER_SUITE_GCMP_256		0x000fac09
#define	WLAN_CIPHER_SUITE_CCMP_256		0x000fac0a
#define	WLAN_CIPHER_SUITE_BIP_GMAC_128		0x000fac0b
#define	WLAN_CIPHER_SUITE_BIP_GMAC_256		0x000fac0c

#define	WIPHY_FLAG_SUPPORTS_TDLS		BIT(0)
#define	WIPHY_FLAG_TDLS_EXTERNAL_SETUP		BIT(1)
#define	WIPHY_FLAG_IBSS_RSN			BIT(2)
#define	WIPHY_FLAG_AP_UAPSD				BIT(3)
#define	WIPHY_FLAG_HAS_CHANNEL_SWITCH			BIT(4)
#define	WIPHY_FLAG_SUPPORTS_EXT_KEK_KCK			BIT(5)
#define	WIPHY_FLAG_SPLIT_SCAN_6GHZ			BIT(6)
#define	WIPHY_FLAG_DISABLE_WEXT				BIT(7)
#define	WIPHY_FLAG_SUPPORTS_MLO				BIT(8)

#define	NL80211_FEATURE_SCAN_RANDOM_MAC_ADDR	BIT(5)
#define	NL80211_EXT_FEATURE_CAN_REPLACE_PTK0	1
#define	NL80211_EXT_FEATURE_SCAN_RANDOM_SN	2
#define	NL80211_EXT_FEATURE_SET_SCAN_DWELL	3
#define	NL80211_SCAN_FLAG_COLOCATED_6GHZ	BIT(2)
#define	NL80211_SCAN_FLAG_RANDOM_ADDR		BIT(0)
#define	NL80211_SCAN_FLAG_RANDOM_SN		BIT(1)

#define	REGULATORY_STRICT_REG			BIT(0)
#define	REGULATORY_COUNTRY_IE_IGNORE		BIT(1)

#define	NL80211_REGDOM_SET_BY_DRIVER		0
#define	NL80211_REGDOM_SET_BY_USER		1
#define	NL80211_REGDOM_SET_BY_CORE		2

#define	NL80211_DFS_UNSET			0
#define	NL80211_DFS_FCC				1
#define	NL80211_DFS_ETSI			2
#define	NL80211_DFS_JP				3

#define	NL80211_SAR_TYPE_POWER			0

enum nl80211_cqm_rssi_threshold_event {
	NL80211_CQM_RSSI_THRESHOLD_EVENT_LOW	= 0,
	NL80211_CQM_RSSI_THRESHOLD_EVENT_HIGH	= 1,
};

#define	RATE_INFO_FLAGS_MCS			BIT(0)
#define	RATE_INFO_FLAGS_SHORT_GI		BIT(1)
#define	RATE_INFO_FLAGS_VHT_MCS			BIT(2)
#define	RATE_INFO_BW_5				0
#define	RATE_INFO_BW_10				1
#define	RATE_INFO_BW_20				2
#define	RATE_INFO_BW_40				3
#define	RATE_INFO_BW_80				4
#define	RATE_INFO_BW_160			5
#define	RATE_INFO_BW_HE_RU			6
#define	RATE_INFO_BW_320			7
#define	RATE_INFO_BW_EHT_RU			8

#define	RATE_INFO_FLAGS_HE_MCS			BIT(5)
#define	RATE_INFO_FLAGS_EHT_MCS			BIT(6)
#define	NL80211_RATE_INFO_HE_GI_0_8		0
#define	NL80211_RATE_INFO_HE_GI_1_6		1
#define	NL80211_RATE_INFO_HE_GI_3_2		2
#define	NL80211_RATE_INFO_EHT_GI_0_8		0
#define	NL80211_RATE_INFO_EHT_GI_1_6		1
#define	NL80211_RATE_INFO_EHT_GI_3_2		2
#define	RATE_INFO_FLAGS_60G			BIT(0)

/* full Linux v7.0 set: fw.c/mac.c test these bits against their own state */
#define	BSS_CHANGED_ASSOC			BIT(0)
#define	BSS_CHANGED_ERP_CTS_PROT		BIT(1)
#define	BSS_CHANGED_ERP_PREAMBLE		BIT(2)
#define	BSS_CHANGED_ERP_SLOT			BIT(3)
#define	BSS_CHANGED_HT				BIT(4)
#define	BSS_CHANGED_BASIC_RATES			BIT(5)
#define	BSS_CHANGED_BEACON_INT			BIT(6)
#define	BSS_CHANGED_BSSID			BIT(7)
#define	BSS_CHANGED_BEACON			BIT(8)
#define	BSS_CHANGED_BEACON_ENABLED		BIT(9)
#define	BSS_CHANGED_ARP_FILTER			BIT(10)
#define	BSS_CHANGED_QOS				BIT(11)
#define	BSS_CHANGED_CQM				BIT(12)
#define	BSS_CHANGED_IBSS			BIT(13)
#define	BSS_CHANGED_IDLE			BIT(14)
#define	BSS_CHANGED_SSID			BIT(15)
#define	BSS_CHANGED_AP_PROBE_RESP		BIT(16)
#define	BSS_CHANGED_PS				BIT(17)
#define	BSS_CHANGED_TXPOWER			BIT(18)
#define	BSS_CHANGED_P2P_PS			BIT(19)
#define	BSS_CHANGED_BEACON_INFO			BIT(20)
#define	BSS_CHANGED_BANDWIDTH			BIT(21)
#define	BSS_CHANGED_OCB				BIT(22)
#define	BSS_CHANGED_MU_GROUPS			BIT(23)
#define	BSS_CHANGED_KEEP_ALIVE			BIT(24)
#define	BSS_CHANGED_MCAST_RATE			BIT(25)
#define	BSS_CHANGED_MLD_VALID_LINKS		BIT(26)
#define	BSS_CHANGED_HE_BSS_COLOR		BIT(27)
#define	BSS_CHANGED_TPE				BIT(28)


/* HE/EHT MAC+PHY capability bits and MCS encodings (Linux values).  Only
 * used to build the advertised sband tables, which the net80211 front end
 * never reads (netbsd-11 net80211 has no HE/EHT). */
#define	IEEE80211_HE_MAC_CAP0_HTC_HE				BIT(0)
#define	IEEE80211_HE_MAC_CAP1_TF_MAC_PAD_DUR_MASK		GENMASK(5, 4)
#define	IEEE80211_HE_MAC_CAP1_TF_MAC_PAD_DUR_16US		0x20
#define	IEEE80211_HE_MAC_CAP2_ALL_ACK				BIT(1)
#define	IEEE80211_HE_MAC_CAP2_BSR				BIT(4)
#define	IEEE80211_HE_MAC_CAP3_OMI_CONTROL			BIT(6)
#define	IEEE80211_HE_MAC_CAP3_MAX_AMPDU_LEN_EXP_MASK		GENMASK(4, 3)
#define	IEEE80211_HE_MAC_CAP3_MAX_AMPDU_LEN_EXP_EXT_2		0x08
#define	IEEE80211_HE_MAC_CAP4_AMSDU_IN_AMPDU			BIT(2)
#define	IEEE80211_HE_MAC_CAP4_OPS				BIT(4)
#define	IEEE80211_HE_MAC_CAP5_HT_VHT_TRIG_FRAME_RX		BIT(2)

#define	IEEE80211_HE_PHY_CAP1_DEVICE_CLASS_A			BIT(4)
#define	IEEE80211_HE_PHY_CAP1_HE_LTF_AND_GI_FOR_HE_PPDUS_0_8US	BIT(5)
#define	IEEE80211_HE_PHY_CAP2_NDP_4x_LTF_AND_3_2US		BIT(0)
#define	IEEE80211_HE_PHY_CAP2_NDP_4				BIT(1)
#define	IEEE80211_HE_PHY_CAP2_STBC_TX_UNDER_80MHZ		BIT(2)
#define	IEEE80211_HE_PHY_CAP2_DOPPLER_TX			BIT(5)
#define	IEEE80211_HE_PHY_CAP3_DCM_MAX_CONST_TX_16_QAM		BIT(2)
#define	IEEE80211_HE_PHY_CAP3_DCM_MAX_TX_NSS_2			BIT(4)
#define	IEEE80211_HE_PHY_CAP3_RX_PARTIAL_BW_SU_IN_20MHZ_MU	BIT(5)
#define	IEEE80211_HE_PHY_CAP4_SU_BEAMFORMEE			BIT(0)
#define	IEEE80211_HE_PHY_CAP4_BEAMFORMEE_MAX_STS_UNDER_80MHZ_4	0x06
#define	IEEE80211_HE_PHY_CAP4_BEAMFORMEE_MAX_STS_ABOVE_80MHZ_4	0x18
#define	IEEE80211_HE_PHY_CAP5_NG16_SU_FEEDBACK			BIT(0)
#define	IEEE80211_HE_PHY_CAP5_NG16_MU_FEEDBACK			BIT(1)
#define	IEEE80211_HE_PHY_CAP6_CODEBOOK_SIZE_42_SU		BIT(0)
#define	IEEE80211_HE_PHY_CAP6_CODEBOOK_SIZE_75_MU		BIT(1)
#define	IEEE80211_HE_PHY_CAP6_TRIG_SU_BEAMFORMING_FB		BIT(2)
#define	IEEE80211_HE_PHY_CAP7_POWER_BOOST_FACTOR_SUPP		BIT(0)
#define	IEEE80211_HE_PHY_CAP7_MAX_NC_1				0x10
#define	IEEE80211_HE_PHY_CAP8_20MHZ_IN_160MHZ_HE_PPDU		BIT(2)
#define	IEEE80211_HE_PHY_CAP8_80MHZ_IN_160MHZ_HE_PPDU		BIT(3)
#define	IEEE80211_HE_PHY_CAP8_HE_ER_SU_1XLTF_AND_08_US_GI	BIT(4)
#define	IEEE80211_HE_PHY_CAP8_DCM_MAX_RU_996			0x02
#define	IEEE80211_HE_PHY_CAP9_LONGER_THAN_16_SIGB_OFDM_SYM	BIT(0)
#define	IEEE80211_HE_PHY_CAP9_RX_FULL_BW_SU_USING_MU_WITH_COMP_SIGB	BIT(1)
#define	IEEE80211_HE_PHY_CAP9_RX_FULL_BW_SU_USING_MU_WITH_NON_COMP_SIGB	BIT(2)
#define	IEEE80211_HE_PHY_CAP9_TX_1024_QAM_LESS_THAN_242_TONE_RU	BIT(3)
#define	IEEE80211_HE_PHY_CAP9_RX_1024_QAM_LESS_THAN_242_TONE_RU	BIT(4)
#define	IEEE80211_HE_PHY_CAP9_NOMINAL_PKT_PADDING_16US		0x20
#define	IEEE80211_HE_MCS_SUPPORT_0_11				2
#define	IEEE80211_HE_MCS_NOT_SUPPORTED				3

#define	IEEE80211_EHT_MAC_CAP0_MAX_MPDU_LEN_MASK		GENMASK(1, 0)
#define	IEEE80211_EHT_PHY_CAP0_320MHZ_IN_6GHZ			BIT(1)
#define	IEEE80211_EHT_PHY_CAP0_NDP_4_EHT_LFT_32_GI		BIT(3)
#define	IEEE80211_EHT_PHY_CAP0_SU_BEAMFORMEE			BIT(4)
#define	IEEE80211_EHT_PHY_CAP0_BEAMFORMEE_SS_80MHZ_MASK		GENMASK(7, 5)
#define	IEEE80211_EHT_PHY_CAP1_BEAMFORMEE_SS_80MHZ_MASK		GENMASK(2, 0)
#define	IEEE80211_EHT_PHY_CAP1_BEAMFORMEE_SS_160MHZ_MASK	GENMASK(5, 3)
#define	IEEE80211_EHT_PHY_CAP1_BEAMFORMEE_SS_320MHZ_MASK	GENMASK(7, 6)
#define	IEEE80211_EHT_PHY_CAP3_TRIG_SU_BF_FDBK			BIT(0)
#define	IEEE80211_EHT_PHY_CAP3_TRIG_MU_BF_PART_BW_FDBK		BIT(1)
#define	IEEE80211_EHT_PHY_CAP3_CODEBOOK_4_2_SU_FDBK		BIT(2)
#define	IEEE80211_EHT_PHY_CAP3_CODEBOOK_7_5_MU_FDBK		BIT(3)
#define	IEEE80211_EHT_PHY_CAP4_POWER_BOOST_FACT_SUPP		BIT(0)
#define	IEEE80211_EHT_PHY_CAP4_MAX_NC_MASK			GENMASK(4, 2)
#define	IEEE80211_EHT_PHY_CAP5_COMMON_NOMINAL_PKT_PAD_20US	0x10
#define	IEEE80211_VHT_EXT_NSS_BW_CAPABLE			BIT(0)
#define	SUPPORTS_VHT_EXT_NSS_BW					0x20

#define	IEEE80211_HE_6GHZ_CAP_MIN_MPDU_START			GENMASK(2, 0)
#define	IEEE80211_HE_6GHZ_CAP_MAX_AMPDU_LEN_EXP			GENMASK(5, 3)
#define	IEEE80211_HE_6GHZ_CAP_MAX_MPDU_LEN			GENMASK(7, 6)
#define	IEEE80211_VHT_MAX_AMPDU_1024K				3
#define	NL80211_CHAN_WIDTH_320					13
#define	struct_size_t(type, member, count)				\
	(sizeof(type) + (count) * sizeof(((type *)0)->member[0]))
#define	IEEE80211_WMM_IE_STA_QOSINFO_SP_ALL			0x0f
#define	IEEE80211_RADIOTAP_MCS_HAVE_FEC				0x01
#define	IEEE80211_RADIOTAP_MCS_HAVE_STBC			0x02
#define	IEEE80211_RADIOTAP_VHT_KNOWN_STBC			0x1000

/* 802.11 frame control */
#define	IEEE80211_FCTL_FTYPE		0x000c
#define	IEEE80211_FCTL_STYPE		0x00f0
#define	IEEE80211_FCTL_TODS		0x0100
#define	IEEE80211_FCTL_FROMDS		0x0200
#define	IEEE80211_FTYPE_MGMT		0x0000
#define	IEEE80211_FTYPE_CTL		0x0004
#define	IEEE80211_FTYPE_DATA		0x0008
#define	IEEE80211_STYPE_BEACON		0x0080
#define	IEEE80211_STYPE_PROBE_RESP	0x0050
#define	IEEE80211_STYPE_QOS_NULLFUNC	0x00c0
#define	IEEE80211_STYPE_NULLFUNC	0x0040
#define	IEEE80211_SCTL_SEQ		0xfff0

#define	IEEE80211_HDRLEN		24

#define	IEEE80211_SKB_CB(skb)	((struct ieee80211_tx_info *)((skb)->cb))
#define	IEEE80211_SKB_RXCB(skb)	((struct ieee80211_rx_status *)((skb)->cb))

/* ------------------------------------------------------------------ */
/* 802.11 headers                                                      */
/* ------------------------------------------------------------------ */

struct ieee80211_hdr {
	__le16		frame_control;
	__le16		duration_id;
	u8		addr1[6];
	u8		addr2[6];
	u8		addr3[6];
	__le16		seq_ctrl;
	u8		addr4[6];
} __packed;

struct ieee80211_hdr_3addr {
	__le16		frame_control;
	__le16		duration_id;
	u8		addr1[6];
	u8		addr2[6];
	u8		addr3[6];
	__le16		seq_ctrl;
} __packed;

struct ieee80211_mgmt {
	__le16		frame_control;
	__le16		duration;
	u8		da[6];
	u8		sa[6];
	u8		bssid[6];
	__le16		seq_ctrl;
	union {
		struct {
			__le64	timestamp;
			__le16	beacon_int;
			__le16	capab_info;
			u8	variable[1];
		} __packed beacon;
		struct {
			__le64	timestamp;
			__le16	beacon_int;
			__le16	capab_info;
			u8	variable[1];
		} __packed probe_resp;
		u8		variable[1];
	} u;
} __packed;

static __always_inline __unused int
ieee80211_is_mgmt(__le16 fc)
{
	return (le16toh(fc) & IEEE80211_FCTL_FTYPE) == IEEE80211_FTYPE_MGMT;
}

static __always_inline __unused int
ieee80211_is_data(__le16 fc)
{
	return (le16toh(fc) & IEEE80211_FCTL_FTYPE) == IEEE80211_FTYPE_DATA;
}

static __always_inline __unused int
ieee80211_is_ctl(__le16 fc)
{
	return (le16toh(fc) & IEEE80211_FCTL_FTYPE) == IEEE80211_FTYPE_CTL;
}

static __always_inline __unused int
ieee80211_is_beacon(__le16 fc)
{
	return (le16toh(fc) & (IEEE80211_FCTL_FTYPE | IEEE80211_FCTL_STYPE)) ==
	    (IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_BEACON);
}

static __always_inline __unused int
ieee80211_is_probe_resp(__le16 fc)
{
	return (le16toh(fc) & (IEEE80211_FCTL_FTYPE | IEEE80211_FCTL_STYPE)) ==
	    (IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_PROBE_RESP);
}

static __always_inline __unused int
ieee80211_is_any_nullfunc(__le16 fc)
{
	u16 v = le16toh(fc) & (IEEE80211_FCTL_FTYPE | IEEE80211_FCTL_STYPE);

	return v == (IEEE80211_FTYPE_DATA | IEEE80211_STYPE_NULLFUNC) ||
	    v == (IEEE80211_FTYPE_DATA | IEEE80211_STYPE_QOS_NULLFUNC);
}

static __always_inline __unused int
ieee80211_has_tods(__le16 fc)
{
	return (le16toh(fc) & IEEE80211_FCTL_TODS) != 0;
}

static __always_inline __unused int
ieee80211_has_fromds(__le16 fc)
{
	return (le16toh(fc) & IEEE80211_FCTL_FROMDS) != 0;
}

/* ------------------------------------------------------------------ */
/* cfg80211 types                                                      */
/* ------------------------------------------------------------------ */

struct ieee80211_channel {
	u16			center_freq;
	u16			hw_value;
	u32			flags;
	u8			band;
	u8			max_antenna_gain;
	s8			max_power;
	s32			max_reg_power;
	bool			beacon_found;
};

struct ieee80211_rate {
	u32			flags;
	u16			bitrate;
	u16			hw_value;
	u16			hw_value_short;
};

enum nl80211_bss_scan_width {
	NL80211_BSS_CHAN_WIDTH_20 = 0,
};

struct ieee80211_sta_ht_cap {
	u16			cap;
	bool			ht_supported;
	u8			ampdu_factor;
	u8			ampdu_density;
	struct {
		u8		rx_mask[10];
		u16		rx_highest;
		u8		tx_params;
	} mcs;
};

struct ieee80211_sta_vht_cap {
	bool			vht_supported;
	u32			cap;
	struct {
		__le16		rx_mcs_map;
		__le16		tx_mcs_map;
		__le16		rx_highest;
		__le16		tx_highest;
	} vht_mcs;
};

struct ieee80211_supported_band {
	struct ieee80211_channel *channels;
	const struct ieee80211_sband_iftype_data *iftype_data;
	int			n_iftype_data;
	const struct ieee80211_rate *bitrates;
	enum nl80211_band	band;
	int			n_channels;
	int			n_bitrates;
	struct ieee80211_sta_ht_cap ht_cap;
	struct ieee80211_sta_vht_cap vht_cap;
};

struct cfg80211_chan_def {
	struct ieee80211_channel *chan;
	enum nl80211_chan_width	width;
	u32			center_freq1;
	u32			center_freq2;
	u16			punctured;
};

struct cfg80211_ssid {
	u8			ssid[IEEE80211_MAX_SSID_LEN];
	u8			ssid_len;
};

struct cfg80211_bitrate_mask {
	struct {
		u32		legacy;
		u8		ht_mcs[10];
		u16		vht_mcs[8];
		u8		he_mcs[8];
		u16		he_gi;
		u16		he_ltf;
		u8		eht_mcs[8];
		u8		eht_gi;
		u16		eht_ltf;
	} control[IEEE80211_NUM_BANDS];
};

struct cfg80211_sar_freq_ranges {
	u32			start_freq;
	u32			end_freq;
};

struct cfg80211_sar_capa {
	enum nl80211_sar_type	type;
	u32			num_freq_ranges;
	const struct cfg80211_sar_freq_ranges *freq_ranges;
};

struct cfg80211_sar_sub_specs {
	u32			freq_range_index;
	s32			power;
};

struct cfg80211_sar_specs {
	enum nl80211_sar_type	type;
	u32			num_sub_specs;
	const struct cfg80211_sar_sub_specs *sub_specs;
};

struct cfg80211_scan_info {
	u64			scan_start_tsf;
	u8			tsf_bssid[6];
	bool			aborted;
};

struct cfg80211_sched_scan_plan {
	u32			interval;
	u32			iterations;
};

struct cfg80211_match_set {
	struct cfg80211_ssid	ssid;
	u8			bssid[6];
	s32			rssi_thold;
	s32			per_band_rssi_thold[IEEE80211_NUM_BANDS];
};

struct cfg80211_scan_request {
	struct cfg80211_ssid	*ssids;
	struct ieee80211_channel **channels;
	struct cfg80211_scan_6ghz_params *scan_6ghz_params;
	u8			n_ssids;
	u32			n_channels;
	u32			n_6ghz_params;
	u32			flags;
	u32			duration;
	bool			duration_mandatory;
	bool			no_cck;
	size_t			ie_len;
	u8			mac_addr[6];
	u8			mac_addr_mask[6];
};

struct cfg80211_wowlan {
	bool			any;
	bool			disconnect;
	bool			magic_pkt;
	bool			gtk_rekey_failure;
	bool			eap_identity_req;
	bool			four_way_handshake;
	bool			rfkill_release;
	struct cfg80211_sched_scan_plan *patterns;
	bool			tcp;
	bool			net_detect;
};

struct wiphy_wowlan_support {
	u32			flags;
	int			n_patterns;
	int			pattern_max_len;
	int			pattern_min_len;
	int			max_pkt_offset;
};

struct ieee80211_iface_limit {
	u16			max;
	u16			types;
};

struct ieee80211_iface_combination {
	const struct ieee80211_iface_limit *limits;
	u32			num_different_channels;
	u16			max_interfaces;
	u8			n_limits;
	bool			beacon_int_infra_match;
	u8			radar_detect_widths;
	u8			radar_detect_regions;
};

struct ieee80211_he_mu_edca_param_ac_rec {
	u8			aifsn;
	u8			ecw_min_max;
	u8			mu_edca_timer;
} __packed;

struct ieee80211_tx_queue_params {
	u16			aifs;
	bool			mu_edca;
	struct ieee80211_he_mu_edca_param_ac_rec mu_edca_param_rec;
	u16			cw_min;
	u16			cw_max;
	u16			txop;
	bool			acm;
	bool			uapsd;
	bool			use_short_slot;
	bool			csa_active;
	struct {
		__le32		params;
	} he_oper;
};

struct ieee80211_scan_ies {
	const u8		*ies[IEEE80211_NUM_BANDS];
	size_t			len[IEEE80211_NUM_BANDS];
	const u8		*common_ies;
	size_t			common_ie_len;
};

struct ieee80211_scan_request {
	struct cfg80211_scan_request req;
	struct list_head	scan_list;
	struct ieee80211_scan_ies ies;
};

enum nl80211_dfs_regions {
	NL80211_DFS_REGION_UNSET = NL80211_DFS_UNSET,
	NL80211_DFS_REGION_FCC = NL80211_DFS_FCC,
	NL80211_DFS_REGION_ETSI = NL80211_DFS_ETSI,
	NL80211_DFS_REGION_JP = NL80211_DFS_JP,
};

struct regulatory_request {
	char			alpha2[2];
	enum nl80211_dfs_regions dfs_region;
	u32			initiator;
};

/*
 * The wiphy is large in Linux; only the members the chip code touches are
 * present.  ieee80211_hw embeds one so that hw->wiphy is always valid.
 */
struct wiphy {
	struct ieee80211_supported_band *bands[IEEE80211_NUM_BANDS];
	const struct wiphy_iftype_ext_capab *iftype_ext_capab;
	int			num_iftype_ext_capab;
	struct {
		u32		vif;
		u32		peer;
	} tid_config_support;
	u32			max_remain_on_channel_duration;
	u32			flags;
	u32			features;
	u32			regulatory_flags;
	u16			interface_modes;
	u32			available_antennas_tx;
	u32			available_antennas_rx;
	int			max_scan_ssids;
	int			max_sched_scan_ssids;
	u16			max_scan_ie_len;
	u32			rts_threshold;
	u8			fw_version[32];
	void			(*reg_notifier)(struct wiphy *,
					    struct regulatory_request *);
	const struct ieee80211_iface_combination *iface_combinations;
	int			n_iface_combinations;
	const struct cfg80211_sar_capa *sar_capa;
	const struct wiphy_wowlan_support *wowlan;
	void			*debugfsdir;
	u8			perm_addr[ETH_ALEN];
	kmutex_t		wiphy_mtx;
};

/* ------------------------------------------------------------------ */
/* mac80211 types                                                      */
/* ------------------------------------------------------------------ */

struct ieee80211_conf {
	u32			flags;
	int			power_level;
	int			max_power_level;
	struct cfg80211_chan_def chandef;
};

/* enough of mac80211's rate_info for the RA report */
struct rate_info {
	u8			flags;
	u8			legacy;
	u8			mcs;
	u8			nss;
	u8			bw;
	u8			he_gi;
	u8			eht_gi;
	u16			he_ru_alloc;
};

/* 6 GHz regulatory power types (regd.c) */
enum ieee80211_reg_power_type {
	IEEE80211_REG_UNSET_REG_POWER = 0,
	IEEE80211_REG_LPI_AP,
	IEEE80211_REG_SP_AP,
	IEEE80211_REG_VLP_AP,
};

/* DMI: x86-only, never matches on ARM (the table in core.c is empty) */
struct dmi_system_id {
	const void		*driver_data;
};
static __always_inline __unused const struct dmi_system_id *
dmi_first_match(const struct dmi_system_id *matches)
{
	return NULL;
}

/* P2P noa (power save off in the port; the type must be complete) */
struct ieee80211_p2p_noa_desc {
	u8			count;
	__le32			start_time;
	__le32			interval;
	__le16			duration;
} __packed;

struct ieee80211_parsed_tpe_eirp {
	bool			valid;
	u8			count;
	s8			power[16];
};

struct ieee80211_parsed_tpe_psd {
	bool			valid;
	u8			count;
	s8			power[16];
};

enum ieee80211_tpe_category {
	IEEE80211_TPE_CAT_6GHZ_DEFAULT = 0,
	IEEE80211_TPE_CAT_6GHZ_SUBORDINATE,
	IEEE80211_TPE_CAT_NUM,
};

struct ieee80211_parsed_tpe {
	struct {
		const u8	*data;
		u8		len;
	} noa, pwr_constraint;
	struct ieee80211_parsed_tpe_eirp max_tx_pwr;
	struct ieee80211_parsed_tpe_psd max_tx_pwr_psd;
	struct ieee80211_parsed_tpe_eirp max_local[IEEE80211_TPE_CAT_NUM];
	struct ieee80211_parsed_tpe_eirp max_reg_client[IEEE80211_TPE_CAT_NUM];
	struct ieee80211_parsed_tpe_psd psd_local[IEEE80211_TPE_CAT_NUM];
	struct ieee80211_parsed_tpe_psd psd_reg_client[IEEE80211_TPE_CAT_NUM];
};

struct ieee80211_chan_req {
	struct cfg80211_chan_def oper;
};

struct ieee80211_bss_conf {
	u8			addr[ETH_ALEN];
	struct ieee80211_chan_req chanreq;
	u8			bssid_index;
	u8			link_id;
	u8			bssid[ETH_ALEN];
	u8			dtim_period;
	bool			eht_support;
	bool			he_support;
	bool			nontransmitted;
	u8			transmitter_bssid[ETH_ALEN];
	struct {
		bool		enabled;
		u8		color;
	} he_bss_color;
	enum ieee80211_reg_power_type power_type;
	struct ieee80211_parsed_tpe tpe;
	struct {
		u8		index;
		u8		oppps_ctwindow;
		struct ieee80211_p2p_noa_desc desc[2];
	} p2p_noa_attr;
	bool			use_short_slot;
	bool			csa_active;
	struct {
		__le32		params;
	} he_oper;
	bool			assoc;
	u32			basic_rates;
	u16			aid;
	u16			beacon_int;
	bool			enable_beacon;
	s32			cqm_rssi_thold;
	u8			cqm_rssi_hyst;
	struct {
		u8		membership[8];
		u8		position[16];
	} mu_group;
};

struct ieee80211_tx_control {
	struct ieee80211_sta	*sta;
	struct ieee80211_txq	*txq;
};

enum mac80211_rate_control_flags {
	IEEE80211_TX_RC_FLAG_MAX		= 0x1000
};

/* mac80211 TX rate (info->control.rates[]) + RC flags */
#define	IEEE80211_TX_RC_MCS			0x0001
#define	IEEE80211_TX_RC_GREEN_FIELD		0x0002
#define	IEEE80211_TX_RC_40_MHZ_WIDTH		0x0004
#define	IEEE80211_TX_RC_80_MHZ_WIDTH		0x0008
#define	IEEE80211_TX_RC_160_MHZ_WIDTH		0x0010
#define	IEEE80211_TX_RC_DUP_DATA		0x0020
#define	IEEE80211_TX_RC_SHORT_GI		0x0040
#define	IEEE80211_TX_RC_VHT_MCS			0x0080
#define	IEEE80211_TX_RC_USE_SHORT_PREAMBLE	0x0100
#define	IEEE80211_TX_RC_USE_RTS_CTS		0x0200
#define	IEEE80211_TX_RC_USE_CTS_PROTECT		0x0400
#define	IEEE80211_TX_RC_EHT_MCS			0x0800

struct ieee80211_tx_rate {
	s8			idx;
	u16			bitrate;
	u8			flags;
	u8			count;
};

struct ieee80211_tx_info_rate {
	struct ieee80211_tx_rate rates[4];
};

struct ieee80211_ra_report {
	struct rate_info	txrate;
	unsigned int		bitrate;
	u8			bw;
	bool			might_fallback_legacy;
	u8			fallback_legacy;
	u16			hw_rate;
};


struct ieee80211_key_conf {
	u32			cipher;
	u8			link_id;
	atomic64_t		tx_pn;
	u8			icv_len;
	u8			iv_len;
	u8			hw_key_idx;
	u8			keyidx;
	u32			flags;
	u8			keylen;
	u8			key[32];
};

/* mac80211 helper: install the iftype_data table into the band */
static __always_inline __unused void
_ieee80211_set_sband_iftype_data(struct ieee80211_supported_band *sband,
    const struct ieee80211_sband_iftype_data *data, int n)
{
	sband->iftype_data = data;
	sband->n_iftype_data = n;
}

int	ieee80211_set_active_links(void *, u16);
struct ieee80211_hw *ieee80211_alloc_hw(unsigned int,
	    const struct ieee80211_ops *);
void	ieee80211_free_hw(struct ieee80211_hw *);

struct ieee80211_hw {
	struct ieee80211_conf	conf;
	struct wiphy		*wiphy;
	const struct ieee80211_ops *ops;
	void			*priv;
	unsigned long		flags;
	u32			extra_tx_headroom;
	u32			queues;
	int			txq_data_size;
	int			sta_data_size;
	int			vif_data_size;
	int			chanctx_data_size;
	u16			max_rx_aggregation_subframes;
	u16			max_tx_aggregation_subframes;
	u8			uapsd_max_sp_len;
	u8			radiotap_mcs_details;
	u16			radiotap_vht_details;
	u16			max_mtu;
};

struct ieee80211_vif {
	enum nl80211_iftype	type;
	u8			addr[ETH_ALEN];
	u16			valid_links;
	u16			active_links;
	u32			driver_flags;
	bool			p2p;
	struct ieee80211_bss_conf __rcu *link_conf[IEEE80211_MLD_MAX_NUM_LINKS];
	struct ieee80211_bss_conf bss_conf;
	struct {
		bool		assoc;
		bool		ps;
		bool		idle;
		bool		smps;
		u16		aid;
		u8		ap_addr[ETH_ALEN];
		__be32		arp_addr_list[4];
		int		arp_addr_cnt;
		bool		filter_flags;
		u32	power_type;
	} cfg;
	struct ieee80211_txq	*txq;
	u8			drv_priv[128] __aligned(sizeof(void *));
};

struct ieee80211_he_cap_elem {
	u8			phy_cap_info[11];
	u8			mac_cap_info[6];
};

struct ieee80211_he_mcs_nss_supp {
	u16			rx_mcs_80;
	u16			tx_mcs_80;
	u16			rx_mcs_160;
	u16			tx_mcs_160;
	u16			rx_mcs_80p80;
	u16			tx_mcs_80p80;
};

struct ieee80211_sta_he_cap {
	bool			has_he;
	struct ieee80211_he_cap_elem he_cap_elem;
	struct ieee80211_he_mcs_nss_supp he_mcs_nss_supp;
	u8			ppe_thres[25];
	u8			ppe_thres_len;
};

struct ieee80211_eht_cap_elem_fixed {
	u8			mac_cap_info[2];
	u8			phy_cap_info[9];
};

#define	IEEE80211_EHT_MCS_NSS_RX		0xf0
#define	IEEE80211_EHT_MCS_NSS_TX		0x0f

struct ieee80211_eht_mcs_nss_supp_20mhz_only {
	u8			rx_tx_max_nss[3];	/* MCS 9/11/13 */
};

struct ieee80211_eht_mcs_nss_supp_bw {
	union {
		u8		rx_tx_max_nss[4];
		struct {
			u8	rx_tx_mcs9_max_nss;
			u8	rx_tx_mcs11_max_nss;
			u8	rx_tx_mcs12_max_nss;
			u8	rx_tx_mcs13_max_nss;
		};
	};
};

struct ieee80211_eht_mcs_nss_supp {
	struct {
		struct ieee80211_eht_mcs_nss_supp_bw _80;
		struct ieee80211_eht_mcs_nss_supp_bw _160;
		struct ieee80211_eht_mcs_nss_supp_bw _320;
	} bw;
	struct ieee80211_eht_mcs_nss_supp_20mhz_only only_20mhz;
};

struct ieee80211_sta_eht_cap {
	bool			has_eht;
	struct ieee80211_eht_cap_elem_fixed eht_cap_elem;
	struct ieee80211_eht_mcs_nss_supp eht_mcs_nss_supp;
	u8			eht_ppe_thres[36];
	u8			eht_ppe_thres_len;
};

struct ieee80211_he_6ghz_capa {
	__le16			capa;
};

struct ieee80211_sband_iftype_data {
	u16			types_mask;
	enum nl80211_iftype	types;
	struct ieee80211_sta_he_cap he_cap;
	struct ieee80211_sta_eht_cap eht_cap;
	struct ieee80211_he_6ghz_capa he_6ghz_capa;
};

struct ieee80211_link_sta {
	u8			link_id;
	u8			addr[ETH_ALEN];
	u8			supp_rates[IEEE80211_NUM_BANDS];
	u8			rx_nss;
	struct ieee80211_sta	*sta;
	struct {
		u16		max_rc_amsdu_len;
		u8		max_amsdu_len;
	} agg;
	u8			bandwidth;
	struct ieee80211_sta_ht_cap ht_cap;
	struct ieee80211_sta_vht_cap vht_cap;
	struct ieee80211_sta_he_cap he_cap;
	struct ieee80211_sta_eht_cap eht_cap;
	u8			drv_priv[64] __aligned(sizeof(void *));
};

struct ieee80211_sta {
	u8			addr[ETH_ALEN];
	bool			mlo;
	u8			max_amsdu_subframes;
	u8			max_rc_amsdu_len;
	struct ieee80211_link_sta __rcu *link[IEEE80211_MLD_MAX_NUM_LINKS];
	u16			aid;
	bool			tdls;
	u8			bandwidth;
	struct ieee80211_sta_ht_cap ht_cap;
	struct ieee80211_sta_vht_cap vht_cap;
	struct ieee80211_txq	*txq[IEEE80211_NUM_TIDS];
	u8			supp_rates[IEEE80211_NUM_BANDS];
	struct ieee80211_link_sta deflink;
	u8			drv_priv[128] __aligned(sizeof(void *));
};

struct ieee80211_txq {
	u8			ac;
	u8			tid;
	struct ieee80211_sta	*sta;
	struct ieee80211_vif	*vif;
	u8			drv_priv[64] __aligned(sizeof(void *));
};

struct ieee80211_tx_info {
	u32			flags;
	u8			band;
	u8			hw_queue;
	struct {
		struct ieee80211_tx_rate rates[4];
		struct ieee80211_key_conf *hw_key;
		struct ieee80211_vif *vif;
		struct ieee80211_sta *sta;
		bool		use_rts;
		bool		use_cts_prot;
		u8		flags;
	} control;
	struct {
		u8		status_driver_data[18];
		u8		rates[4];
		u8		acked;
	} status;
	void			*driver_data[8];
};

struct ieee80211_rx_status {
	u64			mactime;
	u64			boottime_ns;
	u32			device_timestamp;
	u16			freq;
	u16			flag;
	u8			band;
	u8			encoding;
	u8			bw;
	u8			nss;
	u8			chains;
	s8			signal;
	s8			chain_signal[IEEE80211_MAX_CHAINS];
	u8			rate_idx;
	s8			vht_flag;
	u8			he_gi;
	u8			he_ltf;
	u16			enc_flags;
	bool			link_valid;
	u8			link_id;
	u8			zero_length_psdu_type;
	struct {
		u8		gi;
		u8		ltf;
	} eht;
};



/* ------------------------------------------------------------------ */
/* mac80211 helpers                                                    */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* mac80211 entry points the chip code calls; implemented in           */
/* rtw89_compat.c.  Only the ones on the driver's runtime path do      */
/* anything (queue_work, the iterate helpers, find_sta); the rest are  */
/* here so the chip code links.                                        */
/* ------------------------------------------------------------------ */

struct ieee80211_hw *wiphy_to_ieee80211_hw(struct wiphy *);
int	regulatory_hint(struct wiphy *, const char *);
void	get_random_mask_addr(u8 *, const u8 *, const u8 *);

int	ieee80211_register_hw(struct ieee80211_hw *);
void	ieee80211_unregister_hw(struct ieee80211_hw *);
void	ieee80211_restart_hw(struct ieee80211_hw *);
void	ieee80211_queue_work(struct ieee80211_hw *, struct work_struct *);
void	ieee80211_queue_delayed_work(struct ieee80211_hw *,
	    struct delayed_work *, unsigned long);
void	ieee80211_wake_queues(struct ieee80211_hw *);
void	ieee80211_stop_queues(struct ieee80211_hw *);
void	ieee80211_tx_status_irqsafe(struct ieee80211_hw *, struct sk_buff *);
void	ieee80211_free_txskb(struct ieee80211_hw *, struct sk_buff *);
void	ieee80211_tx_info_clear_status(struct ieee80211_tx_info *);
struct sk_buff *ieee80211_tx_dequeue(struct ieee80211_hw *,
	    struct ieee80211_txq *);
int	ieee80211_txq_get_depth(struct ieee80211_txq *, unsigned long *,
	    unsigned long *);
struct ieee80211_sta *ieee80211_find_sta(struct ieee80211_vif *,
	    const u8 *);
struct ieee80211_sta *ieee80211_find_sta_by_ifaddr(struct ieee80211_hw *,
	    const u8 *, const u8 *);
void	ieee80211_iterate_active_interfaces_atomic(struct ieee80211_hw *,
	    u32, void (*)(void *, u8 *, struct ieee80211_vif *), void *);
void	ieee80211_iterate_active_interfaces(struct ieee80211_hw *, u32,
	    void (*)(void *, u8 *, struct ieee80211_vif *), void *);
void	ieee80211_iterate_stations_atomic(struct ieee80211_hw *,
	    void (*)(void *, struct ieee80211_sta *), void *);
void	ieee80211_iterate_stations(struct ieee80211_hw *,
	    void (*)(void *, struct ieee80211_sta *), void *);
void	ieee80211_iter_keys(struct ieee80211_hw *, struct ieee80211_vif *,
	    void (*)(struct ieee80211_hw *, struct ieee80211_vif *,
	    struct ieee80211_sta *, struct ieee80211_key_conf *, void *),
	    void *);
void	ieee80211_iter_keys_rcu(struct ieee80211_hw *, struct ieee80211_vif *,
	    void (*)(struct ieee80211_hw *, struct ieee80211_vif *,
	    struct ieee80211_sta *, struct ieee80211_key_conf *, void *),
	    void *);
void	ieee80211_request_smps(struct ieee80211_vif *, int,
	    enum ieee80211_smps_mode);
int	ieee80211_start_tx_ba_session(struct ieee80211_sta *, u16, u16);
void	ieee80211_scan_completed(struct ieee80211_hw *,
	    struct cfg80211_scan_info *);
void	ieee80211_connection_loss(struct ieee80211_vif *);
void	ieee80211_cqm_rssi_notify(struct ieee80211_vif *,
	    enum nl80211_cqm_rssi_threshold_event, s32, gfp_t);
struct sk_buff *ieee80211_nullfunc_get(struct ieee80211_hw *,
	    struct ieee80211_vif *, int, bool);
struct sk_buff *ieee80211_probereq_get(struct ieee80211_hw *, const u8 *,
	    const u8 *, size_t, size_t);
struct sk_buff *ieee80211_proberesp_get(struct ieee80211_hw *,
	    struct ieee80211_vif *);
struct sk_buff *ieee80211_pspoll_get(struct ieee80211_hw *,
	    struct ieee80211_vif *);
struct sk_buff *ieee80211_beacon_get_tim(struct ieee80211_hw *,
	    struct ieee80211_vif *, u16 *, u16 *, int);



static __always_inline __unused void
ieee80211_hw_set_flag(struct ieee80211_hw *hw, unsigned int flag)
{
	set_bit(flag, &hw->flags);
}

#define	ieee80211_hw_set(hw, _name)	\
	ieee80211_hw_set_flag((hw), IEEE80211_HW_##_name)

static __always_inline __unused void
wiphy_ext_feature_set(struct wiphy *wiphy, unsigned int feature)
{
	set_bit(feature, (unsigned long *)&wiphy->features);
}

struct ieee80211_hw *wiphy_to_ieee80211_hw(struct wiphy *);
int	regulatory_hint(struct wiphy *, const char *);
void	get_random_mask_addr(u8 *, const u8 *, const u8 *);

#define	SET_IEEE80211_DEV(hw, dev)	do { } while (0)
#define	SET_IEEE80211_PERM_ADDR(hw, addr)				\
	do {								\
		if ((hw)->wiphy != NULL)				\
			memcpy((hw)->wiphy->perm_addr, (addr), ETH_ALEN); \
	} while (0)

static __always_inline __unused int
ieee80211_channel_to_frequency(int chan, enum nl80211_band band)
{
	if (band == NL80211_BAND_2GHZ) {
		if (chan == 14)
			return 2484;
		return 2407 + chan * 5;
	}
	return 5000 + chan * 5;
}

static __always_inline __unused int
cfg80211_get_ies_channel_number(const u8 *ies, size_t ielen,
    enum nl80211_band band)
{
	return 1;
}

static __always_inline __unused u32
cfg80211_calculate_bitrate(void *rate)
{
	return 0;
}

static __always_inline __unused bool
cfg80211_ssid_eq(struct cfg80211_ssid *a, struct cfg80211_ssid *b)
{
	if (a == NULL || b == NULL || a->ssid_len != b->ssid_len)
		return false;
	return memcmp(a->ssid, b->ssid, a->ssid_len) == 0;
}


/* ------------------------------------------------------------------ */
/* rtw89 (v7.0 snapshot) extensions                                    */
/* ------------------------------------------------------------------ */

/* RCU plumbing: rcu_read_lock() is a no-op (see rtw89_compat.h), so a
 * "call_rcu" callback runs immediately; there is no grace period to
 * wait for because no reader can hold a reference. */
struct rcu_head {
	void			*next;
	void			(*func)(struct rcu_head *);
};

#define	kfree_rcu(p, member)	kfree(p)
#define	call_rcu(h, f)		do { (f)((h)); } while (0)

/* Dummy net_device: the napi plumbing wants something to hold on to. */
struct net_device;
struct net_device *alloc_netdev_dummy(int sizeof_priv);
void	free_netdev(struct net_device *);
#define	netif_napi_add(dev, napi, poll)					\
	do { (void)(dev); (void)(napi); (void)(poll); } while (0)
#define	netif_napi_add_weight(dev, napi, poll, w)			\
	do { (void)(dev); (void)(napi); (void)(poll); (void)(w); } while (0)

/* wiphy mutex: mac80211 serialises driver callbacks against it */
#define	wiphy_lock(w)		mutex_enter(&(w)->wiphy_mtx)
#define	wiphy_unlock(w)		mutex_exit(&(w)->wiphy_mtx)
int	wiphy_locked_debugfs_read(struct wiphy *, struct file *, char *,
	    size_t, const void *, void (*)(struct wiphy *,
	    struct wiphy_work *, void *), char *);
int	wiphy_locked_debugfs_write(struct wiphy *, struct file *, char *,
	    size_t, const void *, void (*)(struct wiphy *,
	    struct wiphy_work *, void *), char *);

/* wiphy_work: mac80211's wiphy-mutex-serialised work.  Runs on the same
 * compat worker; the wiphy pointer is captured at queue time. */
typedef void (*wiphy_work_func_t)(struct wiphy *, struct wiphy_work *);
struct wiphy_delayed_work;

struct wiphy_work {
	struct work_struct	work;
	wiphy_work_func_t	func;
	struct wiphy		*wiphy;
	volatile bool		cancelled;
};

struct wiphy_delayed_work {
	struct wiphy_work	work;	/* func/wiphy live in the inner work */
	struct callout		dl_callout;
	volatile bool		cancelled;
	volatile bool		scheduled;
};

void	wiphy_work_init(struct wiphy_work *, wiphy_work_func_t);
void	wiphy_work_queue(struct wiphy *, struct wiphy_work *);
bool	wiphy_work_cancel(struct wiphy *, struct wiphy_work *);
void	wiphy_delayed_work_init(struct wiphy_delayed_work *,
	    wiphy_work_func_t);
void	wiphy_delayed_work_queue(struct wiphy *, struct wiphy_delayed_work *,
	    unsigned long);
bool	wiphy_delayed_work_cancel(struct wiphy *, struct wiphy_delayed_work *);
void	wiphy_delayed_work_flush(struct wiphy *, struct wiphy_delayed_work *);

/* rfkill plumbing: nothing drives it natively here */
#define	wiphy_rfkill_set_hw_state(w, b)	do { (void)(w); (void)(b); } while (0)
#define	wiphy_rfkill_start_polling(w)	do { (void)(w); } while (0)
#define	wiphy_rfkill_stop_polling(w)	do { (void)(w); } while (0)

/* NAPI: USB never polls, but core.c embeds a napi_struct and calls the
 * entry points from paths that must link. */
struct napi_struct {
	bool			enabled;
	bool			scheduled;
};

#define	napi_enable(n)		do { (n)->enabled = true; } while (0)
#define	napi_disable(n)		do { (n)->enabled = false; } while (0)
#define	napi_schedule(n)	do { (n)->scheduled = true; } while (0)
#define	napi_schedule_prep(n)	((n)->scheduled = true)
#define	napi_complete(n)	do { (n)->scheduled = false; } while (0)
#define	napi_complete_done(n, w)	do { (void)(w); (n)->scheduled = false; } while (0)
#define	napi_is_scheduled(n)	((n)->scheduled)
#define	napi_synchronize(n)	do { (void)(n); } while (0)
#define	netif_napi_del(n)	do { } while (0)

/* ------------------------------------------------------------------ */
/* cfg80211 v7.0 additions                                             */
/* ------------------------------------------------------------------ */

struct cfg80211_gtk_rekey_data {
	u8			replay_ctr[8];
	u8			kck[32];
	u8			kek[32];
	size_t			kck_len;
	size_t			kek_len;
	size_t			replay_ctr_len;
};

struct cfg80211_scan_6ghz_params {
	bool			short_ssid_valid;
	u32			short_ssid;
	u8			channel_idx;
	u8			bssid[ETH_ALEN];
};

struct cfg80211_tid_cfg {
	u8			tids;
	u8			mask;
	u8			ampdu;
	u8			amsdu;
	u8			amsdu_ctrl;
	u8			amsdu_num;
	u64			tx_rate;
	u8			tsinfo;
	u8			sz_tsinfo;
	u8			rts_ctrl;
	u8			retry_limit;
	u8			sz_retry_limit;
	u8			noack;
	u8			noack_index;
	u8			ba_session;
	u8			tx_agg_max_num;
	u8			rx_agg_max_num;
	u8			tx_agg_autosize;
	u8			rx_agg_autosize;
};

struct cfg80211_tid_config {
	u8			n_tid_conf;
	struct cfg80211_tid_cfg	tid_conf[];
};

struct cfg80211_sched_scan_request {
	struct cfg80211_ssid	*ssids;
	struct cfg80211_match_set *match_sets;
	struct ieee80211_channel **channels;
	int			n_ssids;
	u32			n_match_sets;
	u32			n_channels;
	u32			delay;
	u8			mac_addr[ETH_ALEN];
	u8			mac_addr_mask[ETH_ALEN];
};

struct cfg80211_bss_ies {
	u8			*data;
	size_t			len;
	u64			tsf;
	bool			from_beacon;
};

struct cfg80211_bss {
	struct cfg80211_bss_ies __rcu *ies;
	struct cfg80211_chan_def chandef;
	u8			bssid[ETH_ALEN];
	u16			beacon_interval;
	u16			capability_info;
};

/* ------------------------------------------------------------------ */
/* mac80211 v7.0 additions                                             */
/* ------------------------------------------------------------------ */

enum ieee80211_roc_type {
	IEEE80211_ROC_TYPE_NORMAL = 0,
	IEEE80211_ROC_TYPE_MGMT_TX,
};

#define	CHANCTX_SWMODE_REASSIGN_VIF	CHANCTX_SWMODE_REASSOCIATE_VIF
enum ieee80211_chanctx_switch_mode {
	CHANCTX_SWMODE_REASSOCIATE_VIF = 0,
	CHANCTX_SWMODE_REASSOCIATE_STA,
	CHANCTX_SWMODE_SWAP_CONTEXTS,
};

enum ieee80211_reconfig_type {
	IEEE80211_RECONFIG_TYPE_RESTART = 0,
	IEEE80211_RECONFIG_TYPE_SUSPEND,
};

enum ieee80211_sta_state {
	IEEE80211_STA_NOTEXIST = 0,
	IEEE80211_STA_NONE,
	IEEE80211_STA_AUTH,
	IEEE80211_STA_ASSOC,
	IEEE80211_STA_AUTHORIZED,
};

enum ieee80211_ampdu_tx_start_type {
	IEEE80211_AMPDU_TX_START_IMMEDIATE = 0,
	IEEE80211_AMPDU_TX_START_DELAY,
};

enum ieee80211_key_len {
	IEEE80211_KEYLEN_WEP40 = 5,
	IEEE80211_KEYLEN_CCMP = 16,
};

#define	NL80211_RATE_INFO_HE_RU_ALLOC_26	0
#define	NL80211_RATE_INFO_HE_RU_ALLOC_52	1
#define	NL80211_RATE_INFO_HE_RU_ALLOC_106	2
#define	NL80211_RATE_INFO_HE_RU_ALLOC_242	3
#define	NL80211_RATE_INFO_HE_RU_ALLOC_484	4
#define	NL80211_RATE_INFO_HE_RU_ALLOC_996	5
#define	NL80211_RATE_INFO_HE_RU_ALLOC_2x996	6

#define	IEEE80211_HE_OPERATION_ER_SU_DISABLE		BIT(5)
#define	IEEE80211_HE_OPERATION_BSS_COLOR_MASK		GENMASK(31, 24)
#define	IEEE80211_HT_MPDU_DENSITY_NONE			7
#define	IEEE80211_VHT_CAP_SUPP_CHAN_WIDTH_160MHZ		BIT(2)
#define	IEEE80211_VHT_CAP_SUPP_CHAN_WIDTH_160_80PLUS80MHZ BIT(3)
#define	NL80211_TID_CONFIG_ATTR_AMPDU_CTRL		1
#define	NL80211_TID_CONFIG_ATTR_AMSDU_CTRL		2
#define	NL80211_TID_CONFIG_ENABLE			1
#define	NL80211_TID_CONFIG_DISABLE			0
#define	IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_40MHZ_IN_2G	BIT(0)
#define	IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_40MHZ_80MHZ_IN_5G BIT(1)
#define	IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_160MHZ_IN_5G	BIT(2)
#define	IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_80PLUS80_MHZ_IN_5G BIT(3)
#define	IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_MASK_ALL	0x0f
#define	IEEE80211_HE_PHY_CAP1_LDPC_CODING_IN_PAYLOAD		BIT(1)
#define	IEEE80211_HE_PHY_CAP2_STBC_RX_UNDER_80MHZ		(BIT(0) | BIT(1))
#define	IEEE80211_HE_PHY_CAP3_DCM_MAX_CONST_RX_16_QAM		BIT(4)
#define	IEEE80211_HE_PHY_CAP3_DCM_MAX_CONST_RX_MASK		0x1c
#define	IEEE80211_HE_PHY_CAP3_DCM_MAX_TX_NSS_MASK		0x60
#define	IEEE80211_HE_PHY_CAP3_SU_BEAMFORMER			BIT(7)
#define	IEEE80211_HE_PHY_CAP4_MU_BEAMFORMER			BIT(1)
#define	IEEE80211_HE_PHY_CAP5_BEAMFORMEE_NUM_SND_DIM_UNDER_80MHZ_MASK (BIT(0) | BIT(1))
#define	IEEE80211_HE_PHY_CAP5_BEAMFORMEE_NUM_SND_DIM_ABOVE_80MHZ_MASK (BIT(2) | BIT(3))
#define	IEEE80211_HE_PHY_CAP6_PARTIAL_BW_EXT_RANGE		BIT(5)
#define	IEEE80211_HE_PHY_CAP6_PPE_THRESHOLD_PRESENT		BIT(7)
#define	IEEE80211_HE_PHY_CAP9_NOMINAL_PKT_PADDING_MASK		0x70
#define	IEEE80211_HE_PHY_CAP9_NOMINAL_PKT_PADDING_POS		4
#define	IEEE80211_PPE_THRES_RU_INDEX_BITMASK_MASK		0x78
#define	IEEE80211_PPE_THRES_RU_INDEX_BITMASK_POS		3
#define	IEEE80211_PPE_THRES_INFO_PPET_SIZE			3
#define	IEEE80211_PPE_THRES_NSS_MASK				0x0f
#define	IEEE80211_EHT_PHY_CAP5_PPE_THRESHOLD_PRESENT		BIT(5)
#define	IEEE80211_EHT_PHY_CAP5_COMMON_NOMINAL_PKT_PAD_MASK	(BIT(4) | BIT(5))
#define	IEEE80211_EHT_PPE_THRES_INFO_HEADER_SIZE		0
#define	IEEE80211_EHT_PPE_THRES_RU_INDEX_BITMASK_MASK		0x1f
#define	IEEE80211_EHT_PPE_THRES_INFO_PPET_SIZE			3
#define	IEEE80211_HE_PHY_CAP7_HE_SU_MU_PPDU_4XLTF_AND_08_US_GI	BIT(3)
#define	IEEE80211_HE_PHY_CAP8_HE_ER_SU_PPDU_4XLTF_AND_08_US_GI	BIT(5)

enum nl80211_he_ru_alloc {
	NL80211_RU_ALLOC_26 = 0,
	NL80211_RU_ALLOC_52,
	NL80211_RU_ALLOC_106,
	NL80211_RU_ALLOC_242,
	NL80211_RU_ALLOC_484,
	NL80211_RU_ALLOC_996,
	NL80211_RU_ALLOC_2x996,
};

/* HE/EHT capability shapes (only parsed, never advertised, by the port:
 * net80211 on netbsd-11 has no HE/EHT). */
struct wiphy_iftype_ext_capab {
	enum nl80211_iftype	iftype;
	const u8		*ext_capab;
	u8			ext_capab_len;
	const u8		*extended_capabilities;
	const u8		*extended_capabilities_mask;
	u8			extended_capabilities_len;
	u16			eml_capabilities;
	u16			mld_capa_and_ops;
};

/* radiotap shapes (only parsed by the driver for its own RX reports) */
struct ieee80211_radiotap_he {
	__le16			data1;
	__le16			data2;
	__le16			data3;
	__le16			data4;
	__le16			data5;
	__le16			data6;
};

struct ieee80211_radiotap_tlv {
	__le16			type;
	__le16			len;
	u8			data[];
};

struct ieee80211_radiotap_eht_usig {
	__le32			common;
	u8			data[];
};

struct ieee80211_radiotap_eht {
	__le32			known;
	__le32			data[8];
	__le32			user_info[];
};

#define	RX_FLAG_RADIOTAP_TLV_AT_END		BIT(30)
#define	IEEE80211_RADIOTAP_EHT			19
#define	IEEE80211_RADIOTAP_EHT_USIG		20
#define	IEEE80211_RADIOTAP_EHT_KNOWN_GI		BIT(0)
#define	IEEE80211_RADIOTAP_EHT_DATA0_GI		GENMASK(1, 0)
#define	IEEE80211_RADIOTAP_EHT_USER_INFO_MCS_KNOWN	0x0002
#define	IEEE80211_RADIOTAP_EHT_USER_INFO_NSS_KNOWN_O	0x0010
#define	IEEE80211_RADIOTAP_EHT_USER_INFO_CODING_KNOWN	0x0040
#define	IEEE80211_RADIOTAP_EHT_USER_INFO_CODING		0x4000
#define	IEEE80211_RADIOTAP_EHT_USER_INFO_MCS		GENMASK(11, 8)
#define	IEEE80211_RADIOTAP_EHT_USER_INFO_NSS_O		GENMASK(14, 12)
#define	IEEE80211_RADIOTAP_EHT_USIG_COMMON_BW_KNOWN	BIT(0)
#define	IEEE80211_RADIOTAP_EHT_USIG_COMMON_BW		GENMASK(5, 3)
#define	IEEE80211_RADIOTAP_HE_DATA1_DATA_MCS_KNOWN	BIT(2)
#define	IEEE80211_RADIOTAP_HE_DATA1_CODING_KNOWN	BIT(3)
#define	IEEE80211_RADIOTAP_HE_DATA1_STBC_KNOWN		BIT(5)
#define	IEEE80211_RADIOTAP_HE_DATA1_BW_RU_ALLOC_KNOWN	BIT(6)
#define	IEEE80211_RADIOTAP_HE_DATA2_GI_KNOWN		BIT(2)
#define	RX_ENC_HE			3
#define	RX_ENC_EHT			4
#define	RX_FLAG_RADIOTAP_HE		BIT(28)

#define	WLAN_EID_SSID			0
#define	WLAN_EID_SUPP_RATES		1
#define	WLAN_EID_DS_PARAMS		3
#define	WLAN_EID_COUNTRY		7
#define	WLAN_EID_VENDOR_SPECIFIC	221
#define	WLAN_EID_EXT_CAPABILITY		127
#define	WLAN_EXT_CAPA10_OBSS_NARROW_BW_RU_TOLERANCE_SUPPORT	BIT(5)
#define	WLAN_EXT_CAPA1_EXT_CHANNEL_SWITCHING			BIT(0)
#define	WLAN_EXT_CAPA3_MULTI_BSSID_SUPPORT			BIT(5)
#define	WLAN_EXT_CAPA8_OPMODE_NOTIF				BIT(6)
#define	IEEE80211_EML_CAP_EMLSR_PADDING_DELAY_32US		0
#define	IEEE80211_EML_CAP_EMLSR_PADDING_DELAY_64US		1
#define	IEEE80211_EML_CAP_EMLSR_PADDING_DELAY_128US		2
#define	IEEE80211_EML_CAP_EMLSR_PADDING_DELAY_256US		3
#define	IEEE80211_EML_CAP_EMLSR_TRANSITION_DELAY_32US		0
#define	IEEE80211_EML_CAP_EMLSR_TRANSITION_DELAY_64US		1
#define	IEEE80211_EML_CAP_EMLSR_TRANSITION_DELAY_128US		2
#define	IEEE80211_EML_CAP_EMLSR_TRANSITION_DELAY_256US		3
#define	WLAN_OUI_WFA			0x506f9a
#define	WLAN_OUI_TYPE_WFA_P2P		4
#define	IEEE80211_P2P_ATTR_ABSENCE_NOTICE	2

/* channel context (single shadow context; see rtw89_compat.c) */
struct ieee80211_chanctx_conf {
	struct cfg80211_chan_def def;
	u8			drv_priv[192] __aligned(sizeof(void *));
};

struct ieee80211_vif_chanctx_switch {
	u8			link_id;
	struct ieee80211_vif	*vif;
	struct ieee80211_bss_conf *link_conf;
	struct ieee80211_chanctx_conf *old_ctx;
	struct ieee80211_chanctx_conf *new_ctx;
};

struct ieee80211_ampdu_params {
	struct ieee80211_sta	*sta;
	enum ieee80211_agg_state action;
	u16			ssn;
	u8			tid;
	u16			buf_size;
	bool			amsdu;
	u16			timeout;
};

/* MAC header trigger shape (only the type is parsed) */
struct ieee80211_trigger {
	__le16			frame_control;
	__le16			duration;
	u8			ta[ETH_ALEN];
	u64			common_info;
	u8			variable[];
} __packed;

#define	IEEE80211_RADIOTAP_EHT_USIG_COMMON_BW_20MHZ	0
#define	IEEE80211_RADIOTAP_EHT_USIG_COMMON_BW_40MHZ	1
#define	IEEE80211_RADIOTAP_EHT_USIG_COMMON_BW_80MHZ	2
#define	IEEE80211_RADIOTAP_EHT_USIG_COMMON_BW_160MHZ	3
#define	IEEE80211_RADIOTAP_EHT_USIG_COMMON_BW_320MHZ_1	5
#define	IEEE80211_TRIGGER_TYPE_MASK		0xf
#define	IEEE80211_TRIGGER_TYPE_BFRP		0x1
#define	IEEE80211_TRIGGER_TYPE_MU_BAR		0x3
#define	IEEE80211_TRIGGER_TYPE_BASIC		0x4
#define	IEEE80211_TRIGGER_TYPE_BSRP		0x5
#define	IEEE80211_TRIGGER_GI_LTF_MASK		0x0c
#define	IEEE80211_TRIGGER_ULBW_MASK		0x70
#define	IEEE80211_TRIGGER_ULBW_20MHZ		0x00
#define	IEEE80211_TRIGGER_ULBW_40MHZ		0x10
#define	IEEE80211_TRIGGER_ULBW_80MHZ		0x20
#define	IEEE80211_TRIGGER_ULBW_160_80P80MHZ	0x30

/* v7.0 link accessors: the port is single-link (non-MLD), so an MLD is
 * never present and exactly one link (0) exists. */
static __always_inline __unused bool
ieee80211_vif_is_mld(struct ieee80211_vif *vif)
{
	(void)vif;
	return false;
}

static __always_inline __unused u16
ieee80211_vif_usable_links(struct ieee80211_vif *vif)
{
	(void)vif;
	return BIT(0);
}

static __always_inline __unused u32
ieee80211_tu_to_usec(size_t tu)
{
	return (u32)(tu * 1024);
}

/* MAC header helpers the rtw88 set did not need */
#define	IEEE80211_STYPE_ASSOC_REQ	0x0000
#define	IEEE80211_STYPE_PSPOLL		0x00a0
#define	IEEE80211_STYPE_TRIGGER		0x0020

static __always_inline __unused int
ieee80211_is_assoc_req(__le16 fc)
{
	return (le16toh(fc) & (IEEE80211_FCTL_FTYPE | IEEE80211_FCTL_STYPE)) ==
	    (IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_ASSOC_REQ);
}

static __always_inline __unused int
ieee80211_is_pspoll(__le16 fc)
{
	return (le16toh(fc) & (IEEE80211_FCTL_FTYPE | IEEE80211_FCTL_STYPE)) ==
	    (IEEE80211_FTYPE_CTL | IEEE80211_STYPE_PSPOLL);
}

static __always_inline __unused int
ieee80211_is_trigger(__le16 fc)
{
	return (le16toh(fc) & (IEEE80211_FCTL_FTYPE | IEEE80211_FCTL_STYPE)) ==
	    (IEEE80211_FTYPE_CTL | IEEE80211_STYPE_TRIGGER);
}

#define	IEEE80211_QOS_CTL			0x0000
#define	IEEE80211_QOS_TID	0x000f
#define	IEEE80211_QOS_CTL_LEN		2
#define	IEEE80211_QOS_CTL_EOSP		0x0100
#define	IEEE80211_QOS_CTL_TAG1D_MASK	0x0070
#define	IEEE80211_HT_CTL_LEN		4
#define	IEEE80211_FCTL_ORDER		0x8000
#define	IEEE80211_FCTL_PROTECTED	0x4000
#define	IEEE80211_STYPE_ACTION		0x00d0
#define	IEEE80211_TX_CTRL_PORT_CTRL_PROTO	BIT(3)
#define	WLAN_CATEGORY_SA_QUERY		8
#define	WLAN_ACTION_SA_QUERY_REQ	0
#define	WLAN_ACTION_SA_QUERY_RESPONSE	1
#define	ETH_P_ARP			0x0806
#define	ETH_P_IPV6			0x86dd

static __always_inline __unused bool
ieee80211_is_data_qos(__le16 fc)
{
	u16 v = le16toh(fc);

	return (v & (IEEE80211_FCTL_FTYPE | IEEE80211_FCTL_STYPE)) ==
	    (IEEE80211_FTYPE_DATA | 0x0080);
}

static __always_inline __unused bool
ieee80211_is_qos_nullfunc(__le16 fc)
{
	return (le16toh(fc) & (IEEE80211_FCTL_FTYPE | IEEE80211_FCTL_STYPE)) ==
	    (IEEE80211_FTYPE_DATA | IEEE80211_STYPE_QOS_NULLFUNC);
}

static __always_inline __unused bool
ieee80211_has_a4(__le16 fc)
{
	return (le16toh(fc) & IEEE80211_FCTL_TODS) != 0 &&
	    (le16toh(fc) & IEEE80211_FCTL_FROMDS) != 0;
}

static __always_inline __unused bool
ieee80211_has_pm(__le16 fc)
{
	return (le16toh(fc) & 0x1000) != 0;
}

static __always_inline __unused unsigned int
ieee80211_hdrlen(__le16 fc)
{
	unsigned int len = 24;

	if (ieee80211_has_a4(fc))
		len += 6;
	if (ieee80211_is_data_qos(fc))
		len += 2;
	return len;
}

static __always_inline __unused unsigned int
ieee80211_get_tid(struct ieee80211_hdr *hdr)
{
	struct {
		__le16	frame_control;
		__le16	duration;
		u8	addr1[6];
		u8	addr2[6];
		u8	addr3[6];
		__le16	seq_ctrl;
		__le16	qos_ctrl;
	} __packed *q = (void *)hdr;

	if (!ieee80211_is_data_qos(hdr->frame_control))
		return 0;
	return le16toh(q->qos_ctrl) & IEEE80211_QOS_TID;
}

static __always_inline __unused bool
ieee80211_hw_check(struct ieee80211_hw *hw, unsigned int flag)
{
	return test_bit(flag, &hw->flags);
}

/* TXQ plumbing: the port never drives mac80211 TXQs (net80211 feeds
 * frames through the driver directly), so these only keep the links
 * happy. */
void	ieee80211_txq_schedule_start(struct ieee80211_hw *, u8 ac);
void	ieee80211_txq_schedule_end(struct ieee80211_hw *, u8 ac);
struct sk_buff *ieee80211_tx_dequeue_ni(struct ieee80211_hw *,
	    struct ieee80211_txq *);
void	ieee80211_schedule_txq(struct ieee80211_hw *, struct ieee80211_txq *);
struct ieee80211_txq *ieee80211_next_txq(struct ieee80211_hw *, u8 ac);
void	ieee80211_return_txq(struct ieee80211_hw *, struct ieee80211_txq *,
	    bool);

/* RX entry: in this port rtw89_compat.c forwards the skb to the driver's
 * RX callback (if_rtw89.c) instead of to a mac80211 core. */
u32	ieee80211_rx_napi(struct ieee80211_hw *, struct ieee80211_sta *,
	    struct sk_buff *, struct napi_struct *);
#define	ieee80211_rx(hw, skb)	ieee80211_rx_napi((hw), NULL, (skb), NULL)

/* BA session plumbing: AMPDU is off in the port */
int	ieee80211_stop_tx_ba_session(struct ieee80211_sta *, u16);
void	ieee80211_stop_tx_ba_cb_irqsafe(struct ieee80211_vif *,
	    const u8 *, u16);

/* power-save hooks: the port keeps the firmware awake */
int	ieee80211_sta_ps_transition(struct ieee80211_sta *, bool);
void	ieee80211_sta_pspoll(struct ieee80211_sta *);
void	ieee80211_sta_uapsd_trigger(struct ieee80211_sta *, u8);

/* ROC/CSA: not driven by the port */
void	ieee80211_ready_on_channel(struct ieee80211_hw *);
void	ieee80211_remain_on_channel_expired(struct ieee80211_hw *);
bool	ieee80211_beacon_cntdwn_is_complete(struct ieee80211_vif *, u8);
void	ieee80211_csa_finish(struct ieee80211_vif *, u8);
void	ieee80211_beacon_loss(struct ieee80211_vif *);
void	ieee80211_stop(struct ieee80211_hw *);

/* cfg80211 IE parsing (real, used by the fw/scan paths) */
const struct element *cfg80211_find_elem(u8 eid, const u8 *ies, size_t len);
const u8 *cfg80211_find_ie(u8 eid, const u8 *ies, size_t len);
void	cfg80211_chandef_create(struct cfg80211_chan_def *,
	    struct ieee80211_channel *, enum nl80211_chan_width);
bool	cfg80211_channel_is_psc(struct ieee80211_channel *);
void	cfg80211_bss_iter(struct wiphy *, struct cfg80211_chan_def *,
	    void (*)(struct wiphy *, struct cfg80211_bss *, void *), void *);


/* cfg80211/mac80211 aux types the ops signatures need */
enum set_key_cmd {
	SET_KEY = 0,
	DISABLE_KEY,
};

struct survey_info {
	struct ieee80211_channel *channel;
	u8			filled;
	u8			noise;
	u64			time;
	u64			time_busy;
	u64			time_ext_busy;
	u16			frequency;
};

#define	NL80211_STA_INFO_INACTIVE_TIME	1
#define	NL80211_STA_INFO_RX_BYTES	2
#define	NL80211_STA_INFO_TX_BYTES	3
#define	NL80211_STA_INFO_SIGNAL		7
#define	NL80211_STA_INFO_TX_BITRATE	8
#define	NL80211_STA_INFO_RX_PACKETS	9
#define	NL80211_STA_INFO_TX_PACKETS	10
#define	NL80211_STA_INFO_TX_RETRIES	11
#define	NL80211_STA_INFO_TX_FAILED	12
#define	NL80211_STA_INFO_RX_BITRATE	14
#define	NL80211_STA_INFO_RX_BYTES64	22
#define	NL80211_STA_INFO_TX_BYTES64	23
#define	IEEE80211_RC_SUPP_RATES_CHANGED	BIT(2)
#define	IEEE80211_RC_BW_CHANGED		BIT(3)
#define	IEEE80211_RC_NSS_CHANGED	BIT(4)

#define	SURVEY_INFO_NOISE_DBM		BIT(0)
#define	SURVEY_INFO_TIME		BIT(1)
#define	SURVEY_INFO_TIME_BUSY		BIT(2)

struct station_info {
	u32			filled;
	struct rate_info	txrate;
	struct rate_info	rxrate;
	u8			connected_time;
	u8			inactive_time;
	s8			signal;
	u8			chains;
	s8			chain_signal[IEEE80211_MAX_CHAINS];
	u64			tx_bytes;
	u64			rx_bytes;
	u32			tx_packets;
	u32			rx_packets;
	u32			tx_failed;
	u32			tx_retries;
};

struct ieee80211_chanctx_ctx {
	struct ieee80211_chanctx_conf conf;
	u8			drv_priv[32] __aligned(sizeof(void *));
};

/*
 * mac80211 ops: the exact surface the imported mac80211.c initializer
 * (struct ieee80211_ops rtw89_ops) names.  Signatures follow the v7.0
 * kernel the snapshot was taken from.
 */
struct cfg80211_tid_config;
struct cfg80211_gtk_rekey_data;
struct cfg80211_wowlan;
struct survey_info;
struct station_info;
struct ieee80211_chanctx_conf;
struct ieee80211_chanctx_ctx;
struct ieee80211_vif_chanctx_switch;
struct ieee80211_link_sta;
struct ieee80211_ops {
	void	(*tx)(struct ieee80211_hw *,
		    struct ieee80211_tx_control *, struct sk_buff *);
	void	(*wake_tx_queue)(struct ieee80211_hw *,
		    struct ieee80211_txq *);
	int	(*start)(struct ieee80211_hw *);
	void	(*stop)(struct ieee80211_hw *, bool suspend);
	int	(*config)(struct ieee80211_hw *, int radio_idx, u32 changed);
	int	(*add_interface)(struct ieee80211_hw *,
		    struct ieee80211_vif *);
	int	(*change_interface)(struct ieee80211_hw *,
		    struct ieee80211_vif *, enum nl80211_iftype, bool p2p);
	void	(*remove_interface)(struct ieee80211_hw *,
		    struct ieee80211_vif *);
	void	(*configure_filter)(struct ieee80211_hw *, unsigned int,
		    unsigned int *, u64 multicast);
	void	(*vif_cfg_changed)(struct ieee80211_hw *,
		    struct ieee80211_vif *, u64 changed);
	void	(*link_info_changed)(struct ieee80211_hw *,
		    struct ieee80211_vif *, struct ieee80211_bss_conf *,
		    u64 changed);
	int	(*start_ap)(struct ieee80211_hw *, struct ieee80211_vif *,
		    struct ieee80211_bss_conf *);
	void	(*stop_ap)(struct ieee80211_hw *, struct ieee80211_vif *,
		    struct ieee80211_bss_conf *);
	int	(*set_tim)(struct ieee80211_hw *, struct ieee80211_sta *,
		    bool set);
	int	(*conf_tx)(struct ieee80211_hw *, struct ieee80211_vif *,
		    unsigned int link_id, u16 ac,
		    const struct ieee80211_tx_queue_params *);
	int	(*sta_state)(struct ieee80211_hw *, struct ieee80211_vif *,
		    struct ieee80211_sta *, enum ieee80211_sta_state,
		    enum ieee80211_sta_state);
	int	(*set_key)(struct ieee80211_hw *, enum set_key_cmd,
		    struct ieee80211_vif *, struct ieee80211_sta *,
		    struct ieee80211_key_conf *);
	int	(*ampdu_action)(struct ieee80211_hw *,
		    struct ieee80211_vif *, struct ieee80211_ampdu_params *);
	int	(*get_survey)(struct ieee80211_hw *, int idx,
		    struct survey_info *);
	int	(*set_rts_threshold)(struct ieee80211_hw *, int radio_idx,
		    u32 value);
	void	(*sta_statistics)(struct ieee80211_hw *,
		    struct ieee80211_vif *, struct ieee80211_sta *,
		    struct station_info *);
	void	(*flush)(struct ieee80211_hw *, struct ieee80211_vif *,
		    u32 queues, bool drop);
	int	(*set_bitrate_mask)(struct ieee80211_hw *,
		    struct ieee80211_vif *,
		    const struct cfg80211_bitrate_mask *);
	int	(*set_antenna)(struct ieee80211_hw *, int radio_idx, u32,
		    u32);
	int	(*get_antenna)(struct ieee80211_hw *, int radio_idx, u32 *,
		    u32 *);
	void	(*sw_scan_start)(struct ieee80211_hw *,
		    struct ieee80211_vif *, const u8 *mac_addr);
	void	(*sw_scan_complete)(struct ieee80211_hw *,
		    struct ieee80211_vif *);
	void	(*reconfig_complete)(struct ieee80211_hw *,
		    enum ieee80211_reconfig_type);
	int	(*hw_scan)(struct ieee80211_hw *, struct ieee80211_vif *,
		    struct ieee80211_scan_request *);
	void	(*cancel_hw_scan)(struct ieee80211_hw *,
		    struct ieee80211_vif *);
	int	(*add_chanctx)(struct ieee80211_hw *,
		    struct ieee80211_chanctx_conf *);
	void	(*remove_chanctx)(struct ieee80211_hw *,
		    struct ieee80211_chanctx_conf *);
	void	(*change_chanctx)(struct ieee80211_hw *,
		    struct ieee80211_chanctx_conf *, u32 changed);
	int	(*assign_vif_chanctx)(struct ieee80211_hw *,
		    struct ieee80211_vif *, struct ieee80211_bss_conf *,
		    struct ieee80211_chanctx_conf *);
	void	(*unassign_vif_chanctx)(struct ieee80211_hw *,
		    struct ieee80211_vif *, struct ieee80211_bss_conf *,
		    struct ieee80211_chanctx_conf *);
	int	(*switch_vif_chanctx)(struct ieee80211_hw *,
		    struct ieee80211_vif_chanctx_switch *, int n_vifs,
		    enum ieee80211_chanctx_switch_mode);
	void	(*channel_switch_beacon)(struct ieee80211_hw *,
		    struct ieee80211_vif *, struct cfg80211_chan_def *);
	int	(*remain_on_channel)(struct ieee80211_hw *,
		    struct ieee80211_vif *, struct ieee80211_channel *,
		    int duration, enum ieee80211_roc_type);
	int	(*cancel_remain_on_channel)(struct ieee80211_hw *,
		    struct ieee80211_vif *);
	int	(*set_sar_specs)(struct ieee80211_hw *,
		    const struct cfg80211_sar_specs *);
	void	(*link_sta_rc_update)(struct ieee80211_hw *,
		    struct ieee80211_vif *, struct ieee80211_link_sta *,
		    u32 changed);
	int	(*set_tid_config)(struct ieee80211_hw *,
		    struct ieee80211_vif *, struct ieee80211_sta *,
		    struct cfg80211_tid_config *);
	bool	(*can_activate_links)(struct ieee80211_hw *,
		    struct ieee80211_vif *, u16 active_links);
	int	(*change_vif_links)(struct ieee80211_hw *,
		    struct ieee80211_vif *, u16 old_links, u16 new_links,
		    struct ieee80211_bss_conf *old[]);
	int	(*change_sta_links)(struct ieee80211_hw *,
		    struct ieee80211_vif *, struct ieee80211_sta *,
		    u16 old_links, u16 new_links);
	int	(*suspend)(struct ieee80211_hw *,
		    struct cfg80211_wowlan *);
	int	(*resume)(struct ieee80211_hw *);
	void	(*set_wakeup)(struct ieee80211_hw *, bool enabled);
	void	(*set_rekey_data)(struct ieee80211_hw *,
		    struct ieee80211_vif *,
		    struct cfg80211_gtk_rekey_data *);
	void	(*rfkill_poll)(struct ieee80211_hw *);
};

/* mac80211 emulate helpers core.c assigns when the chip runs chanctx-less */
int	ieee80211_emulate_add_chanctx(struct ieee80211_hw *,
	    struct ieee80211_chanctx_conf *);
void	ieee80211_emulate_remove_chanctx(struct ieee80211_hw *,
	    struct ieee80211_chanctx_conf *);
void	ieee80211_emulate_change_chanctx(struct ieee80211_hw *,
	    struct ieee80211_chanctx_conf *, u32);
int	ieee80211_emulate_switch_vif_chanctx(struct ieee80211_hw *,
	    struct ieee80211_vif_chanctx_switch *, int,
	    enum ieee80211_chanctx_switch_mode);

#endif /* _RTW89_MAC80211_H_ */
