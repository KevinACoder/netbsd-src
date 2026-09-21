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

#ifndef _DEV_USB_AIC8800MSG_H_
#define _DEV_USB_AIC8800MSG_H_

#include <sys/types.h>
#include <sys/cdefs.h>		/* offsetof */

/*
 * AIC8800D80 lmac_msg protocol facts, transcribed from the vendor SDK
 * (os/aic8800) per the AIC8800D80 ground truth.  These are interface
 * constants (message ids, struct layouts, RAM addresses), not vendor
 * source; the implementation in aic8800_chip.c / aic8800_usb.c is native.
 *
 * Ground truth: doc/rk3568-itx-wiki/runs/20260913-linux-aic8800-usb/
 * (AIC8800D80-GROUND-TRUTH.md §3-§4).
 */

/* ---- lmac_msg task ids (lmac_msg.h) --------------------------------- */

#define AIC8800_TASK_MM			0
#define AIC8800_TASK_DBG		1
#define AIC8800_TASK_SCANU		4
#define AIC8800_TASK_ME			5
#define AIC8800_TASK_SM			6
#define AIC8800_DRV_TASK_ID		100

#define AIC8800_LMAC_FIRST_MSG(task)	((task) << 10)

/*
 * DBG task message ids: the DBG block is the second enum member of the
 * dbg_msg_tag list, so DBG_MEM_READ_REQ = LMAC_FIRST_MSG(TASK_DBG) and
 * the rest follow consecutively.  Every REQ has a CFM at id + 1.
 */
#define AIC8800_DBG_MEM_READ_REQ	0x400
#define AIC8800_DBG_MEM_READ_CFM	0x401
#define AIC8800_DBG_MEM_WRITE_REQ	0x402
#define AIC8800_DBG_MEM_WRITE_CFM	0x403
#define AIC8800_DBG_MEM_BLOCK_WRITE_REQ	0x40b
#define AIC8800_DBG_MEM_BLOCK_WRITE_CFM	0x40c
#define AIC8800_DBG_START_APP_REQ	0x40d
#define AIC8800_DBG_START_APP_CFM	0x40e
#define AIC8800_DBG_MEM_MASK_WRITE_REQ	0x411
#define AIC8800_DBG_MEM_MASK_WRITE_CFM	0x412

#define AIC8800_HOST_START_APP_AUTO	1

/* ---- param struct layouts (dbg_mem_*) ------------------------------- */

#define AIC8800_MEM_READ_REQ_LEN	4	/* { memaddr } */
#define AIC8800_MEM_READ_CFM_LEN	8	/* { memaddr, memdata } */
#define AIC8800_MEM_WRITE_REQ_LEN	8	/* { memaddr, memdata } */
#define AIC8800_MEM_WRITE_CFM_LEN	8	/* { memaddr, memdata } */
#define AIC8800_MEM_MASK_WRITE_REQ_LEN	12	/* { memaddr, memmask, memdata } */
#define AIC8800_MEM_BLOCK_WRITE_HDR	8	/* { memaddr, memsize } */
#define AIC8800_MEM_BLOCK_WRITE_DATA	1024	/* inline payload, zero padded */
#define AIC8800_MEM_BLOCK_WRITE_REQ_LEN	(AIC8800_MEM_BLOCK_WRITE_HDR + \
					 AIC8800_MEM_BLOCK_WRITE_DATA)
#define AIC8800_MEM_BLOCK_WRITE_CFM_LEN	4	/* { wstatus } */
#define AIC8800_START_APP_REQ_LEN	8	/* { bootaddr, boottype } */
#define AIC8800_START_APP_CFM_LEN	4	/* { bootstatus } */

/* ---- USB frame framing (aicwf_usb.h / aicbluetooth_cmds.c) ---------- */

#define AIC8800_USB_TYPE_DATA		0x00
#define AIC8800_USB_TYPE_CFG		0x10
#define AIC8800_USB_TYPE_CFG_CMD_RSP	0x11
#define AIC8800_USB_TYPE_CFG_DATA_CFM	0x12
#define AIC8800_USB_TYPE_CFG_PRINT	0x13

/*
 * TX (host -> device) command frame:
 *   [0..1] 12-bit length = lmac_msg bytes (8 + param_len) + 4
 *   [2]    frame type 0x11
 *   [3]    0x00
 *   [4..7] dummy word (zeros)
 *   [8..15] lmac_msg { id, dest_id, src_id, param_len } (u16 LE each)
 *   [16..] param
 * total on the wire = param_len + 16.
 *
 * RX (device -> host) event/CFM frame -- same length field, but the msg
 * header follows the 4-byte frame header directly and the dummy word is
 * carried as the e2a "pattern" field between the header and the param:
 *   [0..1] 12-bit length, [2] type, [3] 0x00
 *   [4..5] id, [6..7] dest, [8..9] src, [10..11] param_len
 *   [12..15] pattern (ignored)
 *   [16..] param
 */
#define AIC8800_TX_FRAME_MAX	(16 + AIC8800_MEM_BLOCK_WRITE_REQ_LEN)
#define AIC8800_RX_BUF_MAX	2048	/* vendor AICWF_USB_MAX_PKT_SIZE */

/* The vendor command manager waits 2 s per queued command. */
#define AIC8800_CMD_TIMEOUT_MS	2000

/* ---- firmware patch table (aicbluetooth.c) -------------------------- */

/* 16-byte tag at the head of fw_patch_table_*.bin */
#define AIC8800_PT_TAG		"AICBT_PT_TAG"

#define AIC8800_PT_INF		0x0	/* patch_info pairs */
#define AIC8800_PT_TRAP		0x01
#define AIC8800_PT_B4		0x02
#define AIC8800_PT_BTMODE	0x03
#define AIC8800_PT_PWRON	0x04
#define AIC8800_PT_AF		0x05
#define AIC8800_PT_VERSION	0x06	/* string, skipped */

#define AIC8800_PT_ENTRY_MAX	1000	/* type >= this or len == 0: skip */

/*
 * Table body: repeated [16B name][u32 type][u32 len][len * {u32 addr,
 * u32 val} pairs]; entries with type >= 1000 or len == 0 carry no data.
 */

/* patch_info pairs (INF entries), u32 LE each.  ext_patch_param points
 * into the INF entry's data buffer (beyond the pairs mapped below) and
 * stays valid until the parsed table list is freed. */
struct aic8800u_patch_info {
	uint32_t	info_len;
	uint32_t	adid_addrinf;
	uint32_t	addr_adid;
	uint32_t	patch_addrinf;
	uint32_t	addr_patch;
	uint32_t	reset_addr;
	uint32_t	reset_val;
	uint32_t	adid_flag_addr;
	uint32_t	adid_flag;
	uint32_t	ext_patch_nb_addr;
	uint32_t	ext_patch_nb;
	const uint32_t	*ext_patch_param; /* {id, addr} pairs, 2 per ext */
};

#define AIC8800_PATCH_MAGIC_NUM		0x48435450	/* "PTCH" */
#define AIC8800_PATCH_MAGIC_NUM_2	0x50544348	/* "HCTP" */
#define AIC8800_PATCH_BLOCK_MAX		4

/* aic_patch_t field offsets (from aic_patch_str_base) */
#define AIC8800_PATCH_OFF_MAGIC_NUM	0
#define AIC8800_PATCH_OFF_PAIR_START	4
#define AIC8800_PATCH_OFF_MAGIC_NUM_2	8
#define AIC8800_PATCH_OFF_PAIR_COUNT	12
#define AIC8800_PATCH_OFF_BLOCK_DST	16
#define AIC8800_PATCH_OFF_BLOCK_SRC	32
#define AIC8800_PATCH_OFF_BLOCK_SIZE	48

/* ---- D80 firmware RAM addresses (aic_compat_8800d80.h) -------------- */

/* U02 silicon (the ground-truth dongle).  U01 addresses differ and the
 * U01 firmware files are not packaged; the loader refuses U01. */
#define AIC8800_RAM_FW_ADDR_U02		0x00120000
#define AIC8800_RAM_ADID_ADDR_U02	0x00201940
#define AIC8800_RAM_PATCH_ADDR_U02	0x0020b43c

/* chip revision register (system_config_8800d80) */
#define AIC8800_SYS_CHIPID_REG		0x40500000
#define AIC8800_CHIP_REV_U01		0x1
#define AIC8800_CHIP_REV_U02		0x3
#define AIC8800_CHIP_REV_U03		0x7

/* patch_config: firmware RAM probes relative to RAM_FW_ADDR_U02 */
#define AIC8800_FW_PATCH_CONFIG_BASE	0x0198
#define AIC8800_FW_PATCH_STR_BASE	0x01a0
#define AIC8800_FW_VERSION_OFF		0x001c
#define AIC8800_FW_PATCH_BUFF_OFF	0x01a4
#define AIC8800_FW_PATCH_BUFF_MIN_VER	0x06090100
#define AIC8800_PATCH_START_ADDR	0x001d7000

/* firmware file names (vendor, non-IPC u02 variant) */
#define AIC8800_FW_FMACFW	"fmacfw_8800d80_u02.bin"
#define AIC8800_FW_ADID		"fw_adid_8800d80_u02.bin"
#define AIC8800_FW_PATCH	"fw_patch_8800d80_u02.bin"
#define AIC8800_FW_PATCH_EXT	"fw_patch_8800d80_u02_ext"
#define AIC8800_FW_PATCH_TABLE	"fw_patch_table_8800d80_u02.bin"

/*
 * patch_tbl_d80 as resolved for the ground-truth build (USE_5G,
 * CONFIG_POWER_LIMIT, CONFIG_RADAR_OR_IR_DETECT, CONFIG_WOWLAN and
 * CONFIG_PMIC_SETTING all undefined in the vendor Makefile): the pair
 * addresses are firmware-relative (patch_config adds config_base).
 */
#define AIC8800_PATCH_TBL_COUNT	3
extern const uint32_t aic8800u_patch_tbl[AIC8800_PATCH_TBL_COUNT][2];

/* ---- fullmac command ids (lmac_msg.h enums; task<<10 | index) -------- */

/* MM task */
#define AIC8800_MM_RESET_REQ		0x000
#define AIC8800_MM_RESET_CFM		0x001
#define AIC8800_MM_START_REQ		0x002
#define AIC8800_MM_START_CFM		0x003
#define AIC8800_MM_ADD_IF_REQ		0x006
#define AIC8800_MM_ADD_IF_CFM		0x007
#define AIC8800_MM_KEY_ADD_REQ		0x024
#define AIC8800_MM_KEY_ADD_CFM		0x025
#define AIC8800_MM_KEY_DEL_REQ		0x026
#define AIC8800_MM_KEY_DEL_CFM		0x027
#define AIC8800_MM_GET_MAC_ADDR_REQ	0x073	/* cfm: 6-byte mac_addr */
#define AIC8800_MM_GET_MAC_ADDR_CFM	0x074

/* SCANU task (firmware-managed scan) */
#define AIC8800_SCANU_START_REQ		0x1000
#define AIC8800_SCANU_START_CFM		0x1001	/* async: scan finished */
#define AIC8800_SCANU_RESULT_IND	0x1004	/* one per BSS */
/* the START_REQ command CFM is the ADDTIONAL id, not SCANU_START_CFM */
#define AIC8800_SCANU_START_CFM_ADDTIONAL 0x1009
#define AIC8800_SCANU_CANCEL_REQ	0x100a
#define AIC8800_SCANU_CANCEL_CFM	0x100b

/* ME task */
#define AIC8800_ME_CONFIG_REQ		0x1400
#define AIC8800_ME_CONFIG_CFM		0x1401
#define AIC8800_ME_CHAN_CONFIG_REQ	0x1402
#define AIC8800_ME_CHAN_CONFIG_CFM	0x1403
#define AIC8800_ME_SET_CONTROL_PORT_REQ	0x1404
#define AIC8800_ME_SET_CONTROL_PORT_CFM	0x1405

/* SM task (firmware connection manager) */
#define AIC8800_SM_CONNECT_REQ		0x1800
#define AIC8800_SM_CONNECT_CFM		0x1801
#define AIC8800_SM_CONNECT_IND		0x1802
#define AIC8800_SM_DISCONNECT_REQ	0x1803
#define AIC8800_SM_DISCONNECT_CFM	0x1804
#define AIC8800_SM_DISCONNECT_IND	0x1805

/* ---- shared tag types (lmac_mac.h) ---------------------------------- */
/*
 * The vendor message structs are NOT packed; every struct below mirrors
 * the member order of the vendor definition, so the compiler reproduces
 * the vendor's natural-alignment layout byte for byte (verified against
 * a probe compiled with the vendor headers).  A CTASSERT pins each size.
 * Multi-byte fields are little-endian on the wire.
 */

struct aic8800u_mac_addr {
	uint16_t	a[3];		/* vendor: u16_l array[3], align 2 */
};

struct aic8800u_mac_ssid {
	uint8_t		length;
	uint8_t		array[32];
};

struct aic8800u_mac_chan_def {
	uint16_t	freq;		/* MHz */
	uint8_t		band;		/* 0 = 2.4G, 1 = 5G */
	uint8_t		flags;		/* CHAN_* */
	int8_t		tx_power;	/* dBm */
};

#define AIC8800_MAC_SEC_KEY_LEN	32
struct aic8800u_mac_sec_key {
	uint8_t		length;
	uint32_t	array[AIC8800_MAC_SEC_KEY_LEN / 4];
};

#define AIC8800_PHY_CFG_BUF_SIZE	16
struct aic8800u_phy_cfg_tag {
	uint32_t	parameters[AIC8800_PHY_CFG_BUF_SIZE];
};

/* capability tags, verbatim member order from lmac_mac.h */
#define AIC8800_MAX_MCS_LEN		16
struct aic8800u_mac_htcapability {
	uint16_t	ht_capa_info;
	uint8_t		a_mpdu_param;
	uint8_t		mcs_rate[AIC8800_MAX_MCS_LEN];
	uint16_t	ht_extended_capa;
	uint32_t	tx_beamforming_capa;
	uint8_t		asel_capa;
};

struct aic8800u_mac_vhtcapability {
	uint32_t	vht_capa_info;
	uint16_t	rx_mcs_map;
	uint16_t	rx_highest;
	uint16_t	tx_mcs_map;
	uint16_t	tx_highest;
};

#define AIC8800_HE_MAC_CAPA_LEN	6
#define AIC8800_HE_PHY_CAPA_LEN	11
#define AIC8800_HE_PPE_THRES_MAX_LEN	25
struct aic8800u_mac_hecapability {
	uint8_t		mac_cap_info[AIC8800_HE_MAC_CAPA_LEN];
	uint8_t		phy_cap_info[AIC8800_HE_PHY_CAPA_LEN];
	struct {
		uint16_t	rx_mcs_80;
		uint16_t	tx_mcs_80;
		uint16_t	rx_mcs_160;
		uint16_t	tx_mcs_160;
		uint16_t	rx_mcs_80p80;
		uint16_t	tx_mcs_80p80;
	} mcs_supp;
	uint8_t		ppe_thres[AIC8800_HE_PPE_THRES_MAX_LEN];
};

#define AIC8800_SCAN_SSID_MAX		3
#define AIC8800_SCAN_CHANNEL_MAX	(14 + 28)
#define AIC8800_SM_ASSOC_IE_LEN		800
#define AIC8800_AC_MAX			4	/* BK, BE, VI, VO */

#define AIC8800_MAC_BAND_2G4		0
#define AIC8800_MAC_BAND_5G		1
#define AIC8800_PHY_BW_20		0

/* mac_connection_flags */
#define AIC8800_CONNECT_CONTROL_PORT_HOST (1u << 0)
#define AIC8800_CONNECT_CONTROL_PORT_NO_ENC (1u << 1)
#define AIC8800_CONNECT_DISABLE_HT	(1u << 2)
#define AIC8800_CONNECT_WPA_WPA2_IN_USE	(1u << 3)

/* mac_cipher_suite */
#define AIC8800_CIPHER_CCMP		2

/* ---- command param structs ------------------------------------------ */

struct aic8800u_mm_start_req {
	struct aic8800u_phy_cfg_tag phy_cfg;
	uint32_t	uapsd_timeout;
	uint16_t	lp_clk_accuracy;
};

struct aic8800u_mm_add_if_req {
	uint8_t		type;		/* 0 = STA */
	struct aic8800u_mac_addr addr;
	uint8_t		p2p;
};

struct aic8800u_mm_add_if_cfm {
	uint8_t		status;
	uint8_t		inst_nbr;
	uint16_t	pad;
};

struct aic8800u_mm_key_add_req {
	uint8_t		key_idx;	/* group keys only */
	uint8_t		sta_idx;	/* pairwise: AP sta idx; group: 0xFF */
	struct aic8800u_mac_sec_key key;
	uint8_t		cipher_suite;
	uint8_t		inst_nbr;
	uint8_t		spp;
	uint8_t		pairwise;
};

struct aic8800u_mm_key_add_cfm {
	uint8_t		status;
	uint8_t		hw_key_idx;
};

struct aic8800u_me_config_req {
	struct aic8800u_mac_htcapability ht_cap;
	struct aic8800u_mac_vhtcapability vht_cap;
	struct aic8800u_mac_hecapability he_cap;
	uint16_t	tx_lft;
	uint8_t		phy_bw_max;	/* PHY_CHNL_BW_* */
	uint8_t		ht_supp;
	uint8_t		vht_supp;
	uint8_t		he_supp;
	uint8_t		he_ul_on;
	uint8_t		ps_on;		/* power save master switch */
	uint8_t		ant_div_on;
	uint8_t		dpsm;
};

struct aic8800u_me_chan_config_req {
	struct aic8800u_mac_chan_def chan2G4[14];
	struct aic8800u_mac_chan_def chan5G[28];
	uint8_t		chan2G4_cnt;
	uint8_t		chan5G_cnt;
};

struct aic8800u_me_set_control_port_req {
	uint8_t		sta_idx;
	uint8_t		control_port_open;
};

struct aic8800u_scanu_start_req {
	struct aic8800u_mac_chan_def chan[AIC8800_SCAN_CHANNEL_MAX];
	struct aic8800u_mac_ssid ssid[AIC8800_SCAN_SSID_MAX];
	struct aic8800u_mac_addr bssid;	/* all ones = wildcard */
	uint32_t	add_ies;	/* host memory address, unused on USB */
	uint16_t	add_ie_len;
	uint8_t		vif_idx;
	uint8_t		chan_cnt;
	uint8_t		ssid_cnt;
	uint8_t		no_cck;
	uint32_t	duration;	/* us, 0 = firmware default dwell */
};

/* scanu_start_cfm: { vif_idx, status, result_cnt } = 3 bytes */

struct aic8800u_scanu_result_ind {
	uint16_t	length;		/* mgmt frame bytes (no FCS) */
	uint16_t	framectrl;
	uint16_t	center_freq;	/* MHz */
	uint8_t		band;
	uint8_t		sta_idx;	/* 0xFF if unknown */
	uint8_t		inst_nbr;	/* 0xFF if unknown */
	int8_t		rssi;		/* dBm */
	uint32_t	payload[];	/* the complete beacon/probe-rsp frame */
};

struct aic8800u_sm_connect_req {
	struct aic8800u_mac_ssid ssid;
	struct aic8800u_mac_addr bssid;
	struct aic8800u_mac_chan_def chan;	/* freq = (uint16_t)-1 if unknown */
	uint32_t	flags;		/* AIC8800_CONNECT_* */
	uint16_t	ctrl_port_ethertype;	/* host order 0x888e */
	uint16_t	ie_len;
	uint16_t	listen_interval;
	uint8_t		dont_wait_bcmc;
	uint8_t		auth_type;	/* 0 = open */
	uint8_t		uapsd_queues;
	uint8_t		vif_idx;
	uint32_t	ie_buf[64];
};

/* sm_connect_cfm: { status } = 1 byte; 0 = procedure started */

struct aic8800u_sm_connect_ind {
	uint16_t	status_code;	/* WLAN status; 0 = connected */
	struct aic8800u_mac_addr bssid;
	uint8_t		roamed;
	uint8_t		vif_idx;
	uint8_t		ap_idx;		/* firmware STA entry for the AP */
	uint8_t		ch_idx;
	uint8_t		qos;
	uint8_t		acm;
	uint16_t	assoc_req_ie_len;
	uint16_t	assoc_rsp_ie_len;
	uint32_t	assoc_ie_buf[AIC8800_SM_ASSOC_IE_LEN / 4];
	uint16_t	aid;
	uint8_t		band;
	uint16_t	center_freq;
	uint8_t		width;
	uint32_t	center_freq1;
	uint32_t	center_freq2;
	uint32_t	ac_param[AIC8800_AC_MAX];
};

struct aic8800u_sm_disconnect_req {
	uint16_t	reason_code;
	uint8_t		vif_idx;
};

/* ---- data plane (ipc_shared.h hostdesc + aicwf_usb.c framing) ------- */

struct aic8800u_hostdesc {
	uint16_t	packet_len;	/* ethernet frame bytes */
	uint16_t	flags_ext;
	uint32_t	status_desc_addr;	/* need_cfm: (1<<31)|slot */
	uint8_t		eth_dest_addr[6];
	uint8_t		eth_src_addr[6];
	uint16_t	ethertype;
	uint8_t		ac;		/* 0=BK 1=BE 2=VI 3=VO */
	uint8_t		tid;		/* 0xFF if not QoS */
	uint8_t		vif_idx;
	uint8_t		staid;		/* 0xFF if unknown */
	uint16_t	flags;		/* TXU_CNTRL_* : mgmt = BIT(3) */
};

#define AIC8800_TXU_CNTRL_MGMT		(1u << 3)

/* TX data frame (data OUT EP):
 *   [0..1] total length INCLUDING this 4-byte header
 *   [2]    0x01 (data)
 *   [3]    0x00
 *   [4..31] hostdesc      [32..] ethernet frame (raw 802.11 for mgmt)
 *   zero-padded to 4; a frame whose total length is a multiple of 512
 *   gets one extra zero byte (short-packet boundary). */
#define AIC8800_DATA_TX_BUF_MAX	2048

/* RX data frame (data IN EP): one packet per transfer.
 *   [0..1] 802.11 MPDU length (excluding the 60B hardware header)
 *   [2]    flags byte (msg frames have bit 4 set)
 *   [4..59] hw_rxhdr (56 bytes) + 4 pad/word-align
 *   [60..] 802.11 MPDU
 * status word u32 @36: decr_status = bit2..4, fcs_err = bit 8. */
#define AIC8800_RX_MPDU_OFF		60
#define AIC8800_RX_STATUS_OFF		36
#define AIC8800_RX_DECR_STATUS(w)	(((w) >> 2) & 7)
#define AIC8800_RX_FCS_ERR(w)		(((w) >> 8) & 1)
/* decr_status values */
#define AIC8800_DECR_UNENC		0
#define AIC8800_DECR_CCMP128		3
/* signed RSSI in dBm; primary at hw_rxhdr+17, fallback at +14 */
#define AIC8800_RX_RSSI1_OFF		17
#define AIC8800_RX_RSSI_LEG_OFF		14

_Static_assert(sizeof(struct aic8800u_mac_chan_def) == 6, "chan_def");
_Static_assert(sizeof(struct aic8800u_mac_sec_key) == 36, "sec_key");
_Static_assert(sizeof(struct aic8800u_mm_start_req) == 72, "mm_start");
_Static_assert(sizeof(struct aic8800u_mm_add_if_req) == 10, "mm_add_if");
_Static_assert(sizeof(struct aic8800u_mm_key_add_req) == 44, "mm_key_add");
_Static_assert(sizeof(struct aic8800u_me_config_req) == 112, "me_config");
_Static_assert(sizeof(struct aic8800u_me_chan_config_req) == 254, "me_chan");
_Static_assert(sizeof(struct aic8800u_scanu_start_req) == 376, "scanu");
_Static_assert(sizeof(struct aic8800u_scanu_result_ind) == 12, "scanu_ind");
_Static_assert(sizeof(struct aic8800u_sm_connect_req) == 320, "sm_connect");
_Static_assert(sizeof(struct aic8800u_sm_connect_ind) == 852, "sm_connect_ind");
_Static_assert(sizeof(struct aic8800u_sm_disconnect_req) == 4, "sm_disconnect");
_Static_assert(sizeof(struct aic8800u_hostdesc) == 28, "hostdesc");
_Static_assert(offsetof(struct aic8800u_sm_connect_ind, assoc_ie_buf) == 20,
    "sm_connect_ind ie");
_Static_assert(offsetof(struct aic8800u_sm_connect_ind, aid) == 820, "aid");
_Static_assert(offsetof(struct aic8800u_scanu_start_req, bssid) == 352,
    "scanu bssid");

#endif	/* _DEV_USB_AIC8800MSG_H_ */
