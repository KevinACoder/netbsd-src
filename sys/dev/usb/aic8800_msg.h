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

#endif	/* _DEV_USB_AIC8800MSG_H_ */
