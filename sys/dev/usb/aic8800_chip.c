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
 * AIC8800D80 chip layer -- firmware download state machine.
 *
 * The download sequence (and every address, file name and patch table)
 * follows the vendor loader's U02 path (aic_load_fw/aic_compat_8800d80.c,
 * FW_NORMAL_MODE) as exercised by the Linux ground truth:
 *
 *   read 0x40500000 -> chip revision
 *   parse fw_patch_table_8800d80_u02.bin (AICBT_PT_TAG format)
 *   upload fw_adid_8800d80_u02.bin      -> addr_adid  (0x00201940)
 *   upload fw_patch_8800d80_u02.bin     -> addr_patch (0x0020b43c)
 *   upload fw_patch_8800d80_u02_extN.bin-> ext_patch_param addrs
 *   write the table entries             (DBG_MEM_WRITE, 100 ms after PWRON)
 *   upload fmacfw_8800d80_u02.bin       -> 0x00120000
 *   patch_config (reads config_base / patch_str_base / fw version from
 *                 firmware RAM, then writes the aic_patch_t structure)
 *   DBG_START_APP_REQ(0x00120000, AUTO) -> device re-enumerates as app
 *
 * The lmac_msg protocol facts live in aic8800_msg.h; this file is a
 * native implementation of the sequence, not a translation of vendor
 * source.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kmem.h>
#include <sys/kernel.h>
#include <sys/proc.h>
#include <sys/device.h>

#include <dev/firmload.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include "aic8800var.h"

#define AIC8800_FWDIR	"aic8800u"

/*
 * patch_tbl_d80 as resolved for the ground-truth build (USE_5G,
 * CONFIG_POWER_LIMIT, CONFIG_RADAR_OR_IR_DETECT, CONFIG_WOWLAN and
 * CONFIG_PMIC_SETTING all undefined in the vendor loader Makefile).
 * Addresses are firmware-relative; patch_config adds config_base.
 */
const uint32_t aic8800u_patch_tbl[AIC8800_PATCH_TBL_COUNT][2] = {
	{ 0x00b4, 0xf3010000 },	/* band config (USE_5G unset) */
	{ 0x0170, 0x0001000a },	/* rx aggr counter */
	{ 0x0188, 0x00000001 },	/* user_ext_flags: PWROFST_COVER_CALIB */
};

const char *
aic8800u_personality_name(enum aic8800u_personality personality)
{
	switch (personality) {
	case AIC8800U_BROM:
		return "boot ROM";
	case AIC8800U_APP:
		return "app";
	}
	return "unknown";
}

static int
aic8800u_load_file(struct aic8800u_softc *sc, const char *name,
    uint8_t **datap, size_t *sizep)
{
	firmware_handle_t fwh;
	size_t size;
	uint8_t *data;
	int error;

	error = firmware_open(AIC8800_FWDIR, name, &fwh);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "firmware %s: open failed (%d)\n",
		    name, error);
		return error;
	}

	size = firmware_get_size(fwh);
	data = kmem_alloc(size, KM_SLEEP);
	error = firmware_read(fwh, 0, data, size);
	firmware_close(fwh);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "firmware %s: read failed (%d)\n",
		    name, error);
		kmem_free(data, size);
		return error;
	}

	*datap = data;
	*sizep = size;
	return 0;
}

static void
aic8800u_free_file(uint8_t *data, size_t size)
{

	if (data != NULL)
		kmem_free(data, size);
}

/*
 * Probe whether firmware(9) can serve the packaged files yet -- on the
 * ramdisk line the USB devices attach before mountroot, so the bring-up
 * thread polls until the root file system is there.
 */
bool
aic8800u_firmware_available(struct aic8800u_softc *sc)
{
	uint8_t *data;
	size_t size;

	if (aic8800u_load_file(sc, AIC8800_FW_PATCH_TABLE, &data, &size) != 0)
		return false;
	aic8800u_free_file(data, size);
	return true;
}

static uint32_t
aic8800u_le32(const uint8_t *p)
{

	return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

/*
 * Upload one firmware file to RAM in 1 KiB DBG_MEM_BLOCK_WRITE chunks.
 * The vendor frame always carries the full 1032-byte parameter (the
 * payload is zero padded), so a short final chunk is written the same
 * way.
 */
static int
aic8800u_upload_file(struct aic8800u_softc *sc, const char *name,
    uint32_t addr, const uint8_t *data, size_t size)
{
	uint8_t req[AIC8800_MEM_BLOCK_WRITE_REQ_LEN];
	size_t off;
	int error;

	aprint_normal_dev(sc->sc_dev, "upload %s (%zu bytes) -> %#x\n",
	    name, size, addr);

	for (off = 0; off < size; off += AIC8800_MEM_BLOCK_WRITE_DATA) {
		size_t chunk = size - off;

		if (chunk > AIC8800_MEM_BLOCK_WRITE_DATA)
			chunk = AIC8800_MEM_BLOCK_WRITE_DATA;

		memset(req, 0, sizeof(req));
		req[0] = addr & 0xff;
		req[1] = (addr >> 8) & 0xff;
		req[2] = (addr >> 16) & 0xff;
		req[3] = (addr >> 24) & 0xff;
		req[4] = chunk & 0xff;
		req[5] = (chunk >> 8) & 0xff;
		req[6] = (chunk >> 16) & 0xff;
		req[7] = (chunk >> 24) & 0xff;
		memcpy(&req[AIC8800_MEM_BLOCK_WRITE_HDR], data + off, chunk);

		error = aic8800u_cmd(sc, AIC8800_DBG_MEM_BLOCK_WRITE_REQ,
		    AIC8800_TASK_DBG, AIC8800_DRV_TASK_ID, req, sizeof(req),
		    NULL, 0);
		if (error != 0) {
			aprint_error_dev(sc->sc_dev,
			    "upload %s: block at %#x+%#zx failed (%d)\n",
			    name, addr, off, error);
			return error;
		}
		addr += chunk;
	}

	return 0;
}

static void
aic8800u_patch_table_free(struct aic8800u_patch_table *head)
{
	struct aic8800u_patch_table *p, *next;

	for (p = head; p != NULL; p = next) {
		next = p->next;
		if (p->data != NULL)
			kmem_free(p->data, p->len * 2 * sizeof(uint32_t));
		kmem_free(p, sizeof(*p));
	}
}

/*
 * Parse fw_patch_table_*.bin: a 16-byte tag, then repeated
 * [16B name][u32 type][u32 len][len * {addr, val} pairs].
 */
static int
aic8800u_patch_table_parse(struct aic8800u_softc *sc, const uint8_t *raw,
    size_t size, struct aic8800u_patch_table **headp)
{
	struct aic8800u_patch_table *head = NULL, *cur = NULL;
	const uint8_t *p = raw;

	*headp = NULL;

	if (size < 16 || memcmp(p, AIC8800_PT_TAG, sizeof(AIC8800_PT_TAG)) != 0) {
		aprint_error_dev(sc->sc_dev, "patch table: bad tag\n");
		return EINVAL;
	}
	p += 16;

	while ((size_t)(p - raw) < size) {
		struct aic8800u_patch_table *t;
		uint32_t type, len;

		if (size - (p - raw) < 24) {
			aprint_error_dev(sc->sc_dev,
			    "patch table: truncated entry header\n");
			goto fail;
		}
		/* [0..15] entry name -- informational, skipped */

		t = kmem_alloc(sizeof(*t), KM_SLEEP);
		memset(t, 0, sizeof(*t));

		type = aic8800u_le32(p + 16);
		len = aic8800u_le32(p + 20);
		p += 24;

		if (type >= AIC8800_PT_ENTRY_MAX || len == 0) {
			/* vendor workaround: skip, no payload */
			len = 0;
		} else {
			if (size - (p - raw) < (size_t)len * 8) {
				aprint_error_dev(sc->sc_dev,
				    "patch table: truncated entry data\n");
				kmem_free(t, sizeof(*t));
				goto fail;
			}
			t->data = kmem_alloc(len * 2 * sizeof(uint32_t),
			    KM_SLEEP);
			memcpy(t->data, p, len * 2 * sizeof(uint32_t));
			p += len * 8;
		}

		t->type = type;
		t->len = len;
		if (head == NULL) {
			head = cur = t;
		} else {
			cur->next = t;
			cur = t;
		}
	}

	*headp = head;
	return 0;

fail:
	aic8800u_patch_table_free(head);
	return EINVAL;
}

/*
 * Fill the patch_info fields from the INF table and write the table
 * entries to firmware RAM.  The BTMODE entry values are overwritten
 * with the vendor loader's static defaults (no aicbt.conf in the
 * ground-truth build).
 */
static int
aic8800u_patch_info_unpack(struct aic8800u_softc *sc,
    struct aic8800u_patch_table *head, struct aic8800u_patch_info *info)
{
	struct aic8800u_patch_table *t;
	uint32_t *data;

	memset(info, 0, sizeof(*info));

	for (t = head; t != NULL; t = t->next) {
		if (t->type == AIC8800_PT_INF)
			break;
	}
	if (t == NULL || t->len == 0 || t->data == NULL)
		return ENOENT;

	data = t->data;
	/*
	 * The INF pairs map 1:1 onto the patch_info fields after
	 * info_len: (adid_addrinf, addr_adid), (patch_addrinf,
	 * addr_patch), (reset_addr, reset_val), (adid_flag_addr,
	 * adid_flag) and, when the table carries a fifth pair,
	 * (ext_patch_nb_addr, ext_patch_nb).  Only the download
	 * addresses and the ext-patch count matter here.
	 */
	if (t->len >= 4) {
		info->addr_adid = aic8800u_le32(
		    (const uint8_t *)&data[1]);
		info->addr_patch = aic8800u_le32(
		    (const uint8_t *)&data[3]);
		info->info_len = 4;
	}
	if (t->len >= 5) {
		info->ext_patch_nb = aic8800u_le32(
		    (const uint8_t *)&data[9]);
		info->info_len = 5;
		/* {id, addr} pairs follow the fifth pair in the buffer */
		info->ext_patch_param = data + 10;
	}

	/* fall back to the vendor's U02 defaults when the table is silent */
	if (info->addr_adid == 0)
		info->addr_adid = AIC8800_RAM_ADID_ADDR_U02;
	if (info->addr_patch == 0)
		info->addr_patch = AIC8800_RAM_PATCH_ADDR_U02;

	return 0;
}

static int
aic8800u_patch_table_load(struct aic8800u_softc *sc,
    struct aic8800u_patch_table *head)
{
	struct aic8800u_patch_table *t;
	int error;

	for (t = head; t != NULL; t = t->next) {
		uint32_t *data;
		uint32_t i;

		if (t->type == AIC8800_PT_VERSION) {
			aprint_normal_dev(sc->sc_dev, "patch table version: %s\n",
			    (const char *)t->data);
			continue;
		}
		if (t->type == AIC8800_PT_BTMODE && t->len >= 9) {
			/*
			 * Vendor defaults (aicbluetooth.c static
			 * initializer): hwinfo present, hwinfo -1,
			 * cpmode WORK, btmode BT_ONLY_COANT, btport MB,
			 * uart baud 1.5M, flow control on, LPM off,
			 * txpwr 0x6f2f.  Each pair is {addr, value}.
			 */
			data = t->data;
			data[1] = 1;
			data[3] = 0xffffffff;
			data[5] = 0;
			data[7] = 5;		/* BT_ONLY_COANT */
			data[9] = 1;		/* BTPORT_MB */
			data[11] = 1500000;	/* UART_BAUD_1_5M */
			data[13] = 1;		/* FLOWCTRL_ENABLE */
			data[15] = 0;		/* LPM off */
			data[17] = 0x00006f2f;	/* TXPWR_LVL_8800d80 */
		}

		data = t->data;
		for (i = 0; i < t->len; i++) {
			error = aic8800u_dbg_write32(sc, data[2 * i],
			    data[2 * i + 1]);
			if (error != 0)
				return error;
		}

		if (t->type == AIC8800_PT_PWRON) {
			/* the firmware needs a beat after power-on writes */
			kpause("aicpwron", false, mstohz(100), NULL);
		}
	}

	return 0;
}

static int
aic8800u_ext_patch_load(struct aic8800u_softc *sc,
    struct aic8800u_patch_info *info)
{
	char name[32];
	uint8_t *data;
	size_t size;
	uint32_t index;
	int error;

	for (index = 0; index < info->ext_patch_nb; index++) {
		uint32_t id, addr;

		id = info->ext_patch_param[2 * index];
		addr = info->ext_patch_param[2 * index + 1];

		snprintf(name, sizeof(name), "%s%u.bin", AIC8800_FW_PATCH_EXT,
		    (unsigned)id);
		aprint_normal_dev(sc->sc_dev, "ext patch %u: %s -> %#x\n",
		    index, name, addr);

		error = aic8800u_load_file(sc, name, &data, &size);
		if (error != 0)
			return error;
		error = aic8800u_upload_file(sc, name, addr, data, size);
		aic8800u_free_file(data, size);
		if (error != 0)
			return error;
	}

	return 0;
}

/*
 * aicwf_patch_config_8800d80: write the aic_patch_t structure and the
 * host-side patch pairs into firmware RAM.
 */
static int
aic8800u_patch_config(struct aic8800u_softc *sc)
{
	uint32_t config_base, patch_str_base, start_addr;
	uint32_t fw_addr = AIC8800_RAM_FW_ADDR_U02;
	uint32_t val, patch_addr;
	unsigned int cnt;
	int error;

	/* where the firmware keeps its config and patch-struct bases */
	error = aic8800u_dbg_read32(sc, fw_addr + AIC8800_FW_PATCH_CONFIG_BASE,
	    &config_base);
	if (error != 0)
		return error;
	error = aic8800u_dbg_read32(sc, fw_addr + AIC8800_FW_PATCH_STR_BASE,
	    &patch_str_base);
	if (error != 0)
		return error;

	error = aic8800u_dbg_read32(sc, fw_addr + AIC8800_FW_VERSION_OFF,
	    &sc->sc_fw_version);
	if (error != 0)
		return error;
	aprint_normal_dev(sc->sc_dev, "fw version %08x, config_base %#x,"
	    " patch_str_base %#x\n", sc->sc_fw_version, config_base,
	    patch_str_base);

	start_addr = AIC8800_PATCH_START_ADDR;
	if (sc->sc_fw_version > AIC8800_FW_PATCH_BUFF_MIN_VER) {
		error = aic8800u_dbg_read32(sc,
		    fw_addr + AIC8800_FW_PATCH_BUFF_OFF, &val);
		if (error != 0)
			return error;
		start_addr = val;
	}
	patch_addr = start_addr;

	/* aic_patch_t header */
	error = aic8800u_dbg_write32(sc,
	    patch_str_base + AIC8800_PATCH_OFF_MAGIC_NUM,
	    AIC8800_PATCH_MAGIC_NUM);
	if (error != 0)
		return error;
	error = aic8800u_dbg_write32(sc,
	    patch_str_base + AIC8800_PATCH_OFF_MAGIC_NUM_2,
	    AIC8800_PATCH_MAGIC_NUM_2);
	if (error != 0)
		return error;
	error = aic8800u_dbg_write32(sc,
	    patch_str_base + AIC8800_PATCH_OFF_PAIR_START, patch_addr);
	if (error != 0)
		return error;
	error = aic8800u_dbg_write32(sc,
	    patch_str_base + AIC8800_PATCH_OFF_PAIR_COUNT,
	    AIC8800_PATCH_TBL_COUNT);
	if (error != 0)
		return error;

	/* the pairs themselves */
	for (cnt = 0; cnt < AIC8800_PATCH_TBL_COUNT; cnt++) {
		error = aic8800u_dbg_write32(sc,
		    start_addr + 8 * cnt,
		    aic8800u_patch_tbl[cnt][0] + config_base);
		if (error != 0)
			return error;
		error = aic8800u_dbg_write32(sc,
		    start_addr + 8 * cnt + 4, aic8800u_patch_tbl[cnt][1]);
		if (error != 0)
			return error;
	}

	/* patch blocks 0..3 are unused */
	for (cnt = 0; cnt < AIC8800_PATCH_BLOCK_MAX; cnt++) {
		error = aic8800u_dbg_write32(sc,
		    patch_str_base + AIC8800_PATCH_OFF_BLOCK_SIZE +
		    4 * cnt, 0);
		if (error != 0)
			return error;
	}

	return 0;
}

/*
 * The whole U02 download sequence.  Runs in the bring-up thread; every
 * step can fail and unwinds with an error print.
 */
void
aic8800u_fw_download(struct aic8800u_softc *sc)
{
	struct aic8800u_patch_table *head = NULL;
	struct aic8800u_patch_info info;
	uint8_t *data = NULL;
	size_t size;
	uint32_t chip_id;
	int error;

	/* chip revision decides the file variant: U02 and U03 both take
	 * the _u02 file set (only U01 uses the unsuffixed files, which we
	 * do not package).  The register's high half carries more fields,
	 * so the revision is the vendor's (u8)(val >> 16) truncation. */
	error = aic8800u_dbg_read32(sc, AIC8800_SYS_CHIPID_REG, &chip_id);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "chip id read failed (%d)\n",
		    error);
		return;
	}
	sc->sc_chip_id = (chip_id >> 16) & 0xff;
	aprint_normal_dev(sc->sc_dev, "chip_id %#x (reg %#x)\n",
	    sc->sc_chip_id, chip_id);
	if (sc->sc_chip_id != AIC8800_CHIP_REV_U02 &&
	    sc->sc_chip_id != AIC8800_CHIP_REV_U03) {
		aprint_error_dev(sc->sc_dev,
		    "chip revision %#x has no firmware packaged (need U02/U03)\n",
		    sc->sc_chip_id);
		return;
	}

	/* patch table: format + per-file download addresses */
	error = aic8800u_load_file(sc, AIC8800_FW_PATCH_TABLE, &data, &size);
	if (error != 0)
		return;
	error = aic8800u_patch_table_parse(sc, data, size, &head);
	aic8800u_free_file(data, size);
	if (error != 0)
		return;

	error = aic8800u_patch_info_unpack(sc, head, &info);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "patch info unpack failed\n");
		goto out;
	}
	aprint_normal_dev(sc->sc_dev, "addr_adid %#x, addr_patch %#x,"
	    " ext_patch_nb %u\n", info.addr_adid, info.addr_patch,
	    info.ext_patch_nb);

	error = aic8800u_load_file(sc, AIC8800_FW_ADID, &data, &size);
	if (error != 0)
		goto out;
	error = aic8800u_upload_file(sc, AIC8800_FW_ADID, info.addr_adid,
	    data, size);
	aic8800u_free_file(data, size);
	if (error != 0)
		goto out;

	error = aic8800u_load_file(sc, AIC8800_FW_PATCH, &data, &size);
	if (error != 0)
		goto out;
	error = aic8800u_upload_file(sc, AIC8800_FW_PATCH, info.addr_patch,
	    data, size);
	aic8800u_free_file(data, size);
	if (error != 0)
		goto out;

	if (info.ext_patch_nb > 0) {
		error = aic8800u_ext_patch_load(sc, &info);
		if (error != 0)
			goto out;
	}

	error = aic8800u_patch_table_load(sc, head);
	if (error != 0)
		goto out;

	error = aic8800u_load_file(sc, AIC8800_FW_FMACFW, &data, &size);
	if (error != 0)
		goto out;
	error = aic8800u_upload_file(sc, AIC8800_FW_FMACFW,
	    AIC8800_RAM_FW_ADDR_U02, data, size);
	aic8800u_free_file(data, size);
	if (error != 0)
		goto out;

	error = aic8800u_patch_config(sc);
	if (error != 0)
		goto out;

	error = aic8800u_start_app(sc, AIC8800_RAM_FW_ADDR_U02);
	if (error != 0) {
		/*
		 * The firmware resets the device right after taking the
		 * START_APP command, so the CFM often never makes it
		 * back before the bus detach tears the transfer down
		 * (seen on the board: EIO 30 ms in, re-enumeration as
		 * the app personality 0.6 s later).  That is the same
		 * "failure is the success sign" situation as the
		 * modeswitch CSW; the real judge is the re-enumeration.
		 */
		aprint_normal_dev(sc->sc_dev, "start_app: CFM not observed"
		    " (%d) -- expected, firmware is resetting\n", error);
		error = 0;
	}

	aprint_normal_dev(sc->sc_dev,
	    "firmware download complete, waiting for re-enumeration\n");
out:
	if (error != 0)
		aprint_error_dev(sc->sc_dev, "firmware download failed (%d)\n",
		    error);
	aic8800u_patch_table_free(head);
}
