/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
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
 * Chip-level routines for the RTL8189FTV: power-on sequence, the
 * power-on self test, eFuse access and firmware download.
 *
 * Sequences are transcribed from the vendor rtl8189fs driver
 * (hal/rtl8188f): the card-enable power flow lives in Hal8188FPwrSeq.h
 * (CARDDIS_TO_CARDEMU + CARDEMU_TO_ACT), the self test in
 * hal/hal_hci/hal_sdio.c sdio_power_on_check(), and the firmware
 * download in rtl8188f_hal_init.c rtl8188f_FirmwareDownload().  The
 * board's Linux ground truth shows the self test reporting
 * val_mix:0x0000063f / 0x1B8 test Pass, which is what these sequences
 * must reproduce.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/kmem.h>
#include <sys/endian.h>
#include <sys/socket.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <dev/sdmmc/sdmmcvar.h>

#include "rtw8189fvar.h"

#ifdef RTW8189F_DEBUG
int rtw8189f_debug = 0;
#endif

/* ------------------------------------------------------------------ */
/* Power sequence                                                      */
/* ------------------------------------------------------------------ */

#define RTW8189F_PWR_BASE_MAC		0	/* WLAN I/O register */
#define RTW8189F_PWR_BASE_SDIO		1	/* SDIO local register */

#define RTW8189F_PWR_CMD_WRITE		1
#define RTW8189F_PWR_CMD_POLLING	2
#define RTW8189F_PWR_CMD_DELAY		3
#define RTW8189F_PWR_CMD_END		0

struct rtw8189f_pwr_cmd {
	uint32_t	addr;
	uint8_t		base;
	uint8_t		cmd;
	uint8_t		mask;
	uint8_t		value;
};

/*
 * rtl8188F_card_enable_flow = TRANS_CARDDIS_TO_CARDEMU +
 * TRANS_CARDEMU_TO_ACT (include/Hal8188FPwrSeq.h), SDIO-relevant steps
 * only.
 */
static const struct rtw8189f_pwr_cmd rtw8189f_card_enable_flow[] = {
	/* leave suspend: SDIO-local 0x86 bit0 = 0, poll bit1 -> 1 */
	{ 0x0086, RTW8189F_PWR_BASE_SDIO, RTW8189F_PWR_CMD_WRITE,
	  __BIT(0), 0 },
	{ 0x0086, RTW8189F_PWR_BASE_SDIO, RTW8189F_PWR_CMD_POLLING,
	  __BIT(1), __BIT(1) },
	/* 0x04[12:11] = 0: disable WL suspend */
	{ 0x0005, RTW8189F_PWR_BASE_MAC, RTW8189F_PWR_CMD_WRITE,
	  __BIT(3) | __BIT(4), 0 },
	/* 0x04[10] = 0: disable SW LPS */
	{ 0x0005, RTW8189F_PWR_BASE_MAC, RTW8189F_PWR_CMD_WRITE,
	  __BIT(2), 0 },
	/* wait until 0x04[17] = 1: power ready */
	{ 0x0006, RTW8189F_PWR_BASE_MAC, RTW8189F_PWR_CMD_POLLING,
	  __BIT(1), __BIT(1) },
	/* 0x04[15] = 0: disable HW power down */
	{ 0x0005, RTW8189F_PWR_BASE_MAC, RTW8189F_PWR_CMD_WRITE,
	  __BIT(7), 0 },
	/* 0x04[11] = 0: disable WL suspend (again, per vendor flow) */
	{ 0x0005, RTW8189F_PWR_BASE_MAC, RTW8189F_PWR_CMD_WRITE,
	  __BIT(3), 0 },
	/* 0x04[8] = 1, then poll until 0: switch the power state machine */
	{ 0x0005, RTW8189F_PWR_BASE_MAC, RTW8189F_PWR_CMD_WRITE,
	  __BIT(0), __BIT(0) },
	{ 0x0005, RTW8189F_PWR_BASE_MAC, RTW8189F_PWR_CMD_POLLING,
	  __BIT(0), 0 },
	/* 0x27 <= 0x35: xtal_qsel = 1, reduces RF noise */
	{ 0x0027, RTW8189F_PWR_BASE_MAC, RTW8189F_PWR_CMD_WRITE,
	  0xff, 0x35 },
	{ 0, 0, RTW8189F_PWR_CMD_END, 0, 0 },
};

static int
rtw8189f_pwr_seq_step(struct rtw8189f_softc *sc,
    const struct rtw8189f_pwr_cmd *cmd)
{
	uint8_t cur;
	int retry;

	switch (cmd->base) {
	case RTW8189F_PWR_BASE_SDIO:
		switch (cmd->cmd) {
		case RTW8189F_PWR_CMD_WRITE:
			cur = rtw8189f_sdiolocal_read_1(sc, cmd->addr);
			rtw8189f_sdiolocal_write_1(sc, cmd->addr,
			    (cur & ~cmd->mask) | (cmd->value & cmd->mask));
			return 0;
		case RTW8189F_PWR_CMD_POLLING:
			for (retry = 0; retry < 100; retry++) {
				cur = rtw8189f_sdiolocal_read_1(sc, cmd->addr);
				if ((cur & cmd->mask) == (cmd->value & cmd->mask))
					return 0;
				kpause("rtw8189fpw", false, 1, NULL);
			}
			return EIO;
		}
		break;

	default: /* RTW8189F_PWR_BASE_MAC */
		switch (cmd->cmd) {
		case RTW8189F_PWR_CMD_WRITE:
			cur = rtw8189f_mac_read_1(sc, cmd->addr);
			rtw8189f_mac_write_1(sc, cmd->addr,
			    (cur & ~cmd->mask) | (cmd->value & cmd->mask));
			return 0;
		case RTW8189F_PWR_CMD_POLLING:
			for (retry = 0; retry < 100; retry++) {
				cur = rtw8189f_mac_read_1(sc, cmd->addr);
				if ((cur & cmd->mask) == (cmd->value & cmd->mask))
					return 0;
				kpause("rtw8189fpw", false, 1, NULL);
			}
			return EIO;
		}
		break;
	}

	return EINVAL;
}

static int
rtw8189f_pwr_seq(struct rtw8189f_softc *sc,
    const struct rtw8189f_pwr_cmd *flow)
{
	int error;

	for (; flow->cmd != RTW8189F_PWR_CMD_END; flow++) {
		error = rtw8189f_pwr_seq_step(sc, flow);
		if (error != 0) {
			aprint_error_dev(sc->sc_dev,
			    "power sequence failed at 0x%04x step %d (%d)\n",
			    flow->addr, flow->cmd, error);
			return error;
		}
	}

	return 0;
}

int
rtw8189f_power_on(struct rtw8189f_softc *sc)
{
	uint16_t cr;

	/* Unlock the ISO/CLK/power control registers. */
	rtw8189f_mac_write_1(sc, RTW8189F_REG_RSV_CTRL, 0x00);

	DNPRINTF(sc, RTW8189F_DBG_PWR, "running card enable flow\n");
	if (rtw8189f_pwr_seq(sc, rtw8189f_card_enable_flow) != 0)
		return EIO;

	/* Enable MAC DMA/WMAC/SCHEDULE/SEC blocks. */
	rtw8189f_mac_write_1(sc, RTW8189F_REG_CR, 0x00);
	cr = rtw8189f_mac_read_2(sc, RTW8189F_REG_CR);
	cr |= RTW8189F_CR_POWERON;
	rtw8189f_mac_write_2(sc, RTW8189F_REG_CR, cr);

	sc->sc_mac_on = true;
	return 0;
}

/* ------------------------------------------------------------------ */
/* Power-on self test                                                  */
/* ------------------------------------------------------------------ */

int
rtw8189f_power_on_check(struct rtw8189f_softc *sc)
{
	uint8_t b0, b1, b2, b3;
	uint32_t val_mix, res;
	int retry;

	b0 = rtw8189f_mac_read_1(sc, RTW8189F_REG_CR + 0);
	b1 = rtw8189f_mac_read_1(sc, RTW8189F_REG_CR + 1);
	b2 = rtw8189f_mac_read_1(sc, RTW8189F_REG_CR + 2);
	b3 = rtw8189f_mac_read_1(sc, RTW8189F_REG_CR + 3);
	if (b0 == 0xea || b1 == 0xea || b2 == 0xea || b3 == 0xea) {
		aprint_error_dev(sc->sc_dev, "power on failed (0x100=EA)\n");
		return EIO;
	}

	val_mix = b3 << 24 | b2 << 16 | b1 << 8 | b0;

	/* CMD53 dword read must agree with the CMD52 bytes. */
	for (retry = 0; retry < 100; retry++) {
		res = rtw8189f_mac_read_4(sc, RTW8189F_REG_CR);
		if (res == val_mix)
			break;
	}
	if (res != val_mix) {
		aprint_error_dev(sc->sc_dev,
		    "power on check failed: cmd52 %08x != cmd53 %08x\n",
		    val_mix, res);
		return EIO;
	}

	/* 0x1B8 write/read-back test through the CMD53 path. */
	for (retry = 0; retry < 100; retry++) {
		rtw8189f_mac_write_4(sc, 0x1b8, 0x12345678);
		res = rtw8189f_mac_read_4(sc, 0x1b8);
		if (res == 0x12345678)
			break;
	}
	if (res != 0x12345678) {
		aprint_error_dev(sc->sc_dev,
		    "power on check failed: 0x1b8 wrote 12345678 read %08x\n",
		    res);
		return EIO;
	}

	aprint_normal_dev(sc->sc_dev,
	    "power on check pass (0x100 val_mix 0x%08x)\n", val_mix);
	return 0;
}

/* ------------------------------------------------------------------ */
/* eFuse                                                               */
/* ------------------------------------------------------------------ */

static uint8_t
rtw8189f_efuse_read_1(struct rtw8189f_softc *sc, uint16_t addr)
{
	uint32_t reg;
	int retry;

	reg = rtw8189f_mac_read_4(sc, RTW8189F_REG_EFUSE_CTRL);
	reg &= ~RTW8189F_EFUSE_CTRL_ADDR_M;
	reg |= addr << RTW8189F_EFUSE_CTRL_ADDR_S;
	reg &= ~RTW8189F_EFUSE_CTRL_VALID;
	reg |= RTW8189F_EFUSE_CTRL_LDOEEN;	/* keep the eFuse LDO up */
	rtw8189f_mac_write_4(sc, RTW8189F_REG_EFUSE_CTRL, reg);

	for (retry = 0; retry < 100; retry++) {
		reg = rtw8189f_mac_read_4(sc, RTW8189F_REG_EFUSE_CTRL);
		if (reg & RTW8189F_EFUSE_CTRL_VALID)
			return reg & RTW8189F_EFUSE_CTRL_DATA_M;
		DELAY(5);
	}

	aprint_error_dev(sc->sc_dev,
	    "could not read efuse byte at 0x%04x (ctrl 0x%08x)\n",
	    addr, reg);
	return 0xff;
}

static void
rtw8189f_efuse_switch_power(struct rtw8189f_softc *sc)
{
	uint16_t reg;

	reg = rtw8189f_mac_read_2(sc, RTW8189F_REG_SYS_ISO_CTRL);
	if ((reg & RTW8189F_PWC_EV12V) == 0)
		rtw8189f_mac_write_2(sc, RTW8189F_REG_SYS_ISO_CTRL,
		    reg | RTW8189F_PWC_EV12V);

	reg = rtw8189f_mac_read_2(sc, RTW8189F_REG_SYS_FUNC_EN);
	if ((reg & RTW8189F_FEN_ELDR) == 0)
		rtw8189f_mac_write_2(sc, RTW8189F_REG_SYS_FUNC_EN,
		    reg | RTW8189F_FEN_ELDR);

	reg = rtw8189f_mac_read_2(sc, RTW8189F_REG_SYS_CLKR);
	if ((reg & (RTW8189F_LOADER_EN | RTW8189F_ANA8M)) !=
	    (RTW8189F_LOADER_EN | RTW8189F_ANA8M))
		rtw8189f_mac_write_2(sc, RTW8189F_REG_SYS_CLKR,
		    reg | RTW8189F_LOADER_EN | RTW8189F_ANA8M);
}

int
rtw8189f_efuse_read(struct rtw8189f_softc *sc)
{
	uint8_t *map = sc->sc_efuse_map;
	uint16_t addr;
	uint8_t hdr, off, msk;
	size_t i;
	int error;

	memset(map, 0xff, RTW8189F_HWSET_MAX_SIZE);

	rtw8189f_efuse_switch_power(sc);

	/*
	 * Diagnostics: dump the first physical bytes and the region around
	 * the MAC address, so an empty/failed parse can be told apart from
	 * a wrong address decode.
	 */
	{
		uint8_t dbg[8];
		int di;

		for (di = 0; di < 8; di++)
			dbg[di] = rtw8189f_efuse_read_1(sc, di);
		aprint_normal_dev(sc->sc_dev, "efuse phys[0..7]: "
		    "%02x %02x %02x %02x %02x %02x %02x %02x (ctrl 0x%08x)\n",
		    dbg[0], dbg[1], dbg[2], dbg[3],
		    dbg[4], dbg[5], dbg[6], dbg[7],
		    rtw8189f_mac_read_4(sc, RTW8189F_REG_EFUSE_CTRL));
	}

	/* Read the physical map (256 bytes) and parse the pg packets.
	 * RTL8188F uses the extended-header format: a packet header with
	 * (hdr & 0x1f) == 0x0f is followed by an extension byte carrying
	 * the high offset bits (logical map is 512 bytes, not 256). */
	addr = 0;
	while (addr < RTW8189F_EFUSE_REAL_CONTENT_LEN) {
		hdr = rtw8189f_efuse_read_1(sc, addr++);
		if (hdr == 0xff)
			break;
		if ((hdr & 0x1f) == 0x0f) {
			uint8_t ext = rtw8189f_efuse_read_1(sc, addr++);
			if (ext == 0xff)
				break;
			off = ((ext & 0xf0) >> 1) | ((hdr & 0xe0) >> 5);
			msk = ext & 0x0f;
		} else {
			off = (hdr >> 4) & 0x0f;
			msk = hdr & 0x0f;
		}
		for (i = 0; i < 4; i++) {
			if (msk & (1U << i))
				continue;
			if (off * 8 + i * 2 + 1 >= RTW8189F_HWSET_MAX_SIZE) {
				aprint_error_dev(sc->sc_dev,
				    "efuse header out of range\n");
				error = EIO;
				goto out;
			}
			map[off * 8 + i * 2 + 0] =
			    rtw8189f_efuse_read_1(sc, addr++);
			map[off * 8 + i * 2 + 1] =
			    rtw8189f_efuse_read_1(sc, addr++);
		}
	}

	aprint_normal_dev(sc->sc_dev,
	    "efuse parse stopped at phys 0x%03x; logical[0x110..0x12f]: "
	    "%02x %02x %02x %02x %02x %02x %02x %02x "
	    "%02x %02x %02x %02x %02x %02x %02x %02x "
	    "%02x %02x %02x %02x %02x %02x %02x %02x "
	    "%02x %02x %02x %02x %02x %02x %02x %02x\n",
	    addr,
	    map[0x110], map[0x111], map[0x112], map[0x113],
	    map[0x114], map[0x115], map[0x116], map[0x117],
	    map[0x118], map[0x119], map[0x11a], map[0x11b],
	    map[0x11c], map[0x11d], map[0x11e], map[0x11f],
	    map[0x120], map[0x121], map[0x122], map[0x123],
	    map[0x124], map[0x125], map[0x126], map[0x127],
	    map[0x128], map[0x129], map[0x12a], map[0x12b],
	    map[0x12c], map[0x12d], map[0x12e], map[0x12f]);

	DNPRINTF(sc, RTW8189F_DBG_EFUSE, "MAC %02x:%02x:%02x:%02x:%02x:%02x "
	    "at 0x%03x\n",
	    map[RTW8189F_EEPROM_MAC_ADDR + 0], map[RTW8189F_EEPROM_MAC_ADDR + 1],
	    map[RTW8189F_EEPROM_MAC_ADDR + 2], map[RTW8189F_EEPROM_MAC_ADDR + 3],
	    map[RTW8189F_EEPROM_MAC_ADDR + 4], map[RTW8189F_EEPROM_MAC_ADDR + 5],
	    RTW8189F_EEPROM_MAC_ADDR);

	/* A valid MAC has the locally-administered bit pattern the vendor
	 * modules ship with; reject all-ff/all-00 (autoload fail). */
	memcpy(sc->sc_mac_addr, map + RTW8189F_EEPROM_MAC_ADDR,
	    IEEE80211_ADDR_LEN);
	if ((sc->sc_mac_addr[0] == 0xff && sc->sc_mac_addr[5] == 0xff) ||
	    (sc->sc_mac_addr[0] == 0x00 && sc->sc_mac_addr[5] == 0x00)) {
		aprint_error_dev(sc->sc_dev, "no valid eFuse MAC\n");
		error = ENOENT;
		goto out;
	}
	sc->sc_mac_valid = true;
	error = 0;
out:
	return error;
}

/* ------------------------------------------------------------------ */
/* Firmware download                                                   */
/* ------------------------------------------------------------------ */

static void
rtw8189f_8051_reset(struct rtw8189f_softc *sc)
{
	uint8_t io, cpu;

	/* Toggle the MCU IO wrapper (0x1c[8]) and CPU enable (0x02[10]). */
	io = rtw8189f_mac_read_1(sc, RTW8189F_REG_RSV_CTRL + 1);
	rtw8189f_mac_write_1(sc, RTW8189F_REG_RSV_CTRL + 1, io & ~__BIT(0));

	cpu = rtw8189f_mac_read_1(sc, RTW8189F_REG_SYS_FUNC_EN + 1);
	rtw8189f_mac_write_1(sc, RTW8189F_REG_SYS_FUNC_EN + 1,
	    cpu & ~__BIT(2));

	io = rtw8189f_mac_read_1(sc, RTW8189F_REG_RSV_CTRL + 1);
	rtw8189f_mac_write_1(sc, RTW8189F_REG_RSV_CTRL + 1, io | __BIT(0));

	cpu = rtw8189f_mac_read_1(sc, RTW8189F_REG_SYS_FUNC_EN + 1);
	rtw8189f_mac_write_1(sc, RTW8189F_REG_SYS_FUNC_EN + 1,
	    cpu | __BIT(2));

	DNPRINTF(sc, RTW8189F_DBG_FW, "8051 reset done\n");
}

static int
rtw8189f_fw_download_enable(struct rtw8189f_softc *sc, bool enable)
{
	uint8_t tmp;
	int retry;

	if (enable) {
		/* 8051 enable: 0x03 |= 0x04 */
		tmp = rtw8189f_mac_read_1(sc, RTW8189F_REG_SYS_FUNC_EN + 1);
		rtw8189f_mac_write_1(sc, RTW8189F_REG_SYS_FUNC_EN + 1,
		    tmp | 0x04);

		/* MCUFWDL.EN: set and poll until it sticks. */
		tmp = rtw8189f_mac_read_1(sc, RTW8189F_REG_MCUFWDL);
		rtw8189f_mac_write_1(sc, RTW8189F_REG_MCUFWDL, tmp | 0x01);
		for (retry = 0; retry < 100; retry++) {
			tmp = rtw8189f_mac_read_1(sc, RTW8189F_REG_MCUFWDL);
			if (tmp & 0x01)
				break;
			rtw8189f_mac_write_1(sc, RTW8189F_REG_MCUFWDL,
			    tmp | 0x01);
			kpause("rtw8189ffw", false, 1, NULL);
		}

		/* 8051 reset: 0x82 &= ~0x08 */
		tmp = rtw8189f_mac_read_1(sc, RTW8189F_REG_MCUFWDL + 2);
		rtw8189f_mac_write_1(sc, RTW8189F_REG_MCUFWDL + 2,
		    tmp & ~0x08);
	} else {
		tmp = rtw8189f_mac_read_1(sc, RTW8189F_REG_MCUFWDL);
		rtw8189f_mac_write_1(sc, RTW8189F_REG_MCUFWDL, tmp & ~0x01);
	}

	return 0;
}

static int
rtw8189f_fw_write_page(struct rtw8189f_softc *sc, uint8_t page,
    const uint8_t *buf, uint32_t size)
{
	uint32_t i;
	uint8_t tmp;

	/* Select the 4 KiB RAM page in MCUFWDL[2] bits [2:0]. */
	tmp = rtw8189f_mac_read_1(sc, RTW8189F_REG_MCUFWDL + 2);
	rtw8189f_mac_write_1(sc, RTW8189F_REG_MCUFWDL + 2,
	    (tmp & 0xf8) | (page & 0x07));

	/* 4-byte IO writes to FW_START_ADDRESS, byte writes for the tail. */
	for (i = 0; i + 4 <= size; i += 4) {
		uint32_t v = le32dec(buf + i);
		rtw8189f_mac_write_4(sc,
		    RTW8189F_FW_START_ADDRESS + i, v);
	}
	for (; i < size; i++)
		rtw8189f_mac_write_1(sc, RTW8189F_FW_START_ADDRESS + i,
		    buf[i]);

	return 0;
}

int
rtw8189f_fw_download(struct rtw8189f_softc *sc)
{
	const uint8_t *fw = sc->sc_fw;
	const uint8_t *buf;
	uint32_t len, off, page;
	uint16_t sig;
	uint8_t tmp;
	int attempt, wait, error;

	if (sc->sc_fw == NULL || sc->sc_fwsize < RTW8189F_FW_HDR_SIZE + 4 ||
	    sc->sc_fwsize > RTW8189F_FW_MAX_SIZE + RTW8189F_FW_HDR_SIZE)
		return EINVAL;

	/* Download-power-state handshake: 0xA3[2:0] = 010, 0xA0[1:0] = 01. */
	tmp = rtw8189f_mac_read_1(sc, 0xa3);
	tmp = (tmp & 0xf8) | 0x02;
	rtw8189f_mac_write_1(sc, 0xa3, tmp);
	tmp = rtw8189f_mac_read_1(sc, 0xa0) & 0x03;
	if (tmp != 0x01)
		aprint_error_dev(sc->sc_dev,
		    "unexpected FW download power state 0xa0=%x\n", tmp);

	sig = le16dec(fw);
	if (sig != RTW8189F_FW_SIGNATURE) {
		aprint_error_dev(sc->sc_dev,
		    "bad firmware signature 0x%04x\n", sig);
		return EINVAL;
	}
	aprint_normal_dev(sc->sc_dev,
	    "firmware v%u, %zu bytes (header included)\n",
	    fw[RTW8189F_FW_VERSION_OFF],
	    sc->sc_fwsize);

	/* Skip the 32-byte image header. */
	buf = fw + RTW8189F_FW_HDR_SIZE;
	len = sc->sc_fwsize - RTW8189F_FW_HDR_SIZE;

	/* If 8051 is running from RAM, tell it to reset. */
	if (rtw8189f_mac_read_1(sc, RTW8189F_REG_MCUFWDL) &
	    RTW8189F_RAM_DL_SEL) {
		rtw8189f_mac_write_1(sc, RTW8189F_REG_MCUFWDL, 0x00);
		rtw8189f_8051_reset(sc);
	}

	error = EIO;
	for (attempt = 0; attempt < 3; attempt++) {
		rtw8189f_fw_download_enable(sc, true);

		/* Reset the FWDL checksum bit so the MCU re-reports. */
		tmp = rtw8189f_mac_read_1(sc, RTW8189F_REG_MCUFWDL);
		rtw8189f_mac_write_1(sc, RTW8189F_REG_MCUFWDL,
		    tmp | RTW8189F_FWDL_CHKSUM_RPT);

		for (page = 0, off = 0; off < len;
		    page++, off += RTW8189F_FW_DL_PAGE_SIZE) {
			uint32_t n =
			    MIN(len - off, RTW8189F_FW_DL_PAGE_SIZE);
			rtw8189f_fw_write_page(sc, page, buf + off, n);
		}

		rtw8189f_fw_download_enable(sc, false);

		/* Poll the checksum report: MCUFWDL.BIT2 within ~250ms. */
		for (wait = 0; wait < 250; wait++) {
			uint32_t v =
			    rtw8189f_mac_read_4(sc, RTW8189F_REG_MCUFWDL);
			if (v & RTW8189F_FWDL_CHKSUM_RPT)
				break;
			kpause("rtw8189fc", false, 1, NULL);
		}
		if (wait < 250)
			break;
		aprint_error_dev(sc->sc_dev,
		    "firmware checksum not reported (attempt %d)\n",
		    attempt + 1);
	}
	if (attempt == 3)
		goto fail;

	aprint_normal_dev(sc->sc_dev, "firmware download checksum OK\n");

	return rtw8189f_fw_ready(sc);

fail:
	return error;
}

int
rtw8189f_fw_ready(struct rtw8189f_softc *sc)
{
	uint32_t expected = RTW8189F_MCUFWDL_RDY | RTW8189F_FWDL_CHKSUM_RPT |
	    RTW8189F_WINTINI_RDY | RTW8189F_RAM_DL_SEL;
	uint32_t v;
	int retry;

	/* MCUFWDL.RDY=1, clear WINTINI, reset the 8051, poll for boot. */
	v = rtw8189f_mac_read_4(sc, RTW8189F_REG_MCUFWDL);
	v |= RTW8189F_MCUFWDL_RDY;
	v &= ~RTW8189F_WINTINI_RDY;
	rtw8189f_mac_write_4(sc, RTW8189F_REG_MCUFWDL, v);

	rtw8189f_8051_reset(sc);

	for (retry = 0; retry < 200; retry++) {
		v = rtw8189f_mac_read_4(sc, RTW8189F_REG_MCUFWDL);
		if ((v & expected) == expected) {
			sc->sc_fw_ready = true;
			aprint_normal_dev(sc->sc_dev,
			    "firmware ready (MCUFWDL 0x%08x, %d ms)\n",
			    v, retry);
			return 0;
		}
		kpause("rtw8189fr", false, 1, NULL);
	}

	aprint_error_dev(sc->sc_dev,
	    "firmware not ready (MCUFWDL 0x%08x)\n", v);
	return EIO;
}
