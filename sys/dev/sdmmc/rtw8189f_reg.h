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
 * Register map and SDIO bus constants for the RTL8189FTV (RTL8188F
 * silicon).  Names follow the vendor rtl8189fs driver (GPL-2.0) so that
 * sequences can be cross-checked line by line; the code itself is
 * written against the NetBSD sdmmc(4) API.
 */

#ifndef _DEV_SDMMC_RTW8189F_REG_H_
#define _DEV_SDMMC_RTW8189F_REG_H_

/*
 * The SDIO function exposes a 17-bit address space, DeviceID in
 * bits [16:13]:
 *
 *	0 SDIO local registers (0x10250000 | 0x000-0xfff)
 *	1 WLAN_TX_EXQ    4 WLAN_TX_HIQ    5 WLAN_TX_MIQ    6 WLAN_TX_LOQ
 *	7 WLAN_RX0FF     8 WLAN I/O registers (MAC/BB/RF space)
 */
#define RTW8189F_SDIO_LOCAL_DEVICE_ID		0
#define RTW8189F_WLAN_TX_EXQ_DEVICE_ID		3
#define RTW8189F_WLAN_TX_HIQ_DEVICE_ID		4
#define RTW8189F_WLAN_TX_MIQ_DEVICE_ID		5
#define RTW8189F_WLAN_TX_LOQ_DEVICE_ID		6
#define RTW8189F_WLAN_RX0FF_DEVICE_ID		7
#define RTW8189F_WLAN_IOREG_DEVICE_ID		8

#define RTW8189F_SDIO_LOCAL_MSK			0x0fff
#define RTW8189F_WLAN_IOREG_MSK			0x7fff
#define RTW8189F_WLAN_FIFO_MSK			0x1fff
#define RTW8189F_WLAN_RX0FF_MSK			0x0003

#define RTW8189F_SDIO_LOCAL_BASE		0x10250000

/* SDIO local (DeviceID 0) registers. */
#define RTW8189F_SDIO_REG_TX_CTRL		0x0000
#define RTW8189F_SDIO_REG_HIMR			0x0014
#define RTW8189F_SDIO_REG_HISR			0x0018
#define RTW8189F_SDIO_REG_RX0_REQ_LEN		0x001c
#define RTW8189F_SDIO_REG_FREE_TXPG		0x0020
#define RTW8189F_SDIO_REG_HRPWM1		0x0080

/* HISR / HIMR bits. */
#define RTW8189F_HIMR_RX_REQUEST		__BIT(0)
#define RTW8189F_HIMR_AVAL			__BIT(1)
#define RTW8189F_HISR_RX_REQUEST		__BIT(0)
#define RTW8189F_HISR_AVAL			__BIT(1)

/* MAC register file (accessed through DeviceID 8). */
#define RTW8189F_REG_SYS_ISO_CTRL		0x0000
#define RTW8189F_REG_SYS_FUNC_EN		0x0002
#define RTW8189F_REG_APS_FSMCO			0x0004
#define RTW8189F_REG_SYS_CLKR			0x0008
#define RTW8189F_REG_RSV_CTRL			0x001c
#define RTW8189F_REG_EFUSE_CTRL			0x0030
#define RTW8189F_REG_EFUSE_TEST			0x0034
#define RTW8189F_REG_MCUFWDL			0x0080
#define RTW8189F_REG_HMETFR			0x01cc
#define RTW8189F_REG_HMEBOX(n)			(0x01d0 + (n) * 4)
#define RTW8189F_REG_HMEBOX_EXT(n)		(0x01f0 + (n) * 4)
#define RTW8189F_REG_CR				0x0100

/* REG_CR bits, as programmed at power-on (0x063f, Linux sdio_power_on_check
 * reports val_mix 0x63f on this very board). */
#define RTW8189F_HCI_TXDMA_EN			__BIT(0)
#define RTW8189F_HCI_RXDMA_EN			__BIT(1)
#define RTW8189F_TXDMA_EN			__BIT(2)
#define RTW8189F_RXDMA_EN			__BIT(3)
#define RTW8189F_PROTOCOL_EN			__BIT(4)
#define RTW8189F_SCHEDULE_EN			__BIT(5)
#define RTW8189F_MACTXEN			__BIT(6)
#define RTW8189F_MACRXEN			__BIT(7)
#define RTW8189F_ENSEC				__BIT(9)
#define RTW8189F_CALTMR_EN			__BIT(10)
#define RTW8189F_CR_POWERON \
	(RTW8189F_HCI_TXDMA_EN | RTW8189F_HCI_RXDMA_EN | RTW8189F_TXDMA_EN | \
	 RTW8189F_RXDMA_EN | RTW8189F_PROTOCOL_EN | RTW8189F_SCHEDULE_EN | \
	 RTW8189F_ENSEC | RTW8189F_CALTMR_EN)

/* REG_SYS_FUNC_EN bits. */
#define RTW8189F_FEN_ELDR			__BIT(12)
#define RTW8189F_FEN_CPUEN			__BIT(10)	/* 0x02[10] */
#define RTW8189F_FEN_CORE_EN			__BIT(8)	/* 0x02[8] */

/* REG_SYS_ISO_CTRL / REG_SYS_CLKR bits used by eFuse access. */
#define RTW8189F_PWC_EV12V			__BIT(15)	/* 0x00[15] */
#define RTW8189F_LOADER_EN			__BIT(5)	/* 0x08[5] */
#define RTW8189F_ANA8M				__BIT(1)	/* 0x08[1] */

/* REG_MCUFWDL bits. */
#define RTW8189F_MCUFWDL_EN			__BIT(0)
#define RTW8189F_MCUFWDL_RDY			__BIT(1)
#define RTW8189F_FWDL_CHKSUM_RPT		__BIT(2)
#define RTW8189F_WINTINI_RDY			__BIT(6)
#define RTW8189F_RAM_DL_SEL			__BIT(7)

/* Firmware download. */
#define RTW8189F_FW_START_ADDRESS		0x1000
#define RTW8189F_FW_DL_PAGE_SIZE		4096
#define RTW8189F_FW_MAX_SIZE			0x8000
#define RTW8189F_FW_HDR_SIZE			32

/* eFuse. */
#define RTW8189F_EFUSE_CTRL_DATA_M		__BITS(7, 0)
#define RTW8189F_EFUSE_CTRL_ADDR_M		__BITS(17, 8)
#define RTW8189F_EFUSE_CTRL_ADDR_S		8	/* bits [17:8] */
#define RTW8189F_EFUSE_CTRL_VALID		__BIT(31)
#define RTW8189F_EFUSE_CTRL_LDOEEN		__BIT(30)
#define RTW8189F_EFUSE_REAL_CONTENT_LEN		256
#define RTW8189F_HWSET_MAX_SIZE			512
#define RTW8189F_EEPROM_MAC_ADDR		0x11a

/* Firmware image header (32 bytes; first two bytes = signature). */
#define RTW8189F_FW_SIGNATURE			0x88f1
#define RTW8189F_FW_VERSION_OFF			4	/* u8 version at [4] */

#endif /* !_DEV_SDMMC_RTW8189F_REG_H_ */
