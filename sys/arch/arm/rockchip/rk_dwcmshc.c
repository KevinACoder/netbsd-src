/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by the RK3568 bring-up lab.
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
 * RK3568 eMMC host controller (Synopsys DWC MSHC, SDHCI-compatible).
 *
 * sdhci @ 0xfe310000 is a standard SDHCI core with a Rockchip vendor
 * area; the DLL/PHY tuning registers live at 0x800+.  Like the vendor
 * Linux board DT and U-Boot on this board, we run it at 8-bit/high-speed
 * <= 50 MHz only, which uses the documented DLL bypass path - no DLL
 * lock/tuning is needed.
 *
 * Sequences mirror three working references:
 *   - Linux drivers/mmc/host/sdhci-of-dwcmshc.c (rk3568: init clears
 *     HOST_CTRL3 cmd-conflict check and DLL regs; the <= 52 MHz branch of
 *     dwcmshc_rk3568_set_clock is the bypass sequence below; post-reset
 *     MISC_CON INTCLK_EN restore is rk35xx_sdhci_reset)
 *   - U-Boot drivers/mmc/rockchip_sdhci.c (same bypass values)
 *   - the bare-metal standalone SDK fdwmshc driver (validated on this
 *     board; also sets EMMC_CTRL CARD_IS_EMMC/RST_N release)
 *
 * The vendor area base (0x500) is hardcoded, matching U-Boot and the
 * standalone driver; Linux derives it from the SDHCI register at 0xe8
 * and gets the same value on rk3568.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/systm.h>
#include <sys/kmem.h>

#include <dev/sdmmc/sdhcreg.h>
#include <dev/sdmmc/sdhcvar.h>
#include <dev/sdmmc/sdmmcvar.h>

#include <dev/clk/clk_backend.h>

#include <dev/fdt/fdtvar.h>

/* Rockchip vendor area 1 (base 0x500). */
#define DWCMSHC_HOST_CTRL3	0x508	/* bit0: cmd conflict check */
#define DWCMSHC_EMMC_CONTROL	0x52c	/* bit0 CARD_IS_EMMC, bit2 RST_N,
					   bit3 RST_N_OE */
/* DWC MSHC DLL/PHY area (absolute). */
#define DWCMSHC_EMMC_DLL_CTRL	0x800
#define DWCMSHC_EMMC_DLL_RXCLK	0x804
#define DWCMSHC_EMMC_DLL_TXCLK	0x808
#define DWCMSHC_EMMC_DLL_STRBIN	0x80c
#define DWCMSHC_EMMC_DLL_CMDOUT	0x810
#define DWCMSHC_EMMC_MISC_CON	0x81c	/* bit1: INTCLK_EN */

#define DWCMSHC_CMD_CONFLICT_CHK	__BIT(0)
#define DWCMSHC_CARD_IS_EMMC		__BIT(0)
#define DWCMSHC_RST_N			__BIT(2)
#define DWCMSHC_RST_N_OE		__BIT(3)
#define DWCMSHC_DLL_CTRL_START		__BIT(0)
#define DWCMSHC_DLL_CTRL_RESET		__BIT(1)
#define DWCMSHC_DLL_BYPASS		__BIT(24)
#define DWCMSHC_DLL_DLYENA		__BIT(27)
#define DWCMSHC_DLL_RXCLK_ORI_GATE	__BIT(31)
#define DWCMSHC_DLL_STRBIN_DELAY_NUM_SEL __BIT(26)
#define DWCMSHC_DLL_STRBIN_DELAY_NUM	__SHIFTIN(16, __BITS(23, 16))
#define DWCMSHC_MISC_INTCLK_EN		__BIT(1)

/* Card clock strategy: the SDHCI clock divider of this controller is
 * unreliable - Linux and U-Boot never use it.  Instead they re-parent
 * CCLK_EMMC (CRU CLKSEL_CON28) to the requested card clock and run the
 * SDHCI divider at 0.  The source table is
 * { xin24m, gpll_200m, gpll_150m, cpll_100m, cpll_50m, osc0_div_375k };
 * identify mode is limited to the 375 kHz source ("Rockchip platform
 * only support 375KHz for identify mode" - Linux sdhci-of-dwcmshc),
 * high-speed runs at the 50 MHz source like the vendor DTs. */
#define RK_DWCMSHC_CORE_CLK_RATE	50000000
#define RK_DWCMSHC_IDENT_CLK_RATE	375000

struct rk_dwcmshc_softc {
	struct sdhc_softc	sc_base;	/* must be first */
	struct sdhc_host *	sc_host;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	bus_size_t		sc_bsz;
	struct clk *		sc_clk_core;
	void *			sc_ih;
};

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "rockchip,rk3568-dwcmshc" },
	{ .compat = "snps,dwcmshc-sdhci" },
	DEVICE_COMPAT_EOL
};

static int	rk_dwcmshc_match(device_t, cfdata_t, void *);
static void	rk_dwcmshc_attach(device_t, device_t, void *);

static int	rk_dwcmshc_vendor_init(struct rk_dwcmshc_softc *);
static int	rk_dwcmshc_bus_clock_pre(struct sdhc_softc *, int);
static int	rk_dwcmshc_bus_clock_post(struct sdhc_softc *, int);static void
rk_dwcmshc_write_4(struct rk_dwcmshc_softc *sc, bus_size_t reg, uint32_t val)
{
	bus_space_write_4(sc->sc_bst, sc->sc_bsh, reg, val);
}

static uint32_t
rk_dwcmshc_read_4(struct rk_dwcmshc_softc *sc, bus_size_t reg)
{
	return bus_space_read_4(sc->sc_bst, sc->sc_bsh, reg);
}

static uint16_t
rk_dwcmshc_read_2(struct rk_dwcmshc_softc *sc, bus_size_t reg)
{
	return bus_space_read_2(sc->sc_bst, sc->sc_bsh, reg);
}

static void
rk_dwcmshc_write_2(struct rk_dwcmshc_softc *sc, bus_size_t reg, uint16_t val)
{
	bus_space_write_2(sc->sc_bst, sc->sc_bsh, reg, val);
}

/*
 * Controller vendor setup shared by attach and every bus clock change.
 * All writes are idempotent and match the values U-Boot leaves behind.
 */
static int
rk_dwcmshc_vendor_init(struct rk_dwcmshc_softc *sc)
{
	uint32_t val;

	/* Restore the internal clock after a controller reset
	 * (Linux rk35xx_sdhci_reset). */
	val = rk_dwcmshc_read_4(sc, DWCMSHC_EMMC_MISC_CON);
	rk_dwcmshc_write_4(sc, DWCMSHC_EMMC_MISC_CON,
	    val | DWCMSHC_MISC_INTCLK_EN);

	/* Disable the cmd conflict check. */
	val = rk_dwcmshc_read_4(sc, DWCMSHC_HOST_CTRL3);
	rk_dwcmshc_write_4(sc, DWCMSHC_HOST_CTRL3,
	    val & ~DWCMSHC_CMD_CONFLICT_CHK);

	/* eMMC mode with the RST# line released (standalone fdwmshc). */
	rk_dwcmshc_write_4(sc, DWCMSHC_EMMC_CONTROL,
	    DWCMSHC_CARD_IS_EMMC | DWCMSHC_RST_N | DWCMSHC_RST_N_OE);

	/*
	 * DLL bypass path for card clocks <= 52 MHz (Linux
	 * dwcmshc_rk3568_set_clock / U-Boot dwcmshc_sdhci_emmc_set_clock).
	 */
	rk_dwcmshc_write_4(sc, DWCMSHC_EMMC_DLL_CTRL, 0);
	rk_dwcmshc_write_4(sc, DWCMSHC_EMMC_DLL_CTRL,
	    DWCMSHC_DLL_BYPASS | DWCMSHC_DLL_CTRL_START);
	rk_dwcmshc_write_4(sc, DWCMSHC_EMMC_DLL_RXCLK,
	    DWCMSHC_DLL_RXCLK_ORI_GATE);
	rk_dwcmshc_write_4(sc, DWCMSHC_EMMC_DLL_TXCLK, 0);
	rk_dwcmshc_write_4(sc, DWCMSHC_EMMC_DLL_CMDOUT, 0);
	rk_dwcmshc_write_4(sc, DWCMSHC_EMMC_DLL_STRBIN,
	    DWCMSHC_DLL_DLYENA |
	    DWCMSHC_DLL_STRBIN_DELAY_NUM_SEL |
	    DWCMSHC_DLL_STRBIN_DELAY_NUM);

	return 0;
}

static int
rk_dwcmshc_bus_clock_pre(struct sdhc_softc *sdhc, int freq)
{
	struct rk_dwcmshc_softc * const sc = device_private(sdhc->sc_dev);
	static const u_int srcs[] = {
	    RK_DWCMSHC_IDENT_CLK_RATE, 24000000, 50000000,
	    100000000, 150000000, 200000000 };
	u_int rate = srcs[0];
	u_int want = (u_int)freq * 1000;

	/* Re-parent CCLK_EMMC to the largest source <= the requested card
	 * clock; the SDHCI divider stays at 0 (see post hook). */
	for (u_int n = 1; n < __arraycount(srcs); n++) {
		if (srcs[n] <= want && srcs[n] > rate)
			rate = srcs[n];
	}
	(void)clk_set_rate(sc->sc_clk_core, rate);

	return rk_dwcmshc_vendor_init(sc);
}

static int
rk_dwcmshc_bus_clock_post(struct sdhc_softc *sdhc, int freq)
{
	struct rk_dwcmshc_softc * const sc = device_private(sdhc->sc_dev);
	uint16_t val;

	/* The sdhc core programs UHS mode select SDR50 for 25..52 MHz
	 * buses; the known-good value for eMMC high-speed on this
	 * controller is SDR25 (Linux/standalone). */
	if (freq > 25000 && freq <= 52000) {
		val = rk_dwcmshc_read_2(sc, SDHC_HOST_CTL2);
		val &= ~SDHC_UHS_MODE_SELECT_MASK;
		val |= SDHC_UHS_MODE_SELECT_SDR25;
		rk_dwcmshc_write_2(sc, SDHC_HOST_CTL2, val);
	}

	/* Force the SDHCI divider to 0: the card clock is CCLK_EMMC itself
	 * (this register matches U-Boot's working state, 0x0007). */
	rk_dwcmshc_write_2(sc, SDHC_CLOCK_CTL,
	    SDHC_INTCLK_STABLE | SDHC_INTCLK_ENABLE | SDHC_SDCLK_ENABLE);

	return 0;
}

static int
rk_dwcmshc_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rk_dwcmshc_attach(device_t parent, device_t self, void *aux)
{
	struct rk_dwcmshc_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct clk *clks[5];
	const char *const clknames[] = { "core", "bus", "axi", "block",
	    "timer" };
	char intrstr[128];
	bus_addr_t addr;
	bus_size_t size;
	u_int bus_width;
	int error;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error(": couldn't decode interrupt\n");
		return;
	}

	for (u_int n = 0; n < __arraycount(clks); n++) {
		clks[n] = fdtbus_clock_get(phandle, clknames[n]);
		if (clks[n] == NULL) {
			aprint_error(": couldn't get clock %s\n", clknames[n]);
			return;
		}
		error = clk_enable(clks[n]);
		if (error != 0) {
			aprint_error(": couldn't enable clock %s: %d\n",
			    clknames[n], error);
			return;
		}
	}

	error = clk_set_rate(clks[0], RK_DWCMSHC_CORE_CLK_RATE);
	if (error != 0) {
		aprint_error(": couldn't set core clock to %d Hz: %d\n",
		    RK_DWCMSHC_CORE_CLK_RATE, error);
		return;
	}
	sc->sc_clk_core = clks[0];

	if (of_getprop_uint32(phandle, "bus-width", &bus_width) != 0)
		bus_width = 4;
	if (bus_width != 4 && bus_width != 8) {
		aprint_error(": unsupported bus-width %u\n", bus_width);
		return;
	}

	sc->sc_base.sc_dev = self;
	sc->sc_base.sc_host = &sc->sc_host;
	sc->sc_base.sc_dmat = faa->faa_dmat;
	/* Capabilities are hand-crafted: high-speed 3.3 V only, no UHS /
	 * HS200 / DDR (caps2 = 0), so the eMMC runs at plain high speed,
	 * like U-Boot and Linux on this board. */
	sc->sc_base.sc_flags = SDHC_FLAG_NO_CLKBASE |
			       SDHC_FLAG_SINGLE_POWER_WRITE |
			       SDHC_FLAG_32BIT_ACCESS |
			       SDHC_FLAG_USE_DMA |
			       SDHC_FLAG_HOSTCAPS |
			       SDHC_FLAG_NON_REMOVABLE;
	if (bus_width == 8)
		sc->sc_base.sc_flags |= SDHC_FLAG_8BIT_MODE;
	sc->sc_base.sc_caps = SDHC_DMA_SUPPORT |
			      SDHC_ADMA2_SUPP |
			      SDHC_HIGH_SPEED_SUPP |
			      SDHC_VOLTAGE_SUPP_3_3V |
			      __SHIFTIN(RK_DWCMSHC_CORE_CLK_RATE / 1000000,
				  __BITS(15, 8)) |
			      SDHC_TIMEOUT_FREQ_UNIT | 31;
	sc->sc_base.sc_caps2 = 0;
	sc->sc_base.sc_clkbase = clk_get_rate(clks[0]) / 1000;
	sc->sc_base.sc_vendor_bus_clock = rk_dwcmshc_bus_clock_pre;
	sc->sc_base.sc_vendor_bus_clock_post = rk_dwcmshc_bus_clock_post;

	/* 32-bit ADMA2 descriptors cannot reach physical memory above
	 * 4 GiB; constrain DMA to the lower 4 GiB like arasan_sdhc_fdt. */
#ifdef _LP64
	{
		bus_dma_tag_t dmat;
		error = bus_dmatag_subregion(sc->sc_base.sc_dmat, 0,
		    __MASK(32), &dmat, BUS_DMA_WAITOK);
		if (error == 0)
			sc->sc_base.sc_dmat = dmat;
	}
#endif

	sc->sc_bst = faa->faa_bst;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
	sc->sc_bsz = size;

	aprint_naive("\n");
	aprint_normal(": RK3568 DWC MSHC eMMC (SDHCI)\n");

	sc->sc_ih = fdtbus_intr_establish_xname(phandle, 0, IPL_SDMMC, 0,
	    sdhc_intr, &sc->sc_base, device_xname(self));
	if (sc->sc_ih == NULL) {
		aprint_error_dev(self, "couldn't establish interrupt on %s\n",
		    intrstr);
		return;
	}
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);

	rk_dwcmshc_vendor_init(sc);

	error = sdhc_host_found(&sc->sc_base, sc->sc_bst, sc->sc_bsh,
	    sc->sc_bsz);
	if (error != 0) {
		aprint_error_dev(self,
		    "couldn't initialize host, error = %d\n", error);
		return;
	}
}

CFATTACH_DECL_NEW(rk_dwcmshc, sizeof(struct rk_dwcmshc_softc),
	rk_dwcmshc_match, rk_dwcmshc_attach, NULL, NULL);
