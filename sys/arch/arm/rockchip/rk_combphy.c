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
 * RK3568 NanoEng combo PHY driver (SATA and PCIe lane modes).
 *
 * The RK3568 SATA controllers (sata0 @ 0xfc000000, sata1 @ 0xfc400000)
 * and the pcie2x1 controller (@ 0xfe260000, M.2 slot) share their lanes
 * with NanoEng combo PHYs that must be switched to the consumer's mode,
 * given a 100 MHz reference clock and a powered PIPE power domain before
 * the controller can see anything on the lane.
 *
 * There is no register-accurate rk3568 CRU upstream (our fixed-rate
 * stub does not program registers), so this driver owns the full
 * pipe-domain power-up sequence.  Offsets, values and ordering mirror
 * the vendor bare-metal SDK (Phytium standalone SDK,
 * drivers/sata/fsata/fsata_rk3568.c) which was validated on this
 * board, and which in turn mirrors Linux
 * phy-rockchip-naneng-combphy.c:
 *
 *   1. PD_PIPE: ensure powered on and clear a stale BIU_PIPE idle
 *      request (a leftover idle blocks all pipe-domain APB access)
 *   2. PMUCRU: align PPLL to the Linux config and select the pciephy
 *      refclk mux to the PPLL ph0 (100 MHz) path with divider 1.
 *      U-Boot leaves a /2 setting (50 MHz) which starves the PHY PLL
 *      and yields zero COMINIT responses
 *   3. CRU: open the PIPE/SATA clock gates and verify the pmalive/
 *      rxoob parent gates (a closed parent gate kills the COMINIT
 *      detection window, SSTS reads 0 forever)
 *   4. combo PHY GRF/mmio config for SATA: phy_grf con0..con3,
 *      pipe_grf pipe_con0, CTLE enable, 50/43.5 ohm TX/RX termination,
 *      SSC tuning for the 100 MHz reference
 *   5. deassert the pipe PHY soft resets, then settle 500 ms before
 *      the consumer (ahcisata) resets the HBA; a COMRESET issued too
 *      early gets zero response
 *   6. disable the USB3 OTG port that shares the lane (this board
 *      dedicates both lanes to SATA) and read-only check of the MPHY
 *      mux straps (GPIO0_A5/A6 must be low for the SATA lane wiring)
 *
 * The SATA controller soft resets in CRU SOFTRST_CON8 are deliberately
 * NOT touched: neither U-Boot nor Linux ever asserts them and doing so
 * can break the combphy link.  The AHCI core resets the HBA itself via
 * GHC.HR.
 *
 * Attaches before the sata nodes in the board DTS (fdt children attach
 * in document order) so the phy controller is registered by the time
 * ahcisata_fdt acquires it.
 */

#include <sys/cdefs.h>

__KERNEL_RCSID(0, "$NetBSD");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/endian.h>
#include <sys/kmem.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>

/* PHY type argument values (dt-bindings/phy/phy.h) */
#define	PHY_TYPE_SATA			1
#define	PHY_TYPE_PCIE			2

/*
 * Physical base addresses.  Everything below sits inside the
 * 0xfdc00000-0xffe00000 window of the rk_platform MMIO devmap, so
 * plain bus_space_map() calls on the fdt bus tag work.
 */
#define	RK3568_PMUCRU_BASE		0xfdd00000
#define	RK3568_PMUCRU_SIZE		0x200
#define	RK3568_CRU_BASE			0xfdd20000
#define	RK3568_CRU_SIZE			0x1000
#define	RK3568_GPIO0_BASE		0xfdd60000
#define	RK3568_GPIO0_SIZE		0x100
#define	RK3568_PMU_BASE			0xfdd90000
#define	RK3568_PMU_SIZE			0x100
#define	RK3568_PIPE_GRF_BASE		0xfdc50000
#define	RK3568_COMBPHY0_BASE		0xfe820000
#define	RK3568_COMBPHY_STEP		0x10000

/* PMU (power domain) registers */
#define	PMU_BUS_IDLE_SFTCON0		0x050
#define	PMU_BUS_IDLE_ACK		0x060
#define	PMU_BUS_IDLE_ST			0x068
#define	PMU_PWR_DWN_ST			0x098
#define	PMU_PWR_GATE_SFTCON		0x0a0
#define	PMU_BIU_PIPE			__BIT(11)
#define	PMU_PD_PIPE			__BIT(8)

/* PMU CRU registers (PPLL / pciephy ref clock) */
#define	PMUCRU_PPLL_CON0		0x000
#define	PMUCRU_PPLL_CON1		0x004
#define	PMUCRU_MODE_CON			0x080
#define	PMUCRU_CLKSEL_CON9		0x124	/* bit[3+idx*4] mux, bit[2:0+idx*4] div */
#define	PMUCRU_CLKGATE_CON2		0x188
#define	PPLL_CON0_100M			0x6064
#define	PPLL_CON1_100M			0x1481
#define	PLL_MODE_NORMAL			1

/* Main CRU: clkgate region at 0x300, softrst region at 0x400 */
#define	CRU_CLKGATE_CON(n)		(0x300 + (n) * 4)
#define	CRU_SOFTRST_CON(n)		(0x400 + (n) * 4)
#define	CRU_CLKSEL_CON29		0x174	/* aclk_pipe mux / pclk_pipe div */
#define	CLKGATE_CON10_PIPE_MASK		0x8f	/* aclk/pclk_pipe family */
#define	CLKGATE_CON11_SATA_MASK		0xf	/* per-controller, << idx*4 */
#define	CLKGATE_CON34_PIPEPHY_MASK	0x10	/* pclk_pipephyN, << idx */
#define	CLKGATE_CON35_SATA_PARENTS	(__BIT(6) | __BIT(13))	/* gpll_20m/cpll_50m */
#define	SOFTRST_CON28_PIPEPHY_APB	0x10	/* presetn, << idx*2 */
#define	SOFTRST_CON28_PIPEPHY_CORE	0x20	/* resetn, << idx*2 */

struct rk_combphy_softc {
	device_t		sc_dev;
	int			sc_phandle;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;		/* combphy mmio */
	bus_space_handle_t	sc_pipegrf_bsh;
	bus_space_handle_t	sc_phygrf_bsh;
	bus_space_handle_t	sc_cru_bsh;
	bus_space_handle_t	sc_pmucru_bsh;
	bus_space_handle_t	sc_pmu_bsh;
	bus_space_handle_t	sc_gpio0_bsh;
	u_int			sc_idx;		/* 0 = combphy0 (sata0), 1 = combphy1 */
	bool			sc_enabled;
};

/* Per-consumer handle: the acquire callback carries the requested lane
 * mode (PHY_TYPE_*) to the enable callback. */
struct rk_combphy_ref {
	struct rk_combphy_softc	*ref_sc;
	u_int			ref_type;
};

static int	rk_combphy_match(device_t, cfdata_t, void *);
static void	rk_combphy_attach(device_t, device_t, void *);

static void *	rk_combphy_acquire(device_t, const void *, size_t);
static void	rk_combphy_release(device_t, void *);
static int	rk_combphy_enable(device_t, void *, bool);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "rockchip,rk3568-naneng-combphy" },
	DEVICE_COMPAT_EOL
};

static const struct fdtbus_phy_controller_func rk_combphy_funcs = {
	.acquire = rk_combphy_acquire,
	.release = rk_combphy_release,
	.enable = rk_combphy_enable,
};

CFATTACH_DECL_NEW(rkcombphy, sizeof(struct rk_combphy_softc),
    rk_combphy_match, rk_combphy_attach, NULL, NULL);

static uint32_t
rk_combphy_rd4(struct rk_combphy_softc *sc, bus_space_handle_t bsh,
    bus_size_t off)
{
	return bus_space_read_4(sc->sc_bst, bsh, off);
}

static void
rk_combphy_wr4(struct rk_combphy_softc *sc, bus_space_handle_t bsh,
    bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->sc_bst, bsh, off, val);
}

/* Rockchip hiword write-enable register update. */
static void
rk_combphy_clrset(struct rk_combphy_softc *sc, bus_space_handle_t bsh,
    bus_size_t off, uint32_t mask, uint32_t val)
{
	bus_space_write_4(sc->sc_bst, bsh, off,
	    ((mask & 0xffff) << 16) | (val & mask));
}

static int
rk_combphy_map_prop(struct rk_combphy_softc *sc, const char *prop,
    bus_space_handle_t *bshp)
{
	bus_addr_t addr;
	bus_size_t size;
	int phandle;

	phandle = fdtbus_get_phandle(sc->sc_phandle, prop);
	if (phandle < 0)
		return ENOENT;
	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0)
		return EINVAL;
	return bus_space_map(sc->sc_bst, addr, size, 0, bshp);
}

static void
rk_combphy_pd_pipe_cycle(struct rk_combphy_softc *sc)
{
	bus_space_handle_t pmu = sc->sc_pmu_bsh;
	u_int n;

	/* Full PD_PIPE power cycle.  Linux's power domain driver powers the
	 * domain off and back on when the AHCI device is first probed, which
	 * gives the combo PHY a clean power-on reset before calibration; a
	 * keep-alive-only approach leaves a stale PHY state that fails to
	 * calibrate. */

	/* Request the bus idle, wait for ack and idle. */
	rk_combphy_clrset(sc, pmu, PMU_BUS_IDLE_SFTCON0, PMU_BIU_PIPE,
	    PMU_BIU_PIPE);
	for (n = 0; n < 100; n++) {
		if (rk_combphy_rd4(sc, pmu, PMU_BUS_IDLE_ACK) & PMU_BIU_PIPE)
			break;
		delay(100);
	}
	for (n = 0; n < 100; n++) {
		if (rk_combphy_rd4(sc, pmu, PMU_BUS_IDLE_ST) & PMU_BIU_PIPE)
			break;
		delay(100);
	}

	/* Power off, wait for the power-down state. */
	rk_combphy_clrset(sc, pmu, PMU_PWR_GATE_SFTCON, PMU_PD_PIPE,
	    PMU_PD_PIPE);
	for (n = 0; n < 100; n++) {
		if (rk_combphy_rd4(sc, pmu, PMU_PWR_DWN_ST) & PMU_PD_PIPE)
			break;
		delay(100);
	}
	delay(10 * 1000);

	/* Power on again. */
	rk_combphy_clrset(sc, pmu, PMU_PWR_GATE_SFTCON, PMU_PD_PIPE, 0);
	for (n = 0; n < 100; n++) {
		if ((rk_combphy_rd4(sc, pmu, PMU_PWR_DWN_ST) & PMU_PD_PIPE) == 0)
			break;
		delay(100);
	}

	/* Release the bus idle request. */
	rk_combphy_clrset(sc, pmu, PMU_BUS_IDLE_SFTCON0, PMU_BIU_PIPE, 0);
	for (n = 0; n < 100; n++) {
		if ((rk_combphy_rd4(sc, pmu, PMU_BUS_IDLE_ACK) & PMU_BIU_PIPE) == 0)
			break;
		delay(100);
	}
	for (n = 0; n < 100; n++) {
		if ((rk_combphy_rd4(sc, pmu, PMU_BUS_IDLE_ST) & PMU_BIU_PIPE) == 0)
			break;
		delay(100);
	}
}

static void
rk_combphy_set_refclk(struct rk_combphy_softc *sc)
{
	bus_space_handle_t pmucru = sc->sc_pmucru_bsh;
	const u_int div_shift = sc->sc_idx * 4;
	const u_int mux_shift = 3 + sc->sc_idx * 4;

	/* Align PPLL to the Linux config if firmware left something else;
	 * the refclk path is PPLL -> ppl_ph0 (100 MHz) -> clk_pciephyN. */
	if (rk_combphy_rd4(sc, pmucru, PMUCRU_PPLL_CON0) != PPLL_CON0_100M ||
	    rk_combphy_rd4(sc, pmucru, PMUCRU_PPLL_CON1) != PPLL_CON1_100M ||
	    (rk_combphy_rd4(sc, pmucru, PMUCRU_MODE_CON) & 0x3) != PLL_MODE_NORMAL) {
		aprint_normal_dev(sc->sc_dev, "aligning PPLL to 100 MHz config\n");
		rk_combphy_clrset(sc, pmucru, PMUCRU_PPLL_CON0, 0xffff, PPLL_CON0_100M);
		rk_combphy_clrset(sc, pmucru, PMUCRU_PPLL_CON1, 0xffff, PPLL_CON1_100M);
		rk_combphy_clrset(sc, pmucru, PMUCRU_MODE_CON, 0x3, PLL_MODE_NORMAL);
		delay(50 * 1000);
	}

	/* Mux to the div path (ppl_ph0) and divide by 1; U-Boot leaves a
	 * /2 setting that starves the PHY PLL (50 MHz reference). */
	rk_combphy_clrset(sc, pmucru, PMUCRU_CLKSEL_CON9,
	    (0x7 << div_shift) | (0x1 << mux_shift), (0x1 << mux_shift));

	/* Open all three lanes' ref clock gates (div and osc paths); the
	 * mux below picks the div path.  U-Boot leaves CLKGATE_CON2 at 0
	 * and its SATA works, while selectively closing the osc gates
	 * (0x1500, the Linux runtime value) left the PHY PLL unlocked on
	 * this board. */
	rk_combphy_clrset(sc, pmucru, PMUCRU_CLKGATE_CON2, 0x3f80, 0x0000);
}

static void
rk_combphy_config_sata(struct rk_combphy_softc *sc)
{
	bus_space_handle_t cru = sc->sc_cru_bsh;
	bus_space_handle_t phygrf = sc->sc_phygrf_bsh;
	bus_space_handle_t pipegrf = sc->sc_pipegrf_bsh;
	bus_space_handle_t mmio = sc->sc_bsh;
	const u_int idx = sc->sc_idx;
	const uint32_t apb_rst = SOFTRST_CON28_PIPEPHY_APB << (idx * 2);
	const uint32_t core_rst = SOFTRST_CON28_PIPEPHY_CORE << (idx * 2);
	uint32_t val, pll9c, plla0;

	/* Make sure the PHY's APB interface is running (needed for the
	 * mmio writes below). */
	rk_combphy_clrset(sc, cru, CRU_SOFTRST_CON(28), apb_rst, 0);

	/* Hold the PHY core in reset while it is being configured: its
	 * internal calibration and PLL only (re)start when the core reset
	 * is released AFTER configuration.  U-Boot may leave the PHY
	 * unconfigured but already running, so the assert cannot be
	 * skipped (mirrors the Linux probe-time assert of the "phy"
	 * reset). */
	rk_combphy_clrset(sc, cru, CRU_SOFTRST_CON(28), core_rst, core_rst);
	delay(100);

	/* CRU clock gates: pipe family, this controller's sata clocks and
	 * its pipephy apb clock.  Gate polarity: 1 = off, 0 = on. */
	rk_combphy_clrset(sc, cru, CRU_CLKGATE_CON(10), CLKGATE_CON10_PIPE_MASK, 0);
	rk_combphy_clrset(sc, cru, CRU_CLKGATE_CON(11),
	    CLKGATE_CON11_SATA_MASK << (idx * 4), 0);
	rk_combphy_clrset(sc, cru, CRU_CLKGATE_CON(34),
	    CLKGATE_CON34_PIPEPHY_MASK << idx, 0);

	/* pmalive/rxoob parent gates (gpll_20m / cpll_50m): when closed,
	 * the COMINIT detection window never runs and SSTS reads 0. */
	val = rk_combphy_rd4(sc, cru, CRU_CLKGATE_CON(35));
	if ((val & CLKGATE_CON35_SATA_PARENTS) != 0)
		rk_combphy_clrset(sc, cru, CRU_CLKGATE_CON(35),
		    CLKGATE_CON35_SATA_PARENTS, 0);

	/* PHY GRF SATA configuration. */
	rk_combphy_clrset(sc, phygrf, 0x00, 0xffff, 0x0119);
	rk_combphy_clrset(sc, phygrf, 0x04, (0x3 << 13) | __BIT(6),
	    (0x2 << 13) | __BIT(6));	/* phy_clk_sel = 100 MHz */
	rk_combphy_clrset(sc, phygrf, 0x08, 0xffff, 0x80c3);
	rk_combphy_clrset(sc, phygrf, 0x0c, 0xffff, 0x4407);

	/* PIPE GRF: SATA lane mux. */
	rk_combphy_clrset(sc, pipegrf, 0x000, 0xffff, 0x2220);

	/* PHY mmio tuning: CTLE enable, 50/43.5 ohm TX/RX termination,
	 * SSC for the 100 MHz reference. */
	val = rk_combphy_rd4(sc, mmio, 0x38);
	rk_combphy_wr4(sc, mmio, 0x38, val | __BIT(0));
	rk_combphy_wr4(sc, mmio, 0x18, 0x8f);
	val = rk_combphy_rd4(sc, mmio, 0x7c);
	rk_combphy_wr4(sc, mmio, 0x7c, (val & ~(0xf << 4)) | (0x5 << 4));

	/* Config done, release the PHY core reset: this is what starts the
	 * internal calibration and PLL. */
	rk_combphy_clrset(sc, cru, CRU_SOFTRST_CON(28), core_rst, 0);
	delay(1000);

	/* PLL settle: a COMRESET issued too early gets zero response. */
	delay(500 * 1000);

	pll9c = rk_combphy_rd4(sc, mmio, 0x9c);
	plla0 = rk_combphy_rd4(sc, mmio, 0xa0);
	aprint_normal_dev(sc->sc_dev,
	    "SATA lane configured (PLL 0x9c=0x%08x 0xa0=0x%08x)\n", pll9c, plla0);
	if (pll9c == 0 && plla0 == 0)
		aprint_error_dev(sc->sc_dev, "PHY PLL looks unlocked\n");

	/* Post-config readback: if the PHY APB is alive, the registers we
	 * wrote read back non-zero; 0x18 was just written with 0x8f. */
	aprint_normal_dev(sc->sc_dev,
	    "readback: ppll=0x%08x/0x%08x/0x%08x clksel9=0x%08x clkgate2=0x%08x "
	    "con10=0x%08x con11=0x%08x con34=0x%08x con35=0x%08x\n",
	    rk_combphy_rd4(sc, sc->sc_pmucru_bsh, PMUCRU_PPLL_CON0),
	    rk_combphy_rd4(sc, sc->sc_pmucru_bsh, PMUCRU_PPLL_CON1),
	    rk_combphy_rd4(sc, sc->sc_pmucru_bsh, PMUCRU_MODE_CON),
	    rk_combphy_rd4(sc, sc->sc_pmucru_bsh, PMUCRU_CLKSEL_CON9),
	    rk_combphy_rd4(sc, sc->sc_pmucru_bsh, PMUCRU_CLKGATE_CON2),
	    rk_combphy_rd4(sc, cru, CRU_CLKGATE_CON(10)),
	    rk_combphy_rd4(sc, cru, CRU_CLKGATE_CON(11)),
	    rk_combphy_rd4(sc, cru, CRU_CLKGATE_CON(34)),
	    rk_combphy_rd4(sc, cru, CRU_CLKGATE_CON(35)));
	aprint_normal_dev(sc->sc_dev,
	    "readback: phygrf=0x%08x/0x%08x/0x%08x/0x%08x pipegrf=0x%08x "
	    "mmio 18=0x%08x 38=0x%08x 7c=0x%08x\n",
	    rk_combphy_rd4(sc, phygrf, 0x00), rk_combphy_rd4(sc, phygrf, 0x04),
	    rk_combphy_rd4(sc, phygrf, 0x08), rk_combphy_rd4(sc, phygrf, 0x0c),
	    rk_combphy_rd4(sc, pipegrf, 0x000),
	    rk_combphy_rd4(sc, mmio, 0x18), rk_combphy_rd4(sc, mmio, 0x38),
	    rk_combphy_rd4(sc, mmio, 0x7c));
	for (int off = 0; off < 0x80; off += 16) {
		aprint_normal_dev(sc->sc_dev,
		    "mmio %02x-%02x: %08x %08x %08x %08x\n",
		    off, off + 12,
		    rk_combphy_rd4(sc, mmio, off),
		    rk_combphy_rd4(sc, mmio, off + 4),
		    rk_combphy_rd4(sc, mmio, off + 8),
		    rk_combphy_rd4(sc, mmio, off + 12));
	}
	aprint_normal_dev(sc->sc_dev,
	    "pmu: idle_req=0x%08x idle_ack=0x%08x idle_st=0x%08x "
	    "pwr_st=0x%08x gate=0x%08x  con28=0x%08x\n",
	    rk_combphy_rd4(sc, sc->sc_pmu_bsh, PMU_BUS_IDLE_SFTCON0),
	    rk_combphy_rd4(sc, sc->sc_pmu_bsh, PMU_BUS_IDLE_ACK),
	    rk_combphy_rd4(sc, sc->sc_pmu_bsh, PMU_BUS_IDLE_ST),
	    rk_combphy_rd4(sc, sc->sc_pmu_bsh, PMU_PWR_DWN_ST),
	    rk_combphy_rd4(sc, sc->sc_pmu_bsh, PMU_PWR_GATE_SFTCON),
	    rk_combphy_rd4(sc, cru, CRU_SOFTRST_CON(28)));

	/* This board wires the lane to SATA; keep the shared USB3 OTG port
	 * disabled (full 16-bit field write, matching Linux). */
	rk_combphy_clrset(sc, pipegrf, 0x104 + idx * 0x40, 0xffff, 0x0181);

	/* aclk_pipe = gpll_400m, pclk_pipe = aclk_pipe/4. */
	rk_combphy_clrset(sc, cru, CRU_CLKSEL_CON29, 0xf3, 0x30);
}

/*
 * PCIe mode (combphy2, the pcie2x1 M.2 lane), mirroring the vendor
 * bare-metal SDK fdwpcie FDwPcieCombphy2Init (validated on this board;
 * KI-008/009), which in turn matches Linux phy-rockchip-naneng-combphy
 * rk3568 PCIe mode.  Unlike the SATA path there is no power-domain cycle
 * and no reset assert: bare-metal boots this lane from a cold controller
 * with the exact sequence below (PPLL/refclk from rk_combphy_set_refclk,
 * then GRF mode words, mmio PLL tuning, softreset release last).
 */
static void
rk_combphy_config_pcie(struct rk_combphy_softc *sc)
{
	bus_space_handle_t cru = sc->sc_cru_bsh;
	bus_space_handle_t phygrf = sc->sc_phygrf_bsh;
	bus_space_handle_t mmio = sc->sc_bsh;
	const u_int idx = sc->sc_idx;
	const uint32_t apb_rst = SOFTRST_CON28_PIPEPHY_APB << (idx * 2);
	const uint32_t core_rst = SOFTRST_CON28_PIPEPHY_CORE << (idx * 2);
	uint32_t val, pll9c, plla0;

	/* Make sure the PHY's APB interface is running, and hold the PHY
	 * core in reset while it is being configured.  Bare-metal
	 * (fdwpcie) skips the assert because its cold PHY never ran; in
	 * our boot flow U-Boot may have left the unconfigured PHY
	 * running, and its PLL then never locks - the same reason the
	 * SATA path asserts the core reset. */
	rk_combphy_clrset(sc, cru, CRU_SOFTRST_CON(28), apb_rst, 0);
	rk_combphy_clrset(sc, cru, CRU_SOFTRST_CON(28), core_rst, core_rst);
	delay(100);

	rk_combphy_set_refclk(sc);

	/* pciephy2 refclk mux: mirror fdwpcie's measured value.  fdwpcie
	 * clears mux[11] (selecting the osc0 path) and its board tests
	 * enumerate the M.2 NVMe; the div path with mux=1 (the SATA lane
	 * value) demonstrably leaves this PHY's PLL unlocked. */
	rk_combphy_clrset(sc, sc->sc_pmucru_bsh, PMUCRU_CLKSEL_CON9,
	    0x1 << 11, 0);

	/* CRU clock gates: pipe family (aclk/pclk_pipe), this lane's
	 * pclk_pipephyN APB clock.  Gate polarity: 1 = off, 0 = on. */
	rk_combphy_clrset(sc, cru, CRU_CLKGATE_CON(10), CLKGATE_CON10_PIPE_MASK, 0);
	rk_combphy_clrset(sc, cru, CRU_CLKGATE_CON(34),
	    CLKGATE_CON34_PIPEPHY_MASK << idx, 0);

	/* PIPE PHY GRF PCIe mode words + pipe_clk_100m (con1 bits[14:13] = 2).
	 * Note: unlike SATA there is no main-PIPE_GRF write in the PCIe
	 * mode - Linux con0-3_for_pcie (and the FreeBSD fork) only touch
	 * the per-lane pipe-phy GRF. */
	rk_combphy_clrset(sc, phygrf, 0x00, 0xffff, 0x1000);
	rk_combphy_clrset(sc, phygrf, 0x04, 0xffff, 0x0000);
	rk_combphy_clrset(sc, phygrf, 0x08, 0xffff, 0x0101);
	rk_combphy_clrset(sc, phygrf, 0x0c, 0xffff, 0x0200);
	rk_combphy_clrset(sc, phygrf, 0x04, 0x3 << 13, 0x2 << 13);

	/* PHY mmio: 100 MHz PCIe PLL parameters (SSC downward, KVCO,
	 * rx_trim, su_trim quadruplet) and the Tx-detect-Rx errata bit. */
	val = rk_combphy_rd4(sc, mmio, 0x7c);
	rk_combphy_wr4(sc, mmio, 0x7c, (val & ~(0x3 << 4)) | (0x1 << 4));
	rk_combphy_wr4(sc, mmio, 0x74, 0xc0);
	val = rk_combphy_rd4(sc, mmio, 0x80);
	rk_combphy_wr4(sc, mmio, 0x80, (val & ~(0x7 << 2)) | (0x2 << 2));
	rk_combphy_wr4(sc, mmio, 0x6c, 0x4c);
	rk_combphy_wr4(sc, mmio, 0x28, 0x90);
	rk_combphy_wr4(sc, mmio, 0x2c, 0x43);
	rk_combphy_wr4(sc, mmio, 0x30, 0x88);
	rk_combphy_wr4(sc, mmio, 0x34, 0x56);
	val = rk_combphy_rd4(sc, mmio, 0x64);
	rk_combphy_wr4(sc, mmio, 0x64, val | __BIT(5));

	/* Config done, release the PHY core reset: this starts the
	 * internal calibration and PLL. */
	rk_combphy_clrset(sc, cru, CRU_SOFTRST_CON(28), core_rst, 0);
	delay(1000);

	/* PLL settle like the SATA path: a link training started too
	 * early sees no echo. */
	delay(500 * 1000);

	pll9c = rk_combphy_rd4(sc, mmio, 0x9c);
	plla0 = rk_combphy_rd4(sc, mmio, 0xa0);
	aprint_normal_dev(sc->sc_dev,
	    "PCIe lane configured (PLL 0x98=0x%08x 0x9c=0x%08x 0xa0=0x%08x)\n",
	    rk_combphy_rd4(sc, mmio, 0x98), pll9c, plla0);
	if (pll9c == 0 && plla0 == 0)
		aprint_error_dev(sc->sc_dev, "PHY PLL looks unlocked\n");
}

static void *
rk_combphy_acquire(device_t dev, const void *data, size_t datalen)
{
	struct rk_combphy_softc * const sc = device_private(dev);
	struct rk_combphy_ref *ref;
	u_int phy_type;

	if (datalen != 4) {
		aprint_error_dev(dev, "unexpected #phy-cells\n");
		return NULL;
	}
	phy_type = be32dec(data);
	if (phy_type != PHY_TYPE_SATA && phy_type != PHY_TYPE_PCIE) {
		aprint_error_dev(dev,
		    "phy type %u not supported (SATA/PCIe only)\n", phy_type);
		return NULL;
	}

	ref = kmem_alloc(sizeof(*ref), KM_SLEEP);
	ref->ref_sc = sc;
	ref->ref_type = phy_type;
	return ref;
}

static void
rk_combphy_release(device_t dev, void *priv)
{
	struct rk_combphy_ref * const ref = priv;

	kmem_free(ref, sizeof(*ref));
}

static int
rk_combphy_enable(device_t dev, void *priv, bool enable)
{
	struct rk_combphy_softc * const sc =
	    ((struct rk_combphy_ref *)priv)->ref_sc;
	const u_int type = ((struct rk_combphy_ref *)priv)->ref_type;

	if (!enable || sc->sc_enabled)
		return 0;

	if (type == PHY_TYPE_PCIE) {
		rk_combphy_config_pcie(sc);
		sc->sc_enabled = true;
		return 0;
	}

	/* If firmware (U-Boot "scsi scan" preboot on this board) already
	 * initialized and calibrated the PHY, keep that state: the registers
	 * read back identical to our own init, but re-running the power
	 * cycle / reset sequence here tears down the working link and the
	 * PHY does not re-calibrate under NetBSD. */
	if (rk_combphy_rd4(sc, sc->sc_bsh, 0x98) == 0x40 &&
	    rk_combphy_rd4(sc, sc->sc_bsh, 0x9c) != 0) {
		aprint_normal_dev(sc->sc_dev,
		    "SATA lane already calibrated, keeping firmware state\n");
		sc->sc_enabled = true;
		return 0;
	}

	rk_combphy_pd_pipe_cycle(sc);
	rk_combphy_set_refclk(sc);
	rk_combphy_config_sata(sc);

	sc->sc_enabled = true;
	return 0;
}

static int
rk_combphy_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rk_combphy_attach(device_t parent, device_t self, void *aux)
{
	struct rk_combphy_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;
	int error;

	sc->sc_dev = self;
	sc->sc_phandle = phandle;
	sc->sc_bst = faa->faa_bst;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}
	if (addr < RK3568_COMBPHY0_BASE ||
	    addr >= RK3568_COMBPHY0_BASE + 3 * RK3568_COMBPHY_STEP) {
		aprint_error(": unsupported lane (0x%08x)\n", (uint32_t)addr);
		return;
	}
	sc->sc_idx = (u_int)(addr - RK3568_COMBPHY0_BASE) / RK3568_COMBPHY_STEP;

	if ((error = bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh)) != 0) {
		aprint_error(": couldn't map PHY registers (%d)\n", error);
		return;
	}
	if ((error = rk_combphy_map_prop(sc, "rockchip,pipe-grf",
	    &sc->sc_pipegrf_bsh)) != 0) {
		aprint_error(": couldn't map %s (%d)\n", "rockchip,pipe-grf", error);
		return;
	}
	if ((error = rk_combphy_map_prop(sc, "rockchip,pipe-phy-grf",
	    &sc->sc_phygrf_bsh)) != 0) {
		aprint_error(": couldn't map %s (%d)\n", "rockchip,pipe-phy-grf", error);
		return;
	}
	if ((error = bus_space_map(sc->sc_bst, RK3568_CRU_BASE, RK3568_CRU_SIZE,
	    0, &sc->sc_cru_bsh)) != 0 ||
	    (error = bus_space_map(sc->sc_bst, RK3568_PMUCRU_BASE,
	    RK3568_PMUCRU_SIZE, 0, &sc->sc_pmucru_bsh)) != 0 ||
	    (error = bus_space_map(sc->sc_bst, RK3568_PMU_BASE, RK3568_PMU_SIZE,
	    0, &sc->sc_pmu_bsh)) != 0 ||
	    (error = bus_space_map(sc->sc_bst, RK3568_GPIO0_BASE,
	    RK3568_GPIO0_SIZE, 0, &sc->sc_gpio0_bsh)) != 0) {
		aprint_error(": couldn't map system controller (%d)\n", error);
		return;
	}

	aprint_naive("\n");
	aprint_normal(": RK3568 NanoEng combo PHY (lane %u)\n", sc->sc_idx);

	fdtbus_register_phy_controller(self, phandle, &rk_combphy_funcs);

	/* Read-only check of the MPHY mux straps: GPIO0_A5/A6 low selects
	 * the SATA lane wiring on this board. */
	if ((rk_combphy_rd4(sc, sc->sc_gpio0_bsh, 0x00) & (0x3 << 5)) != 0)
		aprint_error_dev(self,
		    "MPHY mux straps high, SATA lane not selected\n");
}
