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
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.  IN NO
 * EVENT SHALL THE FOUNDATION OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * RK3568 PCIe3 NanoEng PHY ("rockchip,rk3568-pcie3-phy" @ 0xfe8c0000),
 * serving the pcie3x2 root complex (the board's PCIe x4 slot).
 *
 * The PHY has internal SRAM that must be loaded with a firmware blob
 * before the link trains (see rk_pcie30phy_fw.c).  The init sequence
 * mirrors the vendor bare-metal SDK fdwpcie FDwPcie30PhyInit /
 * FDwPcie30PhyCalibrate (validated on this board, KI-008/009), which in
 * turn matches Linux phy-rockchip-snps-pcie3.c rk3568:
 *
 *   first enable (fdwpcie FDwPcie30PhyInit; called with the consumer
 *   controller held in soft reset):
 *     - open the refclk M/N and PCLK APB clock gates
 *     - assert SRST_PCIE30PHY, release PMA output clamp (CON9),
 *       clear sdram_ld_done / sdram_bypass (CON4)
 *     - deassert SRST_PCIE30PHY, wait for SRAM_INIT_DONE (STATUS0 bit14)
 *     - map the SRAM via CON9 and burn the 8192-entry firmware,
 *       unmap, set sdram_ld_done
 *
 *   second enable (fdwpcie FDwPcie30PhyCalibrate; the consumer calls it
 *   after releasing the controller soft reset, because the PHY only
 *   reaches its ready state then):
 *     - phase 1: SRAM_INIT_DONE set
 *     - phase 2: stable READY bit pattern with no TRANSIT bits; returning
 *       from the intermediate state leaves the LNKCAP speed vector unset
 *       and training stuck at Config.Linkwidth (fdwpcie measurement)
 */

#include <sys/cdefs.h>

__KERNEL_RCSID(0, "$NetBSD");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>

#include <arm/rockchip/rk_pcie30phy.h>

/*
 * Physical base addresses inside the rk_platform MMIO devmap window.
 */
#define	RK3568_CRU_BASE			0xfdd20000
#define	RK3568_CRU_SIZE			0x1000
#define	RK3568_PMUCRU_BASE		0xfdd00000
#define	RK3568_PMUCRU_SIZE		0x200

/* Main CRU: clkgate region at 0x300, softrst region at 0x400. */
#define	CRU_CLKGATE_CON(n)		(0x300 + (n) * 4)
#define	CRU_SOFTRST_CON(n)		(0x400 + (n) * 4)
#define	CLKGATE_CON33_PCIE30PHY		__BIT(8)	/* PCLK_PCIE30PHY */
#define	SRST_PCIE30PHY			446		/* bank 27 bit 14 */

/* PMU CRU: clkgate region at 0x180. */
#define	PMUCRU_CLKGATE_CON2		0x188
#define	 PCIE30PHY_REF_MN_GATES		(__BIT(13) | __BIT(14))

/* pcie30phy GRF registers (hiword-write-enable encoded in the value). */
#define	PCIE30PHY_CON4			0x10
#define	PCIE30PHY_CON9			0x24
#define	PCIE30PHY_STATUS0		0x80
#define	 CON4_SDRAM_LD_DONE		__BIT(14)
#define	 CON4_SDRAM_LD_DONE_EN		__BIT(30)
#define	 CON4_SDRAM_BYPASS		__BIT(13)
#define	 CON4_SDRAM_BYPASS_EN		__BIT(29)
#define	 CON9_OCM_CLK_SEL		__BITS(9,8)
#define	 CON9_OCM_ADDR_SEL		__BIT(15)
#define	 CON9_OCM_WRITE_SEL		__BIT(24)
#define	 CON9_CLAMP_RELEASE		(__BIT(15) | __BIT(31))
#define	STATUS0_SRAM_INIT_DONE		__BIT(14)
/* Stable-ready pattern (fdwpcie): READY bits set, TRANSIT bits clear. */
#define	STATUS0_READY_MASK		(__BIT(27) | __BIT(26) | __BIT(25) | \
					 __BIT(16) | __BIT(14))
#define	STATUS0_TRANSIT_MASK		(__BIT(30) | __BIT(23) | __BIT(19) | \
					 __BIT(18) | __BIT(17))

struct rk_pcie30phy_softc {
	device_t		sc_dev;
	int			sc_phandle;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;		/* PHY mmio (0xfe8c0000) */
	bus_space_handle_t	sc_phygrf_bsh;
	bus_space_handle_t	sc_cru_bsh;
	bus_space_handle_t	sc_pmucru_bsh;
	bool			sc_init_done;
	bool			sc_calibrated;
};

static int	rk_pcie30phy_match(device_t, cfdata_t, void *);
static void	rk_pcie30phy_attach(device_t, device_t, void *);

static void *	rk_pcie30phy_acquire(device_t, const void *, size_t);
static void	rk_pcie30phy_release(device_t, void *);
static int	rk_pcie30phy_enable(device_t, void *, bool);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "rockchip,rk3568-pcie3-phy" },
	DEVICE_COMPAT_EOL
};

static const struct fdtbus_phy_controller_func rk_pcie30phy_funcs = {
	.acquire = rk_pcie30phy_acquire,
	.release = rk_pcie30phy_release,
	.enable = rk_pcie30phy_enable,
};

CFATTACH_DECL_NEW(rk30pciephy, sizeof(struct rk_pcie30phy_softc),
    rk_pcie30phy_match, rk_pcie30phy_attach, NULL, NULL);

static uint32_t
rk_pcie30phy_rd4(struct rk_pcie30phy_softc *sc, bus_space_handle_t bsh,
    bus_size_t off)
{
	return bus_space_read_4(sc->sc_bst, bsh, off);
}

static void
rk_pcie30phy_wr4(struct rk_pcie30phy_softc *sc, bus_space_handle_t bsh,
    bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->sc_bst, bsh, off, val);
}

/* Rockchip hiword write-enable register update. */
static void
rk_pcie30phy_clrset(struct rk_pcie30phy_softc *sc, bus_space_handle_t bsh,
    bus_size_t off, uint32_t mask, uint32_t val)
{
	bus_space_write_4(sc->sc_bst, bsh, off,
	    ((mask & 0xffff) << 16) | (val & mask));
}

/* Assert (true) or release (false) SRST_PCIE30PHY. */
static void
rk_pcie30phy_reset(struct rk_pcie30phy_softc *sc, bool assert)
{
	const uint32_t mask = __BIT(SRST_PCIE30PHY % 16);

	rk_pcie30phy_clrset(sc, sc->sc_cru_bsh,
	    CRU_SOFTRST_CON(SRST_PCIE30PHY / 16), mask, assert ? mask : 0);
}

static int
rk_pcie30phy_init(struct rk_pcie30phy_softc *sc)
{
	bus_space_handle_t phygrf = sc->sc_phygrf_bsh;
	bus_space_handle_t mmio = sc->sc_bsh;
	uint32_t val;
	u_int n;

	/* Clock gates: refclk M/N (PMU CRU) and the PCLK APB clock. */
	rk_pcie30phy_clrset(sc, sc->sc_pmucru_bsh, PMUCRU_CLKGATE_CON2,
	    PCIE30PHY_REF_MN_GATES, 0);
	rk_pcie30phy_clrset(sc, sc->sc_cru_bsh, CRU_CLKGATE_CON(33),
	    CLKGATE_CON33_PCIE30PHY, 0);

	/* Hold the PHY in reset while the GRF is being configured; a cold
	 * controller never completes SRAM init without this restart. */
	rk_pcie30phy_reset(sc, true);
	delay(1);

	/* Release the PCIe PMA output clamp. */
	rk_pcie30phy_wr4(sc, phygrf, PCIE30PHY_CON9, CON9_CLAMP_RELEASE);

	/* sdram_ld_done = 0, sdram_bypass = 0. */
	rk_pcie30phy_wr4(sc, phygrf, PCIE30PHY_CON4,
	    CON4_SDRAM_LD_DONE_EN);
	rk_pcie30phy_wr4(sc, phygrf, PCIE30PHY_CON4,
	    CON4_SDRAM_BYPASS_EN);

	/* Release the reset and wait for the internal SRAM boot. */
	rk_pcie30phy_reset(sc, false);
	delay(5);

	val = 0;
	for (n = 0; n < 500; n++) {
		val = rk_pcie30phy_rd4(sc, phygrf, PCIE30PHY_STATUS0);
		if (val & STATUS0_SRAM_INIT_DONE)
			break;
		delay(100);
	}
	if ((val & STATUS0_SRAM_INIT_DONE) == 0) {
		aprint_error_dev(sc->sc_dev,
		    "SRAM_INIT_DONE timeout (STATUS0=0x%08x)\n", val);
		return EIO;
	}

	/* Map the instruction SRAM window and burn the firmware. */
	rk_pcie30phy_wr4(sc, phygrf, PCIE30PHY_CON9,
	    __SHIFTIN(3, CON9_OCM_CLK_SEL) | CON9_OCM_WRITE_SEL);
	for (n = 0; n < RK3568_PCIE30PHY_FW_ENTRIES; n++)
		rk_pcie30phy_wr4(sc, mmio, n * 4, rk3568_pcie30phy_fw[n]);
	rk_pcie30phy_wr4(sc, phygrf, PCIE30PHY_CON9,
	    __SHIFTIN(0, CON9_OCM_CLK_SEL) | CON9_OCM_WRITE_SEL);

	/* sdram_ld_done = 1: the PHY switches to the loaded firmware. */
	rk_pcie30phy_wr4(sc, phygrf, PCIE30PHY_CON4,
	    CON4_SDRAM_LD_DONE | CON4_SDRAM_LD_DONE_EN);
	delay(10);

	aprint_normal_dev(sc->sc_dev, "firmware loaded (%u entries)\n",
	    (u_int)RK3568_PCIE30PHY_FW_ENTRIES);
	return 0;
}

static int
rk_pcie30phy_calibrate(struct rk_pcie30phy_softc *sc)
{
	bus_space_handle_t phygrf = sc->sc_phygrf_bsh;
	uint32_t val = 0;
	u_int n;

	/* Phase 1: SRAM_INIT_DONE. */
	for (n = 0; n < 5000; n++) {
		val = rk_pcie30phy_rd4(sc, phygrf, PCIE30PHY_STATUS0);
		if (val & STATUS0_SRAM_INIT_DONE)
			break;
		delay(1000);
	}
	if ((val & STATUS0_SRAM_INIT_DONE) == 0) {
		aprint_error_dev(sc->sc_dev,
		    "calibrate: SRAM timeout (STATUS0=0x%08x)\n", val);
		return EIO;
	}

	/* Phase 2: stable ready state.  Returning during the transition
	 * leaves the root port LNKCAP speed vector unset and training
	 * stops at Config.Linkwidth (fdwpcie measurement). */
	for (n = 0; n < 5000; n++) {
		val = rk_pcie30phy_rd4(sc, phygrf, PCIE30PHY_STATUS0);
		if ((val & STATUS0_READY_MASK) == STATUS0_READY_MASK &&
		    (val & STATUS0_TRANSIT_MASK) == 0) {
			aprint_normal_dev(sc->sc_dev,
			    "calibrated (STATUS0=0x%08x)\n", val);
			return 0;
		}
		delay(1000);
	}
	aprint_error_dev(sc->sc_dev,
	    "calibrate timeout (STATUS0=0x%08x)\n", val);
	return EIO;
}

static int
rk_pcie30phy_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rk_pcie30phy_attach(device_t parent, device_t self, void *aux)
{
	struct rk_pcie30phy_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	int grf_phandle;
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
	if ((error = bus_space_map(sc->sc_bst, addr, size, 0,
	    &sc->sc_bsh)) != 0) {
		aprint_error(": couldn't map PHY registers (%d)\n", error);
		return;
	}

	grf_phandle = fdtbus_get_phandle(phandle, "rockchip,phy-grf");
	if (grf_phandle < 0 ||
	    fdtbus_get_reg(grf_phandle, 0, &addr, &size) != 0 ||
	    (error = bus_space_map(sc->sc_bst, addr, size, 0,
	    &sc->sc_phygrf_bsh)) != 0) {
		aprint_error(": couldn't map rockchip,phy-grf\n");
		return;
	}

	if ((error = bus_space_map(sc->sc_bst, RK3568_CRU_BASE,
	    RK3568_CRU_SIZE, 0, &sc->sc_cru_bsh)) != 0 ||
	    (error = bus_space_map(sc->sc_bst, RK3568_PMUCRU_BASE,
	    RK3568_PMUCRU_SIZE, 0, &sc->sc_pmucru_bsh)) != 0) {
		aprint_error(": couldn't map system controller (%d)\n", error);
		return;
	}

	/* Resolve the clocks so the stub reports them, and record the
	 * refclk rate for the log.  U-Boot left the 100 MHz refclk path
	 * gated on; the init opens the gates. */
	for (u_int i = 0; i < 3; i++) {
		struct clk *clk = fdtbus_clock_get_index(phandle, i);
		if (clk != NULL)
			clk_enable(clk);
	}

	aprint_naive("\n");
	aprint_normal(": RK3568 PCIe3 NanoEng PHY\n");

	fdtbus_register_phy_controller(self, faa->faa_phandle,
	    &rk_pcie30phy_funcs);
}

static void *
rk_pcie30phy_acquire(device_t dev, const void *data, size_t datalen)
{
	/* #phy-cells = <0>: the only consumer mode is the PCIe3 root. */
	if (datalen != 0) {
		aprint_error_dev(dev, "unexpected #phy-cells\n");
		return NULL;
	}
	return device_private(dev);
}

static void
rk_pcie30phy_release(device_t dev, void *priv)
{
}

static int
rk_pcie30phy_enable(device_t dev, void *priv, bool enable)
{
	struct rk_pcie30phy_softc * const sc = device_private(dev);
	int error;

	if (!enable)
		return 0;

	if (!sc->sc_init_done) {
		error = rk_pcie30phy_init(sc);
		if (error == 0)
			sc->sc_init_done = true;
		return error;
	}

	/* Second enable: the consumer released its controller soft reset;
	 * wait for the PHY to reach its ready state. */
	if (!sc->sc_calibrated) {
		error = rk_pcie30phy_calibrate(sc);
		if (error == 0)
			sc->sc_calibrated = true;
		return error;
	}

	return 0;
}
