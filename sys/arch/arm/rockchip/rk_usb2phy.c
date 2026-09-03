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
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.  IN NO EVENT
 * SHALL THE FOUNDATION OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * RK3568 USB2 PHY driver and USB domain bring-up (dwc3 xHCI lanes and
 * the EHCI/OHCI panel group).
 *
 * On this board the two USB3.0 Type-A connectors are wired to the two
 * dwc3 controllers: usbdrd30 @ 0xfcc00000 (OTG0 forced host, upper
 * connector) and usbhost30 @ 0xfd000000 (lower connector).  Both run
 * high-speed only through the usb2phy0 UTMI ports - the SuperSpeed
 * lanes are hardware-muxed to the SATA combphys and every OS on this
 * board runs these ports HS-only.  The rear-panel USB2.0 Type-A group
 * is driven by usb_host0/1_ehci+ohci through usb2phy1 (each EHCI line
 * behind an onboard CH334P hub).
 *
 * The fixed-rate CRU stub does not program registers and no reset
 * controller is registered, so this driver owns the full USB domain
 * power-up, mirroring register-exact sequences validated on this board
 * (Phytium standalone SDK cherryusb glue usb_hc_glue_rk3568.c /
 * usb_dc_glue_rk3568.c, and the FreeBSD rk_usb2phy.c GRF writes plus
 * the Linux working-state GRF truth recorded in known-issues KI-006 /
 * KI-012):
 *
 *   1. PD_PIPE: idempotent ensure-powered (clear a stale BIU_PIPE idle
 *      request, then make sure the domain is powered).  Never power-
 *      cycle: the SATA combphy lanes may already be up.
 *   2. CRU: open the USB3OTG clock gates (clkgate_con10 bits 8-10 and
 *      12-14), the USB2 host H/ARB gates are on by default, and the
 *      pmucru usbphy ref gates (pmu_clkgate_con2 bits 0-2, one per
 *      phy + clk_ref24m).  The pipe-family gates are left to
 *      rk_combphy.
 *   3. CRU: assert + release SRST_USB3OTG0/1 (softrst_con9 bits 4/5)
 *      and the USB2 host H/ARB/UTMI resets (softrst_con14 bits 4-9).
 *      ATF leaves the cores held in reset otherwise (GSNPSID reads 0
 *      for the dwc3s).  Also release-only the usb2phy1 POR / port
 *      resets (softrst_con29 bits 3-5) and its GRF pclk
 *      (softrst_con28 bit 11).
 *   4. VBUS: drive GPIO3_A1 (usb3_vbus_en, both USB3.0 Type-A ports)
 *      and GPIO3_A0 (usb2_vbus_en, panel group) high - the connectors
 *      are dead without it.  GPIO3 sits at 0xfe760000 (the RK3568 GPIO
 *      banks are not contiguous), SWPORT_DR_L @ 0x00 / SWPORT_DDR_L @
 *      0x08.
 *   5. Per-instance GRF: deassert the port suspend overrides with the
 *      measured working values - otg port CON0 @ 0x000 <- 0x0c00
 *      (host role incl. the iddig force-host bits), host port CON1 @
 *      0x004 <- 0x1d2 (Linux/FBSD/U-Boot agree on both), and enable
 *      the 480MHz clock output (CON2 @ 0x008 bit 4 <- 0) as the Linux
 *      inno-usb2phy driver does - the EHCI/OHCI group takes its UTMI
 *      clock from the usb2phy1 480m output.
 *
 * Two instances attach (usb2phy0 @ fe8a0000 / usb2phy1 @ fe8b0000,
 * distinguished only by their rockchip,usbgrf syscon - the port GRF
 * offsets are identical).  The domain-level sequence above runs once
 * on the first instance only: re-running the soft resets on the
 * second instance would bounce the already-attached dwc3 cores.
 *
 * Attaches before the dwc3 wrapper nodes in the board DTS (fdt
 * children attach in document order) so the phy controllers are
 * registered by the time dwc3_fdt acquires them.  The enable callback
 * re-applies the port GRF write (idempotent).
 */

#include <sys/cdefs.h>

__KERNEL_RCSID(0, "$NetBSD");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>

/*
 * Physical base addresses.  Everything below sits inside the
 * 0xfdc00000-0xffe00000 window of the rk_platform MMIO devmap, so
 * plain bus_space_map() calls on the fdt bus tag work.
 */
#define	RK3568_PMUCRU_BASE		0xfdd00000
#define	RK3568_PMUCRU_SIZE		0x200
#define	RK3568_CRU_BASE			0xfdd20000
#define	RK3568_CRU_SIZE			0x1000
#define	RK3568_PMU_BASE			0xfdd90000
#define	RK3568_PMU_SIZE			0x100
#define	RK3568_GPIO3_BASE		0xfe760000
#define	RK3568_GPIO3_SIZE		0x100

/* PMU (power domain) registers - same block as rk_combphy.c */
#define	PMU_BUS_IDLE_SFTCON0		0x050
#define	PMU_BUS_IDLE_ACK		0x060
#define	PMU_PWR_DWN_ST			0x098
#define	PMU_PWR_GATE_SFTCON		0x0a0
#define	PMU_BIU_PIPE			__BIT(11)
#define	PMU_PD_PIPE			__BIT(8)

/* Main CRU: clkgate region at 0x300, softrst region at 0x400 */
#define	CRU_CLKGATE_CON(n)		(0x300 + (n) * 4)
#define	CRU_SOFTRST_CON(n)		(0x400 + (n) * 4)
#define	CLKGATE_CON10_USB3OTG		0x7300	/* bits 8-10 otg0, 12-14 otg1 */
#define	CLKGATE_CON10_PIPE		0x0003	/* bits 0-1, in case rk_combphy hasn't run yet */
#define	SOFTRST_CON9_USB3OTG		0x30	/* bit4 otg0, bit5 otg1 */
#define	SOFTRST_CON14_USB2HOST		0x3f0	/* bits 4-9: h/arb/utmi host0 + host1 */
#define	SOFTRST_CON28_USB2PHY1_GRF	__BIT(11) /* P_USB2PHY1_GRF, release only */
#define	SOFTRST_CON29_USB2PHY1		0x38	/* bits 3-5: phy1 POR + usb2host0/1, release only */

/* PMU CRU */
#define	PMUCRU_CLKGATE_CON2		0x188
#define	CLKGATE_CON2_USBPHY		0x7	/* clk_ref24m, xin_osc0_usbphy0/1_g */

/* GPIO3 SWPORT low-pin registers */
#define	GPIO3_SWPORT_DR_L		0x00
#define	GPIO3_SWPORT_DDR_L		0x08
#define	GPIO3_VBUS_PINS			0x3	/* A0 usb2_vbus_en, A1 usb3_vbus_en */

/* Per-instance usb2phy GRF (0xfdca0000 / 0xfdca8000), hiword write-enable */
#define	USB2PHY_GRF_OTG_CON0		0x000	/* otg-port phy_sus */
#define	USB2PHY_GRF_HOST_CON1		0x004	/* host-port phy_sus */
#define	USB2PHY_GRF_CLKOUT_CON2		0x008	/* bit4 set = 480m output off */
#define	USB2PHY_OTG_HOST_VAL		0x0c00	/* measured Linux host-role state (KI-006) */
#define	USB2PHY_OTG_HOST_MASK		0xfff
#define	USB2PHY_HOST_VAL		0x1d2	/* U-Boot/Linux host deassert value */
#define	USB2PHY_HOST_MASK		0x1ff
#define	USB2PHY_CLKOUT_480M_OFF		__BIT(4)

struct rk_usb2phy_softc;

struct rk_usb2phy_port {
	struct rk_usb2phy_softc *sc;
	bus_size_t		off;
	uint32_t		val;
	uint32_t		mask;
	bool			on;
};

struct rk_usb2phy_softc {
	device_t		sc_dev;
	int			sc_phandle;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_grf_bsh;
	bus_space_handle_t	sc_cru_bsh;
	bus_space_handle_t	sc_pmucru_bsh;
	bus_space_handle_t	sc_pmu_bsh;
	bus_space_handle_t	sc_gpio3_bsh;
	struct rk_usb2phy_port	sc_ports[2];	/* 0 = otg, 1 = host */
};

static int	rk_usb2phy_match(device_t, cfdata_t, void *);
static void	rk_usb2phy_attach(device_t, device_t, void *);

static uint32_t
rk_usb2phy_rd4(struct rk_usb2phy_softc *sc, bus_space_handle_t bsh,
    bus_size_t off)
{
	return bus_space_read_4(sc->sc_bst, bsh, off);
}

/* Rockchip hiword write-enable register update. */
static void
rk_usb2phy_clrset(struct rk_usb2phy_softc *sc, bus_space_handle_t bsh,
    bus_size_t off, uint32_t mask, uint32_t val)
{
	bus_space_write_4(sc->sc_bst, bsh, off,
	    ((mask & 0xffff) << 16) | (val & mask));
}

static int
rk_usb2phy_map_prop(struct rk_usb2phy_softc *sc, const char *prop,
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

/*
 * Ensure the PIPE power domain is powered and no stale BIU_PIPE idle
 * request blocks its APB.  Idempotent - the SATA combphy may have
 * already brought the domain up (or U-Boot's scsi scan did), and a
 * power cycle here would tear down the working SATA links.
 */
static void
rk_usb2phy_pd_pipe_ensure(struct rk_usb2phy_softc *sc)
{
	bus_space_handle_t pmu = sc->sc_pmu_bsh;
	u_int n;

	rk_usb2phy_clrset(sc, pmu, PMU_BUS_IDLE_SFTCON0, PMU_BIU_PIPE, 0);
	for (n = 0; n < 100; n++) {
		if ((rk_usb2phy_rd4(sc, pmu, PMU_BUS_IDLE_ACK) & PMU_BIU_PIPE) == 0)
			break;
		delay(100);
	}

	rk_usb2phy_clrset(sc, pmu, PMU_PWR_GATE_SFTCON, PMU_PD_PIPE, 0);
	for (n = 0; n < 100; n++) {
		if ((rk_usb2phy_rd4(sc, pmu, PMU_PWR_DWN_ST) & PMU_PD_PIPE) == 0)
			break;
		delay(100);
	}
}

/* Clock gates + core soft resets for the dwc3 and USB2 host blocks. */
static void
rk_usb2phy_domain_init(struct rk_usb2phy_softc *sc)
{
	bus_space_handle_t cru = sc->sc_cru_bsh;
	bus_space_handle_t pmucru = sc->sc_pmucru_bsh;

	rk_usb2phy_pd_pipe_ensure(sc);

	/* Open the USB3OTG clock gates (pipe family included in case the
	 * combphy driver has not run yet).  Gate polarity: 1 = off. */
	rk_usb2phy_clrset(sc, cru, CRU_CLKGATE_CON(10),
	    CLKGATE_CON10_USB3OTG | CLKGATE_CON10_PIPE, 0);
	rk_usb2phy_clrset(sc, pmucru, PMUCRU_CLKGATE_CON2,
	    CLKGATE_CON2_USBPHY, 0);

	/* Assert + release SRST_USB3OTG0/1 and the USB2 host H/ARB/UTMI
	 * resets: ATF leaves the cores in reset otherwise (GSNPSID reads
	 * 0 for the dwc3s). */
	rk_usb2phy_clrset(sc, cru, CRU_SOFTRST_CON(9), SOFTRST_CON9_USB3OTG,
	    SOFTRST_CON9_USB3OTG);
	rk_usb2phy_clrset(sc, cru, CRU_SOFTRST_CON(14), SOFTRST_CON14_USB2HOST,
	    SOFTRST_CON14_USB2HOST);
	delay(100);
	rk_usb2phy_clrset(sc, cru, CRU_SOFTRST_CON(9), SOFTRST_CON9_USB3OTG, 0);
	rk_usb2phy_clrset(sc, cru, CRU_SOFTRST_CON(14), SOFTRST_CON14_USB2HOST,
	    0);
	delay(100);

	/* Release-only the usb2phy1 POR / port resets and its GRF pclk -
	 * asserting those would glitch the panel phy, releasing a reset
	 * that is not asserted is a no-op. */
	rk_usb2phy_clrset(sc, cru, CRU_SOFTRST_CON(29),
	    SOFTRST_CON29_USB2PHY1, 0);
	rk_usb2phy_clrset(sc, cru, CRU_SOFTRST_CON(28),
	    SOFTRST_CON28_USB2PHY1_GRF, 0);
}

/* Both USB3.0 Type-A connectors are dead until GPIO3_A1 is driven. */
static void
rk_usb2phy_vbus_enable(struct rk_usb2phy_softc *sc)
{
	rk_usb2phy_clrset(sc, sc->sc_gpio3_bsh, GPIO3_SWPORT_DDR_L,
	    GPIO3_VBUS_PINS, GPIO3_VBUS_PINS);
	rk_usb2phy_clrset(sc, sc->sc_gpio3_bsh, GPIO3_SWPORT_DR_L,
	    GPIO3_VBUS_PINS, GPIO3_VBUS_PINS);
}

static void
rk_usb2phy_port_write(struct rk_usb2phy_port *port)
{
	struct rk_usb2phy_softc * const sc = port->sc;

	rk_usb2phy_clrset(sc, sc->sc_grf_bsh, port->off, port->mask,
	    port->val);
	delay(2000);
	port->on = true;
}

static void *
rk_usb2phy_acquire(device_t dev, const void *data, size_t datalen, u_int portno)
{
	struct rk_usb2phy_softc * const sc = device_private(dev);

	if (datalen != 0) {
		aprint_error_dev(dev, "unexpected #phy-cells\n");
		return NULL;
	}

	return &sc->sc_ports[portno];
}

static void *
rk_usb2phy_acquire_otg(device_t dev, const void *data, size_t datalen)
{
	return rk_usb2phy_acquire(dev, data, datalen, 0);
}

static void *
rk_usb2phy_acquire_host(device_t dev, const void *data, size_t datalen)
{
	return rk_usb2phy_acquire(dev, data, datalen, 1);
}

static void
rk_usb2phy_release(device_t dev, void *priv)
{
}

static int
rk_usb2phy_enable(device_t dev, void *priv, bool enable)
{
	struct rk_usb2phy_port * const port = priv;

	if (!enable)
		return 0;

	rk_usb2phy_port_write(port);

	return 0;
}

static const struct fdtbus_phy_controller_func rk_usb2phy_otg_funcs = {
	.acquire = rk_usb2phy_acquire_otg,
	.release = rk_usb2phy_release,
	.enable = rk_usb2phy_enable,
};

static const struct fdtbus_phy_controller_func rk_usb2phy_host_funcs = {
	.acquire = rk_usb2phy_acquire_host,
	.release = rk_usb2phy_release,
	.enable = rk_usb2phy_enable,
};

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "rockchip,rk3568-usb2phy" },
	DEVICE_COMPAT_EOL
};

/*
 * The USB-domain sequence (PD_PIPE, clock gates, soft resets, VBUS)
 * must run exactly once per boot: a second run would re-assert the
 * dwc3 soft resets under the already-attached xHCI controllers.
 */
static bool	rk_usb2phy_domain_done;

CFATTACH_DECL_NEW(rkusb2phy, sizeof(struct rk_usb2phy_softc),
    rk_usb2phy_match, rk_usb2phy_attach, NULL, NULL);

static int
rk_usb2phy_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rk_usb2phy_attach(device_t parent, device_t self, void *aux)
{
	struct rk_usb2phy_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct clk *clk;
	struct rk_usb2phy_port *otg, *host;
	int otg_phandle, host_phandle;
	int error;

	sc->sc_dev = self;
	sc->sc_phandle = phandle;
	sc->sc_bst = faa->faa_bst;

	if ((error = rk_usb2phy_map_prop(sc, "rockchip,usbgrf",
	    &sc->sc_grf_bsh)) != 0) {
		aprint_error(": couldn't map %s (%d)\n", "rockchip,usbgrf",
		    error);
		return;
	}
	if ((error = bus_space_map(sc->sc_bst, RK3568_CRU_BASE,
	    RK3568_CRU_SIZE, 0, &sc->sc_cru_bsh)) != 0 ||
	    (error = bus_space_map(sc->sc_bst, RK3568_PMUCRU_BASE,
	    RK3568_PMUCRU_SIZE, 0, &sc->sc_pmucru_bsh)) != 0 ||
	    (error = bus_space_map(sc->sc_bst, RK3568_PMU_BASE,
	    RK3568_PMU_SIZE, 0, &sc->sc_pmu_bsh)) != 0 ||
	    (error = bus_space_map(sc->sc_bst, RK3568_GPIO3_BASE,
	    RK3568_GPIO3_SIZE, 0, &sc->sc_gpio3_bsh)) != 0) {
		aprint_error(": couldn't map system controller (%d)\n", error);
		return;
	}

	/* 24 MHz xtal reference for the PHY (report only; the stub does
	 * not program clocks). */
	clk = fdtbus_clock_get_index(phandle, 0);
	if (clk != NULL)
		clk_enable(clk);

	aprint_naive("\n");
	aprint_normal(": RK3568 USB2 PHY (otg + host ports)\n");

	if (!rk_usb2phy_domain_done) {
		rk_usb2phy_domain_init(sc);
		rk_usb2phy_vbus_enable(sc);
		rk_usb2phy_domain_done = true;
	}

	/* Enable the 480MHz clock output - the EHCI/OHCI group takes its
	 * UTMI clock from the usb2phy1 480m output (Linux inno-usb2phy
	 * clkout_ctl: CON2 @ 0x008 bit4 <- 0). */
	rk_usb2phy_clrset(sc, sc->sc_grf_bsh, USB2PHY_GRF_CLKOUT_CON2,
	    USB2PHY_CLKOUT_480M_OFF, 0);
	delay(100);

	otg = &sc->sc_ports[0];
	otg->sc = sc;
	otg->off = USB2PHY_GRF_OTG_CON0;
	otg->val = USB2PHY_OTG_HOST_VAL;
	otg->mask = USB2PHY_OTG_HOST_MASK;
	rk_usb2phy_port_write(otg);

	host = &sc->sc_ports[1];
	host->sc = sc;
	host->off = USB2PHY_GRF_HOST_CON1;
	host->val = USB2PHY_HOST_VAL;
	host->mask = USB2PHY_HOST_MASK;
	rk_usb2phy_port_write(host);

	aprint_normal_dev(self,
	    "USB domain up (grf con0=0x%08x con1=0x%08x con2=0x%08x "
	    "cru con10=0x%08x con9=0x%08x con14=0x%08x con29=0x%08x "
	    "pmucru gate2=0x%08x gpio3 dr=0x%08x ddr=0x%08x)\n",
	    rk_usb2phy_rd4(sc, sc->sc_grf_bsh, USB2PHY_GRF_OTG_CON0),
	    rk_usb2phy_rd4(sc, sc->sc_grf_bsh, USB2PHY_GRF_HOST_CON1),
	    rk_usb2phy_rd4(sc, sc->sc_grf_bsh, USB2PHY_GRF_CLKOUT_CON2),
	    rk_usb2phy_rd4(sc, sc->sc_cru_bsh, CRU_CLKGATE_CON(10)),
	    rk_usb2phy_rd4(sc, sc->sc_cru_bsh, CRU_SOFTRST_CON(9)),
	    rk_usb2phy_rd4(sc, sc->sc_cru_bsh, CRU_SOFTRST_CON(14)),
	    rk_usb2phy_rd4(sc, sc->sc_cru_bsh, CRU_SOFTRST_CON(29)),
	    rk_usb2phy_rd4(sc, sc->sc_pmucru_bsh, PMUCRU_CLKGATE_CON2),
	    rk_usb2phy_rd4(sc, sc->sc_gpio3_bsh, GPIO3_SWPORT_DR_L),
	    rk_usb2phy_rd4(sc, sc->sc_gpio3_bsh, GPIO3_SWPORT_DDR_L));

	host_phandle = of_find_firstchild_byname(phandle, "host-port");
	if (host_phandle > 0)
		fdtbus_register_phy_controller(self, host_phandle,
		    &rk_usb2phy_host_funcs);
	else
		aprint_error_dev(self, "no host-port child\n");
	otg_phandle = of_find_firstchild_byname(phandle, "otg-port");
	if (otg_phandle > 0)
		fdtbus_register_phy_controller(self, otg_phandle,
		    &rk_usb2phy_otg_funcs);
	else
		aprint_error_dev(self, "no otg-port child\n");
}
