/* $NetBSD$ */

/*-
 * Minimal RK3568 clock-and-reset unit driver (bring-up stage).
 *
 * This is NOT a register-accurate CRU implementation.  It registers the
 * CRU as a fixed-rate clock controller so that standard FDT consumers
 * (dw_apb_uart, dwcmmc, rkiic, rkspi, ...) can resolve their "clocks"
 * phandles and proceed to attach.  The actual clock rates were left
 * configured by U-Boot at boot, so no register programming is performed.
 *
 * Two controllers are matched:
 *   - the main CRU (rockchip,rk3568-cru) at 0xfdd20000
 *   - the PMU CRU (rockchip,rk3568-pmucru) at 0xfdd00000
 *
 * Each clock is reported at a fixed nominal rate (24 MHz / 24 MHz
 * reference, 150 MHz SDMMC, ...).  UART2's console baud clock is 24 MHz;
 * SDMMC `ciu` at 150 MHz.  Rates are only used by consumers to compute
 * divider values / baud rates; when the firmware default is close enough
 * the boot proceeds.  Once a register-accurate CRU (with real PLL/composite
 * tables like rk3328_cru.c / rk3588_cru.c) lands upstream, replace this
 * driver with it.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: rk3568_cru.c,v 1.1 2026/09/01 00:00:00 rk3568-bringup Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/kmem.h>
#include <sys/bus.h>

#include <dev/clk/clk_backend.h>
#include <dev/fdt/fdtvar.h>

#include <arm/rockchip/rk3568_cru.h>

static int	rk3568_cru_match(device_t, cfdata_t, void *);
static void	rk3568_cru_attach(device_t, device_t, void *);

static struct clk *rk3568_cru_decode(device_t, int, const void *, size_t);
static struct clk *rk3568_cru_get(void *, const char *);
static void	rk3568_cru_put(void *, struct clk *);
static u_int	rk3568_cru_get_rate(void *, struct clk *);

static const struct fdtbus_clock_controller_func rk3568_cru_fdtclock_funcs = {
	.decode = rk3568_cru_decode,
};

static const struct clk_funcs rk3568_cru_clk_funcs = {
	.get = rk3568_cru_get,
	.put = rk3568_cru_put,
	.get_rate = rk3568_cru_get_rate,
};

/* Clock ID -> fixed rate map.  IDs come from the DT binding header
 * dt-bindings/clock/rk3568-cru.h (vendored under external/gpl2/dts). */
struct rk3568_cru_rate {
	uint32_t	id;
	uint32_t	rate;
};

static const struct rk3568_cru_rate rk3568_cru_rates[] = {
	/* PMU CRU */
	{ RK3568_CLK_PCIEPHY0_REF,	100000000 },
	{ RK3568_CLK_PCIEPHY1_REF,	100000000 },
	{ RK3568_CLK_I2C0,	24000000 },
	{ RK3568_PCLK_I2C0,	100000000 },
	{ RK3568_SCLK_UART0,	24000000 },
	{ RK3568_PCLK_UART0,	100000000 },
	/* Main CRU */
	{ RK3568_SCLK_UART1,	24000000 },
	{ RK3568_PCLK_UART1,	100000000 },
	{ RK3568_SCLK_UART2,	24000000 },
	{ RK3568_PCLK_UART2,	100000000 },
	{ RK3568_SCLK_UART3,	24000000 },
	{ RK3568_PCLK_UART3,	100000000 },
	{ RK3568_SCLK_UART4,	24000000 },
	{ RK3568_PCLK_UART4,	100000000 },
	{ RK3568_SCLK_UART5,	24000000 },
	{ RK3568_PCLK_UART5,	100000000 },
	{ RK3568_SCLK_UART6,	24000000 },
	{ RK3568_PCLK_UART6,	100000000 },
	{ RK3568_SCLK_UART7,	24000000 },
	{ RK3568_PCLK_UART7,	100000000 },
	{ RK3568_SCLK_UART8,	24000000 },
	{ RK3568_PCLK_UART8,	100000000 },
	{ RK3568_SCLK_UART9,	24000000 },
	{ RK3568_PCLK_UART9,	100000000 },
	/* I2C */
	{ RK3568_CLK_I2C1,	100000000 },
	{ RK3568_PCLK_I2C1,	100000000 },
	{ RK3568_CLK_I2C2,	100000000 },
	{ RK3568_PCLK_I2C2,	100000000 },
	{ RK3568_CLK_I2C3,	100000000 },
	{ RK3568_PCLK_I2C3,	100000000 },
	{ RK3568_CLK_I2C4,	100000000 },
	{ RK3568_PCLK_I2C4,	100000000 },
	{ RK3568_CLK_I2C5,	100000000 },
	{ RK3568_PCLK_I2C5,	100000000 },
	/* SDMMC (DesignWare MMC: biu = hclk, ciu = clk) */
	{ RK3568_HCLK_SDMMC0,	150000000 },
	{ RK3568_CLK_SDMMC0,	150000000 },
	{ RK3568_HCLK_SDMMC1,	150000000 },
	{ RK3568_CLK_SDMMC1,	150000000 },
	{ RK3568_HCLK_SDMMC2,	150000000 },
	{ RK3568_CLK_SDMMC2,	150000000 },
	/* SPI */
	{ RK3568_CLK_SPI0,	100000000 },
	{ RK3568_PCLK_SPI0,	100000000 },
	{ RK3568_CLK_SPI1,	100000000 },
	{ RK3568_PCLK_SPI1,	100000000 },
	{ RK3568_CLK_SPI2,	100000000 },
	{ RK3568_PCLK_SPI2,	100000000 },
	{ RK3568_CLK_SPI3,	100000000 },
	{ RK3568_PCLK_SPI3,	100000000 },
	/* TSADC */
	{ RK3568_PCLK_TSADC,	100000000 },
	{ RK3568_CLK_TSADC,	1000000 },
	/* Crypto (rkv1crypto: aclk/hclk) */
	{ RK3568_ACLK_CRYPTO_NS,	200000000 },
	{ RK3568_HCLK_CRYPTO_NS,	200000000 },
	{ RK3568_CLK_CRYPTO_NS_CORE,	300000000 },
	{ RK3568_CLK_CRYPTO_NS_PKA,	300000000 },
	{ RK3568_CLK_CRYPTO_NS_RNG,	100000000 },
	/* GMAC (eqos): U-Boot leaves the MAC clock muxes at 125 MHz from
	 * CPLL_125M; aclk/pclk at their 300/100 MHz php/usb defaults. */
	{ RK3568_ACLK_GMAC0,	300000000 },
	{ RK3568_PCLK_GMAC0,	100000000 },
	{ RK3568_CLK_MAC0_2TOP,	125000000 },
	{ RK3568_CLK_MAC0_OUT,	125000000 },
	{ RK3568_CLK_MAC0_REFOUT,	125000000 },
	{ RK3568_CLK_GMAC0_PTP_REF,	62500000 },
	{ RK3568_ACLK_GMAC1,	300000000 },
	{ RK3568_PCLK_GMAC1,	100000000 },
	{ RK3568_CLK_MAC1_2TOP,	125000000 },
	{ RK3568_CLK_MAC1_OUT,	125000000 },
	{ RK3568_CLK_MAC1_REFOUT,	125000000 },
	{ RK3568_CLK_GMAC1_PTP_REF,	62500000 },
	{ RK3568_SCLK_GMAC0,	125000000 },
	{ RK3568_SCLK_GMAC0_RGMII_SPEED,	125000000 },
	{ RK3568_SCLK_GMAC0_RMII_SPEED,	25000000 },
	{ RK3568_SCLK_GMAC0_RX_TX,	125000000 },
	{ RK3568_SCLK_GMAC1,	125000000 },
	{ RK3568_SCLK_GMAC1_RGMII_SPEED,	125000000 },
	{ RK3568_SCLK_GMAC1_RMII_SPEED,	25000000 },
	{ RK3568_SCLK_GMAC1_RX_TX,	125000000 },
	/* Watchdog */
	{ RK3568_PCLK_WDT_NS,	100000000 },
	{ RK3568_TCLK_WDT_NS,	32768 },
	/* SATA / pipe domain.  Rates are the nominal U-Boot/Linux values
	 * (aclk_pipe = gpll_400m = 396 MHz, pclk_pipe = aclk_pipe/4); the
	 * gates themselves are opened by the rk_combphy driver, the stub
	 * only reports them. */
	{ RK3568_ACLK_PIPE,	396000000 },
	{ RK3568_PCLK_PIPE,	100000000 },
	{ RK3568_ACLK_SATA0,	396000000 },
	{ RK3568_CLK_SATA0_PMALIVE,	20000000 },
	{ RK3568_CLK_SATA0_RXOOB,	50000000 },
	{ RK3568_ACLK_SATA1,	396000000 },
	{ RK3568_CLK_SATA1_PMALIVE,	20000000 },
	{ RK3568_CLK_SATA1_RXOOB,	50000000 },
	{ RK3568_PCLK_PIPEPHY0,	100000000 },
	{ RK3568_PCLK_PIPEPHY1,	100000000 },
};

#define RK3568_CRU_NRATES	__arraycount(rk3568_cru_rates)

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "rockchip,rk3568-cru" },
	{ .compat = "rockchip,rk3568-pmucru" },
	DEVICE_COMPAT_EOL
};

CFATTACH_DECL_NEW(rk3568_cru,
	sizeof(struct rk3568_cru_softc) +
	    RK3568_CRU_NRATES * sizeof(struct rk3568_cru_clk),
	rk3568_cru_match, rk3568_cru_attach, NULL, NULL);

static int
rk3568_cru_match(device_t parent, cfdata_t cf, void *aux)
{
	const struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rk3568_cru_attach(device_t parent, device_t self, void *aux)
{
	struct rk3568_cru_softc * const sc = device_private(self);
	const struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;

	sc->sc_dev = self;
	sc->sc_phandle = phandle;
	sc->sc_clkdom.name = device_xname(self);
	sc->sc_clkdom.funcs = &rk3568_cru_clk_funcs;
	sc->sc_clkdom.priv = sc;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}
	if (bus_space_map(faa->faa_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	/* Register a fixed-rate clock for every known ID.  All clocks share
	 * one clk_domain and one funcs table; rk3568_cru_get_rate returns
	 * the fixed rate via the clk's private rate field. */
	for (u_int i = 0; i < RK3568_CRU_NRATES; i++) {
		struct rk3568_cru_clk * const ck = &sc->sc_clks[i];

		ck->id = rk3568_cru_rates[i].id;
		ck->rate = rk3568_cru_rates[i].rate;
		ck->base.domain = &sc->sc_clkdom;
		ck->base.name = kmem_asprintf("%s:%u",
		    device_xname(self), rk3568_cru_rates[i].id);
		clk_attach(&ck->base);
	}
	sc->sc_nclks = RK3568_CRU_NRATES;

	fdtbus_register_clock_controller(self, phandle,
	    &rk3568_cru_fdtclock_funcs);

	aprint_naive("\n");
	aprint_normal(": RK3568 CRU (fixed-rate bring-up stub)\n");
}

static struct clk *
rk3568_cru_decode(device_t dev, int cc_phandle, const void *data, size_t len)
{
	struct rk3568_cru_softc * const sc = device_private(dev);
	uint32_t clock_id;
	u_int i;

	if (len != 4)
		return NULL;

	clock_id = be32dec(data);

	for (i = 0; i < sc->sc_nclks; i++) {
		if (sc->sc_clks[i].id == clock_id)
			return &sc->sc_clks[i].base;
	}

	return NULL;
}

static struct clk *
rk3568_cru_get(void *priv, const char *name)
{
	struct rk3568_cru_softc * const sc = priv;

	for (u_int i = 0; i < sc->sc_nclks; i++) {
		if (strcmp(sc->sc_clks[i].base.name, name) == 0)
			return &sc->sc_clks[i].base;
	}

	return NULL;
}

static void
rk3568_cru_put(void *priv, struct clk *clk)
{
}

static u_int
rk3568_cru_get_rate(void *priv, struct clk *clkp)
{
	struct rk3568_cru_clk * const ck = (struct rk3568_cru_clk *)clkp;

	return ck->rate;
}
