/*	$NetBSD: rk_eqos.c,v 1.3 2024/02/07 04:20:27 msaitoh Exp $	*/

/*-
 * Copyright (c) 2022 Ryo Shimizu
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: rk_eqos.c,v 1.3 2024/02/07 04:20:27 msaitoh Exp $");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/rndsource.h>

#include <net/if_ether.h>
#include <net/if_media.h>

#include <dev/fdt/fdtvar.h>
#include <dev/fdt/syscon.h>

#include <dev/mii/miivar.h>
#include <dev/ic/dwc_eqos_var.h>

struct rk_eqos_softc {
	struct eqos_softc sc_base;

	struct syscon *sc_grf;
	struct syscon *sc_php_grf;
	int sc_id;	/* ethernet0 or 1? */
};

static int rk_eqos_match(device_t, cfdata_t, void *);
static void rk_eqos_attach(device_t, device_t, void *);

struct rk_eqos_ops {
	void (*set_mode_rgmii)(struct rk_eqos_softc *, int, int);
	void (*set_speed_rgmii)(struct rk_eqos_softc *, int);
	void (*clock_selection)(struct rk_eqos_softc *, int);
	int (*get_unit)(struct rk_eqos_softc *, int);
	int (*reset_gpio)(struct rk_eqos_softc *, int);
	bool require_php_grf;
};

CFATTACH_DECL_NEW(rk_eqos, sizeof(struct rk_eqos_softc),
    rk_eqos_match, rk_eqos_attach, NULL, NULL);

/*
 * RK3588 specific
 */
#define RK3588_ETHERNET1_ADDR			0xfe1c0000

/* grf */
#define RK3588_GRF_GMAC_TXRXCLK_DELAY_EN_REG	(0x0300 + (4 * 7))
#define  RK3588_GMAC_RXCLK_DELAY_EN(id)		__BIT(3 + 2 * (id))
#define   RK3588_GMAC_RXCLK_DELAY_DISABLE	0
#define   RK3588_GMAC_RXCLK_DELAY_ENABLE	1
#define  RK3588_GMAC_TXCLK_DELAY_EN(id)		__BIT(2 + 2 * (id))
#define   RK3588_GMAC_TXCLK_DELAY_DISABLE	0
#define   RK3588_GMAC_TXCLK_DELAY_ENABLE	1
#define RK3588_GRF_GMAC_TXRX_DELAY_CFG_REG(id)	(0x0300 + (4 * 8) + (4 * (id)))
#define  RK3588_GMAC_RXCLK_DELAY_CFG		__BITS(15,8)
#define  RK3588_GMAC_TXCLK_DELAY_CFG		__BITS(7,0)

/* grf_php */
#define RK3588_GRF_GMAC_PHY_REG			0x0008
#define  RK3588_GMAC_PHY_IFACE_SEL(id)		(__BITS(5,3) << ((id) * 6))
#define   RK3588_GMAC_PHY_IFACE_SEL_RGMII	1
#define   RK3588_GMAC_PHY_IFACE_SEL_RMII	4
#define RK3588_GRF_CLK_CON1			0x0070
#define RK3588_GRF_GMAC_CLK_REG			0x0070
#define  RK3588_GMAC_CLK_SELECT(id)		__BIT(4 + 5 * (id))
#define   RK3588_GMAC_CLK_SELECT_IO		0
#define   RK3588_GMAC_CLK_SELECT_CRU		1
#define  RK3588_GMAC_CLK_RMII_DIV(id)		__BIT(2 + 5 * (id))
#define   RK3588_GMA_CLK_RMII_DIV_DIV20		0
#define   RK3588_GMA_CLK_RMII_DIV_DIV2		1
#define  RK3588_GMAC_CLK_RGMII_DIV(id)		(__BITS(3,2) << ((id) * 5))
#define   RK3588_GMAC_CLK_RGMII_DIV_DIV1	1
#define   RK3588_GMAC_CLK_RGMII_DIV_DIV50	2
#define   RK3588_GMAC_CLK_RGMII_DIV_DIV5	3
#define  RK3588_GMAC_CLK_RMII_GATE_EN(id)	__BIT(1 + (id) * 5)
#define   RK3588_GMAC_CLK_RMII_GATE_DISABLE	0
#define   RK3588_GMAC_CLK_RMII_GATE_ENABLE	1
#define  RK3588_GMAC_CLK_MODE(id)		__BIT(0 + (id) * 5)
#define   RK3588_GMAC_CLK_MODE_RGMII		0
#define   RK3588_GMAC_CLK_MODE_RMII		1

static void
rk3588_eqos_set_mode_rgmii(struct rk_eqos_softc *rk_sc,
    int tx_delay, int rx_delay)
{
	const int id = rk_sc->sc_id;
	uint32_t txen, rxen;

	if (tx_delay >= 0) {
		txen = RK3588_GMAC_TXCLK_DELAY_ENABLE;
	} else {
		txen = RK3588_GMAC_TXCLK_DELAY_DISABLE;
		tx_delay = 0;
	}
	if (rx_delay >= 0) {
		rxen = RK3588_GMAC_RXCLK_DELAY_ENABLE;
	} else {
		rxen = RK3588_GMAC_RXCLK_DELAY_DISABLE;
		rx_delay = 0;
	}

	syscon_lock(rk_sc->sc_grf);
	syscon_write_4(rk_sc->sc_grf, RK3588_GRF_GMAC_TXRXCLK_DELAY_EN_REG,
	    RK3588_GMAC_TXCLK_DELAY_EN(id) << 16 |		/* masks */
	    RK3588_GMAC_RXCLK_DELAY_EN(id) << 16 |
	    __SHIFTIN(txen, RK3588_GMAC_TXCLK_DELAY_EN(id)) |	/* values */
	    __SHIFTIN(rxen, RK3588_GMAC_RXCLK_DELAY_EN(id)));
	syscon_write_4(rk_sc->sc_grf, RK3588_GRF_GMAC_TXRX_DELAY_CFG_REG(id),
	    RK3588_GMAC_TXCLK_DELAY_CFG << 16 |			/* masks */
	    RK3588_GMAC_RXCLK_DELAY_CFG << 16 |
	    __SHIFTIN(tx_delay, RK3588_GMAC_TXCLK_DELAY_CFG) |	/* values */
	    __SHIFTIN(rx_delay, RK3588_GMAC_RXCLK_DELAY_CFG));
	syscon_unlock(rk_sc->sc_grf);

	syscon_lock(rk_sc->sc_php_grf);
	syscon_write_4(rk_sc->sc_php_grf, RK3588_GRF_GMAC_PHY_REG,
	    RK3588_GMAC_PHY_IFACE_SEL(id) << 16 |		/* mask */
	    __SHIFTIN(RK3588_GMAC_PHY_IFACE_SEL_RGMII,		/* value */
	    RK3588_GMAC_PHY_IFACE_SEL(id)));
	syscon_write_4(rk_sc->sc_php_grf, RK3588_GRF_GMAC_CLK_REG,
	    RK3588_GMAC_CLK_MODE(id) << 16 |			/* mask */
	    __SHIFTIN(RK3588_GMAC_CLK_MODE_RGMII,		/* value */
	    RK3588_GMAC_CLK_MODE(id)));
	syscon_unlock(rk_sc->sc_php_grf);
}

static void
rk3588_eqos_set_speed_rgmii(struct rk_eqos_softc *rk_sc, int speed)
{
	const int id = rk_sc->sc_id;
	u_int clksel;

	switch (speed) {
	case IFM_10_T:
		clksel = RK3588_GMAC_CLK_RGMII_DIV_DIV50;
		break;
	case IFM_100_TX:
		clksel = RK3588_GMAC_CLK_RGMII_DIV_DIV5;
		break;
	case IFM_1000_T:
	default:
		clksel = RK3588_GMAC_CLK_RGMII_DIV_DIV1;
		break;
	}

	syscon_lock(rk_sc->sc_php_grf);
	syscon_write_4(rk_sc->sc_php_grf, RK3588_GRF_GMAC_CLK_REG,
	    RK3588_GMAC_CLK_RGMII_DIV(id) << 16 |		/* mask */
	    __SHIFTIN(clksel, RK3588_GMAC_CLK_RGMII_DIV(id)));	/* value */
	syscon_unlock(rk_sc->sc_php_grf);
}

static void
rk3588_eqos_clock_selection(struct rk_eqos_softc *rk_sc, int phandle)
{
	const int id = rk_sc->sc_id;
	const char *clock_in_out;

	clock_in_out = fdtbus_get_string(phandle, "clock_in_out");
	if (clock_in_out != NULL) {
		bool input = (strcmp(clock_in_out, "input") == 0) ?
		    true : false;
		uint32_t clksel, gate;

		if (input) {
			clksel = RK3588_GMAC_CLK_SELECT_IO;
			gate = RK3588_GMAC_CLK_RMII_GATE_DISABLE;
		} else {
			clksel = RK3588_GMAC_CLK_SELECT_CRU;
			gate = RK3588_GMAC_CLK_RMII_GATE_ENABLE;
		}

		syscon_lock(rk_sc->sc_php_grf);
		syscon_write_4(rk_sc->sc_php_grf, RK3588_GRF_GMAC_CLK_REG,
		    /* masks */
		    RK3588_GMAC_CLK_SELECT(id) << 16 |
		    RK3588_GMAC_CLK_RMII_GATE_EN(id) << 16 |
		    /* values */
		    __SHIFTIN(clksel, RK3588_GMAC_CLK_SELECT(id)) |
		    __SHIFTIN(gate, RK3588_GMAC_CLK_RMII_GATE_EN(id)));
		syscon_unlock(rk_sc->sc_php_grf);
	}
}

static int
rk3588_eqos_get_unit(struct rk_eqos_softc *rk_sc, int phandle)
{
	bus_addr_t addr;
	bus_size_t size;

	fdtbus_get_reg(phandle, 0, &addr, &size);
	if (addr == RK3588_ETHERNET1_ADDR)
		return 1;
	return 0;
}

static int
rk_eqos_reset_gpio(const int phandle)
{
	struct fdtbus_gpio_pin *pin_reset;
	const u_int *reset_delay_us;
	bool reset_active_low;
	int len;

	if (!of_hasprop(phandle, "snps,reset-gpio"))
		return 0;

	pin_reset = fdtbus_gpio_acquire(phandle, "snps,reset-gpio",
	    GPIO_PIN_OUTPUT);
	if (pin_reset == NULL)
		return ENOENT;

	reset_delay_us = fdtbus_get_prop(phandle, "snps,reset-delays-us", &len);
	if (reset_delay_us == NULL || len != 12)
		return ENXIO;

	reset_active_low = of_hasprop(phandle, "snps,reset-active-low");

	fdtbus_gpio_write_raw(pin_reset, reset_active_low ? 1 : 0);
	delay(be32toh(reset_delay_us[0]));
	fdtbus_gpio_write_raw(pin_reset, reset_active_low ? 0 : 1);
	delay(be32toh(reset_delay_us[1]));
	fdtbus_gpio_write_raw(pin_reset, reset_active_low ? 1 : 0);
	delay(be32toh(reset_delay_us[2]));

	return 0;
}

static int
rk3588_eqos_reset_gpio(struct rk_eqos_softc *rk_sc, int phandle)
{

	return rk_eqos_reset_gpio(phandle);
}

static const struct rk_eqos_ops rk3588_ops = {
	.set_mode_rgmii = rk3588_eqos_set_mode_rgmii,
	.set_speed_rgmii = rk3588_eqos_set_speed_rgmii,
	.clock_selection = rk3588_eqos_clock_selection,
	.get_unit = rk3588_eqos_get_unit,
	.reset_gpio = rk3588_eqos_reset_gpio,
	.require_php_grf = true,
};

/*
 * RK3568 specific
 *
 * Both GMACs are DWC Ethernet QoS (dwmac-4.20a) cores.  Interface mode,
 * RGMII delays and the GMAC pin groups all live in the main GRF at
 * 0xfdc60000; there is no separate php_grf on this SoC.  NetBSD has no
 * iomux/pinctrl driver for the RK3568 yet (and U-Boot only muxes the
 * ports it probes), so the GMAC pin groups are muxed directly from a
 * static table below; the values match the vendor Linux device tree and
 * the writes are idempotent with whatever the firmware left behind.
 * There is no GPIO controller driver either, so the PHY reset GPIO is
 * pulsed by hand from its "snps,reset-gpio" specifier.
 */
#define RK3568_ETHERNET0_ADDR		0xfe2a0000
#define RK3568_ETHERNET1_ADDR		0xfe010000

/* grf: per-port interface mode / RGMII delay control */
#define RK3568_GRF_GMAC0_CON0		0x0380
#define RK3568_GRF_GMAC0_CON1		0x0384
#define RK3568_GRF_GMAC1_CON0		0x0388
#define RK3568_GRF_GMAC1_CON1		0x038c
/* grf: gmac1 io route (m0/m1); bit8 = 0 selects m0 */
#define RK3568_GRF_GMAC1_ROUTE		0x0300

/* GRF write format: <value> in bits[15:0], <write-enable> in bits[31:16] */
#define RK3568_GRF_WRITE(val, mask)	((((mask) & 0xffff) << 16) | \
					  ((val) & 0xffff))
#define RK3568_GMAC_PHY_INTF_SEL_RGMII	RK3568_GRF_WRITE(__BIT(4), __BITS(6,4))
#define RK3568_GMAC_TXCLK_DLY_ENABLE	RK3568_GRF_WRITE(__BIT(0), __BIT(0))
#define RK3568_GMAC_RXCLK_DLY_ENABLE	RK3568_GRF_WRITE(__BIT(1), __BIT(1))

/*
 * RK3568 GPIO banks are version 2: 16 pins per data/direction register,
 * value in bits[15:0] and write-enable in bits[31:16].
 */
#define RK3568_GPIO_DR_L		0x00
#define RK3568_GPIO_DR_H		0x04
#define RK3568_GPIO_DDR_L		0x08
#define RK3568_GPIO_DDR_H		0x0c

/* pin mux: 4 bits per pin, 4 pins per 32-bit iomux register */
struct rk3568_eqos_pin {
	u_int	pin;
	u_int	func;
};

static const struct rk3568_eqos_pin rk3568_eqos_gmac0_pins[] = {
	{  3, 2 },	/* rxd2 */
	{  4, 2 },	/* rxd3 */
	{  5, 2 },	/* rxclk */
	{  6, 2 },	/* txd2 */
	{  7, 2 },	/* txd3 */
	{  8, 2 },	/* txclk */
	{ 11, 1 },	/* txd0 */
	{ 12, 1 },	/* txd1 */
	{ 13, 1 },	/* txen */
	{ 14, 1 },	/* rxd0 */
	{ 15, 2 },	/* rxd1 */
	{ 16, 2 },	/* rxdvcrs */
	{ 19, 2 },	/* mdc */
	{ 20, 2 },	/* mdio */
};

static const struct rk3568_eqos_pin rk3568_eqos_gmac1_pins[] = {
	{  2, 3 },	/* txd2 */
	{  3, 3 },	/* txd3 */
	{  4, 3 },	/* rxd2 */
	{  5, 3 },	/* rxd3 */
	{  6, 3 },	/* txclk */
	{  7, 3 },	/* rxclk */
	{  9, 3 },	/* rxd0 */
	{ 10, 3 },	/* rxd1 */
	{ 11, 3 },	/* rxdvcrs */
	{ 13, 3 },	/* txd0 */
	{ 14, 3 },	/* txd1 */
	{ 15, 3 },	/* txen */
	{ 16, 3 },	/* mclkinout */
	{ 20, 3 },	/* mdc */
	{ 21, 3 },	/* mdio */
};

static void
rk3568_eqos_mux_pin(struct rk_eqos_softc *rk_sc, u_int bankoff, u_int pin,
    u_int func)
{
	const bus_size_t reg =
	    bankoff + (pin / 8) * 8 + (((pin % 8) / 4) * 4);
	const u_int shift = (pin % 4) * 4;

	syscon_write_4(rk_sc->sc_grf, reg,
	    RK3568_GRF_WRITE(func << shift, 0xf << shift));
}

static void
rk3568_eqos_clock_selection(struct rk_eqos_softc *rk_sc, int phandle)
{
	const u_int bankoff = (rk_sc->sc_id == 1) ? 0x040 : 0x020;
	const struct rk3568_eqos_pin *pins;
	u_int npins, i;

	if (rk_sc->sc_id == 1) {
		pins = rk3568_eqos_gmac1_pins;
		npins = __arraycount(rk3568_eqos_gmac1_pins);
	} else {
		pins = rk3568_eqos_gmac0_pins;
		npins = __arraycount(rk3568_eqos_gmac0_pins);
	}

	syscon_lock(rk_sc->sc_grf);
	if (rk_sc->sc_id == 1) {
		syscon_write_4(rk_sc->sc_grf, RK3568_GRF_GMAC1_ROUTE,
		    RK3568_GRF_WRITE(0, __BIT(8)));
	}
	for (i = 0; i < npins; i++)
		rk3568_eqos_mux_pin(rk_sc, bankoff, pins[i].pin, pins[i].func);
	syscon_unlock(rk_sc->sc_grf);
}

static void
rk3568_eqos_set_mode_rgmii(struct rk_eqos_softc *rk_sc,
    int tx_delay, int rx_delay)
{
	const bus_size_t con0 = (rk_sc->sc_id == 1) ?
	    RK3568_GRF_GMAC1_CON0 : RK3568_GRF_GMAC0_CON0;
	const bus_size_t con1 = (rk_sc->sc_id == 1) ?
	    RK3568_GRF_GMAC1_CON1 : RK3568_GRF_GMAC0_CON1;
	const bool txen = tx_delay >= 0;
	const bool rxen = rx_delay >= 0;
	uint32_t val;

	if (!txen)
		tx_delay = 0;
	if (!rxen)
		rx_delay = 0;

	val = RK3568_GMAC_PHY_INTF_SEL_RGMII;
	if (txen)
		val |= RK3568_GMAC_TXCLK_DLY_ENABLE;
	if (rxen)
		val |= RK3568_GMAC_RXCLK_DLY_ENABLE;

	syscon_lock(rk_sc->sc_grf);
	syscon_write_4(rk_sc->sc_grf, con1, val);
	syscon_write_4(rk_sc->sc_grf, con0,
	    RK3568_GRF_WRITE(((uint32_t)rx_delay << 8) | (uint32_t)tx_delay,
	    __BITS(14,8) | __BITS(6,0)));
	syscon_unlock(rk_sc->sc_grf);
}

static int
rk3568_eqos_get_unit(struct rk_eqos_softc *rk_sc, int phandle)
{
	bus_addr_t addr;
	bus_size_t size;

	fdtbus_get_reg(phandle, 0, &addr, &size);
	if (addr == RK3568_ETHERNET1_ADDR)
		return 1;
	return 0;
}

static void
rk3568_eqos_gpio_write(bus_space_tag_t bst, bus_space_handle_t bsh,
    u_int pin, bool output, bool value)
{
	const bus_size_t dr = (pin < 16) ? RK3568_GPIO_DR_L : RK3568_GPIO_DR_H;
	const bus_size_t ddr = (pin < 16) ? RK3568_GPIO_DDR_L : RK3568_GPIO_DDR_H;
	const u_int bit = pin & 15;

	/* set the data first, then switch the direction */
	bus_space_write_4(bst, bsh, dr,
	    __BIT(bit + 16) | (value ? __BIT(bit) : 0));
	bus_space_write_4(bst, bsh, ddr,
	    __BIT(bit + 16) | (output ? __BIT(bit) : 0));
}

/*
 * The RK3568 has no GPIO controller driver under NetBSD yet, so pulse
 * the PHY reset pin described by "snps,reset-gpio" by hand.
 */
static int
rk3568_eqos_reset_gpio(struct rk_eqos_softc *rk_sc, int phandle)
{
	const u_int *gpio_spec, *delays;
	bus_addr_t addr;
	bus_size_t size;
	bus_space_handle_t bsh;
	bool active_low;
	u_int pin;
	int len;

	if (!of_hasprop(phandle, "snps,reset-gpio"))
		return 0;

	gpio_spec = fdtbus_get_prop(phandle, "snps,reset-gpio", &len);
	if (gpio_spec == NULL || len != 12)
		return ENXIO;

	delays = fdtbus_get_prop(phandle, "snps,reset-delays-us", &len);
	if (delays == NULL || len != 12)
		return ENXIO;

	const int gpio_phandle = fdtbus_get_phandle(phandle, "snps,reset-gpio");
	if (gpio_phandle < 0)
		return ENOENT;
	if (fdtbus_get_reg(gpio_phandle, 0, &addr, &size) != 0)
		return ENOENT;
	if (bus_space_map(rk_sc->sc_base.sc_bst, addr, 0x10, 0, &bsh) != 0)
		return ENOMEM;

	pin = be32toh(gpio_spec[1]);
	active_low = of_hasprop(phandle, "snps,reset-active-low") ||
	    (be32toh(gpio_spec[2]) & 1);	/* GPIO_ACTIVE_LOW */

	delay(be32toh(delays[0]));
	rk3568_eqos_gpio_write(rk_sc->sc_base.sc_bst, bsh, pin, true,
	    !active_low);
	delay(be32toh(delays[1]));
	rk3568_eqos_gpio_write(rk_sc->sc_base.sc_bst, bsh, pin, true,
	    active_low);
	delay(be32toh(delays[2]));
	bus_space_unmap(rk_sc->sc_base.sc_bst, bsh, 0x10);

	return 0;
}

static const struct rk_eqos_ops rk3568_ops = {
	.set_mode_rgmii = rk3568_eqos_set_mode_rgmii,
	.set_speed_rgmii = NULL,
	.clock_selection = rk3568_eqos_clock_selection,
	.get_unit = rk3568_eqos_get_unit,
	.reset_gpio = rk3568_eqos_reset_gpio,
	.require_php_grf = false,
};

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "rockchip,rk3588-gmac", .value = (uintptr_t)&rk3588_ops },
	{ .compat = "rockchip,rk3568-gmac", .value = (uintptr_t)&rk3568_ops },
	DEVICE_COMPAT_EOL
};

static void
rk_eqos_set_macaddr(struct eqos_softc *sc, int phandle)
{
	prop_data_t pd;
	const u_int *mac;
	int len;

	mac = fdtbus_get_prop(phandle, "local-mac-address", &len);
	if (mac == NULL || len != ETHER_ADDR_LEN)
		mac = fdtbus_get_prop(phandle, "mac-address", &len);
	if (mac == NULL || len != ETHER_ADDR_LEN)
		return;

	pd = prop_data_create_data(mac, ETHER_ADDR_LEN);
	if (pd == NULL)
		return;
	prop_dictionary_set(device_properties(sc->sc_dev), "mac-address", pd);
	prop_object_release(pd);
}

static void
rk_eqos_init_props(struct eqos_softc *sc, int phandle)
{
	prop_dictionary_t prop = device_properties(sc->sc_dev);

	/* Defaults */
	prop_dictionary_set_uint(prop, "snps,wr_osr_lmt", 4);
	prop_dictionary_set_uint(prop, "snps,rd_osr_lmt", 8);

	if (of_hasprop(phandle, "snps,mixed-burst"))
		prop_dictionary_set_bool(prop, "snps,mixed-burst", true);
	if (of_hasprop(phandle, "snps,tso"))
		prop_dictionary_set_bool(prop, "snps,tso", true);
}

static int
rk_eqos_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rk_eqos_attach(device_t parent, device_t self, void *aux)
{
	struct rk_eqos_softc * const rk_sc = device_private(self);
	struct eqos_softc * const sc = &rk_sc->sc_base;
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	const char *phy_mode;
	char intrstr[128];
	bus_addr_t addr;
	bus_size_t size;
	u_int tx_delay, rx_delay;
	int n;

	struct rk_eqos_ops *ops = (struct rk_eqos_ops *)
	    of_compatible_lookup(phandle, compat_data)->value;

	/* multiple ethernet? */
	if (ops->get_unit != NULL)
		rk_sc->sc_id = ops->get_unit(rk_sc, phandle);

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	rk_sc->sc_grf = fdtbus_syscon_acquire(phandle, "rockchip,grf");
	if (rk_sc->sc_grf == NULL) {
		aprint_error(": couldn't get grf syscon\n");
		return;
	}
	if (ops->require_php_grf) {
		rk_sc->sc_php_grf =
		    fdtbus_syscon_acquire(phandle, "rockchip,php_grf");
		if (rk_sc->sc_php_grf == NULL) {
			aprint_error(": couldn't get php_grf syscon\n");
			return;
		}
	}

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
	sc->sc_dmat = faa->faa_dmat;

	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error(": failed to decode interrupt\n");
		return;
	}

	/* enable clocks */
	struct clk *clk;
	fdtbus_clock_assign(phandle);
	for (n = 0; (clk = fdtbus_clock_get_index(phandle, n)) != NULL; n++) {
		if (clk_enable(clk) != 0) {
			aprint_error(": couldn't enable clock #%d\n", n);
			return;
		}
	}
	/* de-assert resets */
	struct fdtbus_reset *rst;
	for (n = 0; (rst = fdtbus_reset_get_index(phandle, n)) != NULL; n++) {
		if (fdtbus_reset_deassert(rst) != 0) {
			aprint_error(": couldn't de-assert reset #%d\n", n);
			return;
		}
	}
	if (ops->reset_gpio != NULL &&
	    ops->reset_gpio(rk_sc, phandle) != 0)
		aprint_error(": GPIO reset failed\n");	/* ignore */

	if (ops->clock_selection != NULL)
		ops->clock_selection(rk_sc, phandle);

	if (of_getprop_uint32(phandle, "tx_delay", &tx_delay) != 0)
		tx_delay = -1;
	if (of_getprop_uint32(phandle, "rx_delay", &rx_delay) != 0)
		rx_delay = -1;

	phy_mode = fdtbus_get_string(phandle, "phy-mode");
	if (phy_mode == NULL)
		phy_mode = "rgmii";	/* default: RGMII */

	if (strncmp(phy_mode, "rgmii", 5) == 0) {
		ops->set_mode_rgmii(rk_sc, tx_delay, rx_delay);
		if (ops->set_speed_rgmii != NULL) {
			/*
			 * XXX: should be called back from
			 *  sys/dev/ic/dwc_eqos.c:eqos_update_link() ?
			 */
			ops->set_speed_rgmii(rk_sc, IFM_1000_T);
		}
	} else {
		aprint_error(": unsupported phy-mode '%s'\n", phy_mode);
		return;
	}

	rk_eqos_init_props(sc, phandle);
	rk_eqos_set_macaddr(sc, phandle);
	sc->sc_phy_id = MII_PHY_ANY;
#define CSR_RATE_RGMII	125000000	/* default */
	sc->sc_csr_clock = CSR_RATE_RGMII;

	if (eqos_attach(sc) != 0)
		return;

	if (fdtbus_intr_establish_xname(phandle, 0, IPL_NET, FDT_INTR_MPSAFE,
	    eqos_intr, sc, device_xname(self)) == NULL) {
		aprint_error_dev(self, "failed to establish interrupt on %s\n",
		    intrstr);
		return;
	}
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);
}
