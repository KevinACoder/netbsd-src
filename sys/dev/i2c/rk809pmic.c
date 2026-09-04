/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by the RK3568 lab bring-up effort.
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
 * Rockchip RK809 PMIC - read-only inventory driver.
 *
 * The RK809 shares the RK817 register map (verified against the
 * standalone fdwi2c_rk809_example on this board and the Linux
 * drivers/mfd/rk808.c + regulator/rk808-regulator.c).  The stock
 * NetBSD rkpmic driver only understands the RK805/RK808 layout, so
 * this driver attaches instead and dumps the power-tree state at
 * attach: chip ID (0xed/0xee, expect 0x8090), the POWER_EN register
 * trio, the DCDC on-VSEL registers and the LDO on-VSEL registers with
 * their decoded voltages.
 *
 * Deliberately read-only and framework-free (no todr, no rkreg
 * provider): the board's system clock is the external RX8025T and
 * the rails are firmware-configured before the kernel boots.  On the
 * RK3568-E4AP5G1-ITX: DCDC1=vdd_logic, DCDC2=vdd_gpu, DCDC3=
 * vcc_pcie_sw, DCDC4=vdd_npu, DCDC5=vcc_1v8, LDO5=vccio_sd.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>

#include <dev/i2c/i2cvar.h>

#if defined(__arm__) || defined(__aarch64__)
#include "opt_fdt.h"
#endif

#ifdef FDT
#include <dev/fdt/fdtvar.h>
#endif

/* Chip identity (RK817 layout) */
#define RK817_ID_MSB		0xed
#define RK817_ID_LSB		0xee
#define RK8XX_ID_MSK		0xfff0
#define RK809_ID		0x8090

/* Power enables: rail id r -> reg 0xb1 + r/4, on bit r%4. */
#define RK817_POWER_EN_REG(i)	(0xb1 + (i))

/* DCDC on-VSEL registers (sleep VSEL is +1, unused here). */
#define RK817_BUCK1_ON_VSEL_REG	0xbb
#define RK817_BUCK2_ON_VSEL_REG	0xbe
#define RK817_BUCK3_ON_VSEL_REG	0xc1
#define RK817_BUCK4_ON_VSEL_REG	0xc4
#define RK817_BUCK_VSEL_MASK	0x7f

/* RK809-only DCDC5 ("boost"/buck5): discrete selector. */
#define RK809_BUCK5_ON_VSEL_REG	0xde
#define RK809_BUCK5_VSEL_MASK	0x7

/* LDO on-VSEL registers: LDO(n) -> 0xcc + (n-1)*2. */
#define RK817_LDO_ON_VSEL_REG(n)	(0xcc + ((n) - 1) * 2)

#define RK809_NLDO		7	/* LDO1..LDO7 wired on this board */

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "rockchip,rk809" },
	DEVICE_COMPAT_EOL
};

struct rk809pmic_softc {
	device_t sc_dev;
	i2c_tag_t sc_tag;
	int sc_addr;
};

static int rk809pmic_match(device_t, cfdata_t, void *);
static void rk809pmic_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rk809pmic, sizeof(struct rk809pmic_softc),
    rk809pmic_match, rk809pmic_attach, NULL, NULL);

static int	rk809pmic_rd1(struct rk809pmic_softc *, uint8_t, uint8_t *);
static u_int	rk809pmic_buck_uv(u_int vsel);
static u_int	rk809pmic_buck5_uv(u_int vsel);
static u_int	rk809pmic_ldo_uv(u_int vsel);
static void	rk809pmic_print_rail(struct rk809pmic_softc *, const char *,
		    uint8_t, u_int, const char *);

static int
rk809pmic_match(device_t parent, cfdata_t cf, void *aux)
{
	struct i2c_attach_args *ia = aux;
	int match_result;

	if (iic_use_direct_match(ia, cf, compat_data, &match_result))
		return match_result;

	/* indirect config - RK809 sits at 0x20 */
	if (ia->ia_addr == 0x20)
		return I2C_MATCH_ADDRESS_ONLY;

	return 0;
}

static void
rk809pmic_attach(device_t parent, device_t self, void *aux)
{
	struct rk809pmic_softc *sc = device_private(self);
	struct i2c_attach_args *ia = aux;
	static const uint8_t buck_regs[] = {
		RK817_BUCK1_ON_VSEL_REG,
		RK817_BUCK2_ON_VSEL_REG,
		RK817_BUCK3_ON_VSEL_REG,
		RK817_BUCK4_ON_VSEL_REG,
	};
	uint8_t id_msb, id_lsb, en[3], vsel;
	uint16_t chip_id;
	u_int rail;
	int error;

	sc->sc_dev = self;
	sc->sc_tag = ia->ia_tag;
	sc->sc_addr = ia->ia_addr;

	if ((error = iic_acquire_bus(sc->sc_tag, 0)) != 0) {
		aprint_error(": failed to acquire bus (error %d)\n", error);
		return;
	}

	error = rk809pmic_rd1(sc, RK817_ID_MSB, &id_msb);
	if (error == 0)
		error = rk809pmic_rd1(sc, RK817_ID_LSB, &id_lsb);
	if (error != 0) {
		iic_release_bus(sc->sc_tag, 0);
		aprint_error(": couldn't read chip ID (error %d)\n", error);
		return;
	}

	memset(en, 0, sizeof(en));
	error = rk809pmic_rd1(sc, RK817_POWER_EN_REG(0), &en[0]);
	if (error == 0)
		error = rk809pmic_rd1(sc, RK817_POWER_EN_REG(1), &en[1]);
	if (error == 0)
		error = rk809pmic_rd1(sc, RK817_POWER_EN_REG(2), &en[2]);

	aprint_naive(": Rockchip RK809 PMIC\n");
	chip_id = ((id_msb << 8) | id_lsb) & RK8XX_ID_MSK;
	aprint_normal(": Rockchip RK809 PMIC (ID 0x%04x)%s\n", chip_id,
	    chip_id == RK809_ID ? "" : " (expected 0x8090)");
	if (error == 0)
		aprint_normal_dev(self, "POWER_EN %02x=%02x %02x=%02x %02x=%02x\n",
		    RK817_POWER_EN_REG(0), en[0],
		    RK817_POWER_EN_REG(1), en[1],
		    RK817_POWER_EN_REG(2), en[2]);

	/* DCDC1..DCDC4: linear 500-1500 mV in 12.5 mV steps, then
	 * 1600-2400 mV in 100 mV steps.  Rail enable bits live at
	 * 0xb1 + rail/4, bit rail%4. */
	for (rail = 0; rail < __arraycount(buck_regs); rail++) {
		char name[sizeof("DCDC4")];
		bool on;
		if (rk809pmic_rd1(sc, buck_regs[rail], &vsel) != 0)
			continue;
		on = (en[rail / 4] & __BIT(rail % 4)) != 0;
		snprintf(name, sizeof(name), "DCDC%u", rail + 1);
		rk809pmic_print_rail(sc, name, vsel,
		    rk809pmic_buck_uv(vsel & RK817_BUCK_VSEL_MASK),
		    on ? "enabled" : "disabled");
	}

	/* DCDC5 (RK809 buck5): discrete 1.5/1.8/.../3.6 V selector. */
	if (rk809pmic_rd1(sc, RK809_BUCK5_ON_VSEL_REG, &vsel) == 0)
		rk809pmic_print_rail(sc, "DCDC5", vsel,
		    rk809pmic_buck5_uv(vsel & 0x7),
		    (en[1] & __BIT(0)) ? "enabled" : "disabled");

	/* LDO1..LDO7: 600 mV + 25 mV per step (mask 0x7f). */
	for (rail = 0; rail < RK809_NLDO; rail++) {
		char name[sizeof("LDO7")];
		bool on;
		if (rk809pmic_rd1(sc, RK817_LDO_ON_VSEL_REG(rail + 1),
		    &vsel) != 0)
			continue;
		on = (en[(5 + rail) / 4] & __BIT((5 + rail) % 4)) != 0;
		snprintf(name, sizeof(name), "LDO%u", rail + 1);
		rk809pmic_print_rail(sc, name, vsel,
		    rk809pmic_ldo_uv(vsel & RK817_BUCK_VSEL_MASK),
		    on ? "enabled" : "disabled");
	}

	iic_release_bus(sc->sc_tag, 0);
}

static int
rk809pmic_rd1(struct rk809pmic_softc *sc, uint8_t reg, uint8_t *valp)
{

	return iic_smbus_read_byte(sc->sc_tag, sc->sc_addr, reg, valp, 0);
}

static void
rk809pmic_print_rail(struct rk809pmic_softc *sc, const char *name,
    uint8_t vsel, u_int uv, const char *state)
{

	aprint_normal_dev(sc->sc_dev,
	    "%-5s vsel 0x%02x -> %u.%03u V  %s\n",
	    name, vsel, uv / 1000000, (uv % 1000000) / 1000, state);
}

/* RK817 BUCK1-4: sel 0..80 -> 500 mV + sel * 12.5 mV,
 * sel >= 81 -> 1600 mV + (sel - 81) * 100 mV. */
static u_int
rk809pmic_buck_uv(u_int vsel)
{

	if (vsel <= 80)
		return 500000 + vsel * 12500;
	return 1600000 + (vsel - 81) * 100000;
}

/* RK809 BUCK5 discrete table (Linux rk809_buck5_voltage_ranges). */
static u_int
rk809pmic_buck5_uv(u_int vsel)
{
	static const u_int table[] = {
		1500000, 1800000, 2000000, 2200000,
		2800000, 3000000, 3300000, 3600000,
	};

	return table[vsel & 0x7];
}

/* RK817 LDO: 600 mV + sel * 25 mV (standalone-verified on LDO5). */
static u_int
rk809pmic_ldo_uv(u_int vsel)
{

	return 600000 + vsel * 25000;
}
