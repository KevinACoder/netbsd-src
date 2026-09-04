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
 * Texas Instruments INA3221 triple-channel power monitor (i2c).
 *
 * Exposes bus voltage (ENVSYS_SVOLTS_DC) and channel current
 * (ENVSYS_SAMPS) for each of the three channels through sysmon_envsys.
 *
 * Scaling follows the datasheet except where the RK3568-E4AP5G1-ITX
 * lab measurement said otherwise:
 *   - shunt voltage: 40 uV/LSB, signed; current = Vshunt / Rshunt with
 *     Rshunt from the FDT "shunt-resistor-micro-ohms" property
 *     (20000 = 20 mOhm on this board)
 *   - bus voltage: the datasheet LSB is 8 mV, but the standalone
 *     fdwi2c_powermon_example measured raw 0x0ce0/0x13b8/0x2fa8 for the
 *     known 3.3/5/12 V rails, i.e. 1 mV/LSB on this part, which is what
 *     we use.
 *
 * On the E4AP5G1-ITX the channels monitor ch1 = VDD3V3, ch2 = VDD5V,
 * ch3 = VDD12V (vendor dtsi sys_powermon @ i2c1 0x40).
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/mutex.h>

#include <dev/i2c/i2cvar.h>
#include <dev/sysmon/sysmonvar.h>

#if defined(__arm__) || defined(__aarch64__)
#include "opt_fdt.h"
#endif

#ifdef FDT
#include <dev/fdt/fdtvar.h>
#endif

/* Registers */
#define INA3221_R_CONF		0x00
#define  INA3221_CONF_CH1_EN	__BIT(15)
#define  INA3221_CONF_CH2_EN	__BIT(14)
#define  INA3221_CONF_CH3_EN	__BIT(13)
#define INA3221_R_SHUNT(n)	(0x01 + (n) * 2)	/* n = 0..2 */
#define INA3221_R_BUS(n)	(0x02 + (n) * 2)
#define INA3221_R_MFR_ID	0xfe
#define INA3221_MFR_TI		0x5449
#define INA3221_R_DIE_ID	0xff

#define INA3221_SHUNT_LSB_UV	40	/* 40 uV/LSB, signed */
#define INA3221_BUS_LSB_UV	1000	/* 1 mV/LSB, board-measured */

#define INA3221_NCHANNELS	3

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "ti,ina3221" },
	DEVICE_COMPAT_EOL
};

struct ina3221_channel {
	envsys_data_t	c_vbus;
	envsys_data_t	c_current;
	const char *	c_desc;
};

struct ina3221_softc {
	device_t		sc_dev;
	i2c_tag_t		sc_tag;
	int			sc_addr;
	kmutex_t		sc_lock;

	struct sysmon_envsys	*sc_sme;
	struct ina3221_channel	sc_chans[INA3221_NCHANNELS];
	u_int			sc_shunt_uohm;
};

static const struct ina3221_channel_desc {
	const char *desc;
	uint32_t chen;
} ina3221_chan_desc[INA3221_NCHANNELS] = {
	{ .desc = "VDD3V3", .chen = INA3221_CONF_CH1_EN },
	{ .desc = "VDD5V",  .chen = INA3221_CONF_CH2_EN },
	{ .desc = "VDD12V", .chen = INA3221_CONF_CH3_EN },
};

static int ina3221_match(device_t, cfdata_t, void *);
static void ina3221_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(ina3221mon, sizeof(struct ina3221_softc),
    ina3221_match, ina3221_attach, NULL, NULL);

static int	ina3221_rd2(struct ina3221_softc *, uint8_t, uint16_t *);
static void	ina3221_refresh(struct sysmon_envsys *, envsys_data_t *);

static int
ina3221_match(device_t parent, cfdata_t cf, void *aux)
{
	struct i2c_attach_args *ia = aux;
	int match_result;

	return iic_use_direct_match(ia, cf, compat_data, &match_result) ?
	    match_result : 0;
}

static void
ina3221_attach(device_t parent, device_t self, void *aux)
{
	struct ina3221_softc *sc = device_private(self);
	struct i2c_attach_args *ia = aux;
	uint16_t mfr, conf;
	int error;

	sc->sc_dev = self;
	sc->sc_tag = ia->ia_tag;
	sc->sc_addr = ia->ia_addr;
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NONE);

	sc->sc_shunt_uohm = 10000;
	of_getprop_uint32(ia->ia_cookie, "shunt-resistor-micro-ohms",
	    &sc->sc_shunt_uohm);

	if ((error = iic_acquire_bus(sc->sc_tag, 0)) != 0) {
		aprint_error(": failed to acquire bus (error %d)\n", error);
		return;
	}
	error = ina3221_rd2(sc, INA3221_R_MFR_ID, &mfr);
	if (error == 0)
		error = ina3221_rd2(sc, INA3221_R_CONF, &conf);
	iic_release_bus(sc->sc_tag, 0);
	if (error != 0) {
		aprint_error(": couldn't read registers (error %d)\n", error);
		return;
	}
	if (mfr != INA3221_MFR_TI) {
		aprint_error(": MFR_ID 0x%04x != 0x%04x, bailing out\n",
		    mfr, INA3221_MFR_TI);
		return;
	}

	aprint_naive(": Power Monitor\n");
	aprint_normal(": TI INA3221 Power Monitor (conf 0x%04x, "
	    "shunt %u mOhm)\n", conf, sc->sc_shunt_uohm / 1000);

	sc->sc_sme = sysmon_envsys_create();
	sc->sc_sme->sme_name = device_xname(self);
	sc->sc_sme->sme_cookie = sc;
	sc->sc_sme->sme_refresh = ina3221_refresh;

	for (u_int n = 0; n < INA3221_NCHANNELS; n++) {
		struct ina3221_channel *ch = &sc->sc_chans[n];

		ch->c_desc = ina3221_chan_desc[n].desc;

		ch->c_vbus.units = ENVSYS_SVOLTS_DC;
		ch->c_vbus.state = ENVSYS_SINVALID;
		snprintf(ch->c_vbus.desc, sizeof(ch->c_vbus.desc), "%s",
		    ch->c_desc);

		ch->c_current.units = ENVSYS_SAMPS;
		ch->c_current.state = ENVSYS_SINVALID;
		snprintf(ch->c_current.desc, sizeof(ch->c_current.desc),
		    "%s I", ch->c_desc);

		if ((conf & ina3221_chan_desc[n].chen) == 0) {
			aprint_normal_dev(self, "channel %u (%s) disabled\n",
			    n + 1, ch->c_desc);
			continue;
		}
		if (sysmon_envsys_sensor_attach(sc->sc_sme, &ch->c_vbus) ||
		    sysmon_envsys_sensor_attach(sc->sc_sme, &ch->c_current)) {
			aprint_error_dev(self, "sensor_attach failed\n");
			goto out;
		}
	}

	sysmon_envsys_register(sc->sc_sme);
	return;

out:
	sysmon_envsys_destroy(sc->sc_sme);
	sc->sc_sme = NULL;
}

static int
ina3221_rd2(struct ina3221_softc *sc, uint8_t reg, uint16_t *valp)
{
	uint8_t buf[2];
	int error;

	error = iic_exec(sc->sc_tag, I2C_OP_READ_WITH_STOP, sc->sc_addr,
	    &reg, 1, buf, 2, 0);
	if (error == 0)
		*valp = (buf[0] << 8) | buf[1];

	return error;
}

static void
ina3221_refresh(struct sysmon_envsys *sme, envsys_data_t *edata)
{
	struct ina3221_softc *sc = sme->sme_cookie;
	struct ina3221_channel *ch = NULL;
	uint16_t bus_raw = 0, shunt_raw = 0;
	u_int n;
	int error;

	for (n = 0; n < INA3221_NCHANNELS; n++) {
		if (edata == &sc->sc_chans[n].c_vbus ||
		    edata == &sc->sc_chans[n].c_current) {
			ch = &sc->sc_chans[n];
			break;
		}
	}
	if (ch == NULL)
		return;

	mutex_enter(&sc->sc_lock);
	error = iic_acquire_bus(sc->sc_tag, 0);
	if (error == 0) {
		error = ina3221_rd2(sc, INA3221_R_BUS(n), &bus_raw);
		if (error == 0)
			error = ina3221_rd2(sc, INA3221_R_SHUNT(n),
			    &shunt_raw);
		iic_release_bus(sc->sc_tag, 0);
	}
	mutex_exit(&sc->sc_lock);

	if (error != 0) {
		edata->state = ENVSYS_SINVALID;
		return;
	}

	if (edata == &ch->c_vbus) {
		edata->value_cur = (uint32_t)bus_raw * INA3221_BUS_LSB_UV;
	} else {
		/* Shunt register: 15-bit signed, 40 uV/LSB.  Current in
		 * uA = shunt_uV * 1e6 / shunt_uOhm. */
		int32_t shunt_uv = (int16_t)shunt_raw * INA3221_SHUNT_LSB_UV;

		if (shunt_uv < 0)
			shunt_uv = 0;
		edata->value_cur =
		    (uint32_t)shunt_uv * 1000000 / sc->sc_shunt_uohm;
	}
	edata->state = ENVSYS_SVALID;
}
