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
 * Epson RX-8025T I2C real-time clock (battery-backed RTC on the
 * RK3568-E4AP5G1-ITX board, i2c1 @ 0x32 behind a CR2032).
 *
 * Register semantics follow the Linux rtc-rx8025 driver and the lab's
 * standalone fdwi2c_rtc_example (which proved the plain register
 * addressing of the -T variant: unlike the RX-8025SA/AC, the command
 * byte is the raw register offset, and one 7-byte transaction covers
 * 0x00-0x06).
 *
 * The interesting bits for a battery-operated board live in CTRL2:
 *   PON (power-on reset) and VDET (backup battery voltage drop) mean
 *   the clock state is invalid until a set_time, and a clear XST
 *   (oscillator stop) means the 32.768 kHz crystal was seen stopped.
 *   Healthy state is XST=1, PON=0, VDET=0; set_time re-arms it.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/kernel.h>

#include <dev/clock_subr.h>

#include <dev/i2c/i2cvar.h>

#if defined(__arm__) || defined(__aarch64__)
#include "opt_fdt.h"
#endif

#ifdef FDT
#include <dev/fdt/fdtvar.h>
#endif

/* Registers */
#define RX8025_R_SECOND		0x00
#define  RX8025_M_SECOND	0x7f
#define  RX8025_B_STOP		0x80	/* stop bit: 1 = clock halted */
#define RX8025_R_MINUTE		0x01
#define  RX8025_M_MINUTE	0x7f
#define RX8025_R_HOUR		0x02
#define  RX8025_M_HOUR		0x3f
#define RX8025_R_WEEKDAY	0x03
#define  RX8025_M_WEEKDAY	0x7f
#define RX8025_R_DAY		0x04
#define  RX8025_M_DAY		0x3f
#define RX8025_R_MONTH		0x05
#define  RX8025_M_MONTH		0x1f
#define RX8025_R_YEAR		0x06
#define  RX8025_M_YEAR		0xff
#define RX8025_R_CTRL1		0x0e
#define  RX8025_B_TEST		0x08	/* test bit: must be 0 */
#define RX8025_R_CTRL2		0x0f
#define  RX8025_B_DAFG		0x01
#define  RX8025_B_WAFG		0x02
#define  RX8025_B_CTFG		0x04
#define  RX8025_B_PON		0x10	/* power-on reset */
#define  RX8025_B_XST		0x20	/* oscillator stop (1 = ok) */
#define  RX8025_B_VDET		0x40	/* battery voltage drop */

#define RX8025_ADDR		0x32
#define RX8025_NTIME_REGS	(RX8025_R_YEAR + 1)

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "epson,rx8025" },
	DEVICE_COMPAT_EOL
};

struct rx8025rtc_softc {
	device_t sc_dev;
	i2c_tag_t sc_tag;
	int sc_addr;
	struct todr_chip_handle sc_todr;
};

static int rx8025rtc_match(device_t, cfdata_t, void *);
static void rx8025rtc_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rx8025rtc, sizeof(struct rx8025rtc_softc),
    rx8025rtc_match, rx8025rtc_attach, NULL, NULL);

static int rx8025rtc_gettime(struct todr_chip_handle *, struct clock_ymdhms *);
static int rx8025rtc_settime(struct todr_chip_handle *, struct clock_ymdhms *);

static int
rx8025rtc_match(device_t parent, cfdata_t cf, void *aux)
{
	struct i2c_attach_args *ia = aux;
	int match_result;

	if (iic_use_direct_match(ia, cf, compat_data, &match_result))
		return match_result;

	/* indirect config - check typical address */
	if (ia->ia_addr == RX8025_ADDR)
		return I2C_MATCH_ADDRESS_ONLY;

	return 0;
}

static void
rx8025rtc_attach(device_t parent, device_t self, void *aux)
{
	struct rx8025rtc_softc *sc = device_private(self);
	struct i2c_attach_args *ia = aux;
	uint8_t ctrl2;
	int error;

	aprint_naive(": Real-time Clock\n");
	aprint_normal(": Epson RX-8025T Real-time Clock\n");

	sc->sc_dev = self;
	sc->sc_tag = ia->ia_tag;
	sc->sc_addr = ia->ia_addr;
	sc->sc_todr.cookie = sc;
	sc->sc_todr.todr_gettime_ymdhms = rx8025rtc_gettime;
	sc->sc_todr.todr_settime_ymdhms = rx8025rtc_settime;
	sc->sc_todr.todr_setwen = NULL;

	if ((error = iic_acquire_bus(sc->sc_tag, 0)) != 0) {
		aprint_error_dev(self, "failed to acquire bus for attach\n");
		return;
	}
	error = iic_smbus_read_byte(sc->sc_tag, sc->sc_addr,
	    RX8025_R_CTRL2, &ctrl2, 0);
	iic_release_bus(sc->sc_tag, 0);
	if (error != 0) {
		aprint_error_dev(self, "couldn't read CTRL2 (error %d)\n",
		    error);
		return;
	}

	/*
	 * Battery / crystal health report - the whole point of an
	 * externally battery-backed RTC.  Healthy: XST set, PON and
	 * VDET clear.
	 */
	if (ctrl2 & RX8025_B_PON) {
		aprint_error_dev(self,
		    "PON: power-on reset, clock state invalid (set date)\n");
	} else if ((ctrl2 & RX8025_B_XST) == 0) {
		aprint_error_dev(self,
		    "XST clear: oscillator stop detected (set date)\n");
	} else if (ctrl2 & RX8025_B_VDET) {
		aprint_error_dev(self,
		    "VDET: backup battery voltage drop detected\n");
	} else {
		aprint_normal_dev(self,
		    "battery/oscillator healthy (XST=1 PON=0 VDET=0)\n");
	}

#ifdef FDT
	fdtbus_todr_attach(self, ia->ia_cookie, &sc->sc_todr);
#else
	todr_attach(&sc->sc_todr);
#endif
}

static int
rx8025rtc_gettime(struct todr_chip_handle *ch, struct clock_ymdhms *dt)
{
	struct rx8025rtc_softc *sc = ch->cookie;
	uint8_t bcd[RX8025_NTIME_REGS];
	uint8_t reg = RX8025_R_SECOND, ctrl2;
	int error;

	if ((error = iic_acquire_bus(sc->sc_tag, 0)) != 0)
		return error;

	error = iic_exec(sc->sc_tag, I2C_OP_READ_WITH_STOP, sc->sc_addr,
	    &reg, 1, bcd, RX8025_NTIME_REGS, 0);
	if (error == 0)
		error = iic_smbus_read_byte(sc->sc_tag, sc->sc_addr,
		    RX8025_R_CTRL2, &ctrl2, 0);

	iic_release_bus(sc->sc_tag, 0);
	if (error != 0)
		return error;

	/* Invalid state until a set_time re-arms it. */
	if ((ctrl2 & RX8025_B_PON) != 0 || (ctrl2 & RX8025_B_XST) == 0)
		return EIO;
	/* Year register reads 0xff on a never-initialized part. */
	if (bcd[RX8025_R_YEAR] > 0x99)
		return EIO;

	dt->dt_sec = bcdtobin(bcd[RX8025_R_SECOND] & RX8025_M_SECOND);
	dt->dt_min = bcdtobin(bcd[RX8025_R_MINUTE] & RX8025_M_MINUTE);
	dt->dt_hour = bcdtobin(bcd[RX8025_R_HOUR] & RX8025_M_HOUR);
	dt->dt_wday = bcdtobin(bcd[RX8025_R_WEEKDAY] & RX8025_M_WEEKDAY);
	dt->dt_day = bcdtobin(bcd[RX8025_R_DAY] & RX8025_M_DAY);
	dt->dt_mon = bcdtobin(bcd[RX8025_R_MONTH] & RX8025_M_MONTH);
	dt->dt_year = 2000 + bcdtobin(bcd[RX8025_R_YEAR] & RX8025_M_YEAR);

	return 0;
}

static int
rx8025rtc_settime(struct todr_chip_handle *ch, struct clock_ymdhms *dt)
{
	struct rx8025rtc_softc *sc = ch->cookie;
	uint8_t bcd[RX8025_NTIME_REGS];
	uint8_t reg = RX8025_R_SECOND, ctrl1, ctrl2;
	int error;

	if (dt->dt_year < 2000 || dt->dt_year > 2099)
		return EINVAL;

	bcd[RX8025_R_SECOND] = bintobcd(dt->dt_sec) & RX8025_M_SECOND;
	bcd[RX8025_R_MINUTE] = bintobcd(dt->dt_min) & RX8025_M_MINUTE;
	bcd[RX8025_R_HOUR] = bintobcd(dt->dt_hour) & RX8025_M_HOUR;
	bcd[RX8025_R_WEEKDAY] = bintobcd(dt->dt_wday) & RX8025_M_WEEKDAY;
	bcd[RX8025_R_DAY] = bintobcd(dt->dt_day) & RX8025_M_DAY;
	bcd[RX8025_R_MONTH] = bintobcd(dt->dt_mon) & RX8025_M_MONTH;
	bcd[RX8025_R_YEAR] = bintobcd(dt->dt_year % 100);

	if ((error = iic_acquire_bus(sc->sc_tag, 0)) != 0)
		return error;

	/* Clear the TEST bit, keep the 12/24 selection. */
	error = iic_smbus_read_byte(sc->sc_tag, sc->sc_addr,
	    RX8025_R_CTRL1, &ctrl1, 0);
	if (error != 0)
		goto out;
	error = iic_smbus_write_byte(sc->sc_tag, sc->sc_addr,
	    RX8025_R_CTRL1, ctrl1 & ~RX8025_B_TEST, 0);
	if (error != 0)
		goto out;

	/*
	 * Two-phase write, per the standalone-verified sequence: halt
	 * the counter via the STOP bit while the registers are written,
	 * then resume.
	 */
	bcd[RX8025_R_SECOND] |= RX8025_B_STOP;
	error = iic_exec(sc->sc_tag, I2C_OP_WRITE_WITH_STOP, sc->sc_addr,
	    &reg, 1, bcd, RX8025_NTIME_REGS, 0);
	if (error != 0)
		goto out;
	bcd[RX8025_R_SECOND] &= ~RX8025_B_STOP;
	error = iic_exec(sc->sc_tag, I2C_OP_WRITE_WITH_STOP, sc->sc_addr,
	    &reg, 1, &bcd[RX8025_R_SECOND], 1, 0);
	if (error != 0)
		goto out;

	/* Re-arm the validity flags: clear PON/VDET/stale IRQ flags,
	 * set XST (Linux rx8025_reset_validity semantics). */
	error = iic_smbus_read_byte(sc->sc_tag, sc->sc_addr,
	    RX8025_R_CTRL2, &ctrl2, 0);
	if (error != 0)
		goto out;
	ctrl2 &= ~(RX8025_B_PON | RX8025_B_VDET | RX8025_B_CTFG |
	    RX8025_B_DAFG | RX8025_B_WAFG);
	ctrl2 |= RX8025_B_XST;
	error = iic_smbus_write_byte(sc->sc_tag, sc->sc_addr,
	    RX8025_R_CTRL2, ctrl2, 0);

out:
	iic_release_bus(sc->sc_tag, 0);

	return error;
}
