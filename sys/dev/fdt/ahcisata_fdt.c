/* $NetBSD: ahcisata_fdt.c,v 1.3 2021/01/27 03:10:21 thorpej Exp $ */

/*-
 * Copyright (c) 2018 Jared McNeill <jmcneill@invisible.ca>
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

#include <sys/cdefs.h>

__KERNEL_RCSID(0, "$NetBSD: ahcisata_fdt.c,v 1.3 2021/01/27 03:10:21 thorpej Exp $");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/systm.h>

#include <dev/ata/atavar.h>
#include <dev/ic/ahcisatavar.h>

#include <dev/fdt/fdtvar.h>

struct ahcisata_fdt_softc {
	struct ahci_softc sc;
	struct fdtbus_phy *sc_phy;
};

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "snps,dwc-ahci" },
	{ .compat = "generic-ahci" },
	DEVICE_COMPAT_EOL
};

static const struct device_compatible_entry dwc_rk3568_compat_data[] = {
	{ .compat = "rockchip,rk3568-dwc-ahci" },
	DEVICE_COMPAT_EOL
};

static int
ahcisata_fdt_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
ahcisata_fdt_attach(device_t parent, device_t self, void *aux)
{
	struct ahcisata_fdt_softc * const fsc = device_private(self);
	struct ahci_softc * const sc = &fsc->sc;
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct fdtbus_reset *rst;
	struct clk *clk;
	char intrstr[128];
	bus_addr_t addr;
	bus_size_t size;
	int error;
	int i;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_atac.atac_dev = self;
	sc->sc_dmat = faa->faa_dmat;
	sc->sc_ahcit = faa->faa_bst;
	sc->sc_ahcis = size;
	if (bus_space_map(sc->sc_ahcit, addr, size, 0, &sc->sc_ahcih) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
	if (of_getprop_uint32(phandle, "ports-implemented", &sc->sc_ahci_ports) != 0)
		sc->sc_save_init_data = true;

	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error(": failed to decode interrupt\n");
		return;
	}

	for (i = 0; (clk = fdtbus_clock_get_index(phandle, i)) != NULL; i++)
		if (clk_enable(clk) != 0) {
			aprint_error(": couldn't enable clock #%d\n", i);
			return;
		}
	for (i = 0; (rst = fdtbus_reset_get_index(phandle, i)) != NULL; i++)
		if (fdtbus_reset_deassert(rst) != 0) {
			aprint_error(": couldn't de-assert reset #%d\n", i);
			return;
		}

	/* Enable the PHY, if any, before touching the controller. */
	fsc->sc_phy = fdtbus_phy_get_index(phandle, 0);
	if (fsc->sc_phy != NULL) {
		error = fdtbus_phy_enable(fsc->sc_phy, true);
		if (error != 0) {
			aprint_error(": couldn't enable PHY (%d)\n", error);
			return;
		}
	}

	/*
	 * DWC AHCI (rk3568): the OOB timing register (OOBR, 0xbc) survives
	 * the HBA reset, and U-Boot leaves 0x02060b14 there, which breaks
	 * COMINIT detection (Linux relies on the power-on default
	 * 0x04070c15).  Program it with the two-step write-enable sequence,
	 * and TIMER1MS to the value Linux uses.
	 */
	if (of_compatible_match(phandle, dwc_rk3568_compat_data)) {
		bus_space_write_4(sc->sc_ahcit, sc->sc_ahcih, 0xbc, 0x80000000);
		bus_space_write_4(sc->sc_ahcit, sc->sc_ahcih, 0xbc, 0x04070c15);
		bus_space_write_4(sc->sc_ahcit, sc->sc_ahcih, 0xe0, 300000);
		aprint_verbose_dev(self, "OOBR=0x04070c15 TIMER1MS=300000\n");
	}

	/* bring-up evidence: raw port link state right after PHY enable.
	 * With the U-Boot preboot SATA init inherited this shows the live
	 * link (SSTS DET=3); from a cold controller it reads 0. */
	if (fsc->sc_phy != NULL) {
		aprint_normal_dev(self,
		    "post-phy SSTS=0x%08x SCTL=0x%08x CMD=0x%08x SERR=0x%08x\n",
		    bus_space_read_4(sc->sc_ahcit, sc->sc_ahcih, 0x128),
		    bus_space_read_4(sc->sc_ahcit, sc->sc_ahcih, 0x12c),
		    bus_space_read_4(sc->sc_ahcit, sc->sc_ahcih, 0x118),
		    bus_space_read_4(sc->sc_ahcit, sc->sc_ahcih, 0x130));
	}

	aprint_naive("\n");
	aprint_normal(": AHCI SATA controller\n");

	if (fdtbus_intr_establish_xname(phandle, 0, IPL_BIO, 0,
	    ahci_intr, sc, device_xname(self)) == NULL) {
		aprint_error_dev(self,
		    "failed to establish interrupt on %s\n", intrstr);
		return;
	}
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);

	ahci_attach(sc);
}

CFATTACH_DECL_NEW(ahcisata_fdt, sizeof(struct ahcisata_fdt_softc),
	ahcisata_fdt_match, ahcisata_fdt_attach, NULL, NULL);
