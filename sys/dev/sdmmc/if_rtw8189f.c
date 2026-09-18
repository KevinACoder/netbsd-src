/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by the RK3568 lab (NetBSD/RTL8189FTV SDIO bring-up).
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
 * Realtek RTL8189FTV SDIO 802.11 driver (RTL8188F silicon, 024c:f179).
 *
 * A clean-room port: the chip sequences come from the vendor rtl8189fs
 * Linux driver used as reference only (see rtw8189f_chip.c), the
 * transport talks sdmmc(4) directly (rtw8189f_sdio.c), and this file is
 * the autoconf + net80211 side, following the same pre-HT net80211
 * softmac pattern as if_rtw88.c (11b/g only, software crypto, scan via
 * ic_newstate + callout).
 *
 * The module has no card-detect pin (broken-cd in the board DTS) and its
 * REG_ON is strapped high, so SDIO enumeration happens on its own; this
 * driver matches CIS 0x024c/0xf179 on function 1.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/mbuf.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/socket.h>
#include <sys/callout.h>
#include <sys/mutex.h>
#include <sys/pmf.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_proto.h>

#include <dev/sdmmc/sdmmcvar.h>
#include <dev/firmload.h>

#include "rtw8189fvar.h"

#define RTW8189F_DRVNAME		"if_rtw8189f"
#define RTW8189F_FWNAME			"rtw8189f_fw.bin"
#define RTW8189F_SDIO_VENDOR		0x024c
#define RTW8189F_SDIO_PRODUCT		0xf179

static int	rtw8189f_match(device_t, cfdata_t, void *);
static void	rtw8189f_attach(device_t, device_t, void *);
static int	rtw8189f_detach(device_t, int);
static int	rtw8189f_activate(device_t, enum devact);

CFATTACH_DECL_NEW(rtw8189f, sizeof(struct rtw8189f_softc), rtw8189f_match,
    rtw8189f_attach, rtw8189f_detach, rtw8189f_activate);

static void	rtw8189f_attachhook(device_t);
static int	rtw8189f_load_firmware(struct rtw8189f_softc *);
static int	rtw8189f_init(struct ifnet *);
static void	rtw8189f_stop(struct ifnet *, int);
static void	rtw8189f_start(struct ifnet *);
static void	rtw8189f_watchdog(struct ifnet *);
static int	rtw8189f_ioctl(struct ifnet *, u_long, void *);
static int	rtw8189f_newstate(struct ieee80211com *, enum ieee80211_state,
		    int);

static int
rtw8189f_match(device_t parent, cfdata_t match, void *aux)
{
	struct sdmmc_attach_args *saa = aux;
	struct sdmmc_cis *cis;

	/* Not SDIO. */
	if (saa->sf == NULL)
		return 0;

	cis = &saa->sf->sc->sc_fn0->cis;
	if (cis->manufacturer != RTW8189F_SDIO_VENDOR ||
	    cis->product != RTW8189F_SDIO_PRODUCT)
		return 0;

	/* One WiFi function only; fn0 is the common control area. */
	if (saa->sf->number != 1)
		return 0;

	return 1;
}

static void
rtw8189f_attach(device_t parent, device_t self, void *aux)
{
	struct rtw8189f_softc *sc = device_private(self);
	struct sdmmc_attach_args *saa = aux;
	uint8_t himr;

	sc->sc_dev = self;
	sc->sc_sf = saa->sf;

	aprint_naive("\n");
	aprint_normal("\n");

	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NONE);
	callout_init(&sc->sc_scan_to, 0);

	/* 512-byte blocks for the CMD53 FIFO/register bursts. */
	if (sdmmc_io_set_blocklen(sc->sc_sf, 512) != 0) {
		aprint_error_dev(self, "cannot set block length\n");
		return;
	}
	if (sdmmc_io_function_enable(sc->sc_sf) != 0) {
		aprint_error_dev(self, "cannot enable function 1\n");
		return;
	}

	/* Transport sanity: SDIO-local HIMR must read back something. */
	himr = rtw8189f_sdiolocal_read_1(sc, RTW8189F_SDIO_REG_HIMR);
	aprint_normal_dev(self, "fn1 enabled, 512B blocks, HIMR 0x%02x\n",
	    himr);

	/*
	 * The chip bring-up needs its firmware image from the root
	 * filesystem, which is not mounted yet at attach time.
	 */
	config_mountroot(self, rtw8189f_attachhook);
}

static void
rtw8189f_attachhook(device_t self)
{
	struct rtw8189f_softc *sc = device_private(self);
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = &sc->sc_if;
	int attempt, error;

	for (attempt = 0; attempt < 40; attempt++) {
		if (sc->sc_dying)
			return;
		error = rtw8189f_load_firmware(sc);
		if (error != ENOENT)
			break;
		/* firmware(9) needs the root filesystem: wait for it. */
		kpause("rtw8189fup", false, mstohz(100), NULL);
	}
	if (error != 0) {
		aprint_error_dev(self, "cannot load firmware (%d)\n", error);
		return;
	}

	/* Card enable + self test + eFuse MAC + firmware download. */
	if (rtw8189f_power_on(sc) != 0) {
		aprint_error_dev(self, "power on failed\n");
		return;
	}
	if (rtw8189f_power_on_check(sc) != 0)
		return;
	if (rtw8189f_efuse_read(sc) != 0) {
		aprint_error_dev(self, "eFuse read failed\n");
		return;
	}
	aprint_normal_dev(self, "Ethernet address %s\n",
	    ether_sprintf(sc->sc_mac_addr));
	if (rtw8189f_fw_download(sc) != 0) {
		aprint_error_dev(self, "firmware download failed\n");
		return;
	}

	/*
	 * Register with net80211.  The MAC/BB/RF init (chip start) is a
	 * later milestone; the interface exists but ifconfig up fails
	 * with EOPNOTSUPP until then.
	 */
	ic->ic_ifp = ifp;
	ic->ic_phytype = IEEE80211_T_OFDM;
	ic->ic_opmode = IEEE80211_M_STA;
	ic->ic_state = IEEE80211_S_INIT;
	ic->ic_caps =
	    IEEE80211_C_MONITOR |
	    IEEE80211_C_SHPREAMBLE |
	    IEEE80211_C_SHSLOT |
	    IEEE80211_C_WPA;		/* 802.11i (software crypto) */

	/* 11b/g only: net80211 here has no HT/VHT. */
	ic->ic_sup_rates[IEEE80211_MODE_11B] = ieee80211_std_rateset_11b;
	ic->ic_sup_rates[IEEE80211_MODE_11G] = ieee80211_std_rateset_11g;

	for (int i = 1; i <= 14; i++) {
		ic->ic_channels[i].ic_freq =
		    ieee80211_ieee2mhz(i, IEEE80211_CHAN_2GHZ);
		ic->ic_channels[i].ic_flags =
		    IEEE80211_CHAN_CCK | IEEE80211_CHAN_OFDM |
		    IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ;
	}

	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_init = rtw8189f_init;
	ifp->if_ioctl = rtw8189f_ioctl;
	ifp->if_start = rtw8189f_start;
	ifp->if_watchdog = rtw8189f_watchdog;
	IFQ_SET_READY(&ifp->if_snd);
	memcpy(ifp->if_xname, device_xname(sc->sc_dev), IFNAMSIZ);

	if_initialize(ifp);
	ieee80211_ifattach(ic);

	sc->sc_newstate = ic->ic_newstate;
	ic->ic_newstate = rtw8189f_newstate;

	ieee80211_media_init(ic, ieee80211_media_change,
	    ieee80211_media_status);

	ifp->if_percpuq = if_percpuq_create(ifp);
	if_register(ifp);

	if_set_sadl(ifp, sc->sc_mac_addr, IEEE80211_ADDR_LEN, false);
	memcpy(ic->ic_myaddr, sc->sc_mac_addr, IEEE80211_ADDR_LEN);

	ieee80211_announce(ic);
	sc->sc_attached = true;

	pmf_device_register(self, NULL, NULL);
}

static int
rtw8189f_load_firmware(struct rtw8189f_softc *sc)
{
	firmware_handle_t fh;
	size_t size;
	int error;

	error = firmware_open(RTW8189F_DRVNAME, RTW8189F_FWNAME, &fh);
	if (error != 0)
		return ENOENT;

	size = firmware_get_size(fh);
	sc->sc_fw = kmem_alloc(size, KM_SLEEP);
	error = firmware_read(fh, 0, sc->sc_fw, size);
	firmware_close(fh);
	if (error != 0) {
		kmem_free(sc->sc_fw, size);
		sc->sc_fw = NULL;
		return error;
	}
	sc->sc_fwsize = size;

	aprint_normal_dev(sc->sc_dev, "loaded %s (%zu bytes)\n",
	    RTW8189F_FWNAME, size);
	return 0;
}

static int
rtw8189f_detach(device_t self, int flags)
{
	struct rtw8189f_softc *sc = device_private(self);
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = &sc->sc_if;
	int s;

	pmf_device_deregister(self);
	s = splnet();
	sc->sc_dying = 1;
	callout_halt(&sc->sc_scan_to, NULL);

	if (sc->sc_attached) {
		ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
		ieee80211_ifdetach(ic);
		if_detach(ifp);
		sc->sc_attached = false;
	}
	splx(s);

	mutex_destroy(&sc->sc_lock);
	callout_destroy(&sc->sc_scan_to);

	return 0;
}

static int
rtw8189f_activate(device_t self, enum devact act)
{
	struct rtw8189f_softc *sc = device_private(self);

	switch (act) {
	case DVACT_DEACTIVATE:
		sc->sc_dying = 1;
		if_deactivate(sc->sc_ic.ic_ifp);
		return 0;
	default:
		return EOPNOTSUPP;
	}
}

/* ------------------------------------------------------------------ */
/* ifnet entry points                                                  */
/* ------------------------------------------------------------------ */

static int
rtw8189f_init(struct ifnet *ifp)
{
	struct rtw8189f_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;

	if (sc->sc_dying)
		return ENXIO;
	if (!sc->sc_fw_ready) {
		aprint_error_dev(sc->sc_dev,
		    "chip not initialised (MAC/BB/RF init pending)\n");
		return EOPNOTSUPP;
	}

	/*
	 * M3 milestone will add the MAC/BB/RF init and start the scan
	 * state machine here.
	 */
	(void)ic;
	return 0;
}

static void
rtw8189f_stop(struct ifnet *ifp, int disable)
{

	ifp->if_timer = 0;
	ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
}

static void
rtw8189f_start(struct ifnet *ifp)
{
	struct rtw8189f_softc *sc = ifp->if_softc;

	if (!sc->sc_fw_ready) {
		if_statinc(ifp, if_oerrors);
		return;
	}
	/* TX path lands with the M3 milestone. */
}

static void
rtw8189f_watchdog(struct ifnet *ifp)
{
	struct rtw8189f_softc *sc = ifp->if_softc;

	ifp->if_timer = 0;
	ieee80211_watchdog(&sc->sc_ic);
}

static int
rtw8189f_ioctl(struct ifnet *ifp, u_long cmd, void *data)
{
	struct rtw8189f_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;
	int error = 0, s;

	s = splnet();

	switch (cmd) {
	case SIOCSIFFLAGS:
		if (sc->sc_dying) {
			error = EIO;
			break;
		}
		if ((error = ifioctl_common(ifp, cmd, data)) != 0)
			break;
		switch (ifp->if_flags & (IFF_UP | IFF_RUNNING)) {
		case IFF_RUNNING:
			rtw8189f_stop(ifp, 1);
			break;
		case IFF_UP:
			error = rtw8189f_init(ifp);
			break;
		default:
			break;
		}
		break;

	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if ((error = ether_ioctl(ifp, cmd, data)) == ENETRESET)
			error = 0;
		break;

	default:
		error = ieee80211_ioctl(ic, cmd, data);
		break;
	}

	splx(s);
	return error;
}

/* ------------------------------------------------------------------ */
/* net80211 state machine                                              */
/* ------------------------------------------------------------------ */

static int
rtw8189f_newstate(struct ieee80211com *ic, enum ieee80211_state nstate,
    int arg)
{
	struct rtw8189f_softc *sc = ic->ic_ifp->if_softc;

	/* M3 milestone: program channel/filters per state. */
	return sc->sc_newstate(ic, nstate, arg);
}
