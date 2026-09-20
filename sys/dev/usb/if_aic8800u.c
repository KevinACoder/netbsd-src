/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by the RK3568 lab (NetBSD/AIC8800D80 bring-up).
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
 * AICSemi AIC8800D80 (aic8800u) -- net80211 front end.
 *
 * The device claims itself as a fake USB stick first; umodeswitch(4)
 * flips it to the AIC boot ROM, this driver downloads the firmware
 * there, and the re-enumerated app personality carries the data plane.
 * The chip logic is a clean-room implementation of the vendor SDK's
 * lmac_msg protocol (os/aic8800, GPL); aic8800_chip.c holds the
 * download state machine and the runtime command wrappers, and
 * aic8800_usb.c the usbdi(9) transport.
 *
 * The firmware is full-mac: it scans all channels itself (SCANU task),
 * runs the association state machine (SM_CONNECT) and does the CCMP
 * encryption with keys pushed via MM_KEY_ADD.  net80211 therefore never
 * sees management frames on its own MLME: its AUTH/ASSOC frames are
 * dropped on ic_mgtq, scan results are fed from SCANU_RESULT_IND via
 * ieee80211_add_scan() (the bwfm(4) model), and the state machine jumps
 * S_AUTH -> SM_CONNECT -> S_RUN on firmware indications.  Keys are
 * picked out of the net80211 software-crypto tables in the
 * cs_key_update_end callback -- the ioctl layer installs them there
 * before any driver hook exists, and this firmware cannot do the data
 * plane without them (it owns the 802.11 data headers).
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/kthread.h>
#include <sys/mbuf.h>
#include <sys/callout.h>
#include <sys/condvar.h>
#include <sys/device.h>
#include <sys/module.h>
#include <sys/pmf.h>
#include <sys/proc.h>		/* kpause */
#include <sys/cprng.h>
#include <sys/socket.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_proto.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>

#include "usbdevs.h"

#include "aic8800var.h"

static const struct usb_devno aic8800u_devs[] = {
	{ USB_VENDOR_AICSEMI, USB_PRODUCT_AICSEMI_AIC8800D80_BROM },
	{ USB_VENDOR_AICSEMI, USB_PRODUCT_AICSEMI_AIC8800D80 },
};

static int	aic8800u_match(device_t, cfdata_t, void *);
static void	aic8800u_attach(device_t, device_t, void *);
static int	aic8800u_detach(device_t, int);
static int	aic8800u_activate(device_t, enum devact);

CFATTACH_DECL_NEW(aic8800u, sizeof(struct aic8800u_softc), aic8800u_match,
    aic8800u_attach, aic8800u_detach, aic8800u_activate);

static void	aic8800u_dump_layout(struct aic8800u_softc *);
static struct usbd_interface *aic8800u_find_wifi_iface(struct aic8800u_softc *);
static void	aic8800u_bringup_task(void *);
static int	aic8800u_app_attach(struct aic8800u_softc *);

static int	aic8800u_init(struct ifnet *);
static void	aic8800u_stop(struct ifnet *, int);
static void	aic8800u_start(struct ifnet *);
static void	aic8800u_watchdog(struct ifnet *);
static int	aic8800u_ioctl(struct ifnet *, u_long, void *);
static int	aic8800u_newstate(struct ieee80211com *,
		    enum ieee80211_state, int);
static void	aic8800u_newstate_cb(struct aic8800u_softc *,
		    enum ieee80211_state, int);
static void	aic8800u_scan_timo(void *);
static void	aic8800u_worker(void *);
static void	aic8800u_worker_stop(struct aic8800u_softc *);
static void	aic8800u_tx_frame(struct aic8800u_softc *, struct mbuf *);
static void	aic8800u_tx_drain(struct aic8800u_softc *,
		    struct aic8800u_txq *);
static void	aic8800u_handle_events(struct aic8800u_softc *);
static void	aic8800u_scan_result(struct aic8800u_softc *,
		    struct aic8800u_event *);
static void	aic8800u_scan_done(struct aic8800u_softc *);
static void	aic8800u_connect_ind(struct aic8800u_softc *,
		    struct aic8800u_event *);
static void	aic8800u_disconnect_ind(struct aic8800u_softc *);
static void	aic8800u_txcfm(struct aic8800u_softc *,
		    struct aic8800u_event *);
static void	aic8800u_key_update_end(struct ieee80211com *);

static int
aic8800u_match(device_t parent, cfdata_t match, void *aux)
{
	struct usb_attach_arg *uaa = aux;

	if (usb_lookup(aic8800u_devs, uaa->uaa_vendor, uaa->uaa_product) != NULL)
		return UMATCH_VENDOR_PRODUCT;

	return UMATCH_NONE;
}

/*
 * The app personality is a composite: interface 1.0 is the WiFi function
 * (vendor class ff/ff/ff), 1.1/1.2 are Bluetooth.  The boot ROM exposes
 * a single vendor-class interface.  Pick the vendor-class interface that
 * carries bulk endpoints, exactly like the vendor driver's
 * aicwf_parse_usb() does.
 */
static struct usbd_interface *
aic8800u_find_wifi_iface(struct aic8800u_softc *sc)
{
	usb_config_descriptor_t *cdesc;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	struct usbd_interface *iface;
	int i, j, error, nin, nout;

	cdesc = usbd_get_config_descriptor(sc->sc_udev);
	if (cdesc == NULL)
		return NULL;

	for (i = 0; i < cdesc->bNumInterface; i++) {
		error = usbd_device2interface_handle(sc->sc_udev, i, &iface);
		if (error != 0)
			continue;
		id = usbd_get_interface_descriptor(iface);
		if (id == NULL || id->bInterfaceClass != 0xff ||
		    id->bInterfaceSubClass != 0xff ||
		    id->bInterfaceProtocol != 0xff)
			continue;

		nin = nout = 0;
		for (j = 0; j < id->bNumEndpoints; j++) {
			ed = usbd_interface2endpoint_descriptor(iface, j);
			if (ed == NULL)
				continue;
			if (UE_GET_XFERTYPE(ed->bmAttributes) != UE_BULK)
				continue;
			if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN)
				nin++;
			else
				nout++;
		}
		if (nin < 1 || nout < 1)
			continue;

		aprint_normal_dev(sc->sc_dev,
		    "using interface %d (class %#x/%#x/%#x, %d endpoints)\n",
		    id->bInterfaceNumber, id->bInterfaceClass,
		    id->bInterfaceSubClass, id->bInterfaceProtocol,
		    id->bNumEndpoints);
		return iface;
	}

	return NULL;
}

/*
 * Record the full interface/endpoint layout of the current personality.
 * The boot-ROM and app descriptors are a gap in the Linux ground truth
 * ([待补] there); this dump is the NetBSD-side record.
 */
static void
aic8800u_dump_layout(struct aic8800u_softc *sc)
{
	usb_config_descriptor_t *cdesc;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	struct usbd_interface *iface;
	int i, j;

	cdesc = usbd_get_config_descriptor(sc->sc_udev);
	if (cdesc == NULL)
		return;

	for (i = 0; i < cdesc->bNumInterface; i++) {
		if (usbd_device2interface_handle(sc->sc_udev, i, &iface) != 0)
			continue;
		id = usbd_get_interface_descriptor(iface);
		if (id == NULL)
			continue;
		aprint_normal_dev(sc->sc_dev,
		    "if %d class %#x/%#x/%#x, %d endpoints\n",
		    id->bInterfaceNumber, id->bInterfaceClass,
		    id->bInterfaceSubClass, id->bInterfaceProtocol,
		    id->bNumEndpoints);
		for (j = 0; j < id->bNumEndpoints; j++) {
			ed = usbd_interface2endpoint_descriptor(iface, j);
			if (ed != NULL)
				aprint_normal_dev(sc->sc_dev,
				    "  ep %#x attr %#x maxpkt %d\n",
				    ed->bEndpointAddress, ed->bmAttributes,
				    UGETW(ed->wMaxPacketSize));
		}
	}
}

/*
 * Thread exit path: clear the lwp pointer so detach can reap the
 * transport (detach owns the cleanup after it observes the thread
 * gone -- no join, but a bounded wait after aborting the pipes).
 */
static void
aic8800u_bringup_done(struct aic8800u_softc *sc)
{

	mutex_enter(&sc->sc_load_mtx);
	sc->sc_bringup_lwp = NULL;
	mutex_exit(&sc->sc_load_mtx);

	kthread_exit(0);
}

static void
aic8800u_bringup_task(void *arg)
{
	struct aic8800u_softc *sc = arg;
	int attempt;

	/*
	 * The loader runs on its own thread: the download waits for CFMs
	 * that only arrive via the USB callbacks, and firmware(9) cannot
	 * read files until the root file system is mounted -- which on
	 * this board happens after USB enumeration (rtw89 lesson).
	 */
	for (attempt = 0; attempt < 300; attempt++) {
		if (sc->sc_dying)
			break;
		if (!sc->sc_transport_ready &&
		    aic8800u_transport_init(sc) != 0)
			goto retry;
		if (aic8800u_firmware_available(sc))
			break;
retry:
		kpause("aicfwup", false, mstohz(100), NULL);
	}
	if (sc->sc_dying)
		aic8800u_bringup_done(sc);
	if (!sc->sc_transport_ready || !aic8800u_firmware_available(sc)) {
		aprint_error_dev(sc->sc_dev,
		    "transport or firmware never became ready\n");
		aic8800u_bringup_done(sc);
	}

	/* let the freshly re-enumerated device settle (KI-036 family) */
	kpause("aicsettle", false, mstohz(200), NULL);

	aic8800u_fw_download(sc);

	aic8800u_bringup_done(sc);
}

/* ------------------------------------------------------------------ */
/* net80211 registration (app personality)                             */
/* ------------------------------------------------------------------ */

static const uint16_t aic8800u_chan_5g_ic[] = {
	36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120,
	124, 128, 132, 136, 140, 149, 153, 157, 161, 165,
};

static int
aic8800u_app_attach(struct aic8800u_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = &sc->sc_if;
	uint32_t rnd;
	unsigned i;
	int error;

	/* No efuse MAC in this flow: the vendor generates one at each
	 * boot (88:00:33:77 + two random bytes). */
	sc->sc_mac_addr[0] = 0x88;
	sc->sc_mac_addr[1] = 0x00;
	sc->sc_mac_addr[2] = 0x33;
	sc->sc_mac_addr[3] = 0x77;
	rnd = cprng_strong32();
	sc->sc_mac_addr[4] = rnd >> 8;
	sc->sc_mac_addr[5] = rnd;

	error = aic8800u_transport_init(sc);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "app transport init failed\n");
		return error;
	}

	/*
	 * The evt/rx threads must exist BEFORE the first firmware
	 * command: in app mode the evt thread owns the message IN pipe
	 * and is what completes commands (the boot ROM read its own
	 * CFMs; the app never answers on an unattended pipe).
	 */
	error = aic8800u_threads_start(sc);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "cannot start transport threads (%d)\n", error);
		usbd_abort_pipe(sc->sc_evt_pipe);
		usbd_abort_pipe(sc->sc_data_in_pipe);
		kpause("aictx", false, mstohz(50), NULL);
		aic8800u_transport_fini(sc);
		return error;
	}

	/* the firmware just booted; give the command engine some slack */
	error = ENODEV;
	for (i = 0; i < 5; i++) {
		error = aic8800u_fw_init(sc);
		if (error == 0)
			break;
		kpause("aicfwi", false, mstohz(100), NULL);
	}
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "fw init failed (%d)\n", error);
		usbd_abort_pipe(sc->sc_evt_pipe);
		usbd_abort_pipe(sc->sc_data_in_pipe);
		kpause("aictx", false, mstohz(50), NULL);
		aic8800u_transport_fini(sc);
		return error;
	}

	ic->ic_ifp = ifp;
	ic->ic_phytype = IEEE80211_T_OFDM;
	ic->ic_opmode = IEEE80211_M_STA;
	ic->ic_state = IEEE80211_S_INIT;
	ic->ic_caps =
	    IEEE80211_C_MONITOR |
	    IEEE80211_C_SHPREAMBLE |
	    IEEE80211_C_SHSLOT |
	    IEEE80211_C_WPA;		/* 802.11i (firmware CCMP) */

	/* pre-HT: net80211 here has no HT/VHT; the firmware is pinned to
	 * legacy 20MHz in aic8800u_fw_init(). */
	ic->ic_sup_rates[IEEE80211_MODE_11A] = ieee80211_std_rateset_11a;
	ic->ic_sup_rates[IEEE80211_MODE_11B] = ieee80211_std_rateset_11b;
	ic->ic_sup_rates[IEEE80211_MODE_11G] = ieee80211_std_rateset_11g;

	for (i = 1; i <= 13; i++) {
		ic->ic_channels[i].ic_freq =
		    ieee80211_ieee2mhz(i, IEEE80211_CHAN_2GHZ);
		ic->ic_channels[i].ic_flags =
		    IEEE80211_CHAN_CCK | IEEE80211_CHAN_OFDM |
		    IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ;
	}
	for (i = 0; i < __arraycount(aic8800u_chan_5g_ic); i++) {
		unsigned ch = aic8800u_chan_5g_ic[i];

		ic->ic_channels[ch].ic_freq =
		    ieee80211_ieee2mhz(ch, IEEE80211_CHAN_5GHZ);
		ic->ic_channels[ch].ic_flags =
		    IEEE80211_CHAN_OFDM | IEEE80211_CHAN_5GHZ;
	}

	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_init = aic8800u_init;
	ifp->if_ioctl = aic8800u_ioctl;
	ifp->if_start = aic8800u_start;
	ifp->if_watchdog = aic8800u_watchdog;
	IFQ_SET_READY(&ifp->if_snd);
	memcpy(ifp->if_xname, device_xname(sc->sc_dev), IFNAMSIZ);

	if_initialize(ifp);
	ieee80211_ifattach(ic);

	sc->sc_newstate = ic->ic_newstate;
	ic->ic_newstate = aic8800u_newstate;
	ic->ic_crypto.cs_key_update_end = aic8800u_key_update_end;

	ieee80211_media_init(ic, ieee80211_media_change,
	    ieee80211_media_status);

	ifp->if_percpuq = if_percpuq_create(ifp);
	if_register(ifp);

	if_set_sadl(ifp, sc->sc_mac_addr, IEEE80211_ADDR_LEN, false);
	memcpy(ic->ic_myaddr, sc->sc_mac_addr, IEEE80211_ADDR_LEN);

	ieee80211_announce(ic);

	error = kthread_create(PRI_NONE, 0, NULL, aic8800u_worker, sc,
	    &sc->sc_worker, "%s-worker", device_xname(sc->sc_dev));
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "cannot start the worker (%d)\n", error);
		sc->sc_worker = NULL;
		ieee80211_ifdetach(ic);
		if_detach(ifp);
		aic8800u_transport_fini(sc);
		return error;
	}

	sc->sc_ap_idx = -1;
	sc->sc_app_started = true;

	aprint_normal_dev(sc->sc_dev, "aic8800u ready (fw vif %u)\n",
	    sc->sc_vif_idx);
	return 0;
}

static void
aic8800u_attach(device_t parent, device_t self, void *aux)
{
	struct aic8800u_softc *sc = device_private(self);
	struct usb_attach_arg *uaa = aux;
	char *devinfop;
	int error;

	sc->sc_dev = self;
	sc->sc_udev = uaa->uaa_device;
	mutex_init(&sc->sc_load_mtx, MUTEX_DEFAULT, IPL_NONE);
	mutex_init(&sc->sc_cmd_mtx, MUTEX_DEFAULT, IPL_NONE);
	mutex_init(&sc->sc_work_mtx, MUTEX_DEFAULT, IPL_NET);
	mutex_init(&sc->sc_evtq_mtx, MUTEX_DEFAULT, IPL_NONE);
	cv_init(&sc->sc_cmd_cv, device_xname(self));
	cv_init(&sc->sc_cv, device_xname(self));
	MBUFQ_INIT(&sc->sc_txq);
	TAILQ_INIT(&sc->sc_evtq);
	callout_init(&sc->sc_scan_to, 0);
	callout_setfunc(&sc->sc_scan_to, aic8800u_scan_timo, sc);
	sc->sc_ap_idx = -1;

	aprint_naive(": AICSemi AIC8800D80\n");
	aprint_normal("\n");

	devinfop = usbd_devinfo_alloc(sc->sc_udev, 0);
	aprint_normal_dev(self, "%s\n", devinfop);
	usbd_devinfo_free(devinfop);

	error = usbd_set_config_no(sc->sc_udev, 1, 0);
	if (error != 0) {
		aprint_error_dev(self, "failed to set configuration, err=%s\n",
		    usbd_errstr(error));
		return;
	}

	sc->sc_personality =
	    uaa->uaa_product == USB_PRODUCT_AICSEMI_AIC8800D80_BROM ?
	    AIC8800U_BROM : AIC8800U_APP;
	aprint_normal_dev(self, "%s personality (0x%04x:0x%04x)\n",
	    aic8800u_personality_name(sc->sc_personality),
	    uaa->uaa_vendor, uaa->uaa_product);

	aic8800u_dump_layout(sc);

	sc->sc_iface = aic8800u_find_wifi_iface(sc);
	if (sc->sc_iface == NULL) {
		aprint_error_dev(self, "no WiFi interface found\n");
		return;
	}

	if (aic8800u_parse_endpoints(sc, sc->sc_iface) != 0) {
		aprint_error_dev(self, "bulk endpoint layout not usable\n");
		return;
	}

	if (sc->sc_personality == AIC8800U_BROM) {
		/*
		 * Download the firmware.  The device disappears and
		 * re-enumerates as the app personality once the
		 * download finishes; this attachment goes away with it.
		 */
		error = kthread_create(PRI_NONE, 0, NULL, aic8800u_bringup_task,
		    sc, &sc->sc_bringup_lwp, "aic8800fw");
		if (error != 0) {
			aprint_error_dev(self,
			    "cannot start the bring-up thread (%d)\n", error);
			return;
		}
	} else {
		/* App personality: the firmware is running.  Wire the
		 * full net80211 attachment here. */
		(void)aic8800u_app_attach(sc);
	}

	pmf_device_register(self, NULL, NULL);
	usbd_add_drv_event(USB_EVENT_DRIVER_ATTACH, sc->sc_udev, sc->sc_dev);
}

static int
aic8800u_activate(device_t self, enum devact act)
{
	struct aic8800u_softc *sc = device_private(self);

	switch (act) {
	case DVACT_DEACTIVATE:
		sc->sc_dying = true;
		break;
	}

	return 0;
}

static int
aic8800u_detach(device_t self, int flags)
{
	struct aic8800u_softc *sc = device_private(self);
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = &sc->sc_if;
	struct mbuf *m;
	struct ieee80211_node *ni;
	struct aic8800u_event *ev;
	bool running;
	unsigned i;
	int s;

	pmf_device_deregister(self);

	s = splnet();
	sc->sc_dying = true;
	splx(s);

	if (sc->sc_app_started) {
		aic8800u_worker_stop(sc);
		s = splnet();
		callout_halt(&sc->sc_scan_to, NULL);
		ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
		ieee80211_ifdetach(ic);
		if_detach(ifp);
		splx(s);
		sc->sc_app_started = false;
	}

	if (sc->sc_transport_ready) {
		/* wake the evt/rx threads (and a BROM bring-up thread)
		 * out of any sync transfer */
		usbd_abort_pipe(sc->sc_evt_pipe);
		usbd_abort_pipe(sc->sc_cmd_pipe);
		if (sc->sc_data_ready) {
			usbd_abort_pipe(sc->sc_data_in_pipe);
			usbd_abort_pipe(sc->sc_data_out_pipe);
		}
	}

	running = false;
	for (i = 0; i < 500; i++) {
		mutex_enter(&sc->sc_load_mtx);
		running = sc->sc_bringup_lwp != NULL ||
		    sc->sc_evt_lwp != NULL || sc->sc_rx_lwp != NULL;
		mutex_exit(&sc->sc_load_mtx);
		if (!running)
			break;
		kpause("aicdet", false, mstohz(10), NULL);
	}
	if (running)
		aprint_error_dev(self,
		    "transport threads did not exit; leaking transport\n");
	else
		aic8800u_transport_fini(sc);

	/* flush whatever the worker and queues still hold */
	mutex_enter(&sc->sc_work_mtx);
	for (;;) {
		MBUFQ_DEQUEUE(&sc->sc_txq, m);
		if (m == NULL)
			break;
		ni = M_GETCTX(m, struct ieee80211_node *);
		if (ni != NULL)
			ieee80211_free_node(ni);
		m_freem(m);
	}
	mutex_exit(&sc->sc_work_mtx);
	for (i = 0; i < AIC8800U_TXCFM_SLOTS; i++) {
		if (sc->sc_txcfm_m[i] != NULL) {
			ni = M_GETCTX(sc->sc_txcfm_m[i],
			    struct ieee80211_node *);
			if (ni != NULL)
				ieee80211_free_node(ni);
			m_freem(sc->sc_txcfm_m[i]);
			sc->sc_txcfm_m[i] = NULL;
		}
	}
	mutex_enter(&sc->sc_evtq_mtx);
	while ((ev = TAILQ_FIRST(&sc->sc_evtq)) != NULL) {
		TAILQ_REMOVE(&sc->sc_evtq, ev, ev_next);
		sc->sc_evtq_count--;
		kmem_free(ev, sizeof(*ev) + ev->ev_len);
	}
	mutex_exit(&sc->sc_evtq_mtx);

	callout_destroy(&sc->sc_scan_to);
	mutex_destroy(&sc->sc_evtq_mtx);
	mutex_destroy(&sc->sc_work_mtx);
	mutex_destroy(&sc->sc_cmd_mtx);
	cv_destroy(&sc->sc_cv);
	cv_destroy(&sc->sc_cmd_cv);
	mutex_destroy(&sc->sc_load_mtx);

	return 0;
}

/* ------------------------------------------------------------------ */
/* ifnet entry points                                                  */
/* ------------------------------------------------------------------ */

static int
aic8800u_init(struct ifnet *ifp)
{
	struct aic8800u_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;
	int s;

	if (sc->sc_dying)
		return ENXIO;
	if (!sc->sc_app_started)
		return EOPNOTSUPP;

	s = splnet();
	ifp->if_flags |= IFF_RUNNING;
	ifp->if_flags &= ~IFF_OACTIVE;
	/* 1 Hz tick: ieee80211_watchdog drives the AUTH/ASSOC mgt timers;
	 * without it the state machine never times out nor retries. */
	ifp->if_timer = 1;
	ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
	splx(s);

	return 0;
}

static void
aic8800u_stop(struct ifnet *ifp, int disable)
{
	struct aic8800u_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;
	int s;

	s = splnet();
	ifp->if_timer = 0;
	ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
	callout_stop(&sc->sc_scan_to);
	if (ic->ic_state != IEEE80211_S_INIT)
		ieee80211_new_state(ic, IEEE80211_S_INIT, -1);
	splx(s);
}

static void
aic8800u_watchdog(struct ifnet *ifp)
{
	struct aic8800u_softc *sc = ifp->if_softc;

	/* Keep ticking while up; stop() clears if_timer. */
	ifp->if_timer = 1;
	ieee80211_watchdog(&sc->sc_ic);
}

static void
aic8800u_start(struct ifnet *ifp)
{
	struct aic8800u_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct mbuf *m;
	bool kick = false;
	int s;

	if (!sc->sc_app_started || (ifp->if_flags & IFF_RUNNING) == 0) {
		if_statinc(ifp, if_oerrors);
		return;
	}

	s = splnet();
	for (;;) {
		struct ether_header *eh;
		struct ieee80211_node *ni;

		/*
		 * Firmware-SME mode: the AUTH/ASSOC frames net80211 puts
		 * on ic_mgtq are dead weight -- the firmware performs the
		 * whole association itself.  Drop them (counted).
		 */
		IF_POLL(&ic->ic_mgtq, m);
		if (m != NULL) {
			struct ieee80211_node *mni;

			IF_DEQUEUE(&ic->ic_mgtq, m);
			mni = M_GETCTX(m, struct ieee80211_node *);
			if (mni != NULL)
				ieee80211_free_node(mni);
			m_freem(m);
			sc->sc_mgmt_dropped++;
			continue;
		}

		if (ic->ic_state != IEEE80211_S_RUN)
			break;

		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
			break;
		IFQ_DEQUEUE(&ifp->if_snd, m);

		eh = mtod(m, struct ether_header *);
		ni = ieee80211_find_txnode(ic, eh->ether_dhost);
		if (ni == NULL) {
			if_statinc(ifp, if_oerrors);
			m_freem(m);
			continue;
		}

		if ((m = ieee80211_encap(ic, m, ni)) == NULL) {
			if_statinc(ifp, if_oerrors);
			ieee80211_free_node(ni);
			continue;
		}

		/*
		 * No crypto_encap here: the firmware encrypts with the
		 * MM_KEY_ADD keys and owns the on-air 802.11 header.
		 * encap() has set the Protected bit and reserved the
		 * crypto headroom; tx_frame() strips both and hands the
		 * firmware a plaintext Ethernet frame.
		 */
		M_SETCTX(m, ni);
		mutex_enter(&sc->sc_work_mtx);
		MBUFQ_ENQUEUE(&sc->sc_txq, m);
		mutex_exit(&sc->sc_work_mtx);
		kick = true;
	}
	splx(s);

	if (kick) {
		mutex_enter(&sc->sc_work_mtx);
		sc->sc_flags |= AIC8800U_F_TX;
		cv_broadcast(&sc->sc_cv);
		mutex_exit(&sc->sc_work_mtx);
	}
}

static int
aic8800u_ioctl(struct ifnet *ifp, u_long cmd, void *data)
{
	struct aic8800u_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;
	int error = 0;

	/* init/stop perform sleeping bus and worker operations. */
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
			aic8800u_stop(ifp, 1);
			break;
		case IFF_UP:
			error = aic8800u_init(ifp);
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

	return error;
}

/* ------------------------------------------------------------------ */
/* State machine, scanning, firmware SME                               */
/* ------------------------------------------------------------------ */

static int
aic8800u_newstate(struct ieee80211com *ic, enum ieee80211_state nstate,
    int arg)
{
	struct aic8800u_softc *sc = ic->ic_ifp != NULL ?
	    ic->ic_ifp->if_softc : NULL;

	/* Sanity: ic lives inside our softc; anything else means the
	 * caller raced a detach and we must not touch softc fields. */
	if (sc == NULL || &sc->sc_ic != ic)
		return EIO;

	/* Only kill the scan watchdog when leaving scan: the first
	 * INIT->SCAN transition re-enters synchronously before the
	 * firmware scan was even started (rtw8189f lesson). */
	if (nstate != IEEE80211_S_SCAN)
		callout_stop(&sc->sc_scan_to);
	if (sc->sc_worker == NULL || sc->sc_dying || !sc->sc_app_started)
		return sc->sc_newstate(ic, nstate, arg);

	mutex_enter(&sc->sc_work_mtx);
	sc->sc_nstate = nstate;
	sc->sc_narg = arg;
	sc->sc_flags |= AIC8800U_F_NEWSTATE;
	cv_broadcast(&sc->sc_cv);
	mutex_exit(&sc->sc_work_mtx);

	return 0;
}

/* Runs on the worker; ic->ic_state still holds the previous state. */
static void
aic8800u_newstate_cb(struct aic8800u_softc *sc, enum ieee80211_state nstate,
    int arg)
{
	struct ieee80211com *ic = &sc->sc_ic;
	enum ieee80211_state ostate = ic->ic_state;
	const uint8_t *ssid = NULL;
	size_t ssid_len = 0;
	int error;

	switch (nstate) {
	case IEEE80211_S_SCAN:
		if (!sc->sc_scanning) {
			sc->sc_scanning = true;
			mutex_enter(&sc->sc_work_mtx);
			sc->sc_flags &= ~AIC8800U_F_SCANTIMO;
			mutex_exit(&sc->sc_work_mtx);

			if (ic->ic_des_esslen > 0) {
				ssid = ic->ic_des_essid;
				ssid_len = ic->ic_des_esslen;
			}
			error = aic8800u_scan_start(sc, ssid, ssid_len);
			if (error != 0)
				aprint_error_dev(sc->sc_dev,
				    "scanu start failed (%d)\n", error);
			/* the watchdog forces end_scan if the firmware
			 * never reports completion */
			callout_schedule(&sc->sc_scan_to, 8 * hz);
		}
		break;

	case IEEE80211_S_AUTH:
	case IEEE80211_S_ASSOC:
		/*
		 * The firmware does the open-auth + association now.
		 * Completion arrives as SM_CONNECT_IND; a firmware that
		 * never answers is caught by the net80211 mgt timer and
		 * sends us back to S_SCAN.
		 */
		if (ostate != IEEE80211_S_AUTH && ostate != IEEE80211_S_ASSOC)
			(void)aic8800u_connect(sc, ic->ic_bss);
		break;

	case IEEE80211_S_RUN:
		break;

	case IEEE80211_S_INIT:
		sc->sc_scanning = false;
		if (sc->sc_connected) {
			sc->sc_connected = false;
			sc->sc_ap_idx = -1;
			aic8800u_disconnect(sc);
		}
		break;
	}

	sc->sc_newstate(ic, nstate, arg);
}

static void
aic8800u_scan_timo(void *arg)
{
	struct aic8800u_softc *sc = arg;

	mutex_enter(&sc->sc_work_mtx);
	sc->sc_flags |= AIC8800U_F_SCANTIMO;
	cv_broadcast(&sc->sc_cv);
	mutex_exit(&sc->sc_work_mtx);
}

/*
 * Scan finish, driver-driven (the iwm(4) model): nothing in the stack
 * completes a firmware scan for us.
 */
static void
aic8800u_scan_done(struct aic8800u_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	int s;

	if (!sc->sc_scanning)
		return;
	sc->sc_scanning = false;
	callout_stop(&sc->sc_scan_to);

	s = splnet();
	if (ic->ic_state == IEEE80211_S_SCAN)
		ieee80211_end_scan(ic);
	splx(s);
}

/*
 * One SCANU_RESULT_IND: feed the BSS straight into the scan cache
 * (the bwfm(4) model).  The result carries the complete beacon frame;
 * parse its fixed fields and IEs into a scanparams.
 */
static void
aic8800u_scan_result(struct aic8800u_softc *sc, struct aic8800u_event *ev)
{
	struct ieee80211com *ic = &sc->sc_ic;
	const struct aic8800u_scanu_result_ind *ind;
	struct ieee80211_frame wh;
	struct ieee80211_scanparams scan;
	uint8_t *frame, *frm, *efrm, *sfrm;
	uint64_t tsf = 0;
	uint8_t chan;

	if (ev->ev_len < sizeof(*ind))
		return;

	ind = (const struct aic8800u_scanu_result_ind *)ev->ev_data;
	frame = ev->ev_data + sizeof(*ind);
	/* payload is word aligned; the frame length is authoritative */
	if (le16toh(ind->length) < 36 ||
	    frame + le16toh(ind->length) > ev->ev_data + ev->ev_len)
		return;
	efrm = frame + le16toh(ind->length);

	/* Fake a wireless header carrying the BSSID (addr2/addr3). */
	memset(&wh, 0, sizeof(wh));
	wh.i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT |
	    IEEE80211_FC0_SUBTYPE_BEACON;
	IEEE80211_ADDR_COPY(wh.i_addr2, &frame[10]);
	IEEE80211_ADDR_COPY(wh.i_addr3, &frame[16]);

	memset(&scan, 0, sizeof(scan));
	scan.sp_tstamp = (uint8_t *)&tsf;
	scan.sp_bintval = le16dec(&frame[32]);
	scan.sp_capinfo = le16dec(&frame[34]);

	chan = ieee80211_mhz2ieee(le16toh(ind->center_freq),
	    le16toh(ind->center_freq) > 4000 ? IEEE80211_CHAN_5GHZ : 0);

	sfrm = frame + 36;		/* past tstamp/bintval/capinfo */
	for (frm = sfrm; frm + 1 < efrm; frm += 2 + frm[1]) {
		if (frm + 2 + frm[1] > efrm)
			break;
		switch (frm[0]) {
		case IEEE80211_ELEMID_SSID:
			scan.sp_ssid = frm;
			break;
		case IEEE80211_ELEMID_RATES:
			scan.sp_rates = frm;
			break;
		case IEEE80211_ELEMID_DSPARMS:
			if (frm[1] == 1)
				chan = frm[2];
			break;
		case IEEE80211_ELEMID_TIM:
			scan.sp_tim = frm;
			scan.sp_timoff = frm - sfrm;
			break;
		case IEEE80211_ELEMID_XRATES:
			scan.sp_xrates = frm;
			break;
		case IEEE80211_ELEMID_ERP:
			if (frm[1] == 1)
				scan.sp_erp = frm[2];
			break;
		case IEEE80211_ELEMID_RSN:
			scan.sp_wpa = frm;
			break;
		case IEEE80211_ELEMID_COUNTRY:
			scan.sp_country = frm;
			break;
		case IEEE80211_ELEMID_VENDOR:
			if (frm[1] > 5 && frm[2] == 0x00 && frm[3] == 0x50 &&
			    frm[4] == 0xf2 && frm[5] == 2)
				scan.sp_wme = frm;
			break;
		}
	}

	scan.sp_chan = scan.sp_bchan = chan;

	if ((ic->ic_flags & IEEE80211_F_SCAN) != 0)
		ieee80211_add_scan(ic, &scan, &wh,
		    IEEE80211_FC0_SUBTYPE_BEACON,
		    ind->rssi < 0 ? -ind->rssi : ind->rssi, 0);
}

/* SM_CONNECT_IND: the firmware finished the association. */
static void
aic8800u_connect_ind(struct aic8800u_softc *sc, struct aic8800u_event *ev)
{
	struct ieee80211com *ic = &sc->sc_ic;
	const struct aic8800u_sm_connect_ind *ind;
	uint16_t status_code, aid;
	uint8_t ap_idx;
	int s;

	if (ev->ev_len < offsetof(struct aic8800u_sm_connect_ind, aid) + 2)
		return;

	ind = (const struct aic8800u_sm_connect_ind *)ev->ev_data;
	status_code = le16toh(ind->status_code);
	ap_idx = ind->ap_idx;
	aid = le16toh(ind->aid) & 0x3fff;

	s = splnet();
	if (status_code == 0) {
		sc->sc_connected = true;
		sc->sc_ap_idx = ap_idx;
		sc->sc_aid = aid;
		if (ic->ic_bss != NULL)
			ic->ic_bss->ni_associd = sc->sc_aid;
		aprint_normal_dev(sc->sc_dev,
		    "connected: ap sta %u, aid %#x\n", sc->sc_ap_idx,
		    sc->sc_aid);
		ieee80211_new_state(ic, IEEE80211_S_RUN, -1);
	} else {
		aprint_normal_dev(sc->sc_dev,
		    "connect failed: status %u\n", status_code);
		ieee80211_new_state(ic, IEEE80211_S_SCAN, 0);
	}
	splx(s);
}

static void
aic8800u_disconnect_ind(struct aic8800u_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	int s;

	sc->sc_connected = false;
	sc->sc_ap_idx = -1;

	s = splnet();
	if (ic->ic_state == IEEE80211_S_RUN || ic->ic_state == IEEE80211_S_AUTH ||
	    ic->ic_state == IEEE80211_S_ASSOC)
		ieee80211_new_state(ic, IEEE80211_S_SCAN, 0);
	splx(s);
}

/* TX confirmation for need_cfm frames: {u32 status, u32 used_idx}. */
static void
aic8800u_txcfm(struct aic8800u_softc *sc, struct aic8800u_event *ev)
{
	struct mbuf *m;
	struct ieee80211_node *ni;
	uint32_t status, used;
	unsigned slot;

	if (ev->ev_len < 8)
		return;
	status = le32dec(&ev->ev_data[0]);
	used = le32dec(&ev->ev_data[4]);

	slot = used % AIC8800U_TXCFM_SLOTS;
	m = sc->sc_txcfm_m[slot];
	sc->sc_txcfm_m[slot] = NULL;
	if (m == NULL) {
		sc->sc_txcfm_lost++;
		return;
	}
	ni = M_GETCTX(m, struct ieee80211_node *);
	if (ni != NULL)
		ieee80211_free_node(ni);
	m_freem(m);

	/* union rwnx_hw_txstatus: bit0 tx_done, bit3 acknowledged */
	if (status & (1u << 3))
		sc->sc_txcfm_acked++;
	else
		sc->sc_txcfm_retried++;
}

static void
aic8800u_handle_events(struct aic8800u_softc *sc)
{
	struct aic8800u_event *ev;

	while (aic8800u_evt_dequeue(sc, &ev) != 0) {
		switch (ev->ev_id) {
		case AIC8800_SCANU_RESULT_IND:
			aic8800u_scan_result(sc, ev);
			break;
		case AIC8800_SCANU_START_CFM:
			aic8800u_scan_done(sc);
			break;
		case AIC8800_SM_CONNECT_IND:
			aic8800u_connect_ind(sc, ev);
			break;
		case AIC8800_SM_DISCONNECT_IND:
			aic8800u_disconnect_ind(sc);
			break;
		case AIC8800U_EVT_TXCFM:
			aic8800u_txcfm(sc, ev);
			break;
		default:
			break;	/* informational indication */
		}
		aic8800u_evt_release(sc, ev);
	}
}

/*
 * Keys: net80211 installs PTK/GTK into its software-crypto tables with
 * no driver hook -- except the cs_key_update_* bracket around every key
 * ioctl.  At update end, mirror every installed key into the firmware
 * (it owns the on-air CCMP) and open the control port.
 */
static void
aic8800u_key_update_end(struct ieee80211com *ic)
{
	struct aic8800u_softc *sc = ic->ic_ifp->if_softc;
	struct ieee80211_key *wk;
	unsigned kid;
	int error;

	if (!sc->sc_connected || sc->sc_ap_idx < 0)
		return;

	wk = &ic->ic_bss->ni_ucastkey;
	if (wk->wk_cipher != NULL && wk->wk_keylen > 0) {
		error = aic8800u_key_add(sc, wk->wk_key, wk->wk_keylen, 0,
		    true);
		if (error == 0)
			aprint_normal_dev(sc->sc_dev,
			    "PTK installed (%zu bytes)\n",
			    (size_t)wk->wk_keylen);
		aic8800u_control_port(sc, true);
	}

	for (kid = 0; kid < IEEE80211_WEP_NKID; kid++) {
		wk = &ic->ic_nw_keys[kid];
		if (wk->wk_cipher != NULL && wk->wk_keylen > 0)
			(void)aic8800u_key_add(sc, wk->wk_key, wk->wk_keylen,
			    kid, false);
	}
}

/* ------------------------------------------------------------------ */
/* TX                                                                  */
/* ------------------------------------------------------------------ */

/*
 * Ship one data frame: the mbuf carries the ieee80211_encap() output
 * ([802.11 hdr][reserved crypto hdr][LLC/SNAP][payload]); rebuild it as
 * the firmware's data-TX frame (4-byte header + hostdesc + Ethernet
 * frame) and push it out the data OUT pipe.  EAPOL frames request a
 * firmware confirmation so AP-ACK vs silent rejection is visible in the
 * counters (the rtw8189f SPE_RPT lesson).
 */
static void
aic8800u_tx_frame(struct aic8800u_softc *sc, struct mbuf *m)
{
	struct ifnet *ifp = &sc->sc_if;
	struct ieee80211_node *ni;
	struct ieee80211_frame *wh;
	struct aic8800u_hostdesc *desc;
	uint8_t *buf = sc->sc_tx_buf;
	uint8_t *mp;
	size_t hdr_len, body_off, plen, len;
	uint16_t ethertype;
	bool cfm = false;
	unsigned slot = 0;
	int error;

	ni = M_GETCTX(m, struct ieee80211_node *);

	wh = mtod(m, struct ieee80211_frame *);
	mp = mtod(m, uint8_t *);
	hdr_len = sizeof(*wh);
	if ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS) ==
	    IEEE80211_FC0_SUBTYPE_QOS)
		hdr_len += 2;
	body_off = hdr_len + ((wh->i_fc[1] & IEEE80211_FC1_WEP) != 0 ? 8 : 0);

	if (m->m_pkthdr.len < (int)(body_off + 8) ||
	    m->m_pkthdr.len > (int)(body_off + 8 + MCLBYTES)) {
		if_statinc(ifp, if_oerrors);
		ieee80211_free_node(ni);
		m_freem(m);
		return;
	}

	ethertype = (mp[body_off + 6] << 8) | mp[body_off + 7];
	plen = m->m_pkthdr.len - body_off - 8;	/* Ethernet payload bytes */

	/* total wire length includes the 4-byte header */
	len = 4 + sizeof(*desc) + 14 + plen;
	memset(buf, 0, len + 8);
	buf[0] = len & 0xff;
	buf[1] = (len >> 8) & 0x0f;
	buf[2] = 0x01;			/* data */
	buf[3] = 0x00;

	desc = (struct aic8800u_hostdesc *)&buf[4];
	desc->packet_len = htole16(14 + plen);
	memcpy(desc->eth_dest_addr, wh->i_addr3, 6);	/* RA = AP */
	memcpy(desc->eth_src_addr, wh->i_addr2, 6);	/* TA = us */
	desc->ethertype = htole16(ethertype);
	desc->ac = 1;			/* BE */
	desc->tid = 0xff;
	desc->vif_idx = sc->sc_vif_idx;
	desc->staid = sc->sc_connected ? sc->sc_ap_idx : 0xff;
	desc->flags = 0;

	if (ethertype == 0x888e && sc->sc_connected) {
		slot = sc->sc_txcfm_free % AIC8800U_TXCFM_SLOTS;
		if (sc->sc_txcfm_m[slot] == NULL) {
			cfm = true;
			desc->status_desc_addr =
			    htole32(0x80000000u | slot);
		}
	}

	/* Ethernet header: dst = addr3 (DA), src = addr2 (TA) */
	memcpy(&buf[4 + sizeof(*desc)], wh->i_addr3, 6);
	memcpy(&buf[4 + sizeof(*desc) + 6], wh->i_addr2, 6);
	buf[4 + sizeof(*desc) + 12] = ethertype >> 8;
	buf[4 + sizeof(*desc) + 13] = ethertype & 0xff;
	m_copydata(m, body_off + 8, plen,
	    &buf[4 + sizeof(*desc) + 14]);

	/* pad to 4; a multiple of 512 needs one extra byte (short packet) */
	len = (len + 3) & ~3u;
	if ((len % 512) == 0)
		len++;

	error = aic8800u_data_write(sc, len);
	if (error != 0) {
		if_statinc(ifp, if_oerrors);
		ieee80211_free_node(ni);
		m_freem(m);
		return;
	}

	if (cfm) {
		sc->sc_txcfm_m[slot] = m;
		sc->sc_txcfm_free++;
		/* node ref rides the mbuf to the confirmation */
	} else {
		ieee80211_free_node(ni);
		m_freem(m);
	}
}

static void
aic8800u_tx_drain(struct aic8800u_softc *sc, struct aic8800u_txq *locq)
{
	struct mbuf *m;

	for (;;) {
		MBUFQ_DEQUEUE(locq, m);
		if (m == NULL)
			break;
		aic8800u_tx_frame(sc, m);
	}
}

/* ------------------------------------------------------------------ */
/* worker                                                              */
/* ------------------------------------------------------------------ */

static void
aic8800u_worker_stop(struct aic8800u_softc *sc)
{
	int wait;

	mutex_enter(&sc->sc_work_mtx);
	sc->sc_flags |= AIC8800U_F_EXIT;
	cv_broadcast(&sc->sc_cv);
	mutex_exit(&sc->sc_work_mtx);

	/* Bounded wait: the worker clears sc_worker itself on exit. */
	for (wait = 0; sc->sc_worker != NULL && wait < 500; wait++)
		kpause("aicwrk", false, mstohz(10), NULL);
}

static void
aic8800u_worker(void *arg)
{
	struct aic8800u_softc *sc = arg;
	struct aic8800u_txq locq;
	struct mbuf *m;
	struct ieee80211_node *ni;
	uint32_t flags;
	enum ieee80211_state nstate;
	int narg;

	MBUFQ_INIT(&locq);

	while (!sc->sc_dying) {
		mutex_enter(&sc->sc_work_mtx);
		while (!(sc->sc_flags & (AIC8800U_F_NEWSTATE |
		    AIC8800U_F_TX | AIC8800U_F_EVENT | AIC8800U_F_SCANTIMO |
		    AIC8800U_F_EXIT)) && !sc->sc_dying) {
			if (cv_timedwait(&sc->sc_cv, &sc->sc_work_mtx,
			    mstohz(100)) == EWOULDBLOCK)
				break;
		}
		flags = sc->sc_flags;
		nstate = sc->sc_nstate;
		narg = sc->sc_narg;
		sc->sc_flags = 0;
		/* Steal the TX queue without holding the mutex on USB. */
		for (;;) {
			MBUFQ_DEQUEUE(&sc->sc_txq, m);
			if (m == NULL)
				break;
			MBUFQ_ENQUEUE(&locq, m);
		}
		mutex_exit(&sc->sc_work_mtx);

		if ((flags & AIC8800U_F_EXIT) || sc->sc_dying)
			break;

		if ((flags & AIC8800U_F_SCANTIMO) &&
		    !(flags & AIC8800U_F_NEWSTATE))
			aic8800u_scan_done(sc);

		if (flags & AIC8800U_F_EVENT)
			aic8800u_handle_events(sc);

		if (flags & AIC8800U_F_NEWSTATE)
			aic8800u_newstate_cb(sc, nstate, narg);

		aic8800u_tx_drain(sc, &locq);
	}

	/* Flush anything still queued. */
	for (;;) {
		MBUFQ_DEQUEUE(&locq, m);
		if (m == NULL)
			break;
		ni = M_GETCTX(m, struct ieee80211_node *);
		if (ni != NULL)
			ieee80211_free_node(ni);
		m_freem(m);
	}
	mutex_enter(&sc->sc_work_mtx);
	for (;;) {
		MBUFQ_DEQUEUE(&sc->sc_txq, m);
		if (m == NULL)
			break;
		ni = M_GETCTX(m, struct ieee80211_node *);
		if (ni != NULL)
			ieee80211_free_node(ni);
		m_freem(m);
	}
	sc->sc_worker = NULL;
	cv_broadcast(&sc->sc_cv);
	mutex_exit(&sc->sc_work_mtx);

	/* rtw89 lane lesson: a kthread must kthread_exit(), never return. */
	kthread_exit(0);
}
