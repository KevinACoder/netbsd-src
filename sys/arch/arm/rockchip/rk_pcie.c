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
 * RK3568 DesignWare PCIe root complex ("rockchip,rk3568-pcie"), serving
 *   - pcie2x1 @ 0xfe260000: M.2 22110 slot (NVMe), PHY = combphy2
 *   - pcie3x2 @ 0xfe280000: PCIe x4 slot (Intel 7260), PHY = pcie30phy
 *
 * The bus framework (bus enumeration, ranges, INTx routing, MSI
 * dispatch) is the standard arm/pcihost FDT framework, driven the same
 * way rk3399_pcie.c drives it.  The controller bring-up sequence and
 * the DWC register constants mirror the vendor bare-metal SDK fdwpcie
 * driver (os/standalone drivers/pcie/fdwpcie), which was validated on
 * this board (KI-008/009), cross-checked against Linux
 * pcie-dw-rockchip.c and the FreeBSD fork rk3568_pcie.c:
 *
 *   - CRU controller clock gates (clkgate_con10/12/13) are opened
 *     explicitly; gate polarity 1 = off (fdwpcie FDwPcieCruClkGateEnable)
 *   - CLKREQ#/WAKE# must be muxed to fn4 (GRF+0x38 for pcie2x1,
 *     GRF+0x3c for pcie3x2) or the endpoint never sees a reference
 *     clock and LTSSM sticks in Detect.Active
 *   - PERST# low -> controller soft reset -> PHY init -> reset release
 *     -> (pcie30phy) calibrate -> RC mode -> link training with the
 *     T_PVPERL 200 ms PERST window and the 1 s link-up recheck
 *   - the PIPE_GRF pcie_link_rst_grt bit is deliberately NOT written:
 *     neither U-Boot nor bare-metal writes it, and bare-metal measured
 *     that writing it stalls the pcie3x2 link in Polling (KI-009)
 *   - after link up the endpoint needs a settle window (SM2263EN NVMe
 *     answers config TLPs only after its firmware init; KI-008), so
 *     attach waits 3 s and then polls the downstream vendor ID before
 *     the one-shot bus scan (pcihost_init2) runs
 *
 * Config space: bus 0 is the root port itself (DBI window), deeper
 * buses go through the outbound iATU CFG0/CFG1 window reprogrammed per
 * access, exactly like fdwpcie.  Windows below 4 GB only: the board dts
 * limits ranges to IO + 32-bit MEM (a MEM64 range breaks endpoint
 * enumeration on this board, see KI-013).
 */

#include <sys/cdefs.h>

__KERNEL_RCSID(0, "$NetBSD");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/mutex.h>

#include <sys/bus.h>
#include <sys/gpio.h>
#include <dev/fdt/fdtvar.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pciconf.h>

#include <arm/fdt/pcihost_fdtvar.h>

/*
 * Physical base addresses inside the rk_platform MMIO devmap window.
 */
#define	RK3568_CRU_BASE			0xfdd20000
#define	RK3568_CRU_SIZE			0x1000
#define	RK3568_GRF_BASE			0xfdc60000
#define	RK3568_GRF_SIZE			0x1000

/* Main CRU: clkgate region at 0x300, softrst region at 0x400. */
#define	CRU_CLKGATE_CON(n)		(0x300 + (n) * 4)
#define	CRU_SOFTRST_CON(n)		(0x400 + (n) * 4)
#define	CLKGATE_CON10_PIPE_MASK		0x3	/* aclk/pclk pipe family */
#define	SRST_PCIE20_POWERUP		161	/* bank 10 bit 1 */
#define	SRST_PCIE30X2_POWERUP		193	/* bank 12 bit 1 */

/* GRF iomux: CLKREQ#/WAKE# fn4 fields (per-pin 4-bit, hiword enable). */
#define	GRF_IOMUX_PCIE20		0x038	/* GPIO2_D0/D1 */
#define	GRF_IOMUX_PCIE30X2		0x03c	/* GPIO2_D4/D5 */

/*
 * Rockchip client APB registers (fdwpcie.h names).
 */
#define	PCIE_CLIENT_GENERAL_CON		0x0000
#define	PCIE_CLIENT_INTR_MASK_LEGACY	0x001c
#define	PCIE_CLIENT_POWER		0x002c
#define	PCIE_CLIENT_GENERAL_DEBUG	0x0104
#define	PCIE_CLIENT_HOT_RESET_CTRL	0x0180
#define	PCIE_CLIENT_LTSSM_STATUS	0x0300
#define	PCIE_CLIENT_DEBUG_FIFO		0x0310

#define	LTSSM_ENABLE			0x000c000c
#define	LTSSM_DISABLE			0x000c0008
#define	GENERAL_CON_RC_MODE		0x000f0040
#define	CLIENT_POWER_RC			0x30011000
#define	LTSSM_ENHANCE			(__BIT(4) | __BIT(20))
#define	LTSSM_STATE_L0			0x11
#define	LTSSM_LINKUP			(__BIT(16) | __BIT(17))

/*
 * DWC DBI registers.
 */
#define	DBI_LINK_CAPABILITY		0x07c	/* PCIE_LCAP */
#define	DBI_LINK_STATUS			0x080	/* PCIE_LSTAT */
#define	DBI_LINK_CTL2			0x0a0	/* PCIE_LCTL2 */
#define	DBI_PORT_LINK_CONTROL		0x710
#define	DBI_LINK_WIDTH_SPEED_CONTROL	0x80c
#define	DBI_MISC_CONTROL_1		0x8bc
#define	DBI2_BASE			0x100000

#define	PORT_LINK_MODE_MASK		__BITS(21,16)
#define	PORT_LINK_MODE(n)		__SHIFTIN((n) - 1, PORT_LINK_MODE_MASK)
#define	PORT_LINK_FAST_LINK_MODE	__BIT(7)
#define	TARGET_LINK_SPEED_MASK		0x0000000f
#define	PORT_LOGIC_LINK_WIDTH_MASK	__BITS(12,8)
#define	PORT_LOGIC_LINK_WIDTH(n)	__SHIFTIN((n), PORT_LOGIC_LINK_WIDTH_MASK)
#define	SPEED_CHANGE			__BIT(17)
#define	DBI_RO_WR_EN			__BIT(0)

/*
 * Outbound iATU (unrolled viewport at dbi + (3 << 20) + (idx << 9)).
 */
#define	ATU_REG_BASE(idx)		(((0x3) << 20) | ((idx) << 9))
#define	ATU_REGION_CTRL1		0x00
#define	ATU_REGION_CTRL2		0x04
#define	ATU_REGION_LOWER_BASE		0x08
#define	ATU_REGION_UPPER_BASE		0x0c
#define	ATU_REGION_LIMIT		0x10
#define	ATU_REGION_LOWER_TARGET		0x14
#define	ATU_REGION_UPPER_TARGET		0x18
#define	ATU_ENABLE			__BIT(31)
#define	ATU_TYPE_MEM			0x0
#define	ATU_TYPE_IO			0x2
#define	ATU_TYPE_CFG0			0x4
#define	ATU_TYPE_CFG1			0x5

#define	ATU_OB_INDEX_MEM		0
#define	ATU_OB_INDEX_IO			1
#define	ATU_OB_INDEX_CFG		2
#define	ATU_OB_REGION_COUNT		6

/* Link training parameters (fdwpcie). */
#define	LINK_WAIT_RETRIES		100
#define	LINK_WAIT_MS			20
#define	LINK_MAX_RETRIES		3
/* Endpoint settle after link up (FreeBSD fork value; fdwpcie uses 10 s). */
#define	EP_SETTLE_MS			3000
#define	EP_POLL_RETRIES			100
#define	EP_POLL_MS			100

struct rk_pcie_softc {
	struct pcihost_softc	sc_phsc;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_apb_bh;
	bus_space_handle_t	sc_dbi_bh;
	bus_space_handle_t	sc_cfg_bh;
	bus_addr_t		sc_cfg_addr;
	bus_size_t		sc_cfg_size;
	kmutex_t		sc_conf_lock;
	struct fdtbus_gpio_pin	*sc_perst;
	struct fdtbus_phy	*sc_phy;
	u_int			sc_num_lanes;
	u_int			sc_max_link_speed;
	u_int			sc_softrst_id;	/* CRU soft reset id */
	u_int			sc_clkgate_con;	/* CRU clkgate con index */
	bool			sc_is_pcie30;	/* pcie30phy consumer */
};

static int	rk_pcie_match(device_t, cfdata_t, void *);
static void	rk_pcie_attach(device_t, device_t, void *);

static int	rk_pcie_bus_maxdevs(void *, int);
static pcitag_t	rk_pcie_make_tag(void *, int, int, int);
static void	rk_pcie_decompose_tag(void *, pcitag_t, int *, int *, int *);
static pcireg_t	rk_pcie_conf_read(void *, pcitag_t, int);
static void	rk_pcie_conf_write(void *, pcitag_t, int, pcireg_t);
static int	rk_pcie_conf_hook(void *, int, int, int, pcireg_t);

static struct fdtbus_interrupt_controller_func rk_pcie_intrfuncs;

CFATTACH_DECL_NEW(rkdwpcie, sizeof(struct rk_pcie_softc),
    rk_pcie_match, rk_pcie_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "rockchip,rk3568-pcie" },
	DEVICE_COMPAT_EOL
};

#define	APB_READ4(sc, reg) \
	bus_space_read_4((sc)->sc_bst, (sc)->sc_apb_bh, (reg))
#define	APB_WRITE4(sc, reg, val) \
	bus_space_write_4((sc)->sc_bst, (sc)->sc_apb_bh, (reg), (val))
#define	DBI_READ4(sc, reg) \
	bus_space_read_4((sc)->sc_bst, (sc)->sc_dbi_bh, (reg))
#define	DBI_WRITE4(sc, reg, val) \
	bus_space_write_4((sc)->sc_bst, (sc)->sc_dbi_bh, (reg), (val))

static int
rk_pcie_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

/* CRU clock gate: 1 = gated, write hiword mask with value 0 to open. */
static void
rk_pcie_clkgate_open(struct rk_pcie_softc *sc, bus_space_handle_t cru,
    u_int con, uint32_t bits)
{
	bus_space_write_4(sc->sc_bst, cru, CRU_CLKGATE_CON(con), bits << 16);
}

static void
rk_pcie_softrst(struct rk_pcie_softc *sc, bus_space_handle_t cru,
    u_int rst_id, bool assert)
{
	const uint32_t mask = __BIT(rst_id % 16);

	bus_space_write_4(sc->sc_bst, cru, CRU_SOFTRST_CON(rst_id / 16),
	    (mask << 16) | (assert ? mask : 0));
}

static bool
rk_pcie_link_up(struct rk_pcie_softc *sc)
{
	uint32_t val = APB_READ4(sc, PCIE_CLIENT_LTSSM_STATUS);

	return ((val & LTSSM_LINKUP) == LTSSM_LINKUP &&
	    (val & 0x3f) == LTSSM_STATE_L0);
}

static void
rk_pcie_perst(struct rk_pcie_softc *sc, bool high)
{
	if (sc->sc_perst == NULL)
		return;
	fdtbus_gpio_write(sc->sc_perst, high);
}

static void
rk_pcie_atu_prog(struct rk_pcie_softc *sc, u_int idx, uint32_t type,
    uint64_t cpu_addr, uint64_t pci_addr, uint64_t size)
{
	const bus_size_t base = ATU_REG_BASE(idx);
	u_int n;

	DBI_WRITE4(sc, base + ATU_REGION_LOWER_BASE, (uint32_t)cpu_addr);
	DBI_WRITE4(sc, base + ATU_REGION_UPPER_BASE, (uint32_t)(cpu_addr >> 32));
	DBI_WRITE4(sc, base + ATU_REGION_LIMIT,
	    (uint32_t)(cpu_addr + size - 1));
	DBI_WRITE4(sc, base + ATU_REGION_LOWER_TARGET, (uint32_t)pci_addr);
	DBI_WRITE4(sc, base + ATU_REGION_UPPER_TARGET,
	    (uint32_t)(pci_addr >> 32));
	DBI_WRITE4(sc, base + ATU_REGION_CTRL1, type);
	DBI_WRITE4(sc, base + ATU_REGION_CTRL2, ATU_ENABLE);

	for (n = 0; n < 5; n++) {
		if ((DBI_READ4(sc, base + ATU_REGION_CTRL2) & ATU_ENABLE) != 0)
			break;
		delay(100);
	}
}

/* Outbound windows: identity mapping for MEM and IO, from the ranges. */
static void
rk_pcie_atu_init(struct rk_pcie_softc *sc)
{
	const u_int *ranges;
	int len;

	ranges = fdtbus_get_prop(sc->sc_phsc.sc_phandle, "ranges", &len);
	if (ranges == NULL)
		return;

	for (int i = 0; i + 7 <= len / 4; i += 7) {
		uint64_t pci_addr, cpu_addr, size;

		pci_addr = ((uint64_t)be32toh(ranges[i + 1]) << 32) |
		    be32toh(ranges[i + 2]);
		cpu_addr = ((uint64_t)be32toh(ranges[i + 3]) << 32) |
		    be32toh(ranges[i + 4]);
		size = ((uint64_t)be32toh(ranges[i + 5]) << 32) |
		    be32toh(ranges[i + 6]);

		switch (be32toh(ranges[i]) & 0x03000000) {
		case 0x01000000:	/* I/O space */
			rk_pcie_atu_prog(sc, ATU_OB_INDEX_IO, ATU_TYPE_IO,
			    cpu_addr, pci_addr, size);
			break;
		case 0x02000000:	/* 32-bit memory */
		case 0x03000000:	/* 64-bit memory */
			rk_pcie_atu_prog(sc, ATU_OB_INDEX_MEM, ATU_TYPE_MEM,
			    cpu_addr, pci_addr, size);
			break;
		}
	}
}

/* Root port (DBI) setup, from fdwpcie FDwPcieSetupHost. */
static void
rk_pcie_setup_host(struct rk_pcie_softc *sc)
{
	uint32_t val;

	DBI_WRITE4(sc, DBI_MISC_CONTROL_1,
	    DBI_READ4(sc, DBI_MISC_CONTROL_1) | DBI_RO_WR_EN);

	/* Root port BAR0 = unused 64-bit MEM window, BAR1 = 0. */
	DBI_WRITE4(sc, PCI_MAPREG_START, 0x4);
	DBI_WRITE4(sc, PCI_BAR(1), 0x0);

	/* Interrupt pin = INTA. */
	DBI_WRITE4(sc, PCI_INTERRUPT_REG, 0x100);

	/* Primary/secondary/subordinate bus numbers. */
	DBI_WRITE4(sc, PCI_BRIDGE_BUS_REG, 0x00ff0100);

	DBI_WRITE4(sc, PCI_COMMAND_STATUS_REG,
	    PCI_COMMAND_IO_ENABLE | PCI_COMMAND_MEM_ENABLE |
	    PCI_COMMAND_MASTER_ENABLE | PCI_COMMAND_SERR_ENABLE);

	DBI_WRITE4(sc, PCI_CLASS_REG,
	    (PCI_CLASS_BRIDGE << PCI_CLASS_SHIFT) |
	    (PCI_SUBCLASS_BRIDGE_PCI << PCI_SUBCLASS_SHIFT));

	val = DBI_READ4(sc, DBI_LINK_WIDTH_SPEED_CONTROL);
	DBI_WRITE4(sc, DBI_LINK_WIDTH_SPEED_CONTROL, val | SPEED_CHANGE);

	/* Disable the DBI2 (type-1 header shadow) BARs. */
	DBI_WRITE4(sc, DBI2_BASE + PCI_MAPREG_START, 0x0);
	DBI_WRITE4(sc, DBI2_BASE + PCI_BAR(1), 0x0);

	DBI_WRITE4(sc, DBI_MISC_CONTROL_1,
	    DBI_READ4(sc, DBI_MISC_CONTROL_1) & ~DBI_RO_WR_EN);
}

/* Link capability/width programming, from fdwpcie FDwPcieConfigureLink. */
static void
rk_pcie_configure_link(struct rk_pcie_softc *sc)
{
	uint32_t val;

	DBI_WRITE4(sc, DBI_MISC_CONTROL_1,
	    DBI_READ4(sc, DBI_MISC_CONTROL_1) | DBI_RO_WR_EN);

	val = DBI_READ4(sc, DBI_LINK_CAPABILITY);
	val &= ~TARGET_LINK_SPEED_MASK;
	val |= sc->sc_max_link_speed;
	DBI_WRITE4(sc, DBI_LINK_CAPABILITY, val);

	val = DBI_READ4(sc, DBI_LINK_CTL2);
	val &= ~TARGET_LINK_SPEED_MASK;
	val |= sc->sc_max_link_speed;
	DBI_WRITE4(sc, DBI_LINK_CTL2, val);

	val = DBI_READ4(sc, DBI_PORT_LINK_CONTROL);
	val &= ~PORT_LINK_FAST_LINK_MODE;
	val &= ~PORT_LINK_MODE_MASK;
	DBI_WRITE4(sc, DBI_PORT_LINK_CONTROL,
	    val | PORT_LINK_MODE(sc->sc_num_lanes));

	val = DBI_READ4(sc, DBI_LINK_WIDTH_SPEED_CONTROL);
	val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
	DBI_WRITE4(sc, DBI_LINK_WIDTH_SPEED_CONTROL,
	    val | PORT_LOGIC_LINK_WIDTH(sc->sc_num_lanes));

	DBI_WRITE4(sc, DBI_MISC_CONTROL_1,
	    DBI_READ4(sc, DBI_MISC_CONTROL_1) & ~DBI_RO_WR_EN);
}

/* One LTSSM training sequence, from fdwpcie FDwPcieEnableLink. */
static void
rk_pcie_enable_link(struct rk_pcie_softc *sc)
{
	uint32_t val;

	rk_pcie_perst(sc, false);
	APB_WRITE4(sc, PCIE_CLIENT_GENERAL_CON, LTSSM_DISABLE);
	APB_WRITE4(sc, PCIE_CLIENT_GENERAL_DEBUG, 0);
	val = APB_READ4(sc, PCIE_CLIENT_DEBUG_FIFO);
	APB_WRITE4(sc, PCIE_CLIENT_DEBUG_FIFO, val | 0xffff0007);
	APB_WRITE4(sc, PCIE_CLIENT_GENERAL_CON, LTSSM_ENABLE);
	delay(200 * 1000);	/* T_PVPERL */
	rk_pcie_perst(sc, true);
	delay(LINK_WAIT_MS * 1000);
}

static int
rk_pcie_wait_link_up(struct rk_pcie_softc *sc)
{
	for (u_int n = 0; n < LINK_WAIT_RETRIES; n++) {
		if (rk_pcie_link_up(sc)) {
			/* Recheck after 1 s: a bare L0 reading can be
			 * transient and drop back to Polling.Speed. */
			delay(1000 * 1000);
			if (rk_pcie_link_up(sc))
				return 0;
		}
		delay(LINK_WAIT_MS * 1000);
	}
	return EIO;
}

static int
rk_pcie_train(struct rk_pcie_softc *sc)
{
	int error = EIO;

	for (u_int n = 0; n < LINK_MAX_RETRIES; n++) {
		rk_pcie_configure_link(sc);
		rk_pcie_enable_link(sc);
		error = rk_pcie_wait_link_up(sc);
		if (error == 0)
			break;
		device_printf(sc->sc_phsc.sc_dev,
		    "link training failed (attempt %u, ltssm_status=0x%08x)\n",
		    n + 1, APB_READ4(sc, PCIE_CLIENT_LTSSM_STATUS));
		APB_WRITE4(sc, PCIE_CLIENT_GENERAL_CON, LTSSM_DISABLE);
		rk_pcie_perst(sc, false);
	}
	if (error == 0) {
		const uint32_t lstat = DBI_READ4(sc, DBI_LINK_STATUS);
		const u_int speed = __SHIFTOUT(lstat, __BITS(19, 16));
		const u_int width = __SHIFTOUT(lstat, __BITS(25, 20));

		device_printf(sc->sc_phsc.sc_dev,
		    "link up, Gen%u x%u (lstat=0x%08x)\n",
		    speed, width, lstat);
	}
	return error;
}

/* INTx interrupt controller (rk3399_pcie pattern). */
static void *
rk_pcie_intx_establish(device_t dev, u_int *specifier, int ipl, int flags,
    int (*func)(void *), void *arg, const char *xname)
{
	struct rk_pcie_softc *sc = device_private(dev);
	void *ih;

	/* Unmask the legacy interrupts. */
	APB_WRITE4(sc, PCIE_CLIENT_INTR_MASK_LEGACY, 0x00ff0000);

	ih = fdtbus_intr_establish_byname(sc->sc_phsc.sc_phandle,
	    "legacy", ipl, flags, func, arg, xname);

	return ih;
}

static void
rk_pcie_intx_disestablish(device_t dev, void *ih)
{
	struct rk_pcie_softc *sc = device_private(dev);

	fdtbus_intr_disestablish(sc->sc_phsc.sc_phandle, ih);
}

static bool
rk_pcie_intx_intrstr(device_t dev, u_int *specifier, char *buf, size_t buflen)
{
	struct rk_pcie_softc *sc = device_private(dev);

	/* "legacy" is the fourth entry of interrupt-names. */
	fdtbus_intr_str(sc->sc_phsc.sc_phandle, 3, buf, buflen);

	return true;
}

static struct fdtbus_interrupt_controller_func rk_pcie_intrfuncs = {
	.establish = rk_pcie_intx_establish,
	.disestablish = rk_pcie_intx_disestablish,
	.intrstr = rk_pcie_intx_intrstr,
};

static void
rk_pcie_rc_mode(struct rk_pcie_softc *sc)
{
	APB_WRITE4(sc, PCIE_CLIENT_POWER, CLIENT_POWER_RC);
	APB_WRITE4(sc, PCIE_CLIENT_GENERAL_CON, GENERAL_CON_RC_MODE);
}

int
rk_pcie_bus_maxdevs(void *v, int bus)
{
	struct rk_pcie_softc *sc = v;

	if (bus == sc->sc_phsc.sc_bus_min ||
	    bus == sc->sc_phsc.sc_bus_min + 1)
		return 1;
	return 32;
}

pcitag_t
rk_pcie_make_tag(void *v, int bus, int device, int function)
{
	return ((bus << 20) | (device << 15) | (function << 12));
}

void
rk_pcie_decompose_tag(void *v, pcitag_t tag, int *bp, int *dp, int *fp)
{
	if (bp != NULL)
		*bp = (tag >> 20) & 0xff;
	if (dp != NULL)
		*dp = (tag >> 15) & 0x1f;
	if (fp != NULL)
		*fp = (tag >> 12) & 0x7;
}

/* Only dev 0 exists on the root and first subordinate bus. */
static bool
rk_pcie_conf_ok(struct rk_pcie_softc *sc, int bus, int dev, int offset)
{
	if ((unsigned int)offset >= 4096)
		return false;
	if (dev != 0 &&
	    (bus == (int)sc->sc_phsc.sc_bus_min ||
	     bus == (int)sc->sc_phsc.sc_bus_min + 1))
		return false;
	return true;
}

pcireg_t
rk_pcie_conf_read(void *v, pcitag_t tag, int offset)
{
	struct rk_pcie_softc *sc = v;
	int bus, dev, fn;
	uint32_t val;

	rk_pcie_decompose_tag(sc, tag, &bus, &dev, &fn);
	if (!rk_pcie_conf_ok(sc, bus, dev, offset))
		return 0xffffffff;

	if (bus == (int)sc->sc_phsc.sc_bus_min) {
		val = DBI_READ4(sc, offset);
	} else {
		const uint32_t type = bus == (int)sc->sc_phsc.sc_bus_min + 1
		    ? ATU_TYPE_CFG0 : ATU_TYPE_CFG1;
		const uint32_t pci_addr =
		    ((uint32_t)bus << 24) | (dev << 19) | (fn << 16);

		mutex_spin_enter(&sc->sc_conf_lock);
		rk_pcie_atu_prog(sc, ATU_OB_INDEX_CFG, type,
		    sc->sc_cfg_addr, pci_addr, sc->sc_cfg_size);
		val = bus_space_read_4(sc->sc_bst, sc->sc_cfg_bh, offset);
		mutex_spin_exit(&sc->sc_conf_lock);
	}
	return val;
}

void
rk_pcie_conf_write(void *v, pcitag_t tag, int offset, pcireg_t data)
{
	struct rk_pcie_softc *sc = v;
	int bus, dev, fn;

	rk_pcie_decompose_tag(sc, tag, &bus, &dev, &fn);
	if (!rk_pcie_conf_ok(sc, bus, dev, offset))
		return;

	if (bus == (int)sc->sc_phsc.sc_bus_min) {
		DBI_WRITE4(sc, offset, data);
	} else {
		const uint32_t type = bus == (int)sc->sc_phsc.sc_bus_min + 1
		    ? ATU_TYPE_CFG0 : ATU_TYPE_CFG1;
		const uint32_t pci_addr =
		    ((uint32_t)bus << 24) | (dev << 19) | (fn << 16);

		mutex_spin_enter(&sc->sc_conf_lock);
		rk_pcie_atu_prog(sc, ATU_OB_INDEX_CFG, type,
		    sc->sc_cfg_addr, pci_addr, sc->sc_cfg_size);
		bus_space_write_4(sc->sc_bst, sc->sc_cfg_bh, offset, data);
		mutex_spin_exit(&sc->sc_conf_lock);
	}
}

static int
rk_pcie_conf_hook(void *v, int b, int d, int f, pcireg_t id)
{
	return (PCI_CONF_DEFAULT & ~PCI_CONF_ENABLE_BM) | PCI_CONF_MAP_ROM;
}

static void
rk_pcie_attach(device_t parent, device_t self, void *aux)
{
	struct rk_pcie_softc * const sc = device_private(self);
	struct pcihost_softc * const phsc = &sc->sc_phsc;
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_space_handle_t cru_bh;
	bus_addr_t dbi_addr, apb_addr, cfg_addr;
	bus_size_t dbi_size, apb_size;
	const u_int *bus_range;
	int len, error;

	phsc->sc_dev = self;
	phsc->sc_bst = faa->faa_bst;
	phsc->sc_pci_bst = faa->faa_bst;
	phsc->sc_dmat = faa->faa_dmat;
	phsc->sc_phandle = phandle;
	sc->sc_bst = faa->faa_bst;

	if (fdtbus_get_reg_byname(phandle, "pcie-dbi", &dbi_addr,
	    &dbi_size) != 0) {
		aprint_error(": couldn't get pcie-dbi registers\n");
		return;
	}
	if (fdtbus_get_reg_byname(phandle, "pcie-apb", &apb_addr,
	    &apb_size) != 0) {
		aprint_error(": couldn't get pcie-apb registers\n");
		return;
	}
	if (fdtbus_get_reg_byname(phandle, "config", &cfg_addr,
	    &sc->sc_cfg_size) != 0) {
		aprint_error(": couldn't get config window\n");
		return;
	}
	sc->sc_cfg_addr = cfg_addr;

	if (bus_space_map(sc->sc_bst, apb_addr, apb_size, 0,
	    &sc->sc_apb_bh) != 0 ||
	    bus_space_map(sc->sc_bst, dbi_addr, dbi_size, 0,
	    &sc->sc_dbi_bh) != 0 ||
	    bus_space_map(sc->sc_bst, cfg_addr, sc->sc_cfg_size, 0,
	    &sc->sc_cfg_bh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	aprint_naive("\n");
	aprint_normal(": RK3568 DWC PCIe (apb 0x%08x)\n",
	    (uint32_t)apb_addr);

	/* Instance parameters follow the APB address: pcie2x1 @ 0xfe260000
	 * (M.2, combphy2) vs pcie3x2 @ 0xfe280000 (x4 slot, pcie30phy). */
	if (apb_addr == 0xfe260000) {
		sc->sc_softrst_id = SRST_PCIE20_POWERUP;
		sc->sc_clkgate_con = 12;
		sc->sc_is_pcie30 = false;
	} else {
		sc->sc_softrst_id = SRST_PCIE30X2_POWERUP;
		sc->sc_clkgate_con = 13;
		sc->sc_is_pcie30 = true;
	}

	/* Clocks and the PERST# GPIO. */
	for (u_int i = 0; i < 8; i++) {
		struct clk *clk = fdtbus_clock_get_index(phandle, i);
		if (clk != NULL)
			clk_enable(clk);
	}
	sc->sc_perst = fdtbus_gpio_acquire(phandle, "reset-gpios",
	    GPIO_PIN_OUTPUT);
	sc->sc_phy = fdtbus_phy_get(phandle, "pcie-phy");
	if (sc->sc_phy == NULL) {
		aprint_error(": couldn't get phy\n");
		return;
	}

	if (of_getprop_uint32(phandle, "max-link-speed",
	    &sc->sc_max_link_speed) != 0)
		sc->sc_max_link_speed = 1;
	if (of_getprop_uint32(phandle, "num-lanes", &sc->sc_num_lanes) != 0)
		sc->sc_num_lanes = 1;

	if ((error = bus_space_map(sc->sc_bst, RK3568_CRU_BASE,
	    RK3568_CRU_SIZE, 0, &cru_bh)) != 0) {
		aprint_error(": couldn't map CRU (%d)\n", error);
		return;
	}

	/* An already-trained link (a firmware bring-up) is kept: a reset
	 * would tear down a working link (fdwpcie/u-boot skip-if-up). */
	if (!rk_pcie_link_up(sc)) {
		bus_space_handle_t grf_bh;

		/* Open the controller clock gates. */
		rk_pcie_clkgate_open(sc, cru_bh, 10, CLKGATE_CON10_PIPE_MASK);
		rk_pcie_clkgate_open(sc, cru_bh, sc->sc_clkgate_con, 0x1f);

		/* CLKREQ#/WAKE# fn4 iomux: without it the endpoint sees no
		 * reference clock and LTSSM sticks in Detect.Active. */
		if (bus_space_map(sc->sc_bst, RK3568_GRF_BASE,
		    RK3568_GRF_SIZE, 0, &grf_bh) == 0) {
			bus_space_write_4(sc->sc_bst, grf_bh,
			    apb_addr == 0xfe260000 ? GRF_IOMUX_PCIE20
			    : GRF_IOMUX_PCIE30X2, (0xff << 16) | 0x44);
			bus_space_unmap(sc->sc_bst, grf_bh, RK3568_GRF_SIZE);
		}

		/* PERST# low, controller reset, PHY init, controller
		 * release, PHY calibrate (pcie30phy only). */
		rk_pcie_perst(sc, false);
		rk_pcie_softrst(sc, cru_bh, sc->sc_softrst_id, true);
		delay(10);
		if (fdtbus_phy_enable(sc->sc_phy, true) != 0) {
			aprint_error(": couldn't init phy\n");
			bus_space_unmap(sc->sc_bst, cru_bh, RK3568_CRU_SIZE);
			return;
		}
		rk_pcie_softrst(sc, cru_bh, sc->sc_softrst_id, false);
		delay(10);
		if (fdtbus_phy_enable(sc->sc_phy, true) != 0) {
			aprint_error(": phy not ready\n");
			bus_space_unmap(sc->sc_bst, cru_bh, RK3568_CRU_SIZE);
			return;
		}

		/* Root-complex client configuration (fdwpcie order:
		 * HOT_RESET_CTRL enhance, then RC mode). */
		APB_WRITE4(sc, PCIE_CLIENT_HOT_RESET_CTRL,
		    APB_READ4(sc, PCIE_CLIENT_HOT_RESET_CTRL) |
		    LTSSM_ENHANCE);
		rk_pcie_rc_mode(sc);

		rk_pcie_setup_host(sc);

		if (rk_pcie_train(sc) != 0) {
			aprint_error(": link training failed\n");
			bus_space_unmap(sc->sc_bst, cru_bh, RK3568_CRU_SIZE);
			return;
		}
		bus_space_unmap(sc->sc_bst, cru_bh, RK3568_CRU_SIZE);

		/* Endpoint firmware settle (SM2263EN answers config TLPs
		 * seconds after PERST# release; KI-008). */
		delay(EP_SETTLE_MS * 1000);
	} else {
		aprint_normal_dev(self,
		    "link already up, keeping firmware state\n");
	}
	rk_pcie_atu_init(sc);

	/* INTx interrupt controller (child node) and bus range. */
	fdtbus_register_interrupt_controller(self, OF_child(phandle),
	    &rk_pcie_intrfuncs);

	phsc->sc_bus_min = 0;
	phsc->sc_bus_max = 15;
	bus_range = fdtbus_get_prop(phandle, "bus-range", &len);
	if (bus_range != NULL && len == 8) {
		phsc->sc_bus_min = be32dec(&bus_range[0]);
		phsc->sc_bus_max = be32dec(&bus_range[1]);
	}
	if (phsc->sc_bus_min != 0) {
		aprint_error_dev(self, "bus-range doesn't start at 0\n");
		return;
	}

	phsc->sc_type = PCIHOST_ECAM;
	if (!of_hasprop(phandle, "netbsd,no-msi")) {
		phsc->sc_pci_flags |= PCI_FLAGS_MSI_OKAY;
		phsc->sc_pci_flags |= PCI_FLAGS_MSIX_OKAY;
	}
	pcihost_init(&phsc->sc_pc, sc);
	phsc->sc_pc.pc_bus_maxdevs = rk_pcie_bus_maxdevs;
	phsc->sc_pc.pc_make_tag = rk_pcie_make_tag;
	phsc->sc_pc.pc_decompose_tag = rk_pcie_decompose_tag;
	phsc->sc_pc.pc_conf_read = rk_pcie_conf_read;
	phsc->sc_pc.pc_conf_write = rk_pcie_conf_write;
	phsc->sc_pc.pc_conf_hook = rk_pcie_conf_hook;

	mutex_init(&sc->sc_conf_lock, MUTEX_DEFAULT, IPL_HIGH);

	/* The bus scan in pcihost_init2 is one-shot: wait until the
	 * downstream device answers config TLPs (or report the timeout
	 * and enumerate anyway, mirroring the FreeBSD fork). */
	for (int n = 0; n < EP_POLL_RETRIES; n++) {
		const pcireg_t id = rk_pcie_conf_read(sc,
		    rk_pcie_make_tag(sc, phsc->sc_bus_min + 1, 0, 0),
		    PCI_ID_REG);

		if (id != 0xffffffff)
			break;
		if (n == 0)
			aprint_normal_dev(self,
			    "waiting for downstream device\n");
		delay(EP_POLL_MS * 1000);
	}
	{
		const pcireg_t id = rk_pcie_conf_read(sc,
		    rk_pcie_make_tag(sc, phsc->sc_bus_min + 1, 0, 0),
		    PCI_ID_REG);

		aprint_normal_dev(self, "downstream 00:01.0: 0x%08x%s\n", id,
		    id == 0xffffffff ? " (no device)" : "");
	}

	pcihost_init2(phsc);
}
