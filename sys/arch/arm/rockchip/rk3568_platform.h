/* $NetBSD$ */

/*-
 * Minimal RK3568 platform glue for the RK3568-E4AP5G1-ITX board.
 *
 * The RK3568 memory map places the whole peripheral area (PMU at
 * 0xfdc00000 through UART2 at 0xfe660000) in a single 32-bit window below
 * 4GB, so one devmap entry covering the common MMIO region suffices for
 * early console and the FDT bus.
 */

#ifndef _ARM_RK3568_PLATFORM_H
#define _ARM_RK3568_PLATFORM_H

#include <arch/evbarm/fdt/platform.h>

#define RK3568_CORE_VBASE	KERNEL_IO_VBASE
#define RK3568_CORE_PBASE	0xfdc00000
#define RK3568_CORE_SIZE	0x03200000

/* UART2 baud clock: 24 MHz PLL "ppll" fed through a divider, set up by
 * U-Boot; matches what the firmware leaves the console UART running at. */
#define RK3568_UART_FREQ	24000000

#endif /* _ARM_RK3568_PLATFORM_H */
