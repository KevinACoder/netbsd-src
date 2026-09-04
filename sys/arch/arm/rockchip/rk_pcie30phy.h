/* $NetBSD$ */

/*-
 * RK3568 PCIe3 NanoEng PHY (pcie30phy) internal definitions.
 */

#ifndef _ARM_RK_PCIE30PHY_H
#define _ARM_RK_PCIE30PHY_H

/* SRAM firmware entry count (u-boot phy-rockchip-snps-pcie3.fw). */
#define	RK3568_PCIE30PHY_FW_ENTRIES	8192

/* Defined in rk_pcie30phy_fw.c. */
extern const uint16_t rk3568_pcie30phy_fw[RK3568_PCIE30PHY_FW_ENTRIES];

#endif /* _ARM_RK_PCIE30PHY_H */
