/* $NetBSD$ */

/*-
 * RK3568 CRU driver (fixed-rate bring-up stub) definitions.
 *
 * Clock IDs below must match the DT binding header
 * dt-bindings/clock/rk3568-cru.h (vendored under external/gpl2/dts);
 * the kernel does not include the dt-bindings headers directly.
 */

#ifndef _ARM_RK3568_CRU_H
#define _ARM_RK3568_CRU_H

#include <sys/device.h>
#include <sys/bus.h>

#include <dev/clk/clk_backend.h>

/* PMU CRU clock IDs */
#define RK3568_CLK_I2C0		7
#define RK3568_SCLK_UART0	11
#define RK3568_PCLK_UART0	44
#define RK3568_PCLK_I2C0	45

/* Main CRU clock IDs */
#define RK3568_PCLK_UART1	284
#define RK3568_SCLK_UART1	287
#define RK3568_PCLK_UART2	288
#define RK3568_SCLK_UART2	291
#define RK3568_PCLK_UART3	292
#define RK3568_SCLK_UART3	295
#define RK3568_PCLK_UART4	296
#define RK3568_SCLK_UART4	299
#define RK3568_PCLK_UART5	300
#define RK3568_SCLK_UART5	303
#define RK3568_PCLK_UART6	304
#define RK3568_SCLK_UART6	307
#define RK3568_PCLK_UART7	308
#define RK3568_SCLK_UART7	311
#define RK3568_PCLK_UART8	312
#define RK3568_SCLK_UART8	315
#define RK3568_PCLK_UART9	316
#define RK3568_SCLK_UART9	319

#define RK3568_PCLK_I2C1	327
#define RK3568_CLK_I2C1		328
#define RK3568_PCLK_I2C2	329
#define RK3568_CLK_I2C2		330
#define RK3568_PCLK_I2C3	331
#define RK3568_CLK_I2C3		332
#define RK3568_PCLK_I2C4	333
#define RK3568_CLK_I2C4		334
#define RK3568_PCLK_I2C5	335
#define RK3568_CLK_I2C5		336

#define RK3568_HCLK_SDMMC0	176
#define RK3568_CLK_SDMMC0	177
#define RK3568_HCLK_SDMMC1	178
#define RK3568_CLK_SDMMC1	179
#define RK3568_HCLK_SDMMC2	193
#define RK3568_CLK_SDMMC2	194

#define RK3568_PCLK_TSADC	271
#define RK3568_CLK_TSADC	273

#define RK3568_PCLK_SPI0	337
#define RK3568_CLK_SPI0		338
#define RK3568_PCLK_SPI1	339
#define RK3568_CLK_SPI1		340
#define RK3568_PCLK_SPI2	341
#define RK3568_CLK_SPI2		342
#define RK3568_PCLK_SPI3	343
#define RK3568_CLK_SPI3		344

#define RK3568_ACLK_CRYPTO_NS	106
#define RK3568_HCLK_CRYPTO_NS	107
#define RK3568_CLK_CRYPTO_NS_CORE 108
#define RK3568_CLK_CRYPTO_NS_PKA 109
#define RK3568_CLK_CRYPTO_NS_RNG 110

/* GMAC (eqos) */
#define RK3568_ACLK_GMAC0	180
#define RK3568_PCLK_GMAC0	181
#define RK3568_CLK_MAC0_2TOP	182
#define RK3568_CLK_MAC0_OUT	183
#define RK3568_CLK_MAC0_REFOUT	184
#define RK3568_CLK_GMAC0_PTP_REF 185
#define RK3568_ACLK_GMAC1	195
#define RK3568_PCLK_GMAC1	196
#define RK3568_CLK_MAC1_2TOP	197
#define RK3568_CLK_MAC1_OUT	198
#define RK3568_CLK_MAC1_REFOUT	199
#define RK3568_CLK_GMAC1_PTP_REF 200

#define RK3568_SCLK_GMAC0	386
#define RK3568_SCLK_GMAC0_RGMII_SPEED 387
#define RK3568_SCLK_GMAC0_RMII_SPEED 388
#define RK3568_SCLK_GMAC0_RX_TX	389
#define RK3568_SCLK_GMAC1	390
#define RK3568_SCLK_GMAC1_RGMII_SPEED 391
#define RK3568_SCLK_GMAC1_RMII_SPEED 392
#define RK3568_SCLK_GMAC1_RX_TX	393

#define RK3568_PCLK_WDT_NS	277
#define RK3568_TCLK_WDT_NS	278

/* One fixed-rate clock.  base.name is allocated by the driver. */
struct rk3568_cru_clk {
	struct clk	base;
	uint32_t	id;
	uint32_t	rate;
};

struct rk3568_cru_softc {
	device_t		sc_dev;
	int			sc_phandle;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;

	struct clk_domain	sc_clkdom;

	u_int			sc_nclks;
	struct rk3568_cru_clk	sc_clks[];
};

#endif /* _ARM_RK3568_CRU_H */
