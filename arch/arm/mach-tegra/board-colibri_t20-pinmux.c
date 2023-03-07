/*
 * arch/arm/mach-tegra/board-colibri_t20-pinmux.c
 *
 * Copyright (c) 2011-2014 Toradex, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <mach/pinmux.h>

#include "board-colibri_t20.h"
#include "gpio-names.h"

#define DEFAULT_DRIVE(_name)					\
	{							\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,	\
		.hsm = TEGRA_HSM_DISABLE,			\
		.schmitt = TEGRA_SCHMITT_ENABLE,		\
		.drive = TEGRA_DRIVE_DIV_1,			\
		.pull_down = TEGRA_PULL_31,			\
		.pull_up = TEGRA_PULL_31,			\
		.slew_rising = TEGRA_SLEW_SLOWEST,		\
		.slew_falling = TEGRA_SLEW_SLOWEST,		\
	}

#define SET_DRIVE(_name, _hsm, _schmitt, _drive, _pulldn_drive, _pullup_drive, _pulldn_slew, _pullup_slew) \
	{                                               \
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,   \
		.hsm = TEGRA_HSM_##_hsm,                    \
		.schmitt = TEGRA_SCHMITT_##_schmitt,        \
		.drive = TEGRA_DRIVE_##_drive,              \
		.pull_down = TEGRA_PULL_##_pulldn_drive,    \
		.pull_up = TEGRA_PULL_##_pullup_drive,		\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,   \
		.slew_falling = TEGRA_SLEW_##_pullup_slew,	\
	}

static __initdata struct tegra_drive_pingroup_config colibri_t20_drive_pinmux[] = {
	DEFAULT_DRIVE(SDIO1),
	DEFAULT_DRIVE(VI1),

	SET_DRIVE(AO1,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),
	SET_DRIVE(AT1,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),
	SET_DRIVE(DBG,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),
	SET_DRIVE(DDC,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),
	SET_DRIVE(VI2,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),
};

static __initdata int colibri_t20_gpio_input_pinmux[] = {
	/* SODIMM pin 152 tri-stating GMI_CLK as OWR can not be tri-stated */
//currently reserved during board platform GPIO handling
//	TEGRA_GPIO_PK1, /* GMI_CLK multiplexed OWR */

	/* Camera interface aka video input */
#ifdef COLIBRI_T20_VI
	TEGRA_GPIO_PA7, /* SDIO3_CMD multiplexed VI_D6 */
	TEGRA_GPIO_PB4, /* SDIO3_DAT3 multiplexed VI_D7 */
#else /* COLIBRI_T20_VI */
	TEGRA_GPIO_PL4, /* VI_D6 multiplexed SDIO3_CMD */
	TEGRA_GPIO_PL5, /* VI_D7 multiplexed SDIO3_DAT3 */
#endif /* COLIBRI_T20_VI */

	/* SODIMM pin 93 RDnWR */
#if 1
	TEGRA_GPIO_PW0, /* gated GMI_WR_N multiplexed LCD_CS1_N */
#endif

	/* SODIMM pin 44 L_BIAS configured as LCD data enable */
#if 0
	TEGRA_GPIO_PJ1, /* LCD_M1 multiplexed LCD_DE */
#else
	TEGRA_GPIO_PW1, /* LCD_DE multiplexed LCD_M1 */
#endif

	/* SODIMM pin 95 RDY */
#if 1
	TEGRA_GPIO_PI7, /* GMI_IORDY multiplexed GMI_WAIT */
#else
	TEGRA_GPIO_PI5, /* GMI_WAIT multiplexed GMI_IORDY */
#endif

	/* SODIMM pin 99 nPWE */
#if defined(CONFIG_CAN_SJA1000) || defined(CONFIG_CAN_SJA1000_MODULE)
	TEGRA_GPIO_PZ3, /* gated GMI_WR_N multiplexed LCD_WR_N */
#endif

	/* 24-bit LCD lines */
#if 0
	TEGRA_GPIO_PM2, /* SPI2_CS0_N multiplexed LCD_D18 */
	TEGRA_GPIO_PM3, /* SPI2_SCK multiplexed LCD_D19 */
	TEGRA_GPIO_PM4, /* SPI2_MISO multiplexed LCD_D20 */
	TEGRA_GPIO_PM5, /* SPI2_MOSI multiplexed LCD_D21 */
	TEGRA_GPIO_PM6, /* DAP2_DOUT multiplexed LCD_D22 */
	TEGRA_GPIO_PM7, /* DAP2_DIN multiplexed LCD_D23 */
#else
	TEGRA_GPIO_PX3, /* LCD_D18 multiplexed SPI2_CS0_N */
	TEGRA_GPIO_PX2, /* LCD_D19 multiplexed SPI2_SCK */
	TEGRA_GPIO_PX1, /* LCD_D20 multiplexed SPI2_MISO */
	TEGRA_GPIO_PX0, /* LCD_D21 multiplexed SPI2_MOSI */
	TEGRA_GPIO_PA5, /* LCD_D22 multiplexed DAP2_DOUT */
	TEGRA_GPIO_PA4, /* LCD_D23 multiplexed DAP2_DIN */
#endif
};

static __initdata struct tegra_pingroup_config colibri_t20_pinmux[] = {
/*	tegra_pingroup		tegra_mux_func		tegra_pullupdown	tegra_tristate
   TRISTATE here means output driver is tri-stated, even configuring GPIO function won't override this.
   Tristating would significantly reduce I/O power consumption. */
	/* nRESET_OUT de-asserted further down below, GPIO I3, I4 and I6 */
	{TEGRA_PINGROUP_ATA,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_ATB,	TEGRA_MUX_SDIO4,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* nCSx, AD0, AD1, AD2, AD3, AD4, AD5, AD6 and AD7, nWR, nOE, GPIO K0, K1, K2, K3 and K4 */
	{TEGRA_PINGROUP_ATC,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* AD8, AD9, AD10 and AD11 */
	{TEGRA_PINGROUP_ATD,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* AD12, AD13, AD14 and AD15 */
	{TEGRA_PINGROUP_ATE,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* WM9715L XTL_IN */
//audio sync clk could be either AC97 or PLLA_OUT0
//SYNC_CLK_DOUBLER_ENB: Enable audio sync clk doubler.
//	{TEGRA_PINGROUP_CDEV1,	TEGRA_MUX_AUDIO_SYNC,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_CDEV1,	TEGRA_MUX_PLLA_OUT,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* USB3340 REFCLK */
	{TEGRA_PINGROUP_CDEV2,	TEGRA_MUX_PLLP_OUT4,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* CLK_32K_IN pin pull-up/down control only */
	{TEGRA_PINGROUP_CK32,	TEGRA_MUX_NONE,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_CRTP,	TEGRA_MUX_CRT,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_CSUS,	TEGRA_MUX_VI_SENSOR_CLK,TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO N0, N1, N2 and N3 */
	{TEGRA_PINGROUP_DAP1,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO A2 and A3 */
	{TEGRA_PINGROUP_DAP2,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_DAP3,	TEGRA_MUX_DAP3,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* A13, A14, A15 and A16, GPIO P4, P5, P6 and P7 */
	{TEGRA_PINGROUP_DAP4,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* DDC pins pulled up to avoid issues if no FFC cable is connected to X3 */
	{TEGRA_PINGROUP_DDC,	TEGRA_MUX_I2C2,		TEGRA_PUPD_PULL_UP,	TEGRA_TRI_NORMAL},
	/* Pull-up/down control only */
	{TEGRA_PINGROUP_DDRC,	TEGRA_MUX_NONE,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO D5 */
	{TEGRA_PINGROUP_DTA,	TEGRA_MUX_VI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO T2 and T3 as KEY_FIND */
#ifdef CONFIG_KEYBOARD_GPIO
	{TEGRA_PINGROUP_DTB,	TEGRA_MUX_VI,		TEGRA_PUPD_PULL_UP,	TEGRA_TRI_NORMAL},
#else
	{TEGRA_PINGROUP_DTB,	TEGRA_MUX_VI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
#endif
	{TEGRA_PINGROUP_DTC,	TEGRA_MUX_VI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO L0, L1, L2, L3, L6 and L7
	   GPIO L4 and L5 multiplexed with PWM<A> and PWM<D> */
	{TEGRA_PINGROUP_DTD,	TEGRA_MUX_VI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* AX88772B V_BUS, WM9715L PENDOWN, GPIO A0, BB4 as KEY_VOLUMEUP and BB5 as KEY_VOLUMEDOWN */
#ifdef CONFIG_KEYBOARD_GPIO
	{TEGRA_PINGROUP_DTE,	TEGRA_MUX_RSVD,		TEGRA_PUPD_PULL_UP,	TEGRA_TRI_NORMAL},
#else
	{TEGRA_PINGROUP_DTE,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
#endif
	/* GPIO BB2 as KEY_BACK and BB3 as KEY_HOME */
#ifdef CONFIG_KEYBOARD_GPIO
	{TEGRA_PINGROUP_DTF,	TEGRA_MUX_I2C3,		TEGRA_PUPD_PULL_UP,	TEGRA_TRI_NORMAL},
#else
	{TEGRA_PINGROUP_DTF,	TEGRA_MUX_I2C3,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
#endif
	{TEGRA_PINGROUP_GMA,	TEGRA_MUX_SDIO4,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* MM_CD */
	{TEGRA_PINGROUP_GMB,	TEGRA_MUX_GMI_INT,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* BT_TXD, BT_RXD, BT_CTS and BT_RTS, GPIO K7 */
	{TEGRA_PINGROUP_GMC,	TEGRA_MUX_UARTD,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_GMD,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO AA4, AA5, AA6 and AA7 */
#ifndef SDHCI_8BIT
	{TEGRA_PINGROUP_GME,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
#else
	{TEGRA_PINGROUP_GME,	TEGRA_MUX_SDIO4,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
#endif
	/* A6, A7, A8, A9, A10, A11 and A12, GPIO U6 */
	{TEGRA_PINGROUP_GPU,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_GPU7,	TEGRA_MUX_RTCK,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* AX88772B RESET_N and EXTWAKEUP_N */
	{TEGRA_PINGROUP_GPV,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* HDMI HOTPLUG_DETECT */
	{TEGRA_PINGROUP_HDINT,	TEGRA_MUX_HDMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_I2CP,	TEGRA_MUX_I2C,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* A0 */
	{TEGRA_PINGROUP_IRRX,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* A1 */
	{TEGRA_PINGROUP_IRTX,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_KBCA,	TEGRA_MUX_NAND,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_KBCB,	TEGRA_MUX_NAND,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_KBCC,	TEGRA_MUX_NAND,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_KBCD,	TEGRA_MUX_NAND,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_KBCE,	TEGRA_MUX_NAND,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_KBCF,	TEGRA_MUX_NAND,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO N4 */
	{TEGRA_PINGROUP_LCSN,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD0,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD1,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD10,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD11,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD12,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD13,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD14,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD15,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD16,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD17,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD2,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD3,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD4,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD5,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD6,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD7,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD8,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LD9,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO N6 */
	{TEGRA_PINGROUP_LDC,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LDI,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LHP0,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LHP1,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LHP2,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LHS,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* Multiplexed RDnWR */
	{TEGRA_PINGROUP_LM0,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LM1,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LPP,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO B2 */
	{TEGRA_PINGROUP_LPW0,	TEGRA_MUX_DISPLAYB,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LPW1,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LPW2,	TEGRA_MUX_DISPLAYB,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LSC0,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* Multiplexed gated nPWE */
	{TEGRA_PINGROUP_LSC1,	TEGRA_MUX_DISPLAYB,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO Z4 */
	{TEGRA_PINGROUP_LSCK,	TEGRA_MUX_DISPLAYB,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO N5 */
	{TEGRA_PINGROUP_LSDA,	TEGRA_MUX_DISPLAYB,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO Z2 */
	{TEGRA_PINGROUP_LSDI,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LSPI,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* THERMD_ALERT */
	{TEGRA_PINGROUP_LVP0,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LVP1,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_LVS,	TEGRA_MUX_DISPLAYA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* OWR multiplexed with GMI_CLK */
	{TEGRA_PINGROUP_OWC,	TEGRA_MUX_OWR,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_PMC,	TEGRA_MUX_PWR_ON,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},

	/* Following five are for specific PMC pin pull-up/down control only */
	/* CLK_32K_OUT */
	{TEGRA_PINGROUP_PMCA,	TEGRA_MUX_NONE,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* SYS_CLK_REQ */
	{TEGRA_PINGROUP_PMCB,	TEGRA_MUX_NONE,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* CORE_PWR_REQ */
	{TEGRA_PINGROUP_PMCC,	TEGRA_MUX_NONE,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* CPU_PWR_REQ */
	{TEGRA_PINGROUP_PMCD,	TEGRA_MUX_NONE,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* PWR_INT_N pulled up */
	{TEGRA_PINGROUP_PMCE,	TEGRA_MUX_NONE,		TEGRA_PUPD_PULL_UP,	TEGRA_TRI_NORMAL},

	/* Gating RDnWR, nPWE through GPIOs further down below */
	{TEGRA_PINGROUP_PTA,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_RM,	TEGRA_MUX_I2C,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* PWM3 */
	/* PWM<D> multiplexed with CIF_DD<6> */
	{TEGRA_PINGROUP_SDB,	TEGRA_MUX_PWM,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO B6 and B7, PWM0 and PWM1 */
	/* PWM<A> multiplexed with CIF_DD<7> */
	{TEGRA_PINGROUP_SDC,	TEGRA_MUX_PWM,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* PWM2 */
	{TEGRA_PINGROUP_SDD,	TEGRA_MUX_PWM,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_SDIO1,	TEGRA_MUX_UARTA,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},

	/* SPI4 */
	{TEGRA_PINGROUP_SLXA,	TEGRA_MUX_SPI4,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_SLXC,	TEGRA_MUX_SPI4,		TEGRA_PUPD_PULL_UP,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_SLXD,	TEGRA_MUX_SPI4,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_SLXK,	TEGRA_MUX_SPI4,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},

	/* GPIO K6 as KEY_MENU multiplexed ACC1_DETECT */
	{TEGRA_PINGROUP_SPDI,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* GPIO K5 multiplexed USB1_VBUS (USBC_DET) */
	{TEGRA_PINGROUP_SPDO,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},

	/* X0, X1, X2, X3, X4, X5, X6 and X7 */
	{TEGRA_PINGROUP_SPIA,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_SPIB,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_SPIC,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_SPID,	TEGRA_MUX_SPI1,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_SPIE,	TEGRA_MUX_SPI1,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_SPIF,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},

	/* USBH_PEN */
	{TEGRA_PINGROUP_SPIG,	TEGRA_MUX_SPI2_ALT,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* USBH_OC */
	{TEGRA_PINGROUP_SPIH,	TEGRA_MUX_SPI2_ALT,	TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},

	/* ULPI data pins pulled up like on NVIDIA's Ventana reference platform */
	{TEGRA_PINGROUP_UAA,	TEGRA_MUX_ULPI,		TEGRA_PUPD_PULL_UP,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_UAB,	TEGRA_MUX_ULPI,		TEGRA_PUPD_PULL_UP,	TEGRA_TRI_NORMAL},

	/* WM9715L RESET#, USB3340 RESETB, WM9715L GENIRQ and GPIO V3 as KEY_POWER */
#ifdef CONFIG_KEYBOARD_GPIO
	{TEGRA_PINGROUP_UAC,	TEGRA_MUX_RSVD,		TEGRA_PUPD_PULL_DOWN,	TEGRA_TRI_NORMAL},
#else
	{TEGRA_PINGROUP_UAC,	TEGRA_MUX_RSVD,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
#endif
	/* STD_TXD and STD_RXD */
	{TEGRA_PINGROUP_UAD,	TEGRA_MUX_IRDA,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* A2 and A3 */
	{TEGRA_PINGROUP_UCA,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	/* A4 and A5 */
	{TEGRA_PINGROUP_UCB,	TEGRA_MUX_GMI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_UDA,	TEGRA_MUX_ULPI,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},

	/* Following two are for specific DDR pin pull-up/down control only */
	{TEGRA_PINGROUP_XM2C,	TEGRA_MUX_NONE,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_XM2D,	TEGRA_MUX_NONE,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
};

#ifdef GMI_32BIT
/* 32-bit wide data and 28-bit wide address bus, more chip selects */
static __initdata struct tegra_pingroup_config colibri_t20_widebus_pinmux[] = {
	/* D28, D29, D30 and D31 */
	{TEGRA_PINGROUP_DAP1,	TEGRA_MUX_GMI, TEGRA_PUPD_NORMAL, TEGRA_TRI_NORMAL},
	/* AD20, AD21, AD22 and AD23 */
	{TEGRA_PINGROUP_GMA,	TEGRA_MUX_GMI, TEGRA_PUPD_NORMAL, TEGRA_TRI_NORMAL},
	/* AD16, AD17, AD18 and AD19 */
	{TEGRA_PINGROUP_GMC,	TEGRA_MUX_GMI, TEGRA_PUPD_NORMAL, TEGRA_TRI_NORMAL},
	/* nCS0 and nCS1 */
	{TEGRA_PINGROUP_GMD,	TEGRA_MUX_GMI, TEGRA_PUPD_NORMAL, TEGRA_TRI_NORMAL},
#ifndef SDHCI_8BIT
	/* AD24, AD25, AD26 and AD27 */
	{TEGRA_PINGROUP_GME,	TEGRA_MUX_GMI, TEGRA_PUPD_NORMAL, TEGRA_TRI_NORMAL},
#endif
};
#endif /* GMI_32BIT */

int __init colibri_t20_pinmux_init(void)
{
	int i;

	tegra_pinmux_config_table(colibri_t20_pinmux, ARRAY_SIZE(colibri_t20_pinmux));
#ifdef GMI_32BIT
	tegra_pinmux_config_table(colibri_t20_widebus_pinmux, ARRAY_SIZE(colibri_t20_widebus_pinmux));
#endif
	tegra_drive_pinmux_config_table(colibri_t20_drive_pinmux,
					ARRAY_SIZE(colibri_t20_drive_pinmux));

	/* configure GPIO inputs on multiplexed balls to tri-state specific functions
	   instead of tri-stating whole groups */
	for(i = 0; i < ARRAY_SIZE(colibri_t20_gpio_input_pinmux); i++) {
		gpio_request(colibri_t20_gpio_input_pinmux[i], "function tri-stated");
		gpio_direction_input(colibri_t20_gpio_input_pinmux[i]);
	}

	/* un-resetting external devices via SODIMM pin 87 nRESET_OUT */
	gpio_request(TEGRA_GPIO_PI4, "SODIMM 87 nRESET_OUT");
	gpio_direction_output(TEGRA_GPIO_PI4, 1);

#if defined(CONFIG_CAN_SJA1000) || defined(CONFIG_CAN_SJA1000_MODULE)
	/* not tri-stating GMI_WR_N on SODIMM pin 99 nPWE */
	gpio_request(TEGRA_GPIO_PT5, "GMI_WR_N on 99");
	gpio_direction_output(TEGRA_GPIO_PT5, 0);
#else /* CONFIG_CAN_SJA1000 | CONFIG_CAN_SJA1000_MODULE */
	/* tri-stating GMI_WR_N on SODIMM pin 99 nPWE */
	gpio_request(TEGRA_GPIO_PT5, "no GMI_WR_N on 99");
	gpio_direction_output(TEGRA_GPIO_PT5, 1);
#endif /* CONFIG_CAN_SJA1000 | CONFIG_CAN_SJA1000_MODULE */

	/* not tri-stating GMI_WR_N on SODIMM pin 93 RDnWR */
	gpio_request(TEGRA_GPIO_PT6, "GMI_WR_N on 93 RDnWR");
	gpio_direction_output(TEGRA_GPIO_PT6, 0);

	return 0;
}
