/*
 * arch/arm/mach-tegra/board-colibri_t30-panel.c
 *
 * Copyright (c) 2012, Toradex, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <asm/atomic.h>
#include <asm/mach-types.h>

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <linux/ion.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <linux/platform_device.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <linux/tegra_ion.h>

#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/smmu.h>

#include "board.h"
#include "board-colibri_t30.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra3_host1x_devices.h"


//static struct regulator *colibri_t30_hdmi_pll = NULL;
//static struct regulator *colibri_t30_hdmi_reg = NULL;


/* colibri_t30 default display board pins */
#define colibri_t30_lvds_avdd_en		TEGRA_GPIO_PH6
#define colibri_t30_lvds_rst			TEGRA_GPIO_PG7
#define colibri_t30_lvds_shutdown		TEGRA_GPIO_PB2
#define colibri_t30_lvds_rs			TEGRA_GPIO_PV6
#define colibri_t30_lvds_lr			TEGRA_GPIO_PG1

/* colibri_t30 A00 display board pins */
#define colibri_t30_lvds_rs_a00		TEGRA_GPIO_PH1

/* common pins( backlight ) for all display boards */
//#define colibri_t30_bl_enb			TEGRA_GPIO_PH3
#define colibri_t30_bl_pwm			TEGRA_GPIO_PH0
#define colibri_t30_hdmi_hpd			TEGRA_GPIO_PN7
static atomic_t sd_brightness = ATOMIC_INIT(255);

static struct regulator *colibri_t30_lvds_reg;
static struct regulator *colibri_t30_lvds_vdd_panel;

static tegra_dc_bl_output colibri_t30_bl_output_measured = {
	0, 13, 13, 13, 13, 13, 13, 13,
	13, 13, 13, 13, 13, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 49, 50, 51, 52, 53, 54,
	55, 56, 57, 58, 59, 60, 61, 62,
	63, 64, 65, 66, 67, 68, 69, 70,
	70, 72, 73, 74, 75, 76, 77, 78,
	79, 80, 81, 82, 83, 84, 85, 86,
	87, 88, 89, 90, 91, 92, 93, 94,
	95, 96, 97, 98, 99, 100, 101, 102,
	103, 104, 105, 106, 107, 108, 110, 111,
	112, 113, 114, 115, 116, 117, 118, 119,
	120, 121, 122, 123, 124, 124, 125, 126,
	127, 128, 129, 130, 131, 132, 133, 133,
	134, 135, 136, 137, 138, 139, 140, 141,
	142, 143, 144, 145, 146, 147, 148, 148,
	149, 150, 151, 152, 153, 154, 155, 156,
	157, 158, 159, 160, 161, 162, 163, 164,
	165, 166, 167, 168, 169, 170, 171, 172,
	173, 174, 175, 176, 177, 179, 180, 181,
	182, 184, 185, 186, 187, 188, 189, 190,
	191, 192, 193, 194, 195, 196, 197, 198,
	199, 200, 201, 202, 203, 204, 205, 206,
	207, 208, 209, 211, 212, 213, 214, 215,
	216, 217, 218, 219, 220, 221, 222, 223,
	224, 225, 226, 227, 228, 229, 230, 231,
	232, 233, 234, 235, 236, 237, 238, 239,
	240, 241, 242, 243, 244, 245, 246, 247,
	248, 249, 250, 251, 252, 253, 254, 255
};

static p_tegra_dc_bl_output bl_output;

static int colibri_t30_backlight_init(struct device *dev)
{
	int ret = 0;

	bl_output = colibri_t30_bl_output_measured;

	if (WARN_ON(ARRAY_SIZE(colibri_t30_bl_output_measured) != 256))
		pr_err("bl_output array does not have 256 elements\n");

	return ret;
};

static int colibri_t30_backlight_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* Set the backlight GPIO pin mode to 'backlight_enable' */
	//gpio_set_value(colibri_t30_bl_enb, !!brightness);

	/* SD brightness is a percentage, 8-bit value. */
	brightness = (brightness * cur_sd_brightness) / 255;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = bl_output[brightness];

	return brightness;
}

static int colibri_t30_disp1_check_fb(struct device *dev, struct fb_info *info);

static struct platform_pwm_backlight_data colibri_t30_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 40,
	.pwm_period_ns	= 50000,
	.init		= colibri_t30_backlight_init,
	.notify		= colibri_t30_backlight_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= colibri_t30_disp1_check_fb,
};

static struct platform_device colibri_t30_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &colibri_t30_backlight_data,
	},
};

static int colibri_t30_panel_prepoweroff(void)
{
	gpio_set_value(colibri_t30_lvds_shutdown, 0);

	return 0;
}

static int colibri_t30_panel_postpoweron(void)
{
	if (colibri_t30_lvds_reg == NULL) {
		colibri_t30_lvds_reg = regulator_get(NULL, "vdd_backlight");
		if (WARN_ON(IS_ERR(colibri_t30_lvds_reg)))
			pr_err("%s: couldn't get regulator vdd_lvds: %ld\n",
			       __func__, PTR_ERR(colibri_t30_lvds_reg));
		else
			regulator_enable(colibri_t30_lvds_reg);
	}

	mdelay(200);

	gpio_set_value(colibri_t30_lvds_shutdown, 1);

	mdelay(50);

	return 0;
}

static int colibri_t30_panel_enable(void)
{
	if (colibri_t30_lvds_vdd_panel == NULL) {
		colibri_t30_lvds_vdd_panel = regulator_get(NULL, "vdd_lcd_panel");
		if (WARN_ON(IS_ERR(colibri_t30_lvds_vdd_panel)))
			pr_err("%s: couldn't get regulator vdd_lcd_panel: %ld\n",
			       __func__, PTR_ERR(colibri_t30_lvds_vdd_panel));
		else
			regulator_enable(colibri_t30_lvds_vdd_panel);
	}

	return 0;
}

static int colibri_t30_panel_disable(void)
{
	mdelay(5);
	if (colibri_t30_lvds_reg) {
		regulator_disable(colibri_t30_lvds_reg);
		regulator_put(colibri_t30_lvds_reg);
		colibri_t30_lvds_reg = NULL;
	}



	if (colibri_t30_lvds_vdd_panel) {
		regulator_disable(colibri_t30_lvds_vdd_panel);
		regulator_put(colibri_t30_lvds_vdd_panel);
		colibri_t30_lvds_vdd_panel = NULL;
	}

	return 0;
}

#ifdef CONFIG_TEGRA_DC
/*
static int colibri_t30_hdmi_enable(void)
{
	int ret;
	if (!colibri_t30_hdmi_reg) {
		colibri_t30_hdmi_reg = regulator_get(NULL, "avdd_hdmi");
		if (IS_ERR_OR_NULL(colibri_t30_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			colibri_t30_hdmi_reg = NULL;
			return PTR_ERR(colibri_t30_hdmi_reg);
		}
	}
	ret = regulator_enable(colibri_t30_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!colibri_t30_hdmi_pll) {
		colibri_t30_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(colibri_t30_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			colibri_t30_hdmi_pll = NULL;
			regulator_put(colibri_t30_hdmi_reg);
			colibri_t30_hdmi_reg = NULL;
			return PTR_ERR(colibri_t30_hdmi_pll);
		}
	}
	ret = regulator_enable(colibri_t30_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int colibri_t30_hdmi_disable(void)
{
//	regulator_disable(colibri_t30_hdmi_reg);
//	regulator_put(colibri_t30_hdmi_reg);
//	colibri_t30_hdmi_reg = NULL;
//	regulator_disable(colibri_t30_hdmi_pll);
//	regulator_put(colibri_t30_hdmi_pll);
//	colibri_t30_hdmi_pll = NULL;
	return 0;
}
*/
static struct resource colibri_t30_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by colibri_t30_panel_init() */
		.end	= 0,	/* Filled in by colibri_t30_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
};
/*
static struct resource colibri_t30_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
		.start	= 0,
		.end	= 0,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
*/
static struct tegra_fb_data colibri_t30_fb_data = {
	.win			= 0,
	.xres			= 1366,
	.yres			= 768,
	.bits_per_pixel	= 32,
};
/*
static struct tegra_fb_data colibri_t30_hdmi_fb_data = {
	.win		= 0,
	.xres		= 800,
	.yres		= 1280,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};
*/
static struct tegra_dc_mode colibri_t30_panel_modes[] = {
	{
		/* 1366x768@60Hz */
		.pclk 		= 71980000,
		.h_ref_to_sync	= 1,
		.v_ref_to_sync	= 1,
		.h_sync_width 	= 14,
		.v_sync_width 	= 1,
		.h_back_porch 	= 106,
		.v_back_porch 	= 6,
		.h_active 	= 1366,
		.v_active 	= 768,
		.h_front_porch	= 56,
		.v_front_porch	= 3,
	},
};

static struct tegra_dc_out colibri_t30_disp1_out = {
	.type			= TEGRA_DC_OUT_RGB,
	.parent_clk		= "pll_d_out0",
	.parent_clk_backup	= "pll_d2_out0",

	.align			= TEGRA_DC_ALIGN_MSB,
	.order			= TEGRA_DC_ORDER_RED_BLUE,
	.depth			= 18,
	.dither			= TEGRA_DC_ORDERED_DITHER,

	.modes		= colibri_t30_panel_modes,
	.n_modes	= ARRAY_SIZE(colibri_t30_panel_modes),

	.prepoweroff	= colibri_t30_panel_prepoweroff,
	.enable		= colibri_t30_panel_enable,
	.disable	= colibri_t30_panel_disable,
	.postpoweron	= colibri_t30_panel_postpoweron,

	.height		= 132,
	.width			= 235,
};
/*
static struct tegra_dc_out colibri_t30_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,
	.parent_clk	= "pll_d2_out0",
	.dcc_bus	= 3,
	.hotplug_gpio	= colibri_t30_hdmi_hpd,
	.max_pixclock	= KHZ2PICOS(148500),

	// Use 32-bit depth and Full HD for android builds //
#ifdef CONFIG_ANDROID
	.default_mode		= "1920x1080-32@60",
#else // CONFIG_ANDROID //
	.default_mode		= "640x480-16@60",
#endif // CONFIG_ANDROID //

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.enable		= colibri_t30_hdmi_enable,
	.disable	= colibri_t30_hdmi_disable,
};
*/
static struct tegra_dc_platform_data colibri_t30_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &colibri_t30_disp1_out,
	.emc_clk_rate	= 300000000,
	.fb		= &colibri_t30_fb_data,
};
/*
static struct tegra_dc_platform_data colibri_t30_disp2_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &colibri_t30_disp2_out,
	.fb		= &colibri_t30_hdmi_fb_data,
	.emc_clk_rate	= 300000000,
};
*/
static struct nvhost_device colibri_t30_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= colibri_t30_disp1_resources,
	.num_resources	= ARRAY_SIZE(colibri_t30_disp1_resources),
	.dev = {
		.platform_data = &colibri_t30_disp1_pdata,
	},
};

#ifndef COLIBRI_T30_VI
static int colibri_t30_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &colibri_t30_disp1_device.dev;
}
#endif /* !COLIBRI_T30_VI */
/*
static struct nvhost_device colibri_t30_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= colibri_t30_disp2_resources,
	.num_resources	= ARRAY_SIZE(colibri_t30_disp2_resources),
	.dev = {
		.platform_data = &colibri_t30_disp2_pdata,
	},
};
*/
#else /* CONFIG_TEGRA_DC */
static int colibri_t30_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return 0;
}
#endif /* CONFIG_TEGRA_DC */

#if defined(CONFIG_TEGRA_NVMAP)
static struct nvmap_platform_carveout colibri_t30_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by colibri_t30_panel_init() */
		.size		= 0,	/* Filled in by colibri_t30_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data colibri_t30_nvmap_data = {
	.carveouts	= colibri_t30_carveouts,
	.nr_carveouts	= ARRAY_SIZE(colibri_t30_carveouts),
};

static struct platform_device colibri_t30_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &colibri_t30_nvmap_data,
	},
};
#endif /* CONFIG_TEGRA_NVMAP */

#if defined(CONFIG_ION_TEGRA)
static struct platform_device tegra_iommu_device = {
	.name = "tegra_iommu_device",
	.id = -1,
	.dev = {
		.platform_data = (void *)((1 << HWGRP_COUNT) - 1),
	},
};

static struct ion_platform_data tegra_ion_data = {
	.nr = 4,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = TEGRA_ION_HEAP_CARVEOUT,
			.name = "carveout",
			.base = 0,
			.size = 0,
		},
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = TEGRA_ION_HEAP_IRAM,
			.name = "iram",
			.base = TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,
			.size = TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,
		},
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = TEGRA_ION_HEAP_VPR,
			.name = "vpr",
			.base = 0,
			.size = 0,
		},
		{
			.type = ION_HEAP_TYPE_IOMMU,
			.id = TEGRA_ION_HEAP_IOMMU,
			.name = "iommu",
			.base = TEGRA_SMMU_BASE,
			.size = TEGRA_SMMU_SIZE,
			.priv = &tegra_iommu_device.dev,
		},
	},
};

static struct platform_device tegra_ion_device = {
	.name = "ion-tegra",
	.id = -1,
	.dev = {
		.platform_data = &tegra_ion_data,
	},
};
#endif /* CONFIG_ION_TEGRA */

static struct platform_device *colibri_t30_gfx_devices[] __initdata = {
#if defined(CONFIG_TEGRA_NVMAP)
	&colibri_t30_nvmap_device,
#endif
#if defined(CONFIG_ION_TEGRA)
	&tegra_ion_device,
#endif
#ifndef COLIBRI_T30_VI
	&tegra_pwfm0_device,
	&colibri_t30_backlight_device,
#endif /* !COLIBRI_T30_VI */
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend colibri_t30_panel_early_suspender;

static void colibri_t30_panel_early_suspend(struct early_suspend *h)
{
	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);
}

static void colibri_t30_panel_late_resume(struct early_suspend *h)
{
	unsigned i;
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

int __init colibri_t30_panel_init(void)
{
	int err = 0;
	struct resource *res;
	void __iomem *to_io;

	/* enable hdmi hotplug gpio for hotplug detection */
	gpio_request(colibri_t30_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(colibri_t30_hdmi_hpd);

#ifdef CONFIG_HAS_EARLYSUSPEND
	colibri_t30_panel_early_suspender.suspend = colibri_t30_panel_early_suspend;
	colibri_t30_panel_early_suspender.resume = colibri_t30_panel_late_resume;
	colibri_t30_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&colibri_t30_panel_early_suspender);
#endif /* CONFIG_HAS_EARLYSUSPEND */

#ifdef CONFIG_TEGRA_NVMAP
	colibri_t30_carveouts[1].base = tegra_carveout_start;
	colibri_t30_carveouts[1].size = tegra_carveout_size;
#endif /* CONFIG_TEGRA_NVMAP */

#ifdef CONFIG_ION_TEGRA
	tegra_ion_data.heaps[0].base = tegra_carveout_start;
	tegra_ion_data.heaps[0].size = tegra_carveout_size;
#endif /* CONFIG_ION_TEGRA */

#ifdef CONFIG_TEGRA_GRHOST
	err = tegra3_register_host1x_devices();
	if (err)
		return err;
#endif /* CONFIG_TEGRA_GRHOST */

	err = platform_add_devices(colibri_t30_gfx_devices,
				ARRAY_SIZE(colibri_t30_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&colibri_t30_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

//	res = nvhost_get_resource_byname(&colibri_t30_disp2_device,
//					 IORESOURCE_MEM, "fbmem");
//	res->start = tegra_fb2_start;
//	res->end = tegra_fb2_start + tegra_fb2_size - 1;
#endif /* CONFIG_TEGRA_GRHOST & CONFIG_TEGRA_DC */

	/* Make sure LVDS framebuffer is cleared. */
	to_io = ioremap(tegra_fb_start, tegra_fb_size);
	if (to_io) {
		memset(to_io, 0, tegra_fb_size);
		iounmap(to_io);
	} else pr_err("%s: Failed to map LVDS framebuffer\n", __func__);

	/* Make sure HDMI framebuffer is cleared.
	   Note: this seems to fix a tegradc.1 initialisation race in case of
	   framebuffer console as well. */
//	to_io = ioremap(tegra_fb2_start, tegra_fb2_size);
//	if (to_io) {
//		memset(to_io, 0, tegra_fb2_size);
//		iounmap(to_io);
//	} else pr_err("%s: Failed to map HDMI framebuffer\n", __func__);

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	if (!err)
		err = nvhost_device_register(&colibri_t30_disp1_device);

//	if (!err)
//		err = nvhost_device_register(&colibri_t30_disp2_device);
#endif /* CONFIG_TEGRA_GRHOST & CONFIG_TEGRA_DC */

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_NVAVP)
	if (!err)
		err = nvhost_device_register(&nvavp_device);
#endif
	return err;
}
