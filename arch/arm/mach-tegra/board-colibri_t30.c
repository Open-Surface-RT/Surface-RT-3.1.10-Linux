/*
 * arch/arm/mach-tegra/board-colibri_t30.c
 *
 * Copyright (c) 2012-2014 Toradex, Inc.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2. See the file COPYING for more details.
 */

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <linux/can/platform/mcp251x.h>
#include <linux/can/platform/sja1000.h>
#include <linux/clk.h>
#include <linux/colibri_usb.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/input.h>
#include <linux/input/fusion_F0710A.h>
#include <linux/io.h>
#include <linux/leds_pwm.h>
#include <linux/lm95245.h>
#include <linux/mfd/stmpe.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/spi-tegra.h>
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>

#include <mach/io_dpd.h>
#include <mach/sdhci.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/tegra_fiq_debugger.h>
#include <mach/thermal.h>
#include <mach/usb_phy.h>
#include <mach/w1.h>

#include <media/soc_camera.h>
#include <media/tegra_v4l2_camera.h>

#include <sound/wm8962.h>
#include <mach/tegra_wm8962_pdata.h>

#include <linux/nct1008.h>


#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>

#include <mach/pinmux.h>
//#include <mach/pinmux-tegra30.h>

#include <linux/mbtc_plat.h>
//#include <linux/wl12xx.h>

//#include <mach/gpio-tegra.h>
#include <mach/io_dpd.h>
#include <mach/irqs.h>

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/wlan_plat.h>
#include <linux/resource.h>

#include <mach/kbc.h>

#include "wdt-recovery.h"
#include "board-colibri_t30.h"
#include "board.h"
#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"


//#define WIFI_DISABLE_BROKEN 1 // uncomment to disable wifi sdhci
//#define SDCARD_DISABLE_BROKEN 1 // uncomment to disable sdcard sdhci
//#define WIFI_NEED_INIT 1 // uncomment to enable wifi init routine

#define TEGRA_BATTERY_EN TEGRA_GPIO_PN3


//from former drivers/mtd/maps/tegra_nor.h
#define TEGRA_GMI_PHYS			0x70009000
#define TEGRA_GMI_BASE			IO_TO_VIRT(TEGRA_GMI_PHYS)
#define TEGRA_SNOR_CONFIG_REG		(TEGRA_GMI_BASE + 0x00)

//from drivers/mtd/maps/tegra_nor.c
#define __BITMASK0(len)			(BIT(len) - 1)
#define REG_FIELD(val, start, len)	(((val) & __BITMASK0(len)) << (start))

#define TEGRA_SNOR_CONFIG_GO		BIT(31)
#define TEGRA_SNOR_CONFIG_SNOR_CS(val)	REG_FIELD((val), 4, 3)


/* Camera */

#ifdef CONFIG_TEGRA_CAMERA
static struct platform_device tegra_camera = {
	.name	= "tegra_camera",
	.id	= -1,
};


#endif /* CONFIG_TEGRA_CAMERA */

//GPIO KEYS





#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button surface_rt_keys[] = {
	[0] = GPIO_KEY(KEY_POWER, PV0, 1),
	[1] = GPIO_KEY(KEY_VOLUMEUP, PS7, 0),
	[2] = GPIO_KEY(KEY_VOLUMEDOWN, PS6, 0),
	[3] = GPIO_KEY(KEY_HOME, PS5, 1),
};

static struct gpio_keys_platform_data surface_rt_keys_platform_data = {
	.buttons	= surface_rt_keys,
	.nbuttons	= ARRAY_SIZE(surface_rt_keys),
};

static struct platform_device surface_rt_keys_device = {
	.name	= "gpio-keys",
	.id		= 0,
	.dev	= {
		.platform_data  = &surface_rt_keys_platform_data,
	},
};

int __init surface_rt_keys_init(void)
{
	pr_info("%s: registering gpio keys\n", __func__);

	platform_device_register(&surface_rt_keys_device);

	return 0;
}



#ifndef WIFI_DISABLE_BROKEN



#define TEGRA_WLAN_PWR	TEGRA_GPIO_PD4
#define TEGRA_WLAN_RST	TEGRA_GPIO_PD3
#define TEGRA_WLAN_WOW	TEGRA_GPIO_PO4

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int tegra_wifi_status_register(void (*callback)(int , void *), void *);


static int tegra_wifi_reset(int on);
static int tegra_wifi_power(int on);
static int tegra_wifi_set_carddetect(int val);

static struct wifi_platform_data tegra_wifi_control = {
	.set_power      = tegra_wifi_power,
	.set_reset      = tegra_wifi_reset,
	.set_carddetect = tegra_wifi_set_carddetect,
//	.host_sleep_cond = 0xa, /* UNICAST + MULTICAST */
			        /* bit 0: broadcast, 1: unicast, 2: mac,
			               3: multicast, 6: mgmt frame,
				       31: don't wakeup on ipv6 */
//	.host_sleep_gpio = 16,  /* GPIO 16 */
//	.host_sleep_gap	= 0xff,  /* level trigger */

};

static void set_pin_pupd_input(int pin , int pupd , int input)
{
	int err;

	err = tegra_pinmux_set_pullupdown(pin , pupd);
	if (err < 0)
		printk(KERN_ERR "%s: can't set pin %d pullupdown to %d\n", __func__, pin , pupd);

	err = tegra_pinmux_set_io(pin , input);
	if (err < 0)
		printk(KERN_ERR "%s: can't set pin %d e_input to %d\n", __func__, pin , input);
}

static int wifi_sdio_gpio[] = {
	TEGRA_GPIO_PA6,
	TEGRA_GPIO_PA7,
	TEGRA_GPIO_PB7,
	TEGRA_GPIO_PB6,
	TEGRA_GPIO_PB5,
	TEGRA_GPIO_PB4,
};

static int enable_wifi_sdio_func(void)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(wifi_sdio_gpio); i++) {
		gpio_free(wifi_sdio_gpio[i]);
	}

	set_pin_pupd_input(TEGRA_PINGROUP_SDMMC3_CLK , TEGRA_PUPD_NORMAL , TEGRA_PIN_INPUT);
	set_pin_pupd_input(TEGRA_PINGROUP_SDMMC3_CMD , TEGRA_PUPD_PULL_UP, TEGRA_PIN_INPUT);
	set_pin_pupd_input(TEGRA_PINGROUP_SDMMC3_DAT3 , TEGRA_PUPD_PULL_UP , TEGRA_PIN_INPUT);
	set_pin_pupd_input(TEGRA_PINGROUP_SDMMC3_DAT2 , TEGRA_PUPD_PULL_UP , TEGRA_PIN_INPUT);
	set_pin_pupd_input(TEGRA_PINGROUP_SDMMC3_DAT1 , TEGRA_PUPD_PULL_UP , TEGRA_PIN_INPUT);
	set_pin_pupd_input(TEGRA_PINGROUP_SDMMC3_DAT0 , TEGRA_PUPD_PULL_UP , TEGRA_PIN_INPUT);

	return 0;
}

static int disable_wifi_sdio_func(void)
{
	unsigned int rc = 0;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(wifi_sdio_gpio); i++) {
		rc = gpio_request(wifi_sdio_gpio[i], NULL);
		if (rc) {
			printk(KERN_INFO "%s, request gpio %d failed !!!\n", __func__, wifi_sdio_gpio[i]);
			return rc;
		}

		rc = gpio_direction_output(wifi_sdio_gpio[i], 0);
		if (rc) {
			printk(KERN_INFO "%s, direction gpio %d failed !!!\n", __func__, wifi_sdio_gpio[i]);
			return rc;
		}
	}

	set_pin_pupd_input(TEGRA_PINGROUP_SDMMC3_CLK , TEGRA_PUPD_NORMAL , TEGRA_PIN_OUTPUT);
	set_pin_pupd_input(TEGRA_PINGROUP_SDMMC3_CMD , TEGRA_PUPD_NORMAL , TEGRA_PIN_OUTPUT);
	set_pin_pupd_input(TEGRA_PINGROUP_SDMMC3_DAT3 , TEGRA_PUPD_NORMAL , TEGRA_PIN_OUTPUT);
	set_pin_pupd_input(TEGRA_PINGROUP_SDMMC3_DAT2 , TEGRA_PUPD_NORMAL , TEGRA_PIN_OUTPUT);
	set_pin_pupd_input(TEGRA_PINGROUP_SDMMC3_DAT1 , TEGRA_PUPD_NORMAL , TEGRA_PIN_OUTPUT);
	set_pin_pupd_input(TEGRA_PINGROUP_SDMMC3_DAT0 , TEGRA_PUPD_NORMAL , TEGRA_PIN_OUTPUT);
	return 0;
}


static int tegra_wifi_power(int on)
{
	//struct tegra_io_dpd *sd_dpd;

	pr_debug("%s: %d\n", __func__, on);

	/*
	 * FIXME : we need to revisit IO DPD code
	 * on how should multiple pins under DPD get controlled
	 *
	 * enterprise GPIO WLAN enable is part of SDMMC1 pin group
	 
	sd_dpd = tegra_io_dpd_get(&tegra_sdhci_device2.dev);
	if (sd_dpd) {
		mutex_lock(&sd_dpd->delay_lock);
		tegra_io_dpd_disable(sd_dpd);
		mutex_unlock(&sd_dpd->delay_lock);
	}
///////////
	if (on) {
		gpio_set_value(TEGRA_WLAN_RST, 1);
		mdelay(100);
		gpio_set_value(TEGRA_WLAN_RST, 0);
		mdelay(100);
		gpio_set_value(TEGRA_WLAN_RST, 1);
		mdelay(100);
		gpio_set_value(TEGRA_WLAN_PWR, 1);
		mdelay(200);
	} else {
		gpio_set_value(TEGRA_WLAN_RST, 0);
		mdelay(100);
		gpio_set_value(TEGRA_WLAN_PWR, 0);
	}

*/

	if (on)
	    gpio_direction_input(TEGRA_WLAN_WOW);
	else
	    gpio_direction_output(TEGRA_WLAN_WOW, 0);

	if (on) {
	
		//gpio_direction_output(TEGRA_BT_RST, 1);
		enable_wifi_sdio_func();
		if (!gpio_get_value(TEGRA_WLAN_PWR)) {
			gpio_set_value(TEGRA_WLAN_PWR, 1);
		}
	}
	mdelay(100);
	gpio_set_value(TEGRA_WLAN_RST, on);
	mdelay(200);

	/*
	 * When BT and Wi-Fi turn off at the same time, the last one must do the VDD off action.
	 * So BT/WI-FI must check the other's status in order to set VDD low at last.
	 */
	if (!on) {
	
		//if (!gpio_get_value(TEGRA_BT_SHUTDOWN)) {
		//	gpio_direction_output(CTEGRA_BT_RST, 0);
		//	gpio_set_value(TEGRA_WLAN_VDD, 0);
		//}
		disable_wifi_sdio_func();
	}

/*
	if (sd_dpd) {
		mutex_lock(&sd_dpd->delay_lock);
		tegra_io_dpd_enable(sd_dpd);
		mutex_unlock(&sd_dpd->delay_lock);
	}
*/
	return 0;
}

static int tegra_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

static int tegra_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int tegra_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

#ifdef CONFIG_TEGRA_PREPOWER_WIFI
static int __init tegra_wifi_prepower(void)
{
	//if ((!machine_is_tegra_enterprise()) && (!machine_is_tai()))
	//	return 0;

	tegra_wifi_power(1);

	return 0;
}
subsys_initcall_sync(tegra_wifi_prepower);
#endif

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "mwifiex_sdio_irq",
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device tegra_mrvl_wifi_device = {
	.name		= "mwifiex_sdio",
	.id		= 1,
	.num_resources	= 1,
	.resource	= wifi_resource,
	.dev		= {
	.platform_data = &tegra_wifi_control,
	},
};

static struct mbtc_platform_data tegra_mbtc_control = {
	.gpio_gap	= 0x04ff, /* GPIO 4, GAP 0xff (level) */
};

static struct resource bt_resource[] = {
	[0] = {
		.name	= "mrvl_bt_irq",
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device tegra_bt_device = {
	.name		= "mrvl_bt",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(bt_resource),
	.resource	= bt_resource,
	.dev		= {
		.platform_data = &tegra_mbtc_control,
	},
};
#endif // WIFI_DISABLE_BROKEN



#ifdef CONFIG_BATTERY_CHARGER_SURFACE_RT 
static struct i2c_board_info surface_rt_i2c4_battery_charger_info[] = {
        {
                I2C_BOARD_INFO("surface-rt-ec", 0x0A),
        },
};
#endif
/////////////////////////////////////
//THERMAL
/*
static struct throttle_table tj_throttle_table[] = {
//		 CPU_THROT_LOW cannot be used by other than CPU 
//		 NO_CAP cannot be used by CPU
//		    CPU,    CBUS,    SCLK,     EMC 
		{ { 1000000,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  760000,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  760000,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  620000,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  620000,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  620000,  437000,  NO_CAP,  NO_CAP } },
		{ {  620000,  352000,  NO_CAP,  NO_CAP } },
		{ {  475000,  352000,  NO_CAP,  NO_CAP } },
		{ {  475000,  352000,  NO_CAP,  NO_CAP } },
		{ {  475000,  352000,  250000,  375000 } },
		{ {  475000,  352000,  250000,  375000 } },
		{ {  475000,  247000,  204000,  375000 } },
		{ {  475000,  247000,  204000,  204000 } },
		{ {  475000,  247000,  204000,  204000 } },
	  { { CPU_THROT_LOW,  247000,  204000,  102000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init surface_rt_throttle_init(void)
{
	if (machine_is_surface_rt())
		balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(surface_rt_throttle_init);

static struct nct1008_platform_data surface_rt_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08, // 0x08 corresponds to 63Hz conversion rate 
	.offset = 8, // 4 * 2C. 1C for device accuracies 

	.shutdown_ext_limit = 90, // C 
	.shutdown_local_limit = 100, // C 

	.num_trips = 1,
	.trips = {
		// Thermal Throttling 
		[0] = {
			.cdev_type = "tegra-balanced",
			.trip_temp = 80000,
			.trip_type = THERMAL_TRIP_PASSIVE,
			.upper = THERMAL_NO_LIMIT,
			.lower = THERMAL_NO_LIMIT,
			.hysteresis = 0,
		},
	},
};

static struct i2c_board_info surface_rt_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &surface_rt_nct1008_pdata,
		.irq = -1,
	}
};

static int surface_rt_nct1008_init(void)
{
	int ret = 0;

//	 FIXME: enable irq when throttling is supported 
	surface_rt_i2c4_nct1008_board_info[0].irq =
				gpio_to_irq(SURFACE_RT_TEMP_ALERT_GPIO);

	ret = gpio_request(SURFACE_RT_TEMP_ALERT_GPIO, "temp_alert");
	if (ret < 0) {
		pr_err("%s: gpio_request failed\n", __func__);
		return ret;
	}

	ret = gpio_direction_input(SURFACE_RT_TEMP_ALERT_GPIO);
	if (ret < 0) {
		pr_err("%s: set gpio to input failed\n", __func__);
		gpio_free(SURFACE_RT_TEMP_ALERT_GPIO);
	}

	tegra_platform_edp_init(surface_rt_nct1008_pdata.trips,
				&surface_rt_nct1008_pdata.num_trips,
				0); // edp temperature margin 

	return ret;
}
//THERMAL
///////////////////////////////////////
*/







//////////////////////////////////////
//SOUND
static struct wm8962_pdata surface_rt_wm8962_pdata = {
        .irq_active_low = 0,
        .mic_cfg = 0,
        .gpio_base = SURFACE_RT_GPIO_WM8962(0),
        .gpio_init = {
                /* Needs to be filled for digital Mic's */
        },
};

static struct i2c_board_info __initdata surface_rt_codec_wm8962_info = {
        I2C_BOARD_INFO("wm8962", 0x1a),
        .platform_data = &surface_rt_wm8962_pdata,
};
//SOUND
////////////////////////////////////


/* Clocks */
static struct tegra_clk_init_table colibri_t30_clk_init_table[] __initdata = {
	/* name		parent		rate		enabled */



	{"apbif",	"clk_m",	12000000,	false},
	{"audio0",	"i2s0_sync",	0,		false},
	{"audio1",	"i2s1_sync",	0,		false},
	{"audio2",	"i2s2_sync",	0,		false},
	{"audio3",	"i2s3_sync",	0,		false},
	{"audio4",	"i2s4_sync",	0,		false},
	{"blink",	"clk_32k",	32768,		true},
	{"clk_out_2",	"extern2",	24000000,	false},
	{"d_audio",	"clk_m",	12000000,	false},
	{"dam0",	"clk_m",	12000000,	false},
	{"dam1",	"clk_m",	12000000,	false},
	{"dam2",	"clk_m",	12000000,	false},
	{"extern2",	"clk_m",	24000000,	false},
	{"hda",		"pll_p",	108000000,	false},
	{"hda2codec_2x","pll_p",	48000000,	false},
	{"i2c1",	"pll_p",	3200000,	false},
	{"i2c2",	"pll_p",	3200000,	false},
	{"i2c3",	"pll_p",	3200000,	false},
	{"i2c4",	"pll_p",	3200000,	false},
	{"i2c5",	"pll_p",	3200000,	false},
	{"i2s0",	"pll_a_out0",	0,		false},
	{"i2s1",	"pll_a_out0",	0,		false},
	{"i2s2",	"pll_a_out0",	0,		false},
	{"i2s3",	"pll_a_out0",	0,		false},
	{"i2s4",	"pll_a_out0",	0,		false},
	{"pll_p",	NULL,		0,		false},
	{"pll_m",	NULL,		0,		false},
	{"pll_c",       NULL,           0,              false},
	{"pwm",		"pll_p",	5100000,	false},
	{"spdif_out",	"pll_a_out0",	0,		false},
	{NULL,		NULL,		0,		0},

};


static struct tegra_i2c_platform_data colibri_t30_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data colibri_t30_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data colibri_t30_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data colibri_t30_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data colibri_t30_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};



static void colibri_t30_i2c_init(void)
{
//        int err;


	tegra_i2c_device1.dev.platform_data = &colibri_t30_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &colibri_t30_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &colibri_t30_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &colibri_t30_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &colibri_t30_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);


//TEMP SENSOR
/*
	err = surface_rt_nct1008_init();
	if (err)
		pr_err("%s: nct1008 init failed\n", __func__);
	else
		i2c_register_board_info(4, surface_rt_i2c4_nct1008_board_info,
			ARRAY_SIZE(surface_rt_i2c4_nct1008_board_info));
*/

//        surface_rt_codec_wm8962_info.irq = gpio_to_irq(TEGRA_GPIO_CDC_IRQ);
 //       i2c_register_board_info(4, &surface_rt_codec_wm8962_info, 1);


}

#ifdef CONFIG_BATTERY_CHARGER_SURFACE_RT
static void surface_rt_ec_init(void)

{
	int ret;

	gpio_direction_output(TEGRA_BATTERY_EN, 1);
	mdelay(100);
        gpio_set_value(TEGRA_BATTERY_EN, 1);
	mdelay(100);
	ret = gpio_get_value(TEGRA_BATTERY_EN);

	printk(KERN_INFO "Set Surface EC GPIO : %d\n",ret);


        i2c_register_board_info(4, surface_rt_i2c4_battery_charger_info,
                        ARRAY_SIZE(surface_rt_i2c4_battery_charger_info));

}
#endif


static void surface_rt_sound_init(void)

{

//	surface_rt_codec_wm8962_info.irq = gpio_to_irq(TEGRA_GPIO_CDC_IRQ);
//	i2c_register_board_info(4, &surface_rt_codec_wm8962_info, 1);

        surface_rt_codec_wm8962_info.irq = gpio_to_irq(TEGRA_GPIO_CDC_IRQ);
        i2c_register_board_info(4, &surface_rt_codec_wm8962_info, 1);


	printk(KERN_INFO "Surface RT Sound Init \n");


}


static struct tegra_asoc_platform_data cardhu_audio_wm8962_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_ext_mic_en	= -1,
//	.gpio_bypass_switch_en	= TEGRA_GPIO_BYPASS_SWITCH_EN,
//	.gpio_debug_switch_en   = TEGRA_GPIO_DEBUG_SWITCH_EN,
	.i2s_param[HIFI_CODEC]  = {
		.audio_port_id  = 0,
		.is_i2s_master  = 1,
		.i2s_mode       = TEGRA_DAIFMT_I2S,
	},
	.i2s_param[BASEBAND]    = {
		.audio_port_id  = -1,
	},
//	.i2s_param[BT_SCO]      = {
//		.audio_port_id  = 3,
//		.is_i2s_master  = 1,
//		.i2s_mode       = TEGRA_DAIFMT_DSP_A,
//	},
};

static struct platform_device cardhu_audio_wm8962_device = {
	.name	= "tegra-snd-wm8962",
	.id	= 0,
	.dev	= {
		.platform_data = &cardhu_audio_wm8962_pdata,
	},
};








#ifndef WIFI_DISABLE_BROKEN
static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
	.mmc_data = {
		.register_status_notify	= tegra_wifi_status_register,
		.built_in = 0,
	},
};

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};


static int __init tegra_wifi_init(void)
{
	int rc;

	rc = gpio_request(TEGRA_WLAN_PWR, "wlan_power");
	if (rc)
		pr_err("WLAN_PWR gpio request failed:%d\n", rc);
	rc = gpio_request(TEGRA_WLAN_RST, "wlan_rst");
	if (rc)
		pr_err("WLAN_RST gpio request failed:%d\n", rc);
	rc = gpio_request(TEGRA_WLAN_WOW, "bcmsdh_sdmmc");
	if (rc)
		pr_err("WLAN_WOW gpio request failed:%d\n", rc);

	rc = gpio_direction_output(TEGRA_WLAN_PWR, 0);
	if (rc)
		pr_err("WLAN_PWR gpio direction configuration failed:%d\n", rc);
	gpio_direction_output(TEGRA_WLAN_RST, 0);
	if (rc)
		pr_err("WLAN_RST gpio direction configuration failed:%d\n", rc);
	rc = gpio_direction_input(TEGRA_WLAN_WOW);
	if (rc)
		pr_err("WLAN_WOW gpio direction configuration failed:%d\n", rc);


	wifi_resource[0].start = wifi_resource[0].end =
	gpio_to_irq(TEGRA_GPIO_PU5);
	platform_device_register(&tegra_mrvl_wifi_device);
	platform_device_register(&tegra_bt_device);	

	return 0;
}


#endif //WIFI_DISABLE_BROKEN

/* MMC/SD */

#ifndef SDCARD_DISABLE_BROKEN


static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

#endif //SDCARD_DISABLE_BROKEN

/* 
static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};*/

/////////////////////////////////////////////////////////////////////
//original
static struct resource sdhci_resource3[] = {
	[0] = {
		.start	= INT_SDMMC4,
		.end	= INT_SDMMC4,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};


//////////////////////////////////////////////////////////////////////////////////

#ifndef SDCARD_DISABLE_BROKEN

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.cd_gpio = TEGRA_GPIO_PI5,
	.wp_gpio = -1,
	.power_gpio = TEGRA_GPIO_PD7,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
//	.is_voltage_switch_supported = true,
//	.vdd_rail_name = "vddio_sdmmc1",
//	.slot_rail_name = "vddio_sd_slot",
//	.vdd_max_uv = 3320000,
//	.vdd_min_uv = 3280000,
//	.max_clk = 208000000,
//	.is_8bit_supported = false, 
};

#endif //SDCARD_DISABLE_BROKEN



/*
static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
	.mmc_data = {
		.register_status_notify	= tegra_wifi_status_register,
		.built_in = 0,
	},
};*/

////////////////////////////////////////////////////////////////
///original
static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
	.mmc_data = {
		.built_in = 1,
		.ocr_mask = MMC_OCR_1V8_MASK,
	}
};
//////////////////////////////////////////////////////////////////



#ifndef SDCARD_DISABLE_BROKEN
//sdcard
static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};

#endif //SDCARD_DISABLE_BROKEN

/*

//wifi
static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};*/

///////////////////////////////////////////////////////////////////////////////
//original
//emmc
static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static void __init colibri_t30_sdhci_init(void)
{
	/* register eMMC first */

	platform_device_register(&tegra_sdhci_device3);
#ifndef SDCARD_DISABLE_BROKEN
        platform_device_register(&tegra_sdhci_device0);
#endif //SDCARD_DISABLE_BROKEN



#ifndef WIFI_DISABLE_BROKEN
	platform_device_register(&tegra_sdhci_device2);

	tegra_wifi_init();


#endif // WIFI_DISABLE_BROKEN

//	platform_device_register(&tegra_sdhci_device2); //not needed for wifi to work
}
/////////////////////////////////////////////////////////////////////////////////



/* RTC */

#ifdef CONFIG_RTC_DRV_TEGRA
static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start	= TEGRA_RTC_BASE,
		.end	= TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_RTC,
		.end	= INT_RTC,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name		= "tegra_rtc",
	.id		= -1,
	.resource	= tegra_rtc_resources,
	.num_resources	= ARRAY_SIZE(tegra_rtc_resources),
};
#endif /* CONFIG_RTC_DRV_TEGRA */

/* SPI */
/*
#if defined(CONFIG_SPI_TEGRA) && defined(CONFIG_SPI_SPIDEV)
static struct tegra_spi_device_controller_data spidev_controller_data = {
	.cs_hold_clk_count	= 1,
	.cs_setup_clk_count	= 1,
	.is_hw_based_cs		= 1,
};

static struct spi_board_info tegra_spi_devices[] __initdata = {
	{
		.bus_num		= 0,		
#if !defined(CONFIG_CAN_MCP251X) && !defined(CONFIG_CAN_MCP251X_MODULE)
		.chip_select		= 0,
#else 
		.chip_select		= 1,
#endif
		.controller_data	= &spidev_controller_data,
		.irq			= 0,
		.max_speed_hz		= 50000000,
		.modalias		= "spidev",
		.mode			= SPI_MODE_0,
		.platform_data		= NULL,
	},
};

static void __init colibri_t30_register_spidev(void)
{
	spi_register_board_info(tegra_spi_devices,
				ARRAY_SIZE(tegra_spi_devices));
}

#else 
#define colibri_t30_register_spidev() do {} while (0)
#endif 
*/
static struct platform_device *colibri_t30_spi_devices[] __initdata = {
	&tegra_spi_device1,
};

static struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else /* !CONFIG_TEGRA_PLLM_RESTRICTED */
	[1] = {.name = "clk_m"},
#endif /* !CONFIG_TEGRA_PLLM_RESTRICTED */
};

static struct tegra_spi_platform_data colibri_t30_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= 16 * 1024,
	.is_clkon_always	= false,
	.max_rate		= 408000000,
};

static void __init colibri_t30_spi_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	colibri_t30_spi_pdata.parent_clk_list = spi_parent_clk;
	colibri_t30_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device1.dev.platform_data = &colibri_t30_spi_pdata;
	platform_add_devices(colibri_t30_spi_devices,
				ARRAY_SIZE(colibri_t30_spi_devices));
}


/* UART */

static struct platform_device *colibri_t30_uart_devices[] __initdata = {
	&tegra_uarta_device, /* Colibri UART_A (formerly FFUART) */
	&tegra_uartd_device, /* Colibri UART_B (formerly BTUART) */
	&tegra_uartb_device, /* Colibri UART_C (formerly STDUART) */
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data colibri_t30_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = get_tegra_uart_debug_port_id();
	if (debug_port_id < 0) {
		debug_port_id = 0;
	}

	switch (debug_port_id) {
	case 0:
		/* UARTA is the debug port. */
		pr_info("Selecting UARTA as the debug console\n");
		colibri_t30_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;

	case 1:
		/* UARTB is the debug port. */
		pr_info("Selecting UARTB as the debug console\n");
		colibri_t30_uart_devices[2] = &debug_uartb_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uartb");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->mapbase;
		break;

	case 3:
		/* UARTD is the debug port. */
		pr_info("Selecting UARTD as the debug console\n");
		colibri_t30_uart_devices[1] = &debug_uartd_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uartd");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
		break;

	default:
		pr_info("The debug console id %d is invalid, Assuming UARTA",
			debug_port_id);
		colibri_t30_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;
	}
}

static void __init colibri_t30_uart_init(void)
{
	struct clk *c;
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	colibri_t30_uart_pdata.parent_clk_list = uart_parent_clk;
	colibri_t30_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uarta_device.dev.platform_data = &colibri_t30_uart_pdata;
	tegra_uartb_device.dev.platform_data = &colibri_t30_uart_pdata;
	tegra_uartd_device.dev.platform_data = &colibri_t30_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, clk_get_rate(c));
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	platform_add_devices(colibri_t30_uart_devices,
				ARRAY_SIZE(colibri_t30_uart_devices));
}

/* USB */

//TODO: overcurrent

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.has_hostpc	= true,
	.op_mode	= TEGRA_USB_OPMODE_DEVICE,
	.phy_intf	= TEGRA_USB_PHY_INTF_UTMI,
	.port_otg	= true,
	.u_cfg.utmi = {
		//.elastic_limit		= 16,
		//.hssync_start_delay	= 0,
		//.idle_wait_delay	= 17,
		//.term_range_adj		= 6,
		//.xcvr_lsfslew		= 2,
		//.xcvr_lsrslew		= 2,
		//.xcvr_setup		= 8,
		//.xcvr_setup_offset	= 0,
		//.xcvr_use_fuses		= 1,
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
	.u_data.dev = {
		.charging_supported		= false,
		.remote_wakeup_supported	= false,
		.vbus_gpio			= TEGRA_GPIO_PDD6,
		.vbus_pmu_irq			= 0,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.has_hostpc	= true,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.phy_intf	= TEGRA_USB_PHY_INTF_UTMI,
	.port_otg	= true,
	.u_cfg.utmi = {
		.elastic_limit		= 16,
		.hssync_start_delay	= 0,
		.idle_wait_delay	= 17,
		.term_range_adj		= 6,
		.xcvr_lsfslew		= 2,
		.xcvr_lsrslew		= 2,
		.xcvr_setup		= 15,
		.xcvr_setup_offset	= 0,
		.xcvr_use_fuses		= 1,
	},
	.u_data.host = {
		.hot_plug			= true,
		.power_off_on_suspend		= true,
		.remote_wakeup_supported	= true,
		.vbus_gpio			= TEGRA_GPIO_PDD6,
		.vbus_reg			= NULL,
	},
};

#ifndef CONFIG_USB_TEGRA_OTG
static struct platform_device *tegra_usb_otg_host_register(void)
{
	struct platform_device *pdev;
	void *platform_data;
	int val;

	pdev = platform_device_alloc(tegra_ehci1_device.name,
				     tegra_ehci1_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci1_device.resource,
					    tegra_ehci1_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask = tegra_ehci1_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci1_device.dev.coherent_dma_mask;

	platform_data = kmalloc(sizeof(struct tegra_usb_platform_data),
				GFP_KERNEL);
	if (!platform_data)
		goto error;

	memcpy(platform_data, &tegra_ehci1_utmi_pdata,
	       sizeof(struct tegra_usb_platform_data));
	pdev->dev.platform_data = platform_data;

	val = platform_device_add(pdev);
	if (val)
		goto error_add;

	return pdev;

error_add:
	kfree(platform_data);
error:
	pr_err("%s: failed to add the host controller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

static void tegra_usb_otg_host_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}

static struct colibri_otg_platform_data colibri_otg_pdata = {
	.cable_detect_gpio	= USBC_DET,
	.host_register		= &tegra_usb_otg_host_register,
	.host_unregister	= &tegra_usb_otg_host_unregister,
};
#else /* !CONFIG_USB_TEGRA_OTG */
static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};
#endif /* !CONFIG_USB_TEGRA_OTG */

#ifndef CONFIG_USB_TEGRA_OTG
struct platform_device colibri_otg_device = {
	.name	= "colibri-otg",
	.id	= -1,
	.dev = {
		.platform_data = &colibri_otg_pdata,
	},
};
#endif /* !CONFIG_USB_TEGRA_OTG */

static void colibri_t30_usb_init(void)
{
	//gpio_request(LAN_V_BUS, "LAN_V_BUS");
	//gpio_direction_output(LAN_V_BUS, 0);
	//gpio_export(LAN_V_BUS, false);

	//gpio_request(LAN_RESET, "LAN_RESET");
	//gpio_direction_output(LAN_RESET, 0);
	//gpio_export(LAN_RESET, false);

	/* OTG should be the first to be registered
	   EHCI instance 0: USB1_DP/N -> USBC_P/N */
#ifndef CONFIG_USB_TEGRA_OTG
	platform_device_register(&colibri_otg_device);
#else /* !CONFIG_USB_TEGRA_OTG */
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);
#endif /* !CONFIG_USB_TEGRA_OTG */

	/* setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
	platform_device_register(&tegra_udc_device);

}

static struct platform_device *colibri_t30_devices[] __initdata = {
	&tegra_pmu_device,
#if defined(CONFIG_RTC_DRV_TEGRA)
	&tegra_rtc_device,
#endif
#if defined(CONFIG_TEGRA_IOVMM_SMMU) || defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt0_device,
//	&tegra_wdt1_device,
//	&tegra_wdt2_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#ifdef CONFIG_TEGRA_CAMERA
	&tegra_camera,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device2,
	&tegra_i2s_device0,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&tegra_pcm_device,
        &cardhu_audio_wm8962_device,

	&tegra_hda_device,
	&tegra_cec_device,

//	&tegra_led_pwm_device,
	&tegra_pwfm1_device,
	&tegra_pwfm2_device,
	&tegra_pwfm3_device,

};

static void __init colibri_t30_init(void)
{
	
	tegra_clk_init_from_table(colibri_t30_clk_init_table);
//	colibri_t30_pinmux_init();

	colibri_t30_i2c_init();
	colibri_t30_spi_init();
	colibri_t30_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	colibri_t30_edp_init();
#endif
	colibri_t30_uart_init();
#ifdef CONFIG_W1_MASTER_TEGRA
	tegra_w1_device.dev.platform_data = &colibri_t30_w1_platform_data;
#endif
	platform_add_devices(colibri_t30_devices, ARRAY_SIZE(colibri_t30_devices));
	tegra_ram_console_debug_init();

	tegra_io_dpd_init();

	colibri_t30_sdhci_init();
	colibri_t30_regulator_init();
	colibri_t30_suspend_init();
	colibri_t30_panel_init();
//	colibri_t30_sensors_init();
	colibri_t30_emc_init();
//	colibri_t30_register_spidev();

#ifdef CONFIG_BATTERY_CHARGER_SURFACE_RT
	surface_rt_ec_init();
#endif
	surface_rt_sound_init();
	surface_rt_keys_init();
	tegra_release_bootloader_fb();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);

	surface_rt_i2c_hid_init();
}

static void __init colibri_t30_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* Support 1920X1080 32bpp,double buffered on HDMI*/
	tegra_reserve(0, SZ_8M + SZ_1M, SZ_16M);
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
	tegra_ram_console_debug_reserve(SZ_1M);
}

static const char *colibri_t30_dt_board_compat[] = {
	"toradex,colibri_t30",
	NULL
};

#ifdef CONFIG_ANDROID
MACHINE_START(COLIBRI_T30, "cardhu")
#else
MACHINE_START(COLIBRI_T30, "Toradex Colibri T30")
#endif
	.boot_params	= 0x80000100,
	.dt_compat	= colibri_t30_dt_board_compat,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.init_machine	= colibri_t30_init,
	.map_io		= tegra_map_common_io,
	.reserve	= colibri_t30_reserve,
	.timer		= &tegra_timer,
MACHINE_END
