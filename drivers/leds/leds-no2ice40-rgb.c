// SPDX-License-Identifier: GPL-2.0-only
/*
 * LEDs driver for the NitroFPGA wrapper found in no2ice40 for the
 * SB_LEDDA_IP block found in Lattice UltraPlus.
 *
 * Copyright (C) 2020 Sylvain Munaut <tnt@246tNt.com>
 */

#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>


struct ice40_ledda_ip {
	uint32_t _rsvd0;
	union {
		struct {
			uint32_t pwrr;	/* 0001 LEDDPWRR - Pulse Width Register Red   */
			uint32_t pwrg;	/* 0010 LEDDPWRG - Pulse Width Register Green */
			uint32_t pwrb;	/* 0011 LEDDPWRB - Pulse Width Register Blue  */
		};
		uint32_t pwr[3];
	};
	uint32_t _rsvd1;
	uint32_t bcrr;			/* 0101 LEDDBCRR - Breathe Control Rise Register */
	uint32_t bcfr;			/* 0101 LEDDBCFR - Breathe Control Fall Register */
	uint32_t _rsvd2;
	uint32_t cr0;			/* 1000 LEDDCR0  - Control Register 0 */
	uint32_t br;			/* 1001 LEDDBR   - Pre-scale Register */
	uint32_t onr;			/* 1010 LEDONR   - ON  Time Register */
	uint32_t ofr;			/* 1011 LEDOFR   - OFF Time Register */
} __attribute__((packed,aligned(4)));

#define LEDDA_IP_CR0_LEDDEN		(1 << 7)
#define LEDDA_IP_CR0_FR250		(1 << 6)
#define LEDDA_IP_CR0_OUTPOL		(1 << 5)
#define LEDDA_IP_CR0_OUTSKEW		(1 << 4)
#define LEDDA_IP_CR0_QUICK_STOP		(1 << 3)
#define LEDDA_IP_CR0_PWM_LINEAR		(0 << 2)
#define LEDDA_IP_CR0_PWM_LFSR		(1 << 2)
#define LEDDA_IP_CR0_SCALE_MSB(x)	(((x) >> 8) & 3)

#define LEDDA_IP_BR_SCALE_LSB(x)	((x) & 0xff)

#define LEDDA_IP_ONOFF_TIME_MS(x)	(((x) >> 5) & 0xff)	/*  32ms interval up to 8s */

#define LEDDA_IP_BREATHE_ENABLE		(1 << 7)
#define LEDDA_IP_BREATHE_MODULATE	(1 << 5)
#define LEDDA_IP_BREATHE_TIME_MS(x)	(((x) >> 7) & 0x0f)	/* 128ms interval up to 2s */


struct no2ice40_rgb_regs {
	uint32_t csr;
	uint32_t _rsvd[15];
	struct ice40_ledda_ip ip;
} __attribute__((packed,aligned(4)));

#define LED_CSR_LEDDEXE		(1 << 1)
#define LED_CSR_RGBLEDEN	(1 << 2)
#define LED_CSR_CURREN		(1 << 3)


struct no2ice40_rgb;

struct no2ice40_rgb_led {
	struct led_classdev cdev;
	struct no2ice40_rgb *ctrl;
	bool enabled;
	bool blinking;
	int  led_num;
};

struct no2ice40_rgb {
	volatile struct no2ice40_rgb_regs __iomem *regs;
	uint32_t freq;
	uint32_t cr0_base;
	struct no2ice40_rgb_led leds[3];
};


static int no2ice40_hw_init(struct no2ice40_rgb *ctrl)
{
	int div;

	/* Clear hw */
	ctrl->regs->csr = 0;

	ctrl->regs->ip.pwrr = 0;
	ctrl->regs->ip.pwrg = 0;
	ctrl->regs->ip.pwrb = 0;

	ctrl->regs->ip.bcrr = 0;
	ctrl->regs->ip.bcfr = 0;

	ctrl->regs->ip.onr  = 0;
	ctrl->regs->ip.ofr  = 0;

	ctrl->regs->ip.br   = 0;
	ctrl->regs->ip.cr0  = 0;

	/* Divider */
	div = (ctrl->freq / 64e3) - 1;
	if ((div <= 0) || (div >= 1024))
		return -EINVAL;

	/* Base config */
	ctrl->cr0_base =
		LEDDA_IP_CR0_LEDDEN |
		LEDDA_IP_CR0_FR250 |
		LEDDA_IP_CR0_OUTSKEW |
		LEDDA_IP_CR0_QUICK_STOP |
		LEDDA_IP_CR0_PWM_LFSR |
		LEDDA_IP_CR0_SCALE_MSB(div);

	ctrl->regs->ip.br  = LEDDA_IP_BR_SCALE_LSB(div);
	ctrl->regs->ip.cr0 = ctrl->cr0_base;

	/* Enable */
	ctrl->regs->csr = LED_CSR_LEDDEXE | LED_CSR_RGBLEDEN | LED_CSR_CURREN;

	return 0;
}


static void no2ice40_rgb_set(struct led_classdev *led_cdev,
                             enum led_brightness brightness)
{
	struct no2ice40_rgb_led *led = container_of(led_cdev, struct no2ice40_rgb_led, cdev);
	struct no2ice40_rgb *ctrl = led->ctrl;

	/* Update hw brightness */
	ctrl->regs->ip.pwr[led->led_num] = brightness;

	/* Should we disable blink ? */
	if (led->blinking && !brightness)
	{
		bool need_blink = false;
		int i;

		/* This led no longer needs blink */
		led->blinking = false;

		/* Any led left ? */
		for (i=0; i<3; i++) {
			if (ctrl->leds[i].blinking)
				need_blink = true;
		}

		/* If not, disable hw blink */
		if (!need_blink) {
			ctrl->regs->csr = LED_CSR_RGBLEDEN | LED_CSR_CURREN;

			ctrl->regs->ip.onr = 0;
			ctrl->regs->ip.ofr = 0;

			ctrl->regs->csr = LED_CSR_LEDDEXE | LED_CSR_RGBLEDEN | LED_CSR_CURREN;
		}
	}
}

static int no2ice40_rgb_blink_set(struct led_classdev *led_cdev,
                                  unsigned long *delay_on,
                                  unsigned long *delay_off)
{
	struct no2ice40_rgb_led *led = container_of(led_cdev, struct no2ice40_rgb_led, cdev);
	struct no2ice40_rgb *ctrl = led->ctrl;
	int time_on, time_off;

	/* Compute times */
	time_on  = *delay_on  >> 5;
	time_off = *delay_off >> 5;

	if ( !time_on || !time_off || (time_on > 255) || (time_off > 255) )
	{
		time_on  = 12;	/* ~ 384 ms */
		time_off = 12;	/* ~ 384 ms */
	}

	*delay_on  = time_on  * 32;
	*delay_off = time_off * 32;

	/* Update hw */
	ctrl->regs->csr = LED_CSR_RGBLEDEN | LED_CSR_CURREN;

	ctrl->regs->ip.onr = time_on;
	ctrl->regs->ip.ofr = time_off;

	ctrl->regs->csr = LED_CSR_LEDDEXE | LED_CSR_RGBLEDEN | LED_CSR_CURREN;

	/* Set this led as requiring blinking */
	led->blinking = true;

	return 0;
}


static int no2ice40_rgb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *child;
	void *mem;
	uint32_t freq;
	bool hw_blink;
	struct no2ice40_rgb *ctrl;
	int res;

	mem = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(mem))
		return PTR_ERR(mem);

	res = device_property_read_u32(dev, "timebase-frequency", &freq);
	if ((res != 0) || (freq == 0))
		return -EINVAL;

	hw_blink = device_property_read_bool(dev, "no2,hw-blink");

	ctrl =  devm_kzalloc(&pdev->dev, sizeof(struct no2ice40_rgb), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, ctrl);

	ctrl->regs = mem;
	ctrl->freq = freq;
	no2ice40_hw_init(ctrl);

	device_for_each_child_node(dev, child)
	{
		struct led_init_data init_data = {};
		struct no2ice40_rgb_led *led;
		u32 reg;
		u32 brightness;

		res = fwnode_property_read_u32(child, "reg", &reg);
		if ((res != 0) || (reg >= 3))
			continue;

		led = &ctrl->leds[reg];
		led->ctrl = ctrl;
		led->enabled = true;
		led->led_num = reg;

		fwnode_property_read_u32(child, "brightness",       &led->cdev.brightness);
		fwnode_property_read_u32(child, "max-brightness",   &led->cdev.max_brightness);
		fwnode_property_read_u32(child, "blink-brightness", &led->cdev.blink_brightness);

		led->cdev.brightness_set = no2ice40_rgb_set;
		led->cdev.blink_set = hw_blink ? no2ice40_rgb_blink_set : NULL;

		init_data.fwnode = child;

		res = devm_led_classdev_register_ext(dev, &led->cdev, &init_data);
		if (res)
			dev_err(dev, "Failed to register LED for node %pfw\n", child);
	}

	return 0;
}


static const struct of_device_id no2ice40_rgb_of_match[] = {
    { .compatible = "no2fpga,no2ice40-rgb" },
    {}
};
MODULE_DEVICE_TABLE(of, no2ice40_rgb_of_match);

static struct platform_driver no2ice40_rgb_driver = {
	.probe = no2ice40_rgb_probe,
	.driver = {
		.name = "no2ice40-rgb",
		.of_match_table = no2ice40_rgb_of_match,
	},
};

module_platform_driver(no2ice40_rgb_driver);

MODULE_AUTHOR("Sylvain Munaut <tnt@246tNt.com>");
MODULE_DESCRIPTION("User LED support for iCE40 UP5k SB_LEDDA_IP wrapper from no2ice40");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:no2ice40-rgb");
