// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Spreadtrum SC2720 USB Type-C
 *
 * Copyright (C) 2020 Spreadtrum Communications Inc.
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/usb/typec.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/extcon.h>
#include <linux/kernel.h>
#include <linux/nvmem-consumer.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/extcon-provider.h>

/* registers definitions for controller REGS_TYPEC */
#define SC2720_EN			0x00
#define SC2720_MODE			0x04
#define SC2720_INT_EN			0x0c
#define SC2720_INT_CLR			0x10
#define SC2720_INT_RAW			0x14
#define SC2720_INT_MASK			0x18
#define SC2720_STATUS			0x1c
#define SC2720_TCCDE_CNT		0x20
#define SC2720_RTRIM			0x3c

/* SC2720_TYPEC_EN */
#define SC2720_TYPEC_USB20_ONLY		BIT(4)

/* SC2720_TYPEC MODE */
#define SC2720_MODE_SNK			0
#define SC2720_MODE_SRC			1
#define SC2720_MODE_DRP			2
#define SC2720_MODE_MASK		3

/* SC2720_INT_EN */
#define SC2720_ATTACH_INT_EN		BIT(0)
#define SC2720_DETACH_INT_EN		BIT(1)

/* SC2720_INT_CLR */
#define SC2720_ATTACH_INT_CLR		BIT(0)
#define SC2720_DETACH_INT_CLR		BIT(1)

/* SC2720_INT_MASK */
#define SC2720_ATTACH_INT		BIT(0)
#define SC2720_DETACH_INT		BIT(1)

#define SC2720_STATE_MASK		GENMASK(4, 0)
#define SC2720_EVENT_MASK		GENMASK(6, 0)

#define SC2720_EFUSE_CC1_SHIFT		11
#define SC2720_EFUSE_CC2_SHIFT		6

#define SC2720_CC1_MASK(n)		GENMASK((n) + 9, (n) + 5)
#define SC2720_CC2_MASK(n)		GENMASK((n) + 4, (n))
#define SC2720_CC_SHIFT(n)		(n)

/* modify sc2720 tcc debunce */
#define SC2720_TCC_DEBOUNCE_CNT		0xc7f

/* sc2720 registers definitions for controller REGS_TYPEC */
#define SC2720_TYPEC_PD_CFG		0x08
/* SC2720_TYPEC_PD_CFG */
#define SC2720_VCONN_LDO_EN		BIT(13)
#define SC2720_VCONN_LDO_RDY		BIT(12)

static struct wakeup_source *typec_wakelock;
enum SC2720_typec_connection_state {
	SC2720_DETACHED_SNK,
	SC2720_ATTACHWAIT_SNK,
	SC2720_ATTACHED_SNK,
	SC2720_DETACHED_SRC,
	SC2720_ATTACHWAIT_SRC,
	SC2720_ATTACHED_SRC,
	SC2720_POWERED_CABLE,
	SC2720_AUDIO_CABLE,
	SC2720_DEBUG_CABLE,
	SC2720_TOGGLE_SLEEP,
	SC2720_ERR_RECOV,
	SC2720_DISABLED,
	SC2720_TRY_SNK,
	SC2720_TRY_WAIT_SRC,
	SC2720_TRY_SRC,
	SC2720_TRY_WAIT_SNK,
	SC2720_UNSUPOORT_ACC,
	SC2720_ORIENTED_DEBUG,
};

struct sprd_typec_variant_data {
	u32 efuse_cc1_shift;
	u32 efuse_cc2_shift;
	u32 int_en;
	u32 int_clr;
	u32 mode;
	u32 attach_en;
	u32 detach_en;
	u32 state_mask;
	u32 event_mask;
};

static const struct sprd_typec_variant_data sc2720_data = {
	.efuse_cc1_shift = SC2720_EFUSE_CC1_SHIFT,
	.efuse_cc2_shift = SC2720_EFUSE_CC2_SHIFT,
	.int_en = SC2720_INT_EN,
	.int_clr = SC2720_INT_CLR,
	.mode = SC2720_MODE,
	.attach_en = SC2720_ATTACH_INT_EN,
	.detach_en = SC2720_DETACH_INT_EN,
	.state_mask = SC2720_STATE_MASK,
	.event_mask = SC2720_EVENT_MASK,
};


struct SC2720_typec {
	struct device *dev;
	struct regmap *regmap;
	u32 base;
	int irq;
	struct extcon_dev *edev;
	struct gpio_desc *gpiod;
	bool usb20_only;

	enum SC2720_typec_connection_state state;
	enum SC2720_typec_connection_state pre_state;
	struct typec_port *port;
	struct typec_partner *partner;
	struct typec_capability typec_cap;
	const struct sprd_typec_variant_data *var_data;
};

static int SC2720_typec_connect(struct SC2720_typec *sc, u32 status)
{
	enum typec_data_role data_role = TYPEC_DEVICE;
	enum typec_role power_role = TYPEC_SOURCE;
	enum typec_role vconn_role = TYPEC_SOURCE;
	struct typec_partner_desc desc;

	if (sc->partner)
		return 0;

	switch (sc->state) {
	case SC2720_ATTACHED_SNK:
	case SC2720_DEBUG_CABLE:
		power_role = TYPEC_SINK;
		data_role = TYPEC_DEVICE;
		vconn_role = TYPEC_SINK;
		break;
	case SC2720_ATTACHED_SRC:
	case SC2720_AUDIO_CABLE:
		power_role = TYPEC_SOURCE;
		data_role = TYPEC_HOST;
		vconn_role = TYPEC_SOURCE;
		break;
	default:
		break;
	}

	desc.usb_pd = 0;
	desc.accessory = TYPEC_ACCESSORY_NONE;
	desc.identity = NULL;

	sc->partner = typec_register_partner(sc->port, &desc);
	if (!sc->partner)
		return -ENODEV;

	typec_set_pwr_opmode(sc->port, TYPEC_PWR_MODE_USB);
	typec_set_pwr_role(sc->port, power_role);
	typec_set_data_role(sc->port, data_role);
	typec_set_vconn_role(sc->port, vconn_role);

	switch (sc->state) {
	case SC2720_ATTACHED_SNK:
	case SC2720_DEBUG_CABLE:
		sc->pre_state = SC2720_ATTACHED_SNK;
		extcon_set_state_sync(sc->edev, EXTCON_USB, true);
		break;
	case SC2720_ATTACHED_SRC:
		sc->pre_state = SC2720_ATTACHED_SRC;
#ifdef PRJ_FEATURE_H_BOARD_DOCKING_SUPPORT
#else
		if (!IS_ERR(sc->gpiod) && gpiod_get_value(sc->gpiod))
			return 0;
#endif
		extcon_set_state_sync(sc->edev, EXTCON_USB_HOST, true);
		break;
	case SC2720_AUDIO_CABLE:
		sc->pre_state = SC2720_AUDIO_CABLE;
		extcon_set_state_sync(sc->edev, EXTCON_JACK_HEADPHONE, true);
		break;
	default:
		break;
	}

	return 0;
}

static void SC2720_typec_disconnect(struct SC2720_typec *sc, u32 status)
{
	typec_unregister_partner(sc->partner);
	sc->partner = NULL;
	typec_set_pwr_opmode(sc->port, TYPEC_PWR_MODE_USB);
	typec_set_pwr_role(sc->port, TYPEC_SINK);
	typec_set_data_role(sc->port, TYPEC_DEVICE);
	typec_set_vconn_role(sc->port, TYPEC_SINK);

	switch (sc->pre_state) {
	case SC2720_ATTACHED_SNK:
	case SC2720_DEBUG_CABLE:
		extcon_set_state_sync(sc->edev, EXTCON_USB, false);
		break;
	case SC2720_ATTACHED_SRC:
		extcon_set_state_sync(sc->edev, EXTCON_USB_HOST, false);
		break;
	case SC2720_AUDIO_CABLE:
		extcon_set_state_sync(sc->edev, EXTCON_JACK_HEADPHONE, false);
		break;
	default:
		break;
	}
}

#if 0
static int SC2720_typec_dr_set(const struct typec_capability *cap,
				enum typec_data_role role)
{
	/* TODO: Data role set */
	return 0;
}

static int SC2720_typec_pr_set(const struct typec_capability *cap,
				enum typec_role role)
{
	/* TODO: Power role set */
	return 0;
}

static int SC2720_typec_vconn_set(const struct typec_capability *cap,
				enum typec_role role)
{
	/* TODO: Vconn set */
	return 0;
}
#endif

static irqreturn_t SC2720_typec_interrupt(int irq, void *data)
{
	struct SC2720_typec *sc = data;
	u32 event;
	int ret;

	ret = regmap_read(sc->regmap, sc->base + SC2720_INT_MASK, &event);
	if (ret)
		return ret;

	event &= sc->var_data->event_mask;

	ret = regmap_read(sc->regmap, sc->base + SC2720_STATUS, &sc->state);
	if (ret)
		goto clear_ints;

	sc->state &= sc->var_data->state_mask;

	if (event & SC2720_ATTACH_INT) {
		ret = SC2720_typec_connect(sc, sc->state);
		if (ret)
			dev_warn(sc->dev, "failed to register partner\n");
	} else if (event & SC2720_DETACH_INT) {
		SC2720_typec_disconnect(sc, sc->state);
	}

clear_ints:
	regmap_write(sc->regmap, sc->base + sc->var_data->int_clr, event);

	dev_info(sc->dev, "now works as DRP and is in %d state, event %d\n",
		sc->state, event);
	return IRQ_HANDLED;
}

static int SC2720_typec_enable(struct SC2720_typec *sc)
{
	int ret;
	u32 val;

	/* Set typec mode */
	ret = regmap_read(sc->regmap, sc->base + sc->var_data->mode, &val);
	if (ret)
		return ret;

	val &= ~SC2720_MODE_MASK;
	switch (sc->typec_cap.type) {
	case TYPEC_PORT_DFP:
		val |= SC2720_MODE_SRC;
		break;
	case TYPEC_PORT_UFP:
		val |= SC2720_MODE_SNK;
		break;
	case TYPEC_PORT_DRP:
		val |= SC2720_MODE_DRP;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_write(sc->regmap, sc->base + sc->var_data->mode, val);
	if (ret)
		return ret;

	/* typec USB20 only flag, only work in snk mode */
	if (sc->typec_cap.data == TYPEC_PORT_UFP && sc->usb20_only) {
		ret = regmap_update_bits(sc->regmap, sc->base + SC2720_EN,
					 SC2720_TYPEC_USB20_ONLY,
					 SC2720_TYPEC_USB20_ONLY);
		if (ret)
			return ret;
	}

	/* modify sc2720 tcc debounce to 100ms while PD signal occur at 150ms
	 * and effect tccde reginize.Reason is hardware signal and clk not
	 * accurate.
	 */
	if (sc->var_data->efuse_cc2_shift == SC2720_EFUSE_CC2_SHIFT) {
		ret = regmap_write(sc->regmap, sc->base + SC2720_TCCDE_CNT,
				SC2720_TCC_DEBOUNCE_CNT);
		if (ret)
			return ret;
	}

	/* Enable typec interrupt and enable typec */
	ret = regmap_read(sc->regmap, sc->base + sc->var_data->int_en, &val);
	if (ret)
		return ret;

	val |= sc->var_data->attach_en | sc->var_data->detach_en;
	return regmap_write(sc->regmap, sc->base + sc->var_data->int_en, val);
}

static const u32 SC2720_typec_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_JACK_HEADPHONE,
	EXTCON_NONE,
};

static int SC2720_typec_get_cc1_efuse(struct SC2720_typec *sc)
{
	struct nvmem_cell *cell;
	u32 calib_data = 0;
	void *buf;
	size_t len;

	cell = nvmem_cell_get(sc->dev, "typec_cc1_cal");
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);

	if (IS_ERR(buf))
		return PTR_ERR(buf);

	memcpy(&calib_data, buf, min(len, sizeof(u32)));
	calib_data = (calib_data & SC2720_CC1_MASK(sc->var_data->efuse_cc1_shift))
			>> SC2720_CC_SHIFT(sc->var_data->efuse_cc1_shift);
	kfree(buf);

	return calib_data;
}

static int SC2720_typec_get_cc2_efuse(struct SC2720_typec *sc)
{
	struct nvmem_cell *cell;
	u32 calib_data = 0;
	void *buf;
	size_t len = 0;

	cell = nvmem_cell_get(sc->dev, "typec_cc2_cal");
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);

	if (IS_ERR(buf))
		return PTR_ERR(buf);

	memcpy(&calib_data, buf, min(len, sizeof(u32)));
	calib_data = (calib_data & SC2720_CC2_MASK(sc->var_data->efuse_cc2_shift))
			>> SC2720_CC_SHIFT(sc->var_data->efuse_cc2_shift);
	kfree(buf);

	return calib_data;
}

static int typec_set_rtrim(struct SC2720_typec *sc)
{
	int calib_cc1;
	int calib_cc2;
	u32 rtrim;

	calib_cc1 = SC2720_typec_get_cc1_efuse(sc);
	if (calib_cc1 < 0)
		return calib_cc1;
	calib_cc2 = SC2720_typec_get_cc2_efuse(sc);
	if (calib_cc2 < 0)
		return calib_cc2;

	rtrim = calib_cc1 | calib_cc2<<5;

	return regmap_write(sc->regmap, sc->base + SC2720_RTRIM, rtrim);
}

static int SC2720_typec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;
	struct SC2720_typec *sc;
	const struct sprd_typec_variant_data *pdata;
	int mode, ret, gpio_num;

	pdata = of_device_get_match_data(dev);
	if (!pdata) {
		dev_err(&pdev->dev, "No matching driver data found\n");
		return -EINVAL;
	}

	sc = devm_kzalloc(&pdev->dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->edev = devm_extcon_dev_allocate(&pdev->dev, SC2720_typec_cable);
	if (IS_ERR(sc->edev)) {
		dev_err(&pdev->dev, "failed to allocate extcon device\n");
		return PTR_ERR(sc->edev);
	}

	ret = devm_extcon_dev_register(&pdev->dev, sc->edev);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't register extcon device: %d\n", ret);
		return ret;
	}

	sc->dev = &pdev->dev;
	sc->irq = platform_get_irq(pdev, 0);
	if (sc->irq < 0) {
		dev_err(sc->dev, "failed to get typec interrupt.\n");
		return sc->irq;
	}

	sc->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!sc->regmap) {
		dev_err(sc->dev, "failed to get regmap.\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(node, "reg", &sc->base);
	if (ret) {
		dev_err(dev, "failed to get reg offset!\n");
		return ret;
	}

	ret = of_property_read_u32(node, "sprd,mode", &mode);
	if (ret) {
		dev_err(dev, "failed to get typec port mode type\n");
		return ret;
	}

	if (mode < TYPEC_PORT_DFP || mode > TYPEC_PORT_DRP
	    || mode == TYPEC_PORT_UFP) {
		mode = TYPEC_PORT_UFP;
		sc->usb20_only = true;
		dev_info(dev, "usb 2.0 only is enabled\n");
	}

	node = of_find_compatible_node(NULL, NULL, "linux,extcon-usb-gpio");
	if (!node) {
		dev_warn(dev, "failed to find vbus gpio node.\n");
	} else {
		gpio_num = of_get_named_gpio(node, "vbus-gpio", 0);
		of_node_put(node);
		if (gpio_is_valid(gpio_num)) {
			sc->gpiod = gpio_to_desc(gpio_num);
			if (IS_ERR(sc->gpiod))
				dev_warn(dev, "failed to get vbus gpio.\n");
		} else {
			dev_warn(dev, "failed to get valid vbus gpio.\n");
		}
	}

	sc->var_data = pdata;
	sc->typec_cap.type = mode;
	sc->typec_cap.data = TYPEC_PORT_DRD;
	//sc->typec_cap.dr_set = SC2720_typec_dr_set;
	//sc->typec_cap.pr_set = SC2720_typec_pr_set;
	//sc->typec_cap.vconn_set = SC2720_typec_vconn_set;
	sc->typec_cap.revision = USB_TYPEC_REV_1_2;
	sc->typec_cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;
	sc->port = typec_register_port(&pdev->dev, &sc->typec_cap);
	if (!sc->port) {
		dev_err(sc->dev, "failed to register port!\n");
		return -ENODEV;
	}

	ret = typec_set_rtrim(sc);
	if (ret < 0) {
		dev_err(sc->dev, "failed to set typec rtrim %d\n", ret);
		goto error;
	}

	ret = devm_request_threaded_irq(sc->dev, sc->irq, NULL,
					SC2720_typec_interrupt,
					IRQF_EARLY_RESUME | IRQF_ONESHOT,
					dev_name(sc->dev), sc);
	if (ret) {
		dev_err(sc->dev, "failed to request irq %d\n", ret);
		goto error;
	}

	ret = SC2720_typec_enable(sc);
	if (ret)
		goto error;

	typec_wakelock = wakeup_source_create("sc27xx_typec_wakelock");
	wakeup_source_add(typec_wakelock);

	platform_set_drvdata(pdev, sc);
	return 0;

error:
	typec_unregister_port(sc->port);
	return ret;
}

static int SC2720_typec_remove(struct platform_device *pdev)
{
	struct SC2720_typec *sc = platform_get_drvdata(pdev);

	wakeup_source_remove(typec_wakelock);
	SC2720_typec_disconnect(sc, 0);
	typec_unregister_port(sc->port);

	return 0;
}

static const struct of_device_id typec_sprd_match[] = {
	{.compatible = "sprd,sc2720-typec", .data = &sc2720_data},
	{},
};
MODULE_DEVICE_TABLE(of, typec_sprd_match);

static struct platform_driver SC2720_typec_driver = {
	.probe = SC2720_typec_probe,
	.remove = SC2720_typec_remove,
	.driver = {
		.name = "SC2720-typec",
		.of_match_table = typec_sprd_match,
	},
};
module_platform_driver(SC2720_typec_driver);
MODULE_LICENSE("GPL v2");

