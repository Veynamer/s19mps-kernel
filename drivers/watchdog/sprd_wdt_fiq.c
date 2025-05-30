/*
 * Spreadtrum watchdog driver
 * Copyright (C) 2017 Spreadtrum - http://www.spreadtrum.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/alarmtimer.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/syscore_ops.h>

#define SPRD_WDT_FIQ_LOAD_LOW		0x0
#define SPRD_WDT_FIQ_LOAD_HIGH		0x4
#define SPRD_WDT_FIQ_CTRL			0x8
#define SPRD_WDT_FIQ_INT_CLR		0xc
#define SPRD_WDT_FIQ_INT_RAW		0x10
#define SPRD_WDT_FIQ_INT_MSK		0x14
#define SPRD_WDT_FIQ_CNT_LOW		0x18
#define SPRD_WDT_FIQ_CNT_HIGH		0x1c
#define SPRD_WDT_FIQ_LOCK			0x20
#define SPRD_WDT_FIQ_IRQ_LOAD_LOW		0x2c
#define SPRD_WDT_FIQ_IRQ_LOAD_HIGH		0x30

/* WDT_CTRL */
#define SPRD_WDT_FIQ_INT_EN_BIT		BIT(0)
#define SPRD_WDT_FIQ_CNT_EN_BIT		BIT(1)
#define SPRD_WDT_FIQ_NEW_VER_EN		BIT(2)
#define SPRD_WDT_FIQ_RST_EN_BIT		BIT(3)

/* WDT_INT_CLR */
#define SPRD_WDT_FIQ_INT_CLEAR_BIT		BIT(0)
#define SPRD_WDT_FIQ_RST_CLEAR_BIT		BIT(3)

/* WDT_INT_RAW */
#define SPRD_WDT_FIQ_INT_RAW_BIT		BIT(0)
#define SPRD_WDT_FIQ_RST_RAW_BIT		BIT(3)
#define SPRD_WDT_FIQ_LD_BUSY_BIT		BIT(4)

/* 1s equal to 32768 counter steps */
#define SPRD_WDT_FIQ_CNT_STEP		32768

#define SPRD_WDT_FIQ_UNLOCK_KEY		0xe551
#define SPRD_WDT_FIQ_MIN_TIMEOUT		3
#define SPRD_WDT_FIQ_MAX_TIMEOUT		60

#define SPRD_WDT_FIQ_CNT_HIGH_SHIFT		16
#define SPRD_WDT_FIQ_LOW_VALUE_MASK		GENMASK(15, 0)
#define SPRD_WDT_FIQ_LOAD_TIMEOUT		1000

#define SPRD_DSWDTEN_MAGIC "enabled"
#define SPRD_DSWDTEN_MAGIC_LEN_MAX  10

#define SPRD_WDT_SLEEP_KICKTIME		540
#define SPRD_WDT_SLEEP_PRETIMEOUT	(600-570)
#define SPRD_WDT_SLEEP_TIMEOUT		600

#define SPRD_WDT_SYSCORE_SUSPEND_RESUME

struct sprd_wdt_fiq {
	void __iomem *base;
	struct watchdog_device wdd;
	struct clk *enable;
	struct clk *rtc_enable;
	struct mutex *lock;
	struct alarm sleep_tmr;
	bool sleep_en;
	u32 wdt_ctrl;
	u64 wdt_load;
};

static DEFINE_MUTEX(sprd_wdt_mutex);
struct sprd_wdt_fiq *wdt_fiq;

static bool sprd_dswdt_fiq_en(void)
{
	struct device_node *cmdline_node;
	const char *cmd_line, *dswdten_name_p;
	char dswdten_value[SPRD_DSWDTEN_MAGIC_LEN_MAX] = "NULL";
	int ret;

	cmdline_node = of_find_node_by_path("/chosen");
	ret = of_property_read_string(cmdline_node, "bootargs", &cmd_line);

	if (ret) {
		pr_err("can't not parse bootargs property\n");
		return false;
	}

	dswdten_name_p = strstr(cmd_line, "androidboot.dswdten=");
	if (!dswdten_name_p) {
		pr_err("can't find androidboot.dswdten\n");
		return false;
	}

	sscanf(dswdten_name_p, "androidboot.dswdten=%8s", dswdten_value);
	if (strncmp(dswdten_value, SPRD_DSWDTEN_MAGIC, strlen(SPRD_DSWDTEN_MAGIC)))
		return false;

	return true;
}

static inline struct sprd_wdt_fiq *to_sprd_wdt_fiq(struct watchdog_device *wdd)
{
	return container_of(wdd, struct sprd_wdt_fiq, wdd);
}

static inline void sprd_wdt_fiq_lock(struct sprd_wdt_fiq *wdt)
{
	void __iomem *addr = wdt->base;

	writel_relaxed(0x0, addr + SPRD_WDT_FIQ_LOCK);
	mutex_unlock(wdt->lock);
}

static inline void sprd_wdt_fiq_unlock(struct sprd_wdt_fiq *wdt)
{
	void __iomem *addr = wdt->base;

	mutex_lock(wdt->lock);
	writel_relaxed(SPRD_WDT_FIQ_UNLOCK_KEY, addr + SPRD_WDT_FIQ_LOCK);
}

static u32 sprd_wdt_fiq_get_cnt_value(struct sprd_wdt_fiq *wdt)
{
	u32 val;

	val = readl_relaxed(wdt->base + SPRD_WDT_FIQ_CNT_HIGH) <<
		SPRD_WDT_FIQ_CNT_HIGH_SHIFT;
	val |= readl_relaxed(wdt->base + SPRD_WDT_FIQ_CNT_LOW) &
		SPRD_WDT_FIQ_LOW_VALUE_MASK;

	return val;
}

static int sprd_wdt_fiq_load_value(struct sprd_wdt_fiq *wdt, u32 timeout,
			       u32 pretimeout)
{
	u32 val, delay_cnt = 0;
	u32 tmr_step = timeout * SPRD_WDT_FIQ_CNT_STEP;
	u32 prtmr_step = pretimeout * SPRD_WDT_FIQ_CNT_STEP;

	pr_debug("sprd_wdt_fiq: sprd wdt load value timeout =%d, pretimeout =%d\n",
	       timeout, pretimeout);
	wdt->wdt_load = jiffies;
	sprd_wdt_fiq_unlock(wdt);
	writel_relaxed((tmr_step >> SPRD_WDT_FIQ_CNT_HIGH_SHIFT) &
		      SPRD_WDT_FIQ_LOW_VALUE_MASK,
		      wdt->base + SPRD_WDT_FIQ_LOAD_HIGH);
	writel_relaxed((tmr_step & SPRD_WDT_FIQ_LOW_VALUE_MASK),
		       wdt->base + SPRD_WDT_FIQ_LOAD_LOW);
	writel_relaxed((prtmr_step >> SPRD_WDT_FIQ_CNT_HIGH_SHIFT) &
			SPRD_WDT_FIQ_LOW_VALUE_MASK,
		       wdt->base + SPRD_WDT_FIQ_IRQ_LOAD_HIGH);
	writel_relaxed(prtmr_step & SPRD_WDT_FIQ_LOW_VALUE_MASK,
		       wdt->base + SPRD_WDT_FIQ_IRQ_LOAD_LOW);
	sprd_wdt_fiq_lock(wdt);

	/*
	 * Waiting the load value operation done,
	 * it needs two or three RTC clock cycles.
	 */
	do {
		val = readl_relaxed(wdt->base + SPRD_WDT_FIQ_INT_RAW);
		if (!(val & SPRD_WDT_FIQ_LD_BUSY_BIT))
			break;

		cpu_relax();
	} while (delay_cnt++ < SPRD_WDT_FIQ_LOAD_TIMEOUT);

	if (delay_cnt >= SPRD_WDT_FIQ_LOAD_TIMEOUT)
		return -EBUSY;
	return 0;
}

static int sprd_wdt_fiq_enable(struct sprd_wdt_fiq *wdt)
{
	u32 val;
	int ret;

	ret = clk_prepare_enable(wdt->enable);
	if (ret)
		return ret;
	ret = clk_prepare_enable(wdt->rtc_enable);
	if (ret) {
		clk_disable_unprepare(wdt->enable);
		return ret;
	}

	sprd_wdt_fiq_unlock(wdt);
	val = readl_relaxed(wdt->base + SPRD_WDT_FIQ_CTRL);
	val |= SPRD_WDT_FIQ_NEW_VER_EN;
	writel_relaxed(val, wdt->base + SPRD_WDT_FIQ_CTRL);
	sprd_wdt_fiq_lock(wdt);
	return 0;
}

static void sprd_wdt_fiq_disable(void *_data)
{
	struct sprd_wdt_fiq *wdt = _data;

	sprd_wdt_fiq_unlock(wdt);
	writel_relaxed(0x0, wdt->base + SPRD_WDT_FIQ_CTRL);
	sprd_wdt_fiq_lock(wdt);

	clk_disable_unprepare(wdt->rtc_enable);
	clk_disable_unprepare(wdt->enable);
}

static int sprd_wdt_fiq_start(struct watchdog_device *wdd)
{
	struct sprd_wdt_fiq *wdt = to_sprd_wdt_fiq(wdd);
	u32 val;
	int ret;

	ret = sprd_wdt_fiq_load_value(wdt, wdd->timeout, wdd->pretimeout);
	if (ret)
		return ret;

	if (watchdog_active(wdd))
		return 0;

	sprd_wdt_fiq_unlock(wdt);
	val = readl_relaxed(wdt->base + SPRD_WDT_FIQ_CTRL);
	val |= SPRD_WDT_FIQ_CNT_EN_BIT | SPRD_WDT_FIQ_INT_EN_BIT | SPRD_WDT_FIQ_RST_EN_BIT;
	writel_relaxed(val, wdt->base + SPRD_WDT_FIQ_CTRL);
	set_bit(WDOG_HW_RUNNING, &wdd->status);
	set_bit(WDOG_ACTIVE, &wdd->status);
	wdt_fiq->wdt_ctrl = readl_relaxed(wdt->base + SPRD_WDT_FIQ_CTRL);
	sprd_wdt_fiq_lock(wdt);


	return 0;
}

static int sprd_wdt_fiq_stop(struct watchdog_device *wdd)
{
	struct sprd_wdt_fiq *wdt = to_sprd_wdt_fiq(wdd);
	u32 val;

	sprd_wdt_fiq_unlock(wdt);
	val = readl_relaxed(wdt->base + SPRD_WDT_FIQ_CTRL);
	val &= ~(SPRD_WDT_FIQ_CNT_EN_BIT | SPRD_WDT_FIQ_RST_EN_BIT |
		SPRD_WDT_FIQ_INT_EN_BIT);
	writel_relaxed(val, wdt->base + SPRD_WDT_FIQ_CTRL);
	clear_bit(WDOG_ACTIVE, &wdd->status);
	sprd_wdt_fiq_lock(wdt);


	return 0;
}

static int sprd_wdt_fiq_set_timeout(struct watchdog_device *wdd,
				u32 timeout)
{
	struct sprd_wdt_fiq *wdt = to_sprd_wdt_fiq(wdd);

	if (timeout == wdd->timeout)
		return 0;

	wdd->timeout = timeout;

	return sprd_wdt_fiq_load_value(wdt, timeout, wdd->pretimeout);
}

static int sprd_wdt_fiq_set_pretimeout(struct watchdog_device *wdd,
				   u32 new_pretimeout)
{
	struct sprd_wdt_fiq *wdt = to_sprd_wdt_fiq(wdd);

	if (new_pretimeout < wdd->min_timeout)
		return -EINVAL;

	wdd->pretimeout = new_pretimeout;

	return sprd_wdt_fiq_load_value(wdt, wdd->timeout, new_pretimeout);
}

static u32 sprd_wdt_fiq_get_timeleft(struct watchdog_device *wdd)
{
	struct sprd_wdt_fiq *wdt = to_sprd_wdt_fiq(wdd);
	u32 val;

	val = sprd_wdt_fiq_get_cnt_value(wdt);
	val = val / SPRD_WDT_FIQ_CNT_STEP;

	return val;
}

int sprd_wdt_fiq_get_dev(struct watchdog_device **wdd)
{
	int ret = -ENODEV;

	if (wdt_fiq) {
		*wdd = &wdt_fiq->wdd;
		ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL(sprd_wdt_fiq_get_dev);

int sprd_wdt_fiq_syscore_suspend(void)
{
#ifdef SPRD_WDT_SYSCORE_SUSPEND_RESUME
	if (!wdt_fiq)
		return -ENODEV;

	if (wdt_fiq->sleep_en) {
		if (watchdog_active(&wdt_fiq->wdd))
			sprd_wdt_fiq_load_value(wdt_fiq, SPRD_WDT_SLEEP_TIMEOUT, SPRD_WDT_SLEEP_PRETIMEOUT);
	}
#endif
	return 0;
}
EXPORT_SYMBOL(sprd_wdt_fiq_syscore_suspend);

void sprd_wdt_fiq_syscore_resume(void)
{
#ifdef SPRD_WDT_SYSCORE_SUSPEND_RESUME
	int ret;

	if (!wdt_fiq)
		return;

	if (wdt_fiq->sleep_en) {
		if (!watchdog_active(&wdt_fiq->wdd)) {
			ret = sprd_wdt_fiq_enable(wdt_fiq);
			if (ret)
				return;
		}
	} else {
		ret = sprd_wdt_fiq_enable(wdt_fiq);
		if (ret)
			return;
	}

	if (watchdog_active(&wdt_fiq->wdd))
		ret = sprd_wdt_fiq_start(&wdt_fiq->wdd);
#endif
}
EXPORT_SYMBOL(sprd_wdt_fiq_syscore_resume);

static int __maybe_unused sprd_wdt_fiq_alarm_prepare(struct device *dev)
{
	struct sprd_wdt_fiq *wdt = dev_get_drvdata(dev);
	ktime_t now, add;

	if (wdt_fiq->sleep_en) {
		if (watchdog_active(&wdt->wdd)) {
			now = ktime_get_boottime();
			add = ktime_set(SPRD_WDT_SLEEP_KICKTIME, 0);
			alarm_start(&wdt->sleep_tmr, ktime_add(now, add));
			pr_info("sprd_wdt:alarm start end\n");
		}
	}
	return 0;
}

static void __maybe_unused sprd_wdt_fiq_alarm_complete(struct device *dev)
{
	struct sprd_wdt_fiq *wdt = dev_get_drvdata(dev);

	 if (wdt_fiq->sleep_en) {
		if (watchdog_active(&wdt->wdd)) {
			alarm_cancel(&wdt->sleep_tmr);
			pr_info("sprd_wdt:alarm_cancel end\n");
		}
	}
}

static struct syscore_ops sprd_wdt_fiq_syscore_ops = {
	.resume = sprd_wdt_fiq_syscore_resume,
	.suspend = sprd_wdt_fiq_syscore_suspend
};

static const struct watchdog_ops sprd_wdt_fiq_ops = {
	.owner = THIS_MODULE,
	.start = sprd_wdt_fiq_start,
	.stop = sprd_wdt_fiq_stop,
	.set_timeout = sprd_wdt_fiq_set_timeout,
	.set_pretimeout = sprd_wdt_fiq_set_pretimeout,
	.get_timeleft = sprd_wdt_fiq_get_timeleft,
};

static const struct watchdog_info sprd_wdt_fiq_info = {
	.options = WDIOF_SETTIMEOUT |
		   WDIOF_PRETIMEOUT |
		   WDIOF_MAGICCLOSE |
		   WDIOF_KEEPALIVEPING,
	.identity = "Spreadtrum Watchdog Timer",
};

static enum alarmtimer_restart sprd_wdt_sleep_callback(struct alarm *p,
						       ktime_t t)
{
	pr_err("sprd_wdt: sprd wdt sleep callback\n");
	return ALARMTIMER_NORESTART;
}

static int sprd_wdt_fiq_probe(struct platform_device *pdev)
{
	struct resource *wdt_res;
	struct sprd_wdt_fiq *wdt;
	int ret;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->lock = &sprd_wdt_mutex;
	mutex_init(wdt->lock);

	wdt_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	wdt->base = devm_ioremap_resource(&pdev->dev, wdt_res);
	if (IS_ERR(wdt->base)) {
		dev_err(&pdev->dev, "failed to map memory resource\n");
		return PTR_ERR(wdt->base);
	}

	wdt->enable = devm_clk_get(&pdev->dev, "enable");
	if (IS_ERR(wdt->enable)) {
		dev_err(&pdev->dev, "can't get the enable clock\n");
		return PTR_ERR(wdt->enable);
	}

	wdt->rtc_enable = devm_clk_get(&pdev->dev, "rtc_enable");
	if (IS_ERR(wdt->rtc_enable)) {
		dev_err(&pdev->dev, "can't get the rtc enable clock\n");
		return PTR_ERR(wdt->rtc_enable);
	}

	wdt->wdd.info = &sprd_wdt_fiq_info;
	wdt->wdd.ops = &sprd_wdt_fiq_ops;
	wdt->wdd.parent = &pdev->dev;
	wdt->wdd.min_timeout = SPRD_WDT_FIQ_MIN_TIMEOUT;
	wdt->wdd.max_timeout = SPRD_WDT_FIQ_MAX_TIMEOUT;
	wdt->wdd.timeout = SPRD_WDT_FIQ_MAX_TIMEOUT;

	wdt->sleep_en = sprd_dswdt_fiq_en();
	ret = sprd_wdt_fiq_enable(wdt);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable wdt\n");
		return ret;
	}
	ret = devm_add_action(&pdev->dev, sprd_wdt_fiq_disable, wdt);
	if (ret) {
		sprd_wdt_fiq_disable(wdt);
		dev_err(&pdev->dev, "Failed to add wdt disable action\n");
		return ret;
	}

	register_syscore_ops(&sprd_wdt_fiq_syscore_ops);
	wdt_fiq = wdt;

	if (wdt->sleep_en) {
		alarm_init(&wdt_fiq->sleep_tmr, ALARM_BOOTTIME,
			   sprd_wdt_sleep_callback);
	}

	platform_set_drvdata(pdev, wdt);

	return 0;
}

static int __maybe_unused sprd_wdt_fiq_pm_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused sprd_wdt_fiq_pm_resume(struct device *dev)
{
#ifndef SPRD_WDT_SYSCORE_SUSPEND_RESUME
	struct watchdog_device *wdd = dev_get_drvdata(dev);
	struct sprd_wdt_fiq *wdt = dev_get_drvdata(dev);
	int ret;

	ret = sprd_wdt_fiq_enable(wdt);
	if (ret)
		return ret;

	if (watchdog_active(wdd))
		ret = sprd_wdt_fiq_start(&wdt->wdd);
#endif
	return 0;
}

static const struct dev_pm_ops sprd_wdt_fiq_pm_ops = {
	.prepare = sprd_wdt_fiq_alarm_prepare,
	.complete = sprd_wdt_fiq_alarm_complete,
	SET_SYSTEM_SLEEP_PM_OPS(sprd_wdt_fiq_pm_suspend,
				sprd_wdt_fiq_pm_resume)
};

static const struct of_device_id sprd_wdt_fiq_match_table[] = {
	{ .compatible = "sprd,wdt-r2p0-fiq", },
	{},
};
MODULE_DEVICE_TABLE(of, sprd_wdt_fiq_match_table);

static struct platform_driver sprd_watchdog_fiq_driver = {
	.probe	= sprd_wdt_fiq_probe,
	.driver	= {
		.name = "sprd-wdt-fiq",
		.of_match_table = sprd_wdt_fiq_match_table,
		.pm = &sprd_wdt_fiq_pm_ops,
	},
};
module_platform_driver(sprd_watchdog_fiq_driver);

MODULE_AUTHOR("Ling Xu <ling_ling.xu@unisoc.com>");
MODULE_DESCRIPTION("Spreadtrum Watchdog Timer Controller Driver");
MODULE_LICENSE("GPL v2");
