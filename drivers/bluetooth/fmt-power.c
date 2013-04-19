/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2012
/*----------------------------------------------------------------------------*/
/*
 * FMT Power Switch Module
 * controls power to external FMT device
 * with interface to power management device
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>

static bool previous;

static int qci_fmt_toggle_radio(void *data, bool blocked)
{
	int ret = 0;
	int (*power_control)(int enable);

	power_control = data;
	if (previous != blocked)
		ret = (*power_control)(!blocked);
	if (!ret)
		previous = blocked;
	return ret;
}

static const struct rfkill_ops qci_fmt_power_rfkill_ops = {
	.set_block = qci_fmt_toggle_radio,
};

static int qci_fmt_power_rfkill_probe(struct platform_device *pdev)
{
	struct rfkill *rfkill;
	int ret;

	rfkill = rfkill_alloc("fmt_power", &pdev->dev, RFKILL_TYPE_FM,
			      &qci_fmt_power_rfkill_ops,
			      pdev->dev.platform_data);

	if (!rfkill) {
		dev_err(&pdev->dev, "rfkill allocate failed\n");
		return -ENOMEM;
	}

	/* force FMT off during init to allow for user control */
	rfkill_init_sw_state(rfkill, 1);
	previous = 1;

	ret = rfkill_register(rfkill);
	if (ret) {
		dev_err(&pdev->dev, "rfkill register failed=%d\n", ret);
		rfkill_destroy(rfkill);
		return ret;
	}

	platform_set_drvdata(pdev, rfkill);

	return 0;
}

static void qci_fmt_power_rfkill_remove(struct platform_device *pdev)
{
	struct rfkill *rfkill;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	rfkill = platform_get_drvdata(pdev);
	if (rfkill)
		rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
	platform_set_drvdata(pdev, NULL);
}

static int __devinit fmt_power_probe(struct platform_device *pdev)
{
	int ret = 0;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "platform data not initialized\n");
		return -ENOSYS;
	}

	ret = qci_fmt_power_rfkill_probe(pdev);

	return ret;
}

static int __devexit fmt_power_remove(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s\n", __func__);

	qci_fmt_power_rfkill_remove(pdev);

	return 0;
}

static struct platform_driver fmt_power_driver = {
	.probe = fmt_power_probe,
	.remove = __devexit_p(fmt_power_remove),
	.driver = {
		.name = "fmt_power",
		.owner = THIS_MODULE,
	},
};

static int __init qci_fmt_power_init(void)
{
	int ret;

	ret = platform_driver_register(&fmt_power_driver);
	return ret;
}

static void __exit qci_fmt_power_exit(void)
{
	platform_driver_unregister(&fmt_power_driver);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("QCI FMT power control driver");
MODULE_VERSION("1.00");

module_init(qci_fmt_power_init);
module_exit(qci_fmt_power_exit);
