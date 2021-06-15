// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for rfkill status on GPIO lines capable of generating
 * interrupts.
 *
 * Copyright 2021 Purism SPC
 *
 * Somewhat based on gpio-keys which is:
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <dt-bindings/rfkill/rfkill.h>

struct rfkill_hks_switch_pdata {
	int gpio;
	int active_low;
	int debounce_interval;
	enum rfkill_type type;
	const char *name;
};

struct rfkill_hks_pdata {
	const struct rfkill_hks_switch_pdata *switches;
	int nswitches;
	const char *name;
};

struct rfkill_hks_data {
	const struct rfkill_hks_switch_pdata *hks;
	struct device *dev;
	struct gpio_desc *gpiod;
	struct rfkill *rfkill;

	struct delayed_work work;
	unsigned int software_debounce;	/* in msecs */

	unsigned int irq;
};

struct rfkill_hks {
	const struct rfkill_hks_pdata *pdata;
	struct rfkill_hks_data data[];
};

static void rfkill_hks_gpio_report_event(struct rfkill_hks_data *sdata)
{
	int blocked;

	blocked = gpiod_get_value_cansleep(sdata->gpiod);
	if (blocked < 0) {
		dev_err(sdata->dev,
			"failed to get gpio state: %d\n", blocked);
		return;
	}

	dev_dbg(sdata->dev, "HKS %s blocked: %d\n", sdata->hks->name, blocked);
	rfkill_set_hw_state(sdata->rfkill, blocked);
}

static void rfkill_hks_gpio_work_func(struct work_struct *work)
{
	struct rfkill_hks_data *sdata =
		container_of(work, struct rfkill_hks_data, work.work);

	rfkill_hks_gpio_report_event(sdata);
}

static irqreturn_t rfkill_hks_gpio_isr(int irq, void *dev_id)
{
	struct rfkill_hks_data *sdata = dev_id;

	WARN_ON(irq != sdata->irq);

	mod_delayed_work(system_wq,
			 &sdata->work,
			 msecs_to_jiffies(sdata->software_debounce));

	return IRQ_HANDLED;
}

static void rfkill_hks_quiesce_switch(void *data)
{
	struct rfkill_hks_data *sdata = data;

	cancel_delayed_work_sync(&sdata->work);
}

static int rfkill_hks_set(void *data, bool blocked)
{
	struct rfkill_hks_data *sdata = data;

	/*
	 * Nothing to do here
	 */
	dev_dbg (sdata->dev, "%s: rfkill %s, blocked: %d\n", __func__, sdata->hks->name, blocked);

	return 0;
}

static const struct rfkill_ops rfkill_hks_ops = {
	.set_block = rfkill_hks_set,
};

static int rfkill_hks_setup_rfkill(struct platform_device *pdev,
				   struct rfkill_hks *ddata,
				   const struct rfkill_hks_switch_pdata *hks,
				   int idx,
				   struct fwnode_handle *child)
{
	const char *desc = hks->name ? hks->name : "rfkill_hks";
	struct device *dev = &pdev->dev;
	struct rfkill_hks_data *sdata = &ddata->data[idx];
	bool active_low;
	int ret;

	sdata->hks = hks;
	sdata->dev = dev;
	sdata->gpiod = devm_fwnode_gpiod_get(dev, child,
					     NULL, GPIOD_IN, desc);
	if (IS_ERR(sdata->gpiod)) {
		ret = PTR_ERR(sdata->gpiod);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get gpio: %d\n",
				ret);
		return ret;
	}

	active_low = gpiod_is_active_low(sdata->gpiod);

	if (hks->debounce_interval) {
		ret = gpiod_set_debounce(sdata->gpiod,
					   hks->debounce_interval * 1000);
		/* use timer if gpiolib doesn't provide debounce */
		if (ret < 0)
			sdata->software_debounce =
				hks->debounce_interval;
	}

	ret = gpiod_to_irq(sdata->gpiod);
	if (ret < 0) {
		dev_err(dev,
			"Unable to get irq number for GPIO %d, error %d\n",
			hks->gpio, ret);
		return ret;
	}
	sdata->irq = ret;

	INIT_DELAYED_WORK(&sdata->work, rfkill_hks_gpio_work_func);

	/*
	 * Install custom action to cancel release timer and
	 * workqueue item.
	 */
	ret = devm_add_action(dev, rfkill_hks_quiesce_switch, sdata);
	if (ret) {
		dev_err(dev, "failed to register quiesce action, error: %d\n",
			ret);
		return ret;
	}

	ret = devm_request_any_context_irq(dev, sdata->irq,
					   rfkill_hks_gpio_isr,
					   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					   desc, sdata);
	if (ret < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			sdata->irq, ret);
		return ret;
	}

	sdata->rfkill = devm_rfkill_alloc(sdata->hks->name,
					  dev,
					  sdata->hks->type,
					  &rfkill_hks_ops,
					  sdata);
	if (!sdata->rfkill)
		return -ENOMEM;

	ret = rfkill_register(sdata->rfkill);
	if (ret)
		return ret;

	rfkill_hks_gpio_report_event (sdata);
	return 0;
}

/*
 * Translate properties into platform_data
 */
static struct rfkill_hks_pdata *
rfkill_hks_get_devtree_pdata(struct device *dev)
{
	struct rfkill_hks_pdata *pdata;
	struct rfkill_hks_switch_pdata *hks;
	struct fwnode_handle *child;
	int nswitches;

	nswitches = device_get_child_node_count(dev);
	if (nswitches == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev,
			     sizeof(*pdata) + nswitches * sizeof(*hks),
			     GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	hks = (struct rfkill_hks_switch_pdata *)(pdata + 1);

	pdata->switches = hks;
	pdata->nswitches = nswitches;

	device_for_each_child_node(dev, child) {
		int ret;

		fwnode_property_read_string(child, "name", &hks->name);
		ret = fwnode_property_read_u32(child, "type", &hks->type);
		if (ret) {
			dev_err(dev, "Missing rfkill type for %s", hks->name);
			return ERR_PTR(ret);
		}

		if (hks->type >= NUM_RFKILL_TYPES) {
			dev_err(dev, "Invalid rfkill type %d for %s", hks->type, hks->name);
			return ERR_PTR(-EINVAL);
		}

		if (fwnode_property_read_u32(child, "debounce-interval",
					     &hks->debounce_interval))
			hks->debounce_interval = 5;
		hks++;
	}

	return pdata;
}

static const struct of_device_id rfkill_hks_of_match[] = {
	{ .compatible = "rfkill-hks", },
	{ },
};
MODULE_DEVICE_TABLE(of, rfkill_hks_of_match);

static int rfkill_hks_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct rfkill_hks_pdata *pdata = dev_get_platdata(dev);
	struct fwnode_handle *child = NULL;
	struct rfkill_hks *ddata;
	int i, ret;

	if (!pdata) {
		pdata = rfkill_hks_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	ddata = devm_kzalloc(dev, struct_size(ddata, data, pdata->nswitches),
			     GFP_KERNEL);
	if (!ddata) {
		dev_err(dev, "failed to allocate state\n");
		return -ENOMEM;
	}

	ddata->pdata = pdata;
	platform_set_drvdata(pdev, ddata);

	for (i = 0; i < pdata->nswitches; i++) {
		const struct rfkill_hks_switch_pdata *hks = &pdata->switches[i];

		if (!dev_get_platdata(dev)) {
			child = device_get_next_child_node(dev, child);
			if (!child) {
				dev_err(dev,
					"missing child device node for entry %d\n",
					i);
				return -EINVAL;
			}
		}

		ret = rfkill_hks_setup_rfkill(pdev, ddata, hks, i, child);
		if (ret) {
			fwnode_handle_put(child);
			return ret;
		}
	}

	fwnode_handle_put(child);
	return 0;
}

static int __maybe_unused rfkill_hks_suspend(struct device *dev)
{
	struct rfkill_hks *ddata = dev_get_drvdata(dev);
	const struct rfkill_hks_pdata *pdata = ddata->pdata;
	int i;

	for (i = 0; i < pdata->nswitches; i++) {
		struct rfkill_hks_data *sdata = &ddata->data[i];

		cancel_delayed_work_sync(&sdata->work);
	}
	return 0;
}

static int __maybe_unused rfkill_hks_resume(struct device *dev)
{
	struct rfkill_hks *ddata = dev_get_drvdata(dev);
	const struct rfkill_hks_pdata *pdata = ddata->pdata;
	int i;

	for (i = 0; i < pdata->nswitches; i++) {
		struct rfkill_hks_data *sdata = &ddata->data[i];

		rfkill_hks_gpio_report_event(sdata);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(rfkill_hks_pm_ops, rfkill_hks_suspend, rfkill_hks_resume);

static void rfkill_hks_shutdown(struct platform_device *pdev)
{
	int ret;

	ret = rfkill_hks_suspend(&pdev->dev);
	if (ret)
		dev_err(&pdev->dev, "failed to shutdown\n");
}

static struct platform_driver rfkill_hks_device_driver = {
	.probe		= rfkill_hks_probe,
	.shutdown	= rfkill_hks_shutdown,
	.driver		= {
		.name	= "rfkill-hks",
		.pm	= &rfkill_hks_pm_ops,
		.of_match_table = rfkill_hks_of_match,
	}
};

static int __init rfkill_hks_init(void)
{
	return platform_driver_register(&rfkill_hks_device_driver);
}

static void __exit rfkill_hks_exit(void)
{
	platform_driver_unregister(&rfkill_hks_device_driver);
}

late_initcall(rfkill_hks_init);
module_exit(rfkill_hks_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Guido Günther <agx@sigxcpu.org>");
MODULE_DESCRIPTION("Hardware kill switch rfkill driver");
MODULE_ALIAS("platform:rfkill-hks");
