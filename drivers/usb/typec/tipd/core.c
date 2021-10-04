// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for TI TPS6598x USB Power Delivery controller family
 *
 * Copyright (C) 2017, Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */

#include <linux/debugfs.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/extcon.h>
#include <linux/extcon-provider.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/usb/typec.h>
#include <linux/usb/role.h>

#include "tps6598x.h"
#include "trace.h"

/* Register offsets */
#define TPS_REG_VID			0x00
#define TPS_REG_MODE			0x03
#define TPS_REG_CUSTOMER_USE		0x06
#define TPS_REG_CMD1			0x08
#define TPS_REG_DATA1			0x09
#define TPS_REG_INT_EVENT1		0x14
#define TPS_REG_INT_EVENT2		0x15
#define TPS_REG_INT_MASK1		0x16
#define TPS_REG_INT_MASK2		0x17
#define TPS_REG_INT_CLEAR1		0x18
#define TPS_REG_INT_CLEAR2		0x19
#define TPS_REG_STATUS			0x1a
#define TPS_REG_SYSTEM_CONF		0x28
#define TPS_REG_CTRL_CONF		0x29
#define TPS_REG_ACTIVE_CONTRACT		0x34
#define TPS_REG_POWER_STATUS		0x3f
#define TPS_REG_RX_IDENTITY_SOP		0x48
#define TPS_REG_DATA_STATUS		0x5f

#define TPS_USB_500mA	  500000
#define TPS_TYPEC_1500mA 1500000
#define TPS_TYPEC_3000mA 3000000
#define TPS_USB_5V	 5000000

#define CC_INT_MASK			TPS_REG_INT_STATUS_UPDATE

/* TPS_REG_SYSTEM_CONF bits */
#define TPS_SYSCONF_PORTINFO(c)		((c) & 7)

enum {
	TPS_PORTINFO_SINK,
	TPS_PORTINFO_SINK_ACCESSORY,
	TPS_PORTINFO_DRP_UFP,
	TPS_PORTINFO_DRP_UFP_DRD,
	TPS_PORTINFO_DRP_DFP,
	TPS_PORTINFO_DRP_DFP_DRD,
	TPS_PORTINFO_SOURCE,
};

/* TPS_REG_RX_IDENTITY_SOP */
struct tps6598x_rx_identity_reg {
	u8 status;
	struct usb_pd_identity identity;
} __packed;

/* Standard Task return codes */
#define TPS_TASK_TIMEOUT		1
#define TPS_TASK_REJECTED		3

enum {
	TPS_MODE_APP,
	TPS_MODE_BOOT,
	TPS_MODE_BIST,
	TPS_MODE_DISC,
};

static const char *const modes[] = {
	[TPS_MODE_APP]	= "APP ",
	[TPS_MODE_BOOT]	= "BOOT",
	[TPS_MODE_BIST]	= "BIST",
	[TPS_MODE_DISC]	= "DISC",
};

/* Unrecognized commands will be replaced with "!CMD" */
#define INVALID_CMD(_cmd_)		(_cmd_ == 0x444d4321)

struct tps6598x {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock; /* device lock */
	u8 i2c_protocol:1;

	struct typec_port *port;
	struct typec_partner *partner;
	struct usb_pd_identity partner_identity;
	struct usb_role_switch *role_sw;
	struct typec_capability typec_cap;

	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	enum power_supply_usb_type usb_type;

	struct tps6598x_pdo terms;

	u32 data_status;
	u16 pwr_status;
	bool dp;
	struct extcon_dev *extcon;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dev_dentry;
	struct dentry *customer_user_dentry;
#endif
};

static enum power_supply_property tps6598x_psy_props[] = {
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static enum power_supply_usb_type tps6598x_psy_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
};

static const char *tps6598x_psy_name_prefix = "tps6598x-source-psy-";

/*
 * Max data bytes for Data1, Data2, and other registers. See ch 1.3.2:
 * https://www.ti.com/lit/ug/slvuan1a/slvuan1a.pdf
 */
#define TPS_MAX_LEN	64

static int
tps6598x_block_read(struct tps6598x *tps, u8 reg, void *val, size_t len)
{
	u8 data[TPS_MAX_LEN + 1];
	int ret;

	if (WARN_ON(len + 1 > sizeof(data)))
		return -EINVAL;

	if (!tps->i2c_protocol)
		return regmap_raw_read(tps->regmap, reg, val, len);

	ret = regmap_raw_read(tps->regmap, reg, data, sizeof(data));
	if (ret)
		return ret;

	if (data[0] < len)
		return -EIO;

	memcpy(val, &data[1], len);
	return 0;
}

static int tps6598x_block_write(struct tps6598x *tps, u8 reg,
				const void *val, size_t len)
{
	u8 data[TPS_MAX_LEN + 1];

	if (!tps->i2c_protocol)
		return regmap_raw_write(tps->regmap, reg, val, len);

	data[0] = len;
	memcpy(&data[1], val, len);

	return regmap_raw_write(tps->regmap, reg, data, sizeof(data));
}

static inline int tps6598x_read16(struct tps6598x *tps, u8 reg, u16 *val)
{
	return tps6598x_block_read(tps, reg, val, sizeof(u16));
}

static inline int tps6598x_read32(struct tps6598x *tps, u8 reg, u32 *val)
{
	return tps6598x_block_read(tps, reg, val, sizeof(u32));
}

static inline int tps6598x_read64(struct tps6598x *tps, u8 reg, u64 *val)
{
	return tps6598x_block_read(tps, reg, val, sizeof(u64));
}

static inline int tps6598x_write16(struct tps6598x *tps, u8 reg, u16 val)
{
	return tps6598x_block_write(tps, reg, &val, sizeof(u16));
}

static inline int tps6598x_write32(struct tps6598x *tps, u8 reg, u32 val)
{
	return tps6598x_block_write(tps, reg, &val, sizeof(u32));
}

static inline int tps6598x_write64(struct tps6598x *tps, u8 reg, u64 val)
{
	return tps6598x_block_write(tps, reg, &val, sizeof(u64));
}

static inline int
tps6598x_write_4cc(struct tps6598x *tps, u8 reg, const char *val)
{
	return tps6598x_block_write(tps, reg, val, 4);
}

static int tps6598x_read_partner_identity(struct tps6598x *tps)
{
	struct tps6598x_rx_identity_reg id;
	int ret;

	ret = tps6598x_block_read(tps, TPS_REG_RX_IDENTITY_SOP,
				  &id, sizeof(id));
	if (ret)
		return ret;

	tps->partner_identity = id.identity;

	return 0;
}

static void tps6598x_set_data_role(struct tps6598x *tps,
				   enum typec_data_role role, bool connected)
{
	enum usb_role role_val;

	if (role == TYPEC_HOST)
		role_val = USB_ROLE_HOST;
	else
		role_val = USB_ROLE_DEVICE;

	if (!connected)
		role_val = USB_ROLE_NONE;

	usb_role_switch_set_role(tps->role_sw, role_val);
	typec_set_data_role(tps->port, role);
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *rootdir;

static int tps6598x_debug_customer_use_show(struct seq_file *s, void *v)
{
	struct tps6598x *tps = (struct tps6598x *)s->private;
	u64 mode64;
	int ret;

	mutex_lock(&tps->lock);

	ret =  tps6598x_block_read(tps, TPS_REG_CUSTOMER_USE, &mode64, sizeof(mode64));
	if (!ret)
		seq_printf(s, "0x%016llx\n", mode64);

	mutex_unlock(&tps->lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(tps6598x_debug_customer_use);

static void tps6598x_debugfs_init(struct tps6598x *tps)
{
	struct dentry *dentry;

	if (!rootdir)
		rootdir = debugfs_create_dir("tps6598x", NULL);

	dentry = debugfs_create_dir(dev_name(tps->dev), rootdir);
	if (IS_ERR(dentry))
		return;
	tps->dev_dentry = dentry;

	dentry = debugfs_create_file("customer_use",
				     S_IFREG | 0444, tps->dev_dentry,
				     tps, &tps6598x_debug_customer_use_fops);
	if (IS_ERR(dentry))
		return;
	tps->customer_user_dentry = dentry;
}

static void tps6598x_debugfs_exit(struct tps6598x *tps)
{
	debugfs_remove(tps->customer_user_dentry);
	debugfs_remove(tps->dev_dentry);
	debugfs_remove(rootdir);
	rootdir = NULL;
}

#else

static void tps6598x_debugfs_init(const struct tps6598x *tps) { }
static void tps6598x_debugfs_exit(const struct tps6598x *tps) { }

#endif

static int tps6598x_mask_reg(struct tps6598x *tps, int reg, u64 mask, bool set)
{
	u64 val;
	int ret;

	ret = tps6598x_read64(tps, reg, &val);
	if (ret < 0) {
		dev_err(tps->dev, "Reading reg 0x%x mask failed %d", reg, ret);
		return ret;
	}
	if (set)
		val |= mask;
	else
		val &= ~mask;
	ret = tps6598x_write64(tps, reg, val);
	if (ret < 0) {
		dev_err(tps->dev, "Writing reg 0x%x mask failed %d", reg, ret);
		return ret;
	}

	dev_dbg(tps->dev, "register mask updated %llx %llx", val, mask);

	return 0;
}

static int tps6598x_mask_cc_int(struct tps6598x *tps, bool disable)
{
	int ret;

	ret = tps6598x_mask_reg(tps, TPS_REG_INT_MASK1, CC_INT_MASK, !disable);
	ret |= tps6598x_mask_reg(tps, TPS_REG_INT_MASK2, CC_INT_MASK, !disable);

	if (ret < 0) {
		dev_err(tps->dev, "Writing interrupt mask failed %d", ret);
		return ret;
	}

	return 0;
}

static int tps6598x_connect(struct tps6598x *tps, u32 status)
{
	struct typec_partner_desc desc;
	enum typec_pwr_opmode mode;
	int ret;

	if (tps->partner)
		return 0;

	mode = TPS_POWER_STATUS_PWROPMODE(tps->pwr_status);

	desc.usb_pd = mode == TYPEC_PWR_MODE_PD;
	desc.accessory = TYPEC_ACCESSORY_NONE; /* XXX: handle accessories */
	desc.identity = NULL;

	if (desc.usb_pd) {
		ret = tps6598x_read_partner_identity(tps);
		if (ret)
			return ret;
		desc.identity = &tps->partner_identity;
	}

	typec_set_pwr_opmode(tps->port, mode);
	typec_set_pwr_role(tps->port, TPS_STATUS_TO_TYPEC_PORTROLE(status));
	typec_set_vconn_role(tps->port, TPS_STATUS_TO_TYPEC_VCONN(status));
	tps6598x_set_data_role(tps, TPS_STATUS_TO_TYPEC_DATAROLE(status), true);

	tps->partner = typec_register_partner(tps->port, &desc);
	if (IS_ERR(tps->partner))
		return PTR_ERR(tps->partner);

	if (desc.identity)
		typec_partner_set_identity(tps->partner);

	tps6598x_mask_cc_int(tps, false);

	power_supply_changed(tps->psy);

	return 0;
}

static int
tps6598x_update_data_status(struct tps6598x *tps, u32 status)
{
	bool dp;

	tps6598x_set_data_role(tps, TPS_STATUS_TO_TYPEC_DATAROLE(status),
			       !!(tps->data_status & TPS_DATA_STATUS_DATA_CONNECTION));
	trace_tps6598x_data_status(tps->data_status);

	dp = !!(tps->data_status & TPS_DATA_STATUS_DP_CONNECTION);
	if (tps->dp != dp) {
		tps->dp = dp;
		extcon_set_state_sync(tps->extcon, EXTCON_DISP_DP, tps->dp);
		extcon_sync(tps->extcon, EXTCON_DISP_DP);
	}
	return 0;
}

static void tps6598x_disconnect(struct tps6598x *tps, u32 status)
{
	if (!IS_ERR(tps->partner))
		typec_unregister_partner(tps->partner);
	tps->partner = NULL;
	typec_set_pwr_opmode(tps->port, TYPEC_PWR_MODE_USB);
	typec_set_pwr_role(tps->port, TPS_STATUS_TO_TYPEC_PORTROLE(status));
	typec_set_vconn_role(tps->port, TPS_STATUS_TO_TYPEC_VCONN(status));
	tps6598x_set_data_role(tps, TPS_STATUS_TO_TYPEC_DATAROLE(status), false);

	memset(&tps->terms, 0, sizeof(struct tps6598x_pdo));

	tps6598x_mask_cc_int(tps, true);
	power_supply_changed(tps->psy);
}

static int tps6598x_exec_cmd(struct tps6598x *tps, const char *cmd,
			     size_t in_len, u8 *in_data,
			     size_t out_len, u8 *out_data)
{
	unsigned long timeout;
	u32 val;
	int ret;

	ret = tps6598x_read32(tps, TPS_REG_CMD1, &val);
	if (ret)
		return ret;
	if (val && !INVALID_CMD(val))
		return -EBUSY;

	if (in_len) {
		ret = tps6598x_block_write(tps, TPS_REG_DATA1,
					   in_data, in_len);
		if (ret)
			return ret;
	}

	ret = tps6598x_write_4cc(tps, TPS_REG_CMD1, cmd);
	if (ret < 0)
		return ret;

	/* XXX: Using 1s for now, but it may not be enough for every command. */
	timeout = jiffies + msecs_to_jiffies(1000);

	do {
		ret = tps6598x_read32(tps, TPS_REG_CMD1, &val);
		if (ret)
			return ret;
		if (INVALID_CMD(val))
			return -EINVAL;

		if (time_is_before_jiffies(timeout))
			return -ETIMEDOUT;
	} while (val);

	if (out_len) {
		ret = tps6598x_block_read(tps, TPS_REG_DATA1,
					  out_data, out_len);
		if (ret)
			return ret;
		val = out_data[0];
	} else {
		ret = tps6598x_block_read(tps, TPS_REG_DATA1, &val, sizeof(u8));
		if (ret)
			return ret;
	}

	switch (val) {
	case TPS_TASK_TIMEOUT:
		return -ETIMEDOUT;
	case TPS_TASK_REJECTED:
		return -EPERM;
	default:
		break;
	}

	return 0;
}

static int tps6598x_dr_set(struct typec_port *port, enum typec_data_role role)
{
	const char *cmd = (role == TYPEC_DEVICE) ? "SWUF" : "SWDF";
	struct tps6598x *tps = typec_get_drvdata(port);
	u32 status;
	int ret;

	mutex_lock(&tps->lock);

	ret = tps6598x_exec_cmd(tps, cmd, 0, NULL, 0, NULL);
	if (ret)
		goto out_unlock;

	ret = tps6598x_read32(tps, TPS_REG_STATUS, &status);
	if (ret)
		goto out_unlock;

	if (role != TPS_STATUS_TO_TYPEC_DATAROLE(status)) {
		ret = -EPROTO;
		goto out_unlock;
	}

out_unlock:
	mutex_unlock(&tps->lock);

	return ret;
}

static int tps6598x_pr_set(struct typec_port *port, enum typec_role role)
{
	const char *cmd = (role == TYPEC_SINK) ? "SWSk" : "SWSr";
	struct tps6598x *tps = typec_get_drvdata(port);
	u32 status;
	int ret;

	mutex_lock(&tps->lock);

	ret = tps6598x_exec_cmd(tps, cmd, 0, NULL, 0, NULL);
	if (ret)
		goto out_unlock;

	ret = tps6598x_read32(tps, TPS_REG_STATUS, &status);
	if (ret)
		goto out_unlock;

	if (role != TPS_STATUS_TO_TYPEC_PORTROLE(status)) {
		ret = -EPROTO;
		goto out_unlock;
	}

	typec_set_pwr_role(tps->port, role);

out_unlock:
	mutex_unlock(&tps->lock);

	return ret;
}

static const struct typec_operations tps6598x_ops = {
	.dr_set = tps6598x_dr_set,
	.pr_set = tps6598x_pr_set,
};

static int tps6598x_get_active_pd_contract(struct tps6598x *tps)
{
	u64 contract;
	int type;
	int max_power;
	int ret = 0;

	ret = tps6598x_block_read(tps, TPS_REG_ACTIVE_CONTRACT, &contract, 6);
	if (ret)
		return ret;

	contract &= 0xFFFFFFFFFFFF;
	type = TPS_PDO_CONTRACT_TYPE(contract);
	memset(&tps->terms, 0, sizeof(struct tps6598x_pdo));

	/* If there's no PD it decodes to all 0 */
	switch (type) {
	case TPS_PDO_CONTRACT_FIXED:
		tps->terms.max_voltage = TPS_PDO_FIXED_CONTRACT_VOLTAGE(contract);
		tps->terms.max_current = TPS_PDO_FIXED_CONTRACT_MAX_CURRENT(contract);
		break;
	case TPS_PDO_CONTRACT_BATTERY:
		tps->terms.max_voltage = TPS_PDO_BAT_CONTRACT_MAX_VOLTAGE(contract);
		max_power = TPS_PDO_BAT_CONTRACT_MAX_POWER(contract);
		tps->terms.max_current = 1000 * 1000 * max_power / tps->terms.max_voltage;
		break;
	case TPS_PDO_CONTRACT_VARIABLE:
		tps->terms.max_voltage = TPS_PDO_VAR_CONTRACT_MAX_VOLTAGE(contract);
		tps->terms.max_current = TPS_PDO_VAR_CONTRACT_MAX_CURRENT(contract);
		break;
	default:
		dev_warn(tps->dev, "Unknown contract type: %d\n", type);
		return -EINVAL;
	}

	tps->terms.pdo = contract;
	trace_tps6598x_pdo(&tps->terms);

	return 0;
}

static irqreturn_t tps6598x_interrupt(int irq, void *data)
{
	struct tps6598x *tps = data;
	u64 event1;
	u64 event2;
	u32 status, data_status;
	u16 pwr_status;
	bool psy_changed = false;
	int ret;
	u64 mask;

	mutex_lock(&tps->lock);

	ret = tps6598x_read64(tps, TPS_REG_INT_EVENT1, &event1);
	ret |= tps6598x_read64(tps, TPS_REG_INT_EVENT2, &event2);
	if (ret) {
		dev_err(tps->dev, "%s: failed to read events\n", __func__);
		goto err_unlock;
	}
	trace_tps6598x_irq(event1, event2);

	ret = tps6598x_read32(tps, TPS_REG_STATUS, &status);
	if (ret) {
		dev_err(tps->dev, "%s: failed to read status\n", __func__);
		goto err_clear_ints;
	}
	trace_tps6598x_status(status);

	/*
	 * In practice it seems like pwr_status can change also if the
	 * TPS_REG_INT_PP_SWITCH_CHANGED bit is set, so we interpret
	 * either of the TPS_REG_INT_POWER_STATUS_UPDATE or
	 * TPS_REG_INT_PP_SWITCH_CHANGED bits being set as a possible
	 * power status change.
	 */
	if ((event1 | event2) & (TPS_REG_INT_POWER_STATUS_UPDATE | TPS_REG_INT_PP_SWITCH_CHANGED)) {
		ret = tps6598x_read16(tps, TPS_REG_POWER_STATUS, &pwr_status);
		if (ret < 0) {
			dev_err(tps->dev, "failed to read power status: %d\n", ret);
			goto err_clear_ints;
		}
		tps->pwr_status = pwr_status;
		psy_changed = true;
		trace_tps6598x_power_status(pwr_status);
	}

	if ((event1 | event2) & TPS_REG_INT_DATA_STATUS_UPDATE) {
		ret = tps6598x_read32(tps, TPS_REG_DATA_STATUS, &data_status);
		if (ret < 0) {
			dev_err(tps->dev, "failed to read data status: %d\n", ret);
			goto err_clear_ints;
		}
		tps->data_status = data_status;
		tps6598x_update_data_status(tps, status);
	}

	if ((event1 | event2) & TPS_REG_INT_NEW_CONTRACT_AS_CONSUMER) {
		if (tps6598x_get_active_pd_contract(tps) < 0) {
			dev_err(tps->dev, "failed to read pd contract: %d\n", ret);
			goto err_clear_ints;
		}
		psy_changed = true;
	}

	/* Handle plug insert or removal */
	if ((event1 | event2) & TPS_REG_INT_PLUG_EVENT) {
		if (status & TPS_STATUS_PLUG_PRESENT) {
			ret = tps6598x_connect(tps, status);
			if (ret)
				dev_err(tps->dev,
					"failed to register partner\n");
		} else {
			tps6598x_disconnect(tps, status);
		}
	}

	if ((event1 | event2) & TPS_REG_INT_HARD_RESET) {
		memset(&tps->terms, 0, sizeof(struct tps6598x_pdo));
		psy_changed = true;
	}

	if ((event1 | event2) & TPS_REG_INT_STATUS_UPDATE) {
		ret = tps6598x_read64(tps, TPS_REG_INT_MASK1, &mask);
		if (ret < 0)
			dev_err( tps->dev, "Reading interrupt mask failed");
		dev_dbg(tps->dev, "Status update: %x %llx", status, mask);
		if (!(mask & TPS_REG_INT_STATUS_UPDATE))
			dev_err( tps->dev, "The interrupt is masked , how did it fire ?? %llx", mask);

		if (!(status & TPS_STATUS_PLUG_PRESENT) ||
		    TPS_STATUS_CONN_STATE(status) !=
		    (TPS_STATUS_CONN_STATE_CONN_NO_R_A | TPS_STATUS_CONN_STATE_CONN_WITH_R_A)) {
			/* the status update register can fire even when masked so try
			   and mask it again */
			ret = tps6598x_mask_cc_int(tps, true);
			if (ret < 0)
				dev_err( tps->dev, "Writing interrupt mask failed");
			else
				dev_dbg( tps->dev, "interrupt mask updated %llx", mask);
		}
	}

err_clear_ints:
	tps6598x_write64(tps, TPS_REG_INT_CLEAR1, event1);
	tps6598x_write64(tps, TPS_REG_INT_CLEAR2, event2);

err_unlock:
	mutex_unlock(&tps->lock);

	if (psy_changed)
		power_supply_changed(tps->psy);

	return IRQ_HANDLED;
}

static int tps6598x_check_mode(struct tps6598x *tps)
{
	char mode[5] = { };
	int ret;

	ret = tps6598x_read32(tps, TPS_REG_MODE, (void *)mode);
	if (ret)
		return ret;

	switch (match_string(modes, ARRAY_SIZE(modes), mode)) {
	case TPS_MODE_APP:
		return 0;
	case TPS_MODE_BOOT:
		dev_warn(tps->dev, "dead-battery condition\n");
		return 0;
	case TPS_MODE_BIST:
	case TPS_MODE_DISC:
	default:
		dev_err(tps->dev, "controller in unsupported mode \"%s\"\n",
			mode);
		break;
	}

	return -ENODEV;
}

static const struct regmap_config tps6598x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7F,
};

static int tps6598x_psy_get_online(struct tps6598x *tps,
				   union power_supply_propval *val)
{
	if (TPS_POWER_STATUS_CONNECTION(tps->pwr_status) &&
	    TPS_POWER_STATUS_SOURCESINK(tps->pwr_status)) {
		val->intval = 1;
	} else {
		val->intval = 0;
	}

	return 0;
}

static int tps6598x_psy_get_max_current(struct tps6598x *tps,
					union power_supply_propval *val)
{
	enum typec_pwr_opmode mode;

	mode = TPS_POWER_STATUS_PWROPMODE(tps->pwr_status);
	switch (mode) {
	case TYPEC_PWR_MODE_1_5A:
		val->intval = TPS_TYPEC_1500mA;
		break;
	case TYPEC_PWR_MODE_3_0A:
		val->intval = TPS_TYPEC_3000mA;
		break;
	case TYPEC_PWR_MODE_PD:
		val->intval = tps->terms.max_current ?: TPS_USB_500mA;
		break;
	default:
	case TYPEC_PWR_MODE_USB:
		val->intval = TPS_USB_500mA;
	}
	return 0;
}

static int tps6598x_psy_get_max_voltage(struct tps6598x *tps,
					union power_supply_propval *val)
{
	enum typec_pwr_opmode mode;

	mode = TPS_POWER_STATUS_PWROPMODE(tps->pwr_status);
	switch (mode) {
	case TYPEC_PWR_MODE_PD:
		val->intval = tps->terms.max_voltage ?: TPS_USB_5V;
		break;
	default:
	case TYPEC_PWR_MODE_1_5A:
	case TYPEC_PWR_MODE_3_0A:
	case TYPEC_PWR_MODE_USB:
		val->intval = TPS_USB_5V;
	}
	return 0;
}

static int tps6598x_psy_get_prop(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct tps6598x *tps = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_USB_TYPE:
		if (TPS_POWER_STATUS_PWROPMODE(tps->pwr_status) == TYPEC_PWR_MODE_PD)
			val->intval = POWER_SUPPLY_USB_TYPE_PD;
		else
			val->intval = POWER_SUPPLY_USB_TYPE_C;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = tps6598x_psy_get_online(tps, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = tps6598x_psy_get_max_current(tps, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = tps6598x_psy_get_max_voltage(tps, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int devm_tps6598_psy_register(struct tps6598x *tps)
{
	struct power_supply_config psy_cfg = {};
	const char *port_dev_name = dev_name(tps->dev);
	char *psy_name;

	psy_cfg.drv_data = tps;
	psy_cfg.fwnode = dev_fwnode(tps->dev);

	psy_name = devm_kasprintf(tps->dev, GFP_KERNEL, "%s%s", tps6598x_psy_name_prefix,
				  port_dev_name);
	if (!psy_name)
		return -ENOMEM;

	tps->psy_desc.name = psy_name;
	tps->psy_desc.type = POWER_SUPPLY_TYPE_USB;
	tps->psy_desc.usb_types = tps6598x_psy_usb_types;
	tps->psy_desc.num_usb_types = ARRAY_SIZE(tps6598x_psy_usb_types);
	tps->psy_desc.properties = tps6598x_psy_props;
	tps->psy_desc.num_properties = ARRAY_SIZE(tps6598x_psy_props);
	tps->psy_desc.get_property = tps6598x_psy_get_prop;

	tps->usb_type = POWER_SUPPLY_USB_TYPE_C;

	tps->psy = devm_power_supply_register(tps->dev, &tps->psy_desc,
					       &psy_cfg);
	return PTR_ERR_OR_ZERO(tps->psy);
}

static const unsigned int tps6598x_extcon_cable[] = {
	EXTCON_DISP_DP,
	EXTCON_NONE,
};

static int tps6598x_probe(struct i2c_client *client)
{
	struct typec_capability typec_cap = { };
	struct tps6598x *tps;
	struct fwnode_handle *fwnode;
	u32 status;
	u32 conf;
	u32 vid;
	int ret;

	tps = devm_kzalloc(&client->dev, sizeof(*tps), GFP_KERNEL);
	if (!tps)
		return -ENOMEM;

	mutex_init(&tps->lock);
	tps->dev = &client->dev;

	tps->regmap = devm_regmap_init_i2c(client, &tps6598x_regmap_config);
	if (IS_ERR(tps->regmap))
		return PTR_ERR(tps->regmap);

	ret = tps6598x_read32(tps, TPS_REG_VID, &vid);
	if (ret < 0 || !vid)
		return -ENODEV;

	/*
	 * Checking can the adapter handle SMBus protocol. If it can not, the
	 * driver needs to take care of block reads separately.
	 *
	 * FIXME: Testing with I2C_FUNC_I2C. regmap-i2c uses I2C protocol
	 * unconditionally if the adapter has I2C_FUNC_I2C set.
	 */
	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		tps->i2c_protocol = true;

	/* Make sure the controller has application firmware running */
	ret = tps6598x_check_mode(tps);
	if (ret)
		return ret;

	ret = tps6598x_read32(tps, TPS_REG_STATUS, &status);
	if (ret < 0)
		return ret;
	trace_tps6598x_status(status);

	ret = tps6598x_read32(tps, TPS_REG_SYSTEM_CONF, &conf);
	if (ret < 0)
		return ret;

	fwnode = device_get_named_child_node(&client->dev, "connector");
	if (!fwnode)
		return -ENODEV;

	/*
	 * This fwnode has a "compatible" property, but is never populated as a
	 * struct device. Instead we simply parse it to read the properties.
	 * This breaks fw_devlink=on. To maintain backward compatibility
	 * with existing DT files, we work around this by deleting any
	 * fwnode_links to/from this fwnode.
	 */
	fw_devlink_purge_absent_suppliers(fwnode);

	tps->role_sw = fwnode_usb_role_switch_get(fwnode);
	if (IS_ERR(tps->role_sw)) {
		ret = PTR_ERR(tps->role_sw);
		goto err_fwnode_put;
	}

	typec_cap.revision = USB_TYPEC_REV_1_2;
	typec_cap.pd_revision = 0x200;
	typec_cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;
	typec_cap.driver_data = tps;
	typec_cap.ops = &tps6598x_ops;
	typec_cap.fwnode = fwnode;

	switch (TPS_SYSCONF_PORTINFO(conf)) {
	case TPS_PORTINFO_SINK_ACCESSORY:
	case TPS_PORTINFO_SINK:
		typec_cap.type = TYPEC_PORT_SNK;
		typec_cap.data = TYPEC_PORT_UFP;
		break;
	case TPS_PORTINFO_DRP_UFP_DRD:
	case TPS_PORTINFO_DRP_DFP_DRD:
		typec_cap.type = TYPEC_PORT_DRP;
		typec_cap.data = TYPEC_PORT_DRD;
		break;
	case TPS_PORTINFO_DRP_UFP:
		typec_cap.type = TYPEC_PORT_DRP;
		typec_cap.data = TYPEC_PORT_UFP;
		break;
	case TPS_PORTINFO_DRP_DFP:
		typec_cap.type = TYPEC_PORT_DRP;
		typec_cap.data = TYPEC_PORT_DFP;
		break;
	case TPS_PORTINFO_SOURCE:
		typec_cap.type = TYPEC_PORT_SRC;
		typec_cap.data = TYPEC_PORT_DFP;
		break;
	default:
		ret = -ENODEV;
		goto err_role_put;
	}

	ret = devm_tps6598_psy_register(tps);
	if (ret)
		return ret;

	tps->port = typec_register_port(&client->dev, &typec_cap);
	if (IS_ERR(tps->port)) {
		ret = PTR_ERR(tps->port);
		goto err_role_put;
	}
	fwnode_handle_put(fwnode);

	tps->extcon = devm_extcon_dev_allocate(tps->dev, tps6598x_extcon_cable);
	if (IS_ERR(tps->extcon)) {
		dev_err(tps->dev, "failed to allocate memory for extcon\n");
		ret = PTR_ERR(tps->extcon);
		goto err_role_put;
	}

	/* Register extcon device */
	ret = devm_extcon_dev_register(tps->dev, tps->extcon);
	if (ret) {
		dev_err(tps->dev, "failed to register extcon device: %d\n", ret);
		goto err_role_put;
	}

	/* set initial state */
	extcon_set_state_sync(tps->extcon, EXTCON_DISP_DP, false);

	if (status & TPS_STATUS_PLUG_PRESENT) {
		ret = tps6598x_read16(tps, TPS_REG_POWER_STATUS, &tps->pwr_status);
		if (ret < 0) {
			dev_err(tps->dev, "failed to read power status: %d\n", ret);
			goto err_role_put;
		}
		ret = tps6598x_connect(tps, status);
		if (ret)
			dev_err(&client->dev, "failed to register partner\n");
	} else {
		tps6598x_mask_cc_int(tps, true);
	}

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					tps6598x_interrupt,
					IRQF_SHARED | IRQF_ONESHOT,
					dev_name(&client->dev), tps);
	if (ret) {
		tps6598x_disconnect(tps, 0);
		typec_unregister_port(tps->port);
		goto err_role_put;
	}

	i2c_set_clientdata(client, tps);
	tps6598x_debugfs_init(tps);

	return 0;

err_role_put:
	usb_role_switch_put(tps->role_sw);
err_fwnode_put:
	fwnode_handle_put(fwnode);

	return ret;
}

static int tps6598x_remove(struct i2c_client *client)
{
	struct tps6598x *tps = i2c_get_clientdata(client);

	tps6598x_debugfs_exit(tps);
	tps6598x_disconnect(tps, 0);
	typec_unregister_port(tps->port);
	usb_role_switch_put(tps->role_sw);

	return 0;
}

static const struct of_device_id tps6598x_of_match[] = {
	{ .compatible = "ti,tps6598x", },
	{}
};
MODULE_DEVICE_TABLE(of, tps6598x_of_match);

static const struct i2c_device_id tps6598x_id[] = {
	{ "tps6598x" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps6598x_id);

static struct i2c_driver tps6598x_i2c_driver = {
	.driver = {
		.name = "tps6598x",
		.of_match_table = tps6598x_of_match,
	},
	.probe_new = tps6598x_probe,
	.remove = tps6598x_remove,
	.id_table = tps6598x_id,
};
module_i2c_driver(tps6598x_i2c_driver);

MODULE_AUTHOR("Heikki Krogerus <heikki.krogerus@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI TPS6598x USB Power Delivery Controller Driver");
