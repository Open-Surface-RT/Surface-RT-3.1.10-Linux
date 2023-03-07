// SPDX-License-Identifier: GPL-2.0+
/*
 * Battery and Charger driver for Surface RT
 * Based on the ACPI Control Method Interface
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/slab.h>

/* Register Addresses (B=byte; W=word; S=string) */
#define REGB_STATUS			0x02
#define REGW_VOLTAGE_NOW		0x20
#define REGW_CURRENT_NOW		0x24
#define REGW_CAPACITY			0x28
#define REGW_CHARGE_FULL		0x2C
#define REGW_CYCLE_COUNT		0x3A
#define REGW_CHARGE_FULL_DESIGN		0x3C
#define REGW_VOLTAGE_MAX_DESIGN		0x3E
#define REGW_SERIAL_NUMBER		0x44
#define REGS_MANUFACTURER		0x46
#define REGS_MODEL_NAME			0x52
#define REGS_TECHNOLOGY			0x5A
#define REGB_ONLINE			0x67
#define REGW_TEMP                       0x68
#define REGW_HEALTH                     0x69

struct srt_ec_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply bat;
	struct power_supply psy;
	struct delayed_work poll_work;
	unsigned int temp;
	unsigned int technology;
	unsigned int capacity;
	unsigned int health;
	char manufacturer[13];
	char model_name[10];
	char serial[5];
};

static enum power_supply_property srt_bat_power_supply_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_HEALTH,

};

static enum power_supply_property srt_psy_power_supply_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
};

static int srt_bat_get_string(struct i2c_client *client, char *buf, u8 reg)
{
	struct i2c_msg msg[2];
	s32 ret = 0;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = 1;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;

	switch (reg) {
	case REGS_MANUFACTURER:
		msg[1].len = 12;
		ret = i2c_transfer(client->adapter, msg, 2);
		break;

	case REGS_MODEL_NAME:
		msg[1].len = 9;
		ret = i2c_transfer(client->adapter, msg, 2);
		break;

	case REGW_SERIAL_NUMBER:
		ret = i2c_smbus_read_word_data(client, reg);
		sprintf(buf, "%04x", ret);
		break;

	case REGS_TECHNOLOGY:
		msg[1].len = 4;
		ret = i2c_transfer(client->adapter, msg, 2);
		break;

	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	return 0;
}

static int srt_bat_get_value(struct i2c_client *client, int reg, int *val)
{
	s32 ret;

	switch (reg) {
	case REGW_CHARGE_FULL_DESIGN:
	case REGW_CHARGE_FULL:
	case REGW_VOLTAGE_MAX_DESIGN:
	case REGW_VOLTAGE_NOW:
		ret = i2c_smbus_read_word_data(client, reg);
		if (ret < 0)
			return ret;

		*val = ret * 1000;
		break;

	case REGW_CURRENT_NOW:
		ret = i2c_smbus_read_word_data(client, reg);
		if (ret < 0)
			return ret;

		*val = (s16)ret * 1000;
		break;

	case REGW_CAPACITY:
	case REGW_CYCLE_COUNT:
		ret = i2c_smbus_read_word_data(client, reg);
		if (ret < 0)
			return ret;

		*val = ret;
		break;

	case REGB_STATUS:
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			return ret;

		if (ret & 0x01)
			*val = POWER_SUPPLY_STATUS_CHARGING;
		else
			*val =  POWER_SUPPLY_STATUS_DISCHARGING;
		break;

	case REGB_ONLINE:
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			return ret;

		*val = (ret & 0x02) >> 1;
		break;

	case REGW_TEMP:
		*val = 20;
                break;

	case REGW_HEALTH:
                *val = 1;
                break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int srt_bat_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct srt_ec_device *srt = container_of(psy, struct srt_ec_device, bat);
	struct i2c_client *client = srt->client;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = srt->manufacturer;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = srt->model_name;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = srt->serial;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = srt_bat_get_value(client, REGW_CAPACITY, &val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = srt_bat_get_value(client, REGW_CHARGE_FULL, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = srt_bat_get_value(client, REGW_CHARGE_FULL_DESIGN,
					&val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = srt_bat_get_value(client, REGW_CURRENT_NOW, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = srt_bat_get_value(client, REGW_CYCLE_COUNT, &val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:
		ret = srt_bat_get_value(client, REGB_ONLINE, &val->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (srt->capacity < 100)
			ret = srt_bat_get_value(client, REGB_STATUS, &val->intval);
		else
			val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = srt->technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		ret = srt_bat_get_value(client, REGW_VOLTAGE_MAX_DESIGN,
					&val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = srt_bat_get_value(client, REGW_VOLTAGE_NOW, &val->intval);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval =20;
		break;

        case POWER_SUPPLY_PROP_HEALTH:
		val->intval =1;
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static int srt_psy_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct srt_ec_device *srt = container_of(psy, struct srt_ec_device, psy);
	s32 ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:
		ret = i2c_smbus_read_byte_data(srt->client, REGB_ONLINE);
		if (ret < 0)
			return ret;

		val->intval = ret & 0x01;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void srt_bat_poll_work(struct work_struct *work)
{
	struct srt_ec_device *srt;
	int err, capacity;

	srt = container_of(work, struct srt_ec_device, poll_work.work);
	err = srt_bat_get_value(srt->client, REGW_CAPACITY, &capacity);

	if (!err && capacity != srt->capacity) {
		srt->capacity = capacity;
		power_supply_changed(&srt->bat);
	}

	/* continuously send uevent notification */
	schedule_delayed_work(&srt->poll_work, 3 * HZ);
}

static irqreturn_t srt_psy_detect_irq(int irq, void *dev_id)
{
	struct srt_ec_device *srt = dev_id;

	power_supply_changed(&srt->psy);
	return IRQ_HANDLED;
}

static int __devinit srt_ec_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct srt_ec_device *srt;
	char str_buf[4];
	int err;
	int ret;

	srt = devm_kzalloc(dev, sizeof(*srt), GFP_KERNEL);
	if (!srt)
		return -ENOMEM;

	i2c_set_clientdata(client, srt);
	srt->client = client;
	srt->dev = dev;
	
	srt->bat.name = "battery",
	srt->bat.type = POWER_SUPPLY_TYPE_BATTERY,
	srt->bat.properties = srt_bat_power_supply_props,
	srt->bat.num_properties = ARRAY_SIZE(srt_bat_power_supply_props),
	srt->bat.get_property = srt_bat_power_supply_get_property,
	srt->bat.external_power_changed = power_supply_changed,
	
	srt->psy.name = "surface-rt-ac-adapter",
	srt->psy.type = POWER_SUPPLY_TYPE_MAINS,
	srt->psy.properties = srt_psy_power_supply_props,
	srt->psy.num_properties = ARRAY_SIZE(srt_psy_power_supply_props),
	srt->psy.get_property = srt_psy_power_supply_get_property,

	/* wait till EC is ready */
	usleep_range(1000, 1500);

	err = srt_bat_get_string(client, srt->manufacturer, REGS_MANUFACTURER);
	if (err)
		return err;

	err = srt_bat_get_string(client, srt->model_name, REGS_MODEL_NAME);
	if (err)
		return err;

	err = srt_bat_get_string(client, srt->serial, REGW_SERIAL_NUMBER);
	if (err)
		return err;

	err = srt_bat_get_string(client, str_buf, REGS_TECHNOLOGY);
	if (err)
		return err;

	if (!strncmp(str_buf, "LION", 4))
		srt->technology = POWER_SUPPLY_TECHNOLOGY_LION;
	else
		srt->technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

	ret = power_supply_register(dev, &srt->bat);
	if (ret) {
		dev_err(&client->dev, "failed to register battery power supply\n");
		kfree(srt);
		return ret;
	}

	ret = power_supply_register(dev, &srt->psy);
	if (ret) {
		dev_err(&client->dev, "failed to register AC power supply\n");
		kfree(srt);
		return ret;
	}

        err = devm_request_threaded_irq(dev, client->irq, NULL, srt_psy_detect_irq,
					IRQF_ONESHOT, client->name, srt);
	if (err) {
		dev_err(&client->dev, "failed to request interrupt: %d\n", err);
	//	return err;
	}

	INIT_DELAYED_WORK(&srt->poll_work, srt_bat_poll_work);
	schedule_delayed_work(&srt->poll_work, HZ);

	return 0;
}

static int srt_ec_remove(struct i2c_client *client)
{
	struct srt_ec_device *srt = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&srt->poll_work);

	return 0;
}

static const struct i2c_device_id srt_ec_id[] = {
	{ "surface-rt-ec", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, srt_ec_id);

static const struct of_device_id srt_ec_of_match[] = {
	{ .compatible = "microsoft,surface-rt-ec", },
	{},
};
MODULE_DEVICE_TABLE(of, srt_ec_of_match);

static struct i2c_driver srt_ec_driver = {
	.driver = {
		.name = "surface-rt-ec",
		.of_match_table = srt_ec_of_match,
	},
	.probe = srt_ec_probe,
	.remove = srt_ec_remove,
	.id_table	= srt_ec_id,
};
module_i2c_driver(srt_ec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jonas Schwöbel <jonasschwoebel@yahoo.de>");
MODULE_DESCRIPTION("Surface RT Embedded Controller driver");
