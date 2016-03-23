/*
 * CY8C4014 LED chip driver
 *
 * Copyright (C) 2014-2015 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/bug.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/leds.h>
#include <linux/debugfs.h>

/* register definitions */
#define P1961_REG_CMD			0x00
#define P1961_REG_CMD_DAT		0x01
#define P1961_REG_CMD_STATUS		0x02
#define P1961_REG_APP_MINOR_REV		0x03
#define P1961_REG_APP_MAJOR_REV		0x04
#define P1961_REG_LED_STATE		0x05
#define P1961_REG_LED_ON_TIME		0x06
#define P1961_REG_LED_OFF_TIME		0x07
#define P1961_REG_MAX_BRIGHT		0x08
#define P1961_REG_NOM_BRIGHT		0x09
#define P1961_REG_LED_RAMP_UP		0x0A
#define P1961_REG_LED_RAMP_DOWN		0x0B
#define P1961_REG_HAPTIC_EN		0x0C
#define P1961_REG_HAPTIC_DRIVE_TIME	0x0D
#define P1961_REG_HAPTIC_BRAKE_DLY	0x0E
#define P1961_REG_HAPTIC_BRAKE_T	0x0F
#define P1961_REG_SOUND_PULSE_LEN	0x10
#define P1961_REG_MAX			0x11
#define P1961_CMD_ENTER_BL		0x01
#define P1961_CMD_WRITE_EEPROM		0x05

#define P1961_LED_STATE_BLINK		0x01
#define P1961_LED_STATE_BREATH		0x02
#define P1961_LED_STATE_SOLID		0x03
#define P1961_LED_STATE_OFF		0x04

struct cy8c_data {
	struct i2c_client *client;
	struct regmap *regmap;
	struct led_classdev led;
	int mode_index; /* mode index to mode_controls */
	bool led_on_plugin;
	u8 default_brightness;
	struct mutex lock;
};

static void set_led_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	struct cy8c_data *data = container_of(led_cdev, struct cy8c_data, led);

	regmap_write(data->regmap, P1961_REG_NOM_BRIGHT, value & 0xff);
}

static enum led_brightness get_led_brightness(struct led_classdev *led_cdev)
{
	unsigned int val;
	int ret;
	struct cy8c_data *data = container_of(led_cdev, struct cy8c_data, led);

	ret = regmap_read(data->regmap,	P1961_REG_NOM_BRIGHT, &val);
	if (ret)
		dev_err(&data->client->dev, "cannot read %d\n", val);
	val = val & 0xff;
	return val;
}

static int of_led_parse_pdata(struct i2c_client *client, struct cy8c_data *data)
{
	struct device_node *np = client->dev.of_node;
	u32 value;
	data->led.name = of_get_property(np, "label", NULL) ? : np->name;

	data->led_on_plugin = of_property_read_bool(np, "led-on-plugin");

	if (of_property_read_u32(np, "default-brightness", &value))
		data->default_brightness = 0xff;
	data->default_brightness = value & 0xff;

	return 0;
}

static int cy8c_debug_set(void *data, u64 val)
{
	struct cy8c_data *cy_data = data;

	val = val & 0xff;
	if (val > P1961_CMD_ENTER_BL)
		return -EINVAL;

	pr_info("%s: send reset cmd %lld\n", __func__, val);
	regmap_write(cy_data->regmap, P1961_REG_CMD, val & 0xff);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cy8c_debug_fops, NULL, cy8c_debug_set, "%lld\n");

static int cy8c_apply_default_settings(struct cy8c_data *data)
{
	int ret;
	u8 state;

	if (!data)
		return -EINVAL;

	pr_debug("%s: led-on-plugin %d, default-brightness %d\n", __func__,
				data->led_on_plugin, data->default_brightness);
	ret = regmap_write(data->regmap, P1961_REG_NOM_BRIGHT,
						data->default_brightness);
	ret |= regmap_write(data->regmap, P1961_REG_CMD_DAT,
						P1961_REG_NOM_BRIGHT);
	ret |= regmap_write(data->regmap,
				P1961_REG_CMD, P1961_CMD_WRITE_EEPROM);
	if (ret)
		dev_err(&data->client->dev, "cannot write brightness\n");

	if (data->led_on_plugin)
		state = P1961_LED_STATE_OFF;
	else
		state = P1961_LED_STATE_SOLID;

	ret = regmap_write(data->regmap, P1961_REG_LED_STATE, state);
	ret |= regmap_write(data->regmap, P1961_REG_CMD_DAT,
						P1961_REG_LED_STATE);
	ret |= regmap_write(data->regmap,
			P1961_REG_CMD, P1961_CMD_WRITE_EEPROM);
	if (ret)
		dev_err(&data->client->dev, "cannot write led state\n");
	return ret;
}

static int cy8c_led_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = client->adapter;
	struct cy8c_data *data;
	struct regmap_config rconfig;
	struct dentry *d;
	int ret, reg;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c functionality check fail.\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* enable print in show/get */
	data->client = client;

	of_led_parse_pdata(client, data);

	data->led.brightness_set = set_led_brightness;
	data->led.brightness_get = get_led_brightness;

	memset(&rconfig, 0, sizeof(rconfig));
	rconfig.reg_bits = 8;
	rconfig.val_bits = 8;
	rconfig.cache_type = REGCACHE_NONE;
	rconfig.max_register = P1961_REG_MAX-1;

	/*This should happen before set clientdata*/
	data->regmap = regmap_init_i2c(client, &rconfig);
	if (!data->regmap) {
		dev_err(&client->dev, "Failed to allocate register map\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, data);

	ret = regmap_read(data->regmap, P1961_REG_APP_MAJOR_REV, &reg);
	if (ret) {
		dev_err(&client->dev, "Failed to read revision-major\n");
		return ret;
	}

	dev_dbg(&client->dev, "[nv-foster] rev: 0x%02x ", reg);

	ret = regmap_read(data->regmap, P1961_REG_APP_MINOR_REV, &reg);
	if (ret) {
		dev_err(&client->dev, "Failed to read revision-minor\n");
		return ret;
	}

	dev_dbg(&client->dev, "0x%02x\n", reg);

	cy8c_apply_default_settings(data);

	ret = led_classdev_register(&client->dev, &data->led);
	if (ret < 0)
		dev_err(&client->dev, "Failed to register foster led\n");
	else
		dev_info(&client->dev, "LED registered (%s)\n", data->led.name);

	/* create debugfs for f/w loading purpose */
	d = debugfs_create_file("cy8c_led", S_IRUGO, NULL, data,
							&cy8c_debug_fops);
	if (!d)
		pr_err("Failed to create suspend_mode debug file\n");

	return ret;
}

static int cy8c_led_remove(struct i2c_client *client)
{
	struct cy8c_data *data = i2c_get_clientdata(client);

	led_classdev_unregister(&data->led);

	return 0;
}

static const struct i2c_device_id cy8c_led_id[] = {
	{"cy8c_led", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, cy8c_led_id);

#ifdef CONFIG_OF
static struct of_device_id cy8c_of_match[] = {
	{.compatible = "nvidia,cy8c_led", },
	{ },
};
#endif

static struct i2c_driver cy8c_led_driver = {
	.driver = {
		.name   = "cy8c_led",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table =
			of_match_ptr(cy8c_of_match),
#endif
	},
	.probe      = cy8c_led_probe,
	.remove     = cy8c_led_remove,
	.id_table   = cy8c_led_id,
};

module_i2c_driver(cy8c_led_driver);

MODULE_AUTHOR("Vinayak Pane <vpane@nvidia.com>");
MODULE_DESCRIPTION("CY8C I2C based LED controller driver");
MODULE_LICENSE("GPL");
