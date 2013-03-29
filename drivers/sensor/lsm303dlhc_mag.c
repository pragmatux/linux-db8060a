/* drivers/input/misc/lsm303dlhc_mag.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/input.h>

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/sensor/lsm303dlhc_mag.h>
#include <linux/regulator/consumer.h>
#include <linux/sensor/sensors_core.h>
/*#define DEBUG_MAG_SENSOR*/
#define DEBUG_ATTR

#define MAG_OVERFLOW -4096
#define ESATURATION -32768
#define MAG_RANGE_MG  8100
/* Magnetometer registers */
#define CRA_REG_M   0x00  /* Configuration register A */
#define CRB_REG_M   0x01  /* Configuration register B */
#define MR_REG_M    0x02  /* Mode register */

/* Output register start address*/
#define OUT_X_M 0x03

/* Magnetic Sensor Operation Mode */
#define NORMAL_MODE       0x00
#define POS_BIAS          0x01
#define NEG_BIAS          0x02
#define CC_MODE           0x00
#define IDLE_MODE         0x03
#define OPERATION_MODE CC_MODE

/* Magnetometer output data rate  */
#define ODR_75  (0 << 2)  /* 0.75Hz output data rate */
#define ODR1_5  (1 << 2)  /* 1.5Hz output data rate */
#define ODR3_0  (2 << 2)  /* 3Hz output data rate */
#define ODR7_5  (3 << 2)  /* 7.5Hz output data rate */
#define ODR15   (4 << 2)  /* 15Hz output data rate */
#define ODR30   (5 << 2)  /* 30Hz output data rate */
#define ODR75   (6 << 2)  /* 75Hz output data rate */
#define ODR220  (7 << 2)  /* 220Hz output data rate */

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

struct lsm303dlhc_mag_gains {
	s32 xy;
	s32 z;
};

static const u8 lsm303dlhc_mag_range_bits[] = {
	0x01 << 5,
	0x02 << 5,
	0x03 << 5,
	0x04 << 5,
	0x05 << 5,
	0x06 << 5,
	0x07 << 5,
};

static const struct output_rate odr_table[] = {
	{ 1334, ODR_75},
	{  667, ODR1_5},
	{  334, ODR3_0},
	{  134, ODR7_5},
	{   67, ODR15 },
	{   34, ODR30 },
	{   14, ODR75 },
	{    5, ODR220 },
};
#define POLL_INT_MIN odr_table[ARRAY_SIZE(odr_table) - 1].poll_rate_ms

#define SCALE (1000 * 1024)
static const struct lsm303dlhc_mag_gains lsm303dlhc_mag_gains[] = {
	{ SCALE / 1055, SCALE / 950 },
	{ SCALE / 795,  SCALE / 710 },
	{ SCALE / 635,  SCALE / 570 },
	{ SCALE / 430,  SCALE / 385 },
	{ SCALE / 375,  SCALE / 335 },
	{ SCALE / 320,  SCALE / 285 },
	{ SCALE / 230,  SCALE / 205 },
};

struct lsm303dlhc_mag_data {
	struct i2c_client *client;
	struct delayed_work work;
	struct mutex lock;
	struct input_dev *input_dev;
	struct device *dev;
	s32 gain[3];
	bool interruptible;	/* interrupt or polling? */
	int poll_interval_ms;
	u32  irq_m;      
	enum lsm303dlhc_mag_range range;
	int (*power_on)(void);
	int (*power_off)(void);
	u8 powered;
#ifdef DEBUG_ATTR
	u8 reg_addr;
#endif
};

#define LOCK(p) do { \
	dev_dbg(&(p)->client->dev, "%s: lock\n", __func__); \
	mutex_lock(&p->lock); \
} while (0)

#define UNLOCK(p) do { \
	dev_dbg(&(p)->client->dev, "%s: unlock\n", __func__); \
	mutex_unlock(&p->lock); \
} while (0)

struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct sensor_regulator lsm303dlhc_mag_vreg[] = {
	{NULL, "vdd2v9", 2400000, 3300000},
	{NULL, "vdd1v8", 1800000, 1900000},
};

static int lsm303dlhc_mag_config_regulator(struct lsm303dlhc_mag_data *mag, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(lsm303dlhc_mag_vreg) / sizeof(struct sensor_regulator);

	if (on) {
		for (i = 0; i < num_reg; i++) {
			lsm303dlhc_mag_vreg[i].vreg =
				regulator_get(&mag->client->dev,
				lsm303dlhc_mag_vreg[i].name);
			if (IS_ERR(lsm303dlhc_mag_vreg[i].vreg)) {
				rc = PTR_ERR(lsm303dlhc_mag_vreg[i].vreg);
				pr_err("%s:regulator get failed rc=%d\n",
								__func__, rc);
				goto error_vdd;
			}

			if (regulator_count_voltages(
				lsm303dlhc_mag_vreg[i].vreg) > 0) {
				rc = regulator_set_voltage(
					lsm303dlhc_mag_vreg[i].vreg,
					lsm303dlhc_mag_vreg[i].min_uV,
					lsm303dlhc_mag_vreg[i].max_uV);
				if (rc) {
					pr_err("%s: set voltage failed rc=%d\n",
					__func__, rc);
					regulator_put(lsm303dlhc_mag_vreg[i].vreg);
					goto error_vdd;
				}
			}

			rc = regulator_enable(lsm303dlhc_mag_vreg[i].vreg);
			if (rc) {
				pr_err("%s: regulator_enable failed rc =%d\n",
					__func__, rc);
				if (regulator_count_voltages(
					lsm303dlhc_mag_vreg[i].vreg) > 0) {
					regulator_set_voltage(
						lsm303dlhc_mag_vreg[i].vreg, 0,
						lsm303dlhc_mag_vreg[i].max_uV);
				}
				regulator_put(lsm303dlhc_mag_vreg[i].vreg);
				goto error_vdd;
			}
		}
		return rc;
	} else {
		i = num_reg;
	}

error_vdd:
	while (--i >= 0) {
		if (regulator_count_voltages(lsm303dlhc_mag_vreg[i].vreg) > 0) {
			regulator_set_voltage(lsm303dlhc_mag_vreg[i].vreg, 0,
						lsm303dlhc_mag_vreg[i].max_uV);
		}
		regulator_disable(lsm303dlhc_mag_vreg[i].vreg);
		regulator_put(lsm303dlhc_mag_vreg[i].vreg);
	}
	return rc;
}

static int lsm303dlhc_mag_set_sampling_rate(struct lsm303dlhc_mag_data *mag)
{
	int i;
	s32 err;
	u8 config;

	for (i = 0; i < ARRAY_SIZE(odr_table) - 1; i++)
		if (odr_table[i].poll_rate_ms <= mag->poll_interval_ms)
			break;

	config = odr_table[i].mask;
	err = i2c_smbus_write_byte_data(mag->client, CRA_REG_M, config);
	if (err)
		return err;
	dev_dbg(&mag->client->dev, "%s sampling rate %d ms\n",
		__func__, odr_table[i].poll_rate_ms);
	if (mag->poll_interval_ms < POLL_INT_MIN) {
		mag->poll_interval_ms = POLL_INT_MIN;
		dev_dbg(&mag->client->dev, "%s polling rate lowered to %d ms\n",
			__func__, POLL_INT_MIN);
	}

	return 0;
}

static int lsm303dlhc_mag_set_range(struct lsm303dlhc_mag_data *mag)
{
	s32 err;
	u8 range_bits = lsm303dlhc_mag_range_bits[mag->range];
	enum lsm303dlhc_mag_range r = mag->range;

	err = i2c_smbus_write_byte_data(mag->client, CRB_REG_M, range_bits);
	if (!err) {
		mag->gain[1] = mag->gain[0] = lsm303dlhc_mag_gains[r].xy;
		mag->gain[2] = lsm303dlhc_mag_gains[r].z;
	}
	dev_dbg(&mag->client->dev, "%s range_id %d (conf_b 0x%02x)\n",
		__func__, r, range_bits);
	dev_dbg(&mag->client->dev, "%s: gains %d %d %d\n", __func__,
		mag->gain[0], mag->gain[1], mag->gain[2]);
	return (int)err;

}

static int lsm303dlhc_mag_power_stub(void)
{
	return 0;
}

static void lsm303dlhc_mag_device_power_off(struct lsm303dlhc_mag_data *mag)
{
	s32 err;

	mag->powered = 0;
	cancel_delayed_work_sync(&mag->work);
	dev_dbg(&mag->client->dev, "%s: soft power-off\n", __func__);
	err = i2c_smbus_write_byte_data(mag->client, MR_REG_M, IDLE_MODE);
	if (err)
		dev_err(&mag->client->dev, "%s: smbus err %d\n", __func__, err);
	if (mag->interruptible)
		disable_irq(gpio_to_irq(mag->irq_m));
	lsm303dlhc_mag_config_regulator( mag , false);
	mag->power_off();
}

static void lsm303dlhc_mag_device_power_on(struct lsm303dlhc_mag_data *mag)
{
	s32 err;

	mag->power_on();
	lsm303dlhc_mag_config_regulator( mag , true);
	dev_dbg(&mag->client->dev, "%s: soft power-on\n", __func__);
	lsm303dlhc_mag_set_sampling_rate(mag);
	lsm303dlhc_mag_set_range(mag);
	err = i2c_smbus_write_byte_data(mag->client, MR_REG_M, OPERATION_MODE);
	if (err)
		dev_err(&mag->client->dev, "%s: smbus err %d\n", __func__, err);
	if (mag->interruptible)
		enable_irq(gpio_to_irq(mag->irq_m));
	else
		schedule_delayed_work(&mag->work,
			msecs_to_jiffies(mag->poll_interval_ms));

	mag->powered = 1;
}

static int lsm303dlhc_mag_get_data(struct lsm303dlhc_mag_data *mag)
{
	s32 err;
	s16 xyz[3];
	s32 *g = mag->gain;
	int value[3];

	err = i2c_smbus_read_i2c_block_data(mag->client, OUT_X_M,
					    sizeof(xyz), (u8 *)xyz);
	if (err < 0)
		return err;
#ifdef __LITTLE_ENDIAN
	xyz[0] = swab16(xyz[0]);
	xyz[1] = swab16(xyz[1]);
	xyz[2] = swab16(xyz[2]);
#endif
	value[0] = (xyz[0] != MAG_OVERFLOW) ? (xyz[0] * g[0]) >> 10 : ESATURATION;
	input_report_abs(mag->input_dev, ABS_X, value[0]);
	value[1] = (xyz[1] != MAG_OVERFLOW) ? (xyz[1] * g[1]) >> 10 : ESATURATION;
	input_report_abs(mag->input_dev, ABS_Y, value[1]);
	value[2] = (xyz[2] != MAG_OVERFLOW) ? (xyz[2] * g[2]) >> 10 : ESATURATION;
	input_report_abs(mag->input_dev, ABS_Z, value[2]);
	input_sync(mag->input_dev);
	return 0;
}

static void lsm303dlhc_mag_poll_func(struct work_struct *work)
{
	struct lsm303dlhc_mag_data *mag =
			container_of((struct delayed_work *)work,
			struct lsm303dlhc_mag_data, work);
	LOCK(mag);
	if (mag->powered) {
		lsm303dlhc_mag_get_data(mag);
		if (!mag->interruptible)
			schedule_delayed_work(&mag->work,
				msecs_to_jiffies(mag->poll_interval_ms));
	}
	UNLOCK(mag);
}

static enum lsm303dlhc_mag_range lsm303dlhc_mag_mG_2_range(unsigned long range)
{
	if (range <= 1300)
		return LSM303_RANGE_1300mG;
	if (range <= 1900)
		return LSM303_RANGE_1900mG;
	if (range <= 2500)
		return LSM303_RANGE_2500mG;
	if (range <= 4000)
		return LSM303_RANGE_4000mG;
	if (range <= 4700)
		return LSM303_RANGE_4700mG;
	if (range <= 5600)
		return LSM303_RANGE_5600mG;
	return LSM303_RANGE_8200mG;
}

static ssize_t attr_set_range(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	unsigned long range;
	if (strict_strtoul(buf, 10, &range))
		return -EINVAL;
	LOCK(mag);
	mag->range = lsm303dlhc_mag_mG_2_range(range);
	if (mag->powered)
		lsm303dlhc_mag_set_range(mag);
	UNLOCK(mag);
	return size;
}

static ssize_t attr_set_poll_rate(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	unsigned long interval_ms;
	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	LOCK(mag);
	mag->poll_interval_ms = interval_ms;
	if (mag->powered)
		lsm303dlhc_mag_set_sampling_rate(mag);
	UNLOCK(mag);
	return size;
}

#ifdef DEBUG_ATTR
static ssize_t attr_reg_set(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	s32 rc;
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	u8 reg;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	LOCK(mag);
	reg = mag->reg_addr;
	UNLOCK(mag);
	rc = i2c_smbus_write_byte_data(mag->client, reg, val);
	dev_dbg(dev, "%s: 0x%02lx -> reg[%02x], rc = %d\n", __func__,
		       val, reg, rc);
	return size;
}

static ssize_t attr_reg_get(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	s32 data;

	LOCK(mag);
	data = mag->reg_addr;
	UNLOCK(mag);
	data = i2c_smbus_read_byte_data(mag->client, data);
	if (data < 0)
		return -EIO;
	dev_dbg(dev, "%s: reg[%02x] <- 0x%02x\n", __func__,
	       mag->reg_addr, data);
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}


static ssize_t attr_addr_set(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	LOCK(mag);
	mag->reg_addr = val;
	UNLOCK(mag);
	return size;
}
#endif /* DEBUG_ATTR */

static ssize_t lsm303dlhc_mag_show_enable(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", mag->powered);
}

static ssize_t lsm303dlhc_mag_set_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	u8 new_enable;

	if (sysfs_streq(buf, "1"))
		new_enable = 1;
	else if (sysfs_streq(buf, "0"))
		new_enable = 0;
	else {
		pr_debug("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	printk(KERN_INFO "%s, %d enable : %d.\n", __func__, __LINE__,\
		new_enable);

	if (new_enable == mag->powered)
		return size;
	LOCK(mag);
	if( new_enable ) 
		lsm303dlhc_mag_device_power_on(mag);
	else 
		lsm303dlhc_mag_device_power_off(mag);
	UNLOCK(mag);
	return size;
}

static DEVICE_ATTR(enable, S_IRUGO, lsm303dlhc_mag_show_enable, lsm303dlhc_mag_set_enable);

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0200, NULL, attr_set_poll_rate),
	__ATTR(range_mg, 0200, NULL, attr_set_range),
#ifdef DEBUG_ATTR
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;
error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: Unable to create interface\n", __func__);
	return -EIO;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}


static irqreturn_t lsm303dlhc_m_gpio_irq(int irq, void *device_data)
{
	struct lsm303dlhc_mag_data *mag = device_data;
#ifdef DEBUG_MAG_SENSOR
	printk(KERN_INFO "%sinterrupt\n", __func__);
#endif
	LOCK(mag);
	lsm303dlhc_mag_get_data(mag);
	UNLOCK(mag);
	return IRQ_HANDLED;

}


static int __devinit lsm303dlhc_mag_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct lsm303dlhc_mag_platform_data *pdata = client->dev.platform_data;
	struct lsm303dlhc_mag_data *mag;
	int result = 0;

	dev_info(&client->dev, "%s \n", __func__);
	
	if (!pdata) {
		result = -EINVAL;
		dev_err(&client->dev, "%s: platform data required.\n",
			__func__);
		goto err_no_platform_data;
	}
	mag = kzalloc(sizeof(*mag), GFP_KERNEL);
	if (NULL == mag) {
		result = -ENOMEM;
		goto err_alloc_data_failed;
	}
	mag->client = client;
	mag->poll_interval_ms = pdata->poll_interval_ms;
	mag->range = pdata->range;
	dev_info(&client->dev, "%s:pdata->range:%d\n", __func__, pdata->range);
	i2c_set_clientdata(client, mag);
	if (pdata->power_on)
		mag->power_on = pdata->power_on;
	else
		mag->power_on = lsm303dlhc_mag_power_stub;
	if (pdata->power_off)
		mag->power_off = pdata->power_off;
	else
		mag->power_off = lsm303dlhc_mag_power_stub;
	mag->power_on();
	lsm303dlhc_mag_config_regulator( mag , true);
	msleep(500);
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		result = -EIO;
		goto err_check_functionality;
	}
	result = i2c_smbus_write_byte_data(client, MR_REG_M, IDLE_MODE);
	mag->power_off();
	lsm303dlhc_mag_config_regulator( mag , false);
	if (result) {
		dev_err(&client->dev, "%s: Device not responding.\n",
			__func__);
		goto err_not_responding;
	}
	INIT_DELAYED_WORK(&mag->work, lsm303dlhc_mag_poll_func);
	mutex_init(&mag->lock);

	mag->dev = sensors_classdev_register("magnetometer");
	
	result = create_sysfs_interfaces(mag->dev);
	if (result)
		goto err_sys_attr;
	dev_set_drvdata(mag->dev, mag);
	mag->input_dev = input_allocate_device();
	if (!mag->input_dev) {
		dev_err(&client->dev, "%s: input_allocate_device failed\n",
			__func__);
		result = -ENOMEM;
		goto err_allocate_device;
	}
	input_set_drvdata(mag->input_dev, mag);

	mag->input_dev->name = "magnetometer";
	set_bit(EV_ABS, mag->input_dev->evbit);
	set_bit(ABS_X, mag->input_dev->absbit);
	set_bit(ABS_Y, mag->input_dev->absbit);
	set_bit(ABS_Z, mag->input_dev->absbit);
	input_set_abs_params(mag->input_dev, ABS_X, -MAG_RANGE_MG,
			MAG_RANGE_MG - 1, 0, 0);
	input_set_abs_params(mag->input_dev, ABS_Y, -MAG_RANGE_MG,
			MAG_RANGE_MG - 1, 0, 0);
	input_set_abs_params(mag->input_dev, ABS_Z, -MAG_RANGE_MG,
			MAG_RANGE_MG - 1, 0, 0);

	result = input_register_device(mag->input_dev);
	if (result) {
		dev_err(&client->dev, "%s: input_register_device failed!",
			__func__);
		goto err_register_device;
	}
	dev_set_drvdata(&mag->input_dev->dev, mag);
	
	if (device_create_file(&mag->input_dev->dev,
				&dev_attr_enable) < 0) {
		pr_err("%s: Failed to create device file(%s)!\n", __func__,
				dev_attr_enable.attr.name);
		goto err_check_functionality;
	}

	mag->irq_m = pdata->irq_m;
	if (mag->irq_m > 0) { /* interrupt */
		
		mag->interruptible = true;
		result = request_threaded_irq(gpio_to_irq(mag->irq_m), NULL,
		   lsm303dlhc_m_gpio_irq,
		   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "lsm303dlhc_m",
		   mag);
		if (result) {
			dev_err(&client->dev, "request irq EGPIO_PIN_1 failed\n");
			goto err_check_functionality;
		}

		disable_irq(gpio_to_irq(mag->irq_m));
	}
	mag->interruptible = false;
	dev_info(&client->dev, "%s mag->interruptible=%d completed.\n", __func__, mag->interruptible);
	return 0;

err_register_device:
	input_free_device(mag->input_dev);
err_allocate_device:
	remove_sysfs_interfaces(mag->dev);
err_sys_attr:
err_not_responding:
	kfree(mag);
err_alloc_data_failed:
err_no_platform_data:
err_check_functionality:
	dev_err(&client->dev, "%s failed.\n", __func__);
	return result;
}

static int __devexit lsm303dlhc_mag_remove(struct i2c_client *client)
{
	struct lsm303dlhc_mag_data *mag = i2c_get_clientdata(client);

	device_remove_file(&mag->input_dev->dev,&dev_attr_enable);
	input_unregister_device(mag->input_dev);
	remove_sysfs_interfaces(mag->dev);
	sensors_classdev_unregister(mag->dev);
	kfree(mag);
	return 0;
}

static const struct i2c_device_id lsm303dlhc_mag_id[] = {
	{LSM303DLHC_MAG_DEV_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, lsm303dlhc_mag_id);

#ifdef CONFIG_PM
static int lsm303dlhc_mag_suspend(struct device *dev)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_mag_data *mag = i2c_get_clientdata(client);

	dev_dbg(dev, "%s\n", __func__);
	LOCK(mag);
	if (mag->powered) 
		lsm303dlhc_mag_device_power_off(mag);
	UNLOCK(mag);
	return 0;
}

static int lsm303dlhc_mag_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_mag_data *mag = i2c_get_clientdata(client);

	dev_dbg(dev, "%s\n", __func__);
	LOCK(mag);
	if (mag->input_dev->users) {
		dev_dbg(dev, "%s: resuming active operation.\n",
			   __func__);
		if (!mag->powered)
			lsm303dlhc_mag_device_power_on(mag);
	}
	UNLOCK(mag);
	return 0;
}
static const struct dev_pm_ops lsm303dlhc_mag_pm = {
	.suspend = lsm303dlhc_mag_suspend,
	.resume = lsm303dlhc_mag_resume,
};
#endif

static struct i2c_driver lsm303dlhc_mag_driver = {
	.driver = {
		.name = LSM303DLHC_MAG_DEV_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &lsm303dlhc_mag_pm,
#endif
	},
	.probe = lsm303dlhc_mag_probe,
	.remove = __devexit_p(lsm303dlhc_mag_remove),
	.id_table = lsm303dlhc_mag_id,
};

static int __init lsm303dlhc_mag_init(void)
{
	int err = i2c_add_driver(&lsm303dlhc_mag_driver);
	return err;
}

static void __exit lsm303dlhc_mag_exit(void)
{
	i2c_del_driver(&lsm303dlhc_mag_driver);
}

module_init(lsm303dlhc_mag_init);
module_exit(lsm303dlhc_mag_exit);

MODULE_AUTHOR("Aleksej Makarov <aleksej.makarov@sonyericsson.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("lsm303dlh magnetometer driver");
