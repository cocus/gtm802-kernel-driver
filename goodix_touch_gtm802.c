/*
 *  Driver for Goodix GT802 ARM touchscreen controllers
 *
 *  Copyright (c) 2015 Priit Laes <plaes@plaes.org>.
 *  Copyright (c) 2025 Santiago Hormazabal <santiagohssl@gmail.com>
 *
 *  This code is based on goodix.c driver (c) 2014 Red Hat Inc,
 *  various Android codedumps (c) 2010 - 2012 Goodix Technology
 *  and cleanups done by Emilio López (turl) for linux-sunxi.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 of the License.
 */
#include <linux/unaligned.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input/mt.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/input/touchscreen.h>

struct gt801x_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int abs_x_max;
	int abs_y_max;
	unsigned int max_touch_num;
	unsigned int int_trigger_type;

	struct gpio_desc *reset_gpio;
	struct touchscreen_properties prop;
};

#define GOODIX_MAX_HEIGHT		768
#define GOODIX_MAX_WIDTH		1024
#define GOODIX_INT_TRIGGER		1
#define GOODIX_MAX_CONTACTS		5
#define MAX_CONTACTS_LOC	5
#define RESOLUTION_LOC		1
#define TRIGGER_LOC		6

/* Register defines */
#define GT801X_COOR_ADDR		0x01
#define GT801X_CONFIG_DATA	0x65
#define GT801X_REG_ID			0xf0

/* Device specific defines */
#define GT801X_CONFIG_MAX_LENGTH	7
#define GT801X_CONTACT_SIZE		5

static const unsigned long goodix_irq_flags[] = {
	IRQ_TYPE_EDGE_RISING,
	IRQ_TYPE_EDGE_FALLING,
	IRQ_TYPE_LEVEL_LOW,
	IRQ_TYPE_LEVEL_HIGH,
};

/**
 * gt801x_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to read from.
 * @buf: raw write data buffer.
 * @len: length of the buffer to write
 */
static int gt801x_i2c_read(struct i2c_client *client,
			   u8 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 1;
	msgs[0].buf   = &reg;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

static int gt801x_i2c_write(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;
	int retries = 0;

	msg.flags=0;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;		
	
	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,&msg, 1);
		if(ret == 1)break;
		retries++;
	}

	if (ret >= 0)
		ret = (ret == 1 ? 0 : -EIO);
	
	return ret;
}

/**
 * gt801x_process_events - Process incoming events
 *
 * @ts: our gt801x_ts_data pointer
 *
 * Called when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void gt801x_process_events(struct gt801x_ts_data *ts)
{
	u8 point_data[3 + GT801X_CONTACT_SIZE * GOODIX_MAX_CONTACTS];
	u8 touch_map[GOODIX_MAX_CONTACTS] = {0};
	int input_x, input_y, input_w;
	u8 checksum = 0;
	u16 touch_raw;
	u8 touch_num;
	int error;
	int loc;
	int i;

	error = gt801x_i2c_read(ts->client, GT801X_COOR_ADDR,
				point_data, sizeof(point_data));
	if (error) {
		dev_err(&ts->client->dev, "I2C transfer error: %d\n", error);
		return;
	}

	/* Fetch touch mapping bits */
	touch_raw = get_unaligned_le16(&point_data[0]);
	if (!touch_raw)
		goto report_not_pressed;

	/* Build touch map */
	touch_num = 0;
	for (i = 0; (touch_raw != 0) && (i < ts->max_touch_num); i++) {
		if (touch_raw & 1)
			touch_map[touch_num++] = i;
		touch_raw >>= 1;
	}

	/* Calculate checksum */
	for (i = 0; i < (touch_num*GT801X_CONTACT_SIZE + 3); i++)
		checksum += point_data[i];
	if (checksum != 0)
		return;

	/* Report touches */
	for (i = 0; i < touch_num; i++) {
		loc = 2 + GT801X_CONTACT_SIZE * i;
		input_x = get_unaligned_be16(&point_data[loc]);
		input_y = get_unaligned_be16(&point_data[loc + 2]);
		input_w = point_data[loc + 4];

		dev_dbg(&ts->client->dev,
			 "Touch %d [i = %d]: x: %d, y: %d, w: %d *****\n",
			 touch_map[i], i, input_x, input_y, input_w);

		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input_dev, &ts->prop,
			input_x, input_y, true);
		//input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
		//input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
		//input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, track_id[index]);

	}

report_not_pressed:
	for (i = touch_num; i < ts->max_touch_num; i++) {
		dev_dbg(&ts->client->dev,
			 "Touch %d [i = %d]: x: 0, y: 0, w: 0 NOT PRESSED\n",
			 touch_map[i], i);
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
	}

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

/**
 * gt801x_ts_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t gt801x_ts_irq_handler(int irq, void *dev_id)
{
	gt801x_process_events((struct gt801x_ts_data *)dev_id);

	return IRQ_HANDLED;
}

static uint8_t cfg_info_group3[] = {
	/* address */
	GT801X_CONFIG_DATA,
	/* data */
	0x03, 0x04, 0x00, 0x03, 0x00, 0x05,
	0x61, 0x02, 0x00, 0x0F, 0x22, 0x02,
	0x08, 0x10, 0x00, 0x00, 0x26, 0x00,
	0x00, 0x88, 0x10, 0x10, 0x32, 0x37,
	0x00, 0x00, 0x0C, 0x0B, 0x0A, 0x09,
	0x08, 0x07, 0x06, 0x05, 0x04, 0x03,
	0x02, 0x01, 0x00, 0xFF, 0xFF, 0xFF,
	0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07,
	0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
	0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
	0x41, 0x55, 0x64, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x20
};

static uint8_t cfg_info_group4[] = {
	/* address */
	GT801X_CONFIG_DATA,
	/* data */
	0x01, 0x04, 0x00, 0x03, 0x00, 0x05,
	0x61, 0x22, 0x00, 0x0F, 0x20, 0x02,
	0x08, 0x10, 0x00, 0x00, 0x20, 0x00,
	0x00, 0x88, 0x10, 0x10, 0x00, 0x37,
	0x00, 0x00, 0x00, 0x01, 0x02, 0x03,
	0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
	0x0A, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
	0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
	0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
	0x3C, 0x64, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};

/**
 * gt801x_read_config - Read the embedded configuration of the panel
 *
 * @ts: our gt801x_ts_data pointer
 *
 * Must be called during probe
 */
static void gt801x_read_config(struct gt801x_ts_data *ts)
{
	u8 config[GT801X_CONFIG_MAX_LENGTH];
	u8 screen_flag;
	int error;
	error = gt801x_i2c_read(ts->client, 0x52,
				&screen_flag, 1);

	dev_info(&ts->client->dev,
		 "Screen flag: %d\n", screen_flag);

	if ((screen_flag == 2) || (screen_flag == 3)) {
		dev_dbg(&ts->client->dev,
			"Applying fixups for screen type\n");

		if (screen_flag == 2)
			error = gt801x_i2c_write(ts->client, cfg_info_group3,
				sizeof(cfg_info_group3));
		else
			error = gt801x_i2c_write(ts->client, cfg_info_group4,
				sizeof(cfg_info_group4));

		if (error) {
			dev_err(&ts->client->dev,
				"Failed to write screen fixups, error: %d\n", error);
		}
	}

	error = gt801x_i2c_read(ts->client, GT801X_CONFIG_DATA,
				config,
				GT801X_CONFIG_MAX_LENGTH);
	if (error) {
		dev_warn(&ts->client->dev,
			 "Error reading config (%d), using defaults\n",
			 error);
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
		ts->int_trigger_type = GOODIX_INT_TRIGGER;
		ts->max_touch_num = GOODIX_MAX_CONTACTS;
		return;
	}

	ts->abs_x_max = get_unaligned_be16(&config[RESOLUTION_LOC]);
	ts->abs_y_max = get_unaligned_be16(&config[RESOLUTION_LOC + 2]);
	ts->int_trigger_type = config[TRIGGER_LOC] & 0x03;
	ts->max_touch_num = config[MAX_CONTACTS_LOC] & 0x0f;
	if (!ts->abs_x_max || !ts->abs_y_max || !ts->max_touch_num) {
		dev_err(&ts->client->dev,
			"Invalid config, using defaults\n");
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
		ts->max_touch_num = GOODIX_MAX_CONTACTS;
	}

	dev_info(&ts->client->dev,
		 "Config: %d x %d, max contacts: %d, trigger type: %d\n",
		 ts->abs_x_max, ts->abs_y_max,
		 ts->max_touch_num, ts->int_trigger_type);

}

/**
 * gt801x_read_version - Read GT801 2+1 touchscreen version
 *
 * @client: the i2c client
 * @version: output buffer containing the version on success
 * @id: output buffer containing the id on success
 */
static int gt801x_read_version(struct i2c_client *client, u16 *version, u16 *id)
{
	int error;
	u8 buf[16];

	error = gt801x_i2c_read(client, GT801X_REG_ID, buf, sizeof(buf));
	if (error) {
		dev_err(&client->dev, "read version failed: %d\n", error);
		return error;
	}
	/* TODO: version info contains 'GT801NI_3R15_1AV' */
	print_hex_dump_bytes("", DUMP_PREFIX_NONE, buf, ARRAY_SIZE(buf));
	*id = 0x802;
	*version = 0x15;
	dev_info(&client->dev, "ID %d, version: %04x\n", *id, *version);
	return 0;
}

/**
 * gt801x_i2c_test - I2C test function to check if the device answers.
 *
 * @client: the i2c client
 */
static int gt801x_i2c_test(struct i2c_client *client)
{
	int retry = 0;
	int error;
	u8 test;

	while (retry++ < 5) {
		error = gt801x_i2c_read(client, GT801X_CONFIG_DATA,
					&test, 1);
		if (!error)
			return 0;

		dev_err(&client->dev, "i2c test failed attempt %d: %d\n",
			retry, error);
		msleep(20);
	}

	return error;
}

/**
 * gt801x_request_input_dev - Allocate, populate and register the input device
 *
 * @ts: our gt801x_ts_data pointer
 * @version: device firmware version
 * @id: device ID
 *
 * Must be called during probe
 */
static int gt801x_request_input_dev(struct gt801x_ts_data *ts,
					u16 version, u16 id)
{
	int error;

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			     0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			     0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	/* Try overriding touchscreen parameters via device properties */
	touchscreen_parse_properties(ts->input_dev, true, &ts->prop);

	input_mt_init_slots(ts->input_dev, ts->max_touch_num,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);

	ts->input_dev->name = "Goodix Capacitive TouchScreen (GT802)";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0416;
	ts->input_dev->id.product = id;
	ts->input_dev->id.version = version;

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to register input device: %d", error);
		return error;
	}

	return 0;
}

/**
 * goodix_reset_no_int_sync - Reset device, leaving interrupt line in output mode
 *
 * @ts: gt801x_ts_data pointer
 */
static int goodix_reset_no_int_sync(struct gt801x_ts_data *ts)
{
	int error;

	dev_info(&ts->client->dev, "Resetting controller\n");

	/* begin select I2C slave addr */
	error = gpiod_direction_output(ts->reset_gpio, 0);
	if (error)
		goto error;

	msleep(20);				/* T2: > 10ms */

	/* HIGH: 0x28/0x29, LOW: 0xBA/0xBB */
//	 error = goodix_irq_direction_output(ts, ts->client->addr == 0x14);
//	 if (error)
//		 goto error;

	usleep_range(100, 2000);		/* T3: > 100us */

	error = gpiod_direction_output(ts->reset_gpio, 1);
	if (error)
		goto error;

	usleep_range(6000, 10000);		/* T4: > 5ms */

	return 0;

error:
	dev_err(&ts->client->dev, "Controller reset failed.\n");
	return error;
}

 
static int goodix_int_sync(struct gt801x_ts_data *ts)
{
	int error;

//	 error = goodix_irq_direction_output(ts, 0);
//	 if (error)
//		 goto error;

	msleep(50);				/* T5: 50ms */

//	error = goodix_irq_direction_input(ts);
//	if (error)
//		goto error;

	return 0;

// error:
	dev_err(&ts->client->dev, "Controller irq sync failed.\n");
	return error;
}

/**
 * goodix_reset - Reset device during power on
 *
 * @ts: goodix_ts_data pointer
 */
static int goodix_reset(struct gt801x_ts_data *ts)
{
	int error;

	dev_dbg(&ts->client->dev, "reset requested\n");

	error = goodix_reset_no_int_sync(ts);
	if (error)
		return error;

	return goodix_int_sync(ts);
}


static int gt801x_ts_probe(struct i2c_client *client)
{
	struct gt801x_ts_data *ts;
	unsigned long irq_flags;
	int error;
	u16 version_info, id_info;

	dev_dbg(&client->dev, "I2C Address: 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C check functionality failed.\n");
		return -ENXIO;
	}

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	struct device *dev;

	if (!ts->client)
		return -EINVAL;
	dev = &ts->client->dev;

	/* Get the interrupt GPIO pin number */
	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset-gpio", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio)) {
		dev_err(&client->dev, "Failed to get reset-gpio GPIO, error %d\n", error);
		return error;
	}

	goodix_reset(ts);

	error = gt801x_i2c_test(client);
	if (error) {
		dev_err(&client->dev, "I2C communication failure: %d\n", error);
		return error;
	}

	error = gt801x_read_version(client, &version_info, &id_info);
	if (error) {
		dev_err(&client->dev, "Read version failed.\n");
		return error;
	}

	gt801x_read_config(ts);

	error = gt801x_request_input_dev(ts, version_info, id_info);
	if (error)
		return error;

	irq_flags = goodix_irq_flags[ts->int_trigger_type] | IRQF_ONESHOT;
	error = devm_request_threaded_irq(&ts->client->dev, client->irq,
					  NULL, gt801x_ts_irq_handler,
					  irq_flags, client->name, ts);
	if (error) {
		dev_err(&client->dev, "request IRQ failed: %d\n", error);
		return error;
	}

	return 0;
}

static const struct i2c_device_id gt801x_ts_id[] = {
	{ "GDIX1002:00", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, gt801x_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id gt801x_of_match[] = {
	{ .compatible = "goodix,gtm802" },
	{ }
};
MODULE_DEVICE_TABLE(of, gt801x_of_match);
#endif

static struct i2c_driver gt801x_ts_driver = {
	.probe = gt801x_ts_probe,
	.id_table = gt801x_ts_id,
	.driver = {
		.name = "gtm802",
		.of_match_table = of_match_ptr(gt801x_of_match),
	},
};
module_i2c_driver(gt801x_ts_driver);

MODULE_AUTHOR("Santiago Hormazabal <santiagohssl@gmail.com>");
MODULE_AUTHOR("Priit Laes <plaes@plaes.org>");
MODULE_DESCRIPTION("Goodix GT802 touchscreen driver");
MODULE_LICENSE("GPL v2");
