#include "main.h"

DECLARE_WAIT_QUEUE_HEAD(GPIO_BMP085_WQ_HEAD);


static const struct kernel_param_ops kp_ops = 
{
	.set = 0,						/* do not permit set of parameters */
	.get = &get_gpio_bmp085_value
};

static struct i2c_device_id gpio_bmp085_idtable[] =
{
	{"gpio_bmp085", GPIO_BMP085_DEVICE_ID},
	{ }
};

static struct i2c_driver gpio_bmp085_driver =
{
	.driver =
	{
		.name = "gpio_bmp085"
	},
	.id_table = gpio_bmp085_idtable
};

static struct i2c_board_info gpio_bmp085_board_info =
{
	.type = "gpio_bmp085",
	.addr = BMP085_ADDR
};

MODULE_DEVICE_TABLE(i2c, gpio_bmp085_idtable);

/* Initialize bmp085 device for usage
*  on success returns 0, on failure returns error code */
static int gpio_bmp085_init(void)
{
	int ret = i2c_add_driver(&gpio_bmp085_driver);

	if(ret)
		return ret;

	init_state = add_driver;

	gpio_i2c_adapter = i2c_get_adapter(1);			/* get twi1 i2c bus */

	if(!gpio_i2c_adapter)
		printk(KERN_ALERT "i2c_adapter is null\n");

	gpio_bmp085_client = i2c_new_device(gpio_i2c_adapter, &gpio_bmp085_board_info);

	if(!gpio_bmp085_client)
	{
		gpio_bmp085_remove();
		printk(KERN_ALERT "i2c_new_device returns null\n");
		ret = -ENODEV;
	}

	init_state = new_device;

	return ret;
}

/* Remove bmp085 driver and clears after it */
static void gpio_bmp085_remove(void)
{
	switch(init_state)
	{
		case new_device:
			i2c_unregister_device(gpio_bmp085_client);
		case add_driver:
			i2c_del_driver(&gpio_bmp085_driver);
		default:
			break;
	}
	init_state = none;
}

static void bmp085_init_calibrations(void)
{
	u8 addr_msb[BMP085_CALIBRATIONS_ARR_LEN];
	u8 addr_lsb[BMP085_CALIBRATIONS_ARR_LEN];
	char buffer;
	u8 i;

	/* Initialize arrays of callibration registers addresses */
	for(i = 0; i < BMP085_CALIBRATIONS_ARR_LEN; i++)
	{
		if(i == 0)
			addr_msb[i] = BMP085_INIT_CALIBRATIONS_ADDR;
		else
			addr_msb[i] = addr_lsb[i - 1] + 1;

		addr_lsb[i] = addr_msb[i] + 1;
	}

	/* Reading data from callibration registers addresses */
	for(i = 0; i < BMP085_CALIBRATIONS_ARR_LEN; i++)
	{
		i2c_master_send(gpio_bmp085_client, &addr_msb[i], 1);
		i2c_master_recv(gpio_bmp085_client, &buffer, 1);

		calibrations_arr[i] = buffer << 8;

		i2c_master_send(gpio_bmp085_client, &addr_lsb[i], 1);
		i2c_master_recv(gpio_bmp085_client, &buffer, 1);

		calibrations_arr[i] |= buffer;

		if(calibrations_arr[i] == 0x0000 || calibrations_arr[i] == 0xFFFF)
			printk(KERN_ALERT "Callibrations init failed (callibration %d)\n", i);
	}
}

/* Request data from bmp085 
  reg_addr - sensor register addres which data is reuqired */
static void i2c_bmp085_request_reg_data(u8 reg_addr)
{
	char buffer[2];

	buffer[0] = BMP085_MEASURE_REG_ADDR;
	buffer[1] = reg_addr;
	i2c_master_send(gpio_bmp085_client, buffer, 2);
}

/* Set wait timeout
  timeout - timeout in ms */
static void inline bmp085_set_wait_timeout(u8 delay_ms)
{
	ulong timeout;

	timeout = HZ / 1000 * delay_ms;					/* HZ / 1000 * 5 = 5ms */
	wait_event_interruptible_timeout(GPIO_BMP085_WQ_HEAD, 0, timeout);
}

/* Read requested data from the register
* Returns 2 byte data from the sensor register */
static u16 i2c_read_reg_data(void)
{
	char buffer;
	u16 ret;

	buffer = BMP085_CONV_RESULT_MSB;
	i2c_master_send(gpio_bmp085_client, &buffer, 1);
	i2c_master_recv(gpio_bmp085_client, &buffer, 1);

	ret = buffer << 8;

	buffer = BMP085_CONV_RESULT_LSB;
	i2c_master_send(gpio_bmp085_client, &buffer, 1);
	i2c_master_recv(gpio_bmp085_client, &buffer, 1);

	ret |= buffer;

	return ret;
}

/* Recieve temperature from bmp085 sensor */
static void i2c_bmp085_get_temp(void)
{
	i2c_bmp085_request_reg_data(BMP085_TEMP_CTRL_ADDR);

	bmp085_set_wait_timeout(5);

	bmp085_temp_value = i2c_read_reg_data();
}

/* Calculate temperature real value */
static uint bmp085_calc_temp_value(void)
{
	s32 x1, x2;

	x1 = (((s32)bmp085_temp_value - (s32)calibrations_arr[5])*(s32)calibrations_arr[4]) >> 15;
	x2 = ((s32)((s16)calibrations_arr[9]) << 11)/(x1 + (s32)calibrations_arr[10]);

	b5 = x1 + x2;

	return (b5 + 8) >> 4;
}

/* Recieve pressure value from bmp085 sensor */
static void i2c_bmp085_get_pres(void)
{
	i2c_bmp085_request_reg_data(BMP085_PRES0_CTRL_ADDR);

	bmp085_set_wait_timeout(5);

	bmp085_pres_value = i2c_read_reg_data();
}

/* Calculate real pressure value */
static uint bmp085_calc_pres_value(void)
{
	s32 b3, b6;
	u32 b4, b7;
	s32 x1, x2, x3;
	uint p;

	b6 = b5 - 4000;

	x1 = ((s32)((s16)calibrations_arr[7]) * (b6 * b6 >> 12)) >> 11;
	x2 = (s32)((s16)calibrations_arr[1]) * b6 >> 11;
	x3 = x1 + x2;
	b3 = ((s32)((s16)calibrations_arr[0]) * 4 + x3) >> 2;

	x1 = (s32)((s16)calibrations_arr[2]) * b6 >> 13;
	x2 = ((s32)((s16)calibrations_arr[6]) * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (s32)calibrations_arr[3] * (u32)(x3 + 32768) >> 15;

	b7 = ((u32)23843 - b3) * 50000;

	if(b7 < 0x80000000)
		p = (b7 << 1) / b4;
	else
		p = (b7 / b4) << 1;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;

	p = p + ((x1 + x2 + 3791) >> 4);

	return p;
}

static int get_gpio_bmp085_value(char *buffer, const struct kernel_param *kp)
{
	int ret;

	i2c_bmp085_get_temp();
	bmp085_temp = bmp085_calc_temp_value(); 
	ret = param_get_uint(buffer, kp);

	if(ret >= 0 && strcmp(kp->name, "bmp085_pres") == 0)
	{
		i2c_bmp085_get_pres();
		bmp085_pres = bmp085_calc_pres_value();
		ret = param_get_uint(buffer, kp);
	}

	if(ret < 0)
	{
		printk(KERN_INFO "Getting gpio  bmp085 value failed %d\n", ret);
	}

	return ret;
}

/* Init function called when module is loaded
** Returns 0, when successfully loaded, otherwise nonzero will be returned */
static int __init gpio_bmp085_module_init(void)
{
	int err = 0;

	err = gpio_bmp085_init();

	if(err)
	{
		printk(KERN_ALERT "GPIO_BMP085 initialization failed %d\n", err);
		return err;
	}

	bmp085_init_calibrations();
	printk(KERN_ALERT "GPIO_BMP085 module init\n");

	return 0;
}

/* Exit function, called when module is removed from memory */
static void __exit gpio_bmp085_module_exit(void)
{
	gpio_bmp085_remove();
	printk(KERN_ALERT "GPIO_BMP085 module unloaded\n");
}

module_init(gpio_bmp085_module_init);			/* Set entry point for kernel module (module_init is a macro) */
module_exit(gpio_bmp085_module_exit);			/* Set exit point from a module (module_exit is a macro) */

/* Show variable in sysfs */
/* Set module param with callback function */
/* 1 - file name
*  2 - pointer to kernel_param_ops structure
*  3 - pointer to a variable containing parameter value
*  4 - permissions on file */

/* Show temperature value */
module_param_cb(bmp085_temp, &kp_ops, &bmp085_temp, 0660);
MODULE_PARM_DESC(bmp085_temp, "Converted temperature value from bmp085 sensor");

/* Show pressure value */
module_param_cb(bmp085_pres, &kp_ops, &bmp085_pres, 0660);
MODULE_PARM_DESC(bmp085_pres, "Converted pressure value from bmp085 sensor");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("mks-a");
MODULE_DESCRIPTION("This module will read temperature and pressure data from BMP085 sensor and show it through sysfs.");