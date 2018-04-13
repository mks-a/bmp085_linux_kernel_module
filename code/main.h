#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/moduleparam.h>
#include <linux/wait.h>
#include <linux/string.h>
#include "../gpio_bmp085/gpio_bmp085.h"

#define GPIO_BMP085_DEVICE_ID 0x0123
#define GPIO_BMP085_WQ_HEAD gpio_bmp085_wq_head		/* Wait queue head name */

static s32 b5;

static struct i2c_adapter *gpio_i2c_adapter;
static struct i2c_client *gpio_bmp085_client;

enum init_states {none, add_driver, new_device};

static enum init_states init_state = none;

/* Temperature values */
static u16 bmp085_temp_value = 0;		/* Temperature value from bmp085 sensor */
static uint bmp085_temp;			/* Converted temperature value in C */

/* Pressure values */
static u16 bmp085_pres_value = 0;		/* Pressure value from sensor */
static uint bmp085_pres;			/* Converted pressure value in Pa */

static u16 calibrations_arr[BMP085_CALIBRATIONS_ARR_LEN];

static void gpio_bmp085_remove(void);
static int get_gpio_bmp085_value(char *buffer, const struct kernel_param *kp);