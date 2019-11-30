/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author          Notes
 * 2019-08-01     LuoGong         the first version.
 * 2019-08-15     MurphyZhao      add lock and modify code style
 *
 */

#include "sensor_dallas_dht11.h"
#include <rtdevice.h>
#include <rthw.h>
#include "sensor.h"
#include "board.h"
#include <stdint.h>

#define SENSOR_DEBUG
#define DBG_TAG               "sensor.dht11"

#ifdef SENSOR_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_ERROR
#endif /* SENSOR_DEBUG */
#include <rtdbg.h>

#define SENSOR_TEMP_RANGE_MAX (100)
#define SENSOR_TEMP_RANGE_MIN (0)
#define SENSOR_HUMI_RANGE_MAX (100)
#define SENSOR_HUMI_RANGE_MIN (0)

#ifndef RT_USING_PIN
#error "Please enable RT_USING_PIN"
#endif

#ifndef RT_SENSOR_VENDOR_DALLAS
#define RT_SENSOR_VENDOR_DALLAS (7u)
#endif

#ifndef rt_hw_us_delay
RT_WEAK void rt_hw_us_delay(rt_uint32_t us)
{
    rt_uint32_t delta;

    us = us * (SysTick->LOAD / (1000000 / RT_TICK_PER_SECOND));
    delta = SysTick->VAL;

    while (delta - SysTick->VAL < us) continue;
}
#endif

static void dht11_reset(rt_base_t pin)
{
    rt_pin_mode(pin, PIN_MODE_OUTPUT);

    rt_pin_write(pin, PIN_LOW);
    rt_thread_mdelay(20);               /* 20ms */

    rt_pin_write(pin, PIN_HIGH);
    rt_hw_us_delay(30);                 /* 30us*/
}

static uint8_t dht11_check(rt_base_t pin)
{
    uint8_t retry = 0;
    rt_pin_mode(pin, PIN_MODE_INPUT);

    while (rt_pin_read(pin) && retry < 100)
    {
        retry++;
        rt_hw_us_delay(1);
    }

    if(retry >= 100)
    {
        return CONNECT_FAILED;
    }

    retry = 0;
    while (!rt_pin_read(pin) && retry < 100)
    {
        retry++;
        rt_hw_us_delay(1);
    };

    if(retry >= 100)
    {
        return CONNECT_FAILED;
    }

    return CONNECT_SUCCESS;
}

static uint8_t dht11_read_bit(rt_base_t pin)
{
	uint8_t retry = 0;
	while (rt_pin_read(pin) && retry < 100)
    {
        retry++;
        rt_hw_us_delay(1);
    }
	retry = 0;

	while (!rt_pin_read(pin) && retry < 100)
    {
        retry++;
        rt_hw_us_delay(1);
    }

	rt_hw_us_delay(40);
	if(rt_pin_read(pin))
		return 1;
    return 0;
}

static uint8_t dht11_read_byte(rt_base_t pin)
{
    uint8_t i, dat = 0;

    for (i = 1; i <= 8; i++)
    {
        dat <<= 1;
		dat |= dht11_read_bit(pin);
    }

    return dat;
}

static uint8_t dht11_read_Data(rt_base_t pin,uint8_t *temp,uint8_t *humi)
{
	uint8_t i, buf[5];
	dht11_reset(pin);

	if(dht11_check(pin) == 0)
	{
		for(i=0; i<5; i++) /* read 40 bits */
		{
			buf[i] = dht11_read_byte(pin);
		}

		if((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
		{
			*humi = buf[0];
			*temp = buf[2];
		}
	}
    else
    {
        return 1;
    }

	return 0;	
}

uint8_t dht11_init(rt_base_t pin)
{
    uint8_t ret = 0;

    dht11_reset(pin);
    ret = dht11_check(pin);
    if (ret != 0)
    {
        dht11_reset(pin);
        ret = dht11_check(pin);
    }

    return ret;
}

int32_t dht11_get_temperature(rt_base_t pin)
{
    static int32_t temOLD = 0;
    uint8_t humi=0, temp = 0;
    int32_t temNEW;

    dht11_read_Data(pin, &temp, &humi);

    temNEW = (humi << 16)|(temp<<0);

    if((temNEW != temOLD) && (temNEW !=0))
    {
        temOLD = temNEW;
    }
    return temOLD;
}

static rt_size_t dht11_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    rt_int32_t temperature_humidity;
    temperature_humidity = dht11_get_temperature((rt_base_t)sensor->config.intf.user_data);
    data->data.temp = temperature_humidity;
    data->timestamp = rt_sensor_get_ts();
    return 1;
}

static rt_size_t dht11_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return dht11_polling_get_data(sensor, buf);
    }

    return 0;
}

static rt_err_t dht11_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    return RT_EOK;
}

static struct rt_sensor_ops sensor_ops =
{
    dht11_fetch_data,
    dht11_control
};

static struct rt_sensor_device dht11_dev;
int rt_hw_dht11_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_err_t result = RT_EOK;
    rt_sensor_t sensor = &dht11_dev;

    rt_memset(sensor, 0x0, sizeof(struct rt_sensor_device));

    if (!dht11_init((rt_base_t)cfg->intf.user_data))
    {
        sensor->module = rt_calloc(1, sizeof(struct rt_sensor_module));
        if (sensor->module == RT_NULL)
        {
            LOG_E("Memory error.");
            result = -RT_ENOMEM;
            goto __exit;
        }

        sensor->info.type       = RT_SENSOR_CLASS_TEMP ;
        sensor->info.vendor     = RT_SENSOR_VENDOR_DALLAS;
        sensor->info.model      = "DHT11";
        sensor->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
        sensor->info.intf_type  = RT_SENSOR_INTF_ONEWIRE;
        sensor->info.range_max  = SENSOR_TEMP_RANGE_MAX;
        sensor->info.range_min  = SENSOR_TEMP_RANGE_MIN;
        sensor->info.period_min = 100; /* Read ten times in 1 second */

        sensor->config = *cfg;
        sensor->ops = &sensor_ops;

        /* dht11 sensor register */
        result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDONLY, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }
    else
    {
        LOG_E("dht11 init failed");
        result = -RT_ERROR;
        goto __exit;
    }
    return RT_EOK;

__exit:
    if (sensor->module)
        rt_free(sensor->module);
    return result;
}
