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

#ifndef __DHT11_H__
#define __DHT11_H__

#include <rtthread.h>
#include "sensor.h"

#define CONNECT_SUCCESS  0
#define CONNECT_FAILED   1

struct dht11_device
{
    rt_base_t pin;
    rt_mutex_t lock;
};
typedef struct dht11_device *dht11_device_t;

uint8_t dht11_init(rt_base_t pin);
int32_t dht11_get_temperature(rt_base_t pin);
int rt_hw_dht11_init(const char *name, struct rt_sensor_config *cfg);

#endif /* __DS18B20_H_ */


