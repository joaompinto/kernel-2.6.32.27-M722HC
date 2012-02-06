/*
 * vibrate_gpio.h
 *
 * Overview:  
 *
 * Copyright (c) 2011, YiFang Digital
 *
 * Version:  1.0
 * Created:  08/03/2011 03:43:44 PM
 * Author:  zqqu <zqqu@yifangdigital.com>
 * Company:  YiFang Digital
 * History:
 *
 * 
 */

#ifndef _LINUX_VIB_GPIO_H
#define _LINUX_VIB_GPIO_H


#define VIB_GPIO_NAME "vib-gpio"

struct vib_gpio_platform_data {
	unsigned        gpio;
	int             max_timeout;
	u8              active_low;
	int             initial_vibrate;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

#endif 

