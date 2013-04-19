/*
 * LED Core
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/

#ifndef __LEDS_H_INCLUDED
#define __LEDS_H_INCLUDED

#include <linux/device.h>
#include <linux/rwsem.h>
#include <linux/leds.h>
/* FUJITSU:2011-12-01 add LED start */
extern int g_chrg_flag;
extern int g_chrg_count;
/* FUJITSU:2011-12-01 add LED end */

static inline void led_set_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	if (value > led_cdev->max_brightness)
		value = led_cdev->max_brightness;
/* FUJITSU:2011-12-01 mod LED start */
#if 0
	led_cdev->brightness = value;
#else
	if(g_chrg_count != 2){
		led_cdev->brightness = value;
	}
#endif
	if (!(led_cdev->flags & LED_SUSPENDED)) {
			led_cdev->brightness_set(led_cdev, value);
	}
	if (!(led_cdev->flags & LED_SUSPENDED))
		led_cdev->brightness_set(led_cdev, value);
}

static inline int led_get_brightness(struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;

#ifdef CONFIG_LEDS_TRIGGERS
void led_trigger_set_default(struct led_classdev *led_cdev);
void led_trigger_set(struct led_classdev *led_cdev,
			struct led_trigger *trigger);
void led_trigger_remove(struct led_classdev *led_cdev);

static inline void *led_get_trigger_data(struct led_classdev *led_cdev)
{
	return led_cdev->trigger_data;
}

#else
#define led_trigger_set_default(x) do {} while (0)
#define led_trigger_set(x, y) do {} while (0)
#define led_trigger_remove(x) do {} while (0)
#define led_get_trigger_data(x) (NULL)
#endif

ssize_t led_trigger_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t led_trigger_show(struct device *dev, struct device_attribute *attr,
			char *buf);
/* FUJITSU:2011-12-01 add LED start */
ssize_t led_notify_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t led_notify_show(struct device *dev, struct device_attribute *attr,
			char *buf);
ssize_t led_color_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t led_color_show(struct device *dev, struct device_attribute *attr,
			char *buf);
/* FUJITSU:2011-12-01 add LED end */

/* FUJITSU:2011-12-01 add AN32155AB start */
#ifdef CONFIG_LEDS_AN32155AB
ssize_t led_modeset_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t led_modeset_show(struct device *dev, struct device_attribute *attr,
			char *buf);
#endif
/* FUJITSU:2011-12-01 add AN32155AB end */
#endif	/* __LEDS_H_INCLUDED */
