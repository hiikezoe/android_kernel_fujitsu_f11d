/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2011 Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
/* FUJITSU:2011-12-1 add VIB start */
#include <../../../drivers/staging/android/timed_output.h>
/* FUJITSU:2011-12-1 add VIB end */
#include <linux/sched.h>
#include "pmic.h"
#include "timed_output.h"

#include <mach/msm_rpcrouter.h>

#define PM_LIBPROG      0x30000061
/* FUJITSU:2011-12-1 mod VIB start */
#if 0
#if (CONFIG_MSM_AMSS_VERSION == 6220) || (CONFIG_MSM_AMSS_VERSION == 6225)
#define PM_LIBVERS      0xfb837d0b
#else
#define PM_LIBVERS      0x10001
#endif
#else
#define PM_LIBVERS      0x30005
#endif
/* FUJITSU:2011-12-1 mod VIB end */

/* FUJITSU:2011-12-1 mod VIB start */
#if 0
#define HTC_PROCEDURE_SET_VIB_ON_OFF	21
#else
#define HTC_PROCEDURE_SET_VIB_ON_OFF	22
#endif
/* FUJITSU:2011-12-1 mod VIB end */
#define PMIC_VIBRATOR_LEVEL	(3000)

static struct work_struct work_vibrator_on;

/* FUJITSU:2011-12-1 mod VIB start	*/
#if 0
static struct work_struct work_vibrator_off;
#else
static struct delayed_work work_vibrator_off;
#endif
/* FUJITSU:2011-12-1 mod VIB  end	*/

static struct hrtimer vibe_timer;

/* FUJITSU:2011-12-1 add VIB start	*/
static int vibrator_value;
static struct mutex vib_mutex;
static void timed_vibrator_off(struct timed_output_dev *sdev, int delay);
static void timed_vibrator_on(struct timed_output_dev *sdev);
static int vibrator_get_time(struct timed_output_dev *dev);
/* FUJITSU:2011-12-1 add VIB  end	*/
#define TIMED_VIBRATOR_FLAG 1
#if TIMED_VIBRATOR_FLAG
static int timed_vibrator_flag;
#endif
/* FUJITSU:2011-12-1 add VIB start	*/
static int vibrator_timer_active(void)
{
	int ret = 0;
	mutex_lock(&vib_mutex);
	if (hrtimer_active(&vibe_timer)) {
		ret = 1;
	}
	mutex_unlock(&vib_mutex);
	return ret;
}
/* FUJITSU:2011-12-1 add VIB  end	*/

#ifdef CONFIG_PM8XXX_RPC_VIBRATOR
static void set_pmic_vibrator(int on)
{
	int rc;

	rc = pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__MANUAL);
	if (rc) {
		pr_err("%s: Vibrator set mode failed", __func__);
		return;
	}

	if (on)
		rc = pmic_vib_mot_set_volt(PMIC_VIBRATOR_LEVEL);
	else
		rc = pmic_vib_mot_set_volt(0);

	if (rc)
		pr_err("%s: Vibrator set voltage level failed", __func__);
}
#else
static void set_pmic_vibrator(int on)
{
/* FUJITSU:2011-12-1 add VIB start */
	int rc = 0;
/* FUJITSU:2011-12-1 add VIB end */
	static struct msm_rpc_endpoint *vib_endpoint;
	struct set_vib_on_off_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	if (!vib_endpoint) {
		vib_endpoint = msm_rpc_connect(PM_LIBPROG, PM_LIBVERS, 0);
		if (IS_ERR(vib_endpoint)) {
			printk(KERN_ERR "init vib rpc failed!\n");
			vib_endpoint = 0;
			return;
		}
	}

	if (on)
		req.data = cpu_to_be32(PMIC_VIBRATOR_LEVEL);
	else
		req.data = cpu_to_be32(0);

/* FUJITSU:2011-12-1 mod VIB start */
#if 0
	msm_rpc_call(vib_endpoint, HTC_PROCEDURE_SET_VIB_ON_OFF, &req,
		sizeof(req), 5 * HZ);
#else
	rc = msm_rpc_call(vib_endpoint, HTC_PROCEDURE_SET_VIB_ON_OFF, &req,
		sizeof(req), 5 * HZ);
	if (rc < 0)
		printk("%s:vibrator set failed! rc:%d req.data:%x\n", __func__, rc, req.data);
#endif
#if TIMED_VIBRATOR_FLAG
	timed_vibrator_flag = 0;
	printk("set_pmic_vibrator flag_off\n");
#endif
/* FUJITSU:2011-12-1 mod VIB end */
}
#endif

static void pmic_vibrator_on(struct work_struct *work)
{
/* FUJITSU:2011-12-1 mod VIB start	*/
#if 0
	set_pmic_vibrator(1);
#else
	hrtimer_start(&vibe_timer,ktime_set(vibrator_value / 1000, (vibrator_value % 1000) * 1000000),HRTIMER_MODE_REL);
	set_pmic_vibrator(1);
	if (!vibrator_timer_active()) {
		/* No timer active, there may in infinite vibration, so switch off */
		timed_vibrator_off(NULL, 0);
	}
#endif
/* FUJITSU:2011-12-1 mod VIB  end	*/
}

static void pmic_vibrator_off(struct work_struct *work)
{
	set_pmic_vibrator(0);
/* FUJITSU:2011-12-1 add VIB start	*/
	if (vibrator_timer_active()) {
		/* Make it on, timer will switch it off */
		set_pmic_vibrator(1);
	}
/* FUJITSU:2011-12-1 add VIB  end	*/
}

static void timed_vibrator_on(struct timed_output_dev *sdev)
{
#if TIMED_VIBRATOR_FLAG
	if(timed_vibrator_flag == 1){
	printk("timed_vibrator_on schedule cancel\n");
		return;
	}
	timed_vibrator_flag = 1;
	printk("timed_vibrator_on on\n");
#endif
	schedule_work(&work_vibrator_on);

}

/* FUJITSU:2011-12-1 mod VIB start	*/
#if 0
static void timed_vibrator_off(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_off);
}
#else
static void timed_vibrator_off(struct timed_output_dev *sdev, int delay)
{
#if TIMED_VIBRATOR_FLAG
	timed_vibrator_flag = 1;
	printk("timed_vibrator_off on\n");
#endif
	schedule_delayed_work(&work_vibrator_off, msecs_to_jiffies(delay));
}
#endif
/* FUJITSU:2011-12-1 mod VIB  end	*/

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
/* FUJITSU:2011-12-1 mod VIB start	*/
#if 0
	hrtimer_cancel(&vibe_timer);

	if (value == 0)
		timed_vibrator_off(dev);
	else {
		value = (value > 15000 ? 15000 : value);

		timed_vibrator_on(dev);

		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
#else
	int vibrator_time = 0;
	if (value == 0) {
		if (vibrator_timer_active()) {
			vibrator_time = vibrator_get_time(dev);
			printk("[VIB] vibrator Off(time[%d])\n", vibrator_time );
			/* Put a delay of 10 ms, in case the vibrator is off immediately */
			if (vibrator_time < 10 || vibrator_time > 50){
				hrtimer_cancel(&vibe_timer);
				vibrator_time = 0;
			}

			vibrator_value = value;
#if TIMED_VIBRATOR_FLAG
			if(timed_vibrator_flag == 1){
				printk("timed_vibrator_enable off schedule cancel\n");
				return;
			}
#endif
			timed_vibrator_off(dev, vibrator_time + 10);
		}
	}
	else {
		printk("[VIB] vibrator On(time[%d])\n", value );
        /* Provisional management                                        */
        /* Normal operation is not carried out in case of a small value. */
        /* For this reason, 10 ms is used when smaller than 10 ms.       */
		if((0 < value )&&(value < 10)){
			value = 10;
		}
		vibrator_value = (value > 15000 ? 15000 : value);
		hrtimer_cancel(&vibe_timer);
		timed_vibrator_on(dev);
	}
#endif
/* FUJITSU:2011-12-1 mod VIB end */
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	}
	return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
/* FUJITSU:2011-12-1 mod VIB start	*/
#if 0
	timed_vibrator_off(NULL);
#else
	printk("[VIB] vibrator_timer_func call\n");
	timed_vibrator_off(NULL,0);
#endif
/* FUJITSU:2011-12-1 mod VIB  end	*/
	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

void __init msm_init_pmic_vibrator(void)
{
/* FUJITSU:2011-12-1 mod VIB start	*/
#if 0
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);
#else
	mutex_init(&vib_mutex);
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_DELAYED_WORK(&work_vibrator_off, pmic_vibrator_off);
#endif
/* FUJITSU:2011-12-1 mod VIB  end	*/

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;
#if TIMED_VIBRATOR_FLAG
	timed_vibrator_flag = 0;
#endif
	timed_output_dev_register(&pmic_vibrator);
}

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

