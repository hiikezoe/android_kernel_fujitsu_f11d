/*
 *  stk_i2c_ps31xx_switch.c - Linux kernel modules for proximity/ambient light sensor
 *  (Polling + Switch Mode)
 *
 *  Copyright (C) 2011 Patrick Chang / SenseTek <patrick_chang@sitronix.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include "../touchpanel/ft5x06_ts.h"
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#include "stk_i2c_ps31xx.h"
#include "stk_defines.h"
#include "stk_lk_defs.h"

#define STKALS_DRV_NAME	"stk_als"
#define STKPS_DRV_NAME "stk_ps"
#define DEVICE_NAME		"stk-oss"
#define DRIVER_VERSION  STK_DRIVER_VER
#define LightSensorDevName "stk_als"
#define ProximitySensorDevName "stk_ps"
#define EINT_GPIO 180

#define ALS_ODR_DELAY (1000/CONFIG_STK_ALS_SAMPLING_RATE)
#define PS_ODR_DELAY (1000/CONFIG_STK_PS_SAMPLING_RATE)

#define stk_writeb i2c_smbus_write_byte_data
#define stk_readb i2c_smbus_read_byte_data

#define stk_writew i2c_smbus_write_word_data
#define stk_readw i2c_smbus_read_word_data

#define STK_LOCK0 mutex_unlock(&stkps_io_lock)
#define STK_LOCK1 mutex_lock(&stkps_io_lock)

static int32_t init_all_setting(void);
static int32_t enable_als(uint8_t enable);
static int32_t software_reset(void);

static int32_t set_als_it(uint8_t it);
static int32_t set_als_gain(uint8_t gain);
static int32_t set_ps_it(uint8_t it);
static int32_t set_ps_slp(uint8_t slp);
static int32_t set_ps_led_driving_current(uint8_t irdr);
static int32_t set_ps_gc(uint8_t gc);

static int32_t set_ps_thd_l(uint8_t thd_l);
static int32_t set_ps_thd_h(uint8_t thd_h);
inline int32_t get_status_reg(void);

static int als_polling_function(void* arg);
static struct task_struct *als_polling_tsk=NULL;

static struct mutex stkps_io_lock;
static struct completion als_thread_completion;
static struct stkps31xx_data* pStkPsData = NULL;
static struct wake_lock proximity_sensor_wakelock;

#ifdef CONFIG_STK_PS_ENGINEER_TUNING
static uint8_t ps_code_low_thd;
static uint8_t ps_code_high_thd;
#endif

#ifdef CONFIG_STK_ALS_TRANSMITTANCE_TUNING

static int32_t als_transmittance = CONFIG_STK_ALS_TRANSMITTANCE;
#endif //CONFIG_STK_ALS_TRANSMITTANCE_TUNING

extern int qci_read_hw_id(void);
int hwid = 0;
static int first = 1;
int AVGSF=100,sf0=1,sf1=1,sf2=1,threshold_h=CONFIG_STK_PS_CODE_HIGH_THRESHOLD,threshold_l=CONFIG_STK_PS_CODE_LOW_THRESHOLD;
int LED_CURRENT=STK_PS_IRLED_DRIVING_CURRENT, INTEGRAL_TIME=STK_PS_INTEGRAL_TIME, led_setting=9;
u8 vender_id = 0xAA, color_lens = 0x07;
int PS_INT_STATE = 1,ONCALL=0, WAKELOCK = 0;

static int ascii_to_int(uint32_t data)
{
	int i =0, result = 0;
	u8 tmp = 0;

	printk("hex: %x\n",data);
	for(i = 0; i < 4 ; i++){
		tmp = (data >> (24 - 8*i)) & 0xff;
		result = result*10 + (tmp -48);
	}
	printk("result: %d\n",result);
	return result;
}

inline uint32_t alscode2lux(uint32_t alscode)
{
   alscode += ((alscode<<7)+(alscode<<3)+(alscode>>1));     // 137.5
      //x1       //   x128         x8            x0.5
    alscode<<=3; // x 8 (software extend to 19 bits)
    // Gain & IT setting ==> x8
    // ==> i.e. code x 8800
    // Org : 1 code = 0.88 Lux
    // 8800 code = 0.88 lux --> this means it must be * 1/10000


#ifdef CONFIG_STK_ALS_TRANSMITTANCE_TUNING
    alscode/=als_transmittance;
#else
    alscode/=CONFIG_STK_ALS_TRANSMITTANCE;
#endif //CONFIG_STK_ALS_TRANSMITTANCE_TUNING

	alscode = alscode *AVGSF/100;
    return alscode;
}

static int32_t enable_ps_int(uint8_t enable){
	int ret = 0;

	printk(" == [ %s ] %d ==\n", __func__, enable);
if( enable != PS_INT_STATE){
	if(!enable){
		ret = disable_irq_wake(pStkPsData->irq);
		if (ret)
		{
			printk("%s: enable_irq_wake(%d,1) failed for (%d)\n",  __func__, pStkPsData->irq, ret);
		}
			disable_irq(pStkPsData->irq);
			printk("== %s == disable irq\n",__func__);
		gpio_tlmm_config(GPIO_CFG(EINT_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		pStkPsData->ps_cmd_reg = 0x41;
		ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
	}else{
		pStkPsData->ps_cmd_reg = 0x42;
		ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
		gpio_tlmm_config(GPIO_CFG(EINT_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		msleep(50);
		enable_irq(pStkPsData->irq);
		printk("== %s == enable irq\n",__func__);
//		printk(" === %s ===  int:%d\n",__func__,gpio_get_value(EINT_GPIO));
		ret = enable_irq_wake(pStkPsData->irq);
		if (ret)
		{
			printk("%s: enable_irq_wake(%d,1) failed for (%d)\n",  __func__, pStkPsData->irq, ret);
		}
	}
	pStkPsData->bPSThreadRunning = enable;
	PS_INT_STATE = enable;
//	printk(" == [ %s ] PS_INT_STATE: %d ==\n", __func__, PS_INT_STATE);
	}else
		WARNING("STK PS : PS running state doesn't change\n");
	return ret;
}

void set_oncall(int value)
{
	ONCALL = value;
	printk("=== [ %s ] === ONCALL: %d \n",__func__,ONCALL);
}
EXPORT_SYMBOL(set_oncall);

static int32_t init_all_setting(void)
{
    if (software_reset()<0)
    {
        ERR("STK PS : error --> device not found\n");
        return 0;
    }

    enable_als(0);
    set_ps_slp(STK_PS_SLEEP_TIME);
    set_ps_gc(CONFIG_STK_PS_GAIN_SETTING);

    if(((vender_id == 0xAA) && (color_lens == 0x38)) || (led_setting < 0) || (led_setting > 7)){
	INTEGRAL_TIME = 0;
	LED_CURRENT = 0;
	printk("%s : [ %d, %d]\n",__func__,INTEGRAL_TIME,LED_CURRENT);
    }

	set_ps_it(INTEGRAL_TIME);
	set_ps_led_driving_current(LED_CURRENT);

    set_als_gain(0x01); // x2
    set_als_it(0x02); // x4
    //set_ps_thd_h(CONFIG_STK_PS_CODE_HIGH_THRESHOLD);
    //set_ps_thd_l(CONFIG_STK_PS_CODE_LOW_THRESHOLD);
    enable_ps_int(0);

    return 1;
}

static int32_t software_reset(void)
{
    // software reset and check stk 83xx is valid
    int32_t r;
    uint8_t w_reg;
    uint8_t org_reg;

    r = i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_STATUS_REG);
    if (r<0)
    {
        ERR("STK PS software reset: read i2c error\n");
        return r;
    }
    if ((r&STK_PS_STATUS_ID_MASK)!=STK_PS31xx_ID)
    {
        ERR("STK PS : invalid ID number");
        return -EINVAL;
    }
    r = i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_GC_REG);
    if (r<0)
    {
        ERR("STK PS software reset: read i2c error\n");
        return r;
    }
    org_reg = (uint8_t)(r&0xf0);
    w_reg = ~((uint8_t)(r&0xff));
    r = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_GC_REG,w_reg);
    if (r<0)
    {
        ERR("STK PS software reset: write i2c error\n");
        return r;
    }
    r = i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_GC_REG);
    if (w_reg!=(uint8_t)(r&0xff))
    {
        ERR("STK PS software reset: read-back value is not  the same\n");
        return -1;
    }
    r = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_SOFTWARE_RESET_REG,0);
    msleep(5);
    if (r<0)
    {
        ERR("STK PS software reset: read error after reset\n");
        return r;
    }
    return 0;
}



static int32_t set_als_it(uint8_t it)
{
    pStkPsData->als_cmd_reg &= (~STK_ALS_CMD_IT_MASK);
    pStkPsData->als_cmd_reg |= (STK_ALS_CMD_IT_MASK & STK_ALS_CMD_IT(it));
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_ALS_CMD_REG,pStkPsData->als_cmd_reg);

}
static int32_t set_als_gain(uint8_t gain)
{
	if(gain >= 2)
	{
		ERR("STK PS : Error! als_gain >= 2\n");
		return -EINVAL;
	}
    pStkPsData->als_cmd_reg &= (~STK_ALS_CMD_GAIN_MASK);
    pStkPsData->als_cmd_reg |= (STK_ALS_CMD_GAIN_MASK & STK_ALS_CMD_GAIN(gain));
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_ALS_CMD_REG,pStkPsData->als_cmd_reg);
}
static int32_t set_ps_it(uint8_t it)
{
    pStkPsData->ps_cmd_reg &= (~STK_PS_CMD_IT_MASK);
    pStkPsData->ps_cmd_reg |= (STK_PS_CMD_IT_MASK & STK_PS_CMD_IT(it));
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
}
static int32_t set_ps_slp(uint8_t slp)
{
    pStkPsData->ps_cmd_reg &= (~STK_PS_CMD_SLP_MASK);
    pStkPsData->ps_cmd_reg |= (STK_PS_CMD_SLP_MASK & STK_PS_CMD_SLP(slp));
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);

}
static int32_t set_ps_led_driving_current(uint8_t irdr)
{
    pStkPsData->ps_cmd_reg &= (~STK_PS_CMD_DR_MASK);
    pStkPsData->ps_cmd_reg |= (STK_PS_CMD_DR_MASK & STK_PS_CMD_DR(irdr));
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
}
static int32_t set_ps_gc(uint8_t gc)
{
    int32_t retval;

    retval = i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_GC_REG);
    if (retval<0)
        return retval;
    pStkPsData->ps_gc_reg = (uint8_t)retval;
    pStkPsData->ps_gc_reg &= (~STK_PS_GC_GAIN_MASK);
    pStkPsData->ps_gc_reg |= (STK_PS_GC_GAIN(gc)&STK_PS_GC_GAIN_MASK);

    return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_GC_REG,pStkPsData->ps_gc_reg);
}


static int32_t set_ps_thd_l(uint8_t thd_l)
{
	int ret = 0;
	ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_THD_L_REG, thd_l);
	if( ret < 0 )
		ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_THD_L_REG, thd_l);
#ifdef CONFIG_STK_PS_ENGINEER_TUNING
    ps_code_low_thd = thd_l;
#endif
  return ret;
}
static int32_t set_ps_thd_h(uint8_t thd_h)
{
	int ret = 0;
	ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_THD_H_REG, thd_h);
	if( ret < 0 )
		ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_THD_H_REG, thd_h);
#ifdef CONFIG_STK_PS_ENGINEER_TUNING
    ps_code_high_thd = thd_h;
#endif
    return ret;
}

inline int32_t get_als_reading(void)
{
    int32_t word_data;
    int32_t lsb,msb;
    msb = i2c_smbus_read_byte_data(pStkPsData->client,STK_ALS_DT1_REG);
    lsb = i2c_smbus_read_byte_data(pStkPsData->client,STK_ALS_DT2_REG);
	if((msb < 0) || (lsb < 0)){
		word_data = pStkPsData->als_reading;
		printk("STK ALS : read data fail\n");
	}else
    word_data = (msb<<8) | lsb;
    return word_data;
}

inline int32_t get_ps_reading(void)
{
	int ret;
	ret = i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_READING_REG);
	if( ret < 0 ){
		printk("== %s == ret: %d\n",__func__,ret);
	}
    return ret;
}

inline void als_report_event(struct input_dev* dev,int32_t report_value)
{
	input_report_abs(dev, ABS_Y, report_value);
	input_sync(dev);
//	INFO("STK PS : als input event %d lux\n",report_value);
}

inline void update_and_check_report_als(int32_t als_code)
{
    int32_t lux_last,lux;
    lux_last = pStkPsData->als_lux_last;
    lux = alscode2lux(als_code);
    if (unlikely(abs(lux - lux_last)>=CONFIG_STK_ALS_CHANGE_THRESHOLD))
    {
        pStkPsData->als_lux_last = lux;
        als_report_event(pStkPsData->als_input_dev,lux);
    }
}

static int als_polling_function(void* arg)
{
    uint32_t delay;
    init_completion(&als_thread_completion);
    while (1)
    {

        STK_LOCK(1);
        delay = pStkPsData->als_delay;
        pStkPsData->als_reading = get_als_reading();
        update_and_check_report_als(pStkPsData->als_reading);
        if (pStkPsData->bALSThreadRunning == 0)
            break;
        STK_LOCK(0);
        msleep(delay);
    };

    STK_LOCK(0);
    complete(&als_thread_completion);

    return 0;
}

inline void ps_report_event(struct input_dev* dev,int32_t report_value)
{
    	pStkPsData->ps_distance_last = report_value;
	input_report_abs(dev, ABS_X, report_value);
	input_sync(dev);
//	INFO("STK PS : ps input event %d\n",report_value);
//	wake_lock_timeout(&proximity_sensor_wakelock, 2*HZ);
}

static int32_t enable_als(uint8_t enable)
{
    if (enable)
    {
        if (pStkPsData->bALSThreadRunning == 0)
        {
            pStkPsData->als_reading = 0;
            pStkPsData->bALSThreadRunning = 1;
            als_polling_tsk = kthread_run(als_polling_function,NULL,"als_polling");
        }
        else
        {
            WARNING("STK ALS : thread has running\n");
        }
    }
    else
    {

        if (pStkPsData->bALSThreadRunning)
        {
            pStkPsData->bALSThreadRunning = 0;
            STK_LOCK(0);
            wait_for_completion(&als_thread_completion);
            STK_LOCK(1);
            als_polling_tsk = NULL;
        }
    }
    printk(" == [ %s ] %d ==\n", __func__, enable);
    pStkPsData->als_cmd_reg &= (~STK_ALS_CMD_SD_MASK);
    pStkPsData->als_cmd_reg |= STK_ALS_CMD_SD(enable?0:1);
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_ALS_CMD_REG,pStkPsData->als_cmd_reg);

}


#ifdef CONFIG_STK_SYSFS_DBG
// For Debug
static ssize_t help_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
     return sprintf(buf, "Usage : cat xxxx\nor echo val > xxxx\
     \nWhere xxxx = ps_code : RO (0~255)\nals_code : RO (0~65535)\nlux : RW (0~by your setting)\ndistance : RW(by your setting)\
     \nals_enable : RW (0~1)\nps_enable : RW(0~1)\nals_transmittance : RW (1~10000)\
     \nps_sleep_time : RW (0~3)\nps_led_driving_current : RW(0~1)\nps_integral_time(0~3)\nps_gain_setting : RW(0~3)\n");

}

static ssize_t driver_version_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    return sprintf(buf,"%s\n",STK_DRIVER_VER);
}

static ssize_t als_code_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t reading;
    STK_LOCK(1);
    reading = pStkPsData->als_reading;
    STK_LOCK(0);
    return sprintf(buf, "%d\n", reading);
}


static ssize_t ps_code_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t reading;
    STK_LOCK(1);
    reading = pStkPsData->ps_reading;
    STK_LOCK(0);
    return sprintf(buf, "%d\n", reading);
}

#endif //CONFIG_STK_SYSFS_DBG

static ssize_t lux_range_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    return sprintf(buf, "%d\n", alscode2lux((1<<16) -1));//full code

}

static ssize_t dist_mode_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    return sprintf(buf,"%d\n",(int32_t)STK_PS_DISTANCE_MODE);
}
static ssize_t dist_res_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    return sprintf(buf, "1\n"); // means 0.001 cm in Android
}
static ssize_t lux_res_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    return sprintf(buf, "1\n"); // means 1 lux in Android
}
static ssize_t distance_range_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    return sprintf(buf, "%d\n",1);
}

static ssize_t ps_enable_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t enable;
    STK_LOCK(1);
    enable = pStkPsData->bPSThreadRunning;
    STK_LOCK(0);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t ps_enable_store(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               const char *buf, size_t len)
{
    uint32_t value = simple_strtoul(buf, NULL, 10);
    INFO("STK PS31xx Driver : Enable PS : %d\n",value);
    STK_LOCK(1);
    enable_ps_int(value);
    STK_LOCK(0);
    return len;
}

static ssize_t als_enable_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t enable;
    STK_LOCK(1);
    enable = pStkPsData->bALSThreadRunning;
    STK_LOCK(0);
    return sprintf(buf, "%d\n", enable);
}


static ssize_t als_enable_store(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                const char *buf, size_t len)
{
    uint32_t value = simple_strtoul(buf, NULL, 10);
    INFO("STK PS31xx Driver : Enable ALS : %d\n",value);
    STK_LOCK(1);
    enable_als(value);
    STK_LOCK(0);
    return len;
}


static ssize_t lux_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t als_reading;
    STK_LOCK(1);
    als_reading = pStkPsData->als_reading;
    STK_LOCK(0);
    return sprintf(buf, "%d lux\n", alscode2lux(als_reading));
}

static ssize_t lux_store(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                const char *buf, size_t len)
{
    unsigned long value = simple_strtoul(buf, NULL, 16);
    STK_LOCK(1);
    als_report_event(pStkPsData->als_input_dev,value);
    STK_LOCK(0);
    return len;
}

static ssize_t distance_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t dist=1;
    STK_LOCK(1);
#ifdef CONFIG_STK_PS_ENGINEER_TUNING
	if (get_ps_reading()>=ps_code_high_thd)
#else
	if (get_ps_reading()>=CONFIG_STK_PS_CODE_HIGH_THRESHOLD)
#endif
	{
		ps_report_event(pStkPsData->ps_input_dev,0);
		dist=0;
	}
	else
	{
		ps_report_event(pStkPsData->ps_input_dev,1);
		dist=1;
	}
    STK_LOCK(0);

    return sprintf(buf, "%d\n", dist);
}

static ssize_t distance_store(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                const char *buf, size_t len)
{
    unsigned long value = simple_strtoul(buf, NULL, 16);
    STK_LOCK(1);
    ps_report_event(pStkPsData->ps_input_dev,value);
    STK_LOCK(0);
    return len;
}
#define __DEBUG_SYSFS_BIN 1


ssize_t stk_bin_sysfs_read(als_lux_range)
{
    uint32_t* pDst = (uint32_t*)buffer;
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read Lux Range(bin) size !=4\n");
        return 0;
    }
#endif

    *pDst = alscode2lux((1<<16)-1);
    return sizeof(uint32_t);
}
ssize_t stk_bin_sysfs_read(ps_distance_range)
{
    int32_t* pDst = (int32_t*)buffer;
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(int32_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read Distance Range(bin) size !=4\n");
        return 0;
    }
#endif

  *pDst = 1;
  return sizeof(int32_t);
}

ssize_t stk_bin_sysfs_read(ps_distance_mode)
{
    uint8_t* pDst = (uint8_t*)buffer;
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint8_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read Distance Range(bin) size !=4\n");
        return 0;
    }
#endif

  *pDst = STK_PS_DISTANCE_MODE;
  return sizeof(uint8_t);
}

ssize_t stk_bin_sysfs_read(ps_distance_resolution)
{
    uint32_t* pDst = (uint32_t*)buffer;
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read Distance Resolution(bin) size !=4\n");
        return 0;
    }
#endif
  // means 1 cm for Android
  *pDst = 1;
  return sizeof(uint32_t);
}

ssize_t stk_bin_sysfs_read(als_lux_resolution)
{
    uint32_t* pDst = (uint32_t*)buffer;
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read Distance Range(bin) size !=4\n");
        return 0;
    }
#endif
   // means 1 lux for Android
  *pDst = 1;
  return sizeof(uint32_t);
}



ssize_t stk_bin_sysfs_read(distance)
{
    int32_t* pDst = (int32_t*) buffer;

#if __DEBUG_SYSFS_BIN
    if (count != sizeof(int32_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read Distance(bin) size !=4\n");
        return 0;
    }
#endif

    STK_LOCK(1);
    *pDst = pStkPsData->ps_distance_last;
    STK_LOCK(0);
    return sizeof(int32_t);
}
ssize_t stk_bin_sysfs_read(lux_bin)
{

    int32_t *pDst = (int32_t*)buffer;

#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read Lux(bin) size !=4\n");
        return 0;
    }
#endif
    STK_LOCK(1);
    *pDst = pStkPsData->als_lux_last;
    STK_LOCK(0);
    return sizeof(uint32_t);
}

ssize_t stk_bin_sysfs_read(ps_enable)
{
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint8_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read ps_enable_bin size !=1\n");
        return 0;
    }
#endif
    STK_LOCK(1);
    buffer[0] = pStkPsData->bPSThreadRunning?1:0;
    STK_LOCK(0);
    return sizeof(uint8_t);
}


ssize_t stk_bin_sysfs_read(als_enable)
{
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint8_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read als_enable_bin size !=1\n");
        return 0;
    }
#endif
    STK_LOCK(1);
    buffer[0] = pStkPsData->bALSThreadRunning?1:0;
    STK_LOCK(0);
    return sizeof(uint8_t);
}
ssize_t  stk_bin_sysfs_write(ps_enable)
{
#if __DEBUG_SYSFS_BIN
    INFO("STK PS31xx Driver : Enable PS : %d\n",(int32_t)(buffer[0]));
#endif
    STK_LOCK(1);
    enable_ps_int(buffer[0]);
    STK_LOCK(0);
    return count;
}
ssize_t  stk_bin_sysfs_write(als_enable)
{
#if __DEBUG_SYSFS_BIN
    INFO("STK PS31xx Driver : Enable ALS : %d\n",(int32_t)(buffer[0]));
#endif
    STK_LOCK(1);
    enable_als(buffer[0]);
    STK_LOCK(0);
    return count;
}

ssize_t stk_bin_sysfs_read(als_delay)
{
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read als_delay size !=4\n");
        return 0;
    }
#endif
    STK_LOCK(1);
    *((uint32_t*)buffer) = pStkPsData->als_delay;
    STK_LOCK(0);
    return sizeof(uint32_t);
}

ssize_t stk_bin_sysfs_write(als_delay)
{
    uint32_t delay;
#if __DEBUG_SYSFS_BIN
    INFO("STK PS31xx Driver : Set ALS Delay: %d\n",*((int32_t*)buffer));
#endif
    delay = *((uint32_t*)buffer);
    if (delay<ALS_MIN_DELAY)
        delay = ALS_MIN_DELAY;
    STK_LOCK(1);
    pStkPsData->als_delay = delay;
    STK_LOCK(0);
    return count;
}

ssize_t stk_bin_sysfs_read(ps_delay)
{
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read ps_delay size !=4\n");
        return 0;
    }
#endif
    STK_LOCK(1);
    *((uint32_t*)buffer) = pStkPsData->ps_delay;
    STK_LOCK(0);
    return sizeof(uint32_t);
}

ssize_t stk_bin_sysfs_write(ps_delay)
{
    uint32_t delay;
#if __DEBUG_SYSFS_BIN
    INFO("STK PS31xx Driver : Set PS Delay: %d\n",*((int32_t*)buffer));
#endif
    delay = *((uint32_t*)buffer);
    if (delay<PS_MIN_DELAY)
        delay = PS_MIN_DELAY;
    STK_LOCK(1);
    pStkPsData->ps_delay = delay;
    STK_LOCK(0);
    return count;
}

ssize_t stk_bin_sysfs_read(als_min_delay)
{
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read als_min_delay size !=4\n");
        return 0;
    }
#endif
    *((uint32_t*)buffer) = ALS_MIN_DELAY;
    return sizeof(uint32_t);
}

ssize_t stk_bin_sysfs_read(ps_min_delay)
{
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK PS31xx Driver : Error --> Read ps_min_delay size !=4\n");
        return 0;
    }
#endif
    *((uint32_t*)buffer) = PS_MIN_DELAY;
    return sizeof(uint32_t);
}

#ifdef CONFIG_STK_SYSFS_DBG
/* Only for debug */
static struct kobj_attribute help_attribute = (struct kobj_attribute)__ATTR_RO(help);
static struct kobj_attribute driver_version_attribute = (struct kobj_attribute)__ATTR_RO(driver_version);
static struct kobj_attribute als_code_attribute = (struct kobj_attribute)__ATTR_RO(als_code);
static struct kobj_attribute ps_code_attribute = (struct kobj_attribute)__ATTR_RO(ps_code);
#endif //CONFIG_STK_SYSFS_DBG

static struct kobj_attribute lux_range_attribute = (struct kobj_attribute)__ATTR_RO(lux_range);
static struct kobj_attribute lux_attribute = (struct kobj_attribute)__ATTR_RW(lux);
static struct kobj_attribute distance_attribute = (struct kobj_attribute)__ATTR_RW(distance);
static struct kobj_attribute ps_enable_attribute = (struct kobj_attribute)__ATTR_RW(ps_enable);
static struct kobj_attribute als_enable_attribute = (struct kobj_attribute)__ATTR_RW(als_enable);
static struct kobj_attribute ps_dist_mode_attribute = (struct kobj_attribute)__ATTR_RO(dist_mode);
static struct kobj_attribute ps_dist_res_attribute = (struct kobj_attribute)__ATTR_RO(dist_res);
static struct kobj_attribute lux_res_attribute = (struct kobj_attribute)__ATTR_RO(lux_res);
static struct kobj_attribute ps_distance_range_attribute = (struct kobj_attribute)__ATTR_RO(distance_range);

static struct bin_attribute ps_distance_range_bin_attribute = __ATTR_BIN_RO(distance_range_bin,ps_distance_range_read,sizeof(int32_t));
static struct bin_attribute ps_distance_bin_attribute = __ATTR_BIN_RO(distance_bin,distance_read,sizeof(int32_t));
static struct bin_attribute ps_dist_res_bin_attribute = __ATTR_BIN_RO(distance_resolution_bin,ps_distance_resolution_read,sizeof(uint32_t));
static struct bin_attribute ps_dist_mode_bin_attribute = __ATTR_BIN_RO(ps_dist_mode_bin,ps_distance_mode_read,sizeof(uint8_t));
static struct bin_attribute ps_enable_bin_attribute = __ATTR_BIN_RW(ps_enable_bin,ps_enable_read,ps_enable_write,sizeof(uint8_t));
static struct bin_attribute als_lux_range_bin_attribute = __ATTR_BIN_RO(lux_range_bin,als_lux_range_read,sizeof(uint32_t));
static struct bin_attribute als_lux_bin_attribute = __ATTR_BIN_RO(lux_bin,lux_bin_read,sizeof(uint32_t));
static struct bin_attribute als_lux_res_bin_attribute = __ATTR_BIN_RO(lux_resolution_bin,als_lux_resolution_read,sizeof(uint32_t));
static struct bin_attribute als_enable_bin_attribute = __ATTR_BIN_RW(als_enable_bin,als_enable_read,als_enable_write,sizeof(uint8_t));
/* <---DEPRECATED */
static struct bin_attribute als_delay_bin_attribute = __ATTR_BIN_RW(als_delay_bin,als_delay_read,als_delay_write,sizeof(uint32_t));
static struct bin_attribute ps_delay_bin_attribute = __ATTR_BIN_RW(ps_delay_bin,ps_delay_read,ps_delay_write,sizeof(uint32_t));
static struct bin_attribute als_min_delay_bin_attribute = __ATTR_BIN_RO(als_min_delay_bin,als_min_delay_read,sizeof(uint32_t));
static struct bin_attribute ps_min_delay_bin_attribute = __ATTR_BIN_RO(ps_min_delay_bin,ps_min_delay_read,sizeof(uint32_t));
/* DEPRECATED---> */
#ifdef CONFIG_STK_SYSFS_DBG

#ifdef CONFIG_STK_ALS_TRANSMITTANCE_TUNING
#pragma message("Enable STK ALS Transmittance Tuning w/ SYSFS")
static ssize_t als_transmittance_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t transmittance;
    STK_LOCK(1);
    transmittance = als_transmittance;
    STK_LOCK(0);
    return sprintf(buf, "%d\n", transmittance);
}


static ssize_t als_transmittance_store(struct kobject *kobj,
                                       struct kobj_attribute *attr,
                                       const char *buf, size_t len)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
    STK_LOCK(1);
    als_transmittance = value;
    STK_LOCK(0);
    return len;
}



static struct kobj_attribute als_transmittance_attribute = (struct kobj_attribute)__ATTR_RW(als_transmittance);
#endif // CONFIG_STK_ALS_TRANSMITTANCE_TUNING



#ifdef CONFIG_STK_PS_ENGINEER_TUNING

#pragma message("Enable STK PS Engineering Tuning w/ SYSFS")
static ssize_t ps_sleep_time_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t value;
    STK_LOCK(1);
    value = pStkPsData->ps_cmd_reg;
    STK_LOCK(0);
    value&=STK_PS_CMD_SLP_MASK;
    value>>=STK_PS_CMD_SLP_SHIFT;
    return sprintf(buf, "0x%x\n", value);
}


static ssize_t ps_sleep_time_store(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                const char *buf, size_t len)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
    STK_LOCK(1);
    set_ps_slp(value);
    STK_LOCK(0);
    return len;
}


static ssize_t ps_led_driving_current_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t value;
    STK_LOCK(1);
    value = pStkPsData->ps_cmd_reg;
    STK_LOCK(0);
    value&=STK_PS_CMD_DR_MASK;
    value>>=STK_PS_CMD_DR_SHIFT;
    return sprintf(buf, "0x%x\n", value);
}


static ssize_t ps_led_driving_current_store(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                const char *buf, size_t len)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
    STK_LOCK(1);
    set_ps_led_driving_current(value);
    STK_LOCK(0);
    return len;
}
static ssize_t ps_integral_time_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t value;
    STK_LOCK(1);
    value = pStkPsData->ps_cmd_reg;
    STK_LOCK(0);
    value&=STK_PS_CMD_IT_MASK;
    value>>=STK_PS_CMD_IT_SHIFT;
    return sprintf(buf, "0x%x\n", value);
}


static ssize_t ps_integral_time_store(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                const char *buf, size_t len)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
    STK_LOCK(1);
    set_ps_it((uint8_t)value);
    STK_LOCK(0);
    return len;
}
static ssize_t ps_gain_setting_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t gc_reg;
    STK_LOCK(1);
    gc_reg = pStkPsData->ps_gc_reg;
    STK_LOCK(0);
    return sprintf(buf, "0x%x\n", gc_reg);
}


static ssize_t ps_gain_setting_store(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                const char *buf, size_t len)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
    STK_LOCK(1);
    set_ps_gc((uint8_t)value);
    STK_LOCK(0);
    return len;
}


static ssize_t ps_code_thd_l_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
	uint8_t ps_low_thd;
	ps_low_thd =  i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_THD_L_REG);
	return sprintf(buf, "%d\n", ps_low_thd);
}


static ssize_t ps_code_thd_l_store(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                const char *buf, size_t len)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
    STK_LOCK(1);
    set_ps_thd_l(value);
    STK_LOCK(0);
    return len;
}

static ssize_t ps_code_thd_h_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
	uint8_t ps_high_thd;
	ps_high_thd =  i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_THD_H_REG);
	return sprintf(buf, "%d\n", ps_high_thd);
}


static ssize_t ps_code_thd_h_store(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                const char *buf, size_t len)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
    STK_LOCK(1);
    set_ps_thd_h(value);
    STK_LOCK(0);
    return len;
}

static struct kobj_attribute ps_sleep_time_attribute = (struct kobj_attribute)__ATTR_RW(ps_sleep_time);
static struct kobj_attribute ps_led_driving_current_attribute = (struct kobj_attribute)__ATTR_RW(ps_led_driving_current);
static struct kobj_attribute ps_integral_time_attribute = (struct kobj_attribute)__ATTR_RW(ps_integral_time);
static struct kobj_attribute ps_gain_setting_attribute = (struct kobj_attribute)__ATTR_RW(ps_gain_setting);
static struct kobj_attribute ps_code_thd_l_attribute = (struct kobj_attribute)__ATTR_RW(ps_code_thd_l);
static struct kobj_attribute ps_code_thd_h_attribute = (struct kobj_attribute)__ATTR_RW(ps_code_thd_h);
#endif //CONFIG_STK_PS_ENGINEER_TUNING

#endif //CONFIG_STK_SYSFS_DBG

static struct attribute* sensetek_optical_sensors_attrs [] =
{
    &lux_range_attribute.attr,
    &lux_attribute.attr,
    &distance_attribute.attr,
    &ps_enable_attribute.attr,
    &als_enable_attribute.attr,
    &ps_dist_mode_attribute.attr,
    &ps_dist_res_attribute.attr,
    &lux_res_attribute.attr,
    &ps_distance_range_attribute.attr,
    NULL,
};

#ifdef CONFIG_STK_SYSFS_DBG

static struct attribute* sensetek_optical_sensors_dbg_attrs [] =
{
    &help_attribute.attr,
    &driver_version_attribute.attr,
    &lux_range_attribute.attr,
    &ps_code_attribute.attr,
    &als_code_attribute.attr,
    &lux_attribute.attr,
    &distance_attribute.attr,
    &ps_enable_attribute.attr,
    &als_enable_attribute.attr,
    &ps_dist_mode_attribute.attr,
    &ps_dist_res_attribute.attr,
    &lux_res_attribute.attr,
    &ps_distance_range_attribute.attr,
#ifdef CONFIG_STK_ALS_TRANSMITTANCE_TUNING
    &als_transmittance_attribute.attr,
#endif
#ifdef CONFIG_STK_PS_ENGINEER_TUNING
    &ps_sleep_time_attribute.attr,
    &ps_led_driving_current_attribute.attr,
    &ps_integral_time_attribute.attr,
    &ps_gain_setting_attribute.attr,
    &ps_code_thd_l_attribute.attr,
    &ps_code_thd_h_attribute.attr,
#endif //CONFIG_STK_PS_ENGINEER_TUNING
    NULL,
};
// those attributes are only for engineer test/debug
static struct attribute_group sensetek_optics_sensors_attrs_group =
{
    .name = "DBG",
    .attrs = sensetek_optical_sensors_dbg_attrs,
};
#endif //CONFIG_STK_SYSFS_DBG

static struct bin_attribute* sensetek_optical_sensors_bin_attrs[] =
{
    &ps_distance_range_bin_attribute,
    &als_lux_range_bin_attribute,
    &ps_distance_bin_attribute,
    &als_lux_bin_attribute,
    &ps_enable_bin_attribute,
    &als_lux_res_bin_attribute,
    &als_enable_bin_attribute,
    &ps_dist_mode_bin_attribute,
    &ps_dist_res_bin_attribute,
    &als_delay_bin_attribute,
    &ps_delay_bin_attribute,
    &als_min_delay_bin_attribute,
    &ps_min_delay_bin_attribute,
    NULL,
};

static struct platform_device *stk_oss_dev = NULL; /* Device structure */

static int stk_sysfs_create_files(struct kobject *kobj,struct attribute** attrs)
{
    int err;
    while(*attrs!=NULL)
    {
        err = sysfs_create_file(kobj,*attrs);
        if (err)
            return err;
        attrs++;
    }
    return 0;
}

static int stk_sysfs_create_bin_files(struct kobject *kobj,struct bin_attribute** bin_attrs)
{
    int err;
    while(*bin_attrs!=NULL)
    {
        err = sysfs_create_bin_file(kobj,*bin_attrs);
        if (err)
            return err;
        bin_attrs++;
    }
    return 0;
}

static void stk_sysfs_remove_bin_files(struct kobject *kobj,struct bin_attribute** bin_attrs)
{
    while(*bin_attrs!=NULL)
    {
        sysfs_remove_bin_file(kobj,*bin_attrs);
        bin_attrs++;
    }
}

/* PROX SYSFS */
static ssize_t prox_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t reading;

	STK_LOCK(1);
	reading = get_ps_reading();
	STK_LOCK(0);
	return sprintf(buf, "%04d\n", reading);
}

static ssize_t prox_dfdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t ps_code, ps_code_last,event=0;
	uint32_t delay;

	STK_LOCK(1);
	delay = pStkPsData->ps_delay;
	pStkPsData->ps_reading = get_ps_reading();
	ps_code = pStkPsData->ps_reading;
	ps_code_last = pStkPsData->ps_code_last;
#ifdef CONFIG_STK_PS_ENGINEER_TUNING
	//INFO("STK PS : ps_code=%d\n",ps_code);
	if (ps_code > ps_code_high_thd)
	{
		pStkPsData->ps_code_last = ps_code;
		event = 0;
	}
#else
	if (ps_code > CONFIG_STK_PS_CODE_HIGH_THRESHOLD)
	{
		pStkPsData->ps_code_last = ps_code;
		event = 0;
	}
#endif

#ifdef CONFIG_STK_PS_ENGINEER_TUNING
	else if (ps_code < ps_code_low_thd)
	{
		pStkPsData->ps_code_last = ps_code;
		event = 1;
	}
#else
	else  if (ps_code < CONFIG_STK_PS_CODE_LOW_THRESHOLD)
	{
		pStkPsData->ps_code_last = ps_code;
		event = 1;
	}
#endif

//	input_report_abs(pStkPsData->ps_input_dev, ABS_X, event);
//	input_sync(pStkPsData->ps_input_dev);

	STK_LOCK(0);
//	printk("--- %s --- ps_code: %d, event: %d \n",__func__,ps_code,event);

	return sprintf(buf, "%hu%hu%d\n", EV_ABS, ABS_X, event);
}

static ssize_t prox_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t enable;
	STK_LOCK(1);
	enable = pStkPsData->bPSThreadRunning;
	STK_LOCK(0);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t prox_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	uint32_t value = simple_strtoul(buf, NULL, 10);
	int ret = 0,temp=0;
	uint32_t param1 = 0, param2 = 0;
	INFO("STK PS31xx Driver : Enable PS : %d\n",value);

	if(first == 1){
		/******* NV item - PROX *******/
		param1 = 2499;
		param2 = 2;
		ret = msm_proc_comm(PCOM_NV_READ, &param1, &param2);
		if(ret != 0){
			printk("[ proximity ] get nv item failed\n");
			threshold_l = CONFIG_STK_PS_CODE_LOW_THRESHOLD;
			threshold_h = CONFIG_STK_PS_CODE_HIGH_THRESHOLD;
		}else{
			threshold_l = ascii_to_int(param2);
			param2 = 3;
			ret = msm_proc_comm(PCOM_NV_READ, &param1, &param2);
			threshold_h = ascii_to_int(param2);
			if((threshold_l < 0) || (threshold_l > 255) || (threshold_h < 0) || (threshold_h > 255)){
				threshold_l = CONFIG_STK_PS_CODE_LOW_THRESHOLD;
				threshold_h = CONFIG_STK_PS_CODE_HIGH_THRESHOLD;
			}else if(threshold_l > threshold_h){
				printk("[ proximity ] get threshold_high(%d) threshold_low(%d) nv item failed\n",threshold_h,threshold_l);
				temp = threshold_h;
				threshold_h = threshold_l;
				threshold_l = temp;
			}
			param2 = 4;
			ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
			led_setting = ascii_to_int(param2);
			printk("[ proximity ] led_setting: %d\n",led_setting);
			if((0 <= led_setting) && (led_setting <= 3)){
				LED_CURRENT = 0;
				INTEGRAL_TIME = led_setting;
			}else if((4 <= led_setting) && (led_setting <= 7)){
				LED_CURRENT = 1;
				INTEGRAL_TIME = led_setting-4;
			}else{
				led_setting = 9;
				if(hwid < 4){
					if((vender_id == 0xAA) && (color_lens == 0x38)){
						INTEGRAL_TIME = 0;
						LED_CURRENT = 0;
					}else{
						LED_CURRENT = STK_PS_IRLED_DRIVING_CURRENT;
						INTEGRAL_TIME = STK_PS_INTEGRAL_TIME;
					}
				}else{
					LED_CURRENT = 0;
					INTEGRAL_TIME = 0;
				}
			}
		}
		printk("prox_h: %d, prox_l: %d\n",threshold_h,threshold_l);

		/******* NV item - ALS *******/
		param1 = 2500;
		param2 = 0;
		ret = msm_proc_comm(PCOM_NV_READ, &param1, &param2);
		if(ret != 0){
			printk("[ light ] get nv item failed\n");
			sf0 = 1;
			sf1 = 1;
			sf2 = 1;
			AVGSF = 100;
		}else{
			if( hwid < 4 ){
				printk(" hwid: %d \n",hwid);
				sf0 = 55*100/ascii_to_int(param2);
				param2 = 1;
				ret = msm_proc_comm(PCOM_NV_READ, &param1, &param2);
				sf1 = 255*100/ascii_to_int(param2);
				param2 = 2;
				ret = msm_proc_comm(PCOM_NV_READ, &param1, &param2);
				sf2 = 700*100/ascii_to_int(param2);
			}else{
				printk(" +hwid: %d \n",hwid);
				sf0 = 200*100/ascii_to_int(param2);
				param2 = 1;
				ret = msm_proc_comm(PCOM_NV_READ, &param1, &param2);
				sf1 = 700*100/ascii_to_int(param2);
				param2 = 2;
				ret = msm_proc_comm(PCOM_NV_READ, &param1, &param2);
				sf2 = 1000*100/ascii_to_int(param2);
			}
			if((sf0 < 0 ) || (sf1 < 0) || (sf2 < 0 )){
				AVGSF = 100;
			}else{
				AVGSF = (sf0 + sf1 + sf2)/3;
			}
		}
		first = 0;
		STK_LOCK(1);
		enable_als(value);
		STK_LOCK(0);
		set_ps_thd_h(threshold_h);
		set_ps_thd_l(threshold_l);
		set_ps_it(INTEGRAL_TIME);
		set_ps_led_driving_current(LED_CURRENT);
	}else{
		STK_LOCK(1);
		set_ps_thd_h(threshold_h);
		set_ps_thd_l(threshold_l);
		enable_ps_int(value);
		STK_LOCK(0);
	}
	return len;
}

static ssize_t prox_oncall_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ONCALL: %d, PS_INT_STATE: %d\n", ONCALL, PS_INT_STATE);
}

static ssize_t prox_oncall_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	uint32_t value = simple_strtoul(buf, NULL, 10);

	ONCALL = value;
	printk("== PROX oncall: %d ==\n",ONCALL);
	return len;
}

static ssize_t prox_cntrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_CMD_REG);
	return sprintf(buf, "0x%x\n", ret);
}

static ssize_t prox_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = get_status_reg();
	return sprintf(buf, "0x%x\n", ret);
}

static ssize_t register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	int i;

	for(i=1;i<13;i++){
		ret = i2c_smbus_read_byte_data(pStkPsData->client, i);
		if (ret < 0) {
			dev_err(&pStkPsData->client->dev, "register_show failed to read reg.\n");
		}
		printk("register[ 0x%02x]: %x\n",i,ret);
	}
	return sprintf(buf, "%d\n",ret);
}

static ssize_t prox_thres_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "high: %d, low: %d\n", threshold_h, threshold_l);
}

static DEVICE_ATTR(prox_data, S_IRUGO,prox_data_show, NULL);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, prox_enable_show, prox_enable_store);
static DEVICE_ATTR(dfdata, S_IRUGO,prox_dfdata_show, NULL);
static DEVICE_ATTR(cntrl, S_IRUGO,prox_cntrl_show, NULL);
static DEVICE_ATTR(status, S_IRUGO,prox_status_show, NULL);
static DEVICE_ATTR(register, S_IRUGO,register_show, NULL);
static DEVICE_ATTR(prox_thres, S_IRUGO,prox_thres_show, NULL);
static DEVICE_ATTR(oncall, S_IRUGO | S_IWUSR, prox_oncall_show, prox_oncall_store);

static struct attribute *prox_attrs_ctrl[] = {
	&dev_attr_prox_data.attr,
	&dev_attr_enable.attr,
	&dev_attr_dfdata.attr,
	&dev_attr_cntrl.attr,
	&dev_attr_status.attr,
	&dev_attr_register.attr,
	&dev_attr_prox_thres.attr,
	&dev_attr_oncall.attr,
	NULL
};

static struct attribute_group prox_attribute_group = {
	.attrs = prox_attrs_ctrl,
};

/* ALS SYSFS */
static ssize_t cal_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t als_reading;
	STK_LOCK(1);
	als_reading = get_als_reading();
	STK_LOCK(0);
	als_reading = alscode2lux(als_reading) * 100 / AVGSF;
	return sprintf(buf, "%04d\n", als_reading);
}

static ssize_t als_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t als_reading;
    STK_LOCK(1);
    als_reading = pStkPsData->als_reading;
    STK_LOCK(0);
    return sprintf(buf, "%d.0\n", alscode2lux(als_reading));
}

static ssize_t als_cntrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = i2c_smbus_read_byte_data(pStkPsData->client,STK_ALS_CMD_REG);
	return sprintf(buf, "0x%x\n", ret);
}

static ssize_t als_sf_show(struct device *dev, struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "[ %d, %d, %d] %d\n", sf0, sf1, sf2, AVGSF);
}

static ssize_t als_sf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	uint32_t value = simple_strtoul(buf, NULL, 10);

	AVGSF = value;
	printk(" AVGSF: %d\n",AVGSF);
	return len;
}

static ssize_t nv_item_show(struct device *dev, struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "led_current [ %d ]; integral_time [ %d ]; prox_cal [ %d, %d]; als_cal [ %d]\n", LED_CURRENT, INTEGRAL_TIME, ps_code_high_thd, ps_code_low_thd, AVGSF);
}

static ssize_t nv_item_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;
	int ret = 0,temp=0;
	uint32_t param1 = 0, param2 = 0;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if(value == 1){
		/******* NV item - PROX *******/
		param1 = 2499;
		param2 = 2;
		ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
		if(ret != 0){
			printk("[%d] %s\n",__LINE__,__func__);
			threshold_l = CONFIG_STK_PS_CODE_LOW_THRESHOLD;
			threshold_h = CONFIG_STK_PS_CODE_HIGH_THRESHOLD;
		}else{
			threshold_l = ascii_to_int(param2);
			param2 = 3;
			ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
			threshold_h = ascii_to_int(param2);
			if((threshold_l < 0) || (threshold_l > 255) || (threshold_h < 0) || (threshold_h > 255)){
				threshold_l = CONFIG_STK_PS_CODE_LOW_THRESHOLD;
				threshold_h = CONFIG_STK_PS_CODE_HIGH_THRESHOLD;
			}else if(threshold_l > threshold_h){
				printk("[ proximity ] get threshold_high(%d) threshold_low(%d) nv item failed\n",threshold_h,threshold_l);
				temp = threshold_h;
				threshold_h = threshold_l;
				threshold_l = temp;
			}
			param2 = 4;
			ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
			led_setting = ascii_to_int(param2);
			printk("[ proximity ] led_setting: %d\n",led_setting);
			if((0 <= led_setting) && (led_setting <= 3)){
				LED_CURRENT = 0;
				INTEGRAL_TIME = led_setting;
			}else if((4 <= led_setting) && (led_setting <= 7)){
				LED_CURRENT = 1;
				INTEGRAL_TIME = led_setting-4;
			}else{
				led_setting = 9;
				if(hwid < 4){
					if((vender_id == 0xAA) && (color_lens == 0x38)){
						INTEGRAL_TIME = 0;
						LED_CURRENT = 0;
					}else{
						LED_CURRENT = STK_PS_IRLED_DRIVING_CURRENT;
						INTEGRAL_TIME = STK_PS_INTEGRAL_TIME;
					}
				}else{
					LED_CURRENT = 0;
					INTEGRAL_TIME = 0;
				}
			}
		}

		/******* NV item - ALS *******/
		param1 = 2500;
		param2 = 0;
		ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
		if(ret != 0){
			printk("[%d] %s\n",__LINE__,__func__);
			sf0 = 1;
			sf1 = 1;
			sf2 = 1;
			AVGSF = 100;
		}else{
			if( hwid < 4 ){
				printk(" hwid: %d \n",hwid);
				sf0 = 55*100/ascii_to_int(param2);
				param2 = 1;
				ret = msm_proc_comm(PCOM_NV_READ, &param1, &param2);
				sf1 = 255*100/ascii_to_int(param2);
				param2 = 2;
				ret = msm_proc_comm(PCOM_NV_READ, &param1, &param2);
				sf2 = 700*100/ascii_to_int(param2);
			}else{
				printk(" +hwid: %d \n",hwid);
				sf0 = 200*100/ascii_to_int(param2);
				param2 = 1;
				ret = msm_proc_comm(PCOM_NV_READ, &param1, &param2);
				sf1 = 700*100/ascii_to_int(param2);
				param2 = 2;
				ret = msm_proc_comm(PCOM_NV_READ, &param1, &param2);
				sf2 = 1000*100/ascii_to_int(param2);
			}
			if((sf0 < 0 ) || (sf1 < 0) || (sf2 < 0 )){
				AVGSF = 100;
			}else{
				AVGSF = (sf0 + sf1 + sf2)/3;
			}
		}

		set_ps_thd_h(threshold_h);
		set_ps_thd_l(threshold_l);
		set_ps_it(INTEGRAL_TIME);
		set_ps_led_driving_current(LED_CURRENT);
	}
	printk("prox threshold [ %d, %d]; als SF [ %d]\n", ps_code_high_thd, ps_code_low_thd, AVGSF);

	return len;
}

static DEVICE_ATTR(cal_data, 0655, cal_lux_show, NULL);
static DEVICE_ATTR(data, 0655, als_lux_show, NULL);
static DEVICE_ATTR(control, 0655, als_cntrl_show, NULL);
static DEVICE_ATTR(AVG_SF, S_IRUGO | S_IWUSR, als_sf_show, als_sf_store);
static DEVICE_ATTR(nv_item, S_IRUGO | S_IWUSR, nv_item_show, nv_item_store);

static struct attribute *als_attrs_ctrl[] = {
	&dev_attr_cal_data.attr,
	&dev_attr_data.attr,
	&dev_attr_control.attr,
	&dev_attr_AVG_SF.attr,
	&dev_attr_nv_item.attr,
	NULL
};

static struct attribute_group als_attribute_group = {
	.attrs = als_attrs_ctrl,
};

static struct workqueue_struct *stk_oss_work_queue = NULL;

static int32_t reset_int_flag(uint8_t org_status,uint8_t disable_flag){
	return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_STATUS_REG,0);
}

inline int32_t get_status_reg(void){
	return i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_STATUS_REG);
}

static void stk_wait_ap_update_work(struct work_struct *work)
{
	int ret=0;
//	printk(" === %s === \n",__func__);
       wake_unlock(&proximity_sensor_wakelock);
	WAKELOCK = 0;
	ret = set_ps_thd_h(threshold_h);
	if( ret < 0 )
		printk("== %s == threshold_high fail\n",__func__);
	ret = set_ps_thd_l(threshold_l);
	if( ret < 0 )
		printk("== %s == threshold_low fail\n",__func__);
	pStkPsData->ps_cmd_reg = 0x42;
	ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
	if(ret < 0){
		ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
		printk("== %s == PS CMD write fail\n",__func__);
	}
	enable_irq(pStkPsData->irq);
	printk("== %s == enable irq\n",__func__);
}

static void stk_oss_work(struct work_struct *work){
	int32_t ret,reading;
	uint8_t disable_flag = 0;

	if(WAKELOCK == 1){
		cancel_delayed_work_sync(&pStkPsData->wait_ap_update_work);
	}else if(WAKELOCK == 0){
		wake_lock(&proximity_sensor_wakelock);
		WAKELOCK = 1;
	}
	STK_LOCK(1);
	msleep(30);
	ret = get_status_reg();
	if(ret < 0)
        {
		STK_LOCK(0);
		ERR("stk_oss_work:get_status_reg fail, ret=%d\n", ret);
		if(pStkPsData->ps_distance_last == 0){
			pStkPsData->ps_distance_last = 1;
			input_report_abs(pStkPsData->ps_input_dev, ABS_X, 1);
			input_sync(pStkPsData->ps_input_dev);
			INFO(" ----FAKE--- STK PS : ps input event 1\n");
		}else{
			pStkPsData->ps_distance_last = 0;
			input_report_abs(pStkPsData->ps_input_dev, ABS_X, 0);
			input_sync(pStkPsData->ps_input_dev);
			INFO(" ----FAKE--- STK PS : ps input event 0\n");
		}
		msleep(30);
		enable_irq(pStkPsData->irq);
		printk("== %s == enable irq(status)\n",__func__);
		return;
	}else if (ret&STK_PS_STATUS_PS_INT_FLAG_MASK){
		reading = get_ps_reading();
		INFO("%s : ps code = %d\n",__func__,reading);

		disable_flag |= STK_PS_STATUS_PS_INT_FLAG_MASK;
		if( reading < 0 ){
			if(pStkPsData->ps_distance_last == 0){
				pStkPsData->ps_distance_last = 1;
				input_report_abs(pStkPsData->ps_input_dev, ABS_X, 1);
				input_sync(pStkPsData->ps_input_dev);
				INFO(" ----FAKE--- STK PS : ps input event 1\n");
			}else{
				pStkPsData->ps_distance_last = 0;
				input_report_abs(pStkPsData->ps_input_dev, ABS_X, 0);
				input_sync(pStkPsData->ps_input_dev);
				INFO(" ----FAKE--- STK PS : ps input event 0\n");
			}
		}
		else if ((reading > ps_code_high_thd) || (reading == ps_code_high_thd))
		{
			pStkPsData->ps_code_last = reading;
			pStkPsData->ps_distance_last = 0;
			input_report_abs(pStkPsData->ps_input_dev, ABS_X, 0);
			input_sync(pStkPsData->ps_input_dev);
			INFO(" ----------- STK PS : ps input event 0\n");
		}
		else if (reading < ps_code_high_thd)
		{
			pStkPsData->ps_code_last = reading;
			pStkPsData->ps_distance_last = 1;
			input_report_abs(pStkPsData->ps_input_dev, ABS_X, 1);
			input_sync(pStkPsData->ps_input_dev);
			INFO("----------- STK PS : ps input event 1\n");
		}
	}
	schedule_delayed_work(&pStkPsData->wait_ap_update_work,50);

	ret = reset_int_flag(ret,disable_flag);
	if(ret < 0)
        {
		STK_LOCK(0);
		ERR("stk_oss_work:reset_int_flag fail, ret=%d", ret);
		msleep(30);
		return;
	}

	msleep(10);
	STK_LOCK(0);

//	printk(" === %s === int:%d\n",__func__,gpio_get_value(EINT_GPIO));
}
static irqreturn_t stk_oss_irq_handler(int irq, void *data){
	struct stkps31xx_data *pData = data;
	printk(" === %s ===\n",__func__);
	disable_irq_nosync(irq);
	printk("== %s == disable irq\n",__func__);
	queue_work(stk_oss_work_queue,&pData->work);
	return IRQ_HANDLED;
	}

static int stk_ps_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    /*
    printk("STKPS -- %s: I2C is probing (%s)%d\n nDetect = %d\n", __func__,id->name,id->driver_data);
    */
    int err;
    struct stkps31xx_data* ps_data;
	
    INFO("STK PS : I2C Probing");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    {
        printk("STKPS -- No Support for I2C_FUNC_SMBUS_BYTE_DATA\n");
        return -ENODEV;
    }
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
    {
        printk("STKPS -- No Support for I2C_FUNC_SMBUS_WORD_DATA\n");
        return -ENODEV;
    }


    if (id->driver_data == 0)
    {

        ps_data = kzalloc(sizeof(struct stkps31xx_data),GFP_KERNEL);
        ps_data->client = client;
        i2c_set_clientdata(client,ps_data);
        mutex_init(&stkps_io_lock);

        ps_data->ps_input_dev = input_allocate_device();
        ps_data->als_input_dev = input_allocate_device();
        if ((ps_data->als_input_dev==NULL)||(ps_data->ps_input_dev==NULL))
        {
            if (ps_data->als_input_dev==NULL)
                input_free_device(ps_data->als_input_dev);
            if (ps_data->ps_input_dev==NULL)
                input_free_device(ps_data->ps_input_dev);
            ERR("%s: could not allocate input device\n", __func__);
            mutex_destroy(&stkps_io_lock);
            kfree(ps_data);
            return -ENOMEM;
        }
        ps_data->als_input_dev->name = ALS_NAME;
        ps_data->ps_input_dev->name = PS_NAME;
        set_bit(EV_ABS, ps_data->als_input_dev->evbit);
        set_bit(EV_ABS, ps_data->ps_input_dev->evbit);
        input_set_abs_params(ps_data->als_input_dev, ABS_Y, 0, alscode2lux((1<<16)-1), 0, 0);
        input_set_abs_params(ps_data->ps_input_dev, ABS_X, 0,1, 0, 0);
        err = input_register_device(ps_data->ps_input_dev);
        if (err<0)
        {
            ERR("STK PS : can not register ps input device\n");
            input_unregister_device(ps_data->als_input_dev);
            mutex_destroy(&stkps_io_lock);
            input_free_device(ps_data->als_input_dev);
            input_free_device(ps_data->ps_input_dev);
            kfree(ps_data);
            return err;

        }
        err = input_register_device(ps_data->als_input_dev);
        if (err<0)
        {
            ERR("STK PS : can not register als input device\n");
            mutex_destroy(&stkps_io_lock);
            input_free_device(ps_data->als_input_dev);
            input_free_device(ps_data->ps_input_dev);
            kfree(ps_data);

            return err;
        }
        INFO("STK PS : register als input device OK\n");
        pStkPsData = ps_data;

	stk_oss_work_queue = create_workqueue("stk_oss_wq");
	INIT_WORK(&ps_data->work, stk_oss_work);
	enable_als(0);
	err = gpio_request(EINT_GPIO, "proximity");
	if (err)
	{
		printk(KERN_INFO "%s: EINT_GPIO gpio_request() failed\n",__func__);
		err = -1;
	}
	err = gpio_tlmm_config(GPIO_CFG(EINT_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (err) {
		printk("gpio_tlmm_config(180, ENA) = %d\n", err);
		goto ON_ERR;
	}
	msleep(10);

	client->irq = gpio_to_irq(EINT_GPIO);
	if ( client->irq< 0 )
	{
		printk(KERN_INFO "%s: EINT_GPIO gpio_to_irq() failed\n",__func__);
		err = -1;
	}
	printk("%s: %d: client->irq: %d, gpio:%d\n",__func__,__LINE__,client->irq,gpio_get_value(EINT_GPIO));

	err = request_irq(client->irq, stk_oss_irq_handler, STK_IRQF_MODE, "stk-oss", ps_data);
	if (err < 0) {
		printk("%s: request_irq(%d) failed for (%d)\n", __func__, client->irq, err);
		return err;
	}
	pStkPsData->irq = client->irq;
	err = enable_irq_wake(pStkPsData->irq);
	if (err)
	{
		printk("%s: enable_irq_wake(%d,1) failed for (%d)\n",  __func__, client->irq, err);
	}

       ps_data->ps_delay = PS_ODR_DELAY;
       ps_data->als_delay = ALS_ODR_DELAY;
	wake_lock_init(&proximity_sensor_wakelock,WAKE_LOCK_SUSPEND,"stk_ps_wakelock");
	INIT_DELAYED_WORK_DEFERRABLE(&ps_data->wait_ap_update_work, stk_wait_ap_update_work);

	set_ps_thd_h(threshold_h);
   	set_ps_thd_l(threshold_l);

        if (!init_all_setting())
        {
            goto ON_ERR;
        }

	err = sysfs_create_group(&pStkPsData->ps_input_dev->dev.kobj, &prox_attribute_group);
	if (err)
		printk("%s: %d ps_input_dev sysfs_create_group fail\n",__func__,__LINE__);

	err = sysfs_create_group(&pStkPsData->als_input_dev->dev.kobj, &als_attribute_group);
	if (err)
		printk("%s: %d als_input_dev sysfs_create_group fail\n",__func__,__LINE__);

	vender_id = get_vender_id();
	color_lens = get_color_lens();
	hwid = qci_read_hw_id();
	printk("=== vender_id: %x, color_lens: %x, hwid: %d ===\n",vender_id,color_lens,hwid);

        return 0;
    }

ON_ERR:
	input_unregister_device(pStkPsData->als_input_dev);
	input_unregister_device(pStkPsData->ps_input_dev);
	input_free_device(pStkPsData->als_input_dev);
	input_free_device(pStkPsData->ps_input_dev);
	kfree(pStkPsData);
	pStkPsData = NULL;
    return -EINVAL;
}


static int stk_ps_remove(struct i2c_client *client)
{

    mutex_destroy(&stkps_io_lock);
    wake_lock_destroy(&proximity_sensor_wakelock);
    if (pStkPsData)
    {
	gpio_free(EINT_GPIO);
        if (stk_oss_work_queue)
            destroy_workqueue(stk_oss_work_queue);
	 sysfs_remove_group(&pStkPsData->ps_input_dev->dev.kobj, &prox_attribute_group);
	 sysfs_remove_group(&pStkPsData->als_input_dev->dev.kobj, &als_attribute_group);
        input_unregister_device(pStkPsData->als_input_dev);
        input_unregister_device(pStkPsData->ps_input_dev);
        input_free_device(pStkPsData->als_input_dev);
        input_free_device(pStkPsData->ps_input_dev);
        kfree(pStkPsData);
        pStkPsData = 0;
    }
    return 0;
}

static int stk_ps_suspend(struct i2c_client *client, pm_message_t state)
{
	int ret = 0;
	printk("== %s ==\n",__func__);
	STK_LOCK(1);
	if(ONCALL == 1){
		pStkPsData->ps_cmd_reg = 0x42;
		ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
		if(ret < 0){
			ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
			printk("== %s == PS CMD WRITE failed\n",__func__);
		}
	}
	ret = set_ps_thd_h(threshold_h);
	if( ret < 0 )
		printk("== %s == threshold_high fail\n",__func__);
	ret = set_ps_thd_l(threshold_l);
	if( ret < 0 )
		printk("== %s == threshold_low fail\n",__func__);
	ret = enable_als(0);
	STK_LOCK(0);
	return ret;
}

static int stk_ps_resume(struct i2c_client *client)
{
	int ret = 0;
	printk("== %s ==\n",__func__);
	STK_LOCK(1);
	if(ONCALL == 1){
		pStkPsData->ps_cmd_reg = 0x42;
		ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
		if(ret < 0){
			ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
			printk("== %s == PS CMD WRITE failed\n",__func__);
		}
	}
	ret = set_ps_thd_h(threshold_h);
	if( ret < 0 )
		printk("== %s == threshold_high fail\n",__func__);
	ret = set_ps_thd_l(threshold_l);
	if( ret < 0 )
		printk("== %s == threshold_low fail\n",__func__);
	ret = enable_als(1);
	STK_LOCK(0);
	return ret;
}

static const struct i2c_device_id stk_ps_id[] =
{
    { "stk_ps", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, stk_ps_id);

static struct i2c_driver stk_ps_driver =
{
    .driver = {
        .name = STKPS_DRV_NAME,

    },
    .probe = stk_ps_probe,
    .remove = stk_ps_remove,
    .suspend = stk_ps_suspend,
    .resume = stk_ps_resume,
    .id_table = stk_ps_id,
};


static int __init stk_i2c_ps31xx_init(void)
{

	int ret;
    ret = i2c_add_driver(&stk_ps_driver);
    if (ret)
        return ret;
    if (pStkPsData == NULL)
        return -EINVAL;

    stk_oss_dev = platform_device_alloc(DEVICE_NAME,-1);
    if (!stk_oss_dev)
    {
       i2c_del_driver(&stk_ps_driver);
       return -ENOMEM;
    }
    if (platform_device_add(stk_oss_dev))
    {
       i2c_del_driver(&stk_ps_driver);
       return -ENOMEM;
    }
    ret = stk_sysfs_create_bin_files(&(stk_oss_dev->dev.kobj),sensetek_optical_sensors_bin_attrs);
    if (ret)
    {
        i2c_del_driver(&stk_ps_driver);
        return -ENOMEM;
    }
    ret = stk_sysfs_create_files(&(stk_oss_dev->dev.kobj),sensetek_optical_sensors_attrs);
    if (ret)
    {
        i2c_del_driver(&stk_ps_driver);
        return -ENOMEM;
    }
#ifdef CONFIG_STK_SYSFS_DBG
    ret = sysfs_create_group(&(stk_oss_dev->dev.kobj), &sensetek_optics_sensors_attrs_group);
    if (ret)
    {
        i2c_del_driver(&stk_ps_driver);
        return -ENOMEM;
    }
#endif //CONFIG_STK_SYSFS_DBG

	INFO("STK PS Module initialized.\n");
    return 0;
}

static void __exit stk_i2c_ps31xx_exit(void)
{

    if (stk_oss_dev);
    {
#ifdef CONFIG_STK_SYSFS_DBG
        sysfs_remove_group(&(stk_oss_dev->dev.kobj), &sensetek_optics_sensors_attrs_group);
#endif
        stk_sysfs_remove_bin_files(&(stk_oss_dev->dev.kobj),sensetek_optical_sensors_bin_attrs);
    }
    platform_device_put(stk_oss_dev);

    i2c_del_driver(&stk_ps_driver);
    platform_device_unregister(stk_oss_dev);
	INFO("STK PS Module removed.\n");
}

MODULE_AUTHOR("Patrick Chang <patrick_chang@sitronix.com>");
MODULE_DESCRIPTION("SenseTek Proximity Sensor driver");
MODULE_LICENSE("GPL");
module_init(stk_i2c_ps31xx_init);
module_exit(stk_i2c_ps31xx_exit);
