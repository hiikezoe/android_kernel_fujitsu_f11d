/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 * VERSION      	DATE			AUTHOR        Note
 *    1.0		  2010-01-05			WenFS    only support mulititouch	Wenfs 2010-10-01
 *    2.0          2011-09-05                   Duxx      Add touch key, and project setting update, auto CLB command
 *    3.0		  2011-09-09			Luowj   
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include "ft5x06_ts.h"
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>   
#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>

#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#if FT5x0x_EXTEND_FUN
#include "ft5x06_ex_fun.h"
#endif
#include <mach/vreg.h>
#include "../../../arch/arm/mach-msm/proc_comm.h"

static struct i2c_client *this_client;

#define CONFIG_FT5X0X_MULTITOUCH 1

#if POLLING_OR_INTERRUPT
static void ft5x0x_polling(unsigned long data);
static struct timer_list test_timer;
#define POLLING_CYCLE 10
#define POLLING_CHECK_TOUCH	0x01
#define POLLING_CHECK_NOTOUCH		0x00
#endif

#if CFG_SUPPORT_TOUCH_KEY
int tsp_keycodes[CFG_NUMOFKEYS] ={
        KEY_MENU,
        KEY_HOME,
        KEY_BACK,
        KEY_SEARCH
};
char *tsp_keyname[CFG_NUMOFKEYS] ={
        "Menu",
        "Home",
        "Back",
        "Search"
};
static bool tsp_keystatus[CFG_NUMOFKEYS];
#endif

int TS_RESET=146;
#define TS_INT_N 142
extern struct vreg *vreg_l16_3p0;
struct vreg *vreg_lvs0;
int after_resume = 5, debug_mode = 0;
int LAST_STATE[5] = {0}, PRESSURE_STATE = 100;
int LAST_X[5] = {0}, LAST_Y[5] = {0};
unsigned char fw_ver;
int ft5x0x_fw_upgrade = 0;

extern inline int qci_read_hw_id(void);

/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata 

Input	:	*rxdata
                     *length

Output	:	ret

function	:	

***********************************************************************************************/
int ft5x0x_i2c_Read(char * writebuf, int writelen, char *readbuf, int readlen)
{
	int ret;

	if(writelen > 0)
	{
		struct i2c_msg msgs[] = {
			{
				.addr	= this_client->addr,
				.flags	= 0,
				.len	= writelen,
				.buf	= writebuf,
			},
			{
				.addr	= this_client->addr,
				.flags	= I2C_M_RD,
				.len	= readlen,
				.buf	= readbuf,
			},
		};
		ret = i2c_transfer(this_client->adapter, msgs, 2);
		if (ret < 0)
			DbgPrintk("msg %s i2c read error: %d\n", __func__, ret);
	}
	else
	{
		struct i2c_msg msgs[] = {
			{
				.addr	= this_client->addr,
				.flags	= I2C_M_RD,
				.len	= readlen,
				.buf	= readbuf,
			},
		};
		ret = i2c_transfer(this_client->adapter, msgs, 1);
		if (ret < 0)
			DbgPrintk("msg %s i2c read error: %d\n", __func__, ret);
	}
	return ret;
}EXPORT_SYMBOL(ft5x0x_i2c_Read);
/***********************************************************************************************
Name	:	 ft5x0x_i2c_Write

Input	:	
                     

Output	:0-write success 	
		other-error code	
function	:	write data by i2c 

***********************************************************************************************/
int ft5x0x_i2c_Write(char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= writelen,
			.buf	= writebuf,
		},
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		DbgPrintk("%s i2c write error: %d\n", __func__, ret);

	return ret;
}EXPORT_SYMBOL(ft5x0x_i2c_Write);

#if 1
#define I2C_Download_Addr 0x38
int ft5x0x_Download_i2c_Read(char * writebuf, int writelen, char *readbuf, int readlen)
{
	int ret;

	if(writelen > 0)
	{
		struct i2c_msg msgs[] = {
			{
				.addr	= I2C_Download_Addr,
				.flags	= 0,
				.len	= writelen,
				.buf	= writebuf,
			},
			{
				.addr	= I2C_Download_Addr,
				.flags	= I2C_M_RD,
				.len	= readlen,
				.buf	= readbuf,
			},
		};
		ret = i2c_transfer(this_client->adapter, msgs, 2);
		if (ret < 0)
			DbgPrintk("msg %s i2c read error: %d\n", __func__, ret);
	}
	else
	{
		struct i2c_msg msgs[] = {
			{
				.addr	= I2C_Download_Addr,
				.flags	= I2C_M_RD,
				.len	= readlen,
				.buf	= readbuf,
			},
		};
		ret = i2c_transfer(this_client->adapter, msgs, 1);
		if (ret < 0)
			DbgPrintk("msg %s i2c read error: %d\n", __func__, ret);
	}
	return ret;
}
/***********************************************************************************************
Name	:	 ft5x0x_i2c_Write

Input	:	
                     

Output	:0-write success 	
		other-error code	
function	:	write data by i2c 

***********************************************************************************************/
int ft5x0x_Download_i2c_Write(char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= I2C_Download_Addr,
			.flags	= 0,
			.len	= writelen,
			.buf	= writebuf,
		},
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		DbgPrintk("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

#endif
void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++)
	{
		for (j = 0; j < 1000; j++)
		{
			 udelay(1);
		}
	}
}

/***************************************/
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

int get_nv_vender_id(void)
{
	int ret = 0;
	uint32_t param1 = 0, param2 = 0;
	int vender_id;

	param1 = 2499;
	param2 = 0;
	ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
	if(ret != 0){
		printk("[%d] %s\n",__LINE__,__func__);
		vender_id = 0xff;
	}else
		vender_id = ascii_to_int(param2);

	return vender_id;
}
EXPORT_SYMBOL(get_nv_vender_id);

int get_nv_color_lens(void)
{
	int ret = 0;
	uint32_t param1 = 0, param2 = 0;
	int color_lens;

	param1 = 2499;
	param2 = 1;
	ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
	if(ret != 0){
		printk("[%d] %s\n",__LINE__,__func__);
		color_lens = 0xff;
	}else
		color_lens = ascii_to_int(param2);

	return color_lens;
}
EXPORT_SYMBOL(get_nv_color_lens);

u8 get_vender_id(void)
{
	u8 reg_vender_id = 0xA8;
	u8 vender_id;

	ft5x0x_i2c_Read(&reg_vender_id, 1, &vender_id, 1);

	return vender_id;
}
EXPORT_SYMBOL(get_vender_id);

u8 get_color_lens(void)
{
	u8 reg_vender_id = 0xB5;
	u8 color_lens;

	ft5x0x_i2c_Read(&reg_vender_id, 1, &color_lens, 1);

	return color_lens;
}
EXPORT_SYMBOL(get_color_lens);

static ssize_t ts_version(struct device *dev,	struct device_attribute *attr, char *buf)
{
	unsigned char uc_tp_fm_ver;
	ft5x0x_read_reg(FT5x0x_REG_FW_VER, &uc_tp_fm_ver);
	return sprintf(buf, "0x%02x\n", uc_tp_fm_ver);
}
static DEVICE_ATTR(fw_version, S_IRUGO, ts_version, NULL);

static ssize_t ts_vender_id(struct device *dev,	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%04d\n", get_vender_id());
}
static DEVICE_ATTR(vender_id, S_IRUGO, ts_vender_id, NULL);

static ssize_t ts_color_lens(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%04d\n", get_color_lens());
}
static DEVICE_ATTR(color_lens, S_IRUGO, ts_color_lens, NULL);

static ssize_t period_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned char data[2];
	data[0] = FT5x0x_REG_POINT_RATE;
	data[1] = simple_strtoul(buf, NULL, 10);
	ft5x0x_i2c_Write(data, sizeof(data));

	return len;
}

static ssize_t period_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr = FT5x0x_REG_POINT_RATE;
	ft5x0x_i2c_Read(&uc_reg_addr, 1, &uc_reg_value, 1);
	return sprintf(buf, "%d\n", uc_reg_value);
}
static DEVICE_ATTR(period, 0655, period_show, period_store);

static ssize_t debug_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	debug_mode = simple_strtoul(buf, NULL, 10);

	return len;
}

static ssize_t debug_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", debug_mode);
}
static DEVICE_ATTR(debug_mode, 0655, debug_mode_show, debug_mode_store);

static ssize_t fw_compare_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	uint32_t value = simple_strtoul(buf, NULL, 10);
	if(value){
		disable_irq(this_client->irq);
		fts_ctpm_auto_upg();
		gpio_set_value(TS_RESET, 1);
		msleep(3);
		gpio_set_value(TS_RESET, 0);
		msleep(10);
		gpio_set_value(TS_RESET, 1);
		msleep(300);
		if(ft5x0x_fw_upgrade == 1)
			ft5x0x_fw_upgrade = 0;
		enable_irq(this_client->irq);
	}

	return len;
}

static ssize_t fw_compare_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ft5x0x_fw_upgrade);
}
static DEVICE_ATTR(fw_compare, 0655, fw_compare_show, fw_compare_store);

static struct attribute *ts_attrs_ctrl[] = {
	&dev_attr_vender_id.attr,
	&dev_attr_color_lens.attr,
	&dev_attr_period.attr,
	&dev_attr_debug_mode.attr,
	&dev_attr_fw_compare.attr,
	&dev_attr_fw_version.attr,
	NULL
};

static struct attribute_group ts_attribute_group = {
	.attrs = ts_attrs_ctrl,
};


/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
/*static void ft5x0x_ts_release(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
	input_sync(data->input_dev);
	if( after_resume >0 ){
		printk("%s: release\n",__func__);
		after_resume = after_resume-1;
	}
}
*/

//read touch point information
static int ft5x0x_read_Touchdata(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[CFG_POINT_READ_BUF] = {0};
	int ret = -1;
	int i;u8 pointid = 0x0f;

	ret = ft5x0x_i2c_Read(buf, 1, buf, CFG_POINT_READ_BUF);
    	if (ret < 0) {
		DbgPrintk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));
#if USE_EVENT_POINT
	event->touch_point = buf[2] & 0x0F; 
#else
	event->touch_point = buf[2] >>4;
#endif
    	if (event->touch_point > CFG_MAX_TOUCH_POINTS)
    	{
        	event->touch_point = CFG_MAX_TOUCH_POINTS;
    	}
	event->touch_point = 0;
    	//for (i = 0; i < event->touch_point; i++)
    	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
    	{
		pointid = (buf[5 + 6*i])>>4;
		if(pointid >= 0x0f)
			break;
		else
			event->touch_point ++;
        	event->au16_x[i] = (s16)(buf[3 + 6*i] & 0x0F)<<8 | (s16)buf[4 + 6*i];
        	event->au16_y[i] = (s16)(buf[5 + 6*i] & 0x0F)<<8 | (s16)buf[6 + 6*i];
		event->au8_touch_event[i] = buf[3 + 6*i] >> 6;
        	event->au8_finger_id[i] = (buf[5 + 6*i])>>4;
		event->pressure[i] = buf[7 + 6*i];
    	}
    	return 0;
}

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/


#if CFG_SUPPORT_TOUCH_KEY
int ft5x0x_touch_key_process(struct input_dev *dev, int x, int y, int touch_event)
{
	int i;
	int key_id;

	if ( y < 517&&y > 497)
	{
		key_id = 1;
	}
	else if ( y < 367&&y > 347)
	{
		key_id = 0;
	}

	else if ( y < 217&&y > 197)
	{
		key_id = 2;
	}  
	else if (y < 67&&y > 47)
	{
		key_id = 3;
	}
	else
	{
		key_id = 0xf;
	}
    
	for(i = 0; i <CFG_NUMOFKEYS; i++ )
	{
		if(tsp_keystatus[i])
		{
			if(touch_event == 1)
			{
				input_report_key(dev, tsp_keycodes[i], 0);

				DbgPrintk("[FTS] %s key is release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);

				tsp_keystatus[i] = KEY_RELEASE;
			}
		}
		else if( key_id == i )
		{
			if( touch_event == 0)                                  // detect
			{
				input_report_key(dev, tsp_keycodes[i], 1);
				DbgPrintk( "[FTS] %s key is pressed. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
				tsp_keystatus[i] = KEY_PRESS;
			}
		}
	}
	return 0;
    
}    
#endif

static void ft5x0x_report_value(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
#if 1
/********** multi-touch protocol B *********/
	int i = 0, press_num = 0, release_num = 0;

	for (i  = 0; i < event->touch_point; i++)
	{
		if (event->au16_x[i] < SCREEN_MAX_X && event->au16_y[i] < SCREEN_MAX_Y && event->au8_finger_id[i] < 5)
		{
			if( debug_mode ){
				printk("%s: [ x , y ] = [ %d , %d ][ev %d][%d]\n",__func__,event->au16_x[i],event->au16_y[i],event->au8_touch_event[i],event->au8_finger_id[i]);
			}else if( after_resume >0 ){
				printk("%s: [ x , y ][%d] = [ %d , %d ][ev %d][%d]\n",__func__,after_resume,event->au16_x[i],event->au16_y[i],event->au8_touch_event[i],event->au8_finger_id[i]);
				after_resume = after_resume-1;
			}

			if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
			{
				press_num ++;
				if(event->au16_x[i] == LAST_X[event->au8_finger_id[i]] && event->au16_y[i] ==LAST_Y[event->au8_finger_id[i]] && LAST_STATE[event->au8_finger_id[i]] == 1){
					//printk("/*/*/ %s: [ x , y ] = [ %d , %d ][ev %d][%d][i %d]\n",__func__,event->au16_x[i],event->au16_y[i],event->au8_touch_event[i],event->au8_finger_id[i],i);
				}else{
					input_mt_slot(data->input_dev, event->au8_finger_id[i]);
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
					if(fw_ver < 0x23){
						input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, PRESSURE_STATE);
						input_report_abs(data->input_dev, ABS_MT_PRESSURE, PRESSURE_STATE);
						if(PRESSURE_STATE == 100){
							PRESSURE_STATE = 99;
						}else if(PRESSURE_STATE == 99){
							PRESSURE_STATE = 100;
						}
					}else{
						if(event->pressure[i] > 127){
							input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 127);
							input_report_abs(data->input_dev, ABS_MT_PRESSURE, 127);
						}else{
							input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure[i]);
							input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
						}
					}
					input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
					input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
					LAST_STATE[event->au8_finger_id[i]] = 1;
					LAST_X[event->au8_finger_id[i]] = event->au16_x[i];
					LAST_Y[event->au8_finger_id[i]] = event->au16_y[i];
				}
			}
			else
			{
				if( LAST_STATE[event->au8_finger_id[i]] == 1){
					release_num++;
					input_mt_slot(data->input_dev, event->au8_finger_id[i]);
					input_report_abs(data->input_dev, ABS_MT_PRESSURE, 0);
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
					LAST_STATE[event->au8_finger_id[i]] = 0;
				}
			}
		}else{
			printk("***** %s: [ x , y ] = [ %d , %d ][ev %d][%d]\n",__func__,event->au16_x[i],event->au16_y[i],event->au8_touch_event[i],event->au8_finger_id[i]);
		}
	}

	if(event->touch_point == release_num){
		input_report_key(data->input_dev, BTN_TOUCH, 0 );
		input_sync(data->input_dev);
	}else if(event->touch_point == (press_num + release_num)){
		input_report_key(data->input_dev, BTN_TOUCH, 1 );
		input_sync(data->input_dev);
	}

#else
/********** multi-touch protocol A *********/
	int i, tp_up = 0, tp_down =0;

	for (i  = 0; i < event->touch_point; i++)
	{
		// LCD view area
	    	if (event->au16_x[i] < SCREEN_MAX_X && event->au16_y[i] < SCREEN_MAX_Y)
	    	{
			if( debug_mode ){
				printk("%s: [ x , y ] = [ %d , %d ][ev %d][%d]\n",__func__,event->au16_x[i],event->au16_y[i],event->au8_touch_event[i],event->au8_finger_id[i]);
			}else if( after_resume >0 ){
				printk("%s: [ x , y ][%d] = [ %d , %d ][ev %d][%d]\n",__func__,after_resume,event->au16_x[i],event->au16_y[i],event->au8_touch_event[i],event->au8_finger_id[i]);
				after_resume = after_resume-1;
			}
	        	input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
    			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
    			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
    			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]);
    			if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
    			{
				tp_down++;
				input_report_abs(data->input_dev, ABS_MT_PRESSURE, 1);
    		    		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
				input_mt_sync(data->input_dev);
    			}
    			else
    			{
				tp_up++;
				input_report_abs(data->input_dev, ABS_MT_PRESSURE, 0);
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				input_mt_sync(data->input_dev);
    			}
	    	}
	    	else //maybe the touch key area
	    	{
#if CFG_SUPPORT_TOUCH_KEY
            		if (event->au16_x[i] >= SCREEN_MAX_X)
            		{
                		ft5x0x_touch_key_process(data->input_dev, event->au16_x[i], event->au16_y[i], event->au8_touch_event[i]);
            		}
#endif
	    	}

	}

	if( tp_down > 0 )
		input_report_key(data->input_dev, BTN_TOUCH, 1 );
	else if( tp_up > 0)
		input_report_key(data->input_dev, BTN_TOUCH, 0 );
	input_sync(data->input_dev);
#endif
}	/*end ft5x0x_report_value*/


/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{

	int ret = -1;

	ret = ft5x0x_read_Touchdata();	
	if(after_resume != 5){
		if (ret == 0) {
		ft5x0x_report_value();
		}
	}else
		after_resume = after_resume-1;
#if POLLING_OR_INTERRUPT
	del_timer(&test_timer);
	add_timer(&test_timer);
#else
	enable_irq(this_client->irq);
//printk("%s: %d, irq: %d\n",__func__,__LINE__,this_client->irq);
#endif

}

#if POLLING_OR_INTERRUPT
static void ft5x0x_polling(unsigned long data)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(this_client);

	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}
}
#endif
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;

	disable_irq_nosync(irq);

	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ts;
	int err = 0, i = 0, count = 0;
	ts = container_of(handler, struct ft5x0x_ts_data, early_suspend);

	printk("==ft5x0x_ts_suspend=\n");

	disable_irq(this_client->irq);
	cancel_work_sync(&ts->pen_event_work);
	flush_workqueue(ts->ts_workqueue);

	for(i = 0; i < CFG_MAX_TOUCH_POINTS; i++){
		if( LAST_STATE[i] == 1){
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			count++;
			LAST_STATE[i] = 0;
		}
	}
	if(count > 0){
		input_report_key(ts->input_dev, BTN_TOUCH, 0 );
		input_sync(ts->input_dev);
	}
	if(!ft5x0x_fw_upgrade){
		printk("==ft5x0x_ts_suspend disable VDD=\n");
		err = vreg_disable(vreg_l16_3p0);
		if(err){
			printk("== [%s] disable VDD fail. ==\n",__func__);
		}
		err = vreg_disable(vreg_lvs0);
		if(err){
			printk("== [%s] disable Pull-high fail. ==\n",__func__);
		}
		err = gpio_tlmm_config(GPIO_CFG(0, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (err) {
			printk(" [%s] config(0)=%d\n",__func__, err);
		}
		err = gpio_tlmm_config(GPIO_CFG(1, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (err) {
			printk(" [%s] config(1)=%d\n",__func__, err);
		}
		gpio_set_value(TS_RESET, 0);
	}
}
/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	int err = 0;
//	u8 period_data[2];

	printk("==ft5x0x_ts_resume=\n");
	if(!ft5x0x_fw_upgrade){
		err = gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (err) {
			printk(" [%s] config(0)=%d\n",__func__, err);
		}
		err = gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (err) {
			printk(" [%s] config(1)=%d\n",__func__, err);
		}

		err = vreg_enable(vreg_l16_3p0);
		if(err){
			printk("== [%s] enable VDD fail. ==\n",__func__);
		}
		msleep(10);
		err = vreg_enable(vreg_lvs0);
		if(err){
			printk("== [%s] enable Pull-high fail. ==\n",__func__);
		}
		msleep(10);

		gpio_set_value(TS_RESET, 1);
		msleep(3);
		gpio_set_value(TS_RESET, 0);
		msleep(10);
		gpio_set_value(TS_RESET, 1);
		msleep(300);
		printk("==ft5x0x_ts_resume enable VDD=\n");
	}

//	period_data[0] = FT5x0x_REG_POINT_RATE;
//	period_data[1] = 16;
//	ft5x0x_i2c_Write(period_data, sizeof(period_data));
//	msleep(35);

	after_resume = 5;
	enable_irq(this_client->irq);
}
#endif  //CONFIG_HAS_EARLYSUSPEND

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int 
ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value; 
	unsigned char uc_reg_addr;
	u8 period_data[2];
#if CFG_SUPPORT_TOUCH_KEY
    	int i;
#endif
	
	DbgPrintk("[FTS] ft5x0x_ts_probe, driver version is %s.\n", CFG_FTS_CTP_DRIVER_VERSION);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(struct ft5x0x_ts_data), GFP_KERNEL);

	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;
	i2c_set_clientdata(client, ft5x0x_ts);
	msleep(300);
	err = gpio_request(TS_INT_N, "ft5x06_ts");
	if (err)
    	{
       	printk(KERN_INFO "%s: TS_INT_N gpio_request() failed\n",__func__);
        	err = -1;
		goto exit_gpio_request;
	}
	err = gpio_direction_input(TS_INT_N);
	if (err < 0)
    	{
       	printk(KERN_INFO "%s: TS_INT_N gpio_direction_input() failed\n",__func__);
        	err = -1;
	}
	client->irq = gpio_to_irq(TS_INT_N);
	if ( client->irq< 0 )
	{
		printk(KERN_INFO "%s: TS_INT_N gpio_to_irq() failed\n",__func__);
		err = -1;
		goto exit_gpio_to_irq;
	}
	printk("%s: %d: client->irq: %d\n",__func__,__LINE__,client->irq);
	this_client->irq = client->irq;
	ft5x0x_ts->irq = client->irq;
	DbgPrintk("INT irq=%d\n", client->irq);
//	mutex_init(&ft5x0x_ts->device_mode_mutex);

	vreg_lvs0 = vreg_get(NULL, "lvsw0");
	if (!vreg_lvs0) {
		printk("%s: VREG LVS0 get failed\n", __func__);
	}

	err = vreg_enable(vreg_lvs0);
	if(err){
		printk("== [%s] enable Pull-high fail. ==\n",__func__);
	}

	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#if POLLING_OR_INTERRUPT
	DbgPrintk("Read TouchData by Polling\n");
#else
	err = request_irq(this_client->irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING, "ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq(this_client->irq);
#endif
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	ft5x0x_ts->input_dev = input_dev;

	input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_PRESSURE, 0, 127, 0, 0);

    	set_bit(EV_ABS, input_dev->evbit);

	//for ICS
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

#if CFG_SUPPORT_TOUCH_KEY
        set_bit(EV_KEY, input_dev->evbit);
    //setup key code area
    	set_bit(EV_SYN, input_dev->evbit);
    	set_bit(BTN_TOUCH, input_dev->keybit);
    	input_dev->keycode = tsp_keycodes;
    	for(i = 0; i < CFG_NUMOFKEYS; i++)
    	{
        	input_set_capability(input_dev, EV_KEY, ((int*)input_dev->keycode)[i]);
        	tsp_keystatus[i] = KEY_RELEASE;
    	}
#endif

	input_dev->name		= FT5X0X_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
				"ft5x0x_ts_probe: failed to register input device: %s\n",
				dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	DbgPrintk("==register_early_suspend =\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

   	 msleep(150);  //make sure CTP already finish startup process
    
    	//get some register information
    	uc_reg_addr = FT5x0x_REG_FW_VER;
    	ft5x0x_i2c_Read(&uc_reg_addr, 1, &uc_reg_value, 1);
    	DbgPrintk("[FTS] Firmware version = 0x%x\n", uc_reg_value);

	uc_reg_addr = FT5x0x_REG_POINT_RATE;
    	ft5x0x_i2c_Read(&uc_reg_addr, 1, &uc_reg_value, 1);
    	DbgPrintk("[FTS] report rate is %dHz.\n", uc_reg_value * 10);
		
    	uc_reg_addr = FT5X0X_REG_THGROUP;
    	ft5x0x_i2c_Read(&uc_reg_addr, 1, &uc_reg_value, 1);
   	 DbgPrintk("[FTS] touch threshold is %d.\n", uc_reg_value * 4);

	#if CFG_SUPPORT_AUTO_UPG
//    fts_ctpm_auto_upg();
	ft5x0x_fw_upgrade = fts_ctpm();
	#endif    

	period_data[0] = FT5x0x_REG_POINT_RATE;
	period_data[1] = 16;
	ft5x0x_i2c_Write(period_data, sizeof(period_data));
	msleep(35);

	//get some register information
	uc_reg_addr = FT5x0x_REG_FW_VER;
	ft5x0x_i2c_Read(&uc_reg_addr, 1, &uc_reg_value, 1);
	DbgPrintk("[FTS] Firmware version = 0x%x\n", uc_reg_value);
	fw_ver = uc_reg_value;

	#if CFG_SUPPORT_UPDATE_PROJECT_SETTING
	fts_ctpm_update_project_setting();
	#endif

#if POLLING_OR_INTERRUPT
	test_timer.function = ft5x06_polling;
	test_timer.expires = jiffies + HZ*2;//POLLING_CYCLE*100; // 100/10
	test_timer.data = 1;
	init_timer(&test_timer);
	add_timer(&test_timer);
#else
	enable_irq(this_client->irq);
#endif
	//you can add sysfs for test
	err = sysfs_create_group(&input_dev->dev.kobj, &ts_attribute_group);
	if (err)
		printk("%s: %d input_dev ts_attribute_group fail\n",__func__,__LINE__);

	after_resume = 5;
	DbgPrintk("[FTS] ==probe over ==\n");
    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
	
exit_input_dev_alloc_failed:
	free_irq(this_client->irq, ft5x0x_ts);
	
exit_irq_request_failed:
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	
exit_create_singlethread:
	DbgPrintk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
exit_gpio_to_irq:
	gpio_free(TS_INT_N);	
exit_gpio_request:	
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);;
	DbgPrintk("==ft5x0x_ts_remove==\n");

	sysfs_remove_group(&ft5x0x_ts->input_dev->dev.kobj, &ts_attribute_group);
	ft5x0x_ts = i2c_get_clientdata(client);
	//unregister_early_suspend(&ft5x0x_ts->early_suspend);
	//mutex_destroy(&ft5x0x_ts->device_mode_mutex);	
	input_unregister_device(ft5x0x_ts->input_dev);
	kfree(ft5x0x_ts);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL); 
#if POLLING_OR_INTERRUPT
	del_timer(&test_timer);
#else
	free_irq(client->irq, ft5x0x_ts);
#endif
	gpio_free(TS_RESET);
	gpio_free(TS_INT_N);
	vreg_disable(vreg_lvs0);
	vreg_disable(vreg_l16_3p0);

	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
};

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int __init ft5x0x_ts_init(void)
{
	int ret,hw_id=1;

	hw_id = qci_read_hw_id();
	if( hw_id == 1)
		TS_RESET = 146;
	else
		TS_RESET = 30;
	DbgPrintk("==ft5x0x_ts_init TS_RESET: %d ==\n",TS_RESET);

	ret = gpio_request(TS_RESET, "ft5x06_ts");
	if (ret)
    	{
       	printk(KERN_INFO "%s: TS_RESET gpio_request() failed\n",__func__);
        	ret = -1;
	}
	ret = gpio_direction_output(TS_RESET, 0);
	if (ret < 0)
	{
		printk(KERN_INFO "%s: TS_RESET gpio_direction_output() failed\n",__func__);
		ret = -1;
		gpio_free(TS_RESET);
	}
	msleep(5);
	ret = gpio_direction_output(TS_RESET, 1);
	if (ret < 0)
    	{
		printk(KERN_INFO "%s: TS_RESET gpio_direction_output() failed\n",__func__);
		ret = -1;
		gpio_free(TS_RESET);
	}
	ret = i2c_add_driver(&ft5x0x_ts_driver);
	DbgPrintk("ret=%d\n",ret);
	return ret;
}

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void __exit ft5x0x_ts_exit(void)
{
	DbgPrintk("==ft5x0x_ts_exit==\n");
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<luowj@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
