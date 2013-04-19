/*
 * Copyright(C) 2011 FUJITSU LIMITED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/regulator/consumer.h>
//#include <linux/adxl345.h>
#include <media/msm_camera.h>
#include <mach/camera.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
//#include <plat/pm.h>
//#include <mach/pm.h>
//#include "sub_pmic.h"
#ifndef Camsensor_gpio_back_H
#include "camsensor_gpio_back.h"
#endif // Camsensor_gpio_back_H
//#include "tsb_model.h"
#include "../../../arch/arm/mach-msm/smd_private.h"
//#include "pm.h"

#define OV5640_DRV_NAME  "ov5640"

#define LOGI(fmt, args...)      printk(KERN_INFO "ov5640: " fmt, ##args)
//#define LOGI(fmt, args...)      do{}while(0)
//#define LOGD(fmt, args...)      printk(KERN_DEBUG "ov5640: " fmt, ##args)
#define LOGD(fmt, args...)      do{}while(0)
#define LOGE(fmt, args...)      printk(KERN_ERR "ov5640: " fmt, ##args)

static int ov5640_pwdn_gpio;
static int ov5640_reset_gpio;
static int OV5640_CSI_CONFIG;
// temp
//#define SUPPORTED_READ_TEMP

/*===================================================================*
    LOCAL DECLARATIONS
 *===================================================================*/
struct ov5640_ctrl {
    const struct msm_camera_sensor_info *sensordata;
//    int model;
//    int led;
};

DEFINE_MUTEX(ov5640_mtx);

struct ov5640_work_t {
	struct work_struct work;
};
static DECLARE_WAIT_QUEUE_HEAD(ov5640_wait_queue);
static struct ov5640_work_t *ov5640_sensorw;
static struct ov5640_ctrl *ov5640_ctrl = NULL;
static struct i2c_client *ov5640_i2c_client = NULL;

/*===================================================================*
    EXTERNAL DECLARATIONS
 *===================================================================*/
extern int _I2C_LOG_;

////////////////////////////////
// I2C
////////////////////////////////
int ov5640_i2c_write_normal(struct CameraSensorI2CCmdType* pI2CCmd)
{
	int rc = 0;

		rc = camsensor_i2c_write_normal(ov5640_i2c_client, pI2CCmd);

	return rc;
}

int ov5640_i2c_read_normal(struct CameraSensorI2CCmdType* pI2CCmd)
{
	int rc = 0;

		rc = camsensor_i2c_read_normal(ov5640_i2c_client, pI2CCmd);

	return rc;
}

////////////////////////////////
// Power ON
////////////////////////////////
static int ov5640_power_on(void)
{
	printk(KERN_ERR"--CAMERA-- %s ...ov5640_pwdn_gpio:%d (Start...)\n", __func__,ov5640_pwdn_gpio);
	gpio_set_value(ov5640_pwdn_gpio, 0);
	printk(KERN_ERR"--CAMERA-- %s ... (End...)\n", __func__);
    return 0;
}

static void ov5640_power_reset(void)
{
	printk(KERN_ERR"--CAMERA-- %s ...ov5640_reset_gpio:%d (Start...)\n", __func__,ov5640_reset_gpio);
	msleep(20);
	gpio_set_value(ov5640_reset_gpio, 1);   /* reset camera reset pin */
	msleep(20);
	gpio_set_value(ov5640_reset_gpio, 0);
	msleep(20);
	gpio_set_value(ov5640_reset_gpio, 1);
	msleep(20);

	printk(KERN_ERR"--CAMERA-- %s ... (End...)\n", __func__);
}

////////////////////////////////
// Power OFF
////////////////////////////////
static void ov5640_power_off(void)
{
	printk(KERN_ERR"--CAMERA-- %s ... (Start...)\n", __func__);
	//gpio_set_value(ov5640_pwdn_gpio, 1);
	gpio_set_value(ov5640_reset_gpio, 0);
	msleep(5);
	printk(KERN_ERR"--CAMERA-- %s ... (End...)\n", __func__);
}

//=====================================================================
// Driver Function
//=====================================================================
//---------------------------------------------------------------------
// msm_open_control
//---------------------------------------------------------------------
int ov5640_sensor_init(const struct msm_camera_sensor_info *data)
{
    LOGI("+%s()\n", __func__);
    
    ov5640_ctrl = kzalloc(sizeof(struct ov5640_ctrl), GFP_KERNEL);
    if (!ov5640_ctrl) {
        LOGE(" -%s kzalloc() Failed!\n",__func__);
        return -ENOMEM;
    }

    if (data)
        ov5640_ctrl->sensordata = data;

	ov5640_power_off();

	CDBG("%s: msm_camio_clk_rate_set\n", __func__);

	msm_camio_clk_rate_set(24000000);
	msleep(20);

    // Sensor Power ON
    if (ov5640_power_on() < 0) {
        kfree(ov5640_ctrl);
        LOGE("-%s Failed.\n", __func__);
        return -1;
    }
	msleep(5);
	ov5640_power_reset();
	msleep(5);
	OV5640_CSI_CONFIG = 0;
    return 0;
}

static int32_t ov5640_video_config(int mode)
{
	struct msm_camera_csi_params ov5640_csi_params;
	int32_t rc = 0;
	LOGI("-%s top.\n", __func__);
	printk(KERN_ERR"ov5640_video_config -OV5640_CSI_CONFIG:%d\n",OV5640_CSI_CONFIG);
	if (!OV5640_CSI_CONFIG) {
		msm_camio_vfe_clk_rate_set(192000000);
		//   ov5640_csi_params.data_format = CSI_10BIT;
		ov5640_csi_params.data_format = CSI_8BIT;//intai0325
		ov5640_csi_params.lane_cnt = 2;
		ov5640_csi_params.lane_assign = 0xe4;
		ov5640_csi_params.dpcm_scheme = 0;
		//ov5640_csi_params.settle_cnt = 0x06;
		ov5640_csi_params.settle_cnt =0x04;
		rc = msm_camio_csi_config(&ov5640_csi_params);
		if (rc < 0)
			LOGE("-%s msm_camio_csi_config Failed(%d).\n", __func__, rc);

		msleep(10);
		OV5640_CSI_CONFIG = 1;
	}
	else	{
		rc = 0;
	}

    return rc;
}

static int32_t ov5640_set_sensor_mode(int mode,
    int res)
{
    int32_t rc = 0;
    LOGD("-%s top.\n", __func__);
    switch (mode) {
    case SENSOR_PREVIEW_MODE:
        LOGI("-%s SENSOR_PREVIEW_MODE.\n", __func__);
        rc = ov5640_video_config(mode);
        break;
    case SENSOR_SNAPSHOT_MODE:
    case SENSOR_RAW_SNAPSHOT_MODE:
    default:
        rc = -EINVAL;
        break;
    }
    LOGI("-%s end(%d).\n", __func__, rc);
    return rc;
}

//---------------------------------------------------------------------
// msm_ioctl_control()
//---------------------------------------------------------------------
int ov5640_sensor_config(void __user *argp)
{
	struct CameraSensorI2CCmdType      I2CCmdNormal;
    struct sensor_cfg_data cfg;
#ifdef SUPPORTED_READ_TEMP
    uint32_t *smem_ptr = NULL;
#endif
    int rc = 0;
	int cpl = 0;

    LOGD("+%s()\n", __func__);
    
    if (copy_from_user(&cfg, (void *)argp, sizeof(struct sensor_cfg_data)))
        return -EFAULT;

    mutex_lock(&ov5640_mtx);
    switch (cfg.cfgtype) {

    case CFG_PWR_UP:
        LOGI("-%s (CFG_PWR_UP)\n", __func__);
        // Sensor Power OFF
        ov5640_power_off();

        mdelay(10);

        // Sensor Power ON
        rc =ov5640_power_on();
        if(rc < 0) {
            LOGE("-%s CFG_PWR_UP Failed!\n", __func__);
        }
        break;

    case CFG_COMMAND_NORMAL:
        LOGD("-%s (CFG_COMMAND_NORMAL)\n", __func__);
        _I2C_LOG_ = cfg.rs;
        I2CCmdNormal.slave_addr = 0x78;
        I2CCmdNormal.pwdata     = cfg.cfg.cmd.wvalue;
        I2CCmdNormal.wlen       = cfg.cfg.cmd.txlen;
        I2CCmdNormal.prdata     = cfg.cfg.cmd.rvalue;
        I2CCmdNormal.rlen       = cfg.cfg.cmd.rxlen;

		if (!cfg.cfg.cmd.rxlen) {
        	rc = ov5640_i2c_write_normal(&I2CCmdNormal);
		} else {
        	rc = ov5640_i2c_read_normal(&I2CCmdNormal);
			if( !rc ) {
				for( cpl=0; cpl<cfg.cfg.cmd.rxlen; cpl++ ) {
					cfg.cfg.cmd.rvalue[cpl] = I2CCmdNormal.prdata[cpl];
//					LOGD("+i2c_read (%02x)\n", cfg.cfg.cmd.rvalue[cpl] );
				}
			}
		}
        if (!rc)
            if (copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data)))
                rc = -EFAULT;
        _I2C_LOG_ = 1;
        break;
	
    case CFG_SET_MODE:
        LOGD("-%s (CFG_SET_MODE)\n", __func__);
        rc = ov5640_set_sensor_mode(cfg.mode, cfg.rs);
        break;

    case CFG_SET_IOPORT:
        LOGD("-%s (CFG_SET_IOPORT)\n", __func__);
        gpio_set_value(cfg.cfg.ioport.pin, cfg.cfg.ioport.val);
        break;

    default:
        LOGE("-%s ERR root:%d\n", __func__, cfg.cfgtype);
        rc = -EINVAL;
        break;
    }
    mutex_unlock(&ov5640_mtx);

    if (rc) LOGI("-%s Done.(%d)\n", __func__, rc);
    return rc;
}

//---------------------------------------------------------------------
// msm_release_control()
//---------------------------------------------------------------------
int ov5640_sensor_release(void)
{
    LOGI("+%s\n", __func__);

    mutex_lock(&ov5640_mtx);

    ov5640_power_off();
    kfree(ov5640_ctrl);
	OV5640_CSI_CONFIG = 0;

    mutex_unlock(&ov5640_mtx);

    LOGI("-%s Done.\n", __func__);
    return 0;
}

/////////////////////////////////////
// Sensor Driver Setup (Kernel Init)
/////////////////////////////////////

// I2C Driver for 1-2Ver. 
static const struct i2c_device_id ov5640_i2c_id[] = {
	{ OV5640_DRV_NAME, 0},
	{ },
};

static int ov5640_init_client(struct i2c_client *client)
{
    LOGI("+%s()\n", __func__);
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov5640_wait_queue);
	return 0;
}


static int __devinit ov5640_i2c_probe( struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;

    LOGI("+%s()\n", __func__);
    
	if( !i2c_check_functionality(client->adapter, I2C_FUNC_I2C) ) {
		LOGE("i2c_check_functionality failed\n");
		return -1;
	}

	ov5640_sensorw = kzalloc(sizeof(struct ov5640_work_t), GFP_KERNEL);
	if (!ov5640_sensorw) {
		LOGE("kzalloc failed.\n");
		rc = -ENOMEM;
		return rc;
	}

	i2c_set_clientdata(client, ov5640_sensorw);
	ov5640_init_client(client);
	ov5640_i2c_client = client;

	msleep(50);

	LOGI("ov5640_probe successed! rc = %d\n", rc);
	return 0;

}

static struct i2c_driver ov5640_i2c_driver = {
	.id_table = ov5640_i2c_id,
	.probe  = ov5640_i2c_probe,
	.remove = __exit_p(ov5640_i2c_remove),
	.driver = {
		.name = OV5640_DRV_NAME,
	},
};

static int ov5640_probe_init_gpio(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	printk(KERN_ERR"--CAMERA-- %s\n", __func__);

	ov5640_pwdn_gpio = data->sensor_pwd;
	ov5640_reset_gpio = data->sensor_reset;
	//ov5640_driver_pwdn_gpio = data->vcm_pwd ;

	//if (data->vcm_enable)
	//	gpio_direction_output(data->vcm_pwd, 1);

	//gpio_direction_output(data->sensor_reset, 1);
	//gpio_direction_output(data->sensor_pwd, 1);
	gpio_tlmm_config( GPIO_CFG(data->sensor_pwd, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_tlmm_config( GPIO_CFG(data->sensor_reset, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	return rc;

}
#if 0
static void ov5640_probe_free_gpio(const struct msm_camera_sensor_info *data)
{
	//gpio_free(ov5640_pwdn_gpio);
	//gpio_free(ov5640_reset_gpio);

	//if (data->vcm_enable) {
	//	gpio_free(ov5640_driver_pwdn_gpio);
	//	ov5640_driver_pwdn_gpio = 0xFF ;
	//}

	//ov5640_pwdn_gpio	= 0xFF;
	//ov5640_reset_gpio	= 0xFF;
	gpio_tlmm_config( GPIO_CFG(data->sensor_pwd, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_DISABLE );
	gpio_tlmm_config( GPIO_CFG(data->sensor_reset, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_DISABLE );	
}
#endif

static int ov5640_sensor_probe(const struct msm_camera_sensor_info *info,
                                struct msm_sensor_ctrl *s)
{
	int rc = 0;

		rc = i2c_add_driver( &ov5640_i2c_driver );
		if( rc < 0 ) {
			rc = -ENOTSUPP;
			return rc;
		}

	rc = ov5640_probe_init_gpio(info);
	if (rc < 0)
		return rc;

	ov5640_power_off();

	/* SENSOR NEED MCLK TO DO I2C COMMUNICTION, OPEN CLK FIRST*/
	msm_camio_clk_rate_set(24000000);

	msleep(20);

	ov5640_power_on();
	ov5640_power_reset();

#if 0
	rc = ov5640_probe_readID(info);

	if (rc < 0) {
		printk(KERN_ERR"--CAMERA--ov5640_probe_readID Fail !!~~~~!!\n");
		printk(KERN_ERR"--CAMERA-- %s, unregister\n", __func__);
		i2c_del_driver(&ov5640_i2c_driver);
		ov5640_power_off();
		ov5640_probe_free_gpio(info);
		return rc;
	}

	ret = info->cam_id();
	printk(KERN_ERR"--Read Camera ID--:%s\n",ret ?"Chicony or no sensor":"MCNEX");
#endif
    s->s_init = ov5640_sensor_init;
    s->s_release = ov5640_sensor_release;
    s->s_config  = ov5640_sensor_config;

	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle  = 90;
    return 0;
}

static int __ov5640_probe(struct platform_device *pdev)
{
    return msm_camera_drv_start(pdev, ov5640_sensor_probe);

}

static struct platform_driver msm_camera_driver = {
    .probe = __ov5640_probe,
    .driver = {
        .name = "msm_camera_ov5640",
        .owner = THIS_MODULE,
    },
};

static int __init ov5640_init(void)
{
    LOGI("+%s()\n", __func__);
    return platform_driver_register(&msm_camera_driver);
}

module_init(ov5640_init);
