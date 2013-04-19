/* Copyright (c) 2009-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2012
/*----------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#ifdef T_QCI_IS3
#include <linux/wifi_tiwlan.h>
#endif
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#ifdef CONFIG_SPI_QSD
#include <linux/spi/spi.h>
#endif
#include <linux/msm_ssbi.h>
#include <linux/mfd/pmic8058.h>
#include <linux/leds.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/smsc911x.h>
#include <linux/ofn_atlab.h>
#include <linux/power_supply.h>
#include <linux/i2c/tsc2007.h>
#include <linux/input/kp_flip_switch.h>
#include <linux/leds-pmic8058.h>
#include <linux/input/cy8c_ts.h>
#include <linux/msm_adc.h>
#include <linux/dma-mapping.h>
#include <linux/regulator/consumer.h>
#if defined(T_QCI_IS3)
#include <linux/walkmotion.h>
#endif
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/msm_spi.h>
#include <mach/qdsp5v2/msm_lpa.h>
#include <mach/dma.h>
#include <linux/android_pmem.h>
#include <linux/input/msm_ts.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>
#include <mach/socinfo.h>
#include <mach/msm_memtypes.h>

#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include <mach/vreg.h>
#include <linux/platform_data/qcom_crypto_device.h>

#include "devices.h"
#include "timer.h"
#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
/* FUJITSU:2011-12-01 add smem start */
#include "proc_comm.h"
/* FUJITSU:2011-12-01 add smem end */
#endif
#include "pm.h"
#include "pm-boot.h"
#include "spm.h"
#include "acpuclock.h"
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/sdio_al.h>
#include "smd_private.h"
#include <linux/gpio_keys.h>

#if defined(T_QCI_IS3)
#include <linux/i2c/is3_power.h>
#endif

#include "board-msm7x30-regulator.h"
#include "pm.h"

#define MSM_UARTDM2_USE
#define MSM_SPI_CRCI_DISABLE

extern inline int qci_read_hw_id(void);

#if defined(T_QCI_IS3)
extern inline int qci_get_offline_charging(void);
extern inline int qci_get_recovery_charging(void);
#endif

static int bcm433x_bt_power = 0;
static int bcm433x_fmt_power = 0;

#define MSM_PMEM_SF_SIZE	0x1700000
#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_SIZE            0x780000
#else
#define MSM_FB_SIZE            0x500000
#endif
#if defined(T_QCI_IS3)
#define MSM_PMEM_ADSP_SIZE      0x2300000
#else
#define MSM_PMEM_ADSP_SIZE      0x1E00000
#endif
#define MSM_FLUID_PMEM_ADSP_SIZE	0x2800000
#define PMEM_KERNEL_EBI0_SIZE   0x600000
#define MSM_PMEM_AUDIO_SIZE     0x200000

#define PMIC_GPIO_INT		27
#define PMIC_VREG_WLAN_LEVEL	2900
#if !defined(T_QCI_IS3)
#define PMIC_GPIO_SDC4_EN_N	17  /* PMIC GPIO Number 18 */
#endif
#if defined(T_QCI_IS3)
static unsigned long vreg_sts, gpio_sts;
#endif

#define FPGA_SDCC_STATUS       0x8E0001A8

#define PMIC_GPIO_SDC4_PWR_EN_N 24  /* PMIC GPIO Number 25 */

#if defined(T_QCI_IS3)
#define PMIC_GPIO_CHARGER_STAT	19	/* PMIC GPIO Number 20 */
#define PMIC_GPIO_OVP_SW1_OFF		1	/* PMIC GPIO Number 2 */
#define PMIC_GPIO_OVP_SW2_OFF		2	/* PMIC GPIO Number 3 */
#define PMIC_GPIO_XCRADLE_STATE	31	/* PMIC GPIO Number 32 */
#define PMIC_USB_VBUS_MPP			7	/* PMIC MPP 8*/
#define PMIC_GPIO_VREFTHERM_FETON	12	/* PMIC GPIO Number 13 */
#define PMIC_GPIO_TEMP_FETON	13	/* PMIC GPIO Number 14 */
#endif

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)
#define PM8058_MPP_BASE			   PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)	   (pm_gpio + PM8058_MPP_BASE)

#define PMIC_GPIO_QUICKVX_CLK 37 /* PMIC GPIO 38 */

#if defined(T_QCI_IS3)
int set_spi_gpio_use_state = 0;
EXPORT_SYMBOL(set_spi_gpio_use_state);
static DEFINE_SPINLOCK(set_spi_gpio_use_state_lock);
#endif

#define	PM_FLIP_MPP 5 /* PMIC MPP 06 */

#if defined (T_QCI_IS3)
#define	PMIC_GPIO_USB_VBUS_MSM_SWON	0 /* PMIC GPIO 01 */
#endif //(T_QCI_IS3)

#define	PMIC_GPIO_WLAN_HOSTWAKEUP				30 /* PMIC GPIO 31 */ 

/* FUJITSU:2012-01-13 factory start */
#define FACTORY_DEFAULT_VALUE 0

static int factory_value = FACTORY_DEFAULT_VALUE;
void factory_read_nvitem_init(void);
#define NV_BOOT_TESTMODE_STATUS_I   10145
#define NV_BOOT_NONE                            0 /* normal boot */
#define NV_BOOT_TESTMODE                        1 /* testmode */
#define NV_BOOT_KERNEL_TEST                     2 /* kernel-testmode */
#define NV_BOOT_ANDROID_TEST            3 /* android-testmode */
/* FUJITSU:2012-01-13 factory end */

#define PMIC_GPIO_VOL_UP	3  /* PMIC GPIO Number 4 */
#define PMIC_GPIO_VOL_DOWN	4  /* PMIC GPIO Number 5 */
#define PMIC_GPIO_HOME	5  /* PMIC GPIO Number 6 */
#define PMIC_GPIO_BACK	6  /* PMIC GPIO Number 7 */
#define PMIC_GPIO_MENU	7  /* PMIC GPIO Number 8 */
struct vreg *vreg_l16_3p0;

#ifdef CONFIG_OV5640_IS3
#define	PMIC_GPIO_CAM_ID	19 /* PMIC GPIO 19 */
#endif

#if defined(T_QCI_IS3) && defined(CONFIG_ANDROID_RAM_CONSOLE)
static struct resource ram_console_resource[] = {
       {
               .flags  = IORESOURCE_MEM,
       }
};

static struct platform_device ram_console_device = {
       .name = "ram_console",
       .id = -1,
       .num_resources  = ARRAY_SIZE(ram_console_resource),
       .resource       = ram_console_resource,
};
#endif

struct pm8xxx_gpio_init_info {
	unsigned			gpio;
	struct pm_gpio			config;
};

#if defined(T_QCI_IS3)
static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable);
#endif

static int pm8058_gpios_init(void)
{
	int rc;

#if !defined(T_QCI_IS3)
	struct pm8xxx_gpio_init_info sdc4_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_EN_N),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
			.out_strength   = PM_GPIO_STRENGTH_MED,
			.output_value   = 0,
		},
	};
#endif

	struct pm8xxx_gpio_init_info sdc4_pwr_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_PWR_EN_N),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
			.out_strength   = PM_GPIO_STRENGTH_LOW,
			.output_value   = 0,
		},
	};

	#if defined (T_QCI_IS3)
	struct pm8xxx_gpio_init_info usb_vbus_msm_swon = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_USB_VBUS_MSM_SWON),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
			.output_value		= 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.out_strength   = PM_GPIO_STRENGTH_MED,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};
	#endif //(T_QCI_IS3)

#if defined(T_QCI_IS3)
   #if defined(CONFIG_IS3_POWER)
        struct pm8xxx_gpio_init_info smb329b_charger_stat = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CHARGER_STAT),
		{
			.direction      = PM_GPIO_DIR_IN,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};
       struct pm8xxx_gpio_init_info smb329b_ovp_sw1_off = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_OVP_SW1_OFF),
		{
			.direction      = PM_GPIO_DIR_IN,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};
	struct pm8xxx_gpio_init_info smb329b_ovp_sw2_off = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_OVP_SW2_OFF),
		{
			.direction      = PM_GPIO_DIR_IN,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};
	struct pm8xxx_gpio_init_info smb329b_xcradle_state = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_XCRADLE_STATE),
		{
			.direction      = PM_GPIO_DIR_IN,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};
	struct pm8xxx_mpp_config_data usb_vbus_mpp = {
		.type = PM8XXX_MPP_TYPE_D_INPUT,
		.level = PM8018_MPP_DIG_LEVEL_S3,
		.control = PM8XXX_MPP_DIN_TO_INT,
	};
	struct pm8xxx_gpio_init_info vreftherm_feton = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_VREFTHERM_FETON),
		{
				.direction		= PM_GPIO_DIR_OUT,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.output_value		= 0,
				.pull			= PM_GPIO_PULL_NO,
				.vin_sel		= PM8058_GPIO_VIN_S3,
				.out_strength		= PM_GPIO_STRENGTH_MED,
				.function		= PM_GPIO_FUNC_NORMAL,
				.inv_int_pol		= 0,
		},
	};
	struct pm8xxx_gpio_init_info temp_feton = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_TEMP_FETON),
		{
				.direction		= PM_GPIO_DIR_OUT,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.output_value		= 0,
				.pull			= PM_GPIO_PULL_NO,
				.vin_sel		= PM8058_GPIO_VIN_S3,
				.out_strength		= PM_GPIO_STRENGTH_MED,
				.function		= PM_GPIO_FUNC_NORMAL,
				.inv_int_pol		= 0,
		},
	};
   #endif
#endif

#ifdef CONFIG_OV5640_IS3
	struct pm8xxx_gpio_init_info ov5640_cam_id = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CAM_ID-1),
		{	
			.direction      = PM_GPIO_DIR_IN,
			.pull           = PM_GPIO_PULL_UP_1P5,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};
#endif
	struct pm8xxx_gpio_init_info vol_up_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_VOL_UP),
		{
			.direction      	= PM_GPIO_DIR_IN,
			.pull           	= PM_GPIO_PULL_UP_31P5,
			.vin_sel        	= PM8058_GPIO_VIN_S3,
			.function       	= PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    	= 1,
			.out_strength	= PM_GPIO_STRENGTH_NO,
		},
	};
	struct pm8xxx_gpio_init_info vol_down_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_VOL_DOWN),
		{
			.direction      	= PM_GPIO_DIR_IN,
			.pull           	= PM_GPIO_PULL_UP_31P5,
			.vin_sel        	= PM8058_GPIO_VIN_S3,
			.function       	= PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    	= 1,
			.out_strength	= PM_GPIO_STRENGTH_NO,
		},
	};
	struct pm8xxx_gpio_init_info menu_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_MENU),
		{
			.direction      	= PM_GPIO_DIR_IN,
			.pull           	= PM_GPIO_PULL_UP_31P5,
			.vin_sel        	= PM8058_GPIO_VIN_S3,
			.function       	= PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    	= 1,
			.out_strength	= PM_GPIO_STRENGTH_NO,
		},
	};
	struct pm8xxx_gpio_init_info back_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_BACK),
		{
			.direction      	= PM_GPIO_DIR_IN,
			.pull           	= PM_GPIO_PULL_UP_31P5,
			.vin_sel        	= PM8058_GPIO_VIN_S3,
			.function       	= PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    	= 1,
			.out_strength	= PM_GPIO_STRENGTH_NO,
		},
	};
	struct pm8xxx_gpio_init_info home_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HOME),
		{
			.direction      	= PM_GPIO_DIR_IN,
			.pull           	= PM_GPIO_PULL_UP_31P5,
			.vin_sel        	= PM8058_GPIO_VIN_S3,
			.function       	= PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    	= 1,
			.out_strength	= PM_GPIO_STRENGTH_NO,
		},
	};

#if defined(T_QCI_IS3)
       rc = pm8xxx_gpio_config(sdc4_pwr_en.gpio, &sdc4_pwr_en.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_SDC4_PWR_EN_N config failed: %d\n",
		       __func__, rc);
		return rc;
	}

	rc = gpio_request(sdc4_pwr_en.gpio, "sdc4_pwr_en");
	if (rc) {
		pr_err("PMIC_GPIO_SDC4_PWR_EN_N gpio_req failed: %d\n",
		       rc);
		return rc;
	}
	if (qci_get_offline_charging() == 1)
	{
	   gpio_set_value_cansleep(sdc4_pwr_en.gpio, 0);
	   clear_bit(4, &vreg_sts);
	}
	else
	{
	   gpio_set_value_cansleep(sdc4_pwr_en.gpio, 1);
	   set_bit(4, &vreg_sts);
	   clear_bit(4, &gpio_sts);
	   msm_sdcc_setup_gpio(4,1);
	}
#else
	if (machine_is_msm7x30_fluid()) {
		/* SCD4 gpio */
		rc = pm8xxx_gpio_config(sdc4_en.gpio, &sdc4_en.config);
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_EN_N config failed\n",
								 __func__);
			return rc;
		}
		rc = gpio_request(sdc4_en.gpio, "sdc4_en");
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_EN_N gpio_request failed\n",
				__func__);
			return rc;
		}
		gpio_set_value_cansleep(sdc4_en.gpio, 0);
	}
	/* FFA -> gpio_25 controls vdd of sdcc4 */
	else {
		/* SCD4 gpio_25 */
		rc = pm8xxx_gpio_config(sdc4_pwr_en.gpio, &sdc4_pwr_en.config);
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_PWR_EN_N config failed: %d\n",
			       __func__, rc);
			return rc;
		}

		rc = gpio_request(sdc4_pwr_en.gpio, "sdc4_pwr_en");
		if (rc) {
			pr_err("PMIC_GPIO_SDC4_PWR_EN_N gpio_req failed: %d\n",
			       rc);
			return rc;
		}
	}
#endif

	#if defined (T_QCI_IS3)
	rc = pm8xxx_gpio_config( usb_vbus_msm_swon.gpio, &usb_vbus_msm_swon.config );
	if( rc )
	{
		pr_err( "%s PMIC_GPIO_USB_VBUS_MSM_SWON config failed\n", __func__ );
		return rc;
	}
	rc = gpio_request( usb_vbus_msm_swon.gpio, "usb_vbus_msm_swon" );
	if( rc )
	{
		pr_err( "%s PMIC_GPIO_USB_VBUS_MSM_SWON gpio_request failed\n", __func__ );
		return rc;
	}
	gpio_set_value_cansleep( usb_vbus_msm_swon.gpio, 0 );
	#endif //(T_QCI_IS3)

#if defined(T_QCI_IS3)
   #if defined(CONFIG_IS3_POWER)
       rc = pm8xxx_gpio_config( smb329b_charger_stat.gpio, &smb329b_charger_stat.config );
	if (rc) {
		pr_err("%s PMIC_GPIO config failed\n", __func__);
		return rc;
	}
	rc = pm8xxx_gpio_config( smb329b_ovp_sw1_off.gpio, &smb329b_ovp_sw1_off.config );
	if (rc) {
		pr_err("%s PMIC_GPIO config failed\n", __func__);
		return rc;
	}
       rc = pm8xxx_gpio_config( smb329b_ovp_sw2_off.gpio, &smb329b_ovp_sw2_off.config );
	if (rc) {
		pr_err("%s PMIC_GPIO config failed\n", __func__);
		return rc;
	}
       rc = pm8xxx_gpio_config( smb329b_xcradle_state.gpio, &smb329b_xcradle_state.config );
	if (rc) {
		pr_err("%s PMIC_GPIO config failed\n", __func__);
		return rc;
	}
	rc = pm8xxx_mpp_config(PM8058_MPP_PM_TO_SYS(PMIC_USB_VBUS_MPP),&usb_vbus_mpp);
	if (rc) {
		pr_err("%s PMIC_MPP config failed\n", __func__);
		return rc;
	}
	if (qci_read_hw_id() >= 4)
	{
           rc = pm8xxx_gpio_config( vreftherm_feton.gpio, &vreftherm_feton.config );
	   if (rc) {
		pr_err("%s PMIC_GPIO 13 config failed\n", __func__);
		return rc;
	   }
           rc = pm8xxx_gpio_config( temp_feton.gpio, &temp_feton.config );
	   if (rc) {
		pr_err("%s PMIC_GPIO 14 config failed\n", __func__);
		return rc;
	   }
	}
   #endif
#endif

#ifdef CONFIG_OV5640_IS3
	rc = pm8xxx_gpio_config(ov5640_cam_id.gpio, &ov5640_cam_id.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_CAM_ID config failed\n", __func__);
		return rc;
	}
#endif
	rc = pm8xxx_gpio_config( vol_up_en.gpio, &vol_up_en.config );
	if (rc) {
		pr_err("%s PMIC_GPIO_VOL_UP config failed\n", __func__);
		return rc;
	}
	rc = pm8xxx_gpio_config( vol_down_en.gpio, &vol_down_en.config );
	if (rc) {
		pr_err("%s PMIC_GPIO_VOL_DOWN config failed\n", __func__);
		return rc;
	}
	rc = pm8xxx_gpio_config( menu_en.gpio, &menu_en.config );
	if (rc) {
		pr_err("%s PMIC_GPIO_MENU config failed\n", __func__);
		return rc;
	}
	rc = pm8xxx_gpio_config( back_en.gpio, &back_en.config );
	if (rc) {
		pr_err("%s PMIC_GPIO_BACK config failed\n", __func__);
		return rc;
	}
	rc = pm8xxx_gpio_config( home_en.gpio, &home_en.config );
	if (rc) {
		pr_err("%s PMIC_GPIO_HOME config failed\n", __func__);
		return rc;
	}
        return 0;
}

#ifdef CONFIG_OV5640_IS3
static unsigned int ov5640_cam_id_status(void)
{
	return (unsigned int)
		gpio_get_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CAM_ID-1));
}
#endif

struct platform_device qci_device_irda = {
	.name	= "MSM_UART_AUX",
	.id	= -1,
};

/* Regulator API support */

#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
static struct platform_device msm_proccomm_regulator_dev = {
	.name = PROCCOMM_REGULATOR_DEV_NAME,
	.id   = -1,
	.dev  = {
		.platform_data = &msm7x30_proccomm_regulator_data
	}
};
#endif

/*virtual key support */
static ssize_t tma300_vkeys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":50:842:80:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":170:842:80:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":290:842:80:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":410:842:80:100"
	"\n");
}

static struct kobj_attribute tma300_vkeys_attr = {
	.attr = {
		.mode = S_IRUGO,
	},
	.show = &tma300_vkeys_show,
};

static struct attribute *tma300_properties_attrs[] = {
	&tma300_vkeys_attr.attr,
	NULL
};

static struct attribute_group tma300_properties_attr_group = {
	.attrs = tma300_properties_attrs,
};

static struct kobject *properties_kobj;

static int pm8058_pwm_config(struct pwm_device *pwm, int ch, int on)
{
	struct pm_gpio pwm_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM8058_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};
	int	rc = -EINVAL;
	int	id, mode, max_mA;

	id = mode = max_mA = 0;
	switch (ch) {
	case 0:
	case 1:
	case 2:
		if (on) {
			id = 24 + ch;
			rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(id - 1),
							&pwm_gpio_config);
			if (rc)
				pr_err("%s: pm8xxx_gpio_config(%d): rc=%d\n",
				       __func__, id, rc);
		}
		break;

	case 3:
		id = PM_PWM_LED_KPD;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	case 4:
		id = PM_PWM_LED_0;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 40;
		break;

	case 5:
		id = PM_PWM_LED_2;
		mode = PM_PWM_CONF_PWM2;
		max_mA = 40;
		break;

	case 6:
		id = PM_PWM_LED_FLASH;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	default:
		break;
	}

	if (ch >= 3 && ch <= 6) {
		if (!on) {
			mode = PM_PWM_CONF_NONE;
			max_mA = 0;
		}
		rc = pm8058_pwm_config_led(pwm, id, mode, max_mA);
		if (rc)
			pr_err("%s: pm8058_pwm_config_led(ch=%d): rc=%d\n",
			       __func__, ch, rc);
	}

	return rc;
}

static int pm8058_pwm_enable(struct pwm_device *pwm, int ch, int on)
{
	int	rc;

	switch (ch) {
	case 7:
		rc = pm8058_pwm_set_dtest(pwm, on);
		if (rc)
			pr_err("%s: pwm_set_dtest(%d): rc=%d\n",
			       __func__, on, rc);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static struct gpio_keys_button gpio_keys_buttons[] = {
	{
		.code			= KEY_VOLUMEUP,
		.type			= EV_KEY,
		.gpio			= PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_VOL_UP),
		.active_low		= 0,
		.wakeup			= 0,
		.debounce_interval	= 5, /* ms */
		.desc			= "VOL_UP",
	},
	{
		.code			= KEY_VOLUMEDOWN,
		.type			= EV_KEY,
		.gpio			= PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_VOL_DOWN),
		.active_low		= 0,
		.wakeup			= 0,
		.debounce_interval	= 5, /* ms */
		.desc			= "VOL_DOWN",
	},
	{
		.code			= KEY_MENU,
		.type			= EV_KEY,
		.gpio			= PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_MENU),
		.active_low		= 0,
		.wakeup			= 0,
		.debounce_interval	= 5, /* ms */
		.desc			= "MENU",
	},
	{
		.code			= KEY_BACK,
		.type			= EV_KEY,
		.gpio			= PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_BACK),
		.active_low		= 0,
		.wakeup			= 0,
		.debounce_interval	= 5, /* ms */
		.desc			= "BACK",
	},
	{
		.code			= KEY_HOME,
		.type			= EV_KEY,
		.gpio			= PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HOME),
		.active_low		= 0,
		.wakeup			= 1,
		.debounce_interval	= 5, /* ms */
		.desc			= "HOME",
	},
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons        = gpio_keys_buttons,
	.nbuttons       = ARRAY_SIZE(gpio_keys_buttons),
	.rep		= 0,
};

static struct platform_device msm_gpio_keys = {
	.name           = "gpio-keys",
	.id             = -1,
	.dev            = {
		.platform_data  = &gpio_keys_data,
	},
};

static struct pm8058_pwm_pdata pm8058_pwm_data = {
	.config         = pm8058_pwm_config,
	.enable         = pm8058_pwm_enable,
};

static struct pmic8058_led pmic8058_ffa_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
};

static struct pmic8058_leds_platform_data pm8058_ffa_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_ffa_leds),
	.leds	= pmic8058_ffa_leds,
};

static struct pmic8058_led pmic8058_surf_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
	[1] = {
		.name		= "voice:red",
		.max_brightness = 20,
		.id		= PMIC8058_ID_LED_0,
	},
	[2] = {
		.name		= "wlan:green",
		.max_brightness = 20,
		.id		= PMIC8058_ID_LED_2,
	},
};

static struct pmic8058_leds_platform_data pm8058_surf_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_surf_leds),
	.leds	= pmic8058_surf_leds,
};

static struct pmic8058_led pmic8058_fluid_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
	[1] = {
		.name		= "flash:led_0",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_0,
	},
	[2] = {
		.name		= "flash:led_1",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_1,
	},
};


/* FUJITSU:2011-12-1 add LED start */
static struct pmic8058_led pmic8058_leds[] = {
        [0] = {
                .name           = "red",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_0,
        },
        [1] = {
                .name           = "green",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_1,
        },
        [2] = {
                .name           = "blue",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_2,
        },
        [3] = {
                .name           = "yellow",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_01,
        },
        [4] = {
                .name           = "magenta",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_02,
        },
        [5] = {
                .name           = "cyan",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_12,
        },
        [6] = {
                .name           = "white",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_012,
        },
};

static struct pmic8058_leds_platform_data pm8058_leds_data = {
        .num_leds = ARRAY_SIZE(pmic8058_leds),
        .leds   = pmic8058_leds,
};
/* FUJITSU:2011-12-1 add LED end */

static struct pmic8058_leds_platform_data pm8058_fluid_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_fluid_leds),
	.leds	= pmic8058_fluid_leds,
};

static struct pm8xxx_irq_platform_data pm8xxx_irq_pdata = {
	.irq_base		= PMIC8058_IRQ_BASE,
	.devirq			= MSM_GPIO_TO_INT(PMIC_GPIO_INT),
	.irq_trigger_flag       = IRQF_TRIGGER_LOW,
};

static struct pm8xxx_gpio_platform_data pm8xxx_gpio_pdata = {
	.gpio_base		= PM8058_GPIO_PM_TO_SYS(0),
};

static struct pm8xxx_mpp_platform_data pm8xxx_mpp_pdata = {
	.mpp_base	= PM8058_MPP_PM_TO_SYS(0),
};

static struct pm8058_platform_data pm8058_7x30_data = {
	.irq_pdata		= &pm8xxx_irq_pdata,
	.gpio_pdata		= &pm8xxx_gpio_pdata,
	.mpp_pdata		= &pm8xxx_mpp_pdata,
	.pwm_pdata		= &pm8058_pwm_data,
};

#ifdef CONFIG_MSM_SSBI
static struct msm_ssbi_platform_data msm7x30_ssbi_pm8058_pdata = {
	.rsl_id = "D:PMIC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI2,
	.slave	= {
		.name			= "pm8058-core",
		.platform_data		= &pm8058_7x30_data,
	},
};
#endif
static struct i2c_board_info ft5x0x_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("ft5x0x_ts", 0x38),
	},
};

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_MT9D112
	{
		I2C_BOARD_INFO("mt9d112", 0x78 >> 1),
	},
#endif
#ifdef CONFIG_WEBCAM_OV9726
	{
		I2C_BOARD_INFO("ov9726", 0x10),
	},
#endif
#ifdef CONFIG_S5K3E2FX
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
#endif
#ifdef CONFIG_MT9P012
	{
		I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
	},
#endif
#ifdef CONFIG_VX6953
	{
		I2C_BOARD_INFO("vx6953", 0x20),
	},
#endif
#ifdef CONFIG_MT9E013
	{
		I2C_BOARD_INFO("mt9e013", 0x6C >> 2),
	},
#endif
#ifdef CONFIG_SN12M0PZ
	{
		I2C_BOARD_INFO("sn12m0pz", 0x34 >> 1),
	},
#endif
#if defined(CONFIG_MT9T013) || defined(CONFIG_SENSORS_MT9T013)
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),
	},
#endif

#if defined(CONFIG_OV5640_IS3)
	{
		I2C_BOARD_INFO("ov5640", 0x3C),
	},	
#endif
};

#ifdef CONFIG_MSM_CAMERA
#define	CAM_STNDBY	143

static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
#ifndef T_QCI_IS3	
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* RST */
	GPIO_CFG(2,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
#endif
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
#ifndef T_QCI_IS3	
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* RST */
	GPIO_CFG(2,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
#endif
	GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}
static int config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));

	return 0;
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));

}

struct resource msm_camera_resources[] = {
	{
		.start	= 0xA6000000,
		.end	= 0xA6000000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.flags  = IORESOURCE_DMA,
	}
};

struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 24000000,
	#ifdef T_QCI_IS3
	.ioclk.vfe_clk_rate  = 192000000,
	#else
	.ioclk.vfe_clk_rate  = 147456000,
	#endif

};

#if !defined(CONFIG_OV5640_IS3)
static struct msm_camera_sensor_flash_src msm_flash_src_pwm = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PWM,
	._fsrc.pwm_src.freq  = 1000,
	._fsrc.pwm_src.max_load = 300,
	._fsrc.pwm_src.low_load = 30,
	._fsrc.pwm_src.high_load = 100,
	._fsrc.pwm_src.channel = 7,
};
#endif

#ifdef CONFIG_MT9D112
static struct msm_camera_sensor_flash_data flash_mt9d112 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9d112_data = {
	.sensor_name    = "mt9d112",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9d112,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_mt9d112 = {
	.name      = "msm_camera_mt9d112",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9d112_data,
	},
};
#endif

#ifdef CONFIG_WEBCAM_OV9726

static struct msm_camera_sensor_platform_info ov9726_sensor_7630_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_ov9726 = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
	.flash_src	= &msm_flash_src_pwm
};
static struct msm_camera_sensor_info msm_camera_sensor_ov9726_data = {
	.sensor_name	= "ov9726",
	.sensor_reset	= 0,
	.sensor_pwd	= 85,
	.vcm_pwd	= 1,
	.vcm_enable	= 0,
	.pdata		= &msm_camera_device_data,
	.resource	= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data	= &flash_ov9726,
	.sensor_platform_info = &ov9726_sensor_7630_info,
	.csi_if		= 1
};
struct platform_device msm_camera_sensor_ov9726 = {
	.name	= "msm_camera_ov9726",
	.dev	= {
		.platform_data = &msm_camera_sensor_ov9726_data,
	},
};
#endif

#ifdef CONFIG_S5K3E2FX
static struct msm_camera_sensor_flash_data flash_s5k3e2fx = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name    = "s5k3e2fx",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_s5k3e2fx,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name      = "msm_camera_s5k3e2fx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};
#endif

#ifdef CONFIG_MT9P012
static struct msm_camera_sensor_flash_data flash_mt9p012 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
	.sensor_name    = "mt9p012",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9p012,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_mt9p012 = {
	.name      = "msm_camera_mt9p012",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p012_data,
	},
};
#endif

#ifdef CONFIG_MT9E013
static struct msm_camera_sensor_platform_info mt9e013_sensor_7630_info = {
	.mount_angle = 0
};

static struct msm_camera_sensor_flash_data flash_mt9e013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data = {
	.sensor_name    = "mt9e013",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9e013,
	.sensor_platform_info = &mt9e013_sensor_7630_info,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9e013 = {
	.name      = "msm_camera_mt9e013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data,
	},
};
#endif

#ifdef CONFIG_VX6953
static struct msm_camera_sensor_platform_info vx6953_sensor_7630_info = {
	.mount_angle = 0
};

static struct msm_camera_sensor_flash_data flash_vx6953 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};
static struct msm_camera_sensor_info msm_camera_sensor_vx6953_data = {
	.sensor_name    = "vx6953",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable		= 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.sensor_platform_info = &vx6953_sensor_7630_info,
	.flash_data     = &flash_vx6953,
	.csi_if         = 1
};
static struct platform_device msm_camera_sensor_vx6953 = {
	.name  	= "msm_camera_vx6953",
	.dev   	= {
		.platform_data = &msm_camera_sensor_vx6953_data,
	},
};
#endif

#ifdef CONFIG_SN12M0PZ
static struct msm_camera_sensor_flash_src msm_flash_src_current_driver = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	._fsrc.current_driver_src.low_current = 210,
	._fsrc.current_driver_src.high_current = 700,
	._fsrc.current_driver_src.driver_channel = &pm8058_fluid_leds_data,
};

static struct msm_camera_sensor_flash_data flash_sn12m0pz = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_current_driver
};
static struct msm_camera_sensor_info msm_camera_sensor_sn12m0pz_data = {
	.sensor_name    = "sn12m0pz",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_sn12m0pz,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_sn12m0pz = {
	.name      = "msm_camera_sn12m0pz",
	.dev       = {
		.platform_data = &msm_camera_sensor_sn12m0pz_data,
	},
};
#endif

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_flash_data flash_mt9t013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
	.sensor_name    = "mt9t013",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9t013,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9t013 = {
	.name      = "msm_camera_mt9t013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9t013_data,
	},
};
#endif

#if defined(CONFIG_OV5640_IS3)
static struct msm_camera_sensor_flash_data flash_ov5640 = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = NULL
};
static struct msm_camera_sensor_info msm_camera_sensor_ov5640_data = {
	.sensor_name    = "ov5640_u_is3",
	.sensor_reset   = 31,
	.sensor_pwd     = 157,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_ov5640,
	.csi_if         = 1,/*0:use CAMIF interface. 1:use CSI interface*/
	.cam_id      = ov5640_cam_id_status
};

static struct platform_device msm_camera_sensor_ov5640 = {
	.name   = "msm_camera_ov5640",
	.dev    = {
		.platform_data = &msm_camera_sensor_ov5640_data,
	},
};
#endif

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0xA3A00000,
		.end    = 0xA3A00000 + 0x0150 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0xAD200000,
		.end	= 0xAD200000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
       .name = "msm_vpe",
       .id   = 0,
       .num_resources = ARRAY_SIZE(msm_vpe_resources),
       .resource = msm_vpe_resources,
};
#endif

#endif /*CONFIG_MSM_CAMERA*/

#ifdef CONFIG_MSM7KV2_AUDIO
#if defined(T_QCI_IS3)
static uint32_t audio_pamp_gpio_config =
   GPIO_CFG(148, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
#else
static uint32_t audio_pamp_gpio_config =
   GPIO_CFG(82, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
#endif

static uint32_t audio_fluid_icodec_tx_config =
  GPIO_CFG(85, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static int __init snddev_poweramp_gpio_init(void)
{
	int rc;

	pr_info("snddev_poweramp_gpio_init \n");
	rc = gpio_tlmm_config(audio_pamp_gpio_config, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, audio_pamp_gpio_config, rc);
	}
	return rc;
}

void msm_snddev_tx_route_config(void)
{
	int rc;

	pr_debug("%s()\n", __func__);

	if (machine_is_msm7x30_fluid()) {
		rc = gpio_tlmm_config(audio_fluid_icodec_tx_config,
		GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_fluid_icodec_tx_config, rc);
		} else
			gpio_set_value(85, 0);
	}
}

void msm_snddev_tx_route_deconfig(void)
{
	int rc;

	pr_debug("%s()\n", __func__);

	if (machine_is_msm7x30_fluid()) {
		rc = gpio_tlmm_config(audio_fluid_icodec_tx_config,
		GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_fluid_icodec_tx_config, rc);
		}
	}
}

void msm_snddev_poweramp_on(void)
{
#if defined(T_QCI_IS3)
	gpio_tlmm_config( GPIO_CFG(148, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_set_value(148,1);
#else
	gpio_set_value(82, 1);	/* enable spkr poweramp */
#endif
	pr_info("%s: power on amplifier\n", __func__);
}

void msm_snddev_poweramp_off(void)
{
#if defined(T_QCI_IS3)
	gpio_set_value(148,0);
	gpio_tlmm_config( GPIO_CFG(148, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
#else
	gpio_set_value(82, 0);	/* disable spkr poweramp */
#endif
	pr_info("%s: power off amplifier\n", __func__);
}

#if defined(T_QCI_IS3)
void msm_snddev_poweramp_on_camera(void)
{
       int i;

       gpio_tlmm_config( GPIO_CFG(148, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
       gpio_set_value(148,1);
       udelay(1500);

       for (i = 0; i < 4; i++)
       {
               gpio_set_value(148,0);
               udelay(10);
               gpio_set_value(148,1);
               udelay(10);
       }
       pr_info("%s: power on amplifier for camera\n", __func__);
}

void msm_snddev_poweramp_off_camera(void)
{
       gpio_set_value(148,0);
       gpio_tlmm_config( GPIO_CFG(148, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
       pr_info("%s: power off amplifier for camera\n", __func__);
}
#endif


#if defined(T_QCI_IS3)
static struct regulator_bulk_data snddev_regs[] = {
	{ .supply = "ncp", .min_uV = 1800000, .max_uV = 1800000 },
};
#else
static struct regulator_bulk_data snddev_regs[] = {
	{ .supply = "gp4", .min_uV = 2600000, .max_uV = 2600000 },
	{ .supply = "ncp", .min_uV = 1800000, .max_uV = 1800000 },
};
#endif

static int __init snddev_hsed_voltage_init(void)
{
	int rc;

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc) {
		pr_err("%s: could not set regulator voltages: %d\n",
				__func__, rc);
		goto regs_free;
	}

	return 0;

regs_free:
	regulator_bulk_free(ARRAY_SIZE(snddev_regs), snddev_regs);
out:
	return rc;
}


void msm_snddev_hsed_voltage_on(void)
{
	int rc = regulator_bulk_enable(ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc)
		pr_err("%s: could not enable regulators: %d\n", __func__, rc);
}

void msm_snddev_hsed_voltage_off(void)
{
	int rc = regulator_bulk_disable(ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc) {
		pr_err("%s: could not disable regulators: %d\n", __func__, rc);
	}
}

static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_CLK  */
};

static int __init aux_pcm_gpio_init(void)
{
	int pin, rc;

	pr_info("aux_pcm_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(aux_pcm_gpio_on); pin++) {
		rc = gpio_tlmm_config(aux_pcm_gpio_on[pin],
					GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, aux_pcm_gpio_on[pin], rc);
		}
	}
	return rc;
}

static struct msm_gpio mi2s_clk_gpios[] = {
	{ GPIO_CFG(144, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),"MI2S_WS"},
	{ GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),"MI2S_SCLK"},
};

static struct msm_gpio mi2s_clk_gpios_HW_A[] = {
};
static unsigned fm_i2s_gpio_on[] = {
	GPIO_CFG(144, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MI2S_WS */
	GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MI2S_SCLK*/
	GPIO_CFG(146, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MI2S_DATA_SD3 */
};

static int  fm_i2s_gpio_init(void)
{
	int pin, rc;

	pr_info("fm_i2s_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(fm_i2s_gpio_on); pin++) {
		rc = gpio_tlmm_config(fm_i2s_gpio_on[pin],
					GPIO_CFG_ENABLE);
		printk("%s: gpio_tlmm_config(%d) rc=%d\n",__func__, GPIO_PIN(fm_i2s_gpio_on[pin]), rc);

		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%d) rc=%d\n",	__func__, GPIO_PIN(fm_i2s_gpio_on[pin]), rc);
		}
	}
	return rc;
}

static struct msm_gpio mi2s_rx_data_lines_gpios[] = {
	{ GPIO_CFG(121, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD0_A"},
	{ GPIO_CFG(122, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD1_A"},
	{ GPIO_CFG(123, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD2_A"},
	{ GPIO_CFG(146, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

static struct msm_gpio mi2s_tx_data_lines_gpios[] = {
	{ GPIO_CFG(146, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

int mi2s_config_clk_gpio(void)
{
	int rc = 0;

if ( qci_read_hw_id() == 1 )  {

	rc = msm_gpios_request_enable(mi2s_clk_gpios_HW_A,
			ARRAY_SIZE(mi2s_clk_gpios_HW_A));
	if (rc) {
		pr_err("%s: enable mi2s clk gpios  failed\n",
					__func__);
		return rc;
	}
}else{
	rc = msm_gpios_request_enable(mi2s_clk_gpios,2);
	if (rc) {
		pr_err("%s: enable mi2s clk gpios  failed\n",
					__func__);
		return rc;
	}
}

	return 0;
}

int  mi2s_unconfig_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i, rc = 0;
	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		msm_gpios_disable_free(mi2s_tx_data_lines_gpios, 1);
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask) {
			if (sd_line_mask & 0x1)
				msm_gpios_disable_free(
					mi2s_rx_data_lines_gpios + i , 1);
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
						__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_config_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i , rc = 0;
	u8 sd_config_done_mask = 0;

	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		if ((sd_line_mask & MI2S_SD_0) || (sd_line_mask & MI2S_SD_1) ||
		   (sd_line_mask & MI2S_SD_2) || !(sd_line_mask & MI2S_SD_3)) {
			pr_err("%s: can not use SD0 or SD1 or SD2 for TX"
				".only can use SD3. sd_line_mask = 0x%x\n",
				__func__ , sd_line_mask);
			rc = -EINVAL;
		} else {
			rc = msm_gpios_request_enable(mi2s_tx_data_lines_gpios,
							 1);
			if (rc)
				pr_err("%s: enable mi2s gpios for TX failed\n",
					   __func__);
		}
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask && (rc == 0)) {
			if (sd_line_mask & 0x1) {
				rc = msm_gpios_request_enable(
					mi2s_rx_data_lines_gpios + i , 1);
				if (rc) {
					pr_err("%s: enable mi2s gpios for"
					 "RX failed.  SD line = %s\n",
					 __func__,
					 (mi2s_rx_data_lines_gpios + i)->label);
					mi2s_unconfig_data_gpio(DIR_RX,
						sd_config_done_mask);
				} else
					sd_config_done_mask |= (1 << i);
			}
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
			__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_unconfig_clk_gpio(void)
{
	msm_gpios_disable_free(mi2s_clk_gpios, ARRAY_SIZE(mi2s_clk_gpios));
	return 0;
}

#endif /* CONFIG_MSM7KV2_AUDIO */

static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 1, GPIO_CFG_INPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);

	return 0;
}
#if !defined(T_QCI_IS3)
#define TIMPANI_RESET_GPIO	1
#endif

struct bahama_config_register{
	u8 reg;
	u8 value;
	u8 mask;
};

enum version{
	VER_1_0,
	VER_2_0,
	VER_UNSUPPORTED = 0xFF
};

static struct regulator *vreg_marimba_1;
static struct regulator *vreg_marimba_2;
static struct regulator *vreg_bahama;
#if !defined(T_QCI_IS3)
static struct msm_gpio timpani_reset_gpio_cfg[] = {
{ GPIO_CFG(TIMPANI_RESET_GPIO, 0, GPIO_CFG_OUTPUT,
	GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "timpani_reset"} };

static int config_timpani_reset(void)
{
	int rc;

	rc = msm_gpios_request_enable(timpani_reset_gpio_cfg,
				ARRAY_SIZE(timpani_reset_gpio_cfg));
	if (rc < 0) {
		printk(KERN_ERR
			"%s: msm_gpios_request_enable failed (%d)\n",
				__func__, rc);
	}
	return rc;
}
#endif
static unsigned int msm_timpani_setup_power(void)
{
	int rc;
#if !defined(T_QCI_IS3)
	rc = config_timpani_reset();
	if (rc < 0)
		goto out;
#endif
	rc = regulator_enable(vreg_marimba_1);
	if (rc) {
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);
		goto out;
	}

	rc = regulator_enable(vreg_marimba_2);
	if (rc) {
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);
		goto disable_marimba_1;
	}
#if !defined(T_QCI_IS3)
	rc = gpio_direction_output(TIMPANI_RESET_GPIO, 1);
	if (rc < 0) {
		pr_err("%s: gpio_direction_output failed (%d)\n",
				__func__, rc);
		msm_gpios_free(timpani_reset_gpio_cfg,
				ARRAY_SIZE(timpani_reset_gpio_cfg));
		goto disable_marimba_2;
	}
#endif

	return 0;
#if !defined(T_QCI_IS3)
disable_marimba_2:
	regulator_disable(vreg_marimba_2);
#endif
disable_marimba_1:
	regulator_disable(vreg_marimba_1);
out:
	return rc;
};

static void msm_timpani_shutdown_power(void)
{
	int rc;

	rc = regulator_disable(vreg_marimba_2);
	if (rc)
		pr_err("%s: regulator_disable failed (%d)\n", __func__, rc);

	rc = regulator_disable(vreg_marimba_1);
	if (rc)
		pr_err("%s: regulator_disable failed (%d)\n", __func__, rc);
#if !defined(T_QCI_IS3)
	rc = gpio_direction_output(TIMPANI_RESET_GPIO, 0);
	if (rc < 0)
		pr_err("%s: gpio_direction_output failed (%d)\n",
				__func__, rc);

	msm_gpios_free(timpani_reset_gpio_cfg,
				   ARRAY_SIZE(timpani_reset_gpio_cfg));
#endif
};


/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66

#define BAHAMA_SLAVE_ID_QMEMBIST_ADDR   0x7B

static struct regulator_bulk_data codec_regs[] = {
	{ .supply = "s4", .min_uV = 2200000, .max_uV = 2200000 },
};

static int __init msm_marimba_codec_init(void)
{
	int rc = regulator_bulk_get(NULL, ARRAY_SIZE(codec_regs), codec_regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(codec_regs), codec_regs);
	if (rc) {
		pr_err("%s: could not set regulator voltages: %d\n",
				__func__, rc);
		goto reg_free;
	}

	return rc;

reg_free:
	regulator_bulk_free(ARRAY_SIZE(codec_regs), codec_regs);
out:
	return rc;
}

static int msm_marimba_codec_power(int vreg_on)
{
	int rc = vreg_on ?
		regulator_bulk_enable(ARRAY_SIZE(codec_regs), codec_regs) :
		regulator_bulk_disable(ARRAY_SIZE(codec_regs), codec_regs);

	if (rc) {
		pr_err("%s: could not %sable regulators: %d",
				__func__, vreg_on ? "en" : "dis", rc);
		return rc;
	}

	return 0;
}

static void __init msm7x30_init_marimba(void)
{
	int rc;

	struct regulator_bulk_data regs[] = {
		{ .supply = "s3",   .min_uV = 1800000, .max_uV = 1800000 },
		{ .supply = "gp16", .min_uV = 1200000, .max_uV = 1200000 },
		{ .supply = "usb2", .min_uV = 1800000, .max_uV = 1800000 },
	};

	rc = msm_marimba_codec_init();

	if (rc) {
		pr_err("%s: msm_marimba_codec_init failed (%d)\n",
				__func__, rc);
		return;
	}

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs), regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		return;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs), regs);

	if (rc) {
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		regulator_bulk_free(ARRAY_SIZE(regs), regs);
		return;
	}

	vreg_marimba_1 = regs[0].consumer;
	vreg_marimba_2 = regs[1].consumer;
	vreg_bahama    = regs[2].consumer;
}

static struct marimba_codec_platform_data timpani_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
#ifdef CONFIG_TIMPANI_CODEC
	.snddev_profile_init = msm_snddev_init_timpani,
#endif
};

static struct marimba_platform_data timpani_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_CDC]	= MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_timpani_setup_power,
	.marimba_shutdown = msm_timpani_shutdown_power,
	.codec = &timpani_codec_pdata,
	.tsadc_ssbi_adap = MARIMBA_SSBI_ADAP,
};

#define TIMPANI_I2C_SLAVE_ADDR	0xD

static struct i2c_board_info msm_i2c_gsbi7_timpani_info[] = {
	{
		I2C_BOARD_INFO("timpani", TIMPANI_I2C_SLAVE_ADDR),
		.platform_data = &timpani_pdata,
	},
};

#ifdef CONFIG_MSM7KV2_AUDIO
static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};

static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x3ff8,
		.sb_min_addr = 0,
		.sb_max_addr = 0,
	},
	.sb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x37f8,
		.sb_min_addr = 0x3800,
		.sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
		.name = "lpa",
		.start = 0xa5000000,
		.end = 0xa50000a0,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name = "aux_codec_reg_addr",
		.start = 0xac9c00c0,
		.end = 0xac9c00c8,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 138,
		.end    = 138,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 139,
		.end    = 139,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 140,
		.end    = 140,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 141,
		.end    = 141,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};
#endif /* CONFIG_MSM7KV2_AUDIO */

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0,
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_MODE_LP)|
	(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	 /* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 2), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 2), /* WMA */
		DEC_INSTANCE(3, 2), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(1, 1), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 1), /* WMAPRO */
	},
	/* Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 3), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 3), /* WMA */
		DEC_INSTANCE(4, 3), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(2, 3), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 2), /* WMAPRO */
	},
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};
#if defined(T_QCI_IS3)
static struct fj_wm_platform_data fj_wm_device_platform_data = {
	.motion_irq = MSM_GPIO_TO_INT(42),
	.mc_init_delay = 20,
};

static struct platform_device fj_walkmotion_device = {
	.name           = "fj-walkmotion",
	.id             = 0,
	.dev = {
		.platform_data = &fj_wm_device_platform_data
	},
};
#endif

static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x8A000300,
		.end = 0x8A0003ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(156),
		.end = MSM_GPIO_TO_INT(156),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface	= PHY_INTERFACE_MODE_MII,
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_32BIT,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start		= 0x8D000000,
		.end		= 0x8D000100,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= MSM_GPIO_TO_INT(88),
		.end		= MSM_GPIO_TO_INT(88),
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static struct msm_gpio smsc911x_gpios[] = {
    { GPIO_CFG(172, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr6" },
    { GPIO_CFG(173, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr5" },
    { GPIO_CFG(174, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr4" },
    { GPIO_CFG(175, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr3" },
    { GPIO_CFG(176, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr2" },
    { GPIO_CFG(177, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr1" },
    { GPIO_CFG(178, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr0" },
    { GPIO_CFG(88, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "smsc911x_irq"  },
};

static void msm7x30_cfg_smsc911x(void)
{
	int rc;

	rc = msm_gpios_request_enable(smsc911x_gpios,
			ARRAY_SIZE(smsc911x_gpios));
	if (rc)
		pr_err("%s: unable to enable gpios\n", __func__);
}

#ifdef CONFIG_USB_G_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

#if defined(T_QCI_IS3)
  #if defined(CONFIG_IS3_POWER)
  static struct smb329b_platform_data smb329b_pdata={
      .charger_state_gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CHARGER_STAT),
      .ovp_sw1_off_gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_OVP_SW1_OFF),
      .ovp_sw2_off_gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_OVP_SW2_OFF),
      .xcradle_state_gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_XCRADLE_STATE),
      .usb_vbus_mpp = PM8058_MPP_PM_TO_SYS(PMIC_USB_VBUS_MPP),
};
  #endif
#endif

#ifdef CONFIG_FUJITSU_LIGHT
static struct i2c_board_info taos_board_info[] = {
	{
		I2C_BOARD_INFO("tsl2771", 0x39),
	},
};
static struct i2c_board_info stk_board_info[] = {
	{
		I2C_BOARD_INFO("stk_ps", 0x48),
	},
};
#endif

static struct i2c_board_info msm_i2c_board_info[] = {
#ifdef CONFIG_FUJITSU_COMPASS
	{
		I2C_BOARD_INFO("compass", 0x0c),
	},
#endif
#ifdef CONFIG_RADIO_MB86A30V
	{
		I2C_BOARD_INFO("mb86a30v", 0x11),
	},
#endif

#if defined(T_QCI_IS3)
   #if defined(CONFIG_IS3_POWER)
       {
		I2C_BOARD_INFO("max17040gt", 0x36),
	},
	{
		I2C_BOARD_INFO("smb329b", 0x37),
		.platform_data = &smb329b_pdata,
	},
   #endif
#endif
};

#if defined (T_QCI_IS3)
static struct i2c_board_info msm_i2c_board_info_offline_charging[] = {
   #if defined(CONFIG_IS3_POWER)
       {
		I2C_BOARD_INFO("max17040gt", 0x36),
	},
	{
		I2C_BOARD_INFO("smb329b", 0x37),
		.platform_data = &smb329b_pdata,
	},
   #endif
};
#endif

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
		.idle_supported = 1,
		.suspend_supported = 1,
/* FUJITSU:2012-05-03 modify disable idle power collapse start */
//		.idle_enabled = 1,
		.idle_enabled = 0,
/* FUJITSU:2012-05-03 modify disable idle power collapse end */
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
		.idle_supported = 1,
		.suspend_supported = 1,
/* FUJITSU:2012-05-03 modify disable idle power collapse start */
//	    .idle_enabled = 1,
	    .idle_enabled = 0,
/* FUJITSU:2012-05-03 modify disable idle power collapse end */
		.suspend_enabled = 1,
		.latency = 4594,
		.residency = 23740,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE] = {
#ifdef CONFIG_MSM_STANDALONE_POWER_COLLAPSE
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 0,
#else /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
		.idle_supported = 0,
		.suspend_supported = 0,
		.idle_enabled = 0,
		.suspend_enabled = 0,
#endif /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
		.latency = 500,
		.residency = 6000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 0,
		.suspend_enabled = 1,
		.latency = 443,
		.residency = 1098,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

u32 msm7x30_power_collapse_latency(enum msm_pm_sleep_mode mode)
{
	switch (mode) {
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency;
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency;
	case MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency;
	default:
	return 0;
	}
}

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_RESET_VECTOR_VIRT,
	.v_addr = (uint32_t *)PAGE_OFFSET,
};

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA8000000,
		.end	= 0xA8000000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "spidm_channels",
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.flags  = IORESOURCE_DMA,
	},
};

#define AMDH0_BASE_PHYS		0xAC200000
#define ADMH0_GP_CTL		(ct_adm_base + 0x3D8)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_adm_base = 0;
	u32 spi_mux = 0;

#ifdef MSM_SPI_CRCI_DISABLE
	u32 gp_ctl = 0;
#endif

	int ret = 0;

	ct_adm_base = ioremap(AMDH0_BASE_PHYS, PAGE_SIZE);
	if (!ct_adm_base) {
		pr_err("%s: Could not remap %x\n", __func__, AMDH0_BASE_PHYS);
		return -ENOMEM;
	}
#ifdef MSM_SPI_CRCI_DISABLE
	gp_ctl = ioread32(ADMH0_GP_CTL);

	gp_ctl = gp_ctl & 0xFFFFCFFF;
	iowrite32(gp_ctl,ADMH0_GP_CTL);

	gp_ctl = ioread32(ADMH0_GP_CTL);
#endif
	spi_mux = (ioread32(ADMH0_GP_CTL) & (0x3 << 12)) >> 12;

	qsd_spi_resources[4].start  = DMOV_USB_CHAN;
	qsd_spi_resources[4].end    = DMOV_TSIF_CHAN;

	switch (spi_mux) {
	case (1):
		qsd_spi_resources[5].start  = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[5].start  = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[5].start  = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end    = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -ENOENT;
	}

	iounmap(ct_adm_base);

	return ret;
}

static struct platform_device qsd_device_spi = {
	.name		= "spi_qsd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qsd_spi_resources),
	.resource	= qsd_spi_resources,
};

#ifdef CONFIG_SPI_QSD
 #if defined(T_QCI_IS3)
static struct spi_board_info lcdc_chimei_spi_board_info[] __initdata = {
	{
		.modalias	= "lcdc_chimei_hx8363",
		.mode		= SPI_MODE_3,
		.bus_num	= 0,
		.chip_select	= 1,
		.max_speed_hz	= 9963243,
	},
	{
		.modalias	= "wm0010_wolfson",
		.mode		= SPI_MODE_0,
		.bus_num	= 0,
		.chip_select	= 0,
/*		.max_speed_hz	= 26331429, FUJITSU:2012-2-6 wolfson clk change */
		.max_speed_hz	= 25896198, /* FUJITSU:2012-2-6 wolfson clk change */
	},
};
#else
static struct spi_board_info lcdc_sharp_spi_board_info[] __initdata = {
	{
		.modalias	= "lcdc_sharp_ls038y7dx01",
		.mode		= SPI_MODE_1,
		.bus_num	= 0,
		.chip_select	= 0,
		.max_speed_hz	= 26331429,
	}
};
static struct spi_board_info lcdc_toshiba_spi_board_info[] __initdata = {
	{
		.modalias       = "lcdc_toshiba_ltm030dd40",
		.mode           = SPI_MODE_3|SPI_CS_HIGH,
		.bus_num        = 0,
		.chip_select    = 0,
		.max_speed_hz   = 9963243,
	}
};
#endif
#endif

#if defined(T_QCI_IS3)
static struct msm_gpio qsd_spi_gpio_config_data[] = {
	{ GPIO_CFG(44, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "spi_cs1" },//#ifdef CONFIG_VARIANT_FJORD change add
	{ GPIO_CFG(45, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "spi_clk" },
	{ GPIO_CFG(46, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "spi_cs0" },
	{ GPIO_CFG(47, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "spi_mosi" },
	{ GPIO_CFG(48, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "spi_miso" },
};
#else
static struct msm_gpio qsd_spi_gpio_config_data[] = {
	{ GPIO_CFG(45, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_clk" },
	{ GPIO_CFG(46, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_cs0" },
	{ GPIO_CFG(47, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "spi_mosi" },
	{ GPIO_CFG(48, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_miso" },
};
#endif

static int msm_qsd_spi_gpio_config(void)
{
	return msm_gpios_request_enable(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26331429,
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
        int rc;
        static int vbus_is_on;
	struct pm8xxx_gpio_init_info usb_vbus = {
		PM8058_GPIO_PM_TO_SYS(36),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 1,
			.vin_sel        = 2,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};

        /* If VBUS is already on (or off), do nothing. */
        if (unlikely(on == vbus_is_on))
                return;

        if (on) {
/* FUJITSU:2012.02.23 USB start */
//		msm_hsusb_vbus_powerup();
/* FUJITSU:2012.02.23 USB end */
		rc = pm8xxx_gpio_config(usb_vbus.gpio, &usb_vbus.config);
		if (rc) {
                        pr_err("%s PMIC GPIO 36 write failed\n", __func__);
                        return;
                }
	} else {
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(36), 0);
/* FUJITSU:2012.02.23 USB start */
//		msm_hsusb_vbus_shutdown();
/* FUJITSU:2012.02.23 USB end */
	}

        vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
        .phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
        .vbus_power = msm_hsusb_vbus_power,
        .power_budget   = 180,
};
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static struct regulator *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	uint32_t version = 0;
	int def_vol = 3400000;

	version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) >= 2 &&
			SOCINFO_VERSION_MINOR(version) >= 1) {
		def_vol = 3075000;
		pr_debug("%s: default voltage:%d\n", __func__, def_vol);
	}

	if (init) {
		vreg_3p3 = regulator_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		regulator_set_voltage(vreg_3p3, def_vol, def_vol);
	} else
		regulator_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return regulator_enable(vreg_3p3);
	else
		return regulator_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return regulator_set_voltage(vreg_3p3, mV*1000, mV*1000);
}
#endif

#ifndef CONFIG_USB_EHCI_MSM_72K
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init);
#endif
static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,

#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init         = msm_hsusb_pmic_notif_init,
#else
	.vbus_power = msm_hsusb_vbus_power,
/* FUJITSU:2012.02.23 USB start */
//	.phy_power = hsusb_notifi_usb_ctrl_power,
//	.atom_notif_init = hsusb_cable_det_notifi_initialize_complete,
//	.atom_notif_cable_disconnect = hsusb_cable_det_notifi_disconnect,
//	.atom_notif_usb_vdd_off = hsusb_cable_usb_vdd_off,
/* FUJITSU:2012.02.23 USB end */
#endif
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
	.chg_connected		 = hsusb_chg_connected,
#if defined(T_QCI_IS3)
#if defined(CONFIG_IS3_POWER)
	.qci_chg_connected  = qci_chg_connected,
	.qci_offline_charging_status  = qci_offline_charging_status,
#endif
#endif
	.chg_init		 = hsusb_chg_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_set_voltage	 = msm_hsusb_ldo_set_voltage,
};

static struct platform_device memread_dev = {
	.name = "mread",
};

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};
#endif
#ifndef CONFIG_USB_EHCI_MSM_72K
typedef void (*notify_vbus_state) (int);
notify_vbus_state notify_vbus_state_func_ptr;
int vbus_on_irq;
static irqreturn_t pmic_vbus_on_irq(int irq, void *data)
{
	pr_info("%s: vbus notification from pmic\n", __func__);

	(*notify_vbus_state_func_ptr) (1);

	return IRQ_HANDLED;
}
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		if (!callback)
			return -ENODEV;

		notify_vbus_state_func_ptr = callback;
		vbus_on_irq = platform_get_irq_byname(&msm_device_otg,
			"vbus_on");
		if (vbus_on_irq <= 0) {
			pr_err("%s: unable to get vbus on irq\n", __func__);
			return -ENODEV;
		}

		ret = request_any_context_irq(vbus_on_irq, pmic_vbus_on_irq,
			IRQF_TRIGGER_RISING, "msm_otg_vbus_on", NULL);
		if (ret < 0) {
			pr_info("%s: request_irq for vbus_on"
				"interrupt failed\n", __func__);
			return ret;
		}
		msm_otg_pdata.pmic_vbus_irq = vbus_on_irq;
		return 0;
	} else {
		free_irq(vbus_on_irq, 0);
		notify_vbus_state_func_ptr = NULL;
		return 0;
	}
}
#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

#ifndef CONFIG_SPI_QSD
static int lcdc_gpio_array_num[] = {
				45, /* spi_clk */
				46, /* spi_cs  */
				47, /* spi_mosi */
				48, /* spi_miso */
				};

static struct msm_gpio lcdc_gpio_config_data[] = {
	{ GPIO_CFG(45, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_clk" },
	{ GPIO_CFG(46, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_cs0" },
	{ GPIO_CFG(47, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_mosi" },
	{ GPIO_CFG(48, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_miso" },
};

static void lcdc_config_gpios(int enable)
{
	if (enable) {
		msm_gpios_request_enable(lcdc_gpio_config_data,
					      ARRAY_SIZE(
						      lcdc_gpio_config_data));
	} else
		msm_gpios_disable_free(lcdc_gpio_config_data,
					    ARRAY_SIZE(
						    lcdc_gpio_config_data));
}
#endif

#if defined(T_QCI_IS3)
static struct msm_panel_common_pdata lcdc_chimei_panel_data = {
#ifndef CONFIG_SPI_QSD
	.panel_config_gpio = lcdc_config_gpios,
	.gpio_num          = lcdc_gpio_array_num,
#endif
	.gpio = 0, 	/* LPG PMIC_GPIO24 channel number */
};

static struct platform_device lcdc_chimei_panel_device = {
	.name   = "lcdc_chimei_wvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_chimei_panel_data,
	}
};
#endif

#ifdef MSM_UARTDM2_USE
static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
       .inject_rx_on_wakeup = 1,
       .rx_to_inject = 0xFD,
};
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
#if defined(T_QCI_IS3)
		if (!strcmp(name, "lcdc_chimei_wvga"))
			return 0;
#else
	if (machine_is_msm7x30_fluid()) {
		if (!strcmp(name, "lcdc_sharp_wvga_pt"))
			return 0;
	} else {
		if (!strncmp(name, "mddi_toshiba_wvga_pt", 20))
			return -EPERM;
		else if (!strncmp(name, "lcdc_toshiba_wvga_pt", 20))
			return 0;
		else if (!strcmp(name, "mddi_orise"))
			return -EPERM;
		else if (!strcmp(name, "mddi_quickvx"))
			return -EPERM;
	}
#endif
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct platform_device msm_migrate_pages_device = {
	.name   = "msm_migrate_pages",
	.id     = -1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
	.memory_type = MEMTYPE_EBI0,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
       .name = "pmem_audio",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
	.memory_type = MEMTYPE_EBI0,
};

static struct platform_device android_pmem_adsp_device = {
       .name = "android_pmem",
       .id = 2,
       .dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_audio_device = {
       .name = "android_pmem",
       .id = 4,
       .dev = { .platform_data = &android_pmem_audio_pdata },
};

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0xA8400000

#define QCE_HW_KEY_SUPPORT	1
#define QCE_SHA_HMAC_SUPPORT	0
#define QCE_SHARE_CE_RESOURCE	0
#define QCE_CE_SHARED		0

static struct resource qcrypto_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = DMOV_CE_HASH_CRCI,
		.end = DMOV_CE_HASH_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

static struct resource qcedev_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = DMOV_CE_HASH_CRCI,
		.end = DMOV_CE_HASH_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)

static struct msm_ce_hw_support qcrypto_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
	/* Bus Scaling declaration*/
	.bus_scale_table = NULL,
};

static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcrypto_resources),
	.resource	= qcrypto_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcrypto_ce_hw_suppport,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

static struct msm_ce_hw_support qcedev_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
	/* Bus Scaling declaration*/
	.bus_scale_table = NULL,
};
static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcedev_resources),
	.resource	= qcedev_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcedev_ce_hw_suppport,
	},
};
#endif

#if !defined(T_QCI_IS3)
static int mddi_toshiba_pmic_bl(int level)
{
	int ret = -EPERM;

	ret = pmic_set_led_intensity(LED_LCD, level);

	if (ret)
		printk(KERN_WARNING "%s: can't set lcd backlight!\n",
					__func__);
	return ret;
}

static struct msm_panel_common_pdata mddi_toshiba_pdata = {
	.pmic_backlight = mddi_toshiba_pmic_bl,
};

static struct platform_device mddi_toshiba_device = {
	.name   = "mddi_toshiba",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_toshiba_pdata,
	}
};
#endif
#if !defined(T_QCI_IS3) 
static unsigned wega_reset_gpio =
	GPIO_CFG(180, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);


static struct msm_gpio fluid_vee_reset_gpio[] = {
	{ GPIO_CFG(20, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "vee_reset" },
};
#endif

static unsigned char quickvx_mddi_client = 1, other_mddi_client = 1;
#if !defined(T_QCI_IS3) 
static unsigned char quickvx_ldo_enabled;

static unsigned quickvx_vlp_gpio =
	GPIO_CFG(97, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL,	GPIO_CFG_2MA);

static struct pm8xxx_gpio_init_info pmic_quickvx_clk_gpio = {
	PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_QUICKVX_CLK),
	{
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM8058_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	},
};
#endif

#if defined(T_QCI_IS3) 
static struct regulator *lcdc_ldo15;
static struct regulator *lcdc_ldo20;

static int display_common_init(void)
{
	struct regulator_bulk_data regs[2] = {
		{ .supply = "ldo15", /* voltage set in display_common_power */},
		{ .supply = "ldo20", /* voltage set in display_common_power */ },

	};

	int rc = 0;


	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs), regs);
	if (rc) {
		pr_err("%s: regulator_bulk_get failed: %d\n",
				__func__, rc);
		goto bail;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs), regs);
	if (rc) {
		pr_err("%s: regulator_bulk_set_voltage failed: %d\n",
				__func__, rc);
		goto put_regs;
	}

	lcdc_ldo15 = regs[0].consumer;
	lcdc_ldo20 = regs[1].consumer;

	return rc;

put_regs:
	regulator_bulk_free(ARRAY_SIZE(regs), regs);
bail:
	return rc;
}


#else

static struct regulator *mddi_ldo20;
static struct regulator *mddi_ldo12;
static struct regulator *mddi_ldo16;
static struct regulator *mddi_ldo6;
static struct regulator *mddi_lcd;

static int display_common_init(void)
{
	struct regulator_bulk_data regs[5] = {
		{ .supply = "ldo20", /* voltage set in display_common_power */},
		{ .supply = "ldo12", .min_uV = 1800000, .max_uV = 1800000 },
		{ .supply = "ldo6",  .min_uV = 3075000, .max_uV = 3400000 },
		{ .supply = "ldo16", .min_uV = 2600000, .max_uV = 2600000 },
		{ .supply = NULL,    /* mddi_lcd, initialized below */ },
	};

	int rc = 0;

	if (machine_is_msm7x30_fluid()) {
		/* lcd: LDO8 @1.8V */
		regs[4].supply = "ldo8";
		regs[4].min_uV = 1800000;
		regs[4].max_uV = 1800000;
	} else {
		/* lcd: LDO15 @3.1V */
		regs[4].supply = "ldo15";
		regs[4].min_uV = 3100000;
		regs[4].max_uV = 3100000;
	}

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs), regs);
	if (rc) {
		pr_err("%s: regulator_bulk_get failed: %d\n",
				__func__, rc);
		goto bail;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs), regs);
	if (rc) {
		pr_err("%s: regulator_bulk_set_voltage failed: %d\n",
				__func__, rc);
		goto put_regs;
	}

	mddi_ldo20 = regs[0].consumer;
	mddi_ldo12 = regs[1].consumer;
	mddi_ldo6  = regs[2].consumer;
	mddi_ldo16 = regs[3].consumer;
	mddi_lcd   = regs[4].consumer;

	return rc;

put_regs:
	regulator_bulk_free(ARRAY_SIZE(regs), regs);
bail:
	return rc;
}

#endif

#if defined(T_QCI_IS3) 
static unsigned wega_reset_gpio =
	GPIO_CFG(33, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA);

static int display_common_power(int on)
{

	int rc = 0;
#if defined(T_QCI_IS3)		 
	return 0;
#endif
		rc = display_common_init();
		if (rc) {
			pr_err("%s: regulator init failed: %d\n",
					__func__, rc);
			return rc;
		}


	if (on) {
		/* reset Toshiba WeGA chip -- toggle reset pin -- gpio_180 */
		rc = gpio_tlmm_config(wega_reset_gpio, GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, wega_reset_gpio, rc);
			return rc;
		}

		/* bring reset line high to hold reset*/
		gpio_set_value(33, 1);

	rc = regulator_set_voltage(lcdc_ldo20, 1800000, 1800000);
	if (rc) {
		pr_err("%s: could not set voltage for ldo20: %d\n",
				__func__, rc);
		return rc;
	}
       mdelay(5);
	rc = regulator_set_voltage(lcdc_ldo15, 3100000, 3100000);
	if (rc) {
		pr_err("%s: could not set voltage for ldo15: %d\n",
				__func__, rc);
		return rc;
	}

	}

	if (on) {
		rc = regulator_enable(lcdc_ldo20);
		if (rc) {
			pr_err("%s: lcdc_ldo20 regulator enable failed (%d)\n",
			       __func__, rc);
			return rc;
		}

		rc = regulator_enable(lcdc_ldo15);
		if (rc) {
			pr_err("%s: lcdc_ldo15 regulator enable failed (%d)\n",
			       __func__, rc);
			return rc;
		}
		mdelay(5);
		gpio_set_value(33, 0);	 /* bring reset line low */
		mdelay(20);
		gpio_set_value(33, 1);	 /* bring reset line high */
		mdelay(20); /* 10 msec before IO can be accessed */


	} else {

	}

	return rc;
}

#else
static int display_common_power(int on)
{
        #if defined(T_QCI_IS3)       
        return 0;
        #else
	int rc = 0, flag_on = !!on;
	static int display_common_power_save_on;
	static bool display_regs_initialized;

	if (display_common_power_save_on == flag_on)
		return 0;

	display_common_power_save_on = flag_on;

	if (unlikely(!display_regs_initialized)) {
		rc = display_common_init();
		if (rc) {
			pr_err("%s: regulator init failed: %d\n",
					__func__, rc);
			return rc;
		}
		display_regs_initialized = true;
	}


	if (on) {
		/* reset Toshiba WeGA chip -- toggle reset pin -- gpio_180 */
		rc = gpio_tlmm_config(wega_reset_gpio, GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, wega_reset_gpio, rc);
			return rc;
		}

		/* bring reset line low to hold reset*/
		gpio_set_value(180, 0);

		if (quickvx_mddi_client) {
			/* QuickVX chip -- VLP pin -- gpio 97 */
			rc = gpio_tlmm_config(quickvx_vlp_gpio,
				GPIO_CFG_ENABLE);
			if (rc) {
				pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, quickvx_vlp_gpio, rc);
				return rc;
			}

			/* bring QuickVX VLP line low */
			gpio_set_value(97, 0);

			rc = pm8xxx_gpio_config(pmic_quickvx_clk_gpio.gpio,
						&pmic_quickvx_clk_gpio.config);
			if (rc) {
				pr_err("%s: pm8xxx_gpio_config(%#x)=%d\n",
					__func__, pmic_quickvx_clk_gpio.gpio,
					rc);
				return rc;
			}

			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_QUICKVX_CLK), 0);
		}
	}

	if (quickvx_mddi_client)
		rc = regulator_set_voltage(mddi_ldo20, 1800000, 1800000);
	else
		rc = regulator_set_voltage(mddi_ldo20, 1500000, 1500000);

	if (rc) {
		pr_err("%s: could not set voltage for ldo20: %d\n",
				__func__, rc);
		return rc;
	}

	if (on) {
		rc = regulator_enable(mddi_ldo20);
		if (rc) {
			pr_err("%s: LDO20 regulator enable failed (%d)\n",
			       __func__, rc);
			return rc;
		}

		rc = regulator_enable(mddi_ldo12);
		if (rc) {
			pr_err("%s: LDO12 regulator enable failed (%d)\n",
			       __func__, rc);
			return rc;
		}

		if (other_mddi_client) {
			rc = regulator_enable(mddi_ldo16);
			if (rc) {
				pr_err("%s: LDO16 regulator enable failed (%d)\n",
					   __func__, rc);
				return rc;
			}
		}

		if (quickvx_ldo_enabled) {
			/* Disable LDO6 during display ON */
			rc = regulator_disable(mddi_ldo6);
			if (rc) {
				pr_err("%s: LDO6 regulator disable failed (%d)\n",
					   __func__, rc);
				return rc;
			}
			quickvx_ldo_enabled = 0;
		}

		rc = regulator_enable(mddi_lcd);
		if (rc) {
			pr_err("%s: LCD regulator enable failed (%d)\n",
				__func__, rc);
			return rc;
		}

		mdelay(5);		/* ensure power is stable */

		if (machine_is_msm7x30_fluid()) {
			rc = msm_gpios_request_enable(fluid_vee_reset_gpio,
					ARRAY_SIZE(fluid_vee_reset_gpio));
			if (rc)
				pr_err("%s gpio_request_enable failed rc=%d\n",
							__func__, rc);
			else {
				/* assert vee reset_n */
				gpio_set_value(20, 1);
				gpio_set_value(20, 0);
				mdelay(1);
				gpio_set_value(20, 1);
			}
		}

		gpio_set_value(180, 1); /* bring reset line high */
		mdelay(10);	/* 10 msec before IO can be accessed */

		if (quickvx_mddi_client) {
			gpio_set_value(97, 1);
			msleep(2);
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_QUICKVX_CLK), 1);
			msleep(2);
		}

		rc = pmapp_display_clock_config(1);
		if (rc) {
			pr_err("%s pmapp_display_clock_config rc=%d\n",
					__func__, rc);
			return rc;
		}

	} else {
		rc = regulator_disable(mddi_ldo20);
		if (rc) {
			pr_err("%s: LDO20 regulator disable failed (%d)\n",
			       __func__, rc);
			return rc;
		}


		if (other_mddi_client) {
			rc = regulator_disable(mddi_ldo16);
			if (rc) {
				pr_err("%s: LDO16 regulator disable failed (%d)\n",
					   __func__, rc);
				return rc;
			}
		}

		if (quickvx_mddi_client && !quickvx_ldo_enabled) {
			/* Enable LDO6 during display OFF for
			   Quicklogic chip to sleep with data retention */
			rc = regulator_enable(mddi_ldo6);
			if (rc) {
				pr_err("%s: LDO6 regulator enable failed (%d)\n",
					   __func__, rc);
				return rc;
			}
			quickvx_ldo_enabled = 1;
		}

		gpio_set_value(180, 0); /* bring reset line low */

		if (quickvx_mddi_client) {
			gpio_set_value(97, 0);
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_QUICKVX_CLK), 0);
		}

		rc = regulator_disable(mddi_lcd);
		if (rc) {
			pr_err("%s: LCD regulator disable failed (%d)\n",
				__func__, rc);
			return rc;
		}

		mdelay(5);	/* ensure power is stable */

		rc = regulator_disable(mddi_ldo12);
		if (rc) {
			pr_err("%s: LDO12 regulator disable failed (%d)\n",
			       __func__, rc);
			return rc;
		}

		if (machine_is_msm7x30_fluid()) {
			msm_gpios_disable_free(fluid_vee_reset_gpio,
					ARRAY_SIZE(fluid_vee_reset_gpio));
		}

		rc = pmapp_display_clock_config(0);
		if (rc) {
			pr_err("%s pmapp_display_clock_config rc=%d\n",
					__func__, rc);
			return rc;
		}
	}

	return rc;
        #endif
}
#endif

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static int msm_fb_mddi_client_power(u32 client_id)
{
#if 0
	int rc;
#endif
	printk(KERN_NOTICE "\n client_id = 0x%x", client_id);
	/* Check if it is Quicklogic client */
	if (client_id == 0xc5835800) {
		printk(KERN_NOTICE "\n Quicklogic MDDI client");
		other_mddi_client = 0;
#if 0
		if (IS_ERR(mddi_ldo16)) {
			rc = PTR_ERR(mddi_ldo16);
			pr_err("%s: gp10 vreg get failed (%d)\n", __func__, rc);
			return rc;
		}
		rc = regulator_disable(mddi_ldo16);
		if (rc) {
			pr_err("%s: LDO16 vreg enable failed (%d)\n",
							__func__, rc);
			return rc;
		}
#endif

	} else {
		printk(KERN_NOTICE "\n Non-Quicklogic MDDI client");
		quickvx_mddi_client = 0;
		gpio_set_value(97, 0);
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(
			PMIC_GPIO_QUICKVX_CLK), 0);
	}

	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = display_common_power,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
	.mddi_client_power = msm_fb_mddi_client_power,
};

int mdp_core_clk_rate_table[] = {
	192000000,
	192000000,
	192000000,
	192000000,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.hw_revision_addr = 0xac001270,
	.gpio = 30,
	.mdp_core_clk_rate = 192000000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
	.mdp_rev = MDP_REV_40,
};

#if !defined(T_QCI_IS3)
static int lcd_panel_spi_gpio_num[] = {
			45, /* spi_clk */
			46, /* spi_cs  */
			47, /* spi_mosi */
			48, /* spi_miso */
		};
#endif
static struct msm_gpio lcd_panel_gpios[] = {
/* Workaround, since HDMI_INT is using the same GPIO line (18), and is used as
 * input.  if there is a hardware revision; we should reassign this GPIO to a
 * new open line; and removing it will just ensure that this will be missed in
 * the future.
	{ GPIO_CFG(18, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn0" },
 */
 #if defined(T_QCI_IS3)
        { GPIO_CFG(18, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn0" },
	{ GPIO_CFG(19, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn1" },
	{ GPIO_CFG(20, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu0" },
	{ GPIO_CFG(21, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu1" },
	{ GPIO_CFG(22, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu2" },
	{ GPIO_CFG(23, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red0" },
	{ GPIO_CFG(24, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red1" },
	{ GPIO_CFG(25, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red2" },
#ifndef CONFIG_SPI_QSD
	{ GPIO_CFG(45, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "spi_clk" },
	{ GPIO_CFG(46, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "spi_cs0" },
	{ GPIO_CFG(47, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "spi_mosi" },
	{ GPIO_CFG(48, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "spi_miso" },
#endif
	{ GPIO_CFG(90, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "lcdc_pclk" },
	{ GPIO_CFG(91, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "lcdc_en" },
	{ GPIO_CFG(92, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "lcdc_vsync" },
	{ GPIO_CFG(93, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "lcdc_hsync" },
	{ GPIO_CFG(94, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn2" },
	{ GPIO_CFG(95, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn3" },
	{ GPIO_CFG(96, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn4" },
	{ GPIO_CFG(97, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn5" },
	{ GPIO_CFG(98, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn6" },
	{ GPIO_CFG(99, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn7" },
	{ GPIO_CFG(100, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu3" },
	{ GPIO_CFG(101, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu4" },
	{ GPIO_CFG(102, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu5" },
	{ GPIO_CFG(103, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu6" },
	{ GPIO_CFG(104, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu7" },
	{ GPIO_CFG(105, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red3" },
	{ GPIO_CFG(106, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red4" },
	{ GPIO_CFG(107, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red5" },
	{ GPIO_CFG(108, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red6" },
	{ GPIO_CFG(109, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red7" },
#endif
};
#if !defined(T_QCI_IS3)
static struct msm_gpio lcd_sharp_panel_gpios[] = {
	{ GPIO_CFG(22, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu2" },
	{ GPIO_CFG(25, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red2" },
	{ GPIO_CFG(90, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_pclk" },
	{ GPIO_CFG(91, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_en" },
	{ GPIO_CFG(92, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_vsync" },
	{ GPIO_CFG(93, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_hsync" },
	{ GPIO_CFG(94, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn2" },
	{ GPIO_CFG(95, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn3" },
	{ GPIO_CFG(96, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn4" },
	{ GPIO_CFG(97, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn5" },
	{ GPIO_CFG(98, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn6" },
	{ GPIO_CFG(99, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn7" },
	{ GPIO_CFG(100, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu3" },
	{ GPIO_CFG(101, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu4" },
	{ GPIO_CFG(102, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu5" },
	{ GPIO_CFG(103, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu6" },
	{ GPIO_CFG(104, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu7" },
	{ GPIO_CFG(105, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red3" },
	{ GPIO_CFG(106, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red4" },
	{ GPIO_CFG(107, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red5" },
	{ GPIO_CFG(108, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red6" },
	{ GPIO_CFG(109, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red7" },
};
#endif
#if !defined(T_QCI_IS3)

static int lcdc_toshiba_panel_power(int on)
{
	int rc, i;
	struct msm_gpio *gp;

	rc = display_common_power(on);
	if (rc < 0) {
		printk(KERN_ERR "%s display_common_power failed: %d\n",
				__func__, rc);
		return rc;
	}

	if (on) {
		rc = msm_gpios_enable(lcd_panel_gpios,
				ARRAY_SIZE(lcd_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio enable failed: %d\n",
					__func__, rc);
		}
	} else {	/* off */
		gp = lcd_panel_gpios;
		for (i = 0; i < ARRAY_SIZE(lcd_panel_gpios); i++) {
			gp++;
		}
	}

	return rc;
}

static int lcdc_sharp_panel_power(int on)
{
	int rc, i;
	struct msm_gpio *gp;

	rc = display_common_power(on);
	if (rc < 0) {
		printk(KERN_ERR "%s display_common_power failed: %d\n",
				__func__, rc);
		return rc;
	}

	if (on) {
		rc = msm_gpios_enable(lcd_sharp_panel_gpios,
				ARRAY_SIZE(lcd_sharp_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio enable failed: %d\n",
				__func__, rc);
		}
	} else {	/* off */
		gp = lcd_sharp_panel_gpios;
		for (i = 0; i < ARRAY_SIZE(lcd_sharp_panel_gpios); i++) {
			gp++;
		}
	}

	return rc;
}
#endif
 #if defined(T_QCI_IS3)
static int lcdc_chimei_panel_power(int on)
{
	int rc, i;
	struct msm_gpio *gp;
       printk(KERN_EMERG "[LCD]%s\n", __func__);
	rc = display_common_power(on);
	if (rc < 0) {
		printk(KERN_ERR "%s display_common_power failed: %d\n",
				__func__, rc);
		return rc;
	}

	if (on) {
		rc = msm_gpios_enable(lcd_panel_gpios,
				ARRAY_SIZE(lcd_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio enable failed: %d\n",
					__func__, rc);
		}
	} else {	/* off */
		gp = lcd_panel_gpios;
		for (i = 0; i < ARRAY_SIZE(lcd_panel_gpios); i++) {
			/* ouput low */
			gpio_set_value(GPIO_PIN(gp->gpio_cfg), 0);
			gp++;
		}
	}

	return rc;
}
 #endif
static int lcdc_panel_power(int on)
{
	int flag_on = !!on;
	static int lcdc_power_save_on, lcdc_power_initialized;

	if (lcdc_power_save_on == flag_on)
		return 0;

	lcdc_power_save_on = flag_on;

	if (unlikely(!lcdc_power_initialized)) {
		quickvx_mddi_client = 0;
		display_common_init();
		lcdc_power_initialized = 1;
	}
 #if defined(T_QCI_IS3)
		return lcdc_chimei_panel_power(on);
 #else
	if (machine_is_msm7x30_fluid())
		return lcdc_sharp_panel_power(on);
	else
		return lcdc_toshiba_panel_power(on);
#endif
}

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_power_save   = lcdc_panel_power,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
	msm_fb_register_device("lcdc", &lcdc_pdata);
#ifdef CONFIG_FB_MSM_TVOUT
	msm_fb_register_device("tvout_device", NULL);
#endif
}

#if !defined(T_QCI_IS3)

static struct msm_panel_common_pdata lcdc_toshiba_panel_data = {
	.gpio_num          = lcd_panel_spi_gpio_num,
};

static struct platform_device lcdc_toshiba_panel_device = {
	.name   = "lcdc_toshiba_wvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_toshiba_panel_data,
	}
};
#endif

static struct platform_device qci_fmt_power_device = {
	.name = "fmt_power",
};

static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

#define KO_PMICUART1_BT_TXD_21   (20)
#define KO_PMICUART1_BT_RXD_33   (32)
#define KO_PMICUART1_BT_PMIC_TXD_36 (35)
#define KO_PMICUART1_BT_PMIC_RXD_37 (36)

static unsigned bt_config_power_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_PCMOUT         */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCMIN          */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_PCM_FSYNC      */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_PCM_DCLK       */
	GPIO_CFG(147, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP,   GPIO_CFG_2MA),/* KO_BT_HOSTWAKEUP     */
	GPIO_CFG(150,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_WAKEUP         */
	GPIO_CFG(41,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_RESET          */
	GPIO_CFG(43,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_PWON           */
	GPIO_CFG(149,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_WLAN_BT_PWR_ONOFF */
	GPIO_CFG(39,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_WLAN_PWON         */
#ifdef MSM_UARTDM2_USE
	GPIO_CFG(49, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(50, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(51, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
#endif
	};

static unsigned bt_config_power_off[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCMOUT         */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCMIN          */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCM_FSYNC      */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCM_DCLK       */
	GPIO_CFG(147, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_HOSTWAKEUP     */
	GPIO_CFG(150,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_WAKEUP         */
	GPIO_CFG(41,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_RESET          */
	GPIO_CFG(43,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PWON           */
	GPIO_CFG(149,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_BT_PWR_ONOFF */
	GPIO_CFG(39,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_PWON         */
	};

/* Status (Power) */
static int wlan_power_flg = 0;
static int bt_power_flg = 0;

// see also: mach/gpio.h
static int bcm43xx_gpio_tlmm_config(unsigned int config, unsigned int disable)
{
	int retval;
	printk(KERN_INFO "%s: calling gpio_tlmm_config(): gpio=%u, func=0x%02X, dir=0x%02X, pull=0x%02X, drvstr=0x%02X => %d\n",
			__func__, GPIO_PIN(config), GPIO_FUNC(config), GPIO_DIR(config),
			GPIO_PULL(config), GPIO_DRVSTR(config), (retval = gpio_tlmm_config(config, disable)));
	return retval;
}

static int bcm43xx_gpio_request(int gpio, const char* label)
{
	int retval;
	printk(KERN_INFO "%s(): calling gpio_request(%d, %s) => %d\n", __func__, gpio, label,(retval = gpio_request(gpio, label)));
	return retval;
}

static void bcm43xx_gpio_set_value(int gpio, int value)
{
	int oldvalue = 0;
	int newvalue = 0;

	oldvalue = gpio_get_value(gpio);
	gpio_set_value(gpio, value);
	newvalue = gpio_get_value(gpio);
	printk(KERN_INFO "%s(): gpio_set_value(%d) : %d -> %d\n", __func__, gpio, oldvalue, newvalue);
}

static void bcm43xx_gpio_direction_output(int gpio, int value)
{
	printk(KERN_INFO "%s(): calling gpio_direction_output(%d) => %d\n", __func__, gpio, value);
	gpio_direction_output(gpio, value);
}

static void bcm43xx_gpio_direction_input(int gpio)
{
	printk(KERN_INFO "%s(): calling gpio_direction_input(%d)\n", __func__, gpio);
	gpio_direction_input(gpio);
}

/* Function (set power) */
void set_power_wlan_bt(int on, int is_wlan)
{
//	printk(KERN_DEBUG "%s S\n", __func__);
printk(KERN_ERR "%s S\n", __func__);

	if ( on )
	{
		printk(KERN_ERR "BT/WLAN-ON \n");
		if ( ( ! wlan_power_flg ) && ( ! bt_power_flg ) )
		{
			bcm43xx_gpio_set_value(149, on); /* KO_WLAN_BT_PWR_ONOFF */
			msleep(1);
			bcm43xx_gpio_set_value(150, on); /* KO_BT_WAKEUP 		*/

			bcm43xx_gpio_set_value(43, on); /* KO_BT_PWON    		*/
		}

		if ( is_wlan )
		{
			wlan_power_flg = 1;
		}
		else
		{
			bt_power_flg = 1;
		}
	}
	else
	{
		printk(KERN_ERR "BT/WLAN-OFF \n");
		if ( ( wlan_power_flg ) || ( bt_power_flg ) )
		{
			if ( is_wlan )
			{
				wlan_power_flg = 0;
			}
			else
			{
				bt_power_flg = 0;
			}
			if ( ( ! wlan_power_flg ) && ( ! bt_power_flg ) )
			{
				bcm43xx_gpio_set_value(43, on); /* KO_BT_PWON    		*/
				bcm43xx_gpio_set_value(150, on); /* KO_BT_WAKEUP 		*/
				bcm43xx_gpio_set_value(149, on); /* KO_WLAN_BT_PWR_ONOFF */
				msleep(1);
			}
		}
	}
	
	printk(KERN_ERR "%s S\n", __func__);
	
	return;
}

/* Function List */
int wlan_power(int on);
int wlan_set_carddetect(void);

/* Platform Data (WLAN) */
static struct wifi_platform_data wlan_wifi_control =
{
	.set_power			=	wlan_power,
	.set_reset			=	NULL,
	.set_carddetect			=	NULL,
	.mem_prealloc			=	NULL,
};

/* Resource (WLAN) */
static struct resource wlan_resources[] =
{
	[0] =
	{
		.name			=	"bcm4329_wlan_irq",
		.start			=	MSM_GPIO_TO_INT(145),
		.end				=	MSM_GPIO_TO_INT(145),		
		.flags			=	IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

/* Platform Device (WLAN) */
static struct platform_device wlan_device =
{
	.name				=	"bcm4329_wlan",
	.num_resources		=	ARRAY_SIZE(wlan_resources),
	.resource				=	wlan_resources,
	.dev					=
	{
		.platform_data	=	&wlan_wifi_control,
	},
};

//Configure PMIC GPIO 31 for bcm4330 WLAN_HOSTWAKEUP.
/*  PMIC_GPIO_WLAN_HOSTWAKEUP   */
struct pm8xxx_gpio_init_info wlan_hostweakup = {
	PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_WLAN_HOSTWAKEUP),
		{
		 .direction      		= PM_GPIO_DIR_IN,
		 .pull           		= PM_GPIO_PULL_DN,
		 .vin_sel        		= PM8058_GPIO_VIN_S3,
		 .function      		= PM_GPIO_FUNC_NORMAL,
		 .inv_int_pol    	= 0,
		},
};

//#endif

int wlan_power(int on)
{
	int rc,pin;

	//	printk(KERN_DEBUG "%s S\n", __func__);
	printk(KERN_ERR "%s S\n", __func__);

	if (on) {
		// GPIO TLMM CONFIG

		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = bcm43xx_gpio_tlmm_config(bt_config_power_on[pin],GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: Err gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_on[pin], rc);
				return -EIO;
			}
		}

		printk(KERN_ERR "%s set_power_wlan_bt(1,1) CALL\n", __func__);
		set_power_wlan_bt(1,1);
		printk(KERN_ERR "%s set_power_wlan_bt(1,1) CALL E\n", __func__);
		
		bcm43xx_gpio_set_value(39, on); /* KO_WLAN_PWON  */
//		msleep(120);  // Fujitsu 2011.07.13 over 118ms wait WLAN
		msleep(160);  // Fujitsu 2011.08.31 over 150ms wait WLAN
	} else {
		// GPIO TLMM CONFIG

		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = bcm43xx_gpio_tlmm_config(bt_config_power_off[pin],GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_off[pin], rc);
				return -EIO;
			}
		}

		// GPIO SET VALUE
		printk(KERN_ERR "%s wlan_set_carddetect CALL\n", __func__);
		wlan_set_carddetect();
		printk(KERN_ERR "%s wlan_set_carddetect CALL E\n", __func__);
		msleep(130);
		// Fujitsu 2011.09.13 BCM4330 (KO_WLAN_PWON)WLAN = OFF Flag start
		// BCM4329 WLAN = OFF()
		bcm43xx_gpio_set_value(39, on); /* KO_WLAN_PWON  */
		// Fujitsu 2011.09.13 BCM4330 WLAN = OFF Flag start
		msleep(110);
		
		printk(KERN_ERR "%s set_power_wlan_bt(0,1) CALL\n", __func__);
		set_power_wlan_bt(0,1);
		printk(KERN_ERR "%s set_power_wlan_bt(0,1) CALL\n", __func__);
	}
	
//	printk(KERN_DEBUG "%s E\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(wlan_power);

void mmc_detect_change_for_bcm4329(unsigned long);

/* Function (set carddetect) */

int wlan_set_carddetect(void)
{
//	mmc_detect_change_for_bcm4329(msecs_to_jiffies(8000));

	return 0;
}

static struct platform_device msm_wlan_power_device = {
	.name = "wlan_power",
};

static void __init wlan_power_init(void)
{
	int pin,rc;
	// GPIO TLMM CONFIG

	for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
		rc = bcm43xx_gpio_tlmm_config(bt_config_power_on[pin],GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
			       "%s: Err gpio_tlmm_config(%#x)=%d\n",
			       __func__, bt_config_power_on[pin], rc);
			return ;
		}
	}

	// GPIO REQUEST
	rc = bcm43xx_gpio_request(149, "KO_WLAN_BT_PWR_ONOFF");// Fujitsu 2011.07.29 WLAN/BT change
	if (rc) 
		printk(KERN_ERR "Err gpio_request KO_WLAN_BT_PWR_ONOFF\n");
	
	rc = bcm43xx_gpio_request(39, "KO_WLAN_PWON");
	if (rc) 
		printk(KERN_ERR "Err gpio_request KO_WLAN_PWON\n");
	

	bcm43xx_gpio_direction_output(149, 0);		// WLAN_BT_PWR_ONOFF(OUT,Low)
	bcm43xx_gpio_direction_output(39, 0);		// WLAN_PWON(OUT,Low)

	if ( qci_read_hw_id() == 1 )  { // A build
		bcm43xx_gpio_direction_input(145);			// WLAN_HOSTWAKEUP(IN)
		rc = bcm43xx_gpio_tlmm_config(GPIO_CFG(145,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
			       "%s: Err gpio_tlmm_config(%#x)=%d\n",
			       __func__, GPIO_CFG(145,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), rc);
			return ;
		}
		
	} else {

		//PMIC_GPIO_WLAN_HOSTWAKEUP, need to modify
		bcm43xx_gpio_direction_input( PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_WLAN_HOSTWAKEUP) );			// WLAN_HOSTWAKEUP(IN)

		wlan_resources[0].start = gpio_to_irq(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_WLAN_HOSTWAKEUP));
		wlan_resources[0].end  = gpio_to_irq(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_WLAN_HOSTWAKEUP));
	}
	msm_wlan_power_device.dev.platform_data = &wlan_power;
}

struct pm8xxx_gpio_init_info bt_txd_en_21 = {
	PM8058_GPIO_PM_TO_SYS(KO_PMICUART1_BT_TXD_21),
	{
		 .direction      = PM_GPIO_DIR_OUT,
		 .pull           = PM_GPIO_PULL_NO,
		 .vin_sel        = PM8058_GPIO_VIN_S3,
		 .function       = PM_GPIO_FUNC_2,
		 .inv_int_pol    = 0,
		 .out_strength   = PM_GPIO_STRENGTH_MED,
		 .output_value   = 0,
	}
};

struct pm8xxx_gpio_init_info bt_rxd_en_33 = {
	PM8058_GPIO_PM_TO_SYS(KO_PMICUART1_BT_RXD_33),
	{
		 .direction      = PM_GPIO_DIR_IN,
		 .pull           = PM_GPIO_PULL_NO,
		 .vin_sel        = PM8058_GPIO_VIN_S3,
		 .function       = PM_GPIO_FUNC_1,
		 .inv_int_pol    = 0,
		 .out_strength   = PM_GPIO_STRENGTH_MED,
		 .output_value   = 0,
	}
};

struct pm8xxx_gpio_init_info bt_pmic_txd_en_36 = {
	PM8058_GPIO_PM_TO_SYS(KO_PMICUART1_BT_PMIC_TXD_36),
	{
		 .direction      = PM_GPIO_DIR_IN,
		 .pull           = PM_GPIO_PULL_NO,
		 .vin_sel        = PM8058_GPIO_VIN_S3,
		 .function       = PM_GPIO_FUNC_1,
		 .inv_int_pol    = 0,
		 .out_strength   = PM_GPIO_STRENGTH_MED,
		 .output_value   = 0,
	}
};

struct pm8xxx_gpio_init_info bt_pmic_rxd_en_37 = {
	PM8058_GPIO_PM_TO_SYS(KO_PMICUART1_BT_PMIC_RXD_37),
	{
		 .direction      = PM_GPIO_DIR_OUT,
		 .pull           = PM_GPIO_PULL_NO,
		 .vin_sel        = PM8058_GPIO_VIN_S3,
		 .function       = PM_GPIO_FUNC_2,
		 .inv_int_pol    = 0,
		 .out_strength   = PM_GPIO_STRENGTH_MED,
		 .output_value   = 0,
	}
};

static int bluetooth_fmt_power(void)
{
	int pin, rc;
	int on;

	if( bcm433x_bt_power || bcm433x_fmt_power){

		on = 1;

	}else{

		on = 0;

	}

	printk("bluetooth_fmt_power = %d\n",on);
	if (on) {

		// PMIC GPIO CONFIG

		// TX
		rc = pm8xxx_gpio_config(bt_txd_en_21.gpio , &bt_txd_en_21.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_TXD config failed\n", __func__);
		return -EIO;
		}

		// RX
		rc = pm8xxx_gpio_config(bt_rxd_en_33.gpio , &bt_rxd_en_33.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_RXD config failed\n", __func__);
			return -EIO;
		}

		rc = pm8058_misc_control(NULL, PM8058_UART_MUX_MASK, PM8058_UART_MUX_1);
		if (rc) {
			printk(KERN_ERR "%s: pm8058_misc_control failed=%d\n",__func__,rc);
		}
		rc = pm8xxx_gpio_config(bt_pmic_txd_en_36.gpio , &bt_pmic_txd_en_36.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_TXD config failed\n", __func__);
			return -EIO;
		}
		rc = pm8xxx_gpio_config(bt_pmic_rxd_en_37.gpio , &bt_pmic_rxd_en_37.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_RXD config failed\n", __func__);
			return -EIO;
		}

		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],GPIO_CFG_ENABLE);
			if (rc) {
					printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_on[pin], rc);
					return -EIO;
				}
			}

		set_power_wlan_bt(1,0);
		gpio_set_value(41, on); /* KO_BT_RESET   		*/

		printk("hw id = %d\n",qci_read_hw_id());
		if ( qci_read_hw_id() == 1 )  {
			;
		}else{
			fm_i2s_gpio_init();
		}

		msleep(110);
		
	} else {
		// GPIO SET VALUE
		gpio_set_value(41, on); /* KO_BT_RESET 			*/
		msleep(110);
		set_power_wlan_bt(0,0); 

		// PMIC GPIO CONFIG
		rc = pm8xxx_gpio_config(bt_txd_en_21.gpio , &bt_txd_en_21.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_TXD_21 config failed\n", __func__);
			return -EIO;
		}
		rc = pm8xxx_gpio_config(bt_rxd_en_33.gpio , &bt_rxd_en_33.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_RXD config failed\n", __func__);
			return -EIO;
		}

		rc = pm8xxx_gpio_config(bt_pmic_txd_en_36.gpio , &bt_pmic_txd_en_36.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_PMIC_TXD config failed\n", __func__);
			return -EIO;
		}
		rc = pm8xxx_gpio_config(bt_pmic_rxd_en_37.gpio , &bt_pmic_rxd_en_37.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_PMIC_RXD config failed\n", __func__);
			return -EIO;
		}

		// GPIO TLMM CONFIG
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = gpio_tlmm_config(bt_config_power_off[pin],GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_off[pin], rc);
				return -EIO;
				}
		}
	}
	return 0;
}

static int fmt_power(int on)
{

	bcm433x_fmt_power = on;
	printk("fmt_power = %d\n",on);

	return bluetooth_fmt_power();

}

static int bluetooth_power(int on)
{

	bcm433x_bt_power = on;
	printk("bluetooth_power = %d\n",on);

	return bluetooth_fmt_power();

}

static void __init fmt_power_init(void)
{
    qci_fmt_power_device.dev.platform_data = &fmt_power;
}

static void __init bt_power_init(void)
{
    msm_bt_power_device.dev.platform_data = &bluetooth_power;
}

static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= 147,
		.end	= 147,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= 150,
		.end	= 150,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= MSM_GPIO_TO_INT(147),
		.end	= MSM_GPIO_TO_INT(147),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

static struct msm_psy_batt_pdata msm_psy_batt_data = {
#if defined(T_QCI_IS3)
       .voltage_min_design 	= 3200,
	.voltage_max_design	= 4200,
#else
	.voltage_min_design 	= 2800,
	.voltage_max_design	= 4300,
#endif
	.avail_chg_sources   	= AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
};

static struct platform_device msm_batt_device = {
	.name 		    = "msm-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

static char *msm_adc_fluid_device_names[] = {
	"LTC_ADC1",
	"LTC_ADC2",
	"LTC_ADC3",
};

static char *msm_adc_surf_device_names[] = {
	"XO_ADC",
};

static struct msm_adc_platform_data msm_adc_pdata;

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};

#ifdef CONFIG_MSM_SDIO_AL
static struct msm_gpio mdm2ap_status = {
	GPIO_CFG(77, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	"mdm2ap_status"
};


static int configure_mdm2ap_status(int on)
{
	if (on)
		return msm_gpios_request_enable(&mdm2ap_status, 1);
	else {
		msm_gpios_disable_free(&mdm2ap_status, 1);
		return 0;
	}
}

static int get_mdm2ap_status(void)
{
	return gpio_get_value(GPIO_PIN(mdm2ap_status.gpio_cfg));
}

static struct sdio_al_platform_data sdio_al_pdata = {
	.config_mdm2ap_status = configure_mdm2ap_status,
	.get_mdm2ap_status = get_mdm2ap_status,
	.allow_sdioc_version_major_2 = 1,
	.peer_sdioc_version_minor = 0x0001,
	.peer_sdioc_version_major = 0x0003,
	.peer_sdioc_boot_version_minor = 0x0001,
	.peer_sdioc_boot_version_major = 0x0003,
};

struct platform_device msm_device_sdio_al = {
	.name = "msm_sdio_al",
	.id = -1,
	.dev		= {
		.platform_data	= &sdio_al_pdata,
	},
};

#endif /* CONFIG_MSM_SDIO_AL */

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart2,
#endif
#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
	&msm_proccomm_regulator_dev,
#endif
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#if defined (CONFIG_SND_MSM_MVS_DAI_SOC)
	&asoc_msm_mvs,
	&asoc_mvs_dai0,
	&asoc_mvs_dai1,
#endif
	&msm_device_smd,
	&msm_device_dmov,
	&smc91x_device,
	&smsc911x_device,
	&msm_device_nand,
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif
#ifdef CONFIG_USB_G_ANDROID
	&android_usb_device,
#endif
	&qsd_device_spi,

#ifdef CONFIG_MSM_SSBI
	&msm_device_ssbi_pmic1,
#endif
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi7,
#endif
	&android_pmem_device,
	&msm_fb_device,
	&msm_migrate_pages_device,
 #if defined(T_QCI_IS3)
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&lcdc_chimei_panel_device,
 #else
	&mddi_toshiba_device,
	&lcdc_toshiba_panel_device,
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&lcdc_sharp_panel_device,
#endif
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&hs_device,

	&qci_device_irda,

#if defined(CONFIG_I2C_GPIO)	
	&is3_i2cGpio_device,	
#endif

#ifdef MSM_UARTDM2_USE
	&msm_device_uart_dm2,
#endif

#ifdef CONFIG_MSM7KV2_AUDIO
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_aux_pcm_device,
#endif
	&msm_device_adspdec,
	&qup_device_i2c,
	&msm_kgsl_3d0,
	&msm_kgsl_2d0,
	&msm_bt_power_device,
	&msm_bluesleep_device,

	&qci_fmt_power_device,

#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
#ifdef CONFIG_MT9D112
	&msm_camera_sensor_mt9d112,
#endif
#ifdef CONFIG_WEBCAM_OV9726
	&msm_camera_sensor_ov9726,
#endif
#ifdef CONFIG_S5K3E2FX
	&msm_camera_sensor_s5k3e2fx,
#endif
#ifdef CONFIG_MT9P012
	&msm_camera_sensor_mt9p012,
#endif
#ifdef CONFIG_MT9E013
	&msm_camera_sensor_mt9e013,
#endif
#ifdef CONFIG_VX6953
	&msm_camera_sensor_vx6953,
#endif
#ifdef CONFIG_SN12M0PZ
	&msm_camera_sensor_sn12m0pz,
#endif
#if defined(CONFIG_OV5640_IS3)
	&msm_camera_sensor_ov5640,
#endif
	&msm_device_vidc_720p,
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
#ifdef CONFIG_MSM_SDIO_AL
	&msm_device_sdio_al,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif
//BCMM4330 WIFI/BT, Porting from Fujitsu
#ifdef CONFIG_WIFI_CONTROL_FUNC
	&wlan_device,
#endif

	&ram_console_device,

	&msm_batt_device,
	&msm_adc_device,
	&msm_ebi0_thermal,
	&msm_ebi1_thermal,
    &memread_dev,
	&fj_walkmotion_device
};

#if defined (T_QCI_IS3)
static struct platform_device *devices_oc[] __initdata = {
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart2,
#endif
#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
	&msm_proccomm_regulator_dev,
#endif
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#if defined (CONFIG_SND_MSM_MVS_DAI_SOC)
	&asoc_msm_mvs,
	&asoc_mvs_dai0,
	&asoc_mvs_dai1,
#endif
	&msm_device_smd,
	&msm_device_dmov,
	&smc91x_device,
	&smsc911x_device,
	&msm_device_nand,
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif
#ifdef CONFIG_USB_G_ANDROID
	&android_usb_device,
#endif
	&qsd_device_spi,

#ifdef CONFIG_MSM_SSBI
	&msm_device_ssbi_pmic1,
#endif
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi7,
#endif
	&android_pmem_device,
	&msm_fb_device,
	&msm_migrate_pages_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&hs_device,
	&msm_device_adspdec,
	&qup_device_i2c,
	&msm_kgsl_3d0,
	&msm_kgsl_2d0,
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
#ifdef CONFIG_MSM_SDIO_AL
	&msm_device_sdio_al,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif
	&msm_batt_device,
	&msm_adc_device,
	&msm_ebi0_thermal,
	&msm_ebi1_thermal,
	&memread_dev,
};

static uint32_t gpio_config_offline_charging[] = {
       GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* RST */
       GPIO_CFG(1,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(2,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
       GPIO_CFG(16, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
       GPIO_CFG(17, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
       GPIO_CFG(27, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
       GPIO_CFG(31, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
       GPIO_CFG(33, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(39,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_PWON         */
	GPIO_CFG(41,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_RESET          */
	GPIO_CFG(43,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PWON           */
	GPIO_CFG(44, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(45, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(47, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(48, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(49, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(50, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(51, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(52, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(58,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), //sdc4_clk
	GPIO_CFG(59,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), //sdc4_cmd
	GPIO_CFG(60,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), //sdc4_dat_3
	GPIO_CFG(61,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), //sdc4_dat_2
	GPIO_CFG(62,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), //sdc4_dat_1
	GPIO_CFG(63,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), //sdc4_dat_0
	GPIO_CFG(88, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(110,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), //sdc3_clk
	GPIO_CFG(111,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), //sdc3_cmd
	GPIO_CFG(112, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(113, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(114, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(115, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(116,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), //sdc3_dat_3
	GPIO_CFG(117,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), //sdc3_dat_2
	GPIO_CFG(118,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), //sdc3_dat_1
	GPIO_CFG(119,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), //sdc3_dat_0
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCMOUT         */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCMIN          */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCM_FSYNC      */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCM_DCLK       */
       GPIO_CFG(143, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(147, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_HOSTWAKEUP     */
	GPIO_CFG(148, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(149,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_BT_PWR_ONOFF */
       GPIO_CFG(150,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_WAKEUP         */
       GPIO_CFG(172, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
       GPIO_CFG(173, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
       GPIO_CFG(174, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
       GPIO_CFG(175, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
       GPIO_CFG(176, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
       GPIO_CFG(177, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
       GPIO_CFG(178, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
       GPIO_CFG(180, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
#endif

static struct msm_gpio msm_i2c_gpios_hw[] = {
	{ GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "i2c_scl" },
	{ GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "i2c_sda" },
};

static struct msm_gpio msm_i2c_gpios_io[] = {
	{ GPIO_CFG(70, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "i2c_scl" },
	{ GPIO_CFG(71, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "i2c_sda" },
};
static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_scl" },
	{ GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_sda" },
};
static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_scl" },
	{ GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_sda" },
};

static void
msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;

	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id*2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id*2];
	msm_gpios_enable(msm_i2c_table, 2);
}
/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
static struct regulator *qup_vreg;
#endif
static void
qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	if (!IS_ERR_OR_NULL(qup_vreg)) {
		rc = regulator_enable(qup_vreg);
		if (rc) {
			pr_err("%s: regulator_enable failed: %d\n",
			__func__, rc);
		}
	}
#endif
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 320000,
	.pri_clk = 70,
	.pri_dat = 71,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000021",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

#if defined(T_QCI_IS3)
static void pm_control_voltage_offline_charging(void)
{
	int rc=0;
	struct vreg *vreg_off,*vreg_l8_1p8;
	int disable_vreg_total = 2;
	char *vreg_name[] = {
		"gp4",	// 1
		"gp10",	// 2
	};
	int i;

	printk(KERN_ERR"%s\n",__func__);

//enable l8 for summit ic and gauge ic
	vreg_l8_1p8 = vreg_get(NULL, "gp7");
	if (!vreg_l8_1p8) {
		pr_err("%s: VREG L8 get failed\n", __func__);
	}
	else
	{
	   rc = vreg_set_level(vreg_l8_1p8, 1800);
	   if (rc) {
		pr_err("%s: VREG L8 set failed\n", __func__);
	   }
	   else
	   {
	       rc = vreg_enable(vreg_l8_1p8);
	       if (rc) {
		    pr_err("%s: VREG L8 enable failed\n", __func__);
	       }
		else{
		    pr_err("%s: VREG L8 enable pass\n", __func__);
		}
	   }
	}

//disable not use vreg
	for (i=0;i<disable_vreg_total;i++)
	{
	     vreg_off = vreg_get(NULL, vreg_name[i]);
	     if (!vreg_off) {
		  pr_err("%s: VREG %s get failed\n", __func__,vreg_name[i]);
	     }
            else
            {
                rc = vreg_disable(vreg_off);
	         if (rc) {
		       pr_err("%s: VREG %s disable failed\n", __func__,vreg_name[i]);
	         }
		  else{
		       pr_err("%s: VREG %s disable pass\n", __func__,vreg_name[i]);
		  }
            }
	}

//ov5640_power_off
	gpio_set_value(157, 1);

//wlen power off
	bcm43xx_gpio_set_value(39, 0); /* KO_WLAN_PWON  */
	set_power_wlan_bt(0,1);

// msm_snddev_poweramp_off
	gpio_set_value(148,0);

// bt power off
       bcm433x_bt_power = 0;
       bluetooth_fmt_power();

//set gpio config
       config_gpio_table(gpio_config_offline_charging,ARRAY_SIZE(gpio_config_offline_charging));
}
#endif

static void i2c_pull_up_voltage(void)
{
	int rc=0;
	struct vreg *vreg_l8_1p8, *vreg_gp4;

	printk(KERN_ERR"%s\n",__func__);

        vreg_gp4 = vreg_get(NULL, "gp4");
	if (!vreg_gp4) {
		pr_err("%s: VREG L10 get failed\n", __func__);
	}

	rc = vreg_set_level(vreg_gp4, 2850);
	if (rc) {
		pr_err("%s: VREG L10 set failed\n", __func__);
	}

	rc = vreg_enable(vreg_gp4);
	if (rc) {
		pr_err("%s: VREG L10 enable failed\n", __func__);
	}
	udelay(100);

	vreg_l8_1p8 = vreg_get(NULL, "gp7");
	if (!vreg_l8_1p8) {
		pr_err("%s: VREG L8 get failed\n", __func__);
		//return rc;
	}

	rc = vreg_set_level(vreg_l8_1p8, 1800);
	if (rc) {
		pr_err("%s: VREG L8 set failed\n", __func__);
		//goto l8_put;
	}

	rc = vreg_enable(vreg_l8_1p8);
	if (rc) {
		pr_err("%s: VREG L8 enable failed\n", __func__);
		//goto l8_put;
	}

	vreg_l16_3p0 = vreg_get(NULL, "gp10");
	if (!vreg_l16_3p0) {
		pr_err("%s: VREG L16 get failed\n", __func__);
	}

	rc = vreg_set_level(vreg_l16_3p0, 3000);
	if (rc) {
		pr_err("%s: VREG L16 set failed\n", __func__);
	}

	rc = vreg_enable(vreg_l16_3p0);
	if (rc) {
		pr_err("%s: VREG L16 enable failed\n", __func__);
	}

	udelay(100);


}
#if defined(CONFIG_I2C_GPIO)	
static void gpio_i2c_pull_up_voltage(void)
{
#if 0
/*Don't enable 1.8V in here that avoid power consumption 20120417*/
	printk(KERN_ERR"%s\n",__func__);
	gpio_tlmm_config( GPIO_CFG(2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_tlmm_config( GPIO_CFG(3, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	
	gpio_tlmm_config( GPIO_CFG(121, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_set_value(121,1);
	udelay(100);
#endif
}
#endif
static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, ARRAY_SIZE(msm_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 100000,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000022",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 384000,
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	if (msm_gpios_request(qup_i2c_gpios_hw, ARRAY_SIZE(qup_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	qup_vreg = regulator_get(&qup_device_i2c.dev, "lvsw1");
	if (IS_ERR(qup_vreg)) {
		dev_err(&qup_device_i2c.dev,
			"%s: regulator_get failed: %ld\n",
			__func__, PTR_ERR(qup_vreg));
	}
#endif
}

#ifdef CONFIG_I2C_SSBI
static struct msm_i2c_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif

static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}

static struct msm_gpio msm_nand_ebi2_cfg_data[] = {
	{GPIO_CFG(86, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_cs1"},
	{GPIO_CFG(115, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_busy1"},
};

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};
#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT)
static struct msm_gpio sdc1_lvlshft_cfg_data[] = {
	{GPIO_CFG(35, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), "sdc1_lvlshft"},
};
#endif
static struct msm_gpio sdc1_cfg_data[] = {
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(64, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc2_clk"},
	{GPIO_CFG(65, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_cmd"},
	{GPIO_CFG(66, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_3"},
	{GPIO_CFG(67, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_2"},
	{GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_1"},
	{GPIO_CFG(69, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_0"},

#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{GPIO_CFG(115, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_4"},
	{GPIO_CFG(114, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_5"},
	{GPIO_CFG(113, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_6"},
	{GPIO_CFG(112, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_7"},
#endif
};

static struct msm_gpio sdc3_cfg_data[] = {
#if 0
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_0"},
#endif
/* FUJITSU:2011-07-08 change WLAN start */
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc3_clk"},// KO_WLAN_SD_CLK
/* FUJITSU:2011-09-01 change WLAN start */
//	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc3_cmd"},// KO_WLAN_SD_CMD
//	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_3"},// KO_WLAN_SD_DAT3
//	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_2"},// KO_WLAN_SD_DAT2
//	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_1"},// KO_WLAN_SD_DAT1
//	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_0"},// KO_WLAN_SD_DAT0
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},// KO_WLAN_SD_CMD
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_3"},// KO_WLAN_SD_DAT3
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_2"},// KO_WLAN_SD_DAT2
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_1"},// KO_WLAN_SD_DAT1
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_0"},// KO_WLAN_SD_DAT0
};

static struct msm_gpio sdc3_sleep_cfg_data[] = {
#if 0
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_clk"},
	{GPIO_CFG(111, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_cmd"},
	{GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_3"},
	{GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_2"},
	{GPIO_CFG(118, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_1"},
	{GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_0"},
#endif
/* FUJITSU:2011-07-08 change WLAN start */
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc3_clk"},// KO_WLAN_SD_CLK
/* FUJITSU:2011-09-01 change WLAN start */
//	{GPIO_CFG(111, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc3_cmd"},// KO_WLAN_SD_CMD
//	{GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_3"},// KO_WLAN_SD_DAT3
//	{GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_2"},// KO_WLAN_SD_DAT2
//	{GPIO_CFG(118, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_1"},// KO_WLAN_SD_DAT1
//	{GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_0"},// KO_WLAN_SD_DAT0
	{GPIO_CFG(111, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},// KO_WLAN_SD_CMD
	{GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_3"},// KO_WLAN_SD_DAT3
	{GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_2"},// KO_WLAN_SD_DAT2
	{GPIO_CFG(118, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_1"},// KO_WLAN_SD_DAT1
	{GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_0"},// KO_WLAN_SD_DAT0
/* FUJITSU:2011-09-01 change WLAN end */
/* FUJITSU:2011-07-08 change WLAN end */
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_0"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
		.sleep_cfg_data = sdc3_sleep_cfg_data,
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
		.sleep_cfg_data = NULL,
	},
};

static struct regulator *sdcc_vreg_data[ARRAY_SIZE(sdcc_cfg_data)];

#if !defined(T_QCI_IS3)
static unsigned long vreg_sts, gpio_sts;
#endif

static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
		} else {
			msm_gpios_disable_free(curr->cfg_data, curr->size);
		}
	}

	return rc;
}

static uint32_t msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct regulator *curr = sdcc_vreg_data[dev_id - 1];
	static int enabled_once[] = {0, 0, 0, 0};

	if (test_bit(dev_id, &vreg_sts) == enable)
		return rc;

	if (dev_id == 4) {
		if (enable) {
			pr_debug("Enable Vdd dev_%d\n", dev_id);
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_PWR_EN_N),
#if defined(T_QCI_IS3)
						1);
#else
						0);
#endif
			set_bit(dev_id, &vreg_sts);
		} else {
			pr_debug("Disable Vdd dev_%d\n", dev_id);
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_PWR_EN_N),
#if defined(T_QCI_IS3)
				0);
#else
				1);
#endif
			clear_bit(dev_id, &vreg_sts);
		}
	}

	if (!enable || enabled_once[dev_id - 1])
			return 0;
	if (!curr)
		return -ENODEV;

	if (IS_ERR(curr))
		return PTR_ERR(curr);

	if (enable) {
		set_bit(dev_id, &vreg_sts);

		rc = regulator_enable(curr);
		if (rc)
			pr_err("%s: could not enable regulator: %d\n",
					__func__, rc);
		enabled_once[dev_id - 1] = 1;
	} else {
		clear_bit(dev_id, &vreg_sts);

		rc = regulator_disable(curr);
		if (rc)
			pr_err("%s: could not disable regulator: %d\n",
					__func__, rc);
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
	if (rc)
		goto out;

	if (pdev->id == 4) /* S3 is always ON and cannot be disabled */
		rc = msm_sdcc_setup_vreg(pdev->id, (vdd ? 1 : 0));
out:
	return rc;
}

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT) && \
	defined(CONFIG_CSDIO_VENDOR_ID) && \
	defined(CONFIG_CSDIO_DEVICE_ID) && \
	(CONFIG_CSDIO_VENDOR_ID == 0x70 && CONFIG_CSDIO_DEVICE_ID == 0x1117)

#define MBP_ON  1
#define MBP_OFF 0

#define MBP_RESET_N \
	GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA)
#define MBP_INT0 \
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA)

#define MBP_MODE_CTRL_0 \
	GPIO_CFG(35, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define MBP_MODE_CTRL_1 \
	GPIO_CFG(36, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define MBP_MODE_CTRL_2 \
	GPIO_CFG(34, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define TSIF_EN \
	GPIO_CFG(35, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,	GPIO_CFG_2MA)
#define TSIF_DATA \
	GPIO_CFG(36, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,	GPIO_CFG_2MA)
#define TSIF_CLK \
	GPIO_CFG(34, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static struct msm_gpio mbp_cfg_data[] = {
	{GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		"mbp_reset"},
	{GPIO_CFG(85, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		"mbp_io_voltage"},
};

static int mbp_config_gpios_pre_init(int enable)
{
	int rc = 0;

	if (enable) {
		rc = msm_gpios_request_enable(mbp_cfg_data,
			ARRAY_SIZE(mbp_cfg_data));
		if (rc) {
			printk(KERN_ERR
				"%s: Failed to turnon GPIOs for mbp chip(%d)\n",
				__func__, rc);
		}
	} else
		msm_gpios_disable_free(mbp_cfg_data, ARRAY_SIZE(mbp_cfg_data));
	return rc;
}

static struct regulator_bulk_data mbp_regs_io[2];
static struct regulator_bulk_data mbp_regs_rf[2];
static struct regulator_bulk_data mbp_regs_adc[1];
static struct regulator_bulk_data mbp_regs_core[1];

static int mbp_init_regs(struct device *dev)
{
	struct regulator_bulk_data regs[] = {
		/* Analog and I/O regs */
		{ .supply = "gp4",  .min_uV = 2600000, .max_uV = 2600000 },
		{ .supply = "s3",   .min_uV = 1800000, .max_uV = 1800000 },
		/* RF regs */
		{ .supply = "s2",   .min_uV = 1300000, .max_uV = 1300000 },
		{ .supply = "rf",   .min_uV = 2600000, .max_uV = 2600000 },
		/* ADC regs */
		{ .supply = "s4",   .min_uV = 2200000, .max_uV = 2200000 },
		/* Core regs */
		{ .supply = "gp16", .min_uV = 1200000, .max_uV = 1200000 },
	};

	struct regulator_bulk_data *regptr = regs;
	int rc;

	rc = regulator_bulk_get(dev, ARRAY_SIZE(regs), regs);

	if (rc) {
		dev_err(dev, "%s: could not get regulators: %d\n",
				__func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs), regs);

	if (rc) {
		dev_err(dev, "%s: could not set voltages: %d\n",
				__func__, rc);
		goto reg_free;
	}

	memcpy(mbp_regs_io, regptr, sizeof(mbp_regs_io));
	regptr += ARRAY_SIZE(mbp_regs_io);

	memcpy(mbp_regs_rf, regptr, sizeof(mbp_regs_rf));
	regptr += ARRAY_SIZE(mbp_regs_rf);

	memcpy(mbp_regs_adc, regptr, sizeof(mbp_regs_adc));
	regptr += ARRAY_SIZE(mbp_regs_adc);

	memcpy(mbp_regs_core, regptr, sizeof(mbp_regs_core));

	return 0;

reg_free:
	regulator_bulk_free(ARRAY_SIZE(regs), regs);
out:
	return rc;
}

static int mbp_setup_rf_vregs(int state)
{
	return state ?
		regulator_bulk_enable(ARRAY_SIZE(mbp_regs_rf), mbp_regs_rf) :
		regulator_bulk_disable(ARRAY_SIZE(mbp_regs_rf), mbp_regs_rf);
}

static int mbp_setup_vregs(int state)
{
	return state ?
		regulator_bulk_enable(ARRAY_SIZE(mbp_regs_io), mbp_regs_io) :
		regulator_bulk_disable(ARRAY_SIZE(mbp_regs_io), mbp_regs_io);
}

static int mbp_set_tcxo_en(int enable)
{
	int rc;
	const char *id = "UBMC";
	struct vreg *vreg_analog = NULL;

	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A1,
		enable ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0) {
		printk(KERN_ERR "%s: unable to %svote for a1 clk\n",
			__func__, enable ? "" : "de-");
		return -EIO;
	}
	return rc;
}

static void mbp_set_freeze_io(int state)
{
	if (state)
		gpio_set_value(85, 0);
	else
		gpio_set_value(85, 1);
}

static int mbp_set_core_voltage_en(int enable)
{
	static bool is_enabled;
	int rc = 0;

	if (enable && !is_enabled) {
		rc = regulator_bulk_enable(ARRAY_SIZE(mbp_regs_core),
				mbp_regs_core);
		if (rc) {
			pr_err("%s: could not enable regulators: %d\n",
					__func__, rc);
		} else {
			is_enabled = true;
		}
	}

	return rc;
}

static void mbp_set_reset(int state)
{
	if (state)
		gpio_set_value(GPIO_PIN(MBP_RESET_N), 0);
	else
		gpio_set_value(GPIO_PIN(MBP_RESET_N), 1);
}

static int mbp_config_interface_mode(int state)
{
	if (state) {
		gpio_tlmm_config(MBP_MODE_CTRL_0, GPIO_CFG_ENABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_1, GPIO_CFG_ENABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_2, GPIO_CFG_ENABLE);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_0), 0);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_1), 1);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_2), 0);
	} else {
		gpio_tlmm_config(MBP_MODE_CTRL_0, GPIO_CFG_DISABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_1, GPIO_CFG_DISABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_2, GPIO_CFG_DISABLE);
	}
	return 0;
}

static int mbp_setup_adc_vregs(int state)
{
	return state ?
		regulator_bulk_enable(ARRAY_SIZE(mbp_regs_adc), mbp_regs_adc) :
		regulator_bulk_disable(ARRAY_SIZE(mbp_regs_adc), mbp_regs_adc);
}

static int mbp_power_up(void)
{
	int rc;

	rc = mbp_config_gpios_pre_init(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_config_gpios_pre_init() done\n", __func__);

	rc = mbp_setup_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: gp4 (2.6) and s3 (1.8) done\n", __func__);

	rc = mbp_set_tcxo_en(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: tcxo clock done\n", __func__);

	mbp_set_freeze_io(MBP_OFF);
	pr_debug("%s: set gpio 85 to 1 done\n", __func__);

	udelay(100);
	mbp_set_reset(MBP_ON);

	udelay(300);
	rc = mbp_config_interface_mode(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_config_interface_mode() done\n", __func__);

	udelay(100 + mbp_set_core_voltage_en(MBP_ON));
	pr_debug("%s: power gp16 1.2V done\n", __func__);

	mbp_set_freeze_io(MBP_ON);
	pr_debug("%s: set gpio 85 to 0 done\n", __func__);

	udelay(100);

	rc = mbp_setup_rf_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: s2 1.3V and rf 2.6V done\n", __func__);

	rc = mbp_setup_adc_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: s4 2.2V  done\n", __func__);

	udelay(200);

	mbp_set_reset(MBP_OFF);
	pr_debug("%s: close gpio 44 done\n", __func__);

	msleep(20);
exit:
	return rc;
}

static int mbp_power_down(void)
{
	int rc;

	mbp_set_reset(MBP_ON);
	pr_debug("%s: mbp_set_reset(MBP_ON) done\n", __func__);

	udelay(100);

	rc = mbp_setup_adc_vregs(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: vreg_disable(vreg_adc) done\n", __func__);

	udelay(5);

	rc = mbp_setup_rf_vregs(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_setup_rf_vregs(MBP_OFF) done\n", __func__);

	udelay(5);

	mbp_set_freeze_io(MBP_OFF);
	pr_debug("%s: mbp_set_freeze_io(MBP_OFF) done\n", __func__);

	udelay(100);
	rc = mbp_set_core_voltage_en(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_set_core_voltage_en(MBP_OFF) done\n", __func__);

	rc = mbp_set_tcxo_en(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_set_tcxo_en(MBP_OFF) done\n", __func__);

	rc = mbp_setup_vregs(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_setup_vregs(MBP_OFF) done\n", __func__);

	rc = mbp_config_gpios_pre_init(MBP_OFF);
	if (rc)
		goto exit;
exit:
	return rc;
}

static void (*mbp_status_notify_cb)(int card_present, void *dev_id);
static void *mbp_status_notify_cb_devid;
static int mbp_power_status;
static int mbp_power_init_done;

static uint32_t mbp_setup_power(struct device *dv,
	unsigned int power_status)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	if (power_status == mbp_power_status)
		goto exit;
	if (power_status) {
		pr_debug("turn on power of mbp slot");
		rc = mbp_power_up();
		mbp_power_status = 1;
	} else {
		pr_debug("turn off power of mbp slot");
		rc = mbp_power_down();
		mbp_power_status = 0;
	}
exit:
	return rc;
};

int mbp_register_status_notify(void (*callback)(int, void *),
	void *dev_id)
{
	mbp_status_notify_cb = callback;
	mbp_status_notify_cb_devid = dev_id;
	return 0;
}

static unsigned int mbp_status(struct device *dev)
{
	return mbp_power_status;
}

static uint32_t msm_sdcc_setup_power_mbp(struct device *dv, unsigned int vdd)
{
	struct platform_device *pdev;
	uint32_t rc = 0;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_power(dv, vdd);
	if (rc) {
		pr_err("%s: Failed to setup power (%d)\n",
			__func__, rc);
		goto out;
	}
	if (!mbp_power_init_done) {
		rc = mbp_init_regs(dv);
		if (rc) {
			dev_err(dv, "%s: regulator init failed: %d\n",
					__func__, rc);
			goto out;
		}
		mbp_setup_power(dv, 1);
		mbp_setup_power(dv, 0);
		mbp_power_init_done = 1;
	}
	if (vdd >= 0x8000) {
		rc = mbp_setup_power(dv, (0x8000 == vdd) ? 0 : 1);
		if (rc) {
			pr_err("%s: Failed to config mbp chip power (%d)\n",
				__func__, rc);
			goto out;
		}
		if (mbp_status_notify_cb) {
			mbp_status_notify_cb(mbp_power_status,
				mbp_status_notify_cb_devid);
		}
	}
out:
	/* should return 0 only */
	return 0;
}

#endif

#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT

static int msm_sdcc_get_wpswitch(struct device *dv)
{
	void __iomem *wp_addr = 0;
	uint32_t ret = 0;
	struct platform_device *pdev;

	if (!(machine_is_msm7x30_surf()))
		return -1;
	pdev = container_of(dv, struct platform_device, dev);

	wp_addr = ioremap(FPGA_SDCC_STATUS, 4);
	if (!wp_addr) {
		pr_err("%s: Could not remap %x\n", __func__, FPGA_SDCC_STATUS);
		return -ENOMEM;
	}

	ret = (((readl(wp_addr) >> 4) >> (pdev->id-1)) & 0x01);
	pr_info("%s: WP Status for Slot %d = 0x%x \n", __func__,
							pdev->id, ret);
	iounmap(wp_addr);

	return ret;
}
#endif

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT)
#if defined(CONFIG_CSDIO_VENDOR_ID) && \
	defined(CONFIG_CSDIO_DEVICE_ID) && \
	(CONFIG_CSDIO_VENDOR_ID == 0x70 && CONFIG_CSDIO_DEVICE_ID == 0x1117)
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power_mbp,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.status	        = mbp_status,
	.register_status_notify = mbp_register_status_notify,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 24576000,
	.nonremovable	= 0,
};
#else
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask	= MMC_VDD_165_195,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data msm7x30_sdc2_data = {
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm7x30_sdc3_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	.sdiowakeup_irq = MSM_GPIO_TO_INT(118),
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.wpswitch    = msm_sdcc_get_wpswitch,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static int msm_sdc1_lvlshft_enable(void)
{
	static struct regulator *ldo5;
	int rc;

	/* Enable LDO5, an input to the FET that powers slot 1 */

	ldo5 = regulator_get(NULL, "ldo5");

	if (IS_ERR(ldo5)) {
		rc = PTR_ERR(ldo5);
		pr_err("%s: could not get ldo5: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_set_voltage(ldo5, 2850000, 2850000);
	if (rc) {
		pr_err("%s: could not set ldo5 voltage: %d\n", __func__, rc);
		goto ldo5_free;
	}

	rc = regulator_enable(ldo5);
	if (rc) {
		pr_err("%s: could not enable ldo5: %d\n", __func__, rc);
		goto ldo5_free;
	}

	/* Enable GPIO 35, to turn on the FET that powers slot 1 */
	rc = msm_gpios_request_enable(sdc1_lvlshft_cfg_data,
				ARRAY_SIZE(sdc1_lvlshft_cfg_data));
	if (rc)
		printk(KERN_ERR "%s: Failed to enable GPIO 35\n", __func__);

	rc = gpio_direction_output(GPIO_PIN(sdc1_lvlshft_cfg_data[0].gpio_cfg),
				1);
	if (rc)
		printk(KERN_ERR "%s: Failed to turn on GPIO 35\n", __func__);

	return 0;

ldo5_free:
	regulator_put(ldo5);
out:
	ldo5 = NULL;
	return rc;
}
#endif

static int mmc_regulator_init(int sdcc_no, const char *supply, int uV)
{
	int rc;

	BUG_ON(sdcc_no < 1 || sdcc_no > 4);

	sdcc_no--;

	sdcc_vreg_data[sdcc_no] = regulator_get(NULL, supply);

	if (IS_ERR(sdcc_vreg_data[sdcc_no])) {
		rc = PTR_ERR(sdcc_vreg_data[sdcc_no]);
		pr_err("%s: could not get regulator \"%s\": %d\n",
				__func__, supply, rc);
		goto out;
	}

	rc = regulator_set_voltage(sdcc_vreg_data[sdcc_no], uV, uV);

	if (rc) {
		pr_err("%s: could not set voltage for \"%s\" to %d uV: %d\n",
				__func__, supply, uV, rc);
		goto reg_free;
	}

	return rc;

reg_free:
	regulator_put(sdcc_vreg_data[sdcc_no]);
out:
	sdcc_vreg_data[sdcc_no] = NULL;
	return rc;
}

static void __init msm7x30_init_mmc(void)
{
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (mmc_regulator_init(1, "s3", 1800000))
		goto out1;
	msm7x30_sdc1_data.swfi_latency = msm7x30_power_collapse_latency(
		MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT);

	if (machine_is_msm7x30_fluid()) {
		msm7x30_sdc1_data.ocr_mask =  MMC_VDD_27_28 | MMC_VDD_28_29;
		if (msm_sdc1_lvlshft_enable()) {
			pr_err("%s: could not enable level shift\n");
			goto out1;
		}
	}

	msm_add_sdcc(1, &msm7x30_sdc1_data);
out1:
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (mmc_regulator_init(2, "s3", 1800000))
		goto out2;
	msm7x30_sdc2_data.swfi_latency = msm7x30_power_collapse_latency(
		MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT);

	if (machine_is_msm8x55_svlte_surf())
		msm7x30_sdc2_data.msmsdcc_fmax =  24576000;
	if (machine_is_msm8x55_svlte_surf() ||
			machine_is_msm8x55_svlte_ffa()) {
		msm7x30_sdc2_data.sdiowakeup_irq = MSM_GPIO_TO_INT(68);
		msm7x30_sdc2_data.is_sdio_al_client = 1;
	}

	msm_add_sdcc(2, &msm7x30_sdc2_data);
out2:
#endif

#if defined (T_QCI_IS3)
if (qci_get_offline_charging() == 1)
   return;
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	if (mmc_regulator_init(3, "s3", 1800000))
		goto out3;
	msm7x30_sdc3_data.swfi_latency = msm7x30_power_collapse_latency(
		MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT);

	msm_sdcc_setup_gpio(3, 1);
	msm_add_sdcc(3, &msm7x30_sdc3_data);
out3:
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	if (mmc_regulator_init(4, "mmc", 2850000))
		return;
	msm7x30_sdc4_data.swfi_latency = msm7x30_power_collapse_latency(
		MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT);

	msm_add_sdcc(4, &msm7x30_sdc4_data);
#endif

}

static void __init msm7x30_init_nand(void)
{
	char *build_id;
	struct flash_platform_data *plat_data;

	build_id = socinfo_get_build_id();
	if (build_id == NULL) {
		pr_err("%s: Build ID not available from socinfo\n", __func__);
		return;
	}

	if (build_id[8] == 'C' &&
			!msm_gpios_request_enable(msm_nand_ebi2_cfg_data,
			ARRAY_SIZE(msm_nand_ebi2_cfg_data))) {
		plat_data = msm_device_nand.dev.platform_data;
		plat_data->interleave = 1;
		printk(KERN_INFO "%s: Interleave mode Build ID found\n",
			__func__);
	}
}

#ifdef CONFIG_SERIAL_MSM_CONSOLE
static struct msm_gpio uart2_config_data[] = {
	{ GPIO_CFG(49, 2, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_RFR"},
	{ GPIO_CFG(50, 2, GPIO_CFG_INPUT,   GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_CTS"},
	{ GPIO_CFG(51, 2, GPIO_CFG_INPUT,   GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_Rx"},
	{ GPIO_CFG(52, 2, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_Tx"},
};

static void msm7x30_init_uart2(void)
{
	msm_gpios_request_enable(uart2_config_data,
			ARRAY_SIZE(uart2_config_data));

}
#endif

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
#ifndef T_QCI_IS3
#define TSIF_B_SYNC      GPIO_CFG(37, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#endif
#define TSIF_B_DATA      GPIO_CFG(36, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_EN        GPIO_CFG(35, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_CLK       GPIO_CFG(34, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_B_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_B_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_B_DATA, .label =  "tsif_data", },
#ifndef T_QCI_IS3	
	{ .gpio_cfg = TSIF_B_SYNC, .label =  "tsif_sync", },
#endif
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_pclk = "iface_clk",
	.tsif_ref_clk = "ref_clk",
};
#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */
/* TSIF end   */

static void __init pmic8058_leds_init(void)
{

/* FUJITSU:2011-12-1 mod LED start */
#if 0
	if (machine_is_msm7x30_surf())
#else
    if (1) {
        pm8058_7x30_data.leds_pdata = &pm8058_leds_data;
	} else if (machine_is_msm7x30_surf())
#endif
/* FUJITSU:2011-12-1 mod LED end */
		pm8058_7x30_data.leds_pdata = &pm8058_surf_leds_data;
	else if (!machine_is_msm7x30_fluid())
		pm8058_7x30_data.leds_pdata = &pm8058_ffa_leds_data;
	else if (machine_is_msm7x30_fluid())
		pm8058_7x30_data.leds_pdata = &pm8058_fluid_leds_data;
}

static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

#if defined(CONFIG_TOUCHSCREEN_TSC2007) || \
	defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)

#define TSC2007_TS_PEN_INT	20

static struct msm_gpio tsc2007_config_data[] = {
	{ GPIO_CFG(TSC2007_TS_PEN_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	"tsc2007_irq" },
};

static struct regulator_bulk_data tsc2007_regs[] = {
	{ .supply = "s3", .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "s2", .min_uV = 1300000, .max_uV = 1300000 },
};

static int tsc2007_init(void)
{
	int rc;

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(tsc2007_regs), tsc2007_regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);

	if (rc) {
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		goto reg_free;
	}

	rc = regulator_bulk_enable(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);

	if (rc) {
		pr_err("%s: could not enable regulators: %d\n", __func__, rc);
		goto reg_free;
	}

	rc = msm_gpios_request_enable(tsc2007_config_data,
			ARRAY_SIZE(tsc2007_config_data));
	if (rc) {
		pr_err("%s: Unable to request gpios\n", __func__);
		goto reg_disable;
	}

	return 0;

reg_disable:
	regulator_bulk_disable(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);
reg_free:
	regulator_bulk_free(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);
out:
	return rc;
}

static int tsc2007_get_pendown_state(void)
{
	int rc;

	rc = gpio_get_value(TSC2007_TS_PEN_INT);
	if (rc < 0) {
		pr_err("%s: MSM GPIO %d read failed\n", __func__,
						TSC2007_TS_PEN_INT);
		return rc;
	}

	return (rc == 0 ? 1 : 0);
}

static void tsc2007_exit(void)
{

	regulator_bulk_disable(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);
	regulator_bulk_free(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);

	msm_gpios_disable_free(tsc2007_config_data,
		ARRAY_SIZE(tsc2007_config_data));
}

static int tsc2007_power_shutdown(bool enable)
{
	int rc;

	rc = (enable == false) ?
		regulator_bulk_enable(ARRAY_SIZE(tsc2007_regs), tsc2007_regs) :
		regulator_bulk_disable(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);

	if (rc) {
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, enable ? "dis" : "en", rc);
		return rc;
	}

	if (enable == false)
		msleep(20);

	return 0;
}

static struct tsc2007_platform_data tsc2007_ts_data = {
	.model = 2007,
	.x_plate_ohms = 300,
	.min_x		= 210,
	.max_x		= 3832,
	.min_y		= 150,
	.max_y		= 3936,
	.irq_flags    = IRQF_TRIGGER_LOW,
	.init_platform_hw = tsc2007_init,
	.exit_platform_hw = tsc2007_exit,
	.power_shutdown	  = tsc2007_power_shutdown,
	.invert_x	  = true,
	.invert_y	  = true,
	/* REVISIT: Temporary fix for reversed pressure */
	.invert_z1	  = true,
	.invert_z2	  = true,
	.get_pendown_state = tsc2007_get_pendown_state,
};

static struct i2c_board_info tsc_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.irq		= MSM_GPIO_TO_INT(TSC2007_TS_PEN_INT),
		.platform_data = &tsc2007_ts_data,
	},
};
#endif

static int kp_flip_mpp_config(void)
{
	struct pm8xxx_mpp_config_data kp_flip_mpp = {
		.type = PM8XXX_MPP_TYPE_D_INPUT,
		.level = PM8018_MPP_DIG_LEVEL_S3,
		.control = PM8XXX_MPP_DIN_TO_INT,
	};

	return pm8xxx_mpp_config(PM8058_MPP_PM_TO_SYS(PM_FLIP_MPP),
						&kp_flip_mpp);
}

static struct flip_switch_pdata flip_switch_data = {
	.name = "kp_flip_switch",
	.flip_gpio = PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS) + PM_FLIP_MPP,
	.left_key = KEY_OPEN,
	.right_key = KEY_CLOSE,
	.active_low = 0,
	.wakeup = 1,
	.flip_mpp_config = kp_flip_mpp_config,
};

static struct platform_device flip_switch_device = {
	.name   = "kp_flip_switch",
	.id	= -1,
	.dev    = {
		.platform_data = &flip_switch_data,
	}
};

static struct regulator_bulk_data regs_tma300[] = {
	{ .supply = "gp6", .min_uV = 3050000, .max_uV = 3100000 },
	{ .supply = "gp7", .min_uV = 1800000, .max_uV = 1800000 },
};

static int tma300_power(int vreg_on)
{
	int rc;

	rc = vreg_on ?
		regulator_bulk_enable(ARRAY_SIZE(regs_tma300), regs_tma300) :
		regulator_bulk_disable(ARRAY_SIZE(regs_tma300), regs_tma300);

	if (rc)
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, vreg_on ? "en" : "dis", rc);
	return rc;
}

#define TS_GPIO_IRQ 150

static int tma300_dev_setup(bool enable)
{
	int rc;

	if (enable) {
		rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs_tma300),
				regs_tma300);

		if (rc) {
			pr_err("%s: could not get regulators: %d\n",
					__func__, rc);
			goto out;
		}

		rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_tma300),
				regs_tma300);

		if (rc) {
			pr_err("%s: could not set voltages: %d\n",
					__func__, rc);
			goto reg_free;
		}

		/* enable interrupt gpio */
		rc = gpio_tlmm_config(GPIO_CFG(TS_GPIO_IRQ, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: Could not configure gpio %d\n",
					__func__, TS_GPIO_IRQ);
			goto reg_free;
		}

		/* virtual keys */
		tma300_vkeys_attr.attr.name = "virtualkeys.msm_tma300_ts";
		properties_kobj = kobject_create_and_add("board_properties",
					NULL);
		if (!properties_kobj) {
			pr_err("%s: failed to create a kobject "
					"for board_properties\n", __func__);
			rc = -ENOMEM;
			goto reg_free;
		}
		rc = sysfs_create_group(properties_kobj,
				&tma300_properties_attr_group);
		if (rc) {
			pr_err("%s: failed to create a sysfs entry %s\n",
					__func__, tma300_vkeys_attr.attr.name);
			goto kobj_free;
		}
	} else {
		regulator_bulk_free(ARRAY_SIZE(regs_tma300), regs_tma300);
		/* destroy virtual keys */
		if (properties_kobj) {
			sysfs_remove_group(properties_kobj,
				&tma300_properties_attr_group);
			kobject_put(properties_kobj);
		}
	}
	return 0;

kobj_free:
	kobject_put(properties_kobj);
	properties_kobj = NULL;
reg_free:
	regulator_bulk_free(ARRAY_SIZE(regs_tma300), regs_tma300);
out:
	return rc;
}

static struct cy8c_ts_platform_data cy8ctma300_pdata = {
	.power_on = tma300_power,
	.dev_setup = tma300_dev_setup,
	.ts_name = "msm_tma300_ts",
	.dis_min_x = 0,
	.dis_max_x = 479,
	.dis_min_y = 0,
	.dis_max_y = 799,
	.res_x	 = 479,
	.res_y	 = 1009,
	.min_tid = 1,
	.max_tid = 255,
	.min_touch = 0,
	.max_touch = 255,
	.min_width = 0,
	.max_width = 255,
	.invert_y = 1,
	.nfingers = 4,
	.irq_gpio = TS_GPIO_IRQ,
	.resout_gpio = -1,
};

static struct i2c_board_info cy8ctma300_board_info[] = {
	{
		I2C_BOARD_INFO("cy8ctma300", 0x2),
		.platform_data = &cy8ctma300_pdata,
	}
};

int factory_mode(void);

static void __init msm7x30_init(void)
{
	int rc;
	unsigned smem_size;
	uint32_t usb_hub_gpio_cfg_value = GPIO_CFG(56,
						0,
						GPIO_CFG_OUTPUT,
						GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA);
	uint32_t soc_version = 0;

	soc_version = socinfo_get_version();

	msm_clock_init(&msm7x30_clock_init_data);
#ifdef CONFIG_SERIAL_MSM_CONSOLE
	msm7x30_init_uart2();
#endif

#if defined (T_QCI_IS3)
if (qci_get_offline_charging() == 0)
{
#endif

	msm_spm_init(&msm_spm_data, 1);
	acpuclk_init(&acpuclk_7x30_soc_data);
	if (machine_is_msm7x30_surf() || machine_is_msm7x30_fluid())
		msm7x30_cfg_smsc911x();

#ifdef CONFIG_USB_MSM_OTG_72K
	if (SOCINFO_VERSION_MAJOR(soc_version) >= 2 &&
			SOCINFO_VERSION_MINOR(soc_version) >= 1) {
		pr_debug("%s: SOC Version:2.(1 or more)\n", __func__);
		msm_otg_pdata.ldo_set_voltage = 0;
	}

	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
 	msm_pm_data
 	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif

#ifdef MSM_UARTDM2_USE
	msm_uart_dm2_pdata.wakeup_irq = gpio_to_irq(51);
	msm_device_uart_dm2.dev.platform_data = &msm_uart_dm2_pdata;
#endif

#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif
	if (machine_is_msm7x30_fluid()) {
		msm_adc_pdata.dev_names = msm_adc_fluid_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_fluid_device_names);
	} else {
		msm_adc_pdata.dev_names = msm_adc_surf_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_surf_device_names);
	}

	pmic8058_leds_init();

	buses_init();

#ifdef CONFIG_MSM_SSBI
	msm_device_ssbi_pmic1.dev.platform_data =
				&msm7x30_ssbi_pm8058_pdata;
#endif

	platform_add_devices(msm_footswitch_devices,
			     msm_num_footswitch_devices);
	platform_add_devices(devices, ARRAY_SIZE(devices));
#ifdef CONFIG_USB_EHCI_MSM_72K
	msm_add_host(0, &msm_usb_host_pdata);
#endif
	msm7x30_init_mmc();
	msm7x30_init_nand();
	msm_qsd_spi_init();

#ifdef CONFIG_SPI_QSD
 #if defined(T_QCI_IS3)
		spi_register_board_info(lcdc_chimei_spi_board_info,
			ARRAY_SIZE(lcdc_chimei_spi_board_info));
 #else
	if (machine_is_msm7x30_fluid())
		spi_register_board_info(lcdc_sharp_spi_board_info,
			ARRAY_SIZE(lcdc_sharp_spi_board_info));
	else
		spi_register_board_info(lcdc_toshiba_spi_board_info,
			ARRAY_SIZE(lcdc_toshiba_spi_board_info));
#endif
#endif

	msm_fb_add_devices();
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));
	i2c_pull_up_voltage();
#if defined(CONFIG_I2C_GPIO)		
	gpio_i2c_pull_up_voltage();
#endif
	msm_device_i2c_init();
	msm_device_i2c_2_init();
	qup_device_i2c_init();
	msm7x30_init_marimba();
#ifdef CONFIG_MSM7KV2_AUDIO
	snddev_poweramp_gpio_init();
	snddev_hsed_voltage_init();
	aux_pcm_gpio_init();
#endif

/* FUJITSU:2012-01-13 factory start */
	factory_read_nvitem_init();
/* FUJITSU:2012-01-13 factory end */

	i2c_register_board_info(0, msm_i2c_board_info,
			ARRAY_SIZE(msm_i2c_board_info));

	gpio_tlmm_config( GPIO_CFG(148, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_tlmm_config( GPIO_CFG(124, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_tlmm_config( GPIO_CFG(125, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_tlmm_config( GPIO_CFG(126, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_set_value(126,0);

	i2c_register_board_info(2, msm_i2c_gsbi7_timpani_info,
			ARRAY_SIZE(msm_i2c_gsbi7_timpani_info));
#if defined(CONFIG_I2C_GPIO)
	i2c_register_board_info(8/* GPIO I2C ID */, msm_camera_boardinfo,
				ARRAY_SIZE(msm_camera_boardinfo));
#else
	i2c_register_board_info(4 /* QUP ID */, msm_camera_boardinfo,
				ARRAY_SIZE(msm_camera_boardinfo));
#endif

// Fujitsu, add for BCM4430 Start
       wlan_power_init();
// Fujitsu, add for BCM4430 End

	i2c_register_board_info(4 /* QUP ID */, ft5x0x_boardinfo, ARRAY_SIZE(ft5x0x_boardinfo));
	if(qci_read_hw_id() < 3){
		i2c_register_board_info(0, taos_board_info, ARRAY_SIZE(taos_board_info));
	}else{
		i2c_register_board_info(0, stk_board_info, ARRAY_SIZE(stk_board_info));
	}

if ( qci_read_hw_id() == 1 )  {
	;
}else{
	fm_i2s_gpio_init();
}

	bt_power_init();

	fmt_power_init();

#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif

#if defined(CONFIG_TOUCHSCREEN_TSC2007) || \
	defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)
	if (machine_is_msm8x55_svlte_ffa())
		i2c_register_board_info(2, tsc_i2c_board_info,
				ARRAY_SIZE(tsc_i2c_board_info));
#endif

	if (machine_is_msm7x30_surf())
		platform_device_register(&flip_switch_device);

	pm8058_gpios_init();

	platform_device_register(&msm_gpio_keys);

	if (machine_is_msm7x30_fluid()) {
		/* Initialize platform data for fluid v2 hardware */
		if (SOCINFO_VERSION_MAJOR(
				socinfo_get_platform_version()) == 2) {
			cy8ctma300_pdata.res_y = 920;
			cy8ctma300_pdata.invert_y = 0;
		}
		i2c_register_board_info(0, cy8ctma300_board_info,
			ARRAY_SIZE(cy8ctma300_board_info));
	}

	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = gpio_tlmm_config(usb_hub_gpio_cfg_value, GPIO_CFG_ENABLE);
		if (rc)
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, usb_hub_gpio_cfg_value, rc);
	}
	msm_init_pmic_vibrator();

#if defined (T_QCI_IS3)
}
else
{
	printk(KERN_ERR "\n\n\n [Offline-Charging] msm7x30_init\n\n\n");

	msm_spm_init(&msm_spm_data, 1);
	acpuclk_init(&acpuclk_7x30_soc_data);
	if (machine_is_msm7x30_surf() || machine_is_msm7x30_fluid())
		msm7x30_cfg_smsc911x();

#ifdef CONFIG_USB_MSM_OTG_72K
	if (SOCINFO_VERSION_MAJOR(soc_version) >= 2 &&
			SOCINFO_VERSION_MINOR(soc_version) >= 1) {
		pr_debug("%s: SOC Version:2.(1 or more)\n", __func__);
		msm_otg_pdata.ldo_set_voltage = 0;
	}

	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
       msm_pm_data
       [MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif

#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif
	if (machine_is_msm7x30_fluid()) {
		msm_adc_pdata.dev_names = msm_adc_fluid_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_fluid_device_names);
	} else {
		msm_adc_pdata.dev_names = msm_adc_surf_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_surf_device_names);
	}

	pmic8058_leds_init();

	buses_init();

#ifdef CONFIG_MSM_SSBI
	msm_device_ssbi_pmic1.dev.platform_data =
				&msm7x30_ssbi_pm8058_pdata;
#endif

	platform_add_devices(msm_footswitch_devices,
			     msm_num_footswitch_devices);
	platform_add_devices(devices_oc, ARRAY_SIZE(devices_oc));
#ifdef CONFIG_USB_EHCI_MSM_72K
	msm_add_host(0, &msm_usb_host_pdata);
#endif

	msm7x30_init_mmc();
	msm7x30_init_nand();

	msm_fb_add_devices();
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));

	pm_control_voltage_offline_charging();
	msm_device_i2c_init();
	i2c_register_board_info(0, msm_i2c_board_info_offline_charging,
			ARRAY_SIZE(msm_i2c_board_info_offline_charging));

#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif

	if (machine_is_msm7x30_surf())
		platform_device_register(&flip_switch_device);

	pm8058_gpios_init();

	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = gpio_tlmm_config(usb_hub_gpio_cfg_value, GPIO_CFG_ENABLE);
		if (rc)
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, usb_hub_gpio_cfg_value, rc);
	}
}
#endif

	boot_reason = *(unsigned int *)
		(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size));
	printk(KERN_NOTICE "Boot Reason = 0x%02x\n", boot_reason);
}

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned fluid_pmem_adsp_size = MSM_FLUID_PMEM_ADSP_SIZE;
static int __init fluid_pmem_adsp_size_setup(char *p)
{
	fluid_pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("fluid_pmem_adsp_size", fluid_pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static unsigned pmem_kernel_ebi0_size = PMEM_KERNEL_EBI0_SIZE;
static int __init pmem_kernel_ebi0_size_setup(char *p)
{
	pmem_kernel_ebi0_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi0_size", pmem_kernel_ebi0_size_setup);

static struct memtype_reserve msm7x30_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	unsigned long size;

	if machine_is_msm7x30_fluid()
		size = fluid_pmem_adsp_size;
	else
		size = pmem_adsp_size;
	android_pmem_adsp_pdata.size = size;
	android_pmem_audio_pdata.size = pmem_audio_size;
	android_pmem_pdata.size = pmem_sf_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x30_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	reserve_memory_for(&android_pmem_pdata);
	msm7x30_reserve_table[MEMTYPE_EBI0].size += pmem_kernel_ebi0_size;
#endif
}

#if defined(T_QCI_IS3)
static int atoi(const char *name)
{
       int val = 0;

       for (;; name++) {
               switch (*name) {
               case '0' ... '9':
                       val = 10*val+(*name-'0');
                       break;
               default:
                       return val;
               }
       }
}
static unsigned hw_id = 0;
int __init hw_id_setup(char *p)
{
       hw_id = atoi(p);

       return 1;
}
__setup("hw_id=", hw_id_setup);

inline int qci_read_hw_id(void)
{
       return hw_id;
}

EXPORT_SYMBOL(qci_read_hw_id);
#endif

#if defined(T_QCI_IS3)
static unsigned offline_charging = 0;
int __init offline_charging_setup(char *p)
{
	if (!strcmp(p,"on"))
	   offline_charging = 1;

	return 1;
}
__setup("offline.charging=", offline_charging_setup);

inline int qci_get_offline_charging(void)
{
       return offline_charging;
}

EXPORT_SYMBOL(qci_get_offline_charging);

static unsigned recovery_charging = 0;
int __init recovery_charging_setup(char *p)
{
	if (!strcmp(p,"on"))
	   recovery_charging = 1;

	return 1;
}
__setup("recovery.charging=", recovery_charging_setup);

inline int qci_get_recovery_charging(void)
{
       return recovery_charging;
}

EXPORT_SYMBOL(qci_get_recovery_charging);
#endif

static void __init msm7x30_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x30_paddr_to_memtype(unsigned int paddr)
{
	if (paddr < 0x40000000)
		return MEMTYPE_EBI0;
	if (paddr >= 0x40000000 && paddr < 0x80000000)
		return MEMTYPE_EBI1;
	return MEMTYPE_NONE;
}

static struct reserve_info msm7x30_reserve_info __initdata = {
	.memtype_reserve_table = msm7x30_reserve_table,
	.calculate_reserve_sizes = msm7x30_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x30_paddr_to_memtype,
};

static void __init msm7x30_reserve(void)
{
	reserve_info = &msm7x30_reserve_info;
	msm_reserve();
}

static void __init msm7x30_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
#if CONFIG_ANDROID_RAM_CONSOLE


        #define MSM_RAM_CONSOLE_BASE    UL(0x60000000 - 128 * SZ_1K)
        #define MSM_RAM_CONSOLE_SIZE	(128 * SZ_1K)

        size = MSM_RAM_CONSOLE_SIZE;
        ram_console_resource[0].start   = MSM_RAM_CONSOLE_BASE;
        ram_console_resource[0].end     = MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1;
        pr_info("allocating %lu bytes at %p (%x physical) for ram_console\n",
             size, __va(ram_console_resource[0].start), ram_console_resource[0].start);
#endif        
}

static void __init msm7x30_map_io(void)
{
	msm_shared_ram_phys = 0x00100000;
	msm_map_msm7x30_io();
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
}

static void __init msm7x30_init_early(void)
{
	msm7x30_allocate_memory_regions();
}

MACHINE_START(MSM7X30_SURF, "QCT MSM7X30 SURF")
	.boot_params = PLAT_PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END

MACHINE_START(MSM7X30_FFA, "QCT MSM7X30 FFA")
	.boot_params = PLAT_PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END

MACHINE_START(MSM7X30_FLUID, "QCT MSM7X30 FLUID")
	.boot_params = PLAT_PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END

MACHINE_START(MSM8X55_SURF, "QCT MSM8X55 SURF")
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END

MACHINE_START(MSM8X55_FFA, "QCT MSM8X55 FFA")
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END
MACHINE_START(MSM8X55_SVLTE_SURF, "QCT MSM8X55 SVLTE SURF")
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END
MACHINE_START(MSM8X55_SVLTE_FFA, "QCT MSM8X55 SVLTE FFA")
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END

#if defined(T_QCI_IS3)
int set_spi_gpio_exclusive_ctrl(int runtype, int csno)
{
	unsigned long spinlock_flags;
	int set_spi_gpio_use_state_old;
	int ret = 0;
	/*******************/
	/* parameter check */
	/*******************/
	if (((csno < 0) || (csno > 3)) || ((runtype < 0) || (runtype > 1))) {
		pr_err("%s: csno input data failed csno = %d runtype = %d\n", __func__, csno, runtype);
		return -2;
	}

	spin_lock_irqsave(&set_spi_gpio_use_state_lock, spinlock_flags);

	/********************/
	/* update use_state */
	/********************/
	set_spi_gpio_use_state_old = set_spi_gpio_use_state;
	if (runtype == 0) {
		set_spi_gpio_use_state &= ~(1<<csno);
	}
	else {
    	set_spi_gpio_use_state |= (1<<csno);
	}

	/***************************/
	/* use_state changing 0->1 */
	/***************************/
	if ((set_spi_gpio_use_state_old == 0) && (set_spi_gpio_use_state != 0)) {
		/* -- start gpio setting -- */
	    /* spi_clk */
	    ret = gpio_tlmm_config(GPIO_CFG(45, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	    if (ret) {
			pr_err("%s: Could not configure gpio errno = %x portno = %d runtype = %d csno = %d\n", __func__, ret, 45, runtype, csno);
			spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
			return -1;
	    }
	    /* spi_mosi */
	    ret = gpio_tlmm_config(GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	    if (ret) {
			pr_err("%s: Could not configure gpio errno = %x portno = %d runtype = %d csno = %d\n", __func__, ret, 47, runtype, csno);
			spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
			return -1;
	    }
	    /* spi_miso */
	    ret = gpio_tlmm_config(GPIO_CFG(48, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	    if (ret) {
			pr_err("%s: Could not configure gpio errno = %x portno = %d runtype = %d csno = %d\n", __func__, ret, 48, runtype, csno);
			spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
			return -1;
	    }
		printk("%s: success runtype = %d csno = %d state = %d\n", __func__, runtype, csno, set_spi_gpio_use_state);
	}

	/***************************/
	/* use_state changing 1->0 */
	/***************************/
	else if ((set_spi_gpio_use_state_old != 0) && (set_spi_gpio_use_state == 0)) {
		/* -- start spi setting -- */
	    /* spi_clk */
	    ret = gpio_tlmm_config(GPIO_CFG(45, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	    if (ret) {
			pr_err("%s: Could not configure gpio errno = %x portno = %d runtype = %d csno = %d\n", __func__, ret, 45, runtype, csno);
			spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
			return -1;
	    }
	    /* spi_mosi */
	    ret = gpio_tlmm_config(GPIO_CFG(47, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	    if (ret) {
			pr_err("%s: Could not configure gpio errno = %x portno = %d runtype = %d csno = %d\n", __func__, ret, 47, runtype, csno);
			spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
			return -1;
	    }
	    /* spi_miso */
	    ret = gpio_tlmm_config(GPIO_CFG(48, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	    if (ret) {
			pr_err("%s: Could not configure gpio errno = %x portno = %d runtype = %d csno = %d\n", __func__, ret, 48, runtype, csno);
			spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
			return -1;
	    }
		printk("%s: success runtype = %d csno = %d state = %d\n", __func__, runtype, csno, set_spi_gpio_use_state);
	}

	/***********************************/
	/* use_state changing 0->0 or 1->1 */
	/***********************************/
	else {
		printk("%s: non setting runtype = %d csno = %d state = %d\n", __func__, runtype, csno, set_spi_gpio_use_state);
	}

	spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
	return 0;

}
EXPORT_SYMBOL(set_spi_gpio_exclusive_ctrl);
#endif

/* FUJITSU:2011-12-1 add factory start */
int factory_mode(void)
{
/* FUJITSU:2012-01-13 factory start */
	return factory_value;
//return NV_BOOT_KERNEL_TEST;
#if 0
	return 0;
#endif
/* FUJITSU:2012-01-13 factory end */
}

/* FUJITSU:2012-01-13 factory start */
void factory_read_nvitem_init(void){
	static unsigned char *dataread = NULL;

	unsigned int nv_data = 0;
	int ret = 0;
	unsigned int tmp_id = NV_BOOT_TESTMODE_STATUS_I;

	dataread = (unsigned char *)smem_alloc_vendor1(SMEM_OEM_013);
	if(dataread == NULL){
		printk( "[fct] fct flg null.\n");
		factory_value = FACTORY_DEFAULT_VALUE;
	}
	else if(*dataread != 0){
		printk( "[fct] nm boot.\n");
		factory_value = FACTORY_DEFAULT_VALUE;
	}
	else if(*dataread == 0){
		printk( "[fct] fct boot.\n");
factory_value=NV_BOOT_KERNEL_TEST;
return;

		ret = msm_proc_comm(PCOM_OEM_011, &tmp_id, &nv_data);

		if (ret == -1) {
			printk("NV read error %d\n", ret);
			factory_value = FACTORY_DEFAULT_VALUE;
		}

		printk("[fct] NV_BOOT_TESTMODE_STATUS_I=%d\n", nv_data);

		switch(nv_data){
		case NV_BOOT_NONE:
			factory_value = NV_BOOT_NONE;
			break;
		case NV_BOOT_TESTMODE:
			factory_value = NV_BOOT_TESTMODE;
			break;
		case NV_BOOT_KERNEL_TEST:
			factory_value = NV_BOOT_KERNEL_TEST;
			break;
		case NV_BOOT_ANDROID_TEST:
			factory_value = NV_BOOT_ANDROID_TEST;
			break;
		default:
			factory_value = FACTORY_DEFAULT_VALUE;
		}
	}
}
/* FUJITSU:2012-01-13 factory end */

EXPORT_SYMBOL(factory_mode);
/* FUJITSU:2011-12-1 add factory end */
