/*
 * Copyright(C) 2012 FUJITSU LIMITED
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
#ifndef __IS3_MAX17040GT_H__
#define __IS3_MAX17040GT_H__

#define  USE_CHARGING_4350MV_BATTERY_PACK	1	// set 1 : use 4350MV battery pack , set 0 : use 4200MV battery pack
#if USE_CHARGING_4350MV_BATTERY_PACK
    #define  MAX_CHARGING_VOLTAGE_MODE		0	// (set 0 : max voltage = 4350 , set 1 : max voltage = 4200) for 4350mV battery pack
#endif

//-----------------------------------------------------------
// MAX17040GT REGISTERS
//-----------------------------------------------------------
#define MAX17040GT_VCELL_MSB	0x02
#define MAX17040GT_VCELL_LSB	0x03
#define MAX17040GT_SOC_MSB	0x04
#define MAX17040GT_SOC_LSB	0x05
#define MAX17040GT_MODE_MSB	0x06
#define MAX17040GT_MODE_LSB	0x07
#define MAX17040GT_VER_MSB	0x08
#define MAX17040GT_VER_LSB		0x09
#define MAX17040GT_RCOMP_MSB	0x0C
#define MAX17040GT_RCOMP_LSB	0x0D
#define MAX17040GT_OCV_MSB	0x0E
#define MAX17040GT_OCV_LSB	0x0F
#define MAX17040GT_UNLOCK_MODEL_MSB	0x3E
#define MAX17040GT_UNLOCK_MODEL_LSB	0x3F
#define MAX17040GT_START_MODEL	0x40
#define MAX17040GT_CMD_MSB	0xFE
#define MAX17040GT_CMD_LSB	0xFF

struct max17040gt_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct delayed_work		msm_work;
	struct delayed_work		pa_work;

	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
};

//Custom Model INI
#if USE_CHARGING_4350MV_BATTERY_PACK
static u8 max17040gt_INI_RCOMP0;
static s16 max17040gt_INI_TempCoUp_mul100;
static s16 max17040gt_INI_TempCoDown_mul100;
static int max17040gt_INI_TempUnit;
static u16 max17040gt_INI_OCVTest;
static u8 max17040gt_INI_SOCCheckA;
static u8 max17040gt_INI_SOCCheckB;
static u8 max17040gt_INI_bit;
static u8 *max17040gt_model_data;
static u8 max17040gt_INI_RCOMP0_4350 = 73;
static s16 max17040gt_INI_TempCoUp_mul100_4350 = -525;
static s16 max17040gt_INI_TempCoDown_mul100_4350 = -5475;
static int max17040gt_INI_TempUnit_4350 = 1000;
static u16 max17040gt_INI_OCVTest_4350 = 57728;
static u8 max17040gt_INI_SOCCheckA_4350 = 231;
static u8 max17040gt_INI_SOCCheckB_4350 = 233;
static u8 max17040gt_INI_bit_4350 = 19;
static u8 max17040gt_model_data_4350[64] = {0xA5,0xF0,0xB7,0xD0,0xBA,0x10,0xBB,0x80,0xBC,0x20,
	                                                                     0xBC,0x80,0xBC,0xF0,0xBD,0xB0,0xBE,0x70,0xBF,0xF0,
                                                                            0xC2,0xC0,0xC5,0x40,0xC8,0x00,0xCA,0x60,0xCF,0xE0,
                                                                            0xD7,0x80,0x03,0xF0,0x25,0x50,0x00,0x60,0x3B,0x90,
                                                                            0x64,0x00,0x70,0xC0,0x45,0x70,0x1B,0xF0,0x30,0xF0,
                                                                            0x10,0xC0,0x13,0xE0,0x12,0xF0,0x12,0x60,0x0E,0xF0,
                                                                            0x0D,0x10,0x0D,0x10};
static u8 max17040gt_INI_RCOMP0_4200 = 60;
static s16 max17040gt_INI_TempCoUp_mul100_4200 = -500;
static s16 max17040gt_INI_TempCoDown_mul100_4200 = -4175;
static int max17040gt_INI_TempUnit_4200 = 1000;
static u16 max17040gt_INI_OCVTest_4200 = 55920;
static u8 max17040gt_INI_SOCCheckA_4200 = 244;
static u8 max17040gt_INI_SOCCheckB_4200 = 246;
static u8 max17040gt_INI_bit_4200 = 19;
static u8 max17040gt_model_data_4200[64] = {0x82,0x30,0xAE,0x20,0xB6,0xC0,0xB8,0xF0,0xBB,0x90,
                                                                            0xBB,0xD0,0xBC,0x50,0xBD,0x40,0xBE,0x20,0xBF,0x70,
                                                                            0xC1,0xA0,0xC4,0x00,0xC7,0x40,0xCA,0x20,0xCD,0x20,
                                                                            0xD0,0x70,0x00,0x20,0x09,0x00,0x14,0x70,0x14,0x10,
                                                                            0x70,0x80,0x41,0x50,0x5D,0x10,0x4F,0xA0,0x32,0xF0,
                                                                            0x14,0xF0,0x13,0xB0,0x17,0xB0,0x13,0xE0,0x11,0xF0,
                                                                            0x11,0xC0,0x11,0xC0};
#else
static u8 max17040gt_INI_RCOMP0 = 72;
static s16 max17040gt_INI_TempCoUp_mul100 = -25;
static s16 max17040gt_INI_TempCoDown_mul100 = -515;
static int max17040gt_INI_TempUnit = 100;
static u16 max17040gt_INI_OCVTest = 55952;
static u8 max17040gt_INI_SOCCheckA = 246;
static u8 max17040gt_INI_SOCCheckB = 248;
static u8 max17040gt_INI_bit = 19;
static u8 max17040gt_model_data[64] = {0xA0,0x30,0xAF,0xB0,0xB8,0x30,0xB9,0x40,0xB9,0xA0,
                                                                   0xBB,0x90,0xBC,0x50,0xBC,0xB0,0xBD,0x00,0xBD,0x80,
                                                                   0xBF,0x50,0xC0,0x90,0xC5,0xC0,0xC8,0xA0,0xCC,0xB0,
                                                                   0xD0,0x90,0x01,0xE0,0x04,0x20,0x37,0x60,0x7B,0xB0,
                                                                   0x00,0xA0,0x56,0xB0,0x3C,0x50,0x61,0xC0,0x7C,0x50,
                                                                   0x1E,0xD0,0x21,0xA0,0x1A,0x20,0x1B,0x50,0x0F,0xA0,
                                                                   0x13,0x00,0x13,0x00};
#endif

static int max17040gt_initial_voltage(struct i2c_client *client);
static int max17040gt_update_RCOMP(struct i2c_client *client,int temp);
static int max17040gt_load_custom_model(struct i2c_client *client,int mode);
static int max17040gt_verify_model(struct i2c_client *client,int type);
static void max17040gt_quick_start(struct i2c_client *client);
static void max17040gt_reset(struct i2c_client *client);
static void max17040gt_get_vcell(struct i2c_client *client);
static int max17040gt_get_soc(struct i2c_client *client);
static int max17040gt_get_version(struct i2c_client *client);
static int max17040gt_get_RCOMP(struct i2c_client *client,u8 *msb,u8 *lsb);
static void max17040gt_work(struct work_struct *work);

#endif
