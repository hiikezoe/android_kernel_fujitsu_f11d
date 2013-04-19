/*
 * Device driver for monitoring ambient light intensity (lux)
 * and proximity (prox) within the TAOS TSL277X family of devices.
 *
 * Copyright (c) 2011, TAOS Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/input-polldev.h>
#include <linux/delay.h>
#include "../../../arch/arm/mach-msm/proc_comm.h"

/* 2571 register offsets */
#define TSL277X_MAX_DEVICE_REGS	32
#define	TSL277X_REG_MAX		16

/* Device Registers and Masks */
#define TSL277X_CNTRL			0x00
#define TSL277X_ALS_TIME		0X01

#define TSL277X_ALS_MINTHRESHLO	0X04
#define TSL277X_ALS_MINTHRESHHI	0X05
#define TSL277X_ALS_MAXTHRESHLO	0X06
#define TSL277X_ALS_MAXTHRESHHI	0X07

#define TSL277X_PRX_MINTHRESHLO	0X08
#define TSL277X_PRX_MINTHRESHHI	0X09
#define TSL277X_PRX_MAXTHRESHLO	0X0A
#define TSL277X_PRX_MAXTHRESHHI	0X0B

#define TSL277X_PERSISTENCE		0x0c

#define TSL277X_PRX_COUNT		0x0E
#define TSL277X_GAIN			0x0f
#define	TSL277X_STATUS			0x13
#define TSL277X_REVID			0x11
#define TSL277X_CHIPID			0x12
#define TSL277X_ALS_CHAN0LO		0x14
#define TSL277X_ALS_CHAN0HI		0x15
#define TSL277X_ALS_CHAN1LO		0x16
#define TSL277X_ALS_CHAN1HI		0x17

#define TSL277X_PRX_LO			0x18
#define TSL277X_PRX_HI			0x19


/* tsl2771 cmd reg masks */
#define TSL277X_CMD_REG			0x80
#define TSL277X_CMD_SPL_FN		0x60

#define TSL277X_CMD_PROX_INT_CLR	0X05
#define TSL277X_CMD_ALS_INT_CLR	0x06
#define CMD_PROXALS_INT_CLR		0X07

/* tsl2771 cntrl reg masks */
#define TSL277X_CNTL_ADC_ENBL	0x02
#define TSL277X_CNTL_PWR_ON		0x01

/* tsl2771 status reg masks */
#define TSL277X_STA_ADC_VALID	0x01
#define TSL277X_STA_PRX_VALID	0x02
#define TSL277X_STA_ADC_PRX_VALID	0x03
#define STA_ALSINTR			0x10
#define STA_ADCINTR			0x10
#define STA_PRXINTR			0x20

#define TSL277X_STA_ADC_INTR	0x10

/* Triton cntrl reg masks */
#define CNTL_REG_CLEAR			0x00
#define CNTL_PROX_INT_ENBL		0X20
#define CNTL_ALS_INT_ENBL		0X10
#define TSL277X_CNTL_WAIT_TMR_ENBL	0X08
#define CNTL_PROX_DET_ENBL		0X04
#define CNTL_ADC_ENBL			0x02
#define TSL277X_CNTL_PWRON		0x01
#define CNTL_ALSPON_ENBL		0x03
#define CNTL_INTALSPON_ENBL		0x13
#define CNTL_PROXPON_ENBL		0x0F
#define CNTL_INTPROXPON_ENBL	0x2F
#define TSL277X_CMD_PROXALS_INTCLR	0X07

/*Prox diode to use */
#define DIODE0				0x10
#define DIODE1				0x20
#define DIODE_BOTH			0x30

/* LED Power */
#define mA100				0x00
#define mA50				0x40
#define mA25				0x80
#define mA13				0xD0

/* Cal defs*/
#define	PROX_STAT_CAL	0
#define	PROX_STAT_SAMP	1
#define	MAX_SAMPLES_CAL	200

/* Lux calculation constants */
#define	TSL277X_LUX_CALC_OVER_FLOW	65535

enum {
	TSL277X_CHIP_UNKNOWN = 0,
	TSL277X_CHIP_WORKING = 1,
	TSL277X_CHIP_SUSPENDED = 2,
	TSL277X_CHIP_ALS_ENABLE = 3,
	TSL277X_CHIP_PROX_ENABLE = 4
} TSL277X_CHIP_WORKING_STATUS;

/* Per-device data */
struct taos_als_info {
	u16 als_ch0;
	u16 als_ch1;
	u16 lux;
};

/* proximity data */
struct taos_prox_info {
	u16 prox_data;
	int prox_event;
};

struct prox_stat {
	u16 min;
	u16 max;
	u16 mean;
	unsigned long stddev;
} ;

struct taos_settings {
	int als_time;
	int als_gain;
	int als_gain_trim;
	int als_cal_target;
	u8	als_interrupt;
	u8	als_persistence;
	int	als_thresh_low;
	int	als_thresh_high;
	int	als_sf;
	int	prox_thres_high;
	int	prox_thres_low;
	int	prox_thres;
	int	prox_pulse_count;
	int	prox_max_samples_cal;	/* for calibration mode*/

};

#define	MAXPROXFILTER	100
u16 taos_prox_filter_data[MAXPROXFILTER];	/*history buffer*/

struct tsl2771_chip {
	struct mutex prox_mutex;
	struct mutex als_mutex;
	struct i2c_client *client;
	struct taos_prox_info prox_cur_info;
	struct taos_als_info als_cur_info;
	struct taos_settings taos_settings;
	int als_time_scale;
	int als_saturation;
	int taos_chip_status;
	u8 taos_config[TSL277X_REG_MAX];
	bool init_done;
	struct work_struct	work_thresh;
	s64	event_timestamp;
	unsigned int irq_no;
	struct input_dev *idev_als;
	struct input_polled_dev *idev_prox;
};

struct i2c_client * tsl2771_client=NULL;
struct tsl2771_chip *tsl2771_chip_dev=NULL;
int last_prox_event = 0;

/*
 * Initial values for device - this values can/will be changed by driver.
 * and applications as needed.
 * These values are dynamic.
 */
static u8 tsl2771_taos_config[] = {
		0x00, 0xee, 0xFF, 0xF5, 0x03, 0x00, 0x00, 0x01,
	/*	Enabl atime ptime wtime AtL0  AtL1  AtH0  AtH1 */
		0x00, 0x00, 0x00, 0x03,	0x30, 0x00, 0x0a, 0x20
	/*	PtL0  PtL1  PtH0  PtH1  Pers  CFG   Pcnt  CTRL */
};

struct taos_lux {
	unsigned int ratio;
	unsigned int ch0;
	unsigned int ch1;
};

/* This structure is intentionally large to accommodate updates via sysfs. */
/* Sized to 11 = max 10 segments + 1 termination segment */
/* Assumption is one and only one type of glass used  */
struct taos_lux taos_device_lux[11] = {
	{ 14461,   611,  1211 },
	{ 18540,   352,   623 },
	{     0,     0,     0 },
};

struct gainadj {
	s16 ch0;
	s16 ch1;
};

/* Used to validate the gain selection index */
static const struct gainadj tsl2771_gainadj[] = {
	{ 1, 1 },
	{ 8, 8 },
	{ 16, 16 },
	{ 120, 120 }
};

static int first = 1;
int AVG_SF = 100,als_sf0 = 1,als_sf1 = 1,als_sf2 = 1,prox_h = 650,prox_l = 400;
static int ascii_to_int(uint32_t data)
{
	int i =0, result = 0;
	u8 tmp = 0;

	printk("hex: %x\n",data);
	for(i = 0; i < 4 ; i++){
		tmp = (data >> (24 - 8*i)) & 0xff;
		result = result*10 + (tmp -48);
	}
	printk("result: %x\n",result);
	return result;
}

/*
 * Read a number of bytes starting at register (reg) location.
 * Return 0, or i2c_smbus_write_byte ERROR code.
 */
static int
taos_i2c_read(struct i2c_client *client, u8 reg, u8 *val, unsigned int len)
{
	int ret;
	int i;

	for (i = 0; i < len; i++) {
		/* select register to write */
		ret = i2c_smbus_write_byte(client, (TSL277X_CMD_REG | reg));
		if (ret < 0) {
			dev_err(&client->dev, "taos_i2c_read failed to write"
				" register %x\n", reg);
			return ret;
		}
		/* read the data */
		*val = i2c_smbus_read_byte(client);
		val++;
		reg++;
	}
	return 0;
}

/*
 * Reads and calculates current lux value.
 * The raw ch0 and ch1 values of the ambient light sensed in the last
 * integration cycle are read from the device.
 * Time scale factor array values are adjusted based on the integration time.
 * The raw values are multiplied by a scale factor, and device gain is obtained
 * using gain index. Limit checks are done next, then the ratio of a multiple
 * of ch1 value, to the ch0 value, is calculated. The array taos_device_lux[]
 * declared above is then scanned to find the first ratio value that is just
 * above the ratio we just calculated. The ch0 and ch1 multiplier constants in
 * the array are then used along with the time scale factor array values, to
 * calculate the lux.
 */
static int taos_get_lux(struct i2c_client *client)
{
	u16 ch0, ch1; /* separated ch0/ch1 data from device */
	u32 lux; /* raw lux calculated from device data */
	u32 ratio;
	u8 buf[4];
	struct taos_lux *p;
	struct tsl2771_chip *chip = i2c_get_clientdata(client);
	int i, ret;
	u32 ch0lux = 0;
	u32 ch1lux = 0;

	if (mutex_trylock(&chip->als_mutex) == 0) {
		dev_info(&client->dev, "taos_get_lux device is busy\n");
		return chip->als_cur_info.lux; /* busy, so return LAST VALUE */
	}

	if ((chip->taos_chip_status != TSL277X_CHIP_WORKING) && (chip->taos_chip_status != TSL277X_CHIP_ALS_ENABLE)) {
		/* device is not enabled */
		dev_err(&client->dev, "taos_get_lux device is not enabled\n");
		ret = -EBUSY ;
		goto out_unlock;
	}

	ret = taos_i2c_read(client,
		(TSL277X_CMD_REG | TSL277X_STATUS), &buf[0], 1);
	if (ret < 0) {
		dev_err(&client->dev,
			"taos_get_lux failed to read CMD_REG\n");
		goto out_unlock;
	}
	/* is data new & valid */
	if (!(buf[0] & TSL277X_STA_ADC_VALID)) {
		dev_err(&client->dev,
			"taos_get_lux data not valid\n");
		ret = chip->als_cur_info.lux; /* return LAST VALUE */
		goto out_unlock;
	}

	for (i = 0; i < 4; i++) {
		ret = taos_i2c_read(client,
			(TSL277X_CMD_REG | (TSL277X_ALS_CHAN0LO + i)),
			&buf[i], 1);
		if (ret < 0) {
			dev_err(&client->dev,
				"taos_get_lux failed to read"
				" ret: %x\n", ret);
			goto out_unlock;
		}
	}

	/* clear status, really interrupt status (interrupts are off),
	but we use the bit anyway */
	ret = i2c_smbus_write_byte(client,
		(TSL277X_CMD_REG |
				TSL277X_CMD_SPL_FN |
				TSL277X_CMD_ALS_INT_CLR));

	if (ret < 0) {
		dev_err(&client->dev,
		"taos_i2c_write_command failed in taos_get_lux, err = %d\n",
			ret);
		goto out_unlock; /* have no data, so return failure */
	}

	/* extract ALS/lux data */
	ch0 = (buf[1] << 8) | buf[0];
	ch1 = (buf[3] << 8) | buf[2];

	chip->als_cur_info.als_ch0 = ch0;
	chip->als_cur_info.als_ch1 = ch1;

	if ((ch0 >= chip->als_saturation) || (ch1 >= chip->als_saturation))
		goto return_max;

	if (ch0 == 0) {
		/* have no data, so return LAST VALUE */
		ret = chip->als_cur_info.lux = 0;
		goto out_unlock;
	}
	/* calculate ratio */
	ratio = (ch1 << 15) / ch0;
	/* convert to unscaled lux using the pointer to the table */
	for (p = (struct taos_lux *) taos_device_lux;
	     p->ratio != 0 && p->ratio < ratio; p++)
		;

	if (p->ratio == 0) {
		lux = 0;
	} else {
		ch0lux = ((ch0 * p->ch0) +
		(tsl2771_gainadj[chip->taos_settings.als_gain].ch0 >> 1))
		/ tsl2771_gainadj[chip->taos_settings.als_gain].ch0;

		ch1lux = ((ch1 * p->ch1) +
		(tsl2771_gainadj[chip->taos_settings.als_gain].ch1 >> 1))
		/ tsl2771_gainadj[chip->taos_settings.als_gain].ch1;

		lux = ch0lux - ch1lux;
	}

	/* note: lux is 31 bit max at this point */
	if (ch1lux > ch0lux) {
		dev_dbg(&client->dev, "No Data - Return last value\n");
		ret = chip->als_cur_info.lux = 0;
		goto out_unlock;
	}

	/* adjust for active time scale */
	if (chip->als_time_scale == 0)
		lux = 0;
	else
		lux = (lux + (chip->als_time_scale >> 1)) /
			chip->als_time_scale;

	/* adjust for active gain scale */
	lux >>= 8;  /* tables have factor of 256 built in for accuracy */
	lux = (lux * chip->taos_settings.als_gain_trim + 500) / 1000;
	if (lux > TSL277X_LUX_CALC_OVER_FLOW) { /* check for overflow */
return_max:
		lux = TSL277X_LUX_CALC_OVER_FLOW;
	}

	/* Update the structure with the latest VALID lux. */
	chip->als_cur_info.lux = lux;
	ret = lux;

out_unlock:
	mutex_unlock(&chip->als_mutex);
	return ret;

}

static int SF_lux(struct i2c_client *client){
	int lux = 0;
	struct tsl2771_chip *chip = i2c_get_clientdata(client);

	lux = taos_get_lux(client);
	lux = lux * chip->taos_settings.als_sf / 100;
	return lux;
}

/*
 * Proximity poll function - if valid data is available, read and form the ch0
 * and prox data values, check for limits on the ch0 value, and check the prox
 * data against the current thresholds, to set the event status accordingly.
 */
u16 taos_prox_poll(struct i2c_client *client)
{
#define CONSECUTIVE_RETRIES 50

int i;
int ret;
u8 status;
u8 chdata[2];
int err_cnt;
struct tsl2771_chip *chip = i2c_get_clientdata(client);

	if (mutex_trylock(&chip->prox_mutex) == 0) {
		dev_err(&client->dev, "Can't get prox mutex\n");
	return -1;
	}

	err_cnt = 0;

tryAgain:
		ret = taos_i2c_read(client,
			(TSL277X_CMD_REG | TSL277X_STATUS), &status, 1);
		if (ret < 0) {
			dev_err(&client->dev, "Read regs failed in taos_prox_poll() - A\n");
			mutex_unlock(&chip->prox_mutex);
			return ret;
		}

		/*Prox interrupt asserted*/
		if (((chip->taos_settings.als_interrupt << 4) & CNTL_PROX_INT_ENBL)) {
			printk("%s: (chip->taos_settings.als_interrupt: 0x%x << 4) & CNTL_PROX_INT_ENBL = 1\n",__func__,chip->taos_settings.als_interrupt);
			if (!(status & TSL277X_STA_ADC_VALID)) {
				err_cnt++;
				if (err_cnt > CONSECUTIVE_RETRIES) {
					mutex_unlock(&chip->prox_mutex);
					dev_err(&client->dev, "Consec. retries exceeded\n");
					return chip->prox_cur_info.prox_event;
				}
				goto tryAgain;
			}
		}

		for (i = 0; i < 2; i++) {
			ret = taos_i2c_read(client,
				(TSL277X_CMD_REG |
						(TSL277X_PRX_LO + i)), &chdata[i], 1);
			if (ret < 0) {
				dev_err(&client->dev,
				"Read regs failed in taos_prox_poll() - B\n");
				mutex_unlock(&chip->prox_mutex);
				return ret;
			}
		}

		chip->prox_cur_info.prox_data = (chdata[1]<<8)|chdata[0];
		if(chip->prox_cur_info.prox_data > 1023)
			goto tryAgain;
		chdata[1] = 0;
		chdata[0] = 0;

	mutex_unlock(&chip->prox_mutex);
	return chip->prox_cur_info.prox_data;
}

/*
 * Proximity detect interrupt BH - called when proximity of an object
 * to the sensor is detected (event = 1), or, once detected, it has moved away
 * from the sensor (event = 0).
 * Prox info is stored into the structure "prox_cur_inf",
 * and signal is issued to any waiting user mode threads -
 * which must be (of course) registered to be signaled.
 * This is the 'bottom half' of IRQ.
 */
void taos_prox_adjust_level(struct work_struct *prox)
{
	int ret = 0, i;
	u16 cdelta;
	u8 prox_low[2];
	u8 prox_high[2];
	struct tsl2771_chip *chip
	= container_of(prox,
			struct tsl2771_chip, work_thresh);

	printk(KERN_INFO "PROX INT\n");

	taos_prox_poll(chip->client);

	if (chip->prox_cur_info.prox_data > chip->taos_settings.prox_thres) {
		dev_info(&chip->client->dev,
			"prox_data > prox_thres");
		/*Rail the threshold so we don't keep interrupting*/
		prox_high[0] = 0xFF;
		prox_high[1] = 0xFF;
		for (i = 0; i < 2; i++) {
			ret = i2c_smbus_write_byte_data(chip->client,
				(TSL277X_CMD_REG |
				(TSL277X_PRX_MAXTHRESHLO + i)),
				prox_high[i]);

			if (ret < 0)
				dev_err(&chip->client->dev,
				"FAILED: to update PROX HIGH THRESH (A).\n");
		}

		printk(KERN_INFO "Turn touchscreen OFF\n");

		cdelta = chip->taos_settings.prox_thres - 100;

		if (cdelta < 10)
			cdelta = chip->taos_settings.prox_thres - 1;

		prox_low[0] = (cdelta) & 0xFF;
		prox_low[1] = (cdelta >> 8) & 0xFF;
		for (i = 0; i < 2; i++) {
			i2c_smbus_write_byte_data(chip->client,
				(TSL277X_CMD_REG |
				(TSL277X_PRX_MINTHRESHLO + i)),
				prox_low[i]);

			if (ret < 0)
				dev_err(&chip->client->dev,
				"FAILED: to update the PROX LOW THRESH (B).\n");
			}
		} else if (chip->prox_cur_info.prox_data <
					chip->taos_settings.prox_thres) {
			dev_info(&chip->client->dev,
			"prox_data <= prox_thres");
			printk(KERN_INFO "Turn touchscreen ON\n");
			prox_low[0] = 0x00;
			prox_low[1] = 0x00;
			for (i = 0; i < 2; i++) {
				ret = i2c_smbus_write_byte_data(chip->client,
					(TSL277X_CMD_REG |
					(TSL277X_PRX_MINTHRESHLO + i)),
					prox_low[i]);

			if (ret < 0)
				dev_err(&chip->client->dev,
				"FAILED: to update the PROX LOW THRESH (C).\n");
			}
			/*Lastly we put the high threshold
			back to where we started*/
			prox_high[0] = (chip->taos_settings.prox_thres) & 0xFF;
			prox_high[1] =
				(chip->taos_settings.prox_thres >> 8) & 0xFF;
			for (i = 0; i < 2; i++) {
				ret = i2c_smbus_write_byte_data(chip->client,
					(TSL277X_CMD_REG |
					(TSL277X_PRX_MAXTHRESHLO + i)),
					prox_high[i]);

				if (ret < 0)
					dev_err(&chip->client->dev,
					"FAILED: to update PROX HIGH THRESH (D).\n");
			}

	}

return;
}

/*
 * Readjust the ALS threshold levels, based on the cause of the last ALS
 * interrupt.
 */
void taos_als_adjust_level(struct work_struct *als)
{
	int ret;
	int i;
	u8 als_int_thresh[4];
	unsigned int raw_ch0, raw_ch1, cdelta;

	struct tsl2771_chip *chip
		= container_of(als,
			struct tsl2771_chip, work_thresh);
	taos_get_lux(chip->client);

	dev_err(&chip->client->dev,
	"ALS Irq\n");

	/* re-adjust our upper and lower thresholds */
	raw_ch0 = chip->als_cur_info.als_ch0;
	raw_ch1 = chip->als_cur_info.als_ch1;
	if (raw_ch0 == 0) {
		chip->taos_settings.als_thresh_low = 0;
		chip->taos_settings.als_thresh_high = 1;
	} else if (raw_ch0 < 10) {
		chip->taos_settings.als_thresh_low = raw_ch0 - 1;
		chip->taos_settings.als_thresh_high = raw_ch0;
	} else {
		cdelta = (raw_ch0 * 5) / 100;
		chip->taos_settings.als_thresh_low = raw_ch0 - cdelta;
		chip->taos_settings.als_thresh_high = raw_ch0 + cdelta;
		if (chip->taos_settings.als_thresh_high > 0xFFFF)
			chip->taos_settings.als_thresh_high = 0xFFFF;
	}

	als_int_thresh[0] = (chip->taos_settings.als_thresh_low) & 0xFF;
	als_int_thresh[1] = (chip->taos_settings.als_thresh_low >> 8) & 0xFF;
	als_int_thresh[2] = (chip->taos_settings.als_thresh_high) & 0xFF;
	als_int_thresh[3] = (chip->taos_settings.als_thresh_high >> 8) & 0xFF;

	for (i = 0; i < 4; i++) {
		ret = i2c_smbus_write_byte_data(chip->client,
			(TSL277X_CMD_REG | (TSL277X_ALS_MINTHRESHLO + i)),
			als_int_thresh[i]);
		if (ret < 0) {
			dev_info(&chip->client->dev,
			"FAILED: update the ALS LOW THRESH (B).\n");
		}
	}

return;

}

/*
 * Ambient light transition sense interrupt BH - called when the ambient
 * light falls above or below a band of ambient light. A signal is issued to
 * any waiting user mode threads, and the above band is adjusted up or down?
 * The ALS interrupt filter is initially set to 0x00 when ALS_ON is called, to
 * force the first interrupt, after which it is set to the configured value.
 */
void taos_interrupt_bh(struct work_struct *als)
{
int ret;
int value;
u8  reg_val;

struct tsl2771_chip *chip
	= container_of(als,
		struct tsl2771_chip, work_thresh);

	value = i2c_smbus_read_byte_data(chip->client,
		TSL277X_CMD_REG | TSL277X_STATUS);

	if ((value & STA_PRXINTR) && (chip->taos_settings.als_interrupt & CNTL_PROX_INT_ENBL)) {
		taos_prox_adjust_level(als);
	}

	if ((value & STA_ALSINTR) && (chip->taos_settings.als_interrupt & CNTL_ALS_INT_ENBL)) {
		taos_als_adjust_level(als);
	}

	if (!chip->init_done) {
		/* Maintain the persistence value */
		ret = taos_i2c_read(chip->client,
			(TSL277X_CMD_REG | (TSL277X_PERSISTENCE)),
			&reg_val, 1);
		if (ret < 0)
			dev_info(&chip->client->dev,
			"Failed to get the persistence register value\n");

			reg_val = chip->taos_settings.als_persistence;

		ret = i2c_smbus_write_byte_data(chip->client,
			(TSL277X_CMD_REG | TSL277X_PERSISTENCE), reg_val);
		if (ret < 0)
			dev_info(&chip->client->dev,
			"FAILED: update the persistence (B).\n");
		}

		chip->init_done = 1;

	/* Clear out any initial prox and ALS. */
	ret = i2c_smbus_write_byte(chip->client,
		(TSL277X_CMD_REG |
				TSL277X_CMD_SPL_FN |
				TSL277X_CMD_PROXALS_INTCLR));
	if (ret < 0)
		dev_info(&chip->client->dev,
		"taos_interrupt_bh FAILED to clear irqs: err = %d\n", ret);

	enable_irq(chip->irq_no);

return;
}

/*
 * Provides initial operational parameter defaults.
 * These defaults may be changed through the device's sysfs files.
 */
static void taos_defaults(struct tsl2771_chip *chip)
{
	/* Operational parameters */
	chip->taos_settings.als_time = 50;
	/* must be a multiple of 50mS */
	chip->taos_settings.als_gain = 2;
	/* this is actually an index into the gain table */
	/* assume clear glass as default */
	chip->taos_settings.als_gain_trim = 1000;
	/* default gain trim to account for aperture effects */
	chip->taos_settings.als_cal_target = 130;
	/* Known external ALS reading used for calibration */
	chip->taos_settings.als_thresh_low = 03;
	/* CH0 'low' count to trigger interrupt */
	chip->taos_settings.als_thresh_high = 256;
	/* CH0 'high' count to trigger interrupt */
	chip->taos_settings.als_persistence = 0x13;
	/* Number of 'out of limits' ADC readings */
	chip->taos_settings.als_interrupt = 0x10;
	/* Default interrupt(s) enabled.
	 * 0x00 = none, 0x10 = als, 0x20 = prx 0x30 = bth */
	chip->taos_settings.prox_thres = 512;
	chip->taos_settings.prox_thres_high = prox_h;
	chip->taos_settings.prox_thres_low = prox_l;
	/*default threshold (adjust either manually or with cal routine*/
	chip->taos_settings.prox_max_samples_cal = 100;
	chip->taos_settings.prox_pulse_count = 5;
	chip->taos_settings.als_sf = AVG_SF;
}

/*
 * Obtain single reading and calculate the als_gain_trim
 * (later used to derive actual lux).
 * Return updated gain_trim value.
 */
int taos_als_calibrate(struct i2c_client *client)
{
	struct tsl2771_chip *chip = i2c_get_clientdata(client);
	u8 reg_val;
	unsigned int gain_trim_val;
	int ret;
	int lux_val;

	ret = i2c_smbus_write_byte(client, (TSL277X_CMD_REG | TSL277X_CNTRL));
	if (ret < 0) {
		dev_err(&client->dev,
		"taos_als_calibrate failed to write CNTRL register, ret=%d\n",
			ret);
		return ret;
	}

	reg_val = i2c_smbus_read_byte(client);
	if ((reg_val & (TSL277X_CNTL_ADC_ENBL | TSL277X_CNTL_PWR_ON))
			!= (TSL277X_CNTL_ADC_ENBL | TSL277X_CNTL_PWR_ON)) {
		dev_err(&client->dev,
			"taos_als_calibrate failed: ADC not enabled\n");
		return -1;
	}

	ret = i2c_smbus_write_byte(client, (TSL277X_CMD_REG | TSL277X_CNTRL));
	if (ret < 0) {
		dev_err(&client->dev,
		"taos_als_calibrate failed to write ctrl reg: ret=%d\n",
			ret);
		return ret;
	}
	reg_val = i2c_smbus_read_byte(client);

	if ((reg_val & TSL277X_STA_ADC_VALID) != TSL277X_STA_ADC_VALID) {
		dev_err(&client->dev,
		"taos_als_calibrate failed: STATUS - ADC not valid.\n");
		return -ENODATA;
	}
	lux_val = taos_get_lux(client);
	if (lux_val < 0) {
		dev_err(&client->dev,
		"taos_als_calibrate failed to get lux\n");
		return lux_val;
	}
	gain_trim_val = (unsigned int) (((chip->taos_settings.als_cal_target)
			* chip->taos_settings.als_gain_trim) / lux_val);

	if ((gain_trim_val < 250) || (gain_trim_val > 4000)) {
		dev_err(&client->dev,
		"taos_als_calibrate failed: trim_val of %d is out of range\n",
			gain_trim_val);
		return -ERANGE;
	}
	chip->taos_settings.als_gain_trim = (int) gain_trim_val;

	return (int) gain_trim_val;
}

/*
 * Turn the device on.
 * Configuration must be set before calling this function.
 */
static int taos_chip_on(struct i2c_client *client)
{
	int i;
	int ret = 0;
	u8 *uP;
	u8 utmp;
	int als_count;
	int als_time;
	struct tsl2771_chip *chip = i2c_get_clientdata(client);
	u8 reg_val;

	/* Non calculated parameters */
	chip->taos_config[TSL277X_ALS_MINTHRESHLO] =
		(chip->taos_settings.als_thresh_low) & 0xFF;
	chip->taos_config[TSL277X_ALS_MINTHRESHHI] =
		(chip->taos_settings.als_thresh_low >> 8) & 0xFF;
	chip->taos_config[TSL277X_ALS_MAXTHRESHLO] =
		(chip->taos_settings.als_thresh_high) & 0xFF;
	chip->taos_config[TSL277X_ALS_MAXTHRESHHI] =
		(chip->taos_settings.als_thresh_high >> 8) & 0xFF;
	chip->taos_config[TSL277X_PERSISTENCE] =
		chip->taos_settings.als_persistence;

	chip->taos_config[TSL277X_PRX_COUNT] =
			chip->taos_settings.prox_pulse_count;
	chip->taos_config[TSL277X_PRX_MINTHRESHLO] = 0;
	chip->taos_config[TSL277X_PRX_MAXTHRESHLO] =
			chip->taos_settings.prox_thres;

	/* and make sure we're not already on */
	if (chip->taos_chip_status == TSL277X_CHIP_WORKING) {
		/* if forcing a register update - turn off, then on */
		dev_info(&client->dev, "device is already enabled\n");
		return -EINVAL;
	}

	/* determine als integration regster */
	als_count = (chip->taos_settings.als_time * 100 + 135) / 270;
	if (als_count == 0)
		als_count = 1; /* ensure at least one cycle */

	/* convert back to time (encompasses overrides) */
	als_time = (als_count * 27 + 5) / 10;
	chip->taos_config[TSL277X_ALS_TIME] = 256 - als_count;

	/* Set the gain based on taos_settings struct */
	chip->taos_config[TSL277X_GAIN] =
			(chip->taos_settings.als_gain | (mA25 | DIODE_BOTH));

	/* set chip struct re scaling and saturation */
	chip->als_saturation = als_count * 922; /* 90% of full scale */
	chip->als_time_scale = (als_time + 25) / 50;

	/* TSL277X Specific power-on / adc enable sequence
	 * Power on the device 1st. */
	utmp = TSL277X_CNTL_PWR_ON;
	ret = i2c_smbus_write_byte_data(client,
		TSL277X_CMD_REG | TSL277X_CNTRL, utmp);
	if (ret < 0) {
		dev_err(&client->dev, "taos_chip_on failed on CNTRL reg.\n");
		return -1;
	}

	/* Use the following shadow copy for our delay before enabling ADC.
	 * Write all the registers. */
	for (i = 0, uP = chip->taos_config; i < TSL277X_REG_MAX; i++) {
		ret = i2c_smbus_write_byte_data(client, TSL277X_CMD_REG + i,
						*uP++);
		if (ret < 0) {
			dev_err(&client->dev,
				"taos_chip_on failed on write to reg %d.\n", i);
			return ret;
		}
	}

	msleep(3);
	/* NOW enable the ADC
	 * initialize the desired mode of operation */
	utmp = TSL277X_CNTL_PWR_ON | TSL277X_CNTL_ADC_ENBL;
	ret = i2c_smbus_write_byte_data(client, TSL277X_CMD_REG | TSL277X_CNTRL,
					utmp);
	if (ret < 0) {
		dev_err(&client->dev, "taos_chip_on failed on 2nd CTRL reg.\n");
		return ret;
		}

	chip->taos_chip_status = TSL277X_CHIP_ALS_ENABLE;

	/* If interrupts are enabled */
	chip->init_done = 0;

	if (chip->taos_settings.als_interrupt) {
		dev_info(&client->dev, "Setting Up Interrupt(s)\n");
		/* first time interrupt */
		chip->init_done = 0;

		/*First make sure we have an ALS persistence > 0
		else we'll interrupt continuously.*/
		ret = taos_i2c_read(client,
			(TSL277X_CMD_REG | TSL277X_PERSISTENCE), &reg_val, 1);
	if (ret < 0)
		dev_err(&client->dev,
			"Failed to get the persistence register value\n");

	/*ALS Interrupt after 3 consecutive reading out of range */
	if ((reg_val & 0x0F) == 0) {
		reg_val |= 0x03;
		ret = i2c_smbus_write_byte_data(client,
			(TSL277X_CMD_REG | (TSL277X_PERSISTENCE)), reg_val);

	if (ret < 0)
		dev_err(&client->dev,
		"taos_i2c_write to update the persistance register.\n");
	}

	reg_val = TSL277X_CNTL_PWR_ON;

	if (chip->taos_settings.als_interrupt == 0x10)
		reg_val |= CNTL_ADC_ENBL;

	if (chip->taos_settings.als_interrupt == 0x20)
		reg_val |= CNTL_PROX_DET_ENBL;

	if (chip->taos_settings.als_interrupt == 0x30)
		reg_val |= (CNTL_ADC_ENBL | CNTL_PROX_DET_ENBL);

	reg_val |= chip->taos_settings.als_interrupt;

	ret = i2c_smbus_write_byte_data(client,
		(TSL277X_CMD_REG | TSL277X_CNTRL), reg_val);
	if (ret < 0)
		dev_err(&client->dev,
		"taos_i2c_write to device failed in TAOS_IOCTL_INT_SET.\n");

	/* Clear out any initial als interrupts */
	ret = i2c_smbus_write_byte(client,
		TSL277X_CMD_REG | TSL277X_CMD_SPL_FN |
		TSL277X_CMD_PROXALS_INTCLR);
	if (ret < 0) {
		dev_err(&client->dev,
			"taos_i2c_write_command failed in taos_chip_on\n");
	    return ret;
		}
	}
	return ret;

}

static int taos_chip_off(struct i2c_client *client)
{
	struct tsl2771_chip *chip = i2c_get_clientdata(client);

	/* turn device off */
	chip->taos_chip_status = TSL277X_CHIP_SUSPENDED;
	return i2c_smbus_write_byte_data(client,
		TSL277X_CMD_REG | TSL277X_CNTRL, 0x00);
}

/**
 * Integer Square Root
 * We need an integer version since 1st Floating point is not allowed
 * in driver world, 2nd, cannot count on the devices having a FPU, and
 * 3rd software FP emulation may be excessive.
 */
unsigned long taos_isqrt(unsigned long x)
{
	register unsigned long op, res, one;
	op = x;
	res = 0;

	one = 1 << 30;
	while (one > op)
		one >>= 2;

	while (one != 0) {
		if (op >= res + one) {
			op -= res + one;
			res += one << 1;
		}
		res >>= 1;
		one >>= 2;
	}
	return res;
}

/*
 * Proximity calibration helper function
 * runs through a collection of data samples,
 * sets the min, max, mean, and std dev.
 */
void taos_prox_calculate(u16 *data, int length, struct prox_stat *statP)
{

	int i;
	int min, max, sum, mean;
	unsigned long stddev;
	int tmp;

	if (length == 0)
		length = 1;

	sum = 0;
	min = 0xffff;
	max = 0;
	for (i = 0; i < length; i++) {
		sum += data[i];
		if (data[i] < min)
			min = data[i];
		if (data[i] > max)
			max = data[i];
	}
	mean = sum/length;
	statP->min = min;
	statP->max = max;
	statP->mean = mean;

	sum = 0;
	for (i = 0; i < length; i++) {
		tmp = data[i]-mean;
		sum += tmp * tmp;
	}
	stddev = taos_isqrt((long)sum)/length;
	statP->stddev = stddev;

}


/**
 * Proximity calibration - collects a number of samples, calculates a standard deviation based on the samples, and
 * sets the threshold accordingly.
 * \param	none
 * \return	none
 */
void taos_prox_cal(struct i2c_client *client)
{
u16 prox_history[MAX_SAMPLES_CAL+1];
int i;
struct prox_stat prox_stat_data[2];
struct prox_stat *calP;
struct tsl2771_chip *chip = i2c_get_clientdata(client);
u8 tmp_irq_settings;

	if (chip->taos_settings.prox_max_samples_cal > MAX_SAMPLES_CAL) {
		dev_err(&client->dev,
			"max prox samples cal is too big: %d\n",
			chip->taos_settings.prox_max_samples_cal);
		chip->taos_settings.prox_max_samples_cal = MAX_SAMPLES_CAL;
	}

	/* have to stop to change settings */
	taos_chip_off(chip->client);

	/* Enable proximity detection save just in case prox not wanted yet*/
	tmp_irq_settings = chip->taos_settings.als_interrupt;
	chip->taos_settings.als_interrupt |= CNTL_PROX_INT_ENBL;

	/*turn on device if not already on*/
	taos_chip_on(chip->client);

	/*gather the samples*/
	for (i = 0; i < chip->taos_settings.prox_max_samples_cal; i++) {
		mdelay(15);
		taos_prox_poll(chip->client);
		prox_history[i] = chip->prox_cur_info.prox_data;
		dev_info(&client->dev, "2 i=%d prox data= %d\n",
			i, chip->prox_cur_info.prox_data);

	}

	taos_chip_off(chip->client);

	calP = &prox_stat_data[PROX_STAT_CAL];
	taos_prox_calculate(prox_history,
		chip->taos_settings.prox_max_samples_cal, calP);
	chip->taos_settings.prox_thres = (calP->max << 1) - calP->mean;

	dev_info(&client->dev, " cal min=%d mean=%d max=%d\n",
		calP->min, calP->mean, calP->max);
	dev_info(&client->dev,
		"TAOS: proximity threshold set to %d, basic mode\n",
		chip->taos_settings.prox_thres);

	/* back to the way they were */
	chip->taos_settings.als_interrupt = tmp_irq_settings;
}

/* ---------- Sysfs Interface Functions ------------- */

static ssize_t taos_device_id(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", tsl2771_client->name); 
}

static ssize_t taos_power_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", tsl2771_chip_dev->taos_chip_status);	
}

static ssize_t taos_power_state_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if (value == 0)
		taos_chip_off(tsl2771_chip_dev->client);
	else
		taos_chip_on(tsl2771_chip_dev->client);

	return len;
}

static ssize_t taos_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",
		tsl2771_gainadj[tsl2771_chip_dev->taos_settings.als_gain].ch0);
}

static ssize_t taos_gain_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;
	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	switch (value) {
	case 1:
		tsl2771_chip_dev->taos_settings.als_gain = 0;
		break;
	case 8:
		tsl2771_chip_dev->taos_settings.als_gain = 1;
		break;
	case 16:
		tsl2771_chip_dev->taos_settings.als_gain = 2;
		break;
	case 111:
		tsl2771_chip_dev->taos_settings.als_gain = 3;
		break;
	default:
		dev_err(dev,
			"Invalid Gain Index\n");
		return -EINVAL;
	}

	return len;
}

static ssize_t taos_gain_available_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "1 8 16 111");
}

static ssize_t taos_als_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", tsl2771_chip_dev->taos_settings.als_time);
}

static ssize_t taos_als_time_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if ((value < 50) || (value > 650))
		return -EINVAL;

	if (value % 50)
		return -EINVAL;

	 tsl2771_chip_dev->taos_settings.als_time = value;

	return len;
}

static ssize_t taos_als_time_available_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n",
		"50 100 150 200 250 300 350 400 450 500 550 600 650");
}

static ssize_t taos_als_trim_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", tsl2771_chip_dev->taos_settings.als_gain_trim);
}

static ssize_t taos_als_trim_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if (value)
		tsl2771_chip_dev->taos_settings.als_gain_trim = value;

	return len;
}

static ssize_t taos_als_cal_target_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", tsl2771_chip_dev->taos_settings.als_cal_target);
}

static ssize_t taos_als_cal_target_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if (value)
		tsl2771_chip_dev->taos_settings.als_cal_target = value;

	return len;
}

static ssize_t taos_als_thresh_low_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", tsl2771_chip_dev->taos_settings.als_thresh_low);
}

static ssize_t taos_als_thresh_low_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	tsl2771_chip_dev->taos_settings.als_thresh_low = value;

	return len;
}

static ssize_t taos_als_thresh_high_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", tsl2771_chip_dev->taos_settings.als_thresh_high);
}

static ssize_t taos_als_thresh_high_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	tsl2771_chip_dev->taos_settings.als_thresh_high = value;

	return len;
}

static ssize_t taos_prox_thresh_high_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", tsl2771_chip_dev->taos_settings.prox_thres);
}

static ssize_t taos_prox_thresh_high_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	tsl2771_chip_dev->taos_settings.prox_thres = value;

	return len;
}

/* sampling_frequency AKA persistence in data sheet */
static ssize_t taos_als_persistence_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%02X\n", tsl2771_chip_dev->taos_settings.als_persistence);
}

static ssize_t taos_als_persistence_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	tsl2771_chip_dev->taos_settings.als_persistence = value;

	return len;
}

static ssize_t taos_als_persistence_available_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x00 - 0xFF (0 - 255)\n");
}

static ssize_t taos_lux_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	int lux;

	lux = taos_get_lux(tsl2771_chip_dev->client);
	return sprintf(buf, "%d.0\n", lux);
}

static ssize_t taos_adc_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	int lux;

	lux = taos_get_lux(tsl2771_chip_dev->client);

	return sprintf(buf, "%d,%d\n",
		tsl2771_chip_dev->als_cur_info.als_ch0, tsl2771_chip_dev->als_cur_info.als_ch1);
}

static ssize_t taos_do_calibrate(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if (value == 1)
		taos_als_calibrate(tsl2771_chip_dev->client);

	return len;
}

static ssize_t taos_luxtable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	int offset = 0;

	for (i = 0; i < ARRAY_SIZE(taos_device_lux); i++) {
		offset += sprintf(buf + offset, "%d,%d,%d,",
			taos_device_lux[i].ratio,
			taos_device_lux[i].ch0,
			taos_device_lux[i].ch1);
		if (taos_device_lux[i].ratio == 0) {
			/* We just printed the first "0" entry.
			 * Now get rid of the extra "," and break. */
			offset--;
			break;
		}
	}

	offset += sprintf(buf + offset, "\n");
	return offset;
}

static ssize_t taos_luxtable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	int value[ARRAY_SIZE(taos_device_lux)];
	int n;

	get_options(buf, ARRAY_SIZE(value), value);

	/* We now have an array of ints starting at value[1], and
	 * enumerated by value[0].
	 * We expect each group of three ints is one table entry,
	 * and the last table entry is all 0.
	 */
	n = value[0];
	if ((n % 3) || n < 6 || n > ((ARRAY_SIZE(taos_device_lux) - 1) * 3)) {
		dev_info(dev, "LUX TABLE INPUT ERROR 1 Value[0]=%d\n", n);
		return -EINVAL;
	}

	if ((value[(n - 2)] | value[(n - 1)] | value[n]) != 0) {
		dev_info(dev, "LUX TABLE INPUT ERROR 2 Value[0]=%d\n", n);
		return -EINVAL;
	}

	if (tsl2771_chip_dev->taos_chip_status == TSL277X_CHIP_WORKING)
		taos_chip_off(tsl2771_chip_dev->client);

	/* Zero out the table */
	memset(taos_device_lux, 0, sizeof(taos_device_lux));
	memcpy(taos_device_lux, &value[1], (value[0] * 4));

	taos_chip_on(tsl2771_chip_dev->client);

	return len;
}

static ssize_t taos_do_prox_calibrate(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if (value == 1)
		taos_prox_cal(tsl2771_chip_dev->client);

	return len;
}

static DEVICE_ATTR(illuminance0_name, S_IRUGO, taos_device_id, NULL);
static DEVICE_ATTR(power_state, S_IRUGO | S_IWUSR,
		taos_power_state_show, taos_power_state_store);

static DEVICE_ATTR(illuminance0_calibscale, S_IRUGO | S_IWUSR,
		taos_gain_show, taos_gain_store);
static DEVICE_ATTR(illuminance0_calibscale_available, S_IRUGO,
		taos_gain_available_show, NULL);

static DEVICE_ATTR(illuminance0_integration_time, S_IRUGO | S_IWUSR,
		taos_als_time_show, taos_als_time_store);
static DEVICE_ATTR(illuminance0_integration_time_available, S_IRUGO,
		taos_als_time_available_show, NULL);

static DEVICE_ATTR(illuminance0_calibbias, S_IRUGO | S_IWUSR,
		taos_als_trim_show, taos_als_trim_store);

static DEVICE_ATTR(illuminance0_input_target, S_IRUGO | S_IWUSR,
		taos_als_cal_target_show, taos_als_cal_target_store);

static DEVICE_ATTR(illuminance0_raw, S_IRUGO, taos_adc_show, NULL);

static DEVICE_ATTR(illuminance0_input, S_IRUGO, taos_lux_show, NULL);
static DEVICE_ATTR(illuminance0_calibrate, S_IWUSR, NULL, taos_do_calibrate);
static DEVICE_ATTR(proximity_calibrate, S_IWUSR, NULL, taos_do_prox_calibrate);

static DEVICE_ATTR(illuminance0_lux_table, S_IRUGO | S_IWUSR,
		taos_luxtable_show, taos_luxtable_store);

static DEVICE_ATTR(illuminance0_thresh_falling_value, S_IRUGO | S_IWUSR,
		taos_als_thresh_low_show, taos_als_thresh_low_store);

static DEVICE_ATTR(illuminance0_thresh_rising_value, S_IRUGO | S_IWUSR,
		taos_als_thresh_high_show, taos_als_thresh_high_store);

static DEVICE_ATTR(sampling_frequency, S_IRUGO | S_IWUSR,
		taos_als_persistence_show, taos_als_persistence_store);

static DEVICE_ATTR(sampling_frequency_available, S_IRUGO,
		taos_als_persistence_available_show, NULL);

static DEVICE_ATTR(proximity_thresh_value, S_IRUGO | S_IWUSR,
		taos_prox_thresh_high_show, taos_prox_thresh_high_store);

static struct attribute *sysfs_attrs_ctrl[] = {
	&dev_attr_illuminance0_name.attr,
	&dev_attr_power_state.attr,
	&dev_attr_illuminance0_calibscale.attr,	/* Gain  */
	&dev_attr_illuminance0_calibscale_available.attr,
	&dev_attr_illuminance0_integration_time.attr, /* I time*/
	&dev_attr_illuminance0_integration_time_available.attr,
	&dev_attr_illuminance0_calibbias.attr,	/* trim  */
	&dev_attr_illuminance0_input_target.attr,
	&dev_attr_illuminance0_raw.attr,
	&dev_attr_illuminance0_input.attr,
	&dev_attr_illuminance0_calibrate.attr,
	&dev_attr_illuminance0_lux_table.attr,
	&dev_attr_illuminance0_thresh_falling_value.attr, /* Low T */
	&dev_attr_illuminance0_thresh_rising_value.attr,  /* High T */
	&dev_attr_sampling_frequency.attr,		/* persist*/
	&dev_attr_sampling_frequency_available.attr,
	&dev_attr_proximity_thresh_value.attr,	/*Prox thresh */
	&dev_attr_proximity_calibrate.attr,
	NULL
};

static struct attribute_group tsl2771_attribute_group = {
	.attrs = sysfs_attrs_ctrl,
};

/* PROX SYSFS */
static ssize_t taos_prox_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16	prox_sum=0,prox_data = 0, refer = 0;
	int	prox_count = 0;

tryAgain:
	for(prox_count = 0; prox_count < 5; prox_count++){
		prox_data = taos_prox_poll(tsl2771_client);
		if(prox_data > 0){
			prox_sum += prox_data;
			refer++;
			mdelay(70);
//			printk("%s[%d] %d (%d)\n",__func__,prox_count,prox_data,refer);
		}
	}
	if(refer != 0){
		prox_data = prox_sum/refer;
	}else{
		goto tryAgain;
	}

	return sprintf(buf, "%04d\n", prox_data);
}

static ssize_t taos_prox_dfdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16	prox_sum=0,prox_data = 0, refer = 0;
	int	prox_count = 0;

	for(prox_count = 0; prox_count < 4; prox_count++){
		prox_data = taos_prox_poll(tsl2771_client);
		if(prox_data > 0){
			prox_sum += prox_data;
			refer++;
			mdelay(70);
//			printk("%s[%d] %d (%d)\n",__func__,prox_count,prox_data,refer);
		}
	}
	if(refer != 0){
		prox_data  = prox_sum/refer;
		if (prox_data >= tsl2771_chip_dev->taos_settings.prox_thres_high)
			tsl2771_chip_dev->prox_cur_info.prox_event = 0; //Near
		else if (prox_data <= tsl2771_chip_dev->taos_settings.prox_thres_low){
			//if(taos_get_lux(tsl2771_client) < 20)
			//	tsl2771_chip_dev->prox_cur_info.prox_event = 0; //Near
			//else
				tsl2771_chip_dev->prox_cur_info.prox_event = 1; //Far
		}
	}else{
		tsl2771_chip_dev->prox_cur_info.prox_event = last_prox_event;
	}

	input_report_abs(tsl2771_chip_dev->idev_prox->input, ABS_X, tsl2771_chip_dev->prox_cur_info.prox_event);
	input_sync(tsl2771_chip_dev->idev_prox->input);

	return sprintf(buf, "%hu%hu%d\n", EV_ABS, ABS_X, tsl2771_chip_dev->prox_cur_info.prox_event);
}

static ssize_t taos_prox_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 status;
	int ret = 0;
	ret = taos_i2c_read(tsl2771_client, (TSL277X_CMD_REG | TSL277X_STATUS), &status, 1);
	return sprintf(buf, "0x%x\n", status);
}

static ssize_t taos_prox_cntrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 status;
	int ret = 0;
	ret = taos_i2c_read(tsl2771_client, (TSL277X_CMD_REG | TSL277X_CNTRL), &status, 1);
	return sprintf(buf, "0x%x\n", status);
}

static ssize_t taos_prox_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if( (tsl2771_chip_dev->taos_chip_status == TSL277X_CHIP_PROX_ENABLE) || (tsl2771_chip_dev->taos_chip_status == TSL277X_CHIP_WORKING) )
		return sprintf(buf, " 1\n");
	else
		return sprintf(buf, " 0\n");
}

static ssize_t taos_prox_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;
	int ret = 0,temp=0;
	u8 data;
	uint32_t param1 = 0, param2 = 0;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;
	if(first == 1){
		/******* NV item - PROX *******/
		param1 = 2499;
		param2 = 2;
		ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
		if(ret != 0){
			printk("[%d] %s\n",__LINE__,__func__);
			prox_l = 400;
			prox_h = 650;
		}else{
			prox_l = ascii_to_int(param2);
			param2 = 3;
			ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
			prox_h = ascii_to_int(param2);
			printk("prox_h: %d, prox_l: %d\n",prox_h,prox_l);
			if(prox_l < 300 || prox_l > 800){
				printk("[ proximity ] get threshold_low(%d) nv item failed\n",prox_l);
				prox_l = 400;
			}
			if(prox_h > 800 || prox_h < 400){
				printk("[ proximity ] get threshold_high(%d) nv item failed\n",prox_h);
				prox_h = 650;
			}
			if(prox_l > prox_h){
				printk("[ proximity ] get threshold_high(%d) nv item failed\n",prox_h);
				temp = prox_h;
				prox_h = prox_l;
				prox_l = temp;
			}
		}

		/******* NV item - ALS *******/
		param1 = 2500;
		param2 = 0;
		ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
		if(ret != 0){
			printk("[%d] %s\n",__LINE__,__func__);
			als_sf0 = 1;
			als_sf1 = 1;
			als_sf2 = 1;
			AVG_SF = 100;
		}else{
			als_sf0 = 55*100/ascii_to_int(param2);
			param2 = 1;
			ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
			als_sf1 = 255*100/ascii_to_int(param2);
			param2 = 2;
			ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
			als_sf2 = 700*100/ascii_to_int(param2);
			if(als_sf0 < 100 || als_sf1 < 100 || als_sf2 < 100){
				als_sf0 = 1;
				als_sf1 = 1;
				als_sf2 = 1;
				AVG_SF = 100;
			}else{
				AVG_SF = (als_sf0 + als_sf1 + als_sf2)/3;
			}
		}
		tsl2771_chip_dev->taos_settings.prox_thres_high = prox_h;
		tsl2771_chip_dev->taos_settings.prox_thres_low = prox_l;
		tsl2771_chip_dev->taos_settings.als_sf  = AVG_SF;
		first = 0;
	}else{
		if(value == 1){
			data = 0x37;
			ret = i2c_smbus_write_byte_data(tsl2771_client, TSL277X_CMD_REG | TSL277X_CNTRL, data);
			if(tsl2771_chip_dev->taos_chip_status == TSL277X_CHIP_SUSPENDED)
				tsl2771_chip_dev->taos_chip_status = TSL277X_CHIP_WORKING;
			else if(tsl2771_chip_dev->taos_chip_status == TSL277X_CHIP_ALS_ENABLE)
				tsl2771_chip_dev->taos_chip_status = TSL277X_CHIP_WORKING;
		}else{
			data = 0x13;
			ret = i2c_smbus_write_byte_data(tsl2771_client, TSL277X_CMD_REG | TSL277X_CNTRL, data);
			if(tsl2771_chip_dev->taos_chip_status == TSL277X_CHIP_SUSPENDED)
				tsl2771_chip_dev->taos_chip_status = TSL277X_CHIP_SUSPENDED;
			else if(tsl2771_chip_dev->taos_chip_status == TSL277X_CHIP_WORKING)
				tsl2771_chip_dev->taos_chip_status = TSL277X_CHIP_ALS_ENABLE;
		}
	}

	return len;
}

static ssize_t taos_register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	u8 data;
	int i;

	for(i=0;i<26;i++){
	ret = taos_i2c_read(tsl2771_client, (TSL277X_CMD_REG | i), &data, 1);
	if (ret < 0) {
		dev_err(&tsl2771_client->dev, "taos_prox_enable_store failed to read CNTRL reg.\n");
	}
	printk("register[ 0x%02x]: %x\n",i,data);
		}
	return sprintf(buf, "%d\n",ret);
}

static ssize_t taos_prox_pulse_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	u8 data;
	ret = taos_i2c_read(tsl2771_client, (TSL277X_CMD_REG | TSL277X_PRX_COUNT), &data, 1);
	if (ret < 0) {
		dev_err(&tsl2771_client->dev, "taos_prox_enable_store failed to read PRX_COUNT reg.\n");
	}
	return sprintf(buf, "%d\n",data);
}

static ssize_t taos_prox_pulse_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;
	int ret = 0;
	u8 data;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;


		ret = taos_i2c_read(tsl2771_client, (TSL277X_CMD_REG | TSL277X_PRX_COUNT), &data, 1);
		if (ret < 0) {
			dev_err(&tsl2771_client->dev, "taos_prox_enable_store failed to read PRX_COUNT reg.\n");
		}
		printk("##### value_before: %d\n",data);
		data = value;
		printk("##### value: %d\n",data);
		ret = i2c_smbus_write_byte_data(tsl2771_client, TSL277X_CMD_REG | TSL277X_PRX_COUNT, data);
		if (ret < 0) {
			dev_err(&tsl2771_client->dev, "taos_prox_enable_store failed on PRX_COUNT reg.\n");
		}

	return len;
}

static ssize_t taos_pdrive_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	u8 data;
	ret = taos_i2c_read(tsl2771_client, (TSL277X_CMD_REG | TSL277X_GAIN), &data, 1);
	if (ret < 0) {
		dev_err(&tsl2771_client->dev, "taos_prox_enable_store failed to read GAIN reg.\n");
	}
	return sprintf(buf, "%d\n",data);
}

static ssize_t taos_pdrive_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;
	int ret = 0;
	u8 data;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;


		ret = taos_i2c_read(tsl2771_client, (TSL277X_CMD_REG | TSL277X_GAIN), &data, 1);
		if (ret < 0) {
			dev_err(&tsl2771_client->dev, "taos_prox_enable_store failed to read GAIN reg.\n");
		}
		printk("##### value_before: %d\n",data);
		data = value;
		printk("##### value: %d\n",data);
		ret = i2c_smbus_write_byte_data(tsl2771_client, TSL277X_CMD_REG | TSL277X_GAIN, data);
		if (ret < 0) {
			dev_err(&tsl2771_client->dev, "taos_prox_enable_store failed on GAIN reg.\n");
		}

	return len;
}

static ssize_t taos_prox_thres_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "high: %d, low: %d\n",tsl2771_chip_dev->taos_settings.prox_thres_high,tsl2771_chip_dev->taos_settings.prox_thres_low);
}

static DEVICE_ATTR(prox_data, 0655,taos_prox_data_show, NULL);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, taos_prox_enable_show, taos_prox_enable_store);
static DEVICE_ATTR(dfdata, 0655,taos_prox_dfdata_show, NULL);
static DEVICE_ATTR(status, 0655,taos_prox_status_show, NULL);
static DEVICE_ATTR(cntrl, 0655,taos_prox_cntrl_show, NULL);
static DEVICE_ATTR(register, 0655,taos_register_show, NULL);
static DEVICE_ATTR(pulse, 0655,taos_prox_pulse_show, taos_prox_pulse_store);
static DEVICE_ATTR(pdrive, 0655,taos_pdrive_show, taos_pdrive_store);
static DEVICE_ATTR(prox_thres, 0655,taos_prox_thres_show, NULL);

static struct attribute *prox_attrs_ctrl[] = {
	&dev_attr_prox_data.attr,
	&dev_attr_enable.attr,
	&dev_attr_dfdata.attr,
	&dev_attr_cntrl.attr,
	&dev_attr_status.attr,
	&dev_attr_pulse.attr,
	&dev_attr_register.attr,
	&dev_attr_pdrive.attr,
	&dev_attr_prox_thres.attr,
	NULL
};

static struct attribute_group prox_attribute_group = {
	.attrs = prox_attrs_ctrl,
};

/* ALS SYSFS */
static ssize_t taos_cal_lux_show(struct device *dev, struct device_attribute *attr,	char *buf)
{
	int lux;

	lux = taos_get_lux(tsl2771_chip_dev->client);
	return sprintf(buf, "%04d\n", lux);
}

static ssize_t taos_als_data_show(struct device *dev, struct device_attribute *attr,	char *buf)
{
	int lux;

	lux = SF_lux(tsl2771_chip_dev->client);
	return sprintf(buf, "%d.0\n", lux);
}

static ssize_t taos_als_sf_show(struct device *dev, struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "[ %d, %d, %d] %d\n", als_sf0, als_sf1, als_sf2, tsl2771_chip_dev->taos_settings.als_sf);
}

static ssize_t taos_als_sf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	tsl2771_chip_dev->taos_settings.als_sf = value;
	printk(" taos_settings.als_sf: %d, AVG_SF: %d\n",tsl2771_chip_dev->taos_settings.als_sf,AVG_SF);
	return len;
}

static ssize_t taos_nv_item_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "prox_cal [ %d, %d]; als_cal [ %d]\n", tsl2771_chip_dev->taos_settings.prox_thres_high, tsl2771_chip_dev->taos_settings.prox_thres_low, tsl2771_chip_dev->taos_settings.als_sf);
}

static ssize_t taos_nv_item_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
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
			prox_l = 400;
			prox_h = 650;
		}else{
			prox_l = ascii_to_int(param2);
			param2 = 3;
			ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
			prox_h = ascii_to_int(param2);
			printk("prox_h: %d, prox_l: %d\n",prox_h,prox_l);
			if(prox_l < 300 || prox_l > 800){
				printk("[ proximity ] get threshold_low(%d) nv item failed\n",prox_l);
				prox_l = 400;
			}
			if(prox_h > 800 || prox_h < 400){
				printk("[ proximity ] get threshold_high(%d) nv item failed\n",prox_h);
				prox_h = 650;
			}
			if(prox_l > prox_h){
				printk("[ proximity ] get threshold_high(%d) nv item failed\n",prox_h);
				temp = prox_h;
				prox_h = prox_l;
				prox_l = temp;
			}
		}

		/******* NV item - ALS *******/
		param1 = 2500;
		param2 = 0;
		ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
		if(ret != 0){
			printk("[%d] %s\n",__LINE__,__func__);
			als_sf0 = 1;
			als_sf1 = 1;
			als_sf2 = 1;
			AVG_SF = 100;
		}else{
			als_sf0 = 55*100/ascii_to_int(param2);
			param2 = 1;
			ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
			als_sf1 = 255*100/ascii_to_int(param2);
			param2 = 2;
			ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
			als_sf2 = 700*100/ascii_to_int(param2);
			if(als_sf0 < 100 || als_sf1 < 100 || als_sf2 < 100){
				als_sf0 = 1;
				als_sf1 = 1;
				als_sf2 = 1;
				AVG_SF = 100;
			}else{
				AVG_SF = (als_sf0 + als_sf1 + als_sf2)/3;
			}
		}
		tsl2771_chip_dev->taos_settings.prox_thres_high = prox_h;
		tsl2771_chip_dev->taos_settings.prox_thres_low = prox_l;
		tsl2771_chip_dev->taos_settings.als_sf  = AVG_SF;
	}
	printk("prox threshold [ %d, %d]; als SF [ %d]\n", tsl2771_chip_dev->taos_settings.prox_thres_high, tsl2771_chip_dev->taos_settings.prox_thres_low, tsl2771_chip_dev->taos_settings.als_sf);

	return len;
}
static DEVICE_ATTR(cal_data, 0655, taos_cal_lux_show, NULL);
static DEVICE_ATTR(data, 0655, taos_als_data_show, NULL);
static DEVICE_ATTR(AVG_SF, 0655, taos_als_sf_show, taos_als_sf_store);
static DEVICE_ATTR(nv_item, S_IRUGO | S_IWUSR, taos_nv_item_show, taos_nv_item_store);

static struct attribute *als_attrs_ctrl[] = {
	&dev_attr_cal_data.attr,
	&dev_attr_data.attr,
	&dev_attr_AVG_SF.attr,
	&dev_attr_nv_item.attr,
	NULL
};

static struct attribute_group als_attribute_group = {
	.attrs = als_attrs_ctrl,
};

/* PS */
static void taos_poll_prox(struct input_polled_dev *idev_prox)
{
	u16	prox_sum=0,prox_data = 0, refer = 0;
	int	prox_count = 0;

	if(tsl2771_chip_dev->taos_chip_status == TSL277X_CHIP_WORKING){
		for(prox_count = 0; prox_count < 4; prox_count++){
			prox_data = taos_prox_poll(tsl2771_client);
			if(prox_data > 0){
				prox_sum += prox_data;
				refer++;
				udelay(100);
				printk("%s[%d] %d (%d)\n",__func__,prox_count,prox_data,refer);
			}
		}
		if(refer != 0){
			prox_data = prox_sum/refer;
			if (prox_data >= tsl2771_chip_dev->taos_settings.prox_thres_high)
				tsl2771_chip_dev->prox_cur_info.prox_event = 0; //Near
			else if (prox_data <= tsl2771_chip_dev->taos_settings.prox_thres_low){
				//if(taos_get_lux(tsl2771_client) < 20)
				//	tsl2771_chip_dev->prox_cur_info.prox_event = 0; //Near
				//else
					tsl2771_chip_dev->prox_cur_info.prox_event = 1; //Far
			}
		}else{
			tsl2771_chip_dev->prox_cur_info.prox_event = last_prox_event;
		}

		if( tsl2771_chip_dev->prox_cur_info.prox_event != last_prox_event ){
			input_report_abs(idev_prox->input, ABS_X, tsl2771_chip_dev->prox_cur_info.prox_event);
			input_sync(idev_prox->input);
		}
		last_prox_event = tsl2771_chip_dev->prox_cur_info.prox_event;
	}
}

static int taos_suspend(struct i2c_client *client, pm_message_t state)
{
	struct tsl2771_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&chip->als_mutex);

	if ((chip->taos_chip_status == TSL277X_CHIP_WORKING) || (chip->taos_chip_status == TSL277X_CHIP_ALS_ENABLE) || (chip->taos_chip_status == TSL277X_CHIP_PROX_ENABLE)) {
		ret = taos_chip_off(client);
		chip->taos_chip_status = TSL277X_CHIP_SUSPENDED;
	}

	mutex_unlock(&chip->als_mutex);
	return ret;
}

static int taos_resume(struct i2c_client *client)
{
	struct tsl2771_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&chip->als_mutex);

	if (chip->taos_chip_status == TSL277X_CHIP_SUSPENDED)
		ret = taos_chip_on(client);

	mutex_unlock(&chip->als_mutex);
	return ret;
}


/* Use the default register values to identify the Taos device */
static int taos_TSL277X_device(unsigned char *bufp)
{
	return (bufp[TSL277X_CHIPID] == 0x29);
}

/*
 * Client probe function - When a valid device is found, the driver's device
 * data structure is updated, and initialization completes successfully.
 */
static int __devinit taos_probe(struct i2c_client *clientp,
	const struct i2c_device_id *idp)
{
	int i, ret = 0;
	unsigned char buf[TSL277X_MAX_DEVICE_REGS];
	static struct tsl2771_chip *chip;
	struct input_dev *input_device;

	if (!i2c_check_functionality(clientp->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&clientp->dev,
			"taos_probe() - i2c smbus byte data "
			"functions unsupported\n");
		return -EOPNOTSUPP;
	}

	chip = kzalloc(sizeof(struct tsl2771_chip), GFP_KERNEL);

	chip->client = clientp;
	i2c_set_clientdata(clientp, chip);
	tsl2771_client = chip->client;

	mutex_init(&chip->prox_mutex);
	mutex_init(&chip->als_mutex);

	chip->taos_chip_status = TSL277X_CHIP_UNKNOWN;
	memcpy(chip->taos_config, tsl2771_taos_config,
		sizeof(chip->taos_config));

	for (i = 0; i < TSL277X_MAX_DEVICE_REGS; i++) {
		ret = i2c_smbus_write_byte(clientp,
				(TSL277X_CMD_REG | (TSL277X_CNTRL + i)));
		if (ret < 0) {
			dev_err(&clientp->dev, "i2c_smbus_write_bytes() to cmd "
				"reg failed in taos_probe(), err = %d\n", ret);
			goto fail1;
		}
		ret = i2c_smbus_read_byte(clientp);
		if (ret < 0) {
			dev_err(&clientp->dev, "i2c_smbus_read_byte from "
				"reg failed in taos_probe(), err = %d\n", ret);

			goto fail1;
		}
		buf[i] = ret;
	}

	if (!taos_TSL277X_device(buf)) {
		dev_info(&clientp->dev, "i2c device found but does not match "
			"expected id in taos_probe()\n");
		goto fail1;
	}

	ret = i2c_smbus_write_byte(clientp, (TSL277X_CMD_REG | TSL277X_CNTRL));
	if (ret < 0) {
		dev_err(&clientp->dev, "i2c_smbus_write_byte() to cmd reg "
			"failed in taos_probe(), err = %d\n", ret);
		goto fail1;
	}

	chip->client->driver->suspend = taos_suspend;
	chip->client->driver->resume = taos_resume;

/* PROX */
	chip->idev_prox = input_allocate_polled_device();
	if(!chip->idev_prox){
		printk("%s: input_allocate_polled_dev idev_PROX fail\n",__func__);
		ret = -ENOMEM;
		goto exit_allocate_prox;
	}

	chip->idev_prox->private = chip;
	chip->idev_prox->poll = taos_poll_prox;
	chip->idev_prox->poll_interval= 250;

	input_device = chip->idev_prox->input;
	input_device->name = "proximity";
	input_device->id.bustype = BUS_HOST;
	input_device->dev.parent = &chip->client->dev;
	
	set_bit(EV_ABS, input_device->evbit);
	input_set_abs_params(input_device, ABS_X, 0, 65535, 0, 0);
	
/* ALS */
	chip->idev_als = input_allocate_device();
	if(!chip->idev_als){
		printk("%s: input_allocate_polled_dev idev_ALS fail\n",__func__);
		ret = -ENOMEM;
		goto exit_allocate_als;
	}

	input_device = chip->idev_als;
	input_device->name = "light";
	input_device->id.bustype = BUS_HOST;
	input_device->dev.parent = &chip->client->dev;
	
	set_bit(EV_ABS, input_device->evbit);
	input_set_abs_params(input_device, ABS_Y, 0, 65535, 0, 0);

	tsl2771_chip_dev = chip;

	ret = input_register_polled_device(tsl2771_chip_dev->idev_prox);
	if (ret) {
		printk("taos_probe: Unable to register input device: %s\n", tsl2771_chip_dev->idev_prox->input->name);
		ret = -ENOMEM;
		goto exit_register_prox;
	}
	
	ret = input_register_device(tsl2771_chip_dev->idev_als);
	if (ret) {
		printk("taos_probe: Unable to register input device: %s\n", tsl2771_chip_dev->idev_als->name);
		ret = -ENOMEM;
		goto exit_register_als;
	}

	ret = sysfs_create_group(&tsl2771_chip_dev->idev_prox->input->dev.kobj, &prox_attribute_group);
	if (ret)
		printk("%s: %d idev_prox sysfs_create_group fail\n",__func__,__LINE__);

	ret = sysfs_create_group(&tsl2771_chip_dev->idev_als->dev.kobj, &als_attribute_group);
	if (ret)
		printk("%s: %d idev_als sysfs_create_group fail\n",__func__,__LINE__);

	/* Load up the defaults
	 * (these are can be changed in the device[x]/ABI) */
	taos_defaults(chip);

	INIT_WORK(&chip->work_thresh, taos_interrupt_bh);

	/* Make sure the chip is on */
	taos_chip_on(clientp);
	
	ret = sysfs_create_group(&clientp->dev.kobj, &tsl2771_attribute_group);
	if (ret)
		printk("%s: %d sysfs_create_group fail\n",__func__,__LINE__);				
	dev_info(&clientp->dev, "Light sensor found.\n");

	return 0;

exit_register_als:
	input_unregister_device(chip->idev_als);
exit_register_prox:
	input_unregister_polled_device(chip->idev_prox);
exit_allocate_als:
	input_free_device(chip->idev_als);
exit_allocate_prox:
	input_free_polled_device(chip->idev_prox);

fail1:
	tsl2771_client = NULL;
	tsl2771_chip_dev = NULL;
	kfree(chip);

	return ret;
}

static int __devexit taos_remove(struct i2c_client *client)
{
	struct tsl2771_chip *chip = i2c_get_clientdata(client);

	taos_chip_off(client);

	if (chip->irq_no)
		free_irq(chip->irq_no, chip->client->name);

	flush_scheduled_work();

	input_unregister_device(chip->idev_als);
	input_free_device(chip->idev_als);
	input_unregister_polled_device(chip->idev_prox);
	input_free_polled_device(chip->idev_prox);
	tsl2771_client = NULL;
	tsl2771_chip_dev = NULL;

	kfree(chip);
	return 0;
}

static struct i2c_device_id taos_idtable[] = {
	{ "tsl2771", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, taos_idtable);

/* Driver definition */
static struct i2c_driver taos_driver = {
	.driver = {
		.name = "tsl2771",
	},
	.id_table = taos_idtable,
	.suspend	= taos_suspend,
	.resume	= taos_resume,
	.probe	= taos_probe,
	.remove	= __devexit_p(taos_remove),
};

static int __init taos_init(void)
{
	return i2c_add_driver(&taos_driver);
}

static void __exit taos_exit(void)
{
	i2c_del_driver(&taos_driver);
}

module_init(taos_init);
module_exit(taos_exit);

MODULE_AUTHOR("J. August Brenner<jbrenner@taosinc.com>");
MODULE_DESCRIPTION("TAOS tsl2771 ambient light sensor driver");
MODULE_LICENSE("GPL");

