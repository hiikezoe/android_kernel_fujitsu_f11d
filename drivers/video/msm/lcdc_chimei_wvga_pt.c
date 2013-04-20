/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2012
/*----------------------------------------------------------------------------*/

#include <linux/delay.h>
#include <linux/pwm.h>
#ifdef CONFIG_SPI_QSD
#include <linux/spi/spi.h>
#endif
#include <mach/gpio.h>
#include "msm_fb.h"
#include <linux/earlysuspend.h>
#include <linux/jiffies.h>
#include <linux/i2c/is3_power.h>
extern int qci_read_hw_id(void);

#ifdef CONFIG_SPI_QSD
#define LCDC_CHIMEI_SPI_DEVICE_NAME	"lcdc_chimei_hx8363"
static struct spi_device *lcdc_spi_client;
#endif

//#define DEBUG
#define SYSFS_DEBUG_CMD
int SPI_SCLK ;
int SPI_CS ;
int SPI_MOSI ;
static int lcdc_chimei_panel_off(struct platform_device *pdev);
static void brightness_control(int brightness);
extern int set_spi_gpio_exclusive_ctrl(int runtype, int csno);
int pre_brightness = -1;
int cur_brightness = 0;
#define LIM_BRIGHTNESS	11

#if 0
static bool spi_status = FALSE;
static void report_spi_status(bool spi_request)
{
	int ret = 0;
	if((spi_status == FALSE) && (spi_request == TRUE))	//	off -> on
	{
		ret = set_spi_gpio_exclusive_ctrl(1,1);
		printk("spi status off->on \n");
		spi_status = TRUE;
	}
	else if((spi_status == TRUE) && (spi_request == FALSE))
	{
		ret = set_spi_gpio_exclusive_ctrl(0,1);
		printk("spi status on->off\n");
		spi_status = FALSE;
	}
	else
	{
		printk("spi status not change\n");
	}

	if(ret != 0)
	{
		printk("set_spi_gpio_exclusive_ctrl() = %d\n",ret);
	}
}
#endif


struct chimei_state_type {
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
	unsigned long sleep_out_time;
	unsigned long sleep_in_time;
};

struct chimei_spi_data {
	u8 addr;
	u8 len;
	u8 data[32];
};

#if 1
static struct chimei_spi_data display_sequence_1[] = {
	{ .addr = 0xB9, .len = 3, .data = { 0xFF, 0x83, 0x63 } },
	{ .addr = 0xB1, .len = 12, .data = { 0x81, 0x30, 0x07, 0x30,  0x02, 0x13, 0x11, 0x11, 0x35, 0x3D, 0x3F, 0x3F} },
};
static struct chimei_spi_data display_sequence_2[] = {
	{ .addr = 0x36, .len = 1, .data = { 0x08 } },
	{ .addr = 0x3A, .len = 1, .data = { 0x70 } },
	{ .addr = 0xB3, .len = 1, .data = { 0x09 } },
	{ .addr = 0xB4, .len = 11, .data = { 0x08, 0x12, 0x72, 0x12,  0x06, 0x03, 0x54, 0x03, 0x4E, 0x00, 0x00 } },
	{ .addr = 0xBF, .len = 2, .data = { 0x00, 0x10 } },
	{ .addr = 0xB6, .len = 1, .data = { 0x00 } },
	{ .addr = 0xCC, .len = 1, .data = { 0x0A } },
	{ .addr = 0xE0, .len = 30, .data = { 0x01, 0x1f, 0x28, 0x38, 0x3e, 0x3f, 0x07, 0x8d,  0xce, 0x91, 0x54, 0x12,0x14, 0x8f, 0x12, 0x01, 0x1f, 0x28, 0x38, 0x3e,  0x3f, 0x07, 0x8d, 0xce, 0x91, 0x54, 0x12, 0x14, 0x8f, 0x12} },
	{ .addr = 0x36, .len = 1, .data = { 0x0b } },
	{ .addr = 0x0b, .len = 1, .data = { 0x0b } },

	// CABC setting
	{ .addr = 0xC9, .len = 3, .data = { 0x0F, 0x3E, 0x01 } },
	{ .addr = 0x51, .len = 1, .data = { 0xFF } },
	{ .addr = 0x52, .len = 1, .data = { 0x2C } },
	{ .addr = 0x55, .len = 1, .data = { 0x00 } },

};
#else
static struct chimei_spi_data display_sequence_1[] = {
	{ .addr = 0xf3, .len = 1, .data = { 0x80 } },
	{ .addr = 0xf1, .len = 1, .data = { 0x5A } },
	{ .addr = 0xff, .len = 4, .data = { 0x00, 0x00, 0x00, 0x40 } },
};
static struct chimei_spi_data display_sequence_2[] = {
	{ .addr = 0x3a, .len = 1, .data = { 0x77 } },
	{ .addr = 0xfd, .len = 2, .data = { 0x22, 0x01 } },
	{ .addr = 0xf6, .len = 2, .data = { 0xc0, 0x81 } },
	{ .addr = 0xf1, .len = 1, .data = { 0x00 } },
	{ .addr = 0xf2, .len = 11, .data = { 0x10, 0x10, 0x03, 0x02, 0x03, 0x02, 0x03, 0x10, 0x06, 0x15, 0x15 } },
	{ .addr = 0xf3, .len = 8, .data = { 0x7f, 0x00, 0x00, 0x09, 0x33, 0x75, 0x70, 0x2c } },
	{ .addr = 0xf4, .len = 5, .data = { 0x60, 0x60, 0x73, 0x73, 0x33, } },
	{ .addr = 0xf5, .len = 5, .data = { 0x12, 0x00, 0x03, 0xf0, 0x70, } },
	{ .addr = 0xf7, .len = 15, .data = { 0x80, 0x31, 0x03, 0x0a, 0x1e, 0x26, 0x2f, 0x36, 0x0c, 0x12, 0x2a, 0x21, 0x07, 0x22, 0x22 } },
	{ .addr = 0xf8, .len = 15, .data = { 0x80, 0x00, 0x03, 0x14, 0x2f, 0x30, 0x31, 0x38, 0x06, 0x00, 0x0a, 0x00, 0x00, 0x22, 0x22 } },
	{ .addr = 0xf9, .len = 15, .data = { 0x8a, 0x31, 0x00, 0x14, 0x25, 0x2f, 0x37, 0x3f, 0x00, 0x00, 0x1f, 0x1d, 0x04, 0x22, 0x22 } },
	{ .addr = 0xfa, .len = 15, .data = { 0x80, 0x0b, 0x00, 0x0a, 0x35, 0x36, 0x39, 0x3f, 0x00, 0x00, 0x0a, 0x03, 0x00, 0x22, 0x22 } },
	{ .addr = 0xfb, .len = 15, .data = { 0x8a, 0x31, 0x03, 0x0e, 0x1e, 0x2f, 0x34, 0x3f, 0x00, 0x00, 0x24, 0x1e, 0x07, 0x22, 0x22 } },
	{ .addr = 0xfc, .len = 15, .data = { 0x80, 0x18, 0x03, 0x14, 0x38, 0x3c, 0x3c, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22 } },
	//{ .addr = 0x36, .len = 1, .data = { 0x08 } },
	{ .addr = 0x36, .len = 1, .data = { 0x09 } },
	{ .addr = 0x0b, .len = 1, .data = { 0x09 } },
	{ .addr = 0x3a, .len = 1, .data = { 0x77 } },
	{ .addr = 0xb3, .len = 1, .data = { 0x0e } },

	// CABC setting
	{ .addr = 0xC9, .len = 3, .data = { 0x0F, 0x3E, 0x01 } },
	{ .addr = 0x51, .len = 1, .data = { 0xFF } },
	{ .addr = 0x52, .len = 1, .data = { 0x2C } },
	{ .addr = 0x55, .len = 1, .data = { 0x01 } },

};
#endif

static struct chimei_spi_data gamma_sequence_level15[] = {		//	brightness:100%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0xff } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level14[] = {		//	brightness:93.33%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0xee } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level13[] = {		//	brightness:86.66%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0xdd } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },

};

static struct chimei_spi_data gamma_sequence_level12[] = {		//	brightness:80%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0xcc } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level11[] = {		//	brightness:73.33%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0xbb } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level10[] = {		//	brightness: 66.66%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0xaa } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level9[] = {		//	brightness:60%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0x99 } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level8[] = {		//	brightness:53.33%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0x88 } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level7[] = {		//	brightness:46.66%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0x77 } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level6[] = {		//	brightness:40%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0x66 } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level5[] = {		//	brightness:33.33%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0x55 } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level4[] = {		//	brightness:26.66%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0x44 } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level3[] = {		//	brightness:20%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0x33 } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level2[] = {		//	brightness:13.33%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0x22 } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level1[] = {		//	brightness:6.66%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0x11 } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_spi_data gamma_sequence_level0[] = {		//	brightness: 0%
	{ .addr = 0x53, .len = 1, .data = { 0x24 } },
	{ .addr = 0x51, .len = 1, .data = { 0x00 } },
	{ .addr = 0x53, .len = 1, .data = { 0x2c } },
};

static struct chimei_state_type chimei_state = { 0 };
static struct msm_panel_common_pdata *lcdc_chimei_pdata;

#ifdef DEBUG
static const char *byte_to_binary(const u8 *buf, int len)
{
	static char b[32*8+1];
	char *p = b;
	int i, z;

	for (i = 0; i < len; ++i) {
		u8 val = *buf++;
		for (z = 1 << 7; z > 0; z >>= 1)
			*p++ = (val & z) ? '1' : '0';
	}
	*p = 0;

	return b;
}
#endif
#define BIT_OFFSET	(bit_size % 8)
#define ADD_BIT(val) do { \
		tx_buf[bit_size / 8] |= \
			(u8)((val ? 1 : 0) << (7 - BIT_OFFSET)); \
		++bit_size; \
	} while (0)

#define ADD_BYTE(data) do { \
		tx_buf[bit_size / 8] |= (u8)(data >> BIT_OFFSET); \
		bit_size += 8; \
		if (BIT_OFFSET != 0) \
			tx_buf[bit_size / 8] |= (u8)(data << (8 - BIT_OFFSET));\
	} while (0)
static void tianma_spi_write_byte(char dc, uint8 data)
{
	uint32 bit;
	int bnum;
	static spinlock_t lock;
	spin_lock_init(&lock);
        spin_lock(&lock);
	gpio_set_value_cansleep(SPI_SCLK, 0); /* clk low */
	udelay(1);
	/* dc: 0 for command, 1 for parameter */
	gpio_set_value_cansleep(SPI_MOSI, dc);
	udelay(1);
	gpio_set_value_cansleep(SPI_SCLK, 1); /* clk high */
	udelay(1);	/* at least 20 ns */
	bnum = 8;	/* 8 data bits */
	bit = 0x80;
	while (bnum) {
		gpio_set_value_cansleep(SPI_SCLK, 0); /* clk low */
		udelay(1);
		if (data & bit)
			gpio_set_value_cansleep(SPI_MOSI, 1);
		else
			gpio_set_value_cansleep(SPI_MOSI, 0);
		udelay(1);
		gpio_set_value_cansleep(SPI_SCLK, 1); /* clk high */
		udelay(1);
		bit >>= 1;
		bnum--;
	}
	spin_unlock(&lock);
}

static int tianma_spi_write(char cmd, uint32 data, int num)
{
	char *bp;

	gpio_set_value_cansleep(SPI_CS, 0);	/* cs high */

	/* command byte first */
	if(cmd)
		tianma_spi_write_byte(0, cmd);

	/* followed by parameter bytes */
	if (num) {
		bp = (char *)&data;;
		bp += (num - 1);
		while (num) {
			tianma_spi_write_byte(1, *bp);
			num--;
			bp--;
		}
	}

	gpio_set_value_cansleep(SPI_CS, 1);	/* cs low */
	udelay(1);
	return 0;

}

static int chimei_write_cmd_b3(u8 cmd)
{
	tianma_spi_write((u8)cmd, 0x00, 0);

	return 0;
}

static int chimei_write_data_b3(u8 cmd)
{
	tianma_spi_write(0x00, (u8)cmd,1);

	return 0;
}


static int chimei_serigo(struct chimei_spi_data data)
{
	int hw_id;
	hw_id = qci_read_hw_id();
	if (hw_id >= 4)
	{
		int i;

		for (i = 0; i < data.len; ++i) {
			chimei_write_cmd_b3(data.addr);
			chimei_write_data_b3(data.data[i]);
		}
		return 0;
	} else {
		char tx_buf[32];
		int bit_size = 0, i, rc;
		struct spi_message  m;
		struct spi_transfer t;

		if (!lcdc_spi_client) {
			pr_err("%s lcdc_spi_client is NULL\n", __func__);
			return -EINVAL;
		}

		memset(&t, 0, sizeof t);
		memset(tx_buf, 0, sizeof tx_buf);
		t.tx_buf = tx_buf;
		spi_setup(lcdc_spi_client);
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);

		ADD_BIT(FALSE);
		ADD_BYTE(data.addr);
		for (i = 0; i < data.len; ++i) {
			ADD_BIT(TRUE);
			ADD_BYTE(data.data[i]);
		}

		/* add padding bits so we round to next byte */
		t.len = (bit_size+7) / 8;
		if (t.len <= 4)
			t.bits_per_word = bit_size;

		rc = spi_sync(lcdc_spi_client, &m);
#ifdef DEBUG
		pr_info("%s: addr=0x%02x, #args=%d[%d] [%s], rc=%d\n",
			__func__, data.addr, t.len, t.bits_per_word,
			byte_to_binary(tx_buf, t.len), rc);
#endif
		return rc;
	}
}
static int chimei_write_cmd(u8 cmd)
{
	int hw_id;
	hw_id = qci_read_hw_id();
	if (hw_id >= 4) {
		chimei_write_cmd_b3(cmd);
		return 0;
	} else {

		char tx_buf[2];
		int bit_size = 0, rc;
		struct spi_message  m;
		struct spi_transfer t;

		if (!lcdc_spi_client) {
			pr_err("%s lcdc_spi_client is NULL\n", __func__);
			return -EINVAL;
		}

		memset(&t, 0, sizeof t);
		memset(tx_buf, 0, sizeof tx_buf);
		t.tx_buf = tx_buf;
		spi_setup(lcdc_spi_client);
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);

		ADD_BIT(FALSE);
		ADD_BYTE(cmd);

		t.len = 2;
		t.bits_per_word = 9;

		rc = spi_sync(lcdc_spi_client, &m);
#ifdef DEBUG
		pr_info("%s: addr=0x%02x, #args=%d[%d] [%s], rc=%d\n",
			__func__, cmd, t.len, t.bits_per_word,
			byte_to_binary(tx_buf, t.len), rc);
#endif
		return rc;
	}
}

static int chimei_write_data(u8 data)
{
	int hw_id;
	hw_id = qci_read_hw_id();
	if (hw_id >= 4) {
		chimei_write_data_b3(data);
		return 0;
	} else {

		char tx_buf[2];
		int bit_size = 0, rc;
		struct spi_message  m;
		struct spi_transfer t;

		if (!lcdc_spi_client) {
			pr_err("%s lcdc_spi_client is NULL\n", __func__);
			return -EINVAL;
		}

		memset(&t, 0, sizeof t);
		memset(tx_buf, 0, sizeof tx_buf);
		t.tx_buf = tx_buf;
		spi_setup(lcdc_spi_client);
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);

		ADD_BIT(TRUE);
		ADD_BYTE(data);

		t.len = 2;
		t.bits_per_word = 9;

		rc = spi_sync(lcdc_spi_client, &m);
#ifdef DEBUG
		pr_info("%s: addr=0x%02x, #args=%d[%d] [%s], rc=%d\n",
			__func__, data, t.len, t.bits_per_word,
			byte_to_binary(tx_buf, t.len), rc);
#endif
		return rc;
	}
}

static int chimei_serigo_list(struct chimei_spi_data *data, int count)
{
	int i, rc;
	for (i = 0; i < count; ++i, ++data) {
		rc = chimei_serigo(*data);
		if (rc)
			return rc;
		msleep(10);
	}
	return 0;
}

static ssize_t chimei_write_cmd_fs(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	uint32 cmd;

	sscanf(buf, "%x", &cmd);
	printk(KERN_EMERG "[LCD]%s--cmd=%x\n", __func__,(u8)cmd);
	chimei_write_cmd((u8)cmd);

	return ret;
}

static ssize_t chimei_write_data_fs(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	uint32 cmd;

	sscanf(buf, "%x", &cmd);
	printk(KERN_EMERG "[LCD]%s--data=%x\n", __func__,(u8)cmd);
	chimei_write_data((u8)cmd);

	return ret;
}

static ssize_t chimei_suspend_fs(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	printk(KERN_EMERG "LCD--%s\n", __func__);
	chimei_write_cmd(0x28);
	mdelay(10);
	chimei_write_cmd(0x10);
	mdelay(200);


	return ret;
}

static ssize_t chimei_resume_fs(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	printk(KERN_EMERG "LCD--%s\n", __func__);
	chimei_write_cmd(0x11);
	mdelay(150);
	chimei_write_cmd(0x29);


	return ret;
}
static ssize_t chimei_reinit_fs(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	printk(KERN_EMERG "LCD--%s\n", __func__);
	gpio_set_value(33, 0);
	mdelay(10);
	gpio_set_value(33, 1);
	mdelay(10);
	chimei_serigo_list(display_sequence_1,
		sizeof(display_sequence_1)/sizeof(*display_sequence_1));
	/* 0x11: Sleep Out */
	chimei_write_cmd(0x11);
	msleep(150);
	chimei_serigo_list(display_sequence_2,
		sizeof(display_sequence_2)/sizeof(*display_sequence_2));
	msleep(120);
	/* 0x29: Display On */
	chimei_write_cmd(0x29);
	//brightness_control(15);


	return ret;
}

static ssize_t get_bl_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	int bl_level;

	bl_level = cur_brightness;
	return sprintf(buf, "%d\n", bl_level);
}


static DEVICE_ATTR(cmd,  S_IWUGO , NULL , chimei_write_cmd_fs);
static DEVICE_ATTR(data,  S_IWUGO , NULL , chimei_write_data_fs);
static DEVICE_ATTR(suspend,  S_IWUGO , NULL , chimei_suspend_fs);
static DEVICE_ATTR(resume,  S_IWUGO , NULL , chimei_resume_fs);
static DEVICE_ATTR(reinit,  S_IWUGO , NULL , chimei_reinit_fs);
static DEVICE_ATTR(bl_level,  S_IALLUGO , get_bl_level , NULL);


static struct attribute *fs_attrs[] = {
	&dev_attr_cmd.attr,
	&dev_attr_data.attr,
	&dev_attr_suspend.attr,
	&dev_attr_resume.attr,
	&dev_attr_reinit.attr,
	&dev_attr_bl_level.attr,
	NULL,
};
static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};

static void chimei_disp_powerup(void)
{
	printk(KERN_EMERG "[LCD]%s\n", __func__);
	if (!chimei_state.disp_powered_up && !chimei_state.display_on)
		chimei_state.disp_powered_up = TRUE;

	//report_spi_status(TRUE);	//spi use
}

static void chimei_disp_on(void)
{
	int hw_id;
	long wait_time;
	hw_id = qci_read_hw_id();
	if (hw_id >= 4)	{
		SPI_SCLK = 174;
		SPI_CS = 168;
		SPI_MOSI = 175;
	} else {
		SPI_SCLK = 45;
		SPI_CS = 44;
		SPI_MOSI = 47;
		//SPI_MISO = 48;
	}

	printk(KERN_EMERG "[LCD]%s\n", __func__);
	if (chimei_state.disp_powered_up && !chimei_state.display_on) {
		gpio_set_value(SPI_CS, 1);	/* low */
		gpio_set_value(SPI_SCLK, 1);	/* high */
		gpio_set_value(SPI_MOSI, 1);
       #if 0
       gpio_set_value(33, 0);
	mdelay(10);
       gpio_set_value(33, 1);
	mdelay(10);	
	printk(KERN_EMERG "[LCD]%s--init start\n", __func__);
	chimei_serigo_list(display_sequence_1,
		sizeof(display_sequence_1)/sizeof(*display_sequence_1));
	/* 0x11: Sleep Out */
	chimei_write_cmd(0x11);
	msleep(150);
	chimei_serigo_list(display_sequence_2,
		sizeof(display_sequence_2)/sizeof(*display_sequence_2));
	msleep(120);
	/* 0x29: Display On */
	chimei_write_cmd(0x29);
	brightness_control(15);
       #endif
		wait_time = (long)(chimei_state.sleep_in_time + HZ * 120 / 1000) - (long)jiffies;
		if (wait_time > 0)
			mdelay(wait_time*1000/HZ > 120 ? 120 : wait_time*1000/HZ);

		chimei_state.sleep_out_time = jiffies;
		chimei_write_cmd(0x11);
		mdelay(5);
		chimei_write_cmd(0x29);
		chimei_state.display_on = TRUE;
	}
}

static int lcdc_chimei_panel_on(struct platform_device *pdev)
{
	printk(KERN_EMERG "[LCD]%s\n", __func__);
	if (!chimei_state.disp_initialized) {
		chimei_disp_powerup();
		chimei_disp_on();
		chimei_state.disp_initialized = TRUE;
	}
	return 0;
}

static int lcdc_chimei_panel_off(struct platform_device *pdev)
{
	printk(KERN_EMERG "[LCD]%s\n", __func__);
	if (chimei_state.disp_powered_up && chimei_state.display_on) {
		chimei_state.display_on = FALSE;
		chimei_state.disp_initialized = FALSE;
	}
	//report_spi_status(FALSE);	//spi not use
	return 0;
}


static void brightness_control(int brightness)
{
    //printk(KERN_EMERG "[LCD]%s: enter Brightness = %d\n", __func__,brightness);
	int overheat;

	if(brightness > 15)
		brightness = 15;

	if(brightness < 0)
		brightness = 0;

	overheat = check_msm_temp_over_heat();
	if ( (overheat == TRUE) && (brightness > LIM_BRIGHTNESS) )
		brightness = LIM_BRIGHTNESS;

	if (brightness == 15) {
		chimei_serigo_list(gamma_sequence_level15,
			sizeof(gamma_sequence_level15)/sizeof(*gamma_sequence_level15));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 15\n");
	} else if (brightness == 14) {
		chimei_serigo_list(gamma_sequence_level14,
			sizeof(gamma_sequence_level14)/sizeof(*gamma_sequence_level14));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 14\n");
	} else if (brightness == 13) {
		chimei_serigo_list(gamma_sequence_level13,
			sizeof(gamma_sequence_level13)/sizeof(*gamma_sequence_level13));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 13\n");
	} else if (brightness == 12) {
		chimei_serigo_list(gamma_sequence_level12,
			sizeof(gamma_sequence_level12)/sizeof(*gamma_sequence_level12));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 12\n");
	} else if (brightness == 11) {
		chimei_serigo_list(gamma_sequence_level11,
			sizeof(gamma_sequence_level11)/sizeof(*gamma_sequence_level11));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 11\n");
	} else if (brightness == 10) {
		chimei_serigo_list(gamma_sequence_level10,
			sizeof(gamma_sequence_level10)/sizeof(*gamma_sequence_level10));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 10\n");
	} else if (brightness == 9) {
		chimei_serigo_list(gamma_sequence_level9,
			sizeof(gamma_sequence_level9)/sizeof(*gamma_sequence_level9));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 9\n");
	} else if (brightness == 8) {
		chimei_serigo_list(gamma_sequence_level8,
			sizeof(gamma_sequence_level8)/sizeof(*gamma_sequence_level8));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 8\n");
	} else if (brightness == 7) {
		chimei_serigo_list(gamma_sequence_level7,
			sizeof(gamma_sequence_level7)/sizeof(*gamma_sequence_level7));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 7\n");
	} else if (brightness == 6) {
		chimei_serigo_list(gamma_sequence_level6,
			sizeof(gamma_sequence_level6)/sizeof(*gamma_sequence_level6));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 6\n");
	} else if (brightness == 5) {
		chimei_serigo_list(gamma_sequence_level5,
			sizeof(gamma_sequence_level5)/sizeof(*gamma_sequence_level5));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 5\n");
	} else if (brightness == 4) {
		chimei_serigo_list(gamma_sequence_level4,
			sizeof(gamma_sequence_level4)/sizeof(*gamma_sequence_level4));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 4\n");
	} else if (brightness == 3) {
		chimei_serigo_list(gamma_sequence_level3,
			sizeof(gamma_sequence_level3)/sizeof(*gamma_sequence_level3));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 3\n");
	} else if (brightness == 2) {
		chimei_serigo_list(gamma_sequence_level2,
			sizeof(gamma_sequence_level2)/sizeof(*gamma_sequence_level2));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 2\n");
	} else if (brightness == 1) {
		chimei_serigo_list(gamma_sequence_level1,
			sizeof(gamma_sequence_level1)/sizeof(*gamma_sequence_level1));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 1\n");
	} else {
		chimei_serigo_list(gamma_sequence_level0,
			sizeof(gamma_sequence_level0)/sizeof(*gamma_sequence_level0));
		printk(KERN_DEBUG "[LCD]: BrightnessLevel = 0\n");
	}
	cur_brightness = brightness;

    //printk(KERN_INFO "[LCD]%s: leave\n", __func__);
	return;
}

void overheat_set_brightness(void)
{
	if (cur_brightness > LIM_BRIGHTNESS)
	{
		pre_brightness = cur_brightness;
		cur_brightness = LIM_BRIGHTNESS;
		brightness_control(cur_brightness);
	}
}
EXPORT_SYMBOL(overheat_set_brightness);

void cooldown_set_brightness(void)
{
	if (pre_brightness != -1)
	{
		brightness_control(pre_brightness);
		pre_brightness = -1;
	}
}
EXPORT_SYMBOL(cooldown_set_brightness);

static void lcdc_chimei_panel_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level;

	bl_level = mfd->bl_level;
	brightness_control(bl_level);
}

static int __devinit chimei_probe(struct platform_device *pdev)
{
#ifdef SYSFS_DEBUG_CMD
	struct platform_device *fb_dev;
	struct msm_fb_data_type *mfd;
	int rc;
#endif

	printk(KERN_EMERG "[LCD]%s\n", __func__);
	if (pdev->id == 0) {
		lcdc_chimei_pdata = pdev->dev.platform_data;
		return 0;
	}

#ifndef SYSFS_DEBUG_CMD
	msm_fb_add_device(pdev);
#else
	fb_dev = msm_fb_add_device(pdev);
	mfd = platform_get_drvdata(fb_dev);
	rc = sysfs_create_group(&mfd->fbi->dev->kobj, &fs_attr_group);
	if (rc) {
		printk(KERN_EMERG"%s: sysfs group creation failed, rc=%d\n", __func__, rc);
		return rc;
	}
#endif

	return 0;
}

#ifdef CONFIG_SPI_QSD
static int __devinit lcdc_chimei_spi_probe(struct spi_device *spi)
{
	printk(KERN_EMERG "[LCD]%s\n", __func__);
	lcdc_spi_client = spi;
	lcdc_spi_client->bits_per_word = 32;
	return 0;
}
static int __devexit lcdc_chimei_spi_remove(struct spi_device *spi)
{
	lcdc_spi_client = NULL;
	return 0;
}
static void chimei_suspend(struct early_suspend *handler) {
	long wait_time;
	printk(KERN_EMERG "LCD--%s\n", __func__);
	chimei_write_cmd(0x28);
	mdelay(5);
	wait_time = (long)(chimei_state.sleep_out_time + HZ * 120 / 1000) - (long)jiffies;
	if (wait_time > 0)
		mdelay(wait_time*1000/HZ > 115 ? 115 : wait_time*1000/HZ);

	chimei_state.sleep_in_time = jiffies;
	chimei_write_cmd(0x10);
	mdelay(5);
}

static void chimei_resume(struct early_suspend *handler) {
	printk(KERN_EMERG "LCD--%s\n", __func__);
	/*chimei_write_cmd(0x11);
	mdelay(5);
	chimei_write_cmd(0x29);
	mdelay(5);*/
}

static struct spi_driver lcdc_chimei_spi_driver = {
	.driver = {
		.name  = LCDC_CHIMEI_SPI_DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.probe         = lcdc_chimei_spi_probe,
	.remove        = __devexit_p(lcdc_chimei_spi_remove),
};
#endif
static struct platform_driver this_driver = {
	.probe  = chimei_probe,
	.driver = {
		.name   = "lcdc_chimei_wvga",
	},
};

static struct msm_fb_panel_data chimei_panel_data = {
	.on = lcdc_chimei_panel_on,
	.off = lcdc_chimei_panel_off,
	.set_backlight = lcdc_chimei_panel_set_backlight,
};

static struct early_suspend chimei_power_suspend = {	
	.suspend = chimei_suspend,
	.resume = chimei_resume,	
	//.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	};


static struct platform_device this_device = {
	.name   = "lcdc_chimei_wvga",
	.id	= 1,
	.dev	= {
		.platform_data = &chimei_panel_data,
	}
};


extern int qci_get_offline_charging(void);
static int __init lcdc_chimei_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;
        printk(KERN_EMERG "[LCD]%s\n", __func__);


	if ( qci_get_offline_charging() != 0 ) {
	    printk(KERN_EMERG "[LCD] off line charging %s\n", __func__);
	    return 0;
	}

	if (msm_fb_detect_client("lcdc_chimei_wvga"))
		return 0;


	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &chimei_panel_data.panel_info;
	pinfo->xres = 480;
	pinfo->yres = 800;
	MSM_FB_SINGLE_MODE_PANEL(pinfo);
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 2;
#if defined(T_QCI_IS3)
	pinfo->clk_rate = 24576000;
#else
	pinfo->clk_rate = 30720000;
#endif /* T_QCI_IS3 */
	pinfo->bl_max = 15;
	pinfo->bl_min = 0;

	pinfo->lcdc.h_back_porch = 15;
	pinfo->lcdc.h_front_porch = 5;
	pinfo->lcdc.h_pulse_width = 6;
	pinfo->lcdc.v_back_porch = 2;
	pinfo->lcdc.v_front_porch = 2;
	pinfo->lcdc.v_pulse_width = 2;
	pinfo->lcdc.border_clr = 0;
	pinfo->lcdc.underflow_clr = 0xff;
	pinfo->lcdc.hsync_skew = 0;
	register_early_suspend(&chimei_power_suspend);
	ret = platform_device_register(&this_device);
	if (ret) {
		printk(KERN_ERR "%s not able to register the device\n",
			__func__);
		goto fail_driver;
	}
#ifdef CONFIG_SPI_QSD
	ret = spi_register_driver(&lcdc_chimei_spi_driver);

	if (ret) {
		printk(KERN_ERR "%s not able to register spi\n", __func__);
		goto fail_device;
	}
#endif
	return ret;
#ifdef CONFIG_SPI_QSD
fail_device:
	platform_device_unregister(&this_device);
	//register_early_suspend(&chimei_power_suspend);
#endif
fail_driver:
		platform_driver_unregister(&this_driver);

	return ret;
}

module_init(lcdc_chimei_panel_init);
#ifdef CONFIG_SPI_QSD
static void __exit lcdc_chimei_panel_exit(void)
{
	spi_unregister_driver(&lcdc_chimei_spi_driver);
}
module_exit(lcdc_chimei_panel_exit);
#endif

