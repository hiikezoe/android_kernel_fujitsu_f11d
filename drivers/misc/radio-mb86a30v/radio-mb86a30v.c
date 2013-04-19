/**
	@file	radio-mb86a30v.c \n
	multimedia tuner module device driver header file. \n
	This file is a header file for multimedia tuner module device driver users.
*/
/* COPYRIGHT FUJITSU SEMICONDUCTOR LIMITED 2011 */
#ifndef	__KERNEL__
#define	__KERNEL__
#endif

#include "radio-mb86a30v-dev.h"

static struct i2c_driver mb86a30v_i2c_driver;
static struct i2c_client *mb86a30v_i2c_client;

//static int devmajor = NODE_MAJOR;
//static char *devname = "mb86a30v";//NODE_PATHNAME;

#define		LICENSE			"GPL v2"
#define		AUTHOR			"FUJITSU SEMICONDUCTOR LIMITED"
#define		DESCRIPTION		"FUJITSU SEMICONDUCTOR LIMITED"
#define		VERSION			"1.2"

MODULE_LICENSE(LICENSE);
MODULE_AUTHOR(AUTHOR);
MODULE_DESCRIPTION(DESCRIPTION);
MODULE_VERSION(VERSION);

/* insmod() Parameter */
static int MB86A30V_DEBUG = 1;
static char *mode = NULL;
module_param(mode, charp, S_IRUGO);	/* Executed mode : DEBUG "mb86a30v_DEBUG" */

#define	MB86A30V_DEF_REG2B		0x08
#define	MB86A30V_DEF_REG2E		0x16
#define	MB86A30V_DEF_REG30		0x04
#define	MB86A30V_DEF_REG31		0x3F
#define	MB86A30V_DEF_REG5A		0x3E

static int mb86a30v_IOCTL_RF_INIT(mb86a30v_cmdcontrol_t * cmdctrl,
				 unsigned int cmd, unsigned long arg);

#define	MB86A30V_CALB_DCOFF_WAIT		10

struct mb86a30v_data {
	struct i2c_client *client;
	struct i2c_adapter *i2c2_adapter;	
	struct mutex lock;	
};
/************************************************************************/
/**
	internal function. \n
	I2C write. \n

	@param	reg		[in] register address.
	@param	value		[in] write data.
	@retval	=0	normal return.
	@retval	!=0	The error occurred. The detailed information is set to an errno.
 */
static int mb86a30v_i2c_master_send(unsigned int reg, unsigned int value)
{
	struct i2c_msg msg[1];
	unsigned char data[2];
	int ret =0;

	data[0] = reg;
	data[1] = value;
	
	msg[0].addr = mb86a30v_i2c_client->addr;
	msg[0].flags = mb86a30v_i2c_client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = &data[0];



	if (i2c_transfer(mb86a30v_i2c_client->adapter, msg,1) < 0) {
		printk(KERN_ERR"%s fail !\n",__func__);
		return -EIO;
	}	
	//printk(KERN_ERR"msg[1].buf:0x%x, (char *)msg[1].buf:0x%x, ret :%d\n",msg[1].buf, (int)rbuf[0],ret );
	//		DBGPRINT(PRINT_LHEADERFMT
	//		 "** reg[0x%x], *rbuf[0x%x], count[0x%x], rbuf[0x%x]\n",
	//		 PRINT_LHEADER, (int)reg, (int)rbuf, (int)count,
	//		 (int)rbuf[0]);
	return ret;	
}

/************************************************************************/
/**
	internal function. \n
	I2C read. \n

	@param	reg		[in] register address.
	@param	rbuf		[in] read data buffer address.
	@param	count		[in] read data count.
	@retval	=0	normal return.
	@retval	!=0	The error occurred. The detailed information is set to an errno.
 */
static int mb86a30v_i2c_master_recv(unsigned char reg, unsigned char *rbuf,
				   size_t count)
{

	struct i2c_msg msg[2];
	int ret=0;

	msg[0].addr = mb86a30v_i2c_client->addr;
	msg[0].flags = mb86a30v_i2c_client->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = (char *)&reg;

	//ret = i2c_transfer(adap, &msg, 1);
	
	msg[1].addr = mb86a30v_i2c_client->addr;
	msg[1].flags = mb86a30v_i2c_client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = count;
	msg[1].buf = rbuf;

	if (i2c_transfer(mb86a30v_i2c_client->adapter, msg, 2) < 0) {
		printk(KERN_ERR"%s fail !\n",__func__);
		return -EIO;
	}		
	//printk(KERN_ERR"msg[1].buf:0x%x, (char *)msg[1].buf:0x%x, ret :%d\n",msg[1].buf, (int)rbuf[0],ret );
			DBGPRINT(PRINT_LHEADERFMT
			 "** reg[0x%x], *rbuf[0x%x], count[0x%x], rbuf[0x%x]\n",
			 PRINT_LHEADER, (int)reg, (int)rbuf, (int)count,
			 (int)rbuf[0]);
	return ret;	
}

/************************************************************************/
/**
	internal function. \n
	I2C write.  selected sub-address.\n

	@param	suba		[in] register address. (SUBA)
	@param	reg		[in] register address.
	@param	subd		[in] register address. (SUBD)
	@param	value		[in] write data.
 */
static void mb86a30v_i2c_slave_send(unsigned char suba, unsigned char reg,
				   unsigned char subd, unsigned int value)
{
	DBGPRINT(PRINT_LHEADERFMT
		 "** SUBA[%02x], sreg[%02x], SUBD[%02x], value[%04x]\n",
		 PRINT_LHEADER, (int)suba, (int)reg, (int)subd, (int)value);

	if (mb86a30v_i2c_client != NULL) {
		mb86a30v_i2c_master_send(suba, reg);

		mb86a30v_i2c_master_send(subd, value);
	} else {
		ERRPRINT("mb86a30v-i2c not attached. [sub write]\n");
	}
	return;
}

/************************************************************************/
/**
	internal function.
	I2C read. selected sub-address.\n

	@param	suba		[in] register address. (SUBA)
	@param	reg		[in] register address.
	@param	subd		[in] register address. (SUBD)
	@param	rbuf		[out] read data buffer.
	@param	count		[in] read data count.
	@retval	=0	normal return.
	@retval	!=0	The error occurred. The detailed information is set to an errno.
 */
static int mb86a30v_i2c_slave_recv(unsigned char suba, unsigned char reg,
				  unsigned char subd, unsigned char *rbuf,
				  size_t count)
{
	unsigned char data[4];
	int indx;
	unsigned char adrreg = reg;
	int err = 0;

	memset(data, 0, sizeof(data));
	if (mb86a30v_i2c_client != NULL) {
		for (indx = 0; indx < count; indx++) {
			mb86a30v_i2c_master_send(suba, adrreg);

			err = mb86a30v_i2c_master_recv(subd, data, 1);
			if (err != 0) {
				ERRPRINT
				    ("sub-register read error : %d bytes transferred.\n",
				     err);
				goto i2c_slave_recv_return;
			}
			rbuf[indx] = data[0];
			adrreg += 1;
		}
	} else {
		ERRPRINT("mb86a30v-i2c not attached. [sub read]\n");
	}

	DBGPRINT(PRINT_LHEADERFMT
		 "** SUBA[%02x], sreg[%02x], SUBD[%02x], *rbuf[%08x], count[%d], rbuf[%02x]\n",
		 PRINT_LHEADER, (int)suba, (int)reg, (int)subd, (int)rbuf,
		 (int)count, (int)rbuf[0]);

i2c_slave_recv_return:
	return err;
}

/************************************************************************/
/**
	internal function. \n
	I2C write.  selected sub-address.\n
	suported 8 / 16 / 24 bits transfer.

	@param	suba		[in] register address. (SUBA)
	@param	reg		[in] register address.
	@param	subd		[in] register address. (SUBD)
	@param	data		[in] write data address.
	@param	mode		[in] 8 / 16 / 24 bits.
 */
static void mb86a30v_i2c_sub_send(unsigned char suba, unsigned char reg,
				 unsigned char subd, unsigned char *data,
				 unsigned char mode)
{
	unsigned char regsubd = subd;
	unsigned int value;

	DBGPRINT(PRINT_LHEADERFMT
		 "** SUBA[%02x], sreg[%02x], SUBD[%02x], data[%08x], mode[%02x]\n",
		 PRINT_LHEADER, (int)suba, (int)reg, (int)subd, (int)data,
		 (int)mode);

	if (mb86a30v_i2c_client != NULL) {
		mb86a30v_i2c_master_send(suba, reg);

		value = data[0];
		mb86a30v_i2c_master_send(regsubd, value);

		switch (mode) {
/*		case PARAM_I2C_MODE_SEND_16:
			regsubd += 1;
			value = data[1];
			mb86a30v_i2c_master_send(regsubd, value);
			break;
*/		case PARAM_I2C_MODE_SEND_24:
			regsubd += 1;
			value = data[1];
			mb86a30v_i2c_master_send(regsubd, value);

			regsubd += 1;
			value = data[2];
			mb86a30v_i2c_master_send(regsubd, value);
			break;
		}
	} else {
		ERRPRINT("mb86a30v-i2c not attached. [sub write]\n");
	}
	return;
}

/************************************************************************/
/**
	internal function.
	I2C read. selected sub-address.\n

	@param	suba		[in] register address. (SUBA)
	@param	reg		[in] register address.
	@param	subd		[in] register address. (SUBD)
	@param	rbuf		[out] read data buffer.
	@param	mode		[in] 8 / 16 / 24 bits.
	@retval	=0	normal return.
	@retval	!=0	The error occurred. The detailed information is set to an errno.
 */
static int mb86a30v_i2c_sub_recv(unsigned char suba, unsigned char reg,
				unsigned char subd, unsigned char *data,
				unsigned char mode)
{
	unsigned char regsubd = subd;
	int err = 0;

	memset(data, 0, sizeof(data));
	if (mb86a30v_i2c_client != NULL) {
		mb86a30v_i2c_master_send(suba, reg);

		err = mb86a30v_i2c_master_recv(regsubd, &data[0], 1);
		if (err != 0) {
			ERRPRINT
			    ("sub-register read error : %d bytes transferred.\n",
			     err);
			goto i2c_sub_recv_return;
		}

		switch (mode) {
/*		case PARAM_I2C_MODE_RECV_16:
			regsubd += 1;
			err = mb86a30v_i2c_master_recv(regsubd, &data[1], 1);
			if (err != 0) {
				ERRPRINT
				    ("sub-register read error : %d bytes transferred.\n",
				     err);
				goto i2c_sub_recv_return;
			}
			break;
*/		case PARAM_I2C_MODE_RECV_24:
			regsubd += 1;
			err = mb86a30v_i2c_master_recv(regsubd, &data[1], 1);
			if (err != 0) {
				ERRPRINT
				    ("sub-register read error : %d bytes transferred.\n",
				     err);
				goto i2c_sub_recv_return;
			}

			regsubd += 1;
			err = mb86a30v_i2c_master_recv(regsubd, &data[2], 1);
			if (err != 0) {
				ERRPRINT
				    ("sub-register read error : %d bytes transferred.\n",
				     err);
				goto i2c_sub_recv_return;
			}
			break;
		}
	} else {
		ERRPRINT("mb86a30v-i2c not attached. [sub read]\n");
	}

	DBGPRINT(PRINT_LHEADERFMT
		 "** SUBA[%02x], sreg[%02x], SUBD[%02x], *data[%08x], mode[%02x] : data[%02x:%02x:%02x]\n",
		 PRINT_LHEADER, (int)suba, (int)reg, (int)subd, (int)data, mode,
		 data[0], data[1], data[2]);

i2c_sub_recv_return:
	return err;
}



/************************************************************************/
/**
	internal function. \n
	I2C write. with mask.\n

	@param	reg		[in] register address.
	@param	value		[in] write data.
	@param	I2C_MASK		[in] I2C mask data.
	@retval	=0	normal return.
	@retval	!=0	The error occurred. The detailed information is set to an errno.
 */
static int mb86a30v_i2c_master_send_mask(unsigned int reg, unsigned int value,
					u8 I2C_MASK, u8 PARAM_MASK)
{
	unsigned int svalue = 0;
	u8 I2C_DATA = 0;
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 "** reg[%02x], value[%04x], I2C_MASK[%02x], PARAM_MASK[%02x]\n",
		 PRINT_LHEADER, (int)reg, (int)value, (int)I2C_MASK,
		 (int)PARAM_MASK);

	rtncode = mb86a30v_i2c_master_recv(reg, &I2C_DATA, 1);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto i2c_master_send_mask_return;
	}
	svalue = I2C_DATA & I2C_MASK;

	svalue |= (value & PARAM_MASK);
	mb86a30v_i2c_master_send(reg, svalue);

i2c_master_send_mask_return:
	return rtncode;
}

/************************************************************************/
/**
	internal function. \n
	I2C write.  selected sub-address.  with mask.\n

	@param	suba		[in] register address. (SUBA)
	@param	reg		[in] register address.
	@param	subd		[in] register address. (SUBD)
	@param	value		[in] write data.
	@param	I2C_MASK		[in] I2C mask data.
	@retval	=0	normal return.
	@retval	!=0	The error occurred. The detailed information is set to an errno.
 */
static int mb86a30v_i2c_slave_send_mask(unsigned char suba, unsigned char reg,
				       unsigned char subd, unsigned int value,
				       u8 I2C_MASK, u8 PARAM_MASK)
{
	unsigned int svalue = 0;
	u8 I2C_DATA = 0;
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 "** SUBA[%02x], sreg[%02x], SUBD[%02x], value[%04x], I2C_MASK[%02x]\n",
		 PRINT_LHEADER, (int)suba, (int)reg, (int)subd, (int)value,
		 (int)I2C_MASK);

	rtncode = mb86a30v_i2c_slave_recv(suba, reg, subd, &I2C_DATA, 1);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto i2c_slave_send_mask_return;
	}
	svalue = I2C_DATA & I2C_MASK;

	svalue |= (value & PARAM_MASK);
	mb86a30v_i2c_slave_send(suba, reg, subd, svalue);

i2c_slave_send_mask_return:
	return rtncode;
}

/************************************************************************/
/**-- long (2011/08/19) --
	ioctl System Call.  IOCTL_RST_SOFT command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_RST_SOFT(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			   unsigned long arg)
{
	ioctl_reset_t *RESET_user = (ioctl_reset_t *) arg;
	ioctl_reset_t *RESET = &cmdctrl->RESET;
	size_t tmpsize = sizeof(ioctl_reset_t);
	int rtncode = 0;
	unsigned int reg;
	unsigned int value;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (RESET_user == NULL) {
		rtncode = -EINVAL;
		goto reset_return;
	}

	memset(RESET, 0, tmpsize);
	if (copy_from_user(RESET, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto reset_return;
	}

	reg = (int)MB86A30V_REG_ADDR_RST;
	if (RESET->RESET == PARAM_RESET_ON) {
		value = (int)0;
		mb86a30v_i2c_master_send(reg, value);
	} else if (RESET->RESET == PARAM_RESET_OFF) {
		value =
		    (int)(MB86A30V_MASK_RST_I2CREG_RESET |
			  MB86A30V_MASK_RST_LOGIC_RESET);
		mb86a30v_i2c_master_send(reg, value);
	} else if (RESET->RESET == PARAM_LOGIC_RESET) {
		mb86a30v_i2c_master_send(reg, 0xf8);
		mb86a30v_i2c_master_send(reg, 0xff);
	}
	else {
		rtncode = -EINVAL;
		goto reset_return;
	}
	

reset_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_RST_SYNC command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_RST_SYNC(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			   unsigned long arg)
{
	ioctl_reset_t *RESET_user = (ioctl_reset_t *) arg;
	ioctl_reset_t *RESET = &cmdctrl->RESET;
	size_t tmpsize = sizeof(ioctl_reset_t);
	int rtncode = 0;
	unsigned int reg;
	unsigned int value;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (RESET_user == NULL) {
		rtncode = -EINVAL;
		goto sync_return;
	}

	memset(RESET, 0, tmpsize);
	if (copy_from_user(RESET, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto sync_return;
	}

	if ((RESET->STATE_INIT != PARAM_STATE_INIT_ON)
	    && (RESET->STATE_INIT != PARAM_STATE_INIT_OFF)) {
		rtncode = -EINVAL;
		goto sync_return;
	}

	reg = MB86A30V_REG_ADDR_STATE_INIT;
	value = RESET->STATE_INIT;
	mb86a30v_i2c_master_send(reg, value);

sync_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}


/************************************************************************/
/**
	ioctl System Call.  IOCTL_SET_SPECT command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_SET_SPECT(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			    unsigned long arg)
{
	ioctl_init_t *INIT_user = (ioctl_init_t *) arg;
	ioctl_init_t *INIT = &cmdctrl->INIT;
	size_t tmpsize = sizeof(ioctl_init_t);
	int rtncode = 0;
	unsigned int reg;
	unsigned int value;
#define	SET_SPECT_PATTERN1	( PARAM_IQINV_IQINV_NORM | PARAM_IQINV_IQINV_INVT )

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (INIT_user == NULL) {
		rtncode = -EINVAL;
		goto spect_return;
	}

	memset(INIT, 0, tmpsize);
	if (copy_from_user(INIT, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto spect_return;
	}

	if ((INIT->IQINV & ~SET_SPECT_PATTERN1) != 0) {
		rtncode = -EINVAL;
		goto spect_return;
	}

	reg = MB86A30V_REG_ADDR_IQINV;
	value = INIT->IQINV;
	rtncode =
	    mb86a30v_i2c_master_send_mask(reg, value, MB86A30V_I2CMASK_IQINV,
					 MB86A30V_MASK_IQINV);
	if (rtncode != 0) {
		goto spect_return;
	}

spect_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}


/************************************************************************/
/**-- long (2011/08/19) --
	ioctl System Call.  IOCTL_SET_ALOG_PDOWN command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_SET_ALOG_PDOWN(mb86a30v_cmdcontrol_t * cmdctrl,
				 unsigned int cmd, unsigned long arg)
{
	ioctl_init_t *INIT_user = (ioctl_init_t *) arg;
	ioctl_init_t *INIT = &cmdctrl->INIT;
	size_t tmpsize = sizeof(ioctl_init_t);
	int rtncode = 0;
	unsigned int reg;
	unsigned int value;
#define	SET_PDOWN_PATTERN1	( PARAM_MACRO_PDOWN_DACPWDN_OFF | PARAM_MACRO_PDOWN_DACPWDN_ON )

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (INIT_user == NULL) {
		rtncode = -EINVAL;
		goto pdown_return;
	}

	memset(INIT, 0, tmpsize);
	if (copy_from_user(INIT, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto pdown_return;
	}

	if ((INIT->MACRO_PDOWN & ~SET_PDOWN_PATTERN1) != 0) {
		rtncode = -EINVAL;
		goto pdown_return;
	}

	reg = (int)MB86A30V_REG_ADDR_MACRO_PWDN;
	value = INIT->MACRO_PDOWN & MB86A30V_MASK_MACRO_PWDN_DACPWDN;
	mb86a30v_i2c_master_send(reg, value);

pdown_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}


/************************************************************************/
/**-- long (2011/08/18) --
	ioctl System Call.  IOCTL_AGC command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_AGC(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
		      unsigned long arg)
{
	ioctl_agc_t *AGC_user = (ioctl_agc_t *) arg;
	ioctl_agc_t *AGC = &cmdctrl->AGC;
	size_t tmpsize = sizeof(ioctl_agc_t);
	int rtncode = 0;
	unsigned char rega = MB86A30V_REG_ADDR_SUBADR;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_SUBDAT;
	unsigned int value;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (AGC_user == NULL) {
		rtncode = -EINVAL;
		goto agc_return;
	}

	memset(AGC, 0, tmpsize);
	if (copy_from_user(AGC, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto agc_return;
	}

	sreg = MB86A30V_REG_SUBR_IFAH;
	value = (AGC->IFA >> 8) & 0x0F;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

	sreg = MB86A30V_REG_SUBR_IFAL;
	value = (AGC->IFA) & 0xFF;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

	sreg = MB86A30V_REG_SUBR_IFBH;
	value = (AGC->IFB >> 8) & 0x0F;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

	sreg = MB86A30V_REG_SUBR_IFBL;
	value = (AGC->IFB) & 0xFF;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

	sreg = MB86A30V_REG_SUBR_MAXIFAGC;
	value = (AGC->MAXIFAGC);
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

	sreg = MB86A30V_REG_SUBR_AGAIN;
	value = (AGC->AGAIN);
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

	mb86a30v_i2c_master_send(0x01, AGC->POIF << 3);  //POIF

agc_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}


/************************************************************************/
/** -- long (2011/08/18) --
	ioctl System Call.  IOCTL_AGC_OPERATING command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_AGC_OPERATING(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			  unsigned long arg)
{
	int rtncode = 0;
	u8 data[3];

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	data[0] = 0x00 ;
	data[1] = 0x00 ;
	data[2] = 0xc0 ;
	mb86a30v_i2c_sub_send(0x28, 0x16, 0x29, data, PARAM_I2C_MODE_SEND_24);

	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/** -- long (2011/08/12) --
	ioctl System Call.  IOCTL_LNA_SETTING command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_LNA_SETTING(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			 unsigned long arg)
{
	ioctl_lna_setting_t *LNA_SETTING_user = (ioctl_lna_setting_t *) arg;
	ioctl_lna_setting_t *LNA_SETTING = &cmdctrl->LNA_SETTING;
	size_t tmpsize = sizeof(ioctl_lna_setting_t);
	unsigned char reg;
	unsigned int value = 0;
	int rtncode = 0;
	//u8 tmp_data = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (LNA_SETTING_user == NULL) {
		rtncode = -EINVAL;
		goto lna_setting_return;
	}

	memset(LNA_SETTING, 0, tmpsize);
	if (copy_from_user(LNA_SETTING, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto lna_setting_return;
	}

	switch (LNA_SETTING->LNA_SETTING) {
	case PARAM_LNA_INITIALIZE:
		reg = 0xe3;
		value = 0x00;
		mb86a30v_i2c_master_send(reg, value);
		break;

	case PARAM_LNA_ON:
		reg = 0xe0;
		value = 0x00;
		mb86a30v_i2c_master_send(reg, value);
		break;

	case PARAM_LNA_OFF:
		reg = 0xe0;
		value = 0x04;
		mb86a30v_i2c_master_send(reg, value);
		break;
	
	case PARAM_LNA_AUTO_ON:
		reg = 0xe0;
		value = 0x01;
		mb86a30v_i2c_master_send(reg, value);
		break;
/*
	case PARAM_LNA_AUTO_OFF:
		reg = 0xe0;
		value = 0x00;
		mb86a30v_i2c_master_send(reg, value);
		break;
*/
	case PARAM_LNA_INIT_SETTING:
		mb86a30v_i2c_master_send( 0xe0, 0x00 ) ;// LNA control
		mb86a30v_i2c_master_send( 0xe3, 0x01 ) ;
		mb86a30v_i2c_master_send( 0xe0, 0x01 ) ;
		mb86a30v_i2c_master_send( 0xe1, 0x02 ) ;
		mb86a30v_i2c_master_send( 0xe2, 0x40 ) ;
		mb86a30v_i2c_master_send( 0xe1, 0x03 ) ;
		mb86a30v_i2c_master_send( 0xe2, 0x04 ) ;
		mb86a30v_i2c_master_send( 0xe1, 0x04 ) ;
		mb86a30v_i2c_master_send( 0xe2, 0x14 ) ;
		break;

	default:
		rtncode = -EINVAL;
		goto lna_setting_return;
		break;
	}
/*
	tmp_data = LNA_SETTING->LNA_GAIN_TH;
	mb86a30v_i2c_sub_send(0xe1, 0x02, 0xe2, &tmp_data, PARAM_I2C_MODE_SEND_8);
	tmp_data = LNA_SETTING->LNA_GAIN_TH_L;
	mb86a30v_i2c_sub_send(0xe1, 0x03, 0xe2, &tmp_data, PARAM_I2C_MODE_SEND_8);
	tmp_data = LNA_SETTING->LNA_GAIN_TH_H;
	mb86a30v_i2c_sub_send(0xe1, 0x04, 0xe2, &tmp_data, PARAM_I2C_MODE_SEND_8);
*/
lna_setting_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/** -- long (2011/08/12) --
	ioctl System Call.  IOCTL_CMOS_SETTING command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_CMOS_SETTING(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			  unsigned long arg)
{
	int rtncode = 0;


	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	mb86a30v_i2c_master_send( 0xe1, 0x0e ) ;
	mb86a30v_i2c_master_send( 0xe2, 0x01 ) ;

	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/** -- long (2011/08/17) --
	ioctl System Call.  IOCTL_SYNC command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context. (NULL)
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_SYNC(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
		       unsigned long arg)
{
  	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	mb86a30v_i2c_master_send( 0x28, 0x01 ) ;// DC offset
	mb86a30v_i2c_master_send( 0x29, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x2a, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x2b, 0x11 ) ;

	mb86a30v_i2c_master_send( 0x01, 0x08 ) ;// AGC pol  

	mb86a30v_i2c_master_send( 0x04, 0x0a ) ;// AGC operating voltage  
	mb86a30v_i2c_master_send( 0x05, 0xe4 ) ;


	mb86a30v_i2c_master_send( 0x04, 0x0b ) ;// AGC loop gain
	mb86a30v_i2c_master_send( 0x05, 0xdd ) ;

	mb86a30v_i2c_master_send( 0x04, 0x0e ) ;// IF operation start point setting
	mb86a30v_i2c_master_send( 0x05, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x04, 0x0f ) ;
	mb86a30v_i2c_master_send( 0x05, 0x1e ) ;

	mb86a30v_i2c_master_send( 0x28, 0x46 ) ;// Threshold for Mode Detecting Setting
	mb86a30v_i2c_master_send( 0x29, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x2a, 0x12 ) ;
	mb86a30v_i2c_master_send( 0x2b, 0x90 ) ;

	mb86a30v_i2c_master_send( 0x28, 0x69 ) ;// No signal Detecting Setting
	mb86a30v_i2c_master_send( 0x29, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x2a, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x2b, 0x07 ) ;

	mb86a30v_i2c_master_send( 0x3b, 0x65 ) ;// Fading detecting
	mb86a30v_i2c_master_send( 0x3c, 0x05 ) ;
	mb86a30v_i2c_master_send( 0x3b, 0x7d ) ;
	mb86a30v_i2c_master_send( 0x3c, 0x02 ) ;

	mb86a30v_i2c_master_send( 0x28, 0x03 ) ;// Spurious removal
	mb86a30v_i2c_master_send( 0x29, 0x05 ) ;
	mb86a30v_i2c_master_send( 0x2a, 0x32 ) ;
	mb86a30v_i2c_master_send( 0x2b, 0x11 ) ;

	mb86a30v_i2c_master_send( 0x25, 0xd3 ) ;// Clock Pll loopgain

	mb86a30v_i2c_master_send( 0x28, 0x5a ) ;// Clock Pll loopgain threshold
	mb86a30v_i2c_master_send( 0x29, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x2a, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x2b, 0x08 ) ;

	mb86a30v_i2c_master_send( 0x28, 0x3d ) ;// Peak detection
	mb86a30v_i2c_master_send( 0x29, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x2a, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x2b, 0xc6 ) ;

	mb86a30v_i2c_master_send( 0x28, 0x16 ) ;// AGC Operating Timing Setting
	mb86a30v_i2c_master_send( 0x29, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x2a, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x2b, 0x1d ) ;


  DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}


/************************************************************************/
/** -- long (2011/08/12) --
	ioctl System Call.  IOCTL_AVDE_INIT command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_AVDE_INIT(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			  unsigned long arg)
{
	int rtncode = 0;


	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	mb86a30v_i2c_master_send(  0x31, 0x00 ) ;
	mb86a30v_i2c_master_send(  0x36, 0x10 ) ;
	mb86a30v_i2c_master_send(  0x32, 0x00 ) ;
	mb86a30v_i2c_master_send(  0x33, 0x97 ) ;
	mb86a30v_i2c_master_send(  0x32, 0x10 ) ;
	mb86a30v_i2c_master_send(  0x33, 0x08 ) ;
	mb86a30v_i2c_master_send(  0x31, 0x80 ) ;

	mb86a30v_i2c_master_send(  0x31, 0x00 ) ;
	mb86a30v_i2c_master_send(  0x36, 0x10 ) ;
	mb86a30v_i2c_master_send(  0x32, 0x00 ) ;
	mb86a30v_i2c_master_send(  0x33, 0x97 ) ;
	mb86a30v_i2c_master_send(  0x32, 0x10 ) ;
	mb86a30v_i2c_master_send(  0x33, 0x00 ) ;
	mb86a30v_i2c_master_send(  0x31, 0x80 ) ;

	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}


/************************************************************************/
/**
	ioctl System Call.  IOCTL_SEQ_GETSTAT command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_SEQ_GETSTAT(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			      unsigned long arg)
{
	ioctl_seq_t *SEQ_user = (ioctl_seq_t *) arg;
	ioctl_seq_t *SEQ = &cmdctrl->SEQ;
	size_t tmpsize = sizeof(ioctl_seq_t);
	int rtncode = 0;
	unsigned char reg;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (SEQ_user == NULL) {
		rtncode = -EINVAL;
		goto seq_getstate_return;
	}

	memset(SEQ, 0, tmpsize);
	if (copy_from_user(SEQ, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto seq_getstate_return;
	}

	reg = MB86A30V_REG_ADDR_SYNC_STATE;
	rtncode = mb86a30v_i2c_master_recv(reg, &SEQ->SYNC_STATE, 1);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto seq_getstate_return;
	}

	rtncode = put_user(SEQ->SYNC_STATE, (char *)&SEQ_user->SYNC_STATE);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto seq_getstate_return;
	}

	DBGPRINT(PRINT_LHEADERFMT " : SEQ.SYNC_STATE [%x]. \n", PRINT_LHEADER,
		 (int)(SEQ->SYNC_STATE));

seq_getstate_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_SEQ_SETMODE command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_SEQ_SETMODE(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			      unsigned long arg)
{
	ioctl_seq_t *SEQ_user = (ioctl_seq_t *) arg;
	ioctl_seq_t *SEQ = &cmdctrl->SEQ;
	size_t tmpsize = sizeof(ioctl_seq_t);
	int rtncode = 0;
	unsigned char reg;
	unsigned int value;
	u8 tmpdata = 0;
	

#define	SEQ_SETMODE_PATTERN1	( PARAM_MODED_CTRL_M3G16 | PARAM_MODED_CTRL_M3G08 | PARAM_MODED_CTRL_M3G04	\
				| PARAM_MODED_CTRL_M2G08 | PARAM_MODED_CTRL_M2G04 )
/*#define	SEQ_SETMODE_PATTERN2	( PARAM_MODED_STAT_MDFIX	\
				| PARAM_MODED_STAT_MODE2 | PARAM_MODED_STAT_MODE3		\
				| PARAM_MODED_STAT_GUARDE14 | PARAM_MODED_STAT_GUARDE18 | PARAM_MODED_STAT_GUARDE116 )
*/
	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (SEQ_user == NULL) {
		rtncode = -EINVAL;
		goto seq_setmode_return;
	}

	memset(SEQ, 0, tmpsize);
	if (copy_from_user(SEQ, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto seq_setmode_return;
	}

	if ((SEQ->MODED_CTRL & ~SEQ_SETMODE_PATTERN1) != 0) {
		rtncode = -EINVAL;
		goto seq_setmode_return;
	}

	//if ((SEQ->MODED_STAT & ~SEQ_SETMODE_PATTERN2) != 0) {
	//	rtncode = -EINVAL;
	//	goto seq_setmode_return;
	//} 

	reg = MB86A30V_REG_ADDR_MODED_CTRL;
	value = SEQ->MODED_CTRL;
	mb86a30v_i2c_master_send(reg, value);

	//reg = MB86A30V_REG_ADDR_MODED_STAT;
	//value = SEQ->MODED_STAT;
	//mb86a30v_i2c_master_send(reg, value);

	reg = MB86A30V_REG_ADDR_MODED_STAT;
	switch (SEQ->MODE_DETECT) {
	case PARAM_MODE_DETECT_ON:
		mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		tmpdata = tmpdata & 0xEF;
		mb86a30v_i2c_master_send( reg, tmpdata ) ;
		break;

	case PARAM_MODE_DETECT_OFF:
		mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		tmpdata = tmpdata & 0xE0;
		tmpdata = tmpdata | 0x10 | SEQ->MODE | SEQ->GUARD;

		mb86a30v_i2c_master_send( reg, tmpdata ) ;
		break;

	default:
		rtncode = -EINVAL;
		goto seq_setmode_return;
		break;
	}

seq_setmode_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_SEQ_GETMODE command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_SEQ_GETMODE(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			      unsigned long arg)
{
	ioctl_seq_t *SEQ_user = (ioctl_seq_t *) arg;
	ioctl_seq_t *SEQ = &cmdctrl->SEQ;
	size_t tmpsize = sizeof(ioctl_seq_t);
	int rtncode = 0;
	unsigned char reg;
	//unsigned int value;
	u8 tmpdata0, tmpdata1 = 0;


	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (SEQ_user == NULL) {
		rtncode = -EINVAL;
		goto seq_getmode_return;
	}

	memset(SEQ, 0, tmpsize);
	if (copy_from_user(SEQ, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto seq_getmode_return;
	}

	reg = MB86A30V_REG_ADDR_MODED_STAT;
	mb86a30v_i2c_master_recv(reg, &tmpdata0, 1);
	tmpdata1 = tmpdata0 & 0x60;
	if (copy_to_user
		((void *)&SEQ_user->MODED_STAT, (void *)&tmpdata1, 1)) {
		rtncode = -EFAULT;
		goto seq_getmode_return;
	}

        tmpdata1 = tmpdata0 & 0x0c;
	tmpdata1 = tmpdata1 >> 2;
        if (copy_to_user
                ((void *)&SEQ_user->MODE, (void *)&tmpdata1, 1)) {
                rtncode = -EFAULT;
                goto seq_getmode_return;
        }

        tmpdata1 = tmpdata0 & 0x03;
        if (copy_to_user
                ((void *)&SEQ_user->GUARD, (void *)&tmpdata1, 1)) {
                rtncode = -EFAULT;
                goto seq_getmode_return;
        }

seq_getmode_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**-- long (2011/08/19) --
	ioctl System Call.  IOCTL_SEQ_GETTMCC command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_SEQ_GETTMCC(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			      unsigned long arg)
{
	ioctl_seq_t *SEQ_user = (ioctl_seq_t *) arg;
	ioctl_seq_t *SEQ = &cmdctrl->SEQ;
	size_t tmpsize = sizeof(ioctl_seq_t);
	int rtncode = 0;
	unsigned char rega = MB86A30V_REG_ADDR_TMCC_SUBA;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_TMCC_SUBD;
	unsigned int value;
#define	SEQ_GETTMCC_PATTERN1	( PARAM_TMCCREAD_TMCCLOCK_AUTO | PARAM_TMCCREAD_TMCCLOCK_NOAUTO )

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (SEQ_user == NULL) {
		rtncode = -EINVAL;
		goto seq_gettmcc_return;
	}

	memset(SEQ, 0, tmpsize);
	if (copy_from_user(SEQ, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto seq_gettmcc_return;
	}

	if ((SEQ->TMCCREAD & ~SEQ_GETTMCC_PATTERN1) != 0) {
		rtncode = -EINVAL;
		goto seq_gettmcc_return;
	}

	sreg = MB86A30V_REG_SUBR_TMCCREAD;
	value = (SEQ->TMCCREAD & MB86A30V_MASK_TMCCREAD_TMCCLOCK);
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

	sreg = MB86A30V_REG_SUBR_TMCC0;
	rtncode = mb86a30v_i2c_slave_recv(rega, sreg, regd, &SEQ->TMCC[0], 32);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto seq_gettmcc_return;
	}

	sreg = MB86A30V_REG_SUBR_FEC_IN;
	rtncode = mb86a30v_i2c_slave_recv(rega, sreg, regd, &SEQ->FEC_IN, 1);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto seq_gettmcc_return;
	}
	SEQ->FEC_IN &=
	    (MB86A30V_MASK_FEC_IN_CORRECT | MB86A30V_MASK_FEC_IN_VALID);

	if (copy_to_user(&SEQ_user->TMCC[0], &SEQ->TMCC[0], 32)) {
		DBGPRINT(PRINT_LHEADERFMT " : copy_to_user error.\n",
			 PRINT_LHEADER);
		rtncode = -EFAULT;
		goto seq_gettmcc_return;
	}

	rtncode = put_user(SEQ->FEC_IN, (char *)&SEQ_user->FEC_IN);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto seq_gettmcc_return;
	}

seq_gettmcc_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_BER_MONICONFIG command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_BER_MONICONFIG(mb86a30v_cmdcontrol_t * cmdctrl,
				 unsigned int cmd, unsigned long arg)
{
	ioctl_ber_moni_t *BER_user = (ioctl_ber_moni_t *) arg;
	ioctl_ber_moni_t *BER = &cmdctrl->BER;
	size_t tmpsize = sizeof(ioctl_ber_moni_t);
	int rtncode = 0;
	unsigned char reg;
	unsigned char rega = MB86A30V_REG_ADDR_FEC_SUBA;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_FEC_SUBD;
	unsigned int value;
#define	BER_MONICONFIG_PATTERN1	( PARAM_S8WAIT_TS8_BER8 | PARAM_S8WAIT_TS9_BER8		\
				| PARAM_S8WAIT_TS8_BER9 | PARAM_S8WAIT_TS9_BER9 )
#define	BER_MONICONFIG_PATTERN2	( PARAM_VBERXRST_VBERXRSTC_C | PARAM_VBERXRST_VBERXRSTC_E	\
				| PARAM_VBERXRST_VBERXRSTB_C | PARAM_VBERXRST_VBERXRSTB_E	\
				| PARAM_VBERXRST_VBERXRSTA_C | PARAM_VBERXRST_VBERXRSTA_E )
#define	BER_MONICONFIG_PATTERN3	( PARAM_RSBERON_RSBERAUTO_M | PARAM_RSBERON_RSBERAUTO_A		\
				| PARAM_RSBERON_RSBERC_S | PARAM_RSBERON_RSBERC_B		\
				| PARAM_RSBERON_RSBERB_S | PARAM_RSBERON_RSBERB_B		\
				| PARAM_RSBERON_RSBERA_S | PARAM_RSBERON_RSBERA_B )
#define	BER_MONICONFIG_PATTERN4	( PARAM_RSBERXRST_RSBERXRSTC_S | PARAM_RSBERXRST_RSBERXRSTC_B	\
				| PARAM_RSBERXRST_RSBERXRSTB_S | PARAM_RSBERXRST_RSBERXRSTB_B	\
				| PARAM_RSBERXRST_RSBERXRSTA_S | PARAM_RSBERXRST_RSBERXRSTA_B )
#define	BER_MONICONFIG_PATTERN5	( PARAM_RSBERCEFLG_SBERCEFC_E | PARAM_RSBERCEFLG_SBERCEFB_E | PARAM_RSBERCEFLG_SBERCEFA_E )
#define	BER_MONICONFIG_PATTERN6	( PARAM_RSBERTHFLG_RSBERTHRC_R | PARAM_RSBERTHFLG_RSBERTHRB_R | PARAM_RSBERTHFLG_RSBERTHRA_R	\
				| PARAM_RSBERTHFLG_RSBERTHFC_F | PARAM_RSBERTHFLG_RSBERTHFB_F | PARAM_RSBERTHFLG_RSBERTHFA_F )
#define	BER_MONICONFIG_PATTERN7	( PARAM_PEREN_PERENC_F | PARAM_PEREN_PERENB_F | PARAM_PEREN_PERENA_F )
	u32 d32;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (BER_user == NULL) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

	memset(BER, 0, tmpsize);
	if (copy_from_user(BER, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto ber_moniconfig_return;
	}

	//if ((BER->S8WAIT & ~BER_MONICONFIG_PATTERN1) != 0) {
	//	rtncode = -EINVAL;
	//	goto ber_moniconfig_return;
	//}

	if ((BER->VBERON != PARAM_VBERON_BEFORE) && (BER->VBERON != 0)) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

	if ((BER->VBERXRST & ~BER_MONICONFIG_PATTERN2) != 0) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

	if (BER->VBERSETA > 16777215) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

/*	if (BER->VBERSETB > 16777215) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

	if (BER->VBERSETC > 16777215) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}
*/
	if ((BER->RSBERON & ~BER_MONICONFIG_PATTERN3) != 0) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

	if ((BER->RSBERXRST & ~BER_MONICONFIG_PATTERN4) != 0) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

	if ((BER->RSBERCEFLG & ~BER_MONICONFIG_PATTERN5) != 0) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

/*	if ((BER->RSBERTHFLG & ~BER_MONICONFIG_PATTERN6) != 0) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}
*/
	if (BER->SBERSETA > 65535) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

/*	if (BER->SBERSETB > 65535) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

	if (BER->SBERSETC > 65535) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}
*/
	if ((BER->PEREN & ~BER_MONICONFIG_PATTERN7) != 0) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

	if (BER->PERSNUMA > 65535) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

/*	if (BER->PERSNUMB > 65535) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}

	if (BER->PERSNUMC > 65535) {
		rtncode = -EINVAL;
		goto ber_moniconfig_return;
	}
*/
	sreg = MB86A30V_REG_SUBR_VBERSETA0;
	/* VBERSETA - C Setting */
	d32 = BER->VBERSETA;
	value = (d32 >> 16) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	value = (d32 >> 8) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	value = (d32) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	/*
	sreg += 1;
	d32 = BER->VBERSETB;
	value = (d32 >> 16) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	value = (d32 >> 8) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	value = (d32) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	d32 = BER->VBERSETC;
	value = (d32 >> 16) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	value = (d32 >> 8) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	value = (d32) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	*/
	reg = MB86A30V_REG_ADDR_VBERON;
	value = BER->VBERON;
	mb86a30v_i2c_master_send(reg, value);

	reg = MB86A30V_REG_ADDR_VBERXRST;
	value = BER->VBERXRST;
	mb86a30v_i2c_master_send(reg, value);

	/* SBERSETA - C Setting */
	sreg = MB86A30V_REG_SUBR_SBERSETA0;
	d32 = BER->SBERSETA;
	value = (d32 >> 8) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	value = (d32) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	/*
	sreg += 1;
	d32 = BER->SBERSETB;
	value = (d32 >> 8) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	value = (d32) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	d32 = BER->SBERSETC;
	value = (d32 >> 8) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	value = (d32) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	*/
	reg = MB86A30V_REG_ADDR_RSBERON;
	value = BER->RSBERON;
	mb86a30v_i2c_master_send(reg, value);

	reg = MB86A30V_REG_ADDR_RSBERXRST;
	value = BER->RSBERXRST;
	mb86a30v_i2c_master_send(reg, value);

	/* PERSNUMA - C Setting */
	sreg = MB86A30V_REG_SUBR_PERSNUMA0;
	d32 = BER->PERSNUMA;
	value = (d32 >> 8) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	value = (d32) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	/*
	sreg += 1;
	d32 = BER->PERSNUMB;
	value = (d32 >> 8) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	value = (d32) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	d32 = BER->PERSNUMC;
	value = (d32 >> 8) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	sreg += 1;
	value = (d32) & 0xff;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	*/
	sreg = MB86A30V_REG_SUBR_PEREN;
	value = BER->PEREN & 0x0f;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

ber_moniconfig_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_BER_MONISTART command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_BER_MONISTART(mb86a30v_cmdcontrol_t * cmdctrl,
				unsigned int cmd, unsigned long arg)
{
	ioctl_ber_moni_t *BER = &cmdctrl->BER;
	int rtncode = 0;
	unsigned char reg;
	unsigned char rega = MB86A30V_REG_ADDR_FEC_SUBA;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_FEC_SUBD;
	unsigned int value;
	u8 VBERON = 0;
	u8 VBERXRST = 0;
	u8 RSBERON = 0;
	u8 PEREN = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	VBERON = BER->VBERON & PARAM_VBERON_BEFORE;
	if (BER->VBERON == PARAM_VBERON_BEFORE) {
		VBERXRST = BER->VBERXRST & (PARAM_VBERXRST_VBERXRSTC_E
					    | PARAM_VBERXRST_VBERXRSTB_E
					    | PARAM_VBERXRST_VBERXRSTA_E);
		BER->VBERXRST_SAVE	= VBERXRST;

		if (VBERXRST != 0) {
			reg = MB86A30V_REG_ADDR_VBERXRST;
			value = 0;
			mb86a30v_i2c_master_send(reg, value);

			value = VBERXRST;
			mb86a30v_i2c_master_send(reg, value);
		}
	}

	RSBERON = BER->RSBERON & (PARAM_RSBERON_RSBERC_B
				  | PARAM_RSBERON_RSBERB_B
				  | PARAM_RSBERON_RSBERA_B);
	if (RSBERON != 0) {
		reg = MB86A30V_REG_ADDR_RSBERON;
		RSBERON |= BER->RSBERON & PARAM_RSBERON_RSBERAUTO_A;
		value = RSBERON;
		mb86a30v_i2c_master_send(reg, value);

		reg = MB86A30V_REG_ADDR_RSBERXRST;
		value = 0;
		mb86a30v_i2c_master_send(reg, value);

		value = BER->RSBERXRST;
		mb86a30v_i2c_master_send(reg, value);
	}

	PEREN = BER->PEREN & (PARAM_PEREN_PERENC_F
			      | PARAM_PEREN_PERENB_F | PARAM_PEREN_PERENA_F);
	if (PEREN != 0) {
		sreg = MB86A30V_REG_SUBR_PERRST;
		value = BER->PERRST;
		mb86a30v_i2c_slave_send(rega, sreg, regd, value);

		value = 0;
		mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	}
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_BER_MONIGET command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_BER_MONIGET(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			      unsigned long arg)
{
	ioctl_ber_moni_t *BER_user = (ioctl_ber_moni_t *) arg;
	ioctl_ber_moni_t *BER = &cmdctrl->BER;
	size_t cpysize = (sizeof(u32) * 6) + (sizeof(u16) * 3);
	int rtncode = 0;
	unsigned char reg;
	unsigned char rega = MB86A30V_REG_ADDR_FEC_SUBA;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_FEC_SUBD;
	unsigned int value = 0;
	u8 tmpdata = 0;
	u8 VBERFLG = 0;
	u8 RSBERCEFLG = 0;
	u8 PERFLG = 0;
	u8 CHECKFLG = 0;
	int loop = 5;
	int automode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (BER_user == NULL) {
		rtncode = -EINVAL;
		goto ber_moniget_return;
	}

	if ((BER->RSBERON & PARAM_RSBERON_RSBERAUTO_A) ==
	    PARAM_RSBERON_RSBERAUTO_A) {
		/*** WAIT ***/
		mdelay(1000);

		automode = 1;
	}

	/*********************************************/
/*	reg = MB86A30V_REG_ADDR_RSBERTHFLG;
	rtncode = mb86a30v_i2c_master_recv(reg, &BER->RSBERTHFLG, 1);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto ber_moniget_return;
	}
	rtncode = put_user(BER->RSBERTHFLG, (char *)&BER_user->RSBERTHFLG);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto ber_moniget_return;
	}
*/	
	reg = MB86A30V_REG_ADDR_RSBERRST;
	rtncode = mb86a30v_i2c_master_recv(reg, &BER->RSBERXRST, 1);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto ber_moniget_return;
	}
	rtncode = put_user(BER->RSBERXRST, (char *)&BER_user->RSBERXRST);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto ber_moniget_return;
	}

	sreg = MB86A30V_REG_SUBR_PEREN;
	rtncode = mb86a30v_i2c_slave_recv(rega, sreg, regd, &BER->PEREN, 1);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto ber_moniget_return;
	}
	rtncode = put_user(BER->PEREN, (char *)&BER_user->PEREN);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto ber_moniget_return;
	}

	sreg = MB86A30V_REG_SUBR_PERRST;
	rtncode = mb86a30v_i2c_slave_recv(rega, sreg, regd, &BER->PERRST, 1);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto ber_moniget_return;
	}
	rtncode = put_user(BER->PERRST, (char *)&BER_user->PERRST);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto ber_moniget_return;
	}

	/*********************************************/
	CHECKFLG = BER->VBERXRST & (PARAM_VBERXRST_VBERXRSTC_E
				    | PARAM_VBERXRST_VBERXRSTB_E
				    | PARAM_VBERXRST_VBERXRSTA_E);
	reg = MB86A30V_REG_ADDR_VBERFLG;
	loop = 5;
	while (loop--) {
		rtncode = mb86a30v_i2c_master_recv(reg, &BER->VBERFLG, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		if ((BER->VBERFLG & CHECKFLG) == CHECKFLG)
			break;

		mdelay(5);
	}

	rtncode = put_user(BER->VBERFLG, (char *)&BER_user->VBERFLG);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto ber_moniget_return;
	}

	if( automode == 1 )	goto moniget_automode;

	CHECKFLG = BER->RSBERON & (PARAM_RSBERON_RSBERC_B
				   | PARAM_RSBERON_RSBERB_B
				   | PARAM_RSBERON_RSBERA_B);
	reg = MB86A30V_REG_ADDR_RSBERCEFLG;
	loop = 5;
	while (loop--) {
		rtncode = mb86a30v_i2c_master_recv(reg, &BER->RSBERCEFLG, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		if ((BER->RSBERCEFLG & CHECKFLG) == CHECKFLG)
			break;

		mdelay(5);
	}

	rtncode = put_user(BER->RSBERCEFLG, (char *)&BER_user->RSBERCEFLG);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto ber_moniget_return;
	}

moniget_automode:
	CHECKFLG = BER->PEREN & (PARAM_PEREN_PERENC_F
				 | PARAM_PEREN_PERENB_F | PARAM_PEREN_PERENA_F);
	sreg = MB86A30V_REG_SUBR_PERFLG;
	loop = 5;
	while (loop--) {
		rtncode =
		    mb86a30v_i2c_slave_recv(rega, sreg, regd, &BER->PERFLG, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		if ((BER->PERFLG & CHECKFLG) == CHECKFLG)
			break;

		mdelay(5);
	}

	rtncode = put_user(BER->PERFLG, (char *)&BER_user->PERFLG);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto ber_moniget_return;
	}

	/*********************************************/
	if (BER->VBERFLG & PARAM_VBERFLG_VBERFLGA) {
		BER->VBERDTA = 0;
		reg = MB86A30V_REG_ADDR_VBERDTA0;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->VBERDTA += ((tmpdata & 0x0F) << 16);

		reg = MB86A30V_REG_ADDR_VBERDTA1;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->VBERDTA += (tmpdata << 8);

		reg = MB86A30V_REG_ADDR_VBERDTA2;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->VBERDTA += tmpdata;
	}

	/*********************************************/
/*	BER->VBERDTB = 0;
	if (BER->VBERFLG & PARAM_VBERFLG_VBERFLGB) {
		reg = MB86A30V_REG_ADDR_VBERDTB0;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->VBERDTB += ((tmpdata & 0x0F) << 16);

		reg = MB86A30V_REG_ADDR_VBERDTB1;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->VBERDTB += (tmpdata << 8);

		reg = MB86A30V_REG_ADDR_VBERDTB2;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->VBERDTB += tmpdata;
	}
*/
	/*********************************************/
/*	BER->VBERDTC = 0;
	if (BER->VBERFLG & PARAM_VBERFLG_VBERFLGC) {
		reg = MB86A30V_REG_ADDR_VBERDTC0;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->VBERDTC += ((tmpdata & 0x0F) << 16);

		reg = MB86A30V_REG_ADDR_VBERDTC1;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->VBERDTC += (tmpdata << 8);

		reg = MB86A30V_REG_ADDR_VBERDTC2;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->VBERDTC += tmpdata;
	}
*/
	/*********************************************/
	BER->RSBERDTA = 0;
	if ((automode == 1)
	|| (BER->RSBERCEFLG & PARAM_RSBERCEFLG_SBERCEFA)) {
		reg = MB86A30V_REG_ADDR_RSBERDTA0;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->RSBERDTA += ((tmpdata & 0x0F) << 16);

		reg = MB86A30V_REG_ADDR_RSBERDTA1;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->RSBERDTA += (tmpdata << 8);

		reg = MB86A30V_REG_ADDR_RSBERDTA2;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->RSBERDTA += tmpdata;
	}

	/*********************************************/
/*	BER->RSBERDTB = 0;
	if ((automode == 1)
	|| (BER->RSBERCEFLG & PARAM_RSBERCEFLG_SBERCEFB)) {
		reg = MB86A30V_REG_ADDR_RSBERDTB0;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->RSBERDTB += ((tmpdata & 0x0F) << 16);

		reg = MB86A30V_REG_ADDR_RSBERDTB1;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->RSBERDTB += (tmpdata << 8);

		reg = MB86A30V_REG_ADDR_RSBERDTB2;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->RSBERDTB += tmpdata;
	}
*/
	/*********************************************/
/*	BER->RSBERDTC = 0;
	if ((automode == 1)
	|| (BER->RSBERCEFLG & PARAM_RSBERCEFLG_SBERCEFC)) {
		reg = MB86A30V_REG_ADDR_RSBERDTC0;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->RSBERDTC += ((tmpdata & 0x0F) << 16);

		reg = MB86A30V_REG_ADDR_RSBERDTC1;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->RSBERDTC += (tmpdata << 8);

		reg = MB86A30V_REG_ADDR_RSBERDTC2;
		rtncode = mb86a30v_i2c_master_recv(reg, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->RSBERDTC += tmpdata;
	}
*/
	/*********************************************/
	BER->PERERRA = 0;
	if (BER->PERFLG & PARAM_PERFLG_PERFLGA) {
		sreg = MB86A30V_REG_SUBR_PERERRA0;
		rtncode = mb86a30v_i2c_slave_recv(rega, sreg, regd, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->PERERRA += (tmpdata << 8);

		sreg = MB86A30V_REG_SUBR_PERERRA1;
		rtncode = mb86a30v_i2c_slave_recv(rega, sreg, regd, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->PERERRA += tmpdata;
	}

	/*********************************************/
/*	BER->PERERRB = 0;
	if (BER->PERFLG & PARAM_PERFLG_PERFLGB) {
		sreg = MB86A30V_REG_SUBR_PERERRB0;
		rtncode = mb86a30v_i2c_slave_recv(rega, sreg, regd, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->PERERRB += (tmpdata << 8);

		value = 0;
		sreg = MB86A30V_REG_SUBR_PERERRB1;
		rtncode = mb86a30v_i2c_slave_recv(rega, sreg, regd, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->PERERRB += tmpdata;
	}
*/
	/*********************************************/
/*	BER->PERERRC = 0;
	if (BER->PERFLG & PARAM_PERFLG_PERFLGC) {
		sreg = MB86A30V_REG_SUBR_PERERRC0;
		rtncode = mb86a30v_i2c_slave_recv(rega, sreg, regd, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->PERERRC += (tmpdata << 8);

		value = 0;
		sreg = MB86A30V_REG_SUBR_PERERRC1;
		rtncode = mb86a30v_i2c_slave_recv(rega, sreg, regd, &tmpdata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto ber_moniget_return;
		}
		BER->PERERRC += tmpdata;
	}
*/
	/*********************************************/
	if (copy_to_user
	    ((void *)&BER_user->VBERDTA, (void *)&BER->VBERDTA, cpysize)) {
		rtncode = -EFAULT;
		goto ber_moniget_return;
	}

	/*********************************************//* Moniter restart */
	VBERFLG = BER->VBERFLG & (PARAM_VBERFLG_VBERFLGC
				  | PARAM_VBERFLG_VBERFLGB
				  | PARAM_VBERFLG_VBERFLGA);
	if (VBERFLG != 0) {
		reg = MB86A30V_REG_ADDR_VBERXRST;
		value = 0;
		mb86a30v_i2c_master_send(reg, value);

	//	value = VBERFLG;
		value = BER->VBERXRST_SAVE;
		mb86a30v_i2c_master_send(reg, value);
	}

	RSBERCEFLG = BER->RSBERCEFLG & (PARAM_RSBERCEFLG_SBERCEFC_E
					| PARAM_RSBERCEFLG_SBERCEFB_E
					| PARAM_RSBERCEFLG_SBERCEFA_E);
	if ( automode == 1 ) {
//		reg = MB86A30V_REG_ADDR_RSBERXRST;
//		value = 0;
//		mb86a30v_i2c_master_send(reg, value);
//
//		value = ( BER->RSBERON & (PARAM_RSBERON_RSBERC_B
//					  | PARAM_RSBERON_RSBERB_B
//					  | PARAM_RSBERON_RSBERA_B ));
//
//		mb86a30v_i2c_master_send(reg, value);
	} else if (RSBERCEFLG != 0) {
		reg = MB86A30V_REG_ADDR_RSBERXRST;
		value = ~RSBERCEFLG;
		mb86a30v_i2c_master_send(reg, value);

	//	value = RSBERCEFLG;
		value = ( BER->RSBERON & (PARAM_RSBERON_RSBERC_B
					  | PARAM_RSBERON_RSBERB_B
					  | PARAM_RSBERON_RSBERA_B ));

		mb86a30v_i2c_master_send(reg, value);
	}

	PERFLG = BER->PERFLG & (PARAM_PERFLG_PERFLGC
				| PARAM_PERFLG_PERFLGB | PARAM_PERFLG_PERFLGA);
	if (PERFLG != 0) {
		sreg = MB86A30V_REG_SUBR_PERRST;
		value = PERFLG;
		mb86a30v_i2c_slave_send(rega, sreg, regd, value);

		value = 0;
		mb86a30v_i2c_slave_send(rega, sreg, regd, value);
	}

ber_moniget_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_BER_MONISTOP command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_BER_MONISTOP(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			       unsigned long arg)
{
	int rtncode = 0;
	unsigned char reg;
	unsigned char rega = MB86A30V_REG_ADDR_FEC_SUBA;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_FEC_SUBD;
	unsigned int value;
	u8 rdata = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	reg = MB86A30V_REG_ADDR_RSBERON;
	rtncode = mb86a30v_i2c_master_recv(reg, &rdata, 1);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto ber_monistop_return;
	}

	reg = MB86A30V_REG_ADDR_VBERON;
	value = 0;
	mb86a30v_i2c_master_send(reg, value);

	reg = MB86A30V_REG_ADDR_RSBERON;
	value = 0;
	mb86a30v_i2c_master_send(reg, value);

	sreg = MB86A30V_REG_SUBR_PEREN;
	value = 0;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

ber_monistop_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_TS_START command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_TS_START(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			   unsigned long arg)
{
	ioctl_ts_t *TS_user = (ioctl_ts_t *) arg;
	ioctl_ts_t *TS = &cmdctrl->TS;
	size_t tmpsize = sizeof(ioctl_ts_t);
	int rtncode = 0;
	unsigned char rega = MB86A30V_REG_ADDR_FEC_SUBA;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_FEC_SUBD;
	unsigned int value = 0;
#define	TS_START_PATTERN1	( PARAM_RS0_RSEN_OFF | PARAM_RS0_RSEN_ON )
#define	TS_START_PATTERN2	( PARAM_SBER_SCLKSEL_OFF | PARAM_SBER_SCLKSEL_ON | PARAM_SBER_SBERSEL_OFF | PARAM_SBER_SBERSEL_ON	\
				| PARAM_SBER_SPACON_OFF | PARAM_SBER_SPACON_ON | PARAM_SBER_SENON_OFF | PARAM_SBER_SENON_ON		\
				| PARAM_SBER_SLAYER_OFF | PARAM_SBER_SLAYER_A | PARAM_SBER_SLAYER_B | PARAM_SBER_SLAYER_C		\
				| PARAM_SBER_SLAYER_AB | PARAM_SBER_SLAYER_AC | PARAM_SBER_SLAYER_BC | PARAM_SBER_SLAYER_ALL )
#define	TS_START_PATTERN3	( PARAM_SBER2_SCLKSEL_OFF | PARAM_SBER2_SCLKSEL_ON		\
				| PARAM_SBER2_SBERSEL_OFF | PARAM_SBER2_SBERSEL_ON		\
				| PARAM_SBER2_SPACON_OFF  | PARAM_SBER2_SPACON_ON		\
				| PARAM_SBER2_SENON_OFF   | PARAM_SBER2_SENON_ON		\
				| PARAM_SBER2_SLAYER_OFF  | PARAM_SBER2_SLAYER_A )

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (TS_user == NULL) {
		rtncode = -EINVAL;
		goto ts_start_return;
	}

	memset(TS, 0, tmpsize);
	if (copy_from_user(TS, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto ts_start_return;
	}

	if ((TS->RS0 & ~TS_START_PATTERN1) != 0) {
		rtncode = -EINVAL;
		goto ts_start_return;
	}

	if ((TS->SBER & ~TS_START_PATTERN2) != 0) {
		rtncode = -EINVAL;
		goto ts_start_return;
	}

/*	if ((TS->SBER2 & ~TS_START_PATTERN3) != 0) {
		rtncode = -EINVAL;
		goto ts_start_return;
	}
*/
	sreg = MB86A30V_REG_SUBR_RS0;
	value = TS->RS0;
	rtncode =
	    mb86a30v_i2c_slave_send_mask(rega, sreg, regd, value,
					MB86A30V_I2CMASK_RS0,
					MB86A30V_MASK_RS0_RSEN);
	if (rtncode != 0) {
		goto ts_start_return;
	}

	sreg = MB86A30V_REG_SUBR_SBER;
	value = TS->SBER;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

/*	sreg = MB86A30V_REG_SUBR_SBER2;
	value = TS->SBER2;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
*/
ts_start_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_TS_STOP command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_TS_STOP(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			  unsigned long arg)
{
	int rtncode = 0;
	unsigned char rega = MB86A30V_REG_ADDR_FEC_SUBA;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_FEC_SUBD;
	unsigned int value = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	sreg = MB86A30V_REG_SUBR_SBER;
	value = 0;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

/*	sreg = MB86A30V_REG_SUBR_SBER2;
	value = 0;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
*/
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_TS_CONFIG command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_TS_CONFIG(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			    unsigned long arg)
{
	ioctl_ts_t *TS_user = (ioctl_ts_t *) arg;
	ioctl_ts_t *TS = &cmdctrl->TS;
	size_t tmpsize = sizeof(ioctl_ts_t);
	int rtncode = 0;
	unsigned char rega = MB86A30V_REG_ADDR_FEC_SUBA;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_FEC_SUBD;
	unsigned int value = 0;
//	u8 TS_CONFIG_PATTERN5 = 0;
//	u8 TS_CONFIG_PATTERN6 = 0;
#define	TS_CONFIG_PATTERN1	( PARAM_TSOUT_TSERRINV_OFF | PARAM_TSOUT_TSERRINV_ON		\
				| PARAM_TSOUT_TSENINV_OFF | PARAM_TSOUT_TSENINV_ON		\
				| PARAM_TSOUT_TSERRMASK_OFF | PARAM_TSOUT_TSERRMASK_ON )
#define	TS_CONFIG_PATTERN2	( PARAM_TSOUT2_TSERRINV_OFF | PARAM_TSOUT2_TSERRINV_ON		\
				| PARAM_TSOUT2_TSENINV_OFF  | PARAM_TSOUT2_TSENINV_ON		\
				| PARAM_TSOUT2_TSSINV_MSB   | PARAM_TSOUT2_TSSINV_LSB )
#define	TS_CONFIG_PATTERN3	( PARAM_PBER_PLAYER_OFF | PARAM_PBER_PLAYER_A | PARAM_PBER_PLAYER_B | PARAM_PBER_PLAYER_C		\
				| PARAM_PBER_PLAYER_AB | PARAM_PBER_PLAYER_AC | PARAM_PBER_PLAYER_BC | PARAM_PBER_PLAYER_ALL )
#define	TS_CONFIG_PATTERN4	( PARAM_SBER_SCLKSEL_OFF | PARAM_SBER_SCLKSEL_ON | PARAM_SBER_SBERSEL_OFF | PARAM_SBER_SBERSEL_ON	\
				| PARAM_SBER_SPACON_OFF | PARAM_SBER_SPACON_ON | PARAM_SBER_SENON_OFF | PARAM_SBER_SENON_ON		\
				| PARAM_SBER_SLAYER_OFF | PARAM_SBER_SLAYER_A | PARAM_SBER_SLAYER_B | PARAM_SBER_SLAYER_C		\
				| PARAM_SBER_SLAYER_AB | PARAM_SBER_SLAYER_AC | PARAM_SBER_SLAYER_BC | PARAM_SBER_SLAYER_ALL )
#define	TS_CONFIG_PATTERN7	( PARAM_SBER2_SCLKSEL_OFF | PARAM_SBER2_SCLKSEL_ON		\
				| PARAM_SBER2_SBERSEL_OFF | PARAM_SBER2_SBERSEL_ON		\
				| PARAM_SBER2_SPACON_OFF | PARAM_SBER2_SPACON_ON		\
				| PARAM_SBER2_SENON_OFF | PARAM_SBER2_SENON_ON )

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (TS_user == NULL) {
		rtncode = -EINVAL;
		goto ts_config_return;
	}

	memset(TS, 0, tmpsize);
	if (copy_from_user(TS, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto ts_config_return;
	}

	if ((TS->TSOUT & ~TS_CONFIG_PATTERN1) != 0) {
		rtncode = -EINVAL;
		goto ts_config_return;
	}

/*	if ((TS->TSOUT2 & ~TS_CONFIG_PATTERN2) != 0) {
		rtncode = -EINVAL;
		goto ts_config_return;
	}
*/
	if ((TS->PBER & ~TS_CONFIG_PATTERN3) != 0) {
		rtncode = -EINVAL;
		goto ts_config_return;
	}

	if ((TS->SBER & ~TS_CONFIG_PATTERN4) != 0) {
		rtncode = -EINVAL;
		goto ts_config_return;
	}

/*	TS_CONFIG_PATTERN5 = TS->PBER2 & 0x07;
	if ((TS_CONFIG_PATTERN5 != PARAM_PBER2_PLAYER_OFF)
	    && (TS_CONFIG_PATTERN5 != PARAM_PBER2_PLAYER_A)) {
		rtncode = -EINVAL;
		goto ts_config_return;
	}

	TS_CONFIG_PATTERN6 = TS->SBER2 & 0xF8;
	if ((TS_CONFIG_PATTERN6 & ~TS_CONFIG_PATTERN7) != 0) {
		rtncode = -EINVAL;
		goto ts_config_return;
	} else {
		TS_CONFIG_PATTERN6 = TS->SBER2 & 0x07;
		if ((TS_CONFIG_PATTERN6 != PARAM_SBER2_SLAYER_OFF)
		    && (TS_CONFIG_PATTERN6 != PARAM_SBER2_SLAYER_A)) {
			rtncode = -EINVAL;
			goto ts_config_return;
		}
	}
*/
	sreg = MB86A30V_REG_SUBR_TSOUT;
	value = TS->TSOUT;
	rtncode =
	    mb86a30v_i2c_slave_send_mask(rega, sreg, regd, value,
					MB86A30V_I2CMASK_TSOUT, 0xFF);
	if (rtncode != 0) {
		goto ts_config_return;
	}

/*	sreg = MB86A30V_REG_SUBR_TSOUT2;
	value = TS->TSOUT2;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
*/
	sreg = MB86A30V_REG_SUBR_PBER;
	value = TS->PBER;
	rtncode =
	    mb86a30v_i2c_slave_send_mask(rega, sreg, regd, value,
					MB86A30V_I2CMASK_PBER, 0xFF);
	if (rtncode != 0) {
		goto ts_config_return;
	}

	sreg = MB86A30V_REG_SUBR_SBER;
	value = TS->SBER;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

/*	sreg = MB86A30V_REG_SUBR_PBER2;
	value = TS->PBER2;
	rtncode =
	    mb86a30v_i2c_slave_send_mask(rega, sreg, regd, value,
					MB86A30V_I2CMASK_PBER, 0xFF);
	if (rtncode != 0) {
		goto ts_config_return;
	}

	sreg = MB86A30V_REG_SUBR_SBER2;
	value = TS->SBER2;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
*/
ts_config_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_TS_PCLOCK command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_TS_PCLOCK(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			    unsigned long arg)
{
	ioctl_ts_t *TS_user = (ioctl_ts_t *) arg;
	ioctl_ts_t *TS = &cmdctrl->TS;
	size_t tmpsize = sizeof(ioctl_ts_t);
	int rtncode = 0;
	unsigned char rega = MB86A30V_REG_ADDR_FEC_SUBA;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_FEC_SUBD;
	unsigned int value = 0;
#define	TS_PCLOCK_PATTERN1	( PARAM_SBER_SCLKSEL_OFF  | PARAM_SBER_SCLKSEL_ON  )
#define	TS_PCLOCK_PATTERN2	( PARAM_SBER2_SCLKSEL_OFF | PARAM_SBER2_SCLKSEL_ON )

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (TS_user == NULL) {
		rtncode = -EINVAL;
		goto ts_pclock_return;
	}

	memset(TS, 0, tmpsize);
	if (copy_from_user(TS, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto ts_pclock_return;
	}

	if ((TS->SBER & ~TS_PCLOCK_PATTERN1) != 0) {
		rtncode = -EINVAL;
		goto ts_pclock_return;
	}

/*	if ((TS->SBER2 & ~TS_PCLOCK_PATTERN2) != 0) {
		rtncode = -EINVAL;
		goto ts_pclock_return;
	}
*/
	sreg = MB86A30V_REG_SUBR_SBER;
	value = TS->SBER;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

/*	sreg = MB86A30V_REG_SUBR_SBER2;
	value = TS->SBER2;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);
*/
ts_pclock_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_TS_OUTMASK command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_TS_OUTMASK(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			     unsigned long arg)
{
	ioctl_ts_t *TS_user = (ioctl_ts_t *) arg;
	ioctl_ts_t *TS = &cmdctrl->TS;
	size_t tmpsize = sizeof(ioctl_ts_t);
	int rtncode = 0;
	unsigned char rega = MB86A30V_REG_ADDR_FEC_SUBA;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_FEC_SUBD;
	unsigned int value = 0;
#define	TS_OUTMASK_PATTERN1	( PARAM_TSMASK0_TSFRMMASK_OFF | PARAM_TSMASK0_TSFRMMASK_ON \
				| PARAM_TSMASK0_TSERRMASK_OFF | PARAM_TSMASK0_TSERRMASK_ON \
				| PARAM_TSMASK0_TSPACMASK_OFF | PARAM_TSMASK0_TSPACMASK_ON \
				| PARAM_TSMASK0_TSENMASK_OFF  | PARAM_TSMASK0_TSENMASK_ON  \
				| PARAM_TSMASK0_TSDTMASK_OFF  | PARAM_TSMASK0_TSDTMASK_ON  \
				| PARAM_TSMASK0_TSCLKMASK_OFF | PARAM_TSMASK0_TSCLKMASK_ON )
#define	TS_OUTMASK_PATTERN2	( PARAM_TSMASK1_TSFRMMASK_OFF | PARAM_TSMASK1_TSFRMMASK_ON \
				| PARAM_TSMASK1_TSERRMASK_OFF | PARAM_TSMASK1_TSERRMASK_ON \
				| PARAM_TSMASK1_TSPACMASK_OFF | PARAM_TSMASK1_TSPACMASK_ON \
				| PARAM_TSMASK1_TSENMASK_OFF  | PARAM_TSMASK1_TSENMASK_ON  \
				| PARAM_TSMASK1_TSDTMASK_OFF  | PARAM_TSMASK1_TSDTMASK_ON  \
				| PARAM_TSMASK1_TSCLKMASK_OFF | PARAM_TSMASK1_TSCLKMASK_ON )

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (TS_user == NULL) {
		rtncode = -EINVAL;
		goto ts_outmask_return;
	}

	memset(TS, 0, tmpsize);
	if (copy_from_user(TS, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto ts_outmask_return;
	}

	if ((TS->TSMASK0 & ~TS_OUTMASK_PATTERN1) != 0) {
		rtncode = -EINVAL;
		goto ts_outmask_return;
	}

/*	if ((TS->TSMASK1 & ~TS_OUTMASK_PATTERN2) != 0) {
		rtncode = -EINVAL;
		goto ts_outmask_return;
	}
*/
	sreg = MB86A30V_REG_SUBR_TSMASK;
	value = TS->TSMASK0;
	rtncode =
	    mb86a30v_i2c_slave_send_mask(rega, sreg, regd, value,
					MB86A30V_I2CMASK_TSMASK0, 0xFF);
	if (rtncode != 0) {
		goto ts_outmask_return;
	}

/*	sreg = MB86A30V_REG_SUBR_TSMASK1;
	value = TS->TSMASK1;
	rtncode =
	    mb86a30v_i2c_slave_send_mask(rega, sreg, regd, value,
					MB86A30V_I2CMASK_TSMASK1, 0xFF);
	if (rtncode != 0) {
		goto ts_outmask_return;
	}
*/
ts_outmask_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}


/************************************************************************/
/** -- long (2011/08/19) --
	ioctl System Call.  IOCTL_IRQ_GETREASON command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_IRQ_GETREASON(mb86a30v_cmdcontrol_t * cmdctrl,
				unsigned int cmd, unsigned long arg)
{
	ioctl_irq_t *IRQ_user = (ioctl_irq_t *) arg;
	ioctl_irq_t *IRQ = &cmdctrl->IRQ;
	size_t tmpsize = sizeof(ioctl_irq_t);
	int rtncode = 0;
	unsigned char reg;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (IRQ_user == NULL) {
		rtncode = -EINVAL;
		goto irq_getreason_return;
	}
	memset(IRQ, 0, tmpsize);
	if (copy_from_user(IRQ, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto irq_getreason_return;
	}

	reg = 0x6c;
	rtncode = mb86a30v_i2c_master_recv(reg, &IRQ->TMCC_IRQ, 1);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto irq_getreason_return;
	}
	IRQ->TMCC_IRQ &= (MB86A30V_MASK_TMCC_IRQ_EMG
			 | MB86A30V_MASK_TMCC_IRQ_CNT | MB86A30V_MASK_TMCC_IRQ_ILL);

	reg = 0x6f;
	rtncode = mb86a30v_i2c_master_recv(reg, &IRQ->SBER_IRQ, 1);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto irq_getreason_return;
	}
	IRQ->SBER_IRQ &= (MB86A30V_MASK_SBER_IRQ_TSERRA
			 | MB86A30V_MASK_SBER_IRQ_SBERCEFA);

	reg = 0xed;
	rtncode = mb86a30v_i2c_master_recv(reg, &IRQ->SEARCH_IRQ, 1);
	if (rtncode != 0) {
		rtncode = -EFAULT;
		goto irq_getreason_return;
	}
	IRQ->SEARCH_IRQ &= ( MB86A30V_SEARCH_IRQ_3WIRE_END | MB86A30V_SEARCH_IRQ_CHSEL_END | MB86A30V_SEARCH_IRQ_SEARCH);

	if ((IRQ->TMCC_IRQ | IRQ->SBER_IRQ | IRQ->SEARCH_IRQ) != 0) {
		IRQ->irq = PARAM_IRQ_YES;
	} else {
		IRQ->irq = PARAM_IRQ_NON;
	}

	if (copy_to_user((void *)IRQ_user, (void *)IRQ, tmpsize)) {
		rtncode = -EFAULT;
		goto irq_getreason_return;
	}

irq_getreason_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/** -- long (2011/08/18) --
	ioctl System Call.  IOCTL_IRQ_SETMASK command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_IRQ_SETMASK(mb86a30v_cmdcontrol_t * cmdctrl,
			      unsigned int cmd, unsigned long arg)
{
	ioctl_irq_t *IRQ_user = (ioctl_irq_t *) arg;
	ioctl_irq_t *IRQ = &cmdctrl->IRQ;
	size_t tmpsize = sizeof(ioctl_irq_t);
	unsigned char rega = MB86A30V_REG_ADDR_TMCC_SUBA;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_TMCC_SUBD;
	unsigned char reg = 0;
	unsigned int value = 0;
	int rtncode = 0;
#define	IRQ_SETMASK_PATTERN1	( PARAM_TMCC_IRQ_MASK_EMG	\
				| PARAM_TMCC_IRQ_MASK_CNTDOWN	\
				| PARAM_TMCC_IRQ_MASK_ILL )
#define	IRQ_SETMASK_PATTERN2	( PARAM_SBER_IRQ_MASK_CEMASKA )
#define	IRQ_SETMASK_PATTERN3	( PARAM_TSERR_IRQ_MASK_A )
#define	IRQ_SETMASK_PATTERN4	( PARAM_TMCC_IRQ_RST_EMG	\
				| PARAM_TMCC_IRQ_RST_CNTDOWN	\
				| PARAM_TMCC_IRQ_RST_ILL )
//#define	IRQ_SETMASK_PATTERN5	( PARAM_RSBERON_RSBERA_S    | PARAM_RSBERON_RSBERA_B )
#define	IRQ_SETMASK_PATTERN6	( PARAM_RSBERRST_SBERXRSTA_R | PARAM_RSBERRST_SBERXRSTA_N )

//#define	IRQ_SETMASK_PATTERN7	( PARAM_RSBERCEFLG_SBERCEFC_E | PARAM_RSBERCEFLG_SBERCEFB_E | PARAM_RSBERCEFLG_SBERCEFA_E )

#define	IRQ_SETMASK_PATTERN9	( PARAM_RSERRFLG_RSERRSTA_R	| PARAM_RSERRFLG_RSERRA_F )
/*
#define	IRQ_SETMASK_PATTERN10	( PARAM_FECIRQ1_RSERRA	\
				| PARAM_FECIRQ1_LOCK   | PARAM_FECIRQ1_EMG   | PARAM_FECIRQ1_CNT | PARAM_FECIRQ1_ILL )
#define	IRQ_SETMASK_PATTERN11	( PARAM_XIRQINV_LOW | PARAM_XIRQINV_HIGH )
				*/
#define	IRQ_SETMASK_PATTERN12		( PARAM_SEARCH_IRQCTL_IRQ_MASK | PARAM_SEARCH_IRQCTL_CHSEL_MASK | PARAM_SEARCH_IRQCTL_3WIRE_MASK )
#define IRQ_SETMASK_PATTERN13		( PARAM_SEARCH_IRQCTL_IRQ_RST | PARAM_SEARCH_IRQCTL_CHSEL_RST | PARAM_SEARCH_IRQCTL_3WIRE_RST )

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (IRQ_user == NULL) {
		rtncode = -EINVAL;
		goto irq_setmask_return;
	}

	memset(IRQ, 0, tmpsize);
	if (copy_from_user(IRQ, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto irq_setmask_return;
	}

	if ((IRQ->TMCC_IRQ_MASK & ~IRQ_SETMASK_PATTERN1) != 0) {
		rtncode = -EINVAL;
		goto irq_setmask_return;
	}

	if ((IRQ->SBER_IRQ_MASK & ~IRQ_SETMASK_PATTERN2) != 0) {
		rtncode = -EINVAL;
		goto irq_setmask_return;
	}

	if ((IRQ->TSERR_IRQ_MASK & ~IRQ_SETMASK_PATTERN3) != 0) {
		rtncode = -EINVAL;
		goto irq_setmask_return;
	}

	if ((IRQ->TMCC_IRQ_RST & ~IRQ_SETMASK_PATTERN4) != 0) {
		rtncode = -EINVAL;
		goto irq_setmask_return;
	}

	//if ((IRQ->RSBERON & ~IRQ_SETMASK_PATTERN5) != 0) {
	//	rtncode = -EINVAL;
	//	goto irq_setmask_return;
	//}

	if ((IRQ->RSBERRST & ~IRQ_SETMASK_PATTERN6) != 0) {
		rtncode = -EINVAL;
		goto irq_setmask_return;
	}

	if ((IRQ->RSERRFLG & ~IRQ_SETMASK_PATTERN9) != 0) {
		rtncode = -EINVAL;
		goto irq_setmask_return;
	}
/*
	if ((IRQ->FECIRQ1 & ~IRQ_SETMASK_PATTERN10) != 0) {
		rtncode = -EINVAL;
		goto irq_setmask_return;
	}

	if ((IRQ->XIRQINV & ~IRQ_SETMASK_PATTERN11) != 0) {
		rtncode = -EINVAL;
		goto irq_setmask_return;
	}
*/
	if ((IRQ->SEARCH_IRQ_MASK & ~IRQ_SETMASK_PATTERN12) != 0) {
		rtncode = -EINVAL;
		goto irq_setmask_return;
	}

	if ((IRQ->SEARCH_IRQ_RST & ~IRQ_SETMASK_PATTERN13) != 0) {
		rtncode = -EINVAL;
		goto irq_setmask_return;
	}

	sreg = MB86A30V_REG_SUBR_TMCC_IRQ_MASK;
	value = IRQ->TMCC_IRQ_MASK;
	rtncode =
	    mb86a30v_i2c_slave_send_mask(rega, sreg, regd, value,
					MB86A30V_I2CMASK_TMCC_IRQ_MASK, 0xFF);
	if (rtncode != 0) {
		goto irq_setmask_return;
	}

	sreg = MB86A30V_REG_SUBR_SBER_IRQ_MASK;
	value = IRQ->SBER_IRQ_MASK;
	rtncode =
	    mb86a30v_i2c_slave_send_mask(0x50, sreg, 0x51, value,
					MB86A30V_I2CMASK_SBER_IRQ_MASK, 0xFF);
	if (rtncode != 0) {
		goto irq_setmask_return;
	}

	sreg = MB86A30V_REG_SUBR_TSERR_IRQ_MASK;
	value = IRQ->TSERR_IRQ_MASK;
	rtncode =
	    mb86a30v_i2c_slave_send_mask(0x50, sreg, 0x51, value,
					MB86A30V_I2CMASK_TSERR_IRQ_MASK, 0xFF);
	if (rtncode != 0) {
		goto irq_setmask_return;
	}

	sreg = MB86A30V_REG_SUBR_TMCC_IRQ_RST;
	value = IRQ->TMCC_IRQ_RST;
	rtncode =
	    mb86a30v_i2c_slave_send_mask(rega, sreg, regd, value,
					MB86A30V_I2CMASK_TMCC_IRQ_RST, 0xFF);
	if (rtncode != 0) {
		goto irq_setmask_return;
	}

	//reg = MB86A30V_REG_ADDR_RSBERON;
	//value = IRQ->RSBERON;
	//mb86a30v_i2c_master_send(reg, value);

	reg = MB86A30V_REG_ADDR_RSBERRST;
	value = IRQ->RSBERRST;
	mb86a30v_i2c_master_send(reg, value);
	
	reg = MB86A30V_REG_ADDR_RSERRFLG;
	value = IRQ->RSERRFLG;
	mb86a30v_i2c_master_send(reg, value);

	/*
	reg = MB86A30V_REG_ADDR_FECIRQ1;
	value = IRQ->FECIRQ1;
	mb86a30v_i2c_master_send(reg, value);
        */	
	/*
	reg = MB86A30V_REG_ADDR_XIRQINV;
	value = IRQ->XIRQINV;
	mb86a30v_i2c_master_send(reg, value);
	*/

	mb86a30v_i2c_master_send(0xee, IRQ->SEARCH_IRQ_RST | IRQ->SEARCH_IRQ_MASK);

irq_setmask_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_IRQ_TMCCPARAM_SET command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_IRQ_TMCCPARAM_SET(mb86a30v_cmdcontrol_t * cmdctrl,
				    unsigned int cmd, unsigned long arg)
{
	ioctl_irq_t *IRQ_user = (ioctl_irq_t *) arg;
	ioctl_irq_t *IRQ = &cmdctrl->IRQ;
	size_t tmpsize = sizeof(ioctl_irq_t);
	unsigned char rega = MB86A30V_REG_ADDR_TMCC_SUBA;
	unsigned char sreg;
	unsigned char regd = MB86A30V_REG_ADDR_TMCC_SUBD;
	unsigned int value = 0;
	int rtncode = 0;
#define	IRQ_TMCCPARAM_PATTERN1	( PARAM_TMCCCHK_14 | PARAM_TMCCCHK_13 | PARAM_TMCCCHK_12	\
				| PARAM_TMCCCHK_11 | PARAM_TMCCCHK_10 | PARAM_TMCCCHK_9 | PARAM_TMCCCHK_8 )
#define	IRQ_TMCCPARAM_PATTERN2	( PARAM_EMG_INV_RECV | PARAM_EMG_INV_NORECV )

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (IRQ_user == NULL) {
		rtncode = -EINVAL;
		goto irq_tmccparam_set_return;
	}

	memset(IRQ, 0, tmpsize);
	if (copy_from_user(IRQ, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto irq_tmccparam_set_return;
	}

	if ((IRQ->EMG_INV & ~IRQ_TMCCPARAM_PATTERN2) != 0) {
		rtncode = -EINVAL;
		goto irq_tmccparam_set_return;
	}

	sreg = MB86A30V_REG_SUBR_TMCC_IRQ_EMG_INV;
	value = IRQ->EMG_INV;
	rtncode =
	    mb86a30v_i2c_slave_send_mask(rega, sreg, regd, value,
					MB86A30V_I2CMASK_EMG_INV,
					PARAM_EMG_INV_NORECV);
	if (rtncode != 0) {
		goto irq_tmccparam_set_return;
	}

irq_tmccparam_set_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_CN_MONI command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_CN_MONI(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			  unsigned long arg)
{
	ioctl_cn_moni_t *CN_user = (ioctl_cn_moni_t *) arg;
	ioctl_cn_moni_t *CN = &cmdctrl->CN;
	size_t tmpsize = sizeof(ioctl_cn_moni_t);
	int rtncode = 0;
	unsigned char reg;
	unsigned int value = 0;
	int indx = 0;
	int mode = 0;
	int loop = 0;
	u8 flag = 0;
	u8 cncnt = 0;
	u8 cndata;
	int wait_time = 0;
	u16 tmpdata = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (CN_user == NULL) {
		rtncode = -EINVAL;
		goto cn_moni_return;
	}

	memset(CN, 0, tmpsize);
	if (copy_from_user(CN, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto cn_moni_return;
	}

	if (CN->CNCNT > 15) {
		rtncode = -EINVAL;
		goto cn_moni_return;
	}

	if ((CN->CNCNT2 & ~PARAM_CNCNT2_MODE_MUNL) != 0) {
		rtncode = -EINVAL;
		goto cn_moni_return;
	}

	if ((CN->CNCNT & ~MB86A30V_MASK_CNCNT_SYMCOUNT) != 0) {
		rtncode = -EINVAL;
		goto cn_moni_return;
	}

	loop = CN->cn_count;
	mode = CN->CNCNT2 & PARAM_CNCNT2_MODE_MUNL;

	cncnt = (CN->CNCNT & MB86A30V_MASK_CNCNT_SYMCOUNT);
	CN->CNCNT = cncnt;
	wait_time = MB86A30V_CN_MONI_WAITTIME(cncnt);

	reg = MB86A30V_REG_ADDR_CNCNT2;
	value = mode;
	mb86a30v_i2c_master_send(reg, value);

	reg = MB86A30V_REG_ADDR_CNCNT;
	value = CN->CNCNT;
	mb86a30v_i2c_master_send(reg, value);

	/* RST = 1 */
	reg = MB86A30V_REG_ADDR_CNCNT;
	value = (CN->CNCNT | MB86A30V_CNCNT_RST);
	mb86a30v_i2c_master_send(reg, value);

	/* RST = 0 */
	reg = MB86A30V_REG_ADDR_CNCNT;
	value = cncnt;
	mb86a30v_i2c_master_send(reg, value);

	for (indx = 0; indx < loop; indx++) {
		if (mode == PARAM_CNCNT2_MODE_AUTO) {
			/* PARAM_CNCNT2_MODE_AUTO */
			/*** WAIT ***/
			mdelay(wait_time);

			reg = MB86A30V_REG_ADDR_CNCNT;
			value = (CN->CNCNT | MB86A30V_CNCNT_LOCK);
			mb86a30v_i2c_master_send(reg, value);
			reg = MB86A30V_REG_ADDR_CNDATHI; /* 20110404 MB86A30V ES2.5 */
			mb86a30v_i2c_master_send(reg, 0);
		} else {
			/* PARAM_CNCNT2_MODE_MUNL */
			reg = MB86A30V_REG_ADDR_CNCNT;
			while ((flag & MB86A30V_CNCNT_FLG) == MB86A30V_CNCNT_FLG) {
				rtncode =
				    mb86a30v_i2c_master_recv(reg, &flag, 1);
				if (rtncode != 0) {
					rtncode = -EFAULT;
					goto cn_moni_return;
				}
			}
		}
		tmpdata = 0;
		reg = MB86A30V_REG_ADDR_CNDATHI;
		rtncode = mb86a30v_i2c_master_recv(reg, &cndata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto cn_moni_return;
		}
		tmpdata = (cndata << 8);
		reg = MB86A30V_REG_ADDR_CNDATLO;
		rtncode = mb86a30v_i2c_master_recv(reg, &cndata, 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto cn_moni_return;
		}
		tmpdata += cndata;
		if (mode == PARAM_CNCNT2_MODE_AUTO) {
			/* PARAM_CNCNT2_MODE_AUTO */
			reg = MB86A30V_REG_ADDR_CNCNT;
			value = cncnt;
			mb86a30v_i2c_master_send(reg, value);
		}
		if (copy_to_user
		    ((void *)&CN_user->CNDATA[indx], (void *)&tmpdata, 2)) {
			rtncode = -EFAULT;
			goto cn_moni_return;
		}
		if (mode == PARAM_CNCNT2_MODE_MUNL) {
			/* PARAM_CNCNT2_MODE_MUNL */
			/* RST = 1 */
			reg = MB86A30V_REG_ADDR_CNCNT;
			value = (cncnt | MB86A30V_CNCNT_RST);
			mb86a30v_i2c_master_send(reg, value);

			/* RST = 0 */
			reg = MB86A30V_REG_ADDR_CNCNT;
			value = cncnt;
			mb86a30v_i2c_master_send(reg, value);
		}
	}

cn_moni_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/** -- long (2011/08/19) --
	ioctl System Call.  IOCTL_MER_MONI command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_MER_MONI(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			   unsigned long arg)
{
	ioctl_mer_moni_t *MER_user = (ioctl_mer_moni_t *) arg;
	ioctl_mer_moni_t *MER = &cmdctrl->MER;
	size_t tmpsize = sizeof(ioctl_mer_moni_t);
	int rtncode = 0;
	unsigned char rega = MB86A30V_REG_ADDR_FEC_SUBA;
	unsigned char sreg = 0;
	unsigned char regd = MB86A30V_REG_ADDR_FEC_SUBD;
	unsigned int value = 0;
	int indx = 0;
	int mode = 0;
	int loop = 0;
	int wait_time = 0;
	u8 flag = 0;
	u8 mercnt = 0;
	u8 merdataA[4];
	//u8 merdataB[4];
	//u8 merdataC[4];
	struct mer_data MERDATA;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (MER_user == NULL) {
		rtncode = -EINVAL;
		goto mer_moni_return;
	}

	memset(MER, 0, tmpsize);
	if (copy_from_user(MER, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto mer_moni_return;
	}

	if (MER->MERSTEP > 7) {
		rtncode = -EINVAL;
		goto mer_moni_return;
	}

	loop = MER->mer_count;
	mode = MER->MERCTRL & PARAM_MERCTRL_MODE_MUNL;

	mercnt = (MER->MERSTEP & MB86A30V_MASK_MERSTEP_SYMCOUNT);
	wait_time = MB86A30V_MER_MONI_WAITTIME(mercnt);

	sreg = MB86A30V_REG_SUBR_MERCTRL;
	value = mode;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

	sreg = MB86A30V_REG_SUBR_MERSTEP;
	value = mercnt;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

	/* RST = 1 */
	sreg = MB86A30V_REG_SUBR_MERCTRL;
	value = (mode | MB86A30V_MERCTRL_RST);
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

	/* RST = 0 */
	sreg = MB86A30V_REG_SUBR_MERCTRL;
	value = mode;
	mb86a30v_i2c_slave_send(rega, sreg, regd, value);

	MER->mer_flag = 1;

	if (copy_to_user((void *)MER_user, (void *)MER, tmpsize)) {
		rtncode = -EFAULT;
		goto mer_moni_return;
	}
	for (indx = 0; indx < loop; indx++) {
		if (mode == PARAM_MERCTRL_MODE_AUTO) {
			/* PARAM_MERCTRL_MODE_AUTO */

			/*** WAIT ***/
			mdelay(wait_time);

			sreg = MB86A30V_REG_SUBR_MERCTRL;
			value = MB86A30V_MERCTRL_LOCK;
			mb86a30v_i2c_slave_send(rega, sreg, regd, value);
		} else {
			/* PARAM_MERCTRL_MODE_MUNL */
			flag = 0;
			sreg = MB86A30V_REG_SUBR_MEREND;
			while ((flag & MB86A30V_MEREND_FLG) !=
			       MB86A30V_MEREND_FLG) {
				rtncode =
				    mb86a30v_i2c_slave_recv(rega, sreg, regd,
							   &flag, 1);
				if (rtncode != 0) {
					rtncode = -EFAULT;
					goto mer_moni_return;
				}
			}
		}
		sreg = MB86A30V_REG_SUBR_MERA0;
		rtncode =
		    mb86a30v_i2c_slave_recv(rega, sreg, regd, &merdataA[1], 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto mer_moni_return;
		}
		sreg = MB86A30V_REG_SUBR_MERA1;
		rtncode =
		    mb86a30v_i2c_slave_recv(rega, sreg, regd, &merdataA[2], 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto mer_moni_return;
		}
		sreg = MB86A30V_REG_SUBR_MERA2;
		rtncode =
		    mb86a30v_i2c_slave_recv(rega, sreg, regd, &merdataA[3], 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto mer_moni_return;
		}
		MERDATA.A =
		    ((merdataA[1] << 16) + (merdataA[2] << 8) + merdataA[3]);
/*
		sreg = MB86A30V_REG_SUBR_MERB0;
		rtncode =
		    mb86a30v_i2c_slave_recv(rega, sreg, regd, &merdataB[1], 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto mer_moni_return;
		}
		sreg = MB86A30V_REG_SUBR_MERB1;
		rtncode =
		    mb86a30v_i2c_slave_recv(rega, sreg, regd, &merdataB[2], 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto mer_moni_return;
		}
		sreg = MB86A30V_REG_SUBR_MERB2;
		rtncode =
		    mb86a30v_i2c_slave_recv(rega, sreg, regd, &merdataB[3], 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto mer_moni_return;
		}
		MERDATA.B =
		    ((merdataB[1] << 16) + (merdataB[2] << 8) + merdataB[3]);

		sreg = MB86A30V_REG_SUBR_MERC0;
		rtncode =
		    mb86a30v_i2c_slave_recv(rega, sreg, regd, &merdataC[1], 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto mer_moni_return;
		}
		sreg = MB86A30V_REG_SUBR_MERC1;
		rtncode =
		    mb86a30v_i2c_slave_recv(rega, sreg, regd, &merdataC[2], 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto mer_moni_return;
		}
		sreg = MB86A30V_REG_SUBR_MERC2;
		rtncode =
		    mb86a30v_i2c_slave_recv(rega, sreg, regd, &merdataC[3], 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto mer_moni_return;
		}
		MERDATA.C =
		    ((merdataC[1] << 16) + (merdataC[2] << 8) + merdataC[3]);
*/
		if (mode == PARAM_MERCTRL_MODE_AUTO) {
			/* PARAM_MERCTRL_MODE_AUTO */
			sreg = MB86A30V_REG_SUBR_MERCTRL;
			value = 0;
			mb86a30v_i2c_slave_send(rega, sreg, regd, value);
		}

		if (copy_to_user
		    ((void *)&MER_user->MER[indx], (void *)&MERDATA,
		     sizeof(struct mer_data))) {
			rtncode = -EFAULT;
			goto mer_moni_return;
		}
	}

mer_moni_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/** -- long (2011/08/17) --
	ioctl System Call.  IOCTL_CH_SEARCH_SETTING command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_CH_SEARCH_SETTING(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			    unsigned long arg)
{
	ioctl_ch_search_t *CHSRH_user = (ioctl_ch_search_t *) arg;
	ioctl_ch_search_t *CHSRH = &cmdctrl->CHSRH;
	size_t tmpsize = sizeof(ioctl_ch_search_t);
	u8 tmpdata = 0;
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (CHSRH_user == NULL) {
		rtncode = -EINVAL;
		goto ch_search_setting_return;
	}

	memset(CHSRH, 0, tmpsize);
	if (copy_from_user(CHSRH, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto ch_search_setting_return;
	}

	switch (CHSRH->MODE_DETECT) {
	case PARAM_MODE_DETECT_ON:
		mb86a30v_i2c_master_recv(0x07, &tmpdata, 1);
		tmpdata = tmpdata & 0xEF;
		mb86a30v_i2c_master_send( 0x07, tmpdata ) ;
		break;

	case PARAM_MODE_DETECT_OFF:
		mb86a30v_i2c_master_recv(0x07, &tmpdata, 1);
		tmpdata = tmpdata & 0xE0;
		tmpdata = tmpdata | 0x10 | CHSRH->SEARCH_MODE << 2 | CHSRH->SEARCH_GUARD;

		mb86a30v_i2c_master_send( 0x07, tmpdata ) ;
		break;

	default:
		rtncode = -EINVAL;
		goto ch_search_setting_return;
		break;
	}

	mb86a30v_i2c_master_send( 0xf0, CHSRH->SEARCH_TMCC ) ;
	mb86a30v_i2c_master_send( 0xf1, CHSRH->PEAK_DETECT_NUM << 4 | CHSRH->MODE_DETECT_NUM) ;
	mb86a30v_i2c_master_send( 0xf2, CHSRH->CH_DETECT_NUM << 4 |  CHSRH->CH_DETECT_OKNUM) ;
	tmpdata = CHSRH->SEARCH_THRES;
	mb86a30v_i2c_sub_send(0x40, 0x35, 0x41, &tmpdata, PARAM_I2C_MODE_SEND_8);
	tmpdata = CHSRH->SEARCH_TIME;
	mb86a30v_i2c_sub_send(0x40, 0x0d, 0x41, &tmpdata, PARAM_I2C_MODE_SEND_8);

	mb86a30v_i2c_master_send( 0xf3, CHSRH->SEARCH_START_CH ) ;
	mb86a30v_i2c_master_send( 0xf4, CHSRH->SEARCH_STOP_CH ) ;

ch_search_setting_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/** -- long (2011/08/18) --
	ioctl System Call.  IOCTL_CH_SEARCH_START command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_CH_SEARCH_START(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			  unsigned long arg)
{
	int rtncode = 0;
	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	mb86a30v_i2c_master_send( 0xe6, 0x10 ) ;
	mb86a30v_i2c_master_send( 0xe6, 0x02 ) ;

	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/** -- long (2011/08/18) --
	ioctl System Call.  IOCTL_CH_SEARCH_STOP command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_CH_SEARCH_STOP(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			  unsigned long arg)
{
	int rtncode = 0;


	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	mb86a30v_i2c_master_send( 0xee, 0x10 ) ;
	mb86a30v_i2c_master_send( 0xee, 0x00 ) ;
	mb86a30v_i2c_master_send( 0xe6, 0x00 ) ;

	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/** -- long (2011/08/18) --
	ioctl System Call.  IOCTL_CH_SEARCH_END command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_CH_SEARCH_END(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			  unsigned long arg)
{
	ioctl_ch_search_t *CHSRH_user = (ioctl_ch_search_t *) arg;
	ioctl_ch_search_t *CHSRH = &cmdctrl->CHSRH;
	size_t tmpsize = sizeof(ioctl_ch_search_t);
	u8 tmpdata = 0;
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (CHSRH_user == NULL) {
		rtncode = -EINVAL;
		goto ch_search_end_return;
	}

	memset(CHSRH, 0, tmpsize);
	if (copy_from_user(CHSRH, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto ch_search_end_return;
	}

	mb86a30v_i2c_master_recv(0xed, &tmpdata, 1);
	tmpdata = tmpdata & 0x01;
	if (copy_to_user
		((void *)&CHSRH_user->SEARCH_END, (void *)&tmpdata, 1)) {
		rtncode = -EFAULT;
		goto ch_search_end_return;
	}

ch_search_end_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;

}

/************************************************************************/
/** -- long (2011/08/18) --
	ioctl System Call.  IOCTL_CH_SEARCH_RESULT command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_CH_SEARCH_RESULT(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			    unsigned long arg)
{
	ioctl_ch_search_t *CHSRH_user = (ioctl_ch_search_t *) arg;
	ioctl_ch_search_t *CHSRH = &cmdctrl->CHSRH;
	size_t tmpsize = sizeof(ioctl_ch_search_t);
	//u8 tmpdata = 0;
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (CHSRH_user == NULL) {
		rtncode = -EINVAL;
		goto ch_search_result_return;
	}

	memset(CHSRH, 0, tmpsize);
	if (copy_from_user(CHSRH, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto ch_search_result_return;
	}

	mb86a30v_i2c_sub_recv(0xe8, 0x01, 0xe9, &CHSRH->CHANNEL[1], PARAM_I2C_MODE_SEND_8);
	mb86a30v_i2c_sub_recv(0xe8, 0x02, 0xe9, &CHSRH->CHANNEL[2], PARAM_I2C_MODE_SEND_8);
	mb86a30v_i2c_sub_recv(0xe8, 0x03, 0xe9, &CHSRH->CHANNEL[3], PARAM_I2C_MODE_SEND_8);
	mb86a30v_i2c_sub_recv(0xe8, 0x04, 0xe9, &CHSRH->CHANNEL[4], PARAM_I2C_MODE_SEND_8);
	mb86a30v_i2c_sub_recv(0xe8, 0x05, 0xe9, &CHSRH->CHANNEL[5], PARAM_I2C_MODE_SEND_8);
	mb86a30v_i2c_sub_recv(0xe8, 0x06, 0xe9, &CHSRH->CHANNEL[6], PARAM_I2C_MODE_SEND_8);
	mb86a30v_i2c_sub_recv(0xe8, 0x07, 0xe9, &CHSRH->CHANNEL[7], PARAM_I2C_MODE_SEND_8);

	if (copy_to_user((void *)CHSRH_user->CHANNEL, (void *)CHSRH->CHANNEL, 9)) {
		rtncode = -EFAULT;
		goto ch_search_result_return;
	}

ch_search_result_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/** -- long (2011/08/17) --
	ioctl System Call.  IOCTL_RF_INIT command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_RF_INIT(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			  unsigned long arg)
{
	ioctl_rf_t *RF_user = (ioctl_rf_t *) arg;
	ioctl_rf_t *RF = &cmdctrl->RF;
	size_t tmpsize = sizeof(ioctl_rf_t);
	//unsigned char reg;
	//unsigned int value = 0;
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (RF_user == NULL) {
		rtncode = -EINVAL;
		goto rf_init_return;
	}

	memset(RF, 0, tmpsize);
	if (copy_from_user(RF, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto rf_init_return;
	}

	switch (RF->RF_INIT) {
	case PARAM_RF_INITIALIZE:
		mb86a30v_i2c_master_send( 0x31, 0x00 ) ;// RF Software reset
		mb86a30v_i2c_master_send( 0x36, 0x10 ) ;
		mb86a30v_i2c_master_send( 0x32, 0x00 ) ;
		mb86a30v_i2c_master_send( 0x33, 0xff ) ;
		mb86a30v_i2c_master_send( 0x32, 0x10 ) ;
		mb86a30v_i2c_master_send( 0x33, 0x00 ) ;
		mb86a30v_i2c_master_send( 0x31, 0x80 ) ;
		break;

	case PARAM_RF_INIT_SETTING:
		mb86a30v_i2c_master_send( 0x31, 0x00 ) ;
		mb86a30v_i2c_master_send( 0x36, 0x10 ) ;
		mb86a30v_i2c_master_send( 0x32, 0x00 ) ;
		mb86a30v_i2c_master_send( 0x33, 0xf7 ) ;
		mb86a30v_i2c_master_send( 0x32, 0x10 ) ;
		mb86a30v_i2c_master_send( 0x33, 0x27 ) ;
		mb86a30v_i2c_master_send( 0x31, 0x80 ) ;
		break;

	default:
		rtncode = -EINVAL;
		goto rf_init_return;
		break;
	}

rf_init_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;

}



/************************************************************************/
/** -- long (2011/08/19) --
	ioctl System Call.  IOCTL_RF_CHANNEL( command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_RF_CHANNEL(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			     unsigned long arg)
{
	ioctl_rf_t *RF_user = (ioctl_rf_t *) arg;
	ioctl_rf_t *RF = &cmdctrl->RF;
	size_t tmpsize = sizeof(ioctl_rf_t);
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (RF_user == NULL) {
		rtncode = -EINVAL;
		goto rf_channel_return;
	}

	memset(RF, 0, tmpsize);
	if (copy_from_user(RF, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto rf_channel_return;
	}

	mb86a30v_i2c_master_send( 0xee, 0x05 ) ; //setting for channel selection
	mb86a30v_i2c_master_send( 0x08, 0x01 ) ; //OFDM state initialization setting 

	//// Channel selection -- begin
		// Upper/Lower auto setting
		mb86a30v_i2c_master_send( 0x34, 0x12 ) ;

		// Channel Select
		mb86a30v_i2c_master_send( 0x31, 0x00 ) ;
		mb86a30v_i2c_master_send( 0x31, (RF->ch_no + 0xc0)&0xff) ;
	//// Channel selection -- end

	mb86a30v_i2c_master_send( 0xee, 0x27 ) ; //Tuning end
	mb86a30v_i2c_master_send( 0xee, 0x07 ) ;

	mb86a30v_i2c_master_send( 0x08, 0x00 ) ; // OFDM state initialization clear setting
	mb86a30v_i2c_master_send( 0x31, 0x00 ) ;

	//// Low Power setting -- begin
		mb86a30v_i2c_master_send( 0x31, 0x00 ) ;
		mb86a30v_i2c_master_send( 0x36, 0x10 ) ;
		mb86a30v_i2c_master_send( 0x32, 0x00 ) ;
		mb86a30v_i2c_master_send( 0x33, 0x87 ) ;
		mb86a30v_i2c_master_send( 0x32, 0x10 ) ;
		mb86a30v_i2c_master_send( 0x33, 0x6c ) ;
		mb86a30v_i2c_master_send( 0x31, 0x80 ) ;
	//// Low Power setting -- end

	mb86a30v_i2c_master_send( 0x31, 0x00 ) ;// AGC target
	mb86a30v_i2c_master_send( 0x36, 0x10 ) ;
	mb86a30v_i2c_master_send( 0x32, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x33, 0xf9 ) ;
	mb86a30v_i2c_master_send( 0x32, 0x10 ) ;
	mb86a30v_i2c_master_send( 0x33, 0x90 ) ;
	mb86a30v_i2c_master_send( 0x31, 0x80 ) ;

	mb86a30v_i2c_master_send( 0xe0, 0x00 ) ;// LNA control
	mb86a30v_i2c_master_send( 0xe3, 0x01 ) ;
	mb86a30v_i2c_master_send( 0xe0, 0x01 ) ;
	mb86a30v_i2c_master_send( 0xe1, 0x02 ) ;
	mb86a30v_i2c_master_send( 0xe2, 0x40 ) ;
	mb86a30v_i2c_master_send( 0xe1, 0x03 ) ;
	mb86a30v_i2c_master_send( 0xe2, 0x04 ) ;
	mb86a30v_i2c_master_send( 0xe1, 0x04 ) ;
	mb86a30v_i2c_master_send( 0xe2, 0x14 ) ;

	mb86a30v_i2c_master_send( 0x70, 0xf8 ) ;// Logic reset setting
	mb86a30v_i2c_master_send( 0x70, 0xff ) ;

	mb86a30v_i2c_master_send( 0x08, 0x00 ) ; // OFDM state initialization clear setting

rf_channel_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}


/************************************************************************/
/** -- long (2011/08/12) --
	ioctl System Call.  IOCTL_STANDBY command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_STANDBY(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			 unsigned long arg)
{
	ioctl_standby_t *STANDBY_user = (ioctl_standby_t *) arg;
	ioctl_standby_t *STANDBY = &cmdctrl->STANDBY;
	size_t tmpsize = sizeof(ioctl_standby_t);
	//unsigned char reg;
	//unsigned int value = 0;
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (STANDBY_user == NULL) {
		rtncode = -EINVAL;
		goto standby_return;
	}

	memset(STANDBY, 0, tmpsize);
	if (copy_from_user(STANDBY, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto standby_return;
	}

	switch (STANDBY->mode) {
	case PARAM_STANDBY_ON:
		mb86a30v_i2c_master_send( 0x08, 0x01 ) ;
		mb86a30v_i2c_master_send( 0x73, 0x01 ) ;
		mb86a30v_i2c_master_send( 0xd0, 0x13 ) ;
		break;

	case PARAM_STANDBY_OFF:
		mb86a30v_i2c_master_send( 0xd0, 0x00 ) ;
		mdelay(2);
		mb86a30v_i2c_master_send( 0x73, 0x00 ) ;
		mb86a30v_i2c_master_send( 0x08, 0x00 ) ;
		break;

	default:
		rtncode = -EINVAL;
		goto standby_return;
		break;
	}

standby_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_I2C_MAIN command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_I2C_MAIN(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			   unsigned long arg)
{
	ioctl_i2c_t *I2C_user = (ioctl_i2c_t *) arg;
	ioctl_i2c_t *I2C = &cmdctrl->I2C;
	size_t tmpsize = sizeof(ioctl_i2c_t);
	unsigned int reg;
	unsigned int value;
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (I2C_user == NULL) {
		rtncode = -EINVAL;
		goto i2c_main_return;
	}

	memset(I2C, 0, tmpsize);
	if (copy_from_user(I2C, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto i2c_main_return;
	}

	switch (I2C->mode) {
	case PARAM_I2C_MODE_SEND:
		reg = I2C->address;
		value = I2C->data[0];
		mb86a30v_i2c_master_send(reg, value);
		break;

	case PARAM_I2C_MODE_RECV:
		reg = I2C->address;
		rtncode = mb86a30v_i2c_master_recv(reg, &I2C->data[0], 1);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto i2c_main_return;
		}

		if (copy_to_user
		    ((void *)&I2C_user->data[0], (void *)&I2C->data[0], 1)) {
			rtncode = -EFAULT;
			goto i2c_main_return;
		}
		break;

	default:
		rtncode = -EINVAL;
		goto i2c_main_return;
		break;
	}

i2c_main_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/**
	ioctl System Call.  IOCTL_I2C_SUB command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_I2C_SUB(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			  unsigned long arg)
{
	ioctl_i2c_t *I2C_user = (ioctl_i2c_t *) arg;
	ioctl_i2c_t *I2C = &cmdctrl->I2C;
	size_t tmpsize = sizeof(ioctl_i2c_t);
	u8 data[4];
	unsigned char rega;
	unsigned char regd;
	unsigned char sreg;
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (I2C_user == NULL) {
		rtncode = -EINVAL;
		goto i2c_sub_return;
	}

	memset(I2C, 0, tmpsize);
	if (copy_from_user(I2C, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto i2c_sub_return;
	}

	memset(data, 0, sizeof(data));
	switch (I2C->mode) {
	case PARAM_I2C_MODE_SEND_8:
	case PARAM_I2C_MODE_SEND_24:
		rega = I2C->address_sub;
		regd = I2C->data_sub;
		sreg = I2C->address;
		data[0] = I2C->data[0];
		data[1] = I2C->data[1];
		data[2] = I2C->data[2];
		mb86a30v_i2c_sub_send(rega, sreg, regd, &data[0], I2C->mode);
		break;

	case PARAM_I2C_MODE_RECV_8:
	case PARAM_I2C_MODE_RECV_24:
		rega = I2C->address_sub;
		regd = I2C->data_sub;
		sreg = I2C->address;
		rtncode =
		    mb86a30v_i2c_sub_recv(rega, sreg, regd, &I2C->data[0],
					 I2C->mode);
		if (rtncode != 0) {
			rtncode = -EFAULT;
			goto i2c_sub_return;
		}
		if (copy_to_user
		    ((void *)&I2C_user->data[0], (void *)&I2C->data[0], 3)) {
			rtncode = -EFAULT;
			goto i2c_sub_return;
		}
		break;

	default:
		rtncode = -EINVAL;
		goto i2c_sub_return;
		break;
	}

i2c_sub_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************/
/** -- long (2011/08/19) --
	ioctl System Call.  IOCTL_I2C_RF command control. \n
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	cmdctrl		[in,out] driver contolr area pointer to structure "mb86a30v_cmdcontrol_t".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	=0	nothing write data data.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_IOCTL_I2C_RF(mb86a30v_cmdcontrol_t * cmdctrl, unsigned int cmd,
			 unsigned long arg)
{
	ioctl_i2c_t *I2C_user = (ioctl_i2c_t *) arg;
	ioctl_i2c_t *I2C = &cmdctrl->I2C;
	size_t tmpsize = sizeof(ioctl_i2c_t);
	unsigned char reg;
	unsigned int value = 0;
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : (cmdctrl:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)cmdctrl, (int)cmd, (int)arg);

	if (I2C_user == NULL) {
		rtncode = -EINVAL;
		goto i2c_rf_return;
	}

	memset(I2C, 0, tmpsize);
	if (copy_from_user(I2C, (void *)arg, tmpsize)) {
		DBGPRINT(PRINT_LHEADERFMT "copy_from_user failed. (len:%d)\n",
			 PRINT_LHEADER, tmpsize);
		rtncode = -EFAULT;
		goto i2c_rf_return;
	}
	
	reg = I2C->address;
	value = I2C->data[0];

	mb86a30v_i2c_master_send( 0x31, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x36, 0x10 ) ;
	mb86a30v_i2c_master_send( 0x32, 0x00 ) ;
	mb86a30v_i2c_master_send( 0x33, reg ) ;
	mb86a30v_i2c_master_send( 0x32, 0x10 ) ;
	mb86a30v_i2c_master_send( 0x33, value ) ;
	mb86a30v_i2c_master_send( 0x31, 0x80 ) ;

i2c_rf_return:
	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

static struct vreg *vreg_l17_1p8, *vreg_l19_2p8,*vreg_l25_1p2;
int mb86a30v_power(unsigned int on)
{
	int rc=0;

	printk(KERN_ERR"%s:%d\n",__func__,on);
	if(on){
		vreg_l17_1p8 = vreg_get(NULL, "gp11");
		if (!vreg_l17_1p8) {
			pr_err("%s: VREG L17 get failed\n", __func__);
			return -1;
		}

		rc = vreg_set_level(vreg_l17_1p8, 1800);
		if (rc) {
			pr_err("%s: vreg_set_level gp11 failed\n", __func__);
			goto fail_gp11_level;
		}
		
		rc = vreg_enable(vreg_l17_1p8);
		if (rc) {
			pr_err("%s: vreg_enable gp11 failed\n", __func__);
			goto fail_gp11_level;
		}
		
		vreg_l25_1p2 = vreg_get(NULL, "gp17");
		if (!vreg_l25_1p2) {
			pr_err("%s: VREG L25 get failed\n", __func__);
			return -1;
		}

		rc = vreg_set_level(vreg_l25_1p2, 1200);
		if (rc) {
			pr_err("%s: vreg_set_level gp17 failed\n", __func__);
			goto fail_gp17_level;
		}
		
		rc = vreg_enable(vreg_l25_1p2);
		if (rc) {
			pr_err("%s: vreg_enable gp17 failed\n", __func__);
			goto fail_gp17_level;
		}
		mdelay(10);

		gpio_set_value(37,1);
		mdelay(10);	

		vreg_l19_2p8 = vreg_get(NULL, "wlan2");
		if (!vreg_l19_2p8) {
			pr_err("%s: VREG L19 get failed\n", __func__);
			return -1;
		}

		rc = vreg_set_level(vreg_l19_2p8, 2800);
		if (rc) {
			pr_err("%s: vreg_set_level wlan2 failed\n", __func__);
			goto fail_wlan2_level;
		}
		
		rc = vreg_enable(vreg_l19_2p8);
		if (rc) {
			pr_err("%s: vreg_enable wlan2 failed\n", __func__);
			goto fail_wlan2_level;
		}	
		mdelay(10);	
	}
	else{
		if (vreg_l19_2p8) {
			vreg_disable(vreg_l19_2p8);
			//vreg_put(vreg_l19_2p8);
			vreg_l19_2p8 = NULL;
		}
		
		mdelay(10);
		
		if (vreg_l17_1p8) {
			vreg_disable(vreg_l17_1p8);
			//vreg_put(vreg_l17_1p8);
			vreg_l17_1p8 = NULL;
		}
		
		if (vreg_l25_1p2) {
			vreg_disable(vreg_l25_1p2);
			//vreg_put(vreg_l25_1p2);
			vreg_l25_1p2 = NULL;
		}
		
		gpio_set_value(37,0);		
		
	}
	
	return 0;

fail_wlan2_level:
	//vreg_put(vreg_l19_2p8);	
	vreg_l19_2p8 = NULL;
	vreg_disable(vreg_l25_1p2);
fail_gp17_level:
	//vreg_put(vreg_l25_1p2);	
	vreg_l25_1p2 = NULL;
	vreg_disable(vreg_l17_1p8);
fail_gp11_level:
	//vreg_put(vreg_l17_1p8);	
	vreg_l17_1p8 = NULL;
	return rc;	
}

/************************************************************************
 * open() system call.
 * [User Interface]
 *     int    open( const char "/dev/radio0", O_RDWR );
 */
/**
	open System Call.
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	inode		[in] pointer to structure "inode".
	@param	filp		[in] pointer to structure "filp".
	@retval	>0	file descriptor. (normal return)
	@retval	0	The error occurred. The detailed information is set to an errno.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static int mb86a30v_open(struct inode *inode, struct file *filp)
{
	unsigned int majorno, minorno;
	unsigned char *devarea;
	mb86a30v_cmdcontrol_t *cmdctrl;
	size_t getmemsize = sizeof(mb86a30v_cmdcontrol_t);
	int rc;
	
	DBGPRINT(PRINT_LHEADERFMT " : open()  called.\n", PRINT_LHEADER);
	DBGPRINT(PRINT_LHEADERFMT
		 " : filp[0x%08x]->f_dentry[0x%08x]->d_inode[0x%08x].\n",
		 PRINT_LHEADER, (int)filp, (int)(filp->f_dentry),
		 (int)(filp->f_dentry->d_inode));

//	gpio_set_value(27, 1);
//	msleep(1);	
//	gpio_set_value(26, 1);
//	msleep(1);		
//	gpio_set_value(34, 1);
//	msleep(100);
	rc = mb86a30v_power(1);
	if(rc < 0)
		printk(KERN_ERR"%s:MB96A30V power on fail !",__func__);
	majorno = imajor(filp->f_dentry->d_inode);
	minorno = iminor(filp->f_dentry->d_inode);
	//if ((majorno != NODE_MAJOR) || (minorno != NODE_MINOR)) {
//	if ((majorno != 10) || (minorno != 21)) {
//		DBGPRINT(PRINT_LHEADERFMT
//			 " : Illegal Operation. major[ %d ], minor[ %d ].\n",
//			 PRINT_LHEADER, majorno, minorno);
//		return -ENODEV;
//	}

	if ((filp->f_flags & O_ACCMODE) != O_RDWR) {
		DBGPRINT(PRINT_LHEADERFMT
			 " : Illegal Operation mode(flags). flags[ %d ].\n",
			 PRINT_LHEADER, (filp->f_flags & O_ACCMODE));
		return -EINVAL;
	}

	devarea = (unsigned char *)filp->private_data;
//	if (devarea != NULL) {
//		DBGPRINT(PRINT_LHEADERFMT
//			 " : Used private_data area. private_data[ 0x%08x ].\n",
//			 PRINT_LHEADER, (int)devarea);
//		return -EBUSY;
//	}

	devarea = (unsigned char *)kmalloc(getmemsize, GFP_KERNEL);
	if (devarea == NULL) {
		DBGPRINT(PRINT_LHEADERFMT
			 " : kernel memory is short[ 0x%08x ].\n",
			 PRINT_LHEADER, (int)devarea);
		return -ENOMEM;
	}
	memset(devarea, 0, getmemsize);
	cmdctrl = (mb86a30v_cmdcontrol_t *) devarea;
	filp->private_data = (void *)devarea;

	DBGPRINT(PRINT_LHEADERFMT
		 " : Normal return. file->private_data[ 0x%08x ]\n",
		 PRINT_LHEADER, (int)filp->private_data);

	return 0;
}

/************************************************************************
 * release() .... close() system call.
 * [User Interface]
 *     int    close( int fd );
 */
/**
	close System Call.
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	inode		[in] pointer to structure "inode".
	@param	filp		[in] pointer to structure "filp".
	@retval	0	normal return.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_close(struct inode *inode, struct file *filp)
{
	unsigned char *devarea;
	mb86a30v_cmdcontrol_t *cmdctrl;
	int rc;
	
	DBGPRINT(PRINT_LHEADERFMT " : close()  called.\n", PRINT_LHEADER);
	DBGPRINT(PRINT_LHEADERFMT
		 " : filp[0x%08x]->f_dentry[0x%08x]->d_inode[0x%08x].\n",
		 PRINT_LHEADER, (int)filp, (int)(filp->f_dentry),
		 (int)(filp->f_dentry->d_inode));

//	gpio_set_value(34, 0);	
//	gpio_set_value(26, 0);
//	msleep(10);	
//	gpio_set_value(27, 0);
//	msleep(50);
	rc = mb86a30v_power(0);
	if(rc < 0)
		printk(KERN_ERR"%s:MB96A30V power off fail !",__func__);
	devarea = (unsigned char *)filp->private_data;
	if (devarea == NULL) {
		DBGPRINT(PRINT_LHEADERFMT
			 " : Not found private_data area. private_data[ 0x%08x ].\n",
			 PRINT_LHEADER, (int)devarea);
		return -EFAULT;
	}
	cmdctrl = (mb86a30v_cmdcontrol_t *) devarea;

	memset(devarea, 0, sizeof(mb86a30v_cmdcontrol_t));

	kfree(devarea);

	filp->private_data = NULL;
	return 0;
}

/************************************************************************
 * read() system call.
 * [User Interface]
 *     ssize_t    read( struct file *filp, char __user *buff, size_t count, loff_t *offp );
 */
/**
	read System Call.
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	filp		[in] pointer to structure "filp".
	@param	buf		[out] pointer to read data area.
	@param	count		[in] size of read data area.
	@param	f_pos		[in,out] offset of read data area.
	@retval	0	nothing read data.
	@retval	>0	number of bytes read is returned.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
ssize_t mb86a30v_read(struct file *filp, char *buf, size_t count, loff_t * f_pos)
{
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : read(filp:0x%08x,buf:0x%08x,count:%d,f_pos:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)filp, (int)buf, count, (int)f_pos);

#ifdef	DEBUG_PRINT_MEM
	if (LOGMEM != NULL) {
		int offset = 0;
		LOGCOPYSIZE = strlen(LOGMEM);
		if (copy_to_user(buf, &LOGMEM[offset], LOGCOPYSIZE)) {
			DBGPRINT(PRINT_LHEADERFMT " : copy_to_user error.\n",
				 PRINT_LHEADER);
			rtncode = -EFAULT;
			goto read_return;
		}
		memset(LOGMEM, 0, DEBUG_PRINT_MEM_SIZE);
		rtncode = LOGCOPYSIZE;
		goto read_return;
	}

read_return:
#endif

	DBGPRINT(PRINT_LHEADERFMT " : Read Length[ %d ], Return Code[ %d ]\n",
		 PRINT_LHEADER, count, rtncode);
	return rtncode;
}

/************************************************************************
 * write() system call.
 * [User Interface]
 *     ssize_t    write( struct file *filp, char __user *buff, size_t count, loff_t *offp );
 */
/**
	write System Call.
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	filp		[in] pointer to structure "filp".
	@param	buf		[in] pointer to write data area.
	@param	count		[in] size of write data area.
	@param	f_pos		[in,out] offset of write data area.
	@retval	0	nothing write data data.
	@retval	>0	number of bytes write is returned.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int mb86a30v_write(struct file *filp, const char __user * buf, size_t count,
		  loff_t * f_pos)
{
	int rtncode = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : write(filp:0x%08x,buf:0x%08x,count:%d,f_pos:0x%08x)  called.\n",
		 PRINT_LHEADER, (int)filp, (int)buf, count, (int)f_pos);

	DBGPRINT(PRINT_LHEADERFMT "**** return[ %d ].\n", PRINT_LHEADER,
		 rtncode);
	return rtncode;
}

/************************************************************************
 * ioctl() system call.
 * [User Interface]
 *     int    ioctl( int fd, unsigned long cmd, ... );
 *     int    ioctl( struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg );
 */
/**
	ioctl System Call.
	Device Driver for Multi mode tuner module. (MB86A30V)

	@param	inode		[in] pointer to structure "inode".
	@param	filp		[in] pointer to structure "filp".
	@param	cmd		[in] command code.
	@param	arg		[in,out] command argument/context.
	@retval	0	normal return.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/
static long mb86a30v_ioctl(struct file *filp, unsigned int cmd,
		  unsigned long arg)
{
	mb86a30v_cmdcontrol_t *cmdctrl;
	int rtn = 0;

	DBGPRINT(PRINT_LHEADERFMT
		 " : ioctl(filp:0x%08x,cmd:0x%08x,arg:0x%08x)  called.\n",
		 PRINT_LHEADER,(int)filp, cmd, (int)arg);

	cmdctrl = (mb86a30v_cmdcontrol_t *) filp->private_data;
	if (cmdctrl == NULL) {
		DBGPRINT(PRINT_LHEADERFMT
			 " : nothing driver working space. cmdctrl[ 0x%08x ].\n",
			 PRINT_LHEADER, (int)cmdctrl);
		return -EBADF;
	}

	switch (cmd) {
		/* Reset */
	case IOCTL_RST_SOFT:	/* OFDM Soft-reset ON / OFF */
		rtn = mb86a30v_IOCTL_RST_SOFT(cmdctrl, cmd, arg);
		break;

	case IOCTL_RST_SYNC:	/* OFDM Sequence reset ON / OFF */
		rtn = mb86a30v_IOCTL_RST_SYNC(cmdctrl, cmd, arg);
		break;

	case IOCTL_SET_SPECT:	/* set Receive spectrum Invert */
		rtn = mb86a30v_IOCTL_SET_SPECT(cmdctrl, cmd, arg);
		break;

	case IOCTL_SET_ALOG_PDOWN:	/* set Analog Power-down (DAC Power-down) */
		rtn = mb86a30v_IOCTL_SET_ALOG_PDOWN(cmdctrl, cmd, arg);
		break;

		/* AGC */
	case IOCTL_AGC:	/* set AGC(Auto Gain Control) register */
		rtn = mb86a30v_IOCTL_AGC(cmdctrl, cmd, arg);
		break;

	// -- long (2011/08/18) --
	case IOCTL_AGC_OPERATING:	/* set AGC(Auto Gain Control) register */
		rtn = mb86a30v_IOCTL_AGC_OPERATING(cmdctrl, cmd, arg);
		break;

	case IOCTL_LNA_SETTING:	/* LNA : LNA_SETTING -- long (2011/08/12) -- */
		rtn = mb86a30v_IOCTL_LNA_SETTING(cmdctrl, cmd, arg);
		break;	

	case IOCTL_CMOS_SETTING:	/* CMOS : CMOS SETTING -- long (2011/08/12) -- */
		rtn = mb86a30v_IOCTL_CMOS_SETTING(cmdctrl, cmd, arg);
		break;	

		/* AGC */
	case IOCTL_OFDM_INIT:	/* set SYNC, set FEC */
		rtn = mb86a30v_IOCTL_SYNC(cmdctrl, cmd, arg);
		break;

	case IOCTL_AVDE_INIT:	/* AVDE INIT -- long (2011/08/12) -- */
		rtn = mb86a30v_IOCTL_AVDE_INIT(cmdctrl, cmd, arg);
		break;	

		/* Sequence control */
	case IOCTL_SEQ_GETSTAT:	/* get Status */
		rtn = mb86a30v_IOCTL_SEQ_GETSTAT(cmdctrl, cmd, arg);
		break;

	// -- long (2011/08/26) --
	case IOCTL_SEQ_SETMODE:	/* set Mode */
		rtn = mb86a30v_IOCTL_SEQ_SETMODE(cmdctrl, cmd, arg);
		break;

	case IOCTL_SEQ_GETMODE:	/* get Mode */
		rtn = mb86a30v_IOCTL_SEQ_GETMODE(cmdctrl, cmd, arg);
		break;

	case IOCTL_SEQ_GETTMCC:	/* get TMCC Information */
		rtn = mb86a30v_IOCTL_SEQ_GETTMCC(cmdctrl, cmd, arg);
		break;

	case IOCTL_BER_MONICONFIG:	/* BER Monitor configuration */
		rtn = mb86a30v_IOCTL_BER_MONICONFIG(cmdctrl, cmd, arg);
		break;

	case IOCTL_BER_MONISTART:	/* BER Monitor start */
		rtn = mb86a30v_IOCTL_BER_MONISTART(cmdctrl, cmd, arg);
		break;

	case IOCTL_BER_MONIGET:	/* get BER Monitor information */
		rtn = mb86a30v_IOCTL_BER_MONIGET(cmdctrl, cmd, arg);
		break;

	case IOCTL_BER_MONISTOP:	/* BER Monitor STOP */
		rtn = mb86a30v_IOCTL_BER_MONISTOP(cmdctrl, cmd, arg);
		break;

		/* TS Output */
	case IOCTL_TS_START:	/* Serial TS Output Start */
		rtn = mb86a30v_IOCTL_TS_START(cmdctrl, cmd, arg);
		break;

	case IOCTL_TS_STOP:	/* Serial TS Output OFF */
		rtn = mb86a30v_IOCTL_TS_STOP(cmdctrl, cmd, arg);
		break;

	case IOCTL_TS_CONFIG:	/* Serial TS Output Configuration */
		rtn = mb86a30v_IOCTL_TS_CONFIG(cmdctrl, cmd, arg);
		break;

	case IOCTL_TS_PCLOCK:	/* set Serial TS Output Clock / Parallel TS Clock */
		rtn = mb86a30v_IOCTL_TS_PCLOCK(cmdctrl, cmd, arg);
		break;

	case IOCTL_TS_OUTMASK:	/* set TS Output Mask OFF */
		rtn = mb86a30v_IOCTL_TS_OUTMASK(cmdctrl, cmd, arg);
		break;

		/* IRQ reason */
	case IOCTL_IRQ_GETREASON:	/* Get IRQ Reason */
		rtn = mb86a30v_IOCTL_IRQ_GETREASON(cmdctrl, cmd, arg);
		break;

	case IOCTL_IRQ_SETMASK:	/* IRQ : Interrupt mask control */
		rtn = mb86a30v_IOCTL_IRQ_SETMASK(cmdctrl, cmd, arg);
		break;

	case IOCTL_IRQ_TMCCPARAM_SET:	/* IRQ : TMCC parameter Interrupt setting */
		rtn = mb86a30v_IOCTL_IRQ_TMCCPARAM_SET(cmdctrl, cmd, arg);
		break;

		/* C/N Monitor */
	case IOCTL_CN_MONI:	/* C/N Monitoring (Auto / Manual) */
		rtn = mb86a30v_IOCTL_CN_MONI(cmdctrl, cmd, arg);
		break;

		/* MER Monitor */
	case IOCTL_MER_MONI:	/* MER Monitoring (Auto / Manual) */
		rtn = mb86a30v_IOCTL_MER_MONI(cmdctrl, cmd, arg);
		break;

		/* High speed Channel Search */
	case IOCTL_CH_SEARCH_SETTING:	/* Hight speed Channel Search */
		rtn = mb86a30v_IOCTL_CH_SEARCH_SETTING(cmdctrl, cmd, arg);
		break;

		// -- long (2011/08/18) --
	case IOCTL_CH_SEARCH_START:	/* Hight speed Channel Search */
		rtn = mb86a30v_IOCTL_CH_SEARCH_START(cmdctrl, cmd, arg);
		break;

		// -- long (2011/08/18) --
	case IOCTL_CH_SEARCH_STOP:	/* Hight speed Channel Search */
		rtn = mb86a30v_IOCTL_CH_SEARCH_STOP(cmdctrl, cmd, arg);
		break;

		// -- long (2011/08/18) --
	case IOCTL_CH_SEARCH_END:	/* Hight speed Channel Search */
		rtn = mb86a30v_IOCTL_CH_SEARCH_END(cmdctrl, cmd, arg);
		break;

		// -- long (2011/08/18) --
	case IOCTL_CH_SEARCH_RESULT:	/* Hight speed Channel Search */
		rtn = mb86a30v_IOCTL_CH_SEARCH_RESULT(cmdctrl, cmd, arg);
		break;

		/* MB86A30V(RF) */
		/* RF Controled */
	case IOCTL_RF_INIT:	/* RF Initialize  UHF/VHF RF-IC Register Initialize */
		rtn = mb86a30v_IOCTL_RF_INIT(cmdctrl, cmd, arg);
		break;

	case IOCTL_RF_CHANNEL:	/* set UHF/VHF Channel (p15 5.5) */
		rtn = mb86a30v_IOCTL_RF_CHANNEL(cmdctrl, cmd, arg);
		break;

	case IOCTL_STANDBY:	/* STANDBY -- long (2011/08/12) -- */
		rtn = mb86a30v_IOCTL_STANDBY(cmdctrl, cmd, arg);
		break;	

		/* I2C Control */
	case IOCTL_I2C_MAIN:	/* I2C : I2C main access */
		rtn = mb86a30v_IOCTL_I2C_MAIN(cmdctrl, cmd, arg);
		break;

	case IOCTL_I2C_SUB:	/* I2C : I2C sub access */
		rtn = mb86a30v_IOCTL_I2C_SUB(cmdctrl, cmd, arg);
		break;

	case IOCTL_I2C_RF:	/* I2C : I2C RF-IC access */
		rtn = mb86a30v_IOCTL_I2C_RF(cmdctrl, cmd, arg);
		break;	
		
	default:
		DBGPRINT(PRINT_LHEADERFMT
			 " : Illegal command operation. cmd[ 0x%04x ].\n",
			 PRINT_LHEADER, cmd);
		return -EBADRQC;
		break;
	}
	DBGPRINT(PRINT_LHEADERFMT
		 " : ioctl(filp:0x%08x,cmd:0x%08x,arg:0x%08x)  return( %d ).\n",
		 PRINT_LHEADER, (int)filp, cmd, (int)arg, rtn);

	return (rtn);
}

static struct file_operations mb86a30v_fops = {
.owner =	THIS_MODULE,		/* owner */
.read=	mb86a30v_read,		/* read() system call entry */
.write=	mb86a30v_write,		/* write() system call entry */
.unlocked_ioctl=	mb86a30v_ioctl,		/* ioctl() system call entry */
.open=	mb86a30v_open,		/* open() system call entry */
.release=mb86a30v_close,		/* close() system call entry */
};
struct miscdevice mb86a30v_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "radio1",
	.fops	= &mb86a30v_fops,
};
/************************************************************************
 */

int read_mb86a30v_id(struct mb86a30v_data *data)
{
	//char tab_i2c_data[8];
	char tab_i2c_tmp[6];
	int tmp16;
	//int ret;

	DBGPRINT(PRINT_LHEADERFMT"Read MB86A30V Chip ID\n",PRINT_LHEADER);
	mutex_lock(&data->lock);
	mb86a30v_i2c_master_recv(0x00, tab_i2c_tmp, 1);
	mutex_unlock(&data->lock);
	tmp16 = tab_i2c_tmp[0];
	DBGPRINT(PRINT_LHEADERFMT"MB86A30V Chip ID:0x%x\n",PRINT_LHEADER,tmp16);
	return 0;
	
}

static int mb86a30v_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct mb86a30v_data *data;


	DBGPRINT(PRINT_LHEADERFMT "Called. \n", PRINT_LHEADER);
#if 0
	// power up
	gpio_set_value(26, 0);	
	gpio_set_value(27, 0);
	gpio_set_value(34, 0);
	msleep(50);	
	gpio_set_value(27, 1);
	msleep(1);	
	gpio_set_value(26, 1);
	msleep(1);		
	gpio_set_value(34, 1);
	msleep(100);
#endif	
	gpio_tlmm_config( GPIO_CFG(37, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_set_value(37,0);

	
	MB86A30V_DEBUG = 1;
	
	data = kzalloc(sizeof(struct  mb86a30v_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "kzalloc failed!\n");
		return -ENOMEM;
	}

	data->client= client;
	i2c_set_clientdata(client, data);
	mb86a30v_i2c_client = data->client;

	//data->i2c2_adapter = i2c_get_adapter(MB86A30V_I2C_ADAPTER_ID);	
	mutex_init(&data->lock);


	if (misc_register(&mb86a30v_misc))
		pr_warn("Couldn't initialize miscdevice /dev/radio1.\n");
	else {
		pr_info("initialized device: /dev/radio1, node (MAJOR %d, MINOR %d)\n",
			MISC_MAJOR, MISC_DYNAMIC_MINOR);
	}	
	mdelay(100);
	//read_mb86a30v_id(data);
	return 0;
}

static int mb86a30v_i2c_remove(struct i2c_client *client)
{
	struct mb86a30v_data *data = i2c_get_clientdata(client);

	misc_deregister( &mb86a30v_misc );

	kfree(data);

	return 0;
}

static const struct i2c_device_id mb86a30v_i2c_id[] = {
	{"mb86a30v", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mb86a30v_i2c_id);

/************************************************************************/
static struct i2c_driver mb86a30v_i2c_driver = {
	.driver = {
		   .name = "mb86a30v",
		   .owner = THIS_MODULE,
		   },
	.probe = mb86a30v_i2c_probe,
	.remove = mb86a30v_i2c_remove,
	.id_table = mb86a30v_i2c_id,
};

/************************************************************************
 * initialization module
 *  called by insmod function.
 */
/**
	__init System Call. (called by Kernel.)
	Device Driver for Multi mode tuner module. (MB86A30V)
	It is called as an initialization processing module.

	@retval	0	normal return.
	@retval	<0	The error occurred. The detailed information is set to an errno.
*/

static
int __init proc_init_module(void)
{
	return i2c_add_driver(&mb86a30v_i2c_driver);
}

/************************************************************************
 * cleanup module.
 *  called  by rmmod function.
 */
/**
	__exit System Call. (called by Kernel.)
	Device Driver for Multi mode tuner module. (MB86A30V)
	It is called as an end processing module.
*/

static
void __exit proc_cleanup_module(void)
{
	i2c_del_driver(&mb86a30v_i2c_driver);
}

/* Declare entry and exit functions */
module_init(proc_init_module);
module_exit(proc_cleanup_module);
/* End of Program */
