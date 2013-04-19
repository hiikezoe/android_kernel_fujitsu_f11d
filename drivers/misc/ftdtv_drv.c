/*
 * Copyright(C) 2010 FUJITSU LIMITED
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/current.h>
#include <mach/vreg.h>

/* IOCTL CMD */
#define FTDTV_IOCTL_MAGIC		'f'

#define FTDTV_IOCTL_GET_SIGNAL	_IOC(_IOC_NONE, FTDTV_IOCTL_MAGIC, 0, 0)
#define FTDTV_IOCTL_POWER_ON	_IOC(_IOC_NONE, FTDTV_IOCTL_MAGIC, 1, 0)
#define FTDTV_IOCTL_POWER_OFF	_IOC(_IOC_NONE, FTDTV_IOCTL_MAGIC, 2, 0)

static struct cdev ftdtvDevs;
static int ftdtv_major = 0;
static struct class* ftdtv_class;


static struct vreg *vreg_l17_1p8, *vreg_l19_2p8,*vreg_l25_1p2;
int ftdtv_power(unsigned int on)
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
		mdelay(10);
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

/*---------------------------------------------------------------------------
    open
---------------------------------------------------------------------------*/
int ftdtv_open(struct inode* inode, struct file* file)
{
	printk(KERN_ERR"%s(%d)\n",__func__,__LINE__);
	return 0;
}

/*---------------------------------------------------------------------------
    release
---------------------------------------------------------------------------*/
int ftdtv_release(struct inode *inode, struct file *file)
{
	printk(KERN_ERR"%s(%d)\n",__func__,__LINE__);
	return 0;
}

/*---------------------------------------------------------------------------
    ftdtv_ioctl
---------------------------------------------------------------------------*/
static long ftdtv_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret;
	int anser;
	int rc;
		
	switch (cmd) {
	printk(KERN_ERR"%s(%d)cmd:%d\n",__func__,__LINE__,cmd);
	case FTDTV_IOCTL_GET_SIGNAL:

		anser = 0;
		
		printk(KERN_ERR"%s - FTDTV_IOCTL_GET_SIGNAL\n",__func__);

		if (copy_to_user((int __user *)arg, &anser, sizeof(int))) {
			printk(KERN_DEBUG "ftdtv_drv:%s: copy_from_user failed\n", __func__);
			ret = (-EFAULT);
		} else {
			ret = 0;
		}
		break;

			
	case FTDTV_IOCTL_POWER_ON:

		ret = 0;
printk(KERN_ERR"%s - FTDTV_IOCTL_POWER_ON\n",__func__);
		// power up
rc =ftdtv_power(1);
			if(rc < 0)
		printk(KERN_ERR"%s:ftdtv power on fail !",__func__);
//	gpio_set_value(27, 1);
//	msleep(1);	
//	gpio_set_value(26, 1);
//	msleep(1);		
//	gpio_set_value(34, 1);
	msleep(50);

		break;
		

	case FTDTV_IOCTL_POWER_OFF:

		ret = 0;
printk(KERN_ERR"%s - FTDTV_IOCTL_POWER_OFF\n",__func__);
	// power up
	rc =ftdtv_power(0);
			if(rc < 0)
		printk(KERN_ERR"%s:ftdtv power on fail !",__func__);
//	gpio_set_value(34, 0);	
//gpio_set_value(26, 0);	
//	msleep(10);	
//	gpio_set_value(27, 0);
	msleep(50);
		break;
		

	default:

		printk(KERN_DEBUG "ftdtv_drv:%s: Command Failed cmd=0x%04x", __func__, cmd);

		ret = (-EINVAL);

		break;

	}

	return ret;

}


static struct file_operations ftdtv_fops = {

	.owner = THIS_MODULE,

	.llseek = no_llseek,

	.open = ftdtv_open,

	.unlocked_ioctl = ftdtv_ioctl,

	.release = ftdtv_release,

};

/*---------------------------------------------------------------------------

    ftdtv_setup_cdev

---------------------------------------------------------------------------*/

static void ftdtv_setup_cdev(struct cdev *dev, int minor, struct file_operations *fops)

{
	int err, devno;


	devno = MKDEV(ftdtv_major, minor);
	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	dev->ops = fops;
	err = cdev_add(dev, devno, 1);

	if (err) {

		printk(KERN_DEBUG "ftdtv_setup_cdev Error %d adding rfs%d\n", err, minor);
	}
	if (IS_ERR(device_create(ftdtv_class, NULL, devno, NULL, "ftdtv"))) {

		printk(KERN_DEBUG "ftdtv_setup_cdev can't create device\n");

	}

}


/*---------------------------------------------------------------------------

    init

---------------------------------------------------------------------------*/

static int __init ftdtv_init(void)

{

	int result = 0;

	dev_t dev;

	//int i;

	int rc;



	dev = MKDEV(ftdtv_major, 0);



	if (ftdtv_major) {
printk(KERN_ERR"%s - register_chrdev_region(dev, 2, ftdtv)\n",__func__);
		result = register_chrdev_region(dev, 2, "ftdtv");

	} else {
printk(KERN_ERR"%s - alloc_chrdev_region(&dev, 0, 2, ftdtv)\n",__func__);
		result = alloc_chrdev_region(&dev, 0, 2, "ftdtv");

		ftdtv_major = MAJOR(dev);

	}

	if (result < 0) {

		printk(KERN_DEBUG "ftdtv error fail to get major %d\n", ftdtv_major);

		return result;

	}

	ftdtv_class = class_create(THIS_MODULE, "ftdtv");
printk(KERN_ERR"%s - ftdtv_class = class_create\n",__func__);
	if (IS_ERR(ftdtv_class)) {

	    printk(KERN_DEBUG "ftdtv IS_ERR(ftdtv_class)\n");

		return PTR_ERR(ftdtv_class);

	}
printk(KERN_ERR"%s - tdtv_setup_cdev\n",__func__);
	ftdtv_setup_cdev(&ftdtvDevs, 0, &ftdtv_fops);
		// power up
	//gpio_set_value(26, 0);	
	//gpio_set_value(27, 0);
	//gpio_set_value(34, 0);
rc =ftdtv_power(0);
			if(rc < 0)
		printk(KERN_ERR"%s:ftdtv power on fail !",__func__);	
	#if 0
	msleep(50);	
	gpio_set_value(27, 1);
	msleep(1);	
	gpio_set_value(26, 1);
	msleep(1);		
	gpio_set_value(34, 1);
	#endif
	msleep(100);
	return 0;

}



/*---------------------------------------------------------------------------

    exit

---------------------------------------------------------------------------*/

static void __exit ftdtv_exit(void)

{

	cdev_del(&ftdtvDevs);

	unregister_chrdev_region(MKDEV(ftdtv_major, 0), 2);

	device_destroy(ftdtv_class, MKDEV(ftdtv_major, 0));

	class_destroy(ftdtv_class);

}



module_init(ftdtv_init);

module_exit(ftdtv_exit);

MODULE_LICENSE("GPL");

