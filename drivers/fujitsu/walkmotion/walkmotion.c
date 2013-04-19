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

#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/walkmotion.h>
#include <linux/ioctl.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/delay.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>
/* FUJITSU:2011-08-04 Implement interrupt handling start */
#include <linux/cdev.h>
#include <linux/input.h>
#include <linux/kdev_t.h>
#include <linux/sched.h>
/* FUJITSU:2011-08-04 Implement interrupt handling end */

/* Driver name */
#define DRIVER_NAME "fj-walkmotion"
/* FUJITSU:2011-08-04 Implement interrupt handling start */
#define DEV_NAME "fj_wm_ctl"
/* FUJITSU:2011-08-04 Implement interrupt handling end */
/* Delay */
#define FJ_WM_DEF_MC_INIT_DELAY 20

/** State bit */
/** Not initialize */
#define STS_BIT_MC_UNINIT           0x01
/** Initializing */
#define STS_BIT_MC_INIT_ONGOING     0x02
/** Initialized */
#define STS_BIT_MC_INIT             0x04
/** Polling IRQ */
#define STS_BIT_MOTION_POLLING      0x10
/** Initializer */
#define FJ_WM_STS_INITIALIZER       0x00

/** Not initialize */
#define FJ_WM_STS_MC_UNINITIALIZED   (STS_BIT_MC_UNINIT)
/** Initializing */
#define FJ_WM_STS_MC_INIT_ONGOING    (STS_BIT_MC_INIT_ONGOING)
/** Wait */
#define FJ_WM_STS_MC_INIT_WAIT \
                           (STS_BIT_MC_INIT_ONGOING|STS_BIT_MOTION_POLLING)
/** Initialized */
#define FJ_WM_STS_MC_INITIALIZED     (STS_BIT_MC_INIT)
/** Polling IRQ */
#define FJ_WM_STS_MOTION_IRQ_POLLING (STS_BIT_MC_INIT|STS_BIT_MOTION_POLLING)

#ifdef CONFIG_MACH_FJI12
#define FJ_WM_GPIO_RESET          120   /** Reset */
#endif
#ifdef CONFIG_MACH_F11APO
#define FJ_WM_GPIO_RESET          120   /** Reset */
#endif
#ifdef CONFIG_MACH_F11SKY
#define FJ_WM_GPIO_RESET           93   /** Reset */
#endif
#ifdef T_QCI_IS3
#define FJ_WM_GPIO_RESET           120   /** Reset */
#endif
/* FUJITSU:2011-07-26 Add new ioctl command start */
#if defined(T_QCI_IS3)
#define FJ_WM_GPIO_HOSU_WUP	      127   /** Wakeup */
#else
#define FJ_WM_GPIO_HOSU_WUP	      64   /** Wakeup */
#endif
/* FUJITSU:2011-07-26 Add new ioctl command end */
#define GPIO_HIGH                  1   /** High */
#define GPIO_LOW                   0   /** Low */

#define RC5T712_IOSEL2           0x21
#define RC5T712_IOOUT1           0x23
#define RC5T712_IOOUT2           0x24
#define RC5T712_GPPUPD2H         0x2C
#define RC5T712_UART1SEL         0x60
#define RC5T712_UART1PUPD        0x61
#define RC5T712_UART1TXMODE      0x62

#define debug_printk(format, arg...) \
    if (0) printk("%s" format, KERN_DEBUG, ##arg)

/* Walk motion data */
struct fj_wm_data {
	/** Driver state */
	int                state;
	/** State lock */
	struct mutex       state_lock;
	/** Motion IRQ wait queue */
	wait_queue_head_t  wait_queue_motion_irq;
	/** Motion IRQ */
	int                motion_irq;
	/** Delay */
	int                mc_init_delay;
	/** Device for debug */
	struct device      *dbg_dev;
	/** enable/disable irq flag */
	int                irq_flag;
#if 1   /* FUJITSU:2011-12-01 ICS add start */
	struct mutex       io_lock;
#endif  /* FUJITSU:2011-12-01 ICS add end */
};
static struct fj_wm_data *wm_data;
/* FUJITSU:2011-08-04 Implement interrupt handling start */
static int fj_wm_major;
static struct input_dev *this_data;
static struct cdev fj_wm_cdev;
static struct class *fj_wm_class;
/* FUJITSU:2011-08-04 Implement interrupt handling end */

static void fj_wm_mc_init(void);
static void fj_wm_motion_irq(struct work_struct *);
static DECLARE_WORK(work_queue_motion_irq, fj_wm_motion_irq);

/** Interrupt handler
 *
 * @param irq  : Not use
 * @param data : Not use
 * @return IRQ_HANDLED
 */
static irqreturn_t fj_wm_irq_handler(int irq, void *data)
{
	debug_printk("%s : start\n", __func__);
	schedule_work(&work_queue_motion_irq);

	return IRQ_HANDLED;
}

/** Observe motion irq
 *
 * @param ws : Not use
 * @return void
 */
static void fj_wm_motion_irq(struct work_struct *ws)
{
	static int ev_value = 1;

	debug_printk("%s : start\n", __func__);
	mutex_lock(&wm_data->state_lock);
	/* State check */
	if (!(wm_data->state & STS_BIT_MOTION_POLLING)) {
		/* Finish observe */
		mutex_unlock(&wm_data->state_lock);
		debug_printk("%s : ign irq (state : %#04x)\n", __func__, wm_data->state);
		return;
	} else if (wm_data->state & STS_BIT_MC_INIT_ONGOING) {
		debug_printk("%s : first motion_irq\n", __func__);
		/* Change state to FJ_WM_STS_MC_INITIALIZED */
		wm_data->state = FJ_WM_STS_MC_INITIALIZED;
	} else {
		/* FUJITSU:2011-08-04 Implement interrupt handling start */
		debug_printk("%s : normal motion_irq(ev_value : %d)\n", __func__, ev_value);
		input_report_abs(this_data, ABS_X, ev_value);
		input_sync(this_data);
		ev_value =  (ev_value == 1) ? 2 : 1;
		mutex_unlock(&wm_data->state_lock);
		return;
		/* FUJITSU:2011-08-04 Implement interrupt handling end */
	}

	wake_up_interruptible(&wm_data->wait_queue_motion_irq);

	mutex_unlock(&wm_data->state_lock);
}

/** Walk motion MC initialization
 *
 * @param ws : Work struct
 * @return void
 */
static void fj_wm_mc_init(void)
{
	
	debug_printk("%s : start\n", __func__);
	mutex_lock(&wm_data->state_lock);

	gpio_set_value(FJ_WM_GPIO_RESET, GPIO_LOW);

	mutex_unlock(&wm_data->state_lock);

	/* Wait for 20ms */
	msleep(wm_data->mc_init_delay);

	mutex_lock(&wm_data->state_lock);

	gpio_set_value(FJ_WM_GPIO_RESET, GPIO_HIGH);

	/* Change state to FJ_WM_STS_MC_INIT_WAIT */
	wm_data->state = FJ_WM_STS_MC_INIT_WAIT;

	mutex_unlock(&wm_data->state_lock);

	return;
}


/** Get driver resource
 *
 * @param inode : Not use
 * @param file  : Not use
 * @return 0 Success
 */
static int fj_wm_open(struct inode *inode, struct file *file)
{
	debug_printk("%s : start\n", __func__);
	/* Have nothing to do */
	return 0;
}

/** Release driver resource
 * @param inode : Not use
 * @param file  : Not use
 * @return 0 Success
 */
static int fj_wm_release(struct inode *inode, struct file *file)
{
	/* Release IRQ resource */
	debug_printk("%s : start\n", __func__);
	if(wm_data->irq_flag == 1) {
		free_irq(wm_data->motion_irq, wm_data);
		wm_data->irq_flag = 0;
		debug_printk("%s : irq_flag = %d\n", __func__, wm_data->irq_flag);
	}
	
	mutex_lock(&wm_data->state_lock);
	wm_data->state = FJ_WM_STS_INITIALIZER;
	mutex_unlock(&wm_data->state_lock);
	
	return 0;
}

/** Control device
 *
 * @param inode : Not use
 * @param file  : Not use
 * @param cmd   : Control command
 * @param arg   : Argument
 * @return 0 Success
 */
static int
fj_wm_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	                                                      unsigned long arg)
{
	int ret;

	switch (cmd) {
	/** Initialize MC */
	case FJ_WM_IOCT_INITIALIZE:
		debug_printk("FJ_WM_IOCT_INITIALIZE : start\n");

		mutex_lock(&wm_data->state_lock);
		
		/* Check state */
		if (wm_data->state & STS_BIT_MC_INIT_ONGOING) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCT_INITIALIZE : illegal state (state : "
				                                   "%#04x)\n", wm_data->state);
			return -EINVAL;
		}
		
		/* Change state to FJ_WM_STS_MC_INIT_ONGOING */
		wm_data->state = FJ_WM_STS_MC_INIT_ONGOING;

		mutex_unlock(&wm_data->state_lock);

		fj_wm_mc_init();

		if(wm_data->irq_flag == 0) {
			ret = request_irq(wm_data->motion_irq, fj_wm_irq_handler,
			                            IRQF_TRIGGER_RISING, "fj_wm", wm_data);
			if (ret < 0) {
				mutex_lock(&wm_data->state_lock);
				wm_data->state = FJ_WM_STS_MC_UNINITIALIZED;
				mutex_unlock(&wm_data->state_lock);
				printk(KERN_ERR "FJ_WM_IOCT_INITIALIZE : request_irq failed "
					                                      "(ret : %d)\n", ret);
				return ret;
			}
			wm_data->irq_flag = 1;
			debug_printk("FJ_WM_IOCT_INITIALIZE : irq_flag = %d\n",
				                                            wm_data->irq_flag);
		}
		
		/* Wait IRQ */
		if ((ret = wait_event_interruptible_timeout(
			                        wm_data->wait_queue_motion_irq,
			                        wm_data->state == FJ_WM_STS_MC_INITIALIZED,
			                        msecs_to_jiffies(250))) <= 0) {
			/* If canceled */
			if(wm_data->irq_flag == 1) {
				free_irq(wm_data->motion_irq, wm_data);
				wm_data->irq_flag = 0;
				debug_printk("FJ_WM_IOCT_INITIALIZE : irq_flag = %d\n",
					                                        wm_data->irq_flag);
			}
			mutex_lock(&wm_data->state_lock);
			/* Change state to FJ_WM_STS_MC_UNINITIALIZED */
			wm_data->state = FJ_WM_STS_MC_UNINITIALIZED;
			mutex_unlock(&wm_data->state_lock);
			                    	
			printk(KERN_ERR "FJ_WM_IOCT_INITIALIZE : wait_event_"
				           "interruptible_timeout canceled (ret : %d)\n", ret);
			return -ECANCELED;
		}
		
		if(wm_data->irq_flag == 1) {
			free_irq(wm_data->motion_irq, wm_data);
			wm_data->irq_flag = 0;
			debug_printk("FJ_WM_IOCT_INITIALIZE : irq_flag = %d\n",
				                                            wm_data->irq_flag);
		}

		break;

	/** Cancel initialization MC */
	case FJ_WM_IOCT_CANCELINITIALIZE:
		debug_printk("FJ_WM_IOCT_CANCELINITIALIZE : start\n");

		mutex_lock(&wm_data->state_lock);

		/* Check state */
		if (!(wm_data->state & STS_BIT_MC_INIT_ONGOING)) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCT_CANCELINITIALIZE : illegal state "
				                          "(state : %#04x)\n", wm_data->state);
			return -EINVAL;
		} else if (wm_data->state & STS_BIT_MOTION_POLLING) {
			/* Wake up */
			wake_up_interruptible(&wm_data->wait_queue_motion_irq);
		}

		/* Clear motion IRQ */
		if(wm_data->irq_flag == 1) {
			free_irq(wm_data->motion_irq, wm_data);
			wm_data->irq_flag = 0;
			debug_printk("FJ_WM_IOCT_CANCELINITIALIZE : irq_flag = %d\n",
				                                            wm_data->irq_flag);
		}

		mutex_unlock(&wm_data->state_lock);

		break;

	/** Request motiot IRQ */
	case FJ_WM_IOCS_REQUESTMOTIONIRQ:
		debug_printk("FJ_WM_IOCS_REQUESTMOTIONIRQ : start\n");

		mutex_lock(&wm_data->state_lock);

		/* Check state */
		if (wm_data->state != FJ_WM_STS_MC_INITIALIZED) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCS_REQUESTMOTIONIRQ : illegal state "
				                          "(state : %#04x)\n", wm_data->state);
			return -EINVAL;
		}

		if (arg == FJ_WM_EDGE_HIGH) {
			/* High-edge detection */
			debug_printk("FJ_WM_IOCS_REQUESTMOTIONIRQ : FJ_WM_EDGE_HIGH\n");
			if(wm_data->irq_flag == 0) {
				ret = request_irq(wm_data->motion_irq, fj_wm_irq_handler,
				                        IRQF_TRIGGER_RISING, "fj_wm", wm_data);
				if (ret < 0) {
					mutex_unlock(&wm_data->state_lock);
					printk(KERN_ERR "FJ_WM_IOCS_REQUESTMOTIONIRQ"
						 "(FJ_WM_EDGE_HIGH) : request_irq failed (ret : %d)\n",
						                                                  ret);
					return  ret;
				}
				wm_data->irq_flag = 1;
				debug_printk("FJ_WM_IOCS_REQUESTMOTIONIRQ(FJ_WM_EDGE_HIGH) : "
					                     "irq_flag = %d\n", wm_data->irq_flag);
			}
		} else if (arg == FJ_WM_EDGE_LOW) {
			/* Low-edge detection */
			debug_printk("FJ_WM_IOCS_REQUESTMOTIONIRQ : FJ_WM_EDGE_LOW\n");
			
			if(wm_data->irq_flag == 0) {
				ret = request_irq(wm_data->motion_irq, fj_wm_irq_handler,
				                       IRQF_TRIGGER_FALLING, "fj_wm", wm_data);
				if (ret < 0) {
					mutex_unlock(&wm_data->state_lock);
					printk(KERN_ERR "FJ_WM_IOCS_REQUESTMOTIONIRQ"
						  "(FJ_WM_EDGE_LOW) : request_irq failed (ret : %d)\n",
						                                                  ret);
					return  ret;
				}
				wm_data->irq_flag = 1;
				debug_printk("FJ_WM_IOCS_REQUESTMOTIONIRQ(FJ_WM_EDGE_LOW) : "
					                     "irq_flag = %d\n", wm_data->irq_flag);
			}
		} else {				/* Invalid arg */
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCS_REQUESTMOTIONIRQ : invalid argument "
				                                         "(arg : %ld)\n", arg);
			return -EINVAL;
		}
		/* FUJITSU:2011-08-04 Implement interrupt handling start */
#if 1   /* FUJITSU:2011-12-01 ICS mod start */
		irq_set_irq_wake(wm_data->motion_irq, 1);
#else
		set_irq_wake(wm_data->motion_irq, 1);
#endif  /* FUJITSU:2011-12-01 ICS mod end */
		wm_data->state |= STS_BIT_MOTION_POLLING; /* Set polling bit */
		mutex_unlock(&wm_data->state_lock);
		/* FUJITSU:2011-08-04 Implement interrupt handling end */

		break;

	/** Cancel request motion IRQ */
	case FJ_WM_IOCT_CANCELMOTIONIRQ:
		debug_printk("FJ_WM_IOCT_CANCELMOTIONIRQ : start\n");

		mutex_lock(&wm_data->state_lock);

		/* Check state */
		if (!(wm_data->state & STS_BIT_MOTION_POLLING)) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCT_CANCELMOTIONIRQ : illegal state(1) "
				                          "(state : %#04x)\n", wm_data->state);
			return -EINVAL; /* Error */
		}
		if (wm_data->state & STS_BIT_MC_INIT_ONGOING) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCT_CANCELMOTIONIRQ : illegal state(2) "
				                          "(state : %#04x)\n", wm_data->state);
			return -EINVAL; /* Error */
		}

		if(wm_data->irq_flag == 1) {
			free_irq(wm_data->motion_irq, wm_data);
			wm_data->irq_flag = 0;
			debug_printk("FJ_WM_IOCT_CANCELMOTIONIRQ : irq_flag = %d\n",
				                                            wm_data->irq_flag);
		}
		wm_data->state &= ~STS_BIT_MOTION_POLLING;
		input_report_abs(this_data, ABS_X, 0);
		input_sync(this_data);

		mutex_unlock(&wm_data->state_lock);

		break;

	/** Set terminal */
	case FJ_WM_IOCS_SETSCIFACONTROL:
		debug_printk("FJ_WM_IOCS_SETSCIFACONTROL : start\n");

		mutex_lock(&wm_data->state_lock);

		/* Check state */
		if (!(wm_data->state & STS_BIT_MC_INIT)) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCS_SETSCIFACONTROL : illegal state "
				                          "(state : %#04x)\n", wm_data->state);
			return -EINVAL; /* Error */
		}

		if (!(arg == FJ_WM_MODE_UART || arg == FJ_WM_MODE_GPIO)) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCS_SETSCIFACONTROL : invalid  argument "
				                                         "(arg = %ld)\n", arg);
			return -EINVAL;
		}

		mutex_unlock(&wm_data->state_lock);

		break;
/* FUJITSU:2011-07-26 Add new ioctl command start */
	/* Wakeup */
	case FJ_WM_IOCS_WAKEUPCONTROL:
		debug_printk("FJ_WM_IOCS_WAKEUPCONTROL : start\n");
		mutex_lock(&wm_data->state_lock);
		if(arg == FJ_WM_WAKEUP_HIGH){
			debug_printk("gpio_set_value(FJ_WM_GPIO_HOSU_WUP,GPIO_HIGH)\n");
			gpio_set_value(FJ_WM_GPIO_HOSU_WUP, GPIO_HIGH);
		}else if(arg == FJ_WM_WAKEUP_LOW){
			debug_printk("gpio_set_value(FJ_WM_GPIO_HOSU_WUP,GPIO_LOW)\n");
			gpio_set_value(FJ_WM_GPIO_HOSU_WUP, GPIO_LOW);
		}else{
			debug_printk("FJ_WM_IOCS_WAKEUPCONTROL : Not support Argument\n");
			mutex_unlock(&wm_data->state_lock);
			return -EINVAL;
		}
		mutex_unlock(&wm_data->state_lock);
		break;
/* FUJITSU:2011-07-26 Add new ioctl command end */
	default:
		printk(KERN_ERR "%s : invalid command (cmd : %d)\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

#if 1   /* FUJITSU:2011-12-01 ICS add start */
static long fj_wm_unlocked_ioctl( struct file *file, unsigned int cmd, unsigned long arg ){
	long lRet = 0;

	mutex_lock( &wm_data->io_lock );
	lRet = fj_wm_ioctl( file->f_dentry->d_inode, file, cmd, arg );
	mutex_unlock( &wm_data->io_lock );

	return lRet;
}
#endif  /* FUJITSU:2011-12-01 ICS add end */

/* FUJITSU:2011-08-04 Implement interrupt handling start */
static void
fj_wm_setup_cdev(struct cdev *dev, int minor,
		struct file_operations *fops)
{
	int err = 0;
	int devno = MKDEV(fj_wm_major, minor);

	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	dev->ops = fops;
	err = cdev_add(dev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_ERR "%s : cdev_add failed(err : %d)\n", __func__, err);
	if (IS_ERR(device_create(fj_wm_class, NULL, devno, NULL, DEV_NAME)))
		printk(KERN_ERR "%s : device_create failed\n", __func__);
}
/* FUJITSU:2011-08-04 Implement interrupt handling end */

/** Initialize file operations */
static struct file_operations fj_wm_fileops = {
	.owner		= THIS_MODULE,
	.open		= fj_wm_open,		/** open */
	.release	= fj_wm_release,	/** release */
#if 1   /* FUJITSU:2011-12-01 ICS mod start */
	.unlocked_ioctl	= fj_wm_unlocked_ioctl		/** ioctl */
#else
	.ioctl		= fj_wm_ioctl		/** ioctl */
#endif  /* FUJITSU:2011-12-01 ICS mod end */
};

/** Set up module
 *
 * @param pdev : Not use
 * @return 0  Success
 */
static int __devinit
fj_walkmotion_probe(struct platform_device *pdev)
{
	struct fj_wm_platform_data *pdata;
	int ret;
	/* FUJITSU:2011-08-04 Implement interrupt handling start */
	static struct input_dev *input_data = NULL;
	/* FUJITSU:2011-08-04 Implement interrupt handling end */

	debug_printk("%s : start\n", __func__);
	wm_data = kzalloc(sizeof(*wm_data), GFP_KERNEL);
	if (!wm_data) {
		printk(KERN_ERR "%s : could not allocate memory\n", __func__);
		return -ENOMEM;
	}

	/* FUJITSU:2011-08-04 Implement interrupt handling start */
	input_data = input_allocate_device();
	if(!input_data) {
		printk(KERN_ERR "%s : input_allocate_device failed\n", __func__);
		kfree(wm_data);
		ret = -ENOMEM;
	}
	set_bit(EV_ABS, input_data->evbit);
	input_set_capability(input_data, EV_ABS, ABS_X);
	input_data->name = DRIVER_NAME;
	
	ret = input_register_device(input_data);
	if(ret) {
		input_unregister_device(input_data);
		goto err;
	}
	
	this_data = input_data;
	/* FUJITSU:2011-08-04 Implement interrupt handling end */

	mutex_init(&wm_data->state_lock);			/* Initialize */
#if 1   /* FUJITSU:2011-12-01 ICS add start */
	mutex_init(&wm_data->io_lock);			/* Initialize */
#endif  /* FUJITSU:2011-12-01 ICS add end */

	/* Initialize work queue */
	INIT_WORK(&work_queue_motion_irq, fj_wm_motion_irq);
	/* Initialize wait queue */
	init_waitqueue_head(&wm_data->wait_queue_motion_irq);

	pdata = pdev->dev.platform_data;
	if (pdata->mc_init_delay < 0) {
		wm_data->mc_init_delay = FJ_WM_DEF_MC_INIT_DELAY;
	} else {
		wm_data->mc_init_delay = pdata->mc_init_delay;
	}
	wm_data->motion_irq = pdata->motion_irq;

	wm_data->state = FJ_WM_STS_INITIALIZER;
	wm_data->dbg_dev = &pdev->dev;
	wm_data->irq_flag = 0;

	return 0;

err:
	kfree(wm_data);
	return ret;
}

/** Remove module
 *
 * @param pdev : not use
 * @return 0  success
 */
static int __devexit fj_walkmotion_remove(struct platform_device *pdev)
{
	debug_printk("%s : start\n", __func__);
	cancel_work_sync(&work_queue_motion_irq);
	/* FUJITSU:2011-08-04 Implement interrupt handling start */
	if (this_data != NULL) {
		input_unregister_device(this_data);
	}
	/* FUJITSU:2011-08-04 Implement interrupt handling end */
	kfree(wm_data);

	return 0;
}

/** Walk motion driver */
static struct platform_driver fj_walkmotion_driver = {
	.driver.name	= DRIVER_NAME,
	.driver.owner	= THIS_MODULE,
	.probe		= fj_walkmotion_probe,
	.remove		= __devexit_p(fj_walkmotion_remove),
};

/** Init module
 *
 * @return 0  Success
 *         !0 Fail
 */
static int __init fj_walkmotion_init(void)
{
	int ret = 0;
	dev_t dev = 0;
	
	debug_printk("%s : start", __func__);
	
	/* FUJITSU:2011-08-04 Implement interrupt handling start */
	fj_wm_class = class_create(THIS_MODULE, DEV_NAME);
	if(IS_ERR(fj_wm_class)) {
		ret = PTR_ERR(fj_wm_class);
		printk(KERN_ERR "%s : class_create error(err : %d )\n", __func__, ret);
		
		return ret;
	}
	
	ret = alloc_chrdev_region(&dev, 0, 2, DEV_NAME);
	if (ret < 0) {
		printk(KERN_ERR "%s : Can't allocate chrdev region(ret : %d)\n",
			                                                    __func__, ret);
		return ret;
	}
	fj_wm_major = MAJOR(dev);
	if (fj_wm_major == 0)
		fj_wm_major = ret;
	
	fj_wm_setup_cdev(&fj_wm_cdev, 0, &fj_wm_fileops);
	/* FUJITSU:2011-08-04 Implement interrupt handling end */
	
	return platform_driver_register(&fj_walkmotion_driver);
}
module_init(fj_walkmotion_init);

/** Exit module
 *
 * @return void
 */
static void __exit fj_walkmotion_exit(void)
{
	debug_printk("%s : start\n", __func__);
	class_destroy(fj_wm_class);
	platform_driver_unregister(&fj_walkmotion_driver);
}
module_exit(fj_walkmotion_exit);

MODULE_ALIAS("platform:fj-walkmotion");
MODULE_AUTHOR("FUJITSU LIMITED");
MODULE_DESCRIPTION("Fujitsu Walk Motion MC Driver");
MODULE_LICENSE("GPL");
