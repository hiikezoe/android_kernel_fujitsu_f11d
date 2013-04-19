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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <asm/current.h>
#include <linux/slab.h>
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include "../../../arch/arm/mach-msm/smd_private.h"
#include "testsmem.h"
#include "../../../arch/arm/include/asm/memory.h"

MODULE_DESCRIPTION("testsmem Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

#define SMEM_OFFSET 0xE0000000
#define TESTSMEM_DISP_DEBUG

struct testsmem_dev        *tesmemdriver;
volatile test_maif_type    *cmd_ptr;
static ma_if_command_type  *Acmd;
static ma_if_disp_type     *Adisp_ptr;
static ma_if_sio_type      *Asio_ptr;
volatile int               *Astatus;
static ma_if_ask_type      *Areq_cmd;

static void testsmem_diap_print(void)
{
#ifdef TESTSMEM_DISP_DEBUG
  int i, max_line;

  if(Adisp_ptr->valid == TRUE)
  {
    if((Adisp_ptr->format) == MA_IF_DISP_TYPE_14x15)
    {
      max_line = MA_DISP_HIGHT14;
    }
    else if((Adisp_ptr->format) == MA_IF_DISP_TYPE_28x30)
    {
      max_line = MA_DISP_HIGHT28;
    }
    else
    {
      max_line = MA_DISP_HIGHT28;
    }
    for(i = 0; i < max_line; i++)
    {
      if( Adisp_ptr->line[i].valid == TRUE )
      {
        Adisp_ptr->line[i].data[MA_DISP_WIDTH30-1] = '\0';
        printk(KERN_INFO "%s\n",&Adisp_ptr->line[i].data[0]);
      }
      else
      {
        printk(KERN_INFO " \n");
      }
    }
  }
#endif
}

static int testsmem_send_command(void)
{
  int result;
  unsigned *data;

  data = (unsigned *)(cmd_ptr - SMEM_OFFSET);
  result = msm_proc_comm(PCOM_OEM_002, (unsigned *)data, NULL);
  return result;
}

static int testsmem_open(struct inode *inode, struct file *file)
{
  int result;
  long tout = 60000;

  result = msm_proc_comm(PCOM_OEM_001, NULL, NULL);
  msleep(100);

  if (tesmemdriver) {

    cmd_ptr = (test_maif_type *)smem_alloc( SMEM_ID_VENDOR2, sizeof(test_maif_type));
    if(cmd_ptr == NULL)
    {
       printk(KERN_INFO "testsmem_open not yet smem allocate \n");
       mdelay(10);
       cmd_ptr = (test_maif_type *)smem_alloc( SMEM_ID_VENDOR2, sizeof(test_maif_type));

      if(cmd_ptr == NULL)
      {
        printk(KERN_INFO "smem allocate err\n");
        return -ENOMEM;
      }
    }
    mutex_lock(&tesmemdriver->testsmem_mutex);

    Acmd      =  (ma_if_command_type *)&cmd_ptr->CmdBuf;
    Adisp_ptr =  (ma_if_disp_type *)&cmd_ptr->DisplayBuf;
    Asio_ptr  =  (ma_if_sio_type *)&cmd_ptr->SioBuf;
    Astatus   =  (int *)&cmd_ptr->Status;
    Areq_cmd  =  (ma_if_ask_type *)&cmd_ptr->Req_cmd;

    memset(Adisp_ptr,0,sizeof(ma_if_disp_type));
    Adisp_ptr->format = MA_IF_DISP_TYPE_14x15;
    Adisp_ptr->valid = TRUE;
    Adisp_ptr->line[0].valid = TRUE;
    memcpy(&Adisp_ptr->line[0].data[0], "               ", MA_DISP_WIDTH15);
    Adisp_ptr->line[1].valid = TRUE;
    memcpy(&Adisp_ptr->line[1].data[0], "               ", MA_DISP_WIDTH15);
    Adisp_ptr->line[2].valid = TRUE;
    memcpy(&Adisp_ptr->line[2].data[0], "               ", MA_DISP_WIDTH15);
    Adisp_ptr->line[3].valid = TRUE;
    memcpy(&Adisp_ptr->line[3].data[0], "               ", MA_DISP_WIDTH15);

    testsmem_diap_print();
    Acmd->cmd = 0;
    *Astatus = TEST_REQUEST;
    Areq_cmd->req_flg = FALSE;
    Areq_cmd->req_cmd_no = 0xFFFFFFFF;
    msleep(50);

    result = testsmem_send_command();

    while(tout > 0)
    {
      if( (*Astatus & TEST_RESP_END) != 0 )
      {
        break;
      }
      tout--;
      mdelay(10);
    }
    msleep(50);

    Acmd->cmd = 1;
    mutex_unlock(&tesmemdriver->testsmem_mutex);
    return 0;
  }
  return -EFAULT;
}

static int testsmem_close(struct inode *inode, struct file *file)
{
  if (tesmemdriver) {
    mutex_lock(&tesmemdriver->testsmem_mutex);


    mutex_unlock(&tesmemdriver->testsmem_mutex);
    return 0;
  }
  return -EFAULT;
}

static int testsmem_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
  int ret = 0;
  long timeout = 60000;

  if( cmd_ptr == NULL )
  {
    printk(KERN_INFO "testsmem : cmd_ptr NULL\n");
    ret = -EFAULT;
    return ret;
  }

  while(*Astatus != TEST_RESP_END)
  {
    if(*Astatus == TEST_RESP_CONT)
    {
      printk(KERN_INFO "testsmem : TEST_RESP_CONT \n");
      testsmem_diap_print();

      if(Asio_ptr->size != 0)
      {
        if (copy_to_user(buf, (void *)&Asio_ptr->data, Asio_ptr->size))
        {
          ret = -EFAULT;
          return ret;
        }
        printk(KERN_INFO "testsmem : TEST_RESP_CONT ret size !0 \n");
        ret = Asio_ptr->size;
      }
      else
      {
        ret = 0;
      }
      memset(Asio_ptr,0,sizeof( ma_if_sio_type ));
      *Astatus = TEST_RESP_REQ;
    }
    else
    {
      printk(KERN_INFO "testsmem : Astate err %x \n",*Astatus);
      ret = -EFAULT;
      return ret;
    }

    timeout = 60000;
    while(*Astatus == TEST_RESP_REQ)
    {
      timeout--;
      mdelay(10);
      if(timeout == 0)
      {
        printk(KERN_INFO "testsmem : wait !TEST_RESP_REQ timeout \n");
        break;
      }
    }
  }

  if(*Astatus == TEST_RESP_END)
  {
    testsmem_diap_print();

    if(Asio_ptr->size != 0)
    {
      if (copy_to_user(buf, (void *)&Asio_ptr->data, Asio_ptr->size))
      {
        ret = -EFAULT;
        return ret;
      }
      ret = Asio_ptr->size;
      memset(Asio_ptr,0,sizeof( ma_if_sio_type ));
    }
    else
    {
      ret = 0;
      memset(Asio_ptr,0,sizeof( ma_if_sio_type ));
    }
  }
  else
  {
    printk(KERN_INFO "testsmem : Astatus err %x\n",*Astatus);
    ret = -EFAULT;
    return ret;
  }
  return ret;
}

static int testsmem_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
  int ret = 0;
  int err;
  int result;
  long timeout = 60000;

  if((MA_CMD_BUFF_SIZE < count) || (count < 4))
  {
    printk(KERN_INFO "testsmem : data length err count = %d\n",count);
    ret = -EFAULT;
    return ret;
  }

  err = copy_from_user(&Acmd->command[0], buf, count);
  if (err) {
    printk(KERN_ERR "testsmem : copy_from_user failed\n");
    ret = -EFAULT;
    *Astatus = TEST_RESP_END;
    memset(&Acmd->command[0],0,MA_CMD_BUFF_SIZE);
    goto fail_free_copy;
  }

  *Astatus = TEST_REQUEST;
  result = testsmem_send_command();

  timeout = 60000;
  while(*Astatus == TEST_REQUEST)
  {
    timeout--;
    mdelay(10);
    if(timeout == 0)
    {
      break;
    }
  }

  fail_free_copy:
  return ret;
}

/* FUJITSU:2012-01-12 DN ICS_001 start */
#if 0
| static int testsmem_ioctl(struct inode *inode, struct file *filp,
|            unsigned int iocmd, unsigned long ioarg)
|     .unlocked_ioctl = testsmem_ioctl,
#endif /* 0 */
static long testsmem_ioctl(struct file *filp,
           unsigned int iocmd, unsigned long ioarg)
/* FUJITSU:2012-01-12 DN ICS_001 end */
{
/* FUJITSU:2012-01-12 DN ICS_001 start */
/*   int ret = 0;	*/
  long ret = 0;
/* FUJITSU:2012-01-12 DN ICS_001 end */

  if( Adisp_ptr == NULL )
  {
    printk(KERN_INFO "testsmem : Adisp_ptr NULL\n");
    return -EFAULT;
  }

    if ( copy_to_user((void *)ioarg,Adisp_ptr,sizeof(ma_if_disp_type)))
    {
      ret = -EFAULT;
    }else
    {
      ret = 0;
    }
  return ret;
}

static const struct file_operations testsmemfops = {
    .owner = THIS_MODULE,
    .read = testsmem_read,
    .write = testsmem_write,
    .open = testsmem_open,
/* FUJITSU:2012-01-12 DN ICS_001 start */
/*     .ioctl = testsmem_ioctl, */
    .unlocked_ioctl = testsmem_ioctl,
/* FUJITSU:2012-01-12 DN ICS_001 end */
    .release = testsmem_close
};

static int testsmem_setup_cdev(dev_t devno)
{

  int err;

  cdev_init(tesmemdriver->cdev, &testsmemfops);

  tesmemdriver->cdev->owner = THIS_MODULE;
  tesmemdriver->cdev->ops = &testsmemfops;

  err = cdev_add(tesmemdriver->cdev, devno, 1);

  if (err) {
    printk(KERN_INFO "testsmem cdev registration failed !\n\n");
    return -1;
  }

  tesmemdriver->testsmem_class = class_create(THIS_MODULE, "tsm");

  if (IS_ERR(tesmemdriver->testsmem_class)) {
    printk(KERN_ERR "Error creating testsmem class.\n");
    return -1;
  }
  device_create(tesmemdriver->testsmem_class, NULL, devno, (void *)tesmemdriver, "tsm");

  return 0;
}

static int testsmem_cleanup(void)
{
  if (tesmemdriver) {
    if (tesmemdriver->cdev) {
      device_destroy(tesmemdriver->testsmem_class,
                     MKDEV(tesmemdriver->major,
                     tesmemdriver->minor_start));
                     cdev_del(tesmemdriver->cdev);
    }
    if (!IS_ERR(tesmemdriver->testsmem_class))
      class_destroy(tesmemdriver->testsmem_class);
    kfree(tesmemdriver);
  }
  return 0;
}

static int __init testsmem_init(void)
{
  dev_t dev;
  int error;

  tesmemdriver = kzalloc(sizeof(struct testsmem_dev) + 5, GFP_KERNEL);
  if (tesmemdriver) {
    mutex_init(&tesmemdriver->testsmem_mutex);
    tesmemdriver->num = 1;
    tesmemdriver->name = ((void *)tesmemdriver) + sizeof(struct testsmem_dev);
    strlcpy(tesmemdriver->name, "tsm", 3);

    error = alloc_chrdev_region(&dev, tesmemdriver->minor_start,
                   tesmemdriver->num, tesmemdriver->name);
    if (!error) {
      tesmemdriver->major = MAJOR(dev);
      tesmemdriver->minor_start = MINOR(dev);
    } else {
      printk(KERN_INFO "Major number not allocated\n");
      goto fail;
    }
    tesmemdriver->cdev = cdev_alloc();
    error = testsmem_setup_cdev(dev);
    if (error)
      goto fail;
  } else {
    printk(KERN_INFO "kzalloc failed\n");
    goto fail;
  }

  return 0;

fail:
  testsmem_cleanup();
  return -1;

}

static void __exit testsmem_exit(void)
{
    printk(KERN_INFO "testsmem exiting ..\n");
    testsmem_cleanup();
    printk(KERN_INFO "done testsmem exit\n");
}

module_init(testsmem_init);
module_exit(testsmem_exit);
