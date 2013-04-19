/*
 * ftadrv_smd.c - FTA (Fault Trace Assistant) 
 *                driver for shared memory
 *
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

/* ==========================================================================
 *  INCLUDE HEADER
 * ========================================================================== */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/wakelock.h>

#include <mach/msm_smd.h>

#include <asm/uaccess.h>

#include "smd_private.h"

#include "fta_config.h"

/* ==========================================================================
 *  DEFINITION
 * ========================================================================== */
static smd_channel_t *channel;
static struct workqueue_struct *smd_work_queue;
static struct work_struct read_smd_work;
static int dummy_priv;
static void ftadrv_smd_notify(void *ctxt, unsigned event);

extern void ftadrv_exec_cmd(const char *cmd,int cmdlen);
extern ssize_t ftadrv_exec_write(const char *buf,size_t size);

/* ==========================================================================
 *  INTERNAL FUNCTIONS
 * ========================================================================== */

/*!
 @brief ftadrv_smd_write

 process for the read data

 @param [in]  buf   pointer of the write data
 @param [in]  len   size of write data

 @retval      0> size of write data
*/
int ftadrv_smd_write(char *buf,int len)
{
    if(!channel) {
      int ret = smd_open("DATA20", &channel, &dummy_priv, ftadrv_smd_notify);
      if(ret < 0) {
         printk(KERN_ERR "smd is not opened %d\n",ret);
      return -1;
    }
    }
    printk(KERN_INFO "write %p %d\n",buf,len);
    return smd_write(channel,buf,len);
}
EXPORT_SYMBOL(ftadrv_smd_write);

static struct completion log_comp;
static struct procDataType *curr;
static struct wake_lock smd_wake_lock;
void ftadrv_smd_getlog(struct procDataType *pd)
{

   char *cmdbuf;

   if(curr) {
     printk(KERN_ERR "getlog is busy\n");
     pd->ret = -EBUSY;
     return;
   }
   cmdbuf= kmalloc(16,GFP_KERNEL);
   if(!cmdbuf) {
     printk(KERN_ERR "no heap\n");
     pd->ret = -ENOSPC;
     return;
   }
   cmdbuf[0] = 0xd0;
   cmdbuf[1] = 0x00;
   *(unsigned short *)&cmdbuf[2] = 9;
   switch(pd->type) {
   case FTA_RA:
      strcpy(&cmdbuf[4],"GETA");
      break;
   case FTA_RC:
      strcpy(&cmdbuf[4],"GETC");
      break;
   case FTA_CC:
      strcpy(&cmdbuf[4],"GETS");
      break;
   default:
     return;
   }
   cmdbuf[8] = 0;
   curr = pd;
   wake_lock(&smd_wake_lock);
   ftadrv_smd_write(cmdbuf,9);
   INIT_COMPLETION(log_comp);
   printk(KERN_INFO "wait_comp(smd)\n");
   wait_for_completion_timeout(&log_comp,(10*HZ));
   strcpy(&cmdbuf[4],"EXIT");
   ftadrv_smd_write(cmdbuf,9);
   kfree(cmdbuf);
   curr = 0;
   wake_unlock(&smd_wake_lock);
   return;
}
EXPORT_SYMBOL(ftadrv_smd_getlog);

static int ftadrv_smd_cmd(char *buf)
{
    static unsigned long total;
    printk(KERN_INFO "smd_cmd %d %d %d\n",buf[0],buf[1],*(unsigned short *)&buf[2]);

    if(!curr) {
      return 0;
    }
    switch(*buf) {
    case 0x61:
    {
        total = *(unsigned long *)&buf[4];
        curr->ptr = vmalloc(total);
        if(curr->ptr == 0) {
            printk(KERN_ERR "alloc error %ld\n",total);
            buf[0] = 0xd3;
            *(unsigned short *)&buf[2] = 4;
            ftadrv_smd_write(buf,4);
            curr->ret = -ENOSPC;
            total = 0;
            printk(KERN_INFO "complete(smd)\n");
            complete(&log_comp);
        }
        curr->size = 0; /* guard */
        break;
    }
    case 0x62:
    {
        int len = *(unsigned short *)&buf[2] - 4;
        if(curr->ptr == 0 || total == 0 ) {
          printk(KERN_ERR "discard %d\n",len);
          break;
        }
        if(len <= 0) {
            buf[0] = 0xd3;
            *(unsigned short *)&buf[2] = 4;
            ftadrv_smd_write(buf,4);
            vfree(curr->ptr);
            curr->size = 0;
            curr->ret = -EIO;
            printk(KERN_INFO "complete(smd)\n");
            complete(&log_comp);
            break;
         }
        len = min_t(size_t,total-curr->size,len);
        memcpy(&curr->ptr[curr->size],&buf[4],len);
        curr->size += len;
        buf[0] = 0xd2;
        *(unsigned short *)&buf[2] = 4;
        len = ftadrv_smd_write(buf,4);
        if(len != 4) {
            printk(KERN_ERR "write failed %d\n",len);
        }
        break;
    }
    case 0x63:
        total = 0;
        printk(KERN_INFO "complete(smd)\n");
        complete(&log_comp);
        break;
    case 0x0a:
    {
        int len = *(unsigned short *)&buf[2] - 4;
        ftadrv_exec_cmd(&buf[4],len);
        ftadrv_exec_write(&buf[4],len);
        break;
    }
    default:
        break;
    }
    return 0;
}

/*!
 @brief ftadrv_read_smd_work

 process for the read event

 @param [in]  buf   pointer of the read data

 @retval      none
*/
static void ftadrv_read_smd_work(struct work_struct *work)
{
    unsigned char *buf;
    mm_segment_t oldfs;

    buf = kmalloc(4096,GFP_KERNEL);
    if (!buf) {
       printk(KERN_ERR "ftadrv: nomem\n");
       return;
    }
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    while(1) {
        int ret = smd_read_avail(channel);
        if (ret <= 0) {
            printk(KERN_DEBUG "ftadrv: packet too short %d\n",ret);
            break;
        }
        if (ret > 4096) {
            printk(KERN_ERR "ftadrv: packet too large %d\n",ret);
            break;
        }
        if(smd_read(channel, buf, ret) != ret) {
            printk(KERN_ERR "ftadrv: not enough data %d\n",ret);
            break;
        }
        if(ftadrv_smd_cmd(buf)) {
            set_fs(oldfs);
            return;
        }
    }
    kfree(buf);
    set_fs(oldfs);
}

/*!
 @brief ftadrv_read_smd_work

 set the read event to workqueue

 @param [in]  buf   pointer of the read data

 @retval      none
*/
static void ftadrv_smd_notify(void *ctxt, unsigned event)
{
    queue_work(smd_work_queue, &read_smd_work);
}

/*!
 @brief ftadrv_smd_probe

 probe the smd driver

 @param [in]  pdev   pointer of the device struct

 @retval      0: success
*/
static int __init ftadrv_smd_probe(struct platform_device *pdev)
{
    int ret;
    printk(KERN_INFO "ftadrv prove %d\n",pdev->id);
    ret = smd_open("DATA20", &channel, &dummy_priv, ftadrv_smd_notify);

    if(ret < 0)
      channel = 0;
    printk(KERN_INFO "ftadrv %d = smd_open()\n",ret);
    return 0;
}

static struct platform_driver ftadrv_smd_driver = {
    .driver = {
           .name = "FTA",
           .owner = THIS_MODULE,
    },
};
static struct platform_device *ftadrv_device;

/* ==========================================================================
 *  EXTERNAL FUNCTIONS
 * ========================================================================== */
/*!
 @brief ftadrv_smd_init

 initialize the smd driver

 @param [in]  none

 @retval      0: success
*/
int __init ftadrv_smd_init(void)
{
   int ret;
   smd_work_queue = create_workqueue("ftadrv_smd");
   INIT_WORK(&read_smd_work, ftadrv_read_smd_work);
   init_completion(&log_comp);
   wake_lock_init(&smd_wake_lock, WAKE_LOCK_SUSPEND, "fta_smd");

   ftadrv_device = platform_device_alloc("FTA",0);
   if(ftadrv_device == 0)
     return -ENOMEM;

   ret = platform_device_add(ftadrv_device);
   if(ret) {
     printk(KERN_ERR "%d = platform_device_add\n",ret);
     goto error_return;
   }
   ret = platform_driver_probe(&ftadrv_smd_driver,&ftadrv_smd_probe);
   if(ret) {
     printk(KERN_ERR "%d = platform_driver_probe\n",ret);
     platform_device_del(ftadrv_device);
   }
   return 0;
error_return:
   platform_device_put(ftadrv_device);
   return ret;
}

MODULE_DESCRIPTION("FTA Driver");
MODULE_LICENSE("GPL v2");
