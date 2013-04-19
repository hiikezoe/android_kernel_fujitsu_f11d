/*
 * ftadrv.c - FTA (Fault Trace Assistant) 
 *            driver main
 *
 * Copyright(C) 2011-2012 FUJITSU LIMITED
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
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/uio.h>
#include <linux/list.h>

#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/ioctls.h>
#include <linux/reboot.h>  /* emargency_restart */

#include <asm/cacheflush.h>
#include <linux/vmalloc.h>

#include "fta_i.h"
#include "fta_config.h"

#if defined (T_QCI_IS3)
inline int qci_get_offline_charging(void);
#endif

/* ==========================================================================
 *  DEFINITION
 * ========================================================================== */
static DEFINE_SPINLOCK(work_lock);
static struct workqueue_struct *work_queue;
static struct work_struct write_work;
static struct work_struct proc_work;
static struct list_head write_head;
static struct list_head proc_head;
#define MAX_BUF_SIZE 3072
struct writeDataType {
    struct list_head list;
    int len;
    char buf[MAX_BUF_SIZE];
};

struct readDataMngType {
    int init;
    wait_queue_head_t wq;
    struct list_head head;
};
static struct readDataMngType readDataMng;
static DEFINE_SPINLOCK(read_lock);

struct readDataType {
    struct list_head list;
    char buf[128];
};

static int ftadrv_cnt;

extern void fta_log_message_kmsg(const unsigned char *srcname, unsigned long info,  const unsigned char *time, const unsigned char *msg,unsigned long msglen);
extern void fta_initialize(void);
extern void fta_terminate(void);
extern unsigned char *fta_var_alloc_log(unsigned long key,unsigned long len);

extern int ftadrv_acpu_init(struct proc_dir_entry *entry);
extern int ftadrv_ccpu_init(struct proc_dir_entry *entry);
extern int __init ftadrv_smd_init(void);

extern int fta_convert_buf(unsigned char *buf);
extern void fta_write(int area,unsigned char *buf,unsigned long len);
extern void ftadrv_smd_write(char *buf,int len);
extern void ftadrv_smd_getlog(struct procDataType *pd);

/* ==========================================================================
 *  INTERNAL FUNCTIONS
 * ========================================================================== */
/*!
 @brief ftadrv_write_work

 process for the write event

 @param [in]  work   pointer of the workqueue struct

 @retval      none
*/
static void ftadrv_write_work(struct work_struct *work)
{
    struct writeDataType *p,*tmp;
    unsigned long irqflags;

    spin_lock_irqsave(&work_lock, irqflags);
    list_for_each_entry_safe(p,tmp,&(write_head),list) {
        int area = fta_convert_buf(p->buf);
        if(area < 0) {
          /* silent discard */
        } else if(area & 0x80) {
            ftadrv_smd_write(p->buf,p->len);
        } else {
            fta_write(area,&p->buf[4],p->len-4);
        }
        list_del(&p->list);
        kfree(p);
    }
    spin_unlock_irqrestore(&work_lock, irqflags);
}

static void ftadrv_proc_work(struct work_struct *work)
{
    unsigned long irqflags;
    struct procDataType *pd;
    char *ptr;

    spin_lock_irqsave(&work_lock, irqflags);
    list_for_each_entry(pd,&proc_head,list) {
      if (pd->state == 0) {
         pd->state = 1;
         spin_unlock_irqrestore(&work_lock, irqflags);
         goto next;
      }
    }
    spin_unlock_irqrestore(&work_lock, irqflags);
    printk(KERN_ERR "proc no work\n");
    pd->ret = -EBUSY;
    complete(&pd->comp);
    return;

next:
    switch(pd->type) {
    case FTA_RA:
    case FTA_RC:
    case FTA_CC:
       ftadrv_smd_getlog(pd);
       if (pd->ptr == 0 || pd->size == 0 || pd->ret != 0) {
          printk(KERN_INFO "no log\n");
          pd->ret = -ENOENT;
          break;
       }
       break;
   case FTA_CA:
       ptr = vmalloc(FTA_CONFIG_STORE_MEMORY_SIZE);
       if (ptr == 0) {
          printk(KERN_ERR "vmalloc error\n");
          pd->ptr = (char *)fta_log_area;
          pd->size = FTA_CONFIG_STORE_MEMORY_SIZE;
          break;
       }
       FTA_ENTER_LOCK();
       memcpy(ptr,(char *)fta_log_area,FTA_CONFIG_STORE_MEMORY_SIZE);
       FTA_LEAVE_LOCK();
       pd->ptr = ptr;
       pd->size = FTA_CONFIG_STORE_MEMORY_SIZE;
       break;
   default:
       pd->ret = -EINVAL;
   }
   complete(&pd->comp);
}

/*!
 @brief ftadrv_open

 driver open function

 @param [in]  inode   pointer of the inode struct
 @param [in]  file    pointer of the file struct

 @retval      0: success
 @retval    <>0: failed
*/
static int ftadrv_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	if (file->f_mode & FMODE_READ) {
        if(readDataMng.init) {
            return -EBUSY;
        }
        INIT_LIST_HEAD(&readDataMng.head);
        init_waitqueue_head(&readDataMng.wq);
        file->private_data = &readDataMng;
        readDataMng.init = 1;
        return 0;
    }
    /* non blocking */
	ret = nonseekable_open(inode, file);
    return ret;
}

/*!
 @brief ftadrv_close

 driver close function

 @param [in]  inode   pointer of the inode struct
 @param [in]  file    pointer of the file struct

 @retval      0: success
 @retval    <>0: failed
*/
static int ftadrv_close(struct inode *inode, struct file *file)
{
	int ret = 0;
	if (file->f_mode & FMODE_READ) {
        struct readDataMngType *mng = file->private_data;
        struct readDataType *p,*tmp;
        unsigned long irqflags;
        spin_lock_irqsave(&read_lock, irqflags);
        if(!mng->init) {
            ret = -EBADF;
            goto end;
        }
        list_for_each_entry_safe(p,tmp,&(mng->head),list) {
            list_del(&p->list);
            kfree(p);
        }
        mng->init = 0;
end:
        spin_unlock_irqrestore(&read_lock, irqflags);
    }
    return ret;
}

/*!
 @brief exec_write

 set the write event to workqueue 

 @param [in/out]  wd   pointer of the write data struct
 @param [in]      len  length of the write data

 @retval      0: success
            <>0: failed
*/
static ssize_t exec_write(struct writeDataType *wd,unsigned long len)
{
    unsigned long irqflags;

    wd->len = len;
    spin_lock_irqsave(&work_lock, irqflags);
    INIT_LIST_HEAD(&wd->list);
    list_add_tail(&wd->list,&write_head);
    queue_work(work_queue, &write_work);
    spin_unlock_irqrestore(&work_lock, irqflags);
    return len;
}

/*!
 @brief ftadrv_aio_write

 driver write function

 @param [in]  kiocb    pointer of the asynchronous IO struct
 @param [in]  iov      pointer of the write data
 @param [in]  nr_segs  number of the iov
 @param [in]  loff_t   offset

 @retval     >0: success
             <0: failed
*/
static ssize_t ftadrv_aio_write(struct kiocb *iocb, const struct iovec *iov,
			 unsigned long nr_segs, loff_t ppos)
{
    ssize_t total = 0;
    struct writeDataType *wd;

    wd = kmalloc(sizeof(struct writeDataType),GFP_KERNEL);
    if (!wd) {
       return -ENOSPC;
    }
    while(nr_segs-- > 0) {
        size_t len;
        len = min_t(size_t,iov->iov_len,MAX_BUF_SIZE-total);
        if (copy_from_user(&wd->buf[total],iov->iov_base,len)) {
            kfree(wd);
            return -EFAULT;
        }
        iov++;
        total += len;
        if(total >= MAX_BUF_SIZE) break;
    }
    return exec_write(wd,total);
}

#if 0
/*!
 @brief ftadrv_ioctl

 driver ioctl function

 @param [in]  inode    pointer of the inode struct
 @param [in]  file     pointer of the file struct
 @param [in]  iocmd    ioctl command no
 @param [in]  ioarg    ioctl command arguments

 @retval      0: success
*/
static int ftadrv_ioctl(struct inode *inode, struct file *filp, unsigned int iocmd, unsigned long ioarg)
{
    return 0;
}
#endif

/*!
 @brief ftadrv_read

 driver read function

 @param [in]  file     pointer of the file struct
 @param [in]  buf      pointer of the read data
 @param [in]  count    length of the read data
 @param [out] pos      pointer of the offset of the read data

 @retval      >=0: success
               <0: failed
*/
static ssize_t ftadrv_read(struct file *file, char __user *buf,
			   size_t count, loff_t *pos)
{
    struct readDataMngType *mng = file->private_data;
    struct readDataType *p,*tmp;
	ssize_t ret;
    unsigned long irqflags;

	DEFINE_WAIT(wait);

start:
	while (1) {
		prepare_to_wait(&mng->wq, &wait, TASK_INTERRUPTIBLE);

        spin_lock_irqsave(&read_lock, irqflags);
		ret = list_empty(&mng->head);
        spin_unlock_irqrestore(&read_lock, irqflags);
		if (!ret)
			break;

		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			ret = -EINTR;
			break;
		}
		schedule();
	}

	finish_wait(&mng->wq, &wait);
	if (ret)
        return ret; /* error return */

    spin_lock_irqsave(&read_lock, irqflags);
    list_for_each_entry_safe(p,tmp,&(mng->head),list) {
	    int len = min(count, sizeof(p->buf));
	    if (copy_to_user(buf, p->buf, len))
		    ret = -EFAULT;
        else
            ret = len;
        list_del(&p->list);
        kfree(p);
        break;
    }
    spin_unlock_irqrestore(&read_lock, irqflags);

    if(!ret)
        goto start;
	return ret;
}

/*!
 @brief ftadrv_poll

 driver poll function

 @param [in]  file     pointer of the file struct
 @param [in]  wait     pointer of the poll struct

 @retval       >0: success
               <0: failed
*/
static unsigned int ftadrv_poll(struct file *file, poll_table *wait)
{
    struct readDataMngType *mng = file->private_data;
	unsigned int ret = POLLOUT | POLLWRNORM;
    unsigned long irqflags;

	if (!(file->f_mode & FMODE_READ))
		return ret;

	poll_wait(file, &mng->wq, wait);

    spin_lock_irqsave(&read_lock, irqflags);
	if (!list_empty(&mng->head))
		ret |= POLLIN | POLLRDNORM;
    spin_unlock_irqrestore(&read_lock, irqflags);

	return ret;
}

/*!
 @brief chrnul

 search for the character or NULL in string

 @param  [in]  str     pointer of string
 @param  [in]  ch      charcter of searching for

 @retval       >0:     length to the found character or NULL
*/
static unsigned long chrnul(const char *str,const char ch,unsigned long *info)
{
  const unsigned char *p = str;
  for(;*p && *p != ch;p++);
  if (*p != ch) {
    *info |= 0x00000080;
  }
  return (unsigned long)p - (unsigned long)str;
}

/* ==========================================================================
 *  DRIVER INITIALIZE/TERMINATE FUNCTIONS
 * ========================================================================== */
static const struct file_operations ftadrv_fops = {
    .owner   = THIS_MODULE,
    .open    = ftadrv_open,
    .release = ftadrv_close,
    .aio_write = ftadrv_aio_write,
    .read    = ftadrv_read,
    .poll    = ftadrv_poll,
//    .ioctl   = ftadrv_ioctl,
};

static struct miscdevice ftadrv_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "log_fta",
    .fops  = &ftadrv_fops,
};

/*!
 @brief ftadrv_init

 driver initialize

 @param        none

 @retval        0: success
 @retval       <0: failed
*/
static int __init ftadrv_init(void)
{
   int ret;

   struct proc_dir_entry *entry;

#if defined (T_QCI_IS3)
     if (qci_get_offline_charging() == 1)
	 return 0;
#endif

   work_queue = create_workqueue("ftadrv_queue");
   INIT_WORK(&write_work, ftadrv_write_work);
   INIT_WORK(&proc_work, ftadrv_proc_work);
   INIT_LIST_HEAD(&write_head);
   INIT_LIST_HEAD(&proc_head);

   ftadrv_smd_init();

   entry = proc_mkdir( "fta", NULL );
   if ( entry == NULL) {
      printk( KERN_ERR "create_proc_entry failed\n" );
      return -EBUSY;
   }
   ftadrv_acpu_init(entry);
   ftadrv_ccpu_init(entry);

   ret = misc_register(&ftadrv_dev);
   if (ret) {
      printk(KERN_ERR "ftadrv: %d = misc_register() error\n",ret);
      return ret;
   }

   printk( KERN_INFO "start ftadrv %d\n",ret);
   return 0;

}

/*!
 @brief ftadrv_exit

 driver terminate

 @param        none

 @retval        0: success
 @retval       <0: failed
*/
static void __exit ftadrv_exit(void)
{
    misc_deregister(&ftadrv_dev);
    fta_terminate();
}

module_init(ftadrv_init);
module_exit(ftadrv_exit);

/* ==========================================================================
 *  EXTERNAL FUNCTIONS
 * ========================================================================== */
/*!
 @brief ftadrv_exec_cmd

 execute the command for inclusion message

 @param  [in]  cmd     pointer of the commad string
 @param  [in]  cmdlen  length of the command string

 @retval       none
*/
void ftadrv_exec_cmd(const unsigned char *cmd,int cmdlen)
{

    struct readDataType *rd;
    unsigned long irqflags;

    if(!readDataMng.init) {
        return;
    }
    rd = kmalloc(sizeof(struct readDataType),GFP_KERNEL);
    if (!rd) {
        return;
    }
    rd->buf[0] = strlen(cmd);
    memcpy(&rd->buf[1],cmd,min_t(int,cmdlen, sizeof(rd->buf)-1));
    INIT_LIST_HEAD(&rd->list);
    spin_lock_irqsave(&read_lock, irqflags);
    list_add_tail(&rd->list,&readDataMng.head);
    spin_unlock_irqrestore(&read_lock, irqflags);

    wake_up_interruptible(&readDataMng.wq);

}
EXPORT_SYMBOL(ftadrv_exec_cmd);

/*!
 @brief ftadrv_send_str

 execute the command for inclusion message

 @param  [in]  cmd     pointer of the commad string
 @param  [in]  cmdlen  length of the command string

 @retval       none
*/
void ftadrv_send_str(const unsigned char *cmd)
{
    struct readDataType *rd;
    int cmdlen = strlen(cmd);
    unsigned long irqflags;

    if(!readDataMng.init) {
        return;
    }
    rd = kmalloc(sizeof(struct readDataType),GFP_KERNEL);
    if (!rd) {
        return;
    }
    rd->buf[0] = 0xb0;
    cmdlen = min_t(int,cmdlen, sizeof(rd->buf)-2);
    memcpy(&rd->buf[1],cmd,cmdlen);
    rd->buf[cmdlen+1] = 0;
    INIT_LIST_HEAD(&rd->list);
    spin_lock_irqsave(&read_lock, irqflags);
    list_add_tail(&rd->list,&readDataMng.head);
    spin_unlock_irqrestore(&read_lock, irqflags);

    wake_up_interruptible(&readDataMng.wq);

}
EXPORT_SYMBOL(ftadrv_send_str);

/*!
 @brief ftadrv_alloc

 allocate in the android log area

 @param  [in]  area    area for the android log
 @param  [in]  size    length of the area

 @retval       <>0:    pointer of the allocated area
 @retval         0:    failed
*/
unsigned char *ftadrv_alloc(int area,int size)
{
  int areasize[] = {  FTA_LOG_MAIN_SIZE,  /* log/main */
                      FTA_LOG_EVENT_SIZE, /* log/event */
                      FTA_LOG_RADIO_SIZE, /* log/radio */
                      FTA_LOG_SYSTEM_SIZE, /* log/system */
                      FTA_LOG_USER0_SIZE, /* log/user0 */
                      FTA_LOG_USER1_SIZE, /* log/user1 */
                   };

  if(area >= sizeof(areasize)/sizeof(int))
    return 0;
  size += areasize[area];
  return fta_var_alloc_log(area,size);
}
EXPORT_SYMBOL(ftadrv_alloc);

/*!
 @brief ftadrv_printk

 output printk message

 @param  [in]  srcname   pointer of the filename
 @param  [in]  line      line no
 @param  [in]  level     log level
 @param  [in]  time      pointer of the output time string
 @param  [in]  msg       pointer of the output message

 @retval       none

 @note         call by printk()
*/
int ftadrv_printk(const unsigned char *srcname, unsigned long line, unsigned long level,  const unsigned char *time,const unsigned char * msg)
{
  unsigned long info = (line << 16) | (0xfe00 | level);
  unsigned long msglen;
  if(ftadrv_cnt==0) {
    fta_initialize();
  }
  
  msglen = chrnul(msg,'\n',&info);
  fta_log_message_kmsg(srcname,info,time,msg,msglen+1);
  return (info & 0x80) ? msglen : msglen+1;
}
EXPORT_SYMBOL(ftadrv_printk);

/*!
 @brief ftadrv_get_time

 get output time

 @param  [in]  ptr   pointer of the time structure

 @retval       none
*/
void ftadrv_get_time(struct timespec *ptr)
{
   extern int timekeeping_suspended;
   static struct timespec tm;
   if(timekeeping_suspended) {
     goto end;
   }
   if(ftadrv_cnt < 200) {
     unsigned long long t;
     ftadrv_cnt++;
     t = cpu_clock(smp_processor_id());
     ptr->tv_nsec = do_div(t, 1000000000);
     ptr->tv_sec = t;
     return;
   }
   getnstimeofday(&tm);
end:
   *ptr = tm;
   return;
}
EXPORT_SYMBOL(ftadrv_get_time);


/*!
 @brief ftadrv_flush

 flush fta area

 @param        none

 @retval       none
*/
void ftadrv_flush(void)
{
}
EXPORT_SYMBOL(ftadrv_flush);

/*!
 @brief ftadrv_exec_write

 flush write function

 @param    buf  [in]    write buffer
 @param    size [in]    size of write buffer

 @retval   0> size
 @retval   0< error

*/
ssize_t ftadrv_exec_write(const char *buf,size_t size)
{
    int len;
    struct writeDataType *wd;

    wd = kmalloc(sizeof(struct writeDataType),GFP_KERNEL);
    if (!wd) {
       return -ENOSPC;
    }
    len = min_t(size_t, size, MAX_BUF_SIZE);
    memcpy(wd->buf,buf,len);
    return exec_write(wd,len);
}
EXPORT_SYMBOL(ftadrv_exec_write);

void ftadrv_free_proc(struct procDataType *pd)
{
    unsigned long irqflags;

    spin_lock_irqsave(&work_lock, irqflags);
    list_del(&pd->list);
    spin_unlock_irqrestore(&work_lock, irqflags);
    if(pd->ptr == 0)
       goto end;
    switch(pd->type) {
    case FTA_RA:
    case FTA_RC:
    case FTA_CC:
      vfree(pd->ptr);
      break;
    case FTA_CA:
      if(pd->ptr != (char *)fta_log_area) {
        vfree(pd->ptr);
      }
      break;
    default:
      break;
    }
    pd->ptr = 0;
end:
    kfree(pd);
}
EXPORT_SYMBOL(ftadrv_free_proc);

struct procDataType *ftadrv_prepare_proc(int type)
{
    unsigned long irqflags;
    struct procDataType *pd;
    unsigned short tid = current->pid;

    spin_lock_irqsave(&work_lock, irqflags);
    list_for_each_entry(pd,&proc_head,list) {
       printk(KERN_INFO "0x%p: %d %d\n",pd,pd->tid,pd->type);
       if (pd->tid == tid && pd->type == type) {
          printk(KERN_ERR "proc is busy %d %d\n",tid,type);
          spin_unlock_irqrestore(&work_lock, irqflags);
          return 0;
       }
    }
    spin_unlock_irqrestore(&work_lock, irqflags);

    pd = kmalloc(sizeof(struct procDataType),GFP_KERNEL);
    if (!pd) {
       printk(KERN_ERR "no memory\n");
       return 0;
    }

    init_completion(&pd->comp);
    pd->ptr = 0;
    pd->size = 0;
    pd->ret = 0;
    pd->type = type;
    pd->tid = tid;
    pd->state = 0;

    spin_lock_irqsave(&work_lock, irqflags);
    INIT_LIST_HEAD(&pd->list);
    list_add_tail(&pd->list,&proc_head);
    queue_work(work_queue, &proc_work);
    spin_unlock_irqrestore(&work_lock, irqflags);

    wait_for_completion(&pd->comp);
    if (pd->ret < 0) {
       ftadrv_free_proc(pd);
       return 0;
    }
    return pd;
}
EXPORT_SYMBOL(ftadrv_prepare_proc);

#include <linux/stacktrace.h>
#include <asm/stacktrace.h>

void ftadrv_stackdump(void)
{
    extern void print_cpu_info(void);
    struct task_struct *g, *t;
    int i = 0;
    printk(KERN_INFO "--stack dump start ------------------\n");
    rcu_read_lock();
    do_each_thread(g, t) {
       struct stack_trace trace;
       unsigned long entry[16];
       int n,m;
       if(t == current)
         continue;
       printk(KERN_INFO "%d pid:tid %d:%d name:%s cpu: %d state: %ld in:%lld out:%lld\n",i++,
           t->tgid,t->pid,t->comm,task_cpu(t),t->state,t->fta_in_timestamp,t->fta_out_timestamp);
       trace.nr_entries = 0;
       trace.max_entries = 16;
       trace.entries = entry;
       trace.skip = 0;
       save_stack_trace_tsk(t,&trace);
       n = min_t(int,16,trace.nr_entries);
       if(entry[n-1] == ULONG_MAX)
         n--;
       for(m=0;m<n;m++) {
          printk("[<%08lx>] %pS\n", entry[m], (void *)entry[m]);
       }
       if(n == 16) {
          printk("...more\n");
       }
    } while_each_thread(g, t);
    rcu_read_unlock();
    printk(KERN_INFO "--stack dump end --------------------\n");
    print_cpu_info();
}
EXPORT_SYMBOL(ftadrv_stackdump);

/*!
 @brief ftadrv_restart

 fta restart 

 @param        none

 @retval       none

 @note  call by machine_restart()
*/
void ftadrv_restart(void)
{
    ftadrv_stackdump();
    fta_stop(FTA_STOP_REASON_ABNORMAL);
    /* memory flush */
    clean_caches((unsigned int)fta_log_area,FTA_CONFIG_STORE_MEMORY_SIZE,virt_to_phys(fta_log_area));
}
EXPORT_SYMBOL(ftadrv_restart);

/*!
 @brief ftadrv_cpu

 get cpu id

 @param        none

 @retval       cpu id
*/
#include <linux/interrupt.h> /* in_interrupt() */
int ftadrv_cpu(void)
{
  return (smp_processor_id()+1) | (in_interrupt() ? 0x100 : 0);
}
EXPORT_SYMBOL(ftadrv_cpu);

MODULE_DESCRIPTION("FTA Driver");
MODULE_LICENSE("GPL v2");
