/*
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
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2012
/*----------------------------------------------------------------------------*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cryptohash.h>
#include <linux/usb_id.h>
#include "../arch/arm/mach-msm/include/mach/msm_smsm.h"
#include "../arch/arm/mach-msm/proc_comm.h"

/*-----*/
MODULE_DESCRIPTION("Fujitsu Debug USB Driver");
MODULE_LICENSE("GPL");

/*-----*/
static int g_certify = 0;
static int g_checked = 0;

/*-----*/
extern int factory_mode(void);

/*-----*/
#ifdef ENABLE_NVREAD_IF
void debugusb_set_pid_to_nv(unsigned long pid);
#endif /* ENABLE_NVREAD_IF */
static int debugusb_set_password(const char *val, struct kernel_param *kp);
static int debugusb_get_password(char *buffer, struct kernel_param *kp);
static int debugusb_get_boot_pid(char *buffer, struct kernel_param *kp);

#define MAX_STRING_LEN  65
static char password_string[MAX_STRING_LEN] = {0};
#ifdef ENABLE_PWSTRING_IF
static unsigned int desired_password[5] = {0x2543405f,0x8bd90dc7,0x5de9ec18,0x9f8102c1,0x8d8e8efd};
#endif // ENABLE_PWSTRING_IF
static unsigned int output_password[5] = {0}; 
static struct kparam_string kps_pass = {
	.maxlen = MAX_STRING_LEN,
	.string = password_string
};

module_param_call(password_string, debugusb_set_password, debugusb_get_password, &kps_pass, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);	//0644
MODULE_PARM_DESC(password_string, "USB debug password string");

static unsigned long boot_pid = 0;
module_param_call(boot_pid, NULL, debugusb_get_boot_pid, &boot_pid, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);	//0644
MODULE_PARM_DESC(boot_pid, "USB debug boot pid");

#ifdef ENABLE_NVREAD_IF
#if 0
static int debugusb_read_nvitem(unsigned int id, unsigned int *data)
{
	int rc;
	if (data == NULL)
		return -1;

	rc = msm_proc_comm(PCOM_OEM_011, &id, (unsigned *)data);
	if (rc)
		pr_info("%s : NV read failed. %d\n", __func__, rc);

	return rc;
}
#endif
static int debugusb_write_nvitem(unsigned int id, unsigned int *data)
{
	int rc;
	if (data == NULL)
		return -1;

	rc = msm_proc_comm(PCOM_OEM_012, &id, (unsigned *)data);
	if (rc)
		pr_info("%s : NV write failed. %d\n", __func__, rc);

	return rc;
}

#define NV_OPEFLAG_I		10035
#define NV_PID_DEBUG_ADB	0x02
#if 0
static int is_debug_port(void)
{
	unsigned int nv_ope	= 0;
	if (debugusb_read_nvitem(NV_OPEFLAG_I, &nv_ope) != 0)
		return 0;

	printk("[fct] NV_OPEFLAG_I=%d\n", nv_ope);
	return ((nv_ope == NV_PID_DEBUG_ADB) ? 1 : 0);
}
//#endif //!(T_QCI_IS3)
#endif

void debugusb_set_pid_to_nv(unsigned long pid)
{
	static unsigned long prev_pid = 0;
	static int prev_nv_ope = 0;
	unsigned int nv_ope	= 0;

	if (prev_pid == pid)
		goto out;

	switch (pid) {
	case USB_ID_DEBUG_ADB:
		nv_ope = NV_PID_DEBUG_ADB;
		break;
	default:
		nv_ope = 0;
		break;
	}

	if (prev_nv_ope != nv_ope) {
		pr_info("%s : update nv val=%d\n", __func__, nv_ope);
		if (debugusb_write_nvitem(NV_OPEFLAG_I, &nv_ope) != 0)
			goto out;
	}

	prev_nv_ope = nv_ope;
	prev_pid = pid;
out:
	pr_info("%s : pid=%lx nv_ope=%d\n", __func__, pid, nv_ope);
}
EXPORT_SYMBOL(debugusb_set_pid_to_nv);
#endif /* ENABLE_NVREAD_IF */

int debugusb_allow_portchange(unsigned long pid)
{
	if (g_checked)
		goto leave;

	if (boot_pid == USB_ID_LOGMOOD ||
		boot_pid == USB_ID_FACTORY)
		g_checked = (boot_pid == pid);

leave:
	return g_checked;
}
EXPORT_SYMBOL(debugusb_allow_portchange);

static void debugusb_hash_password(const char *kmessage)
{
	unsigned int hash[5], workspace[SHA_WORKSPACE_WORDS];
	char local_buf[MAX_STRING_LEN] = {0};
	int i =0;

	strlcpy(local_buf, kmessage, MAX_STRING_LEN);
	local_buf[64] = '\0';
	sha_init(hash);
	sha_transform(hash, (unsigned char *)local_buf, workspace);
	for (i = 0; i < 5; i++) {
		output_password[i]= hash[i];
	}
}

static int debugusb_set_password(const char *kmessage, struct kernel_param *kp)
{
	int len = 0;
	int i = 0;
#ifdef ENABLE_PWSTRING_IF
	unsigned int buffer [5] = {0};
	int loop = 0;
#endif /* ENABLE_PWSTRING_IF */
	
	if (kmessage) {
		len = strlen(kmessage);
	}
	else {
		for (i=0; i<5; i++) {
			output_password[i] = 0;
		}
		return 0;
	}
	
	if (len > 64) {
		return -ENOSPC;
	}

	debugusb_hash_password(kmessage);
#ifdef ENABLE_PWSTRING_IF
	debugusb_get_password((char *)buffer, kp);
	for (loop =0; loop<5; loop ++) {
		if( buffer[loop] != desired_password[loop] ) {
			return -EINVAL;
		}
	}
	g_certify = 1;
#endif /* ENABLE_PWSTRING_IF */
	
	return 0;
}

static int debugusb_get_password(char *buffer, struct kernel_param *kp)
{
	int ret = 0;
	int i = 0;
	unsigned int *sv;

	sv = (unsigned int *)buffer;
	for (i = 0 ; i < 5 ; i++) {
		sv[i] = output_password[i];
		output_password[i] = 0;
	}
	return ret;
}

static int debugusb_get_boot_pid(char *buffer, struct kernel_param *kp)
{
	pr_info("%s : %lx\n", __func__, boot_pid);
	return sprintf(buffer, "%lx", boot_pid);
}

static int __init debugusb_init(void)
{
	unsigned char *smemprm = NULL;
	
	g_certify = 0;
	boot_pid = 0;
	
	smemprm = (unsigned char *)smem_alloc_vendor1(SMEM_OEM_016);
	printk("%s :*smemprm = %d\n", __func__, *smemprm);
	if( smemprm )
	{
	if (*smemprm == 1) {
		printk( "[fct] ven1 SMEM_OEM_016\n");
		boot_pid = USB_ID_LOGMOOD;
		goto leave;
	}
	}

	if (factory_mode()) {
		printk("%s : In factory_mode\n", __func__);
		boot_pid = USB_ID_FACTORY;
		goto leave;
	}

#ifdef ENABLE_NVREAD_IF
	#if !defined (T_QCI_IS3)
	if (is_debug_port()) {
		boot_pid = USB_ID_DEBUG_ADB;
		goto leave;
	}
	#else
	boot_pid = USB_ID_DEBUG_ADB;
	#endif //!(T_QCI_IS3)
#endif /* ENABLE_NVREAD_IF */
leave:
	//if (!boot_pid || boot_pid == USB_ID_DEBUG_ADB)
		g_checked = 1;

	printk("%s : boot_pid = %lx\n", __func__, boot_pid);
	return 0;
}
module_init(debugusb_init);

static void __exit debugusb_deinit(void)
{
	return;
}
module_exit(debugusb_deinit);
