#include <../../../../../kernel/arch/arm/mach-msm/proc_comm.h>
#include <linux/kernel.h>

extern int read_macaddr(unsigned char *addr_buf)
{
	unsigned nvid = 4678;	/* NV_WLAN_MAC_ADDRESS_I */
	unsigned data;
	int       ret;
	
	ret = msm_proc_comm(PCOM_OEM_011, &nvid, &data); 
	if (ret == 0)
	{
		/* Success */
		addr_buf[0] = data		& 0xFF;
		addr_buf[1] = (data>>8) & 0xFF;
		addr_buf[2] = (data>>16)& 0xFF;
		addr_buf[3] = (data>>24)& 0xFF;
		addr_buf[4] = nvid		& 0xFF;
		addr_buf[5] = (nvid>>8) & 0xFF;
	}
	else
	{
		/* Fail */
		printk("%s: msm_proc_comm err ret=%d\n", __FUNCTION__, ret);
	}

	printk(KERN_INFO "read_macaddr -> read NV mac = %02x:%02x:%02x:%02x:%02x:%02x, data=%u\n", 
		         addr_buf[0], addr_buf[1], addr_buf[2], addr_buf[3], addr_buf[4], addr_buf[5], data);

	//To avoid "00:00:00:00:00:00" case
	if ( addr_buf[0] == 0 && addr_buf[1] == 0 && addr_buf[2] == 0 && 
	     addr_buf[3] == 0 && addr_buf[4] == 0 && addr_buf[5] == 0) 
	    ret = -5;


	return ret;
}

