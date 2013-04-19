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

#ifndef __LINUX_USBID_H
#define __LINUX_USBID_H

#define USB_ID_FACTORY			0x1111
/* FUJITSU:2011-10-18 LOGMOOD start */
#define USB_ID_LOGMOOD			0x1112
/* FUJITSU:2011-10-18 LOGMOOD end */

#ifdef CONFIG_MACH_F11SKY
#define ENABLE_NVREAD_IF
//#define ENABLE_PWSTRING_IF
#define USB_ID_MSC 				0x12e8
#define USB_ID_MSC_ADB	 		0x12e8
#define USB_ID_RNDIS 			0x12e9
#define USB_ID_RNDIS_ADB		0x12e9
#define USB_ID_DEBUG			0x12ea
#define USB_ID_DEBUG_ADB		0x12ea
#define USB_ID_RNDIS_DEBUG		0x12eb
#define USB_ID_RNDIS_DEBUG_ADB	0x12eb
#define USB_ID_ALADN			0x12ed
#endif /* CONFIG_MACH_F11SKY */

#ifdef CONFIG_MACH_F11APO
#define ENABLE_NVREAD_IF
//#define ENABLE_PWSTRING_IF
#define USB_ID_MSC 				0x12f3
#define USB_ID_MSC_ADB	 		0x12f3
#define USB_ID_RNDIS 			0x12f4
#define USB_ID_RNDIS_ADB		0x12f4
#define USB_ID_DEBUG			0x12f5
#define USB_ID_DEBUG_ADB		0x12f5
#define USB_ID_RNDIS_DEBUG		0x12f6
#define USB_ID_RNDIS_DEBUG_ADB	0x12f6
#define USB_ID_ALADN			0x12f8
#endif /* CONFIG_MACH_F11APO */

/* FUJITSU:2011-12-15 F12APON start */
#ifdef CONFIG_MACH_F12APON
#define ENABLE_NVREAD_IF
//#define ENABLE_PWSTRING_IF
#define USB_ID_MTP				0x135c
#define USB_ID_MTP_ADB			0x135c
#define USB_ID_PTP				0x135d
#define USB_ID_PTP_ADB			0x135d
#define USB_ID_MSC 				0x12f3
#define USB_ID_MSC_ADB	 		0x12f3
#define USB_ID_RNDIS 			0x12f4
#define USB_ID_RNDIS_ADB		0x12f4
#define USB_ID_DEBUG			0x12f5
#define USB_ID_DEBUG_ADB		0x12f5
#define USB_ID_ALADN			0x12f8
#endif /* CONFIG_MACH_F12APON */
/* FUJITSU:2011-12-15 F12APON end */

/* FUJITSU:2012-03-06 F09D start */
#ifdef CONFIG_MACH_F09D
//#define ENABLE_NVREAD_IF
//#define ENABLE_PWSTRING_IF
#define USB_ID_MTP				0x1342
#define USB_ID_MTP_ADB			0x1342
#define USB_ID_PTP				0x1343
#define USB_ID_PTP_ADB			0x1343
#define USB_ID_RNDIS 			0x1344
#define USB_ID_RNDIS_ADB		0x1344
#define USB_ID_ALADN			0x1345
#define USB_ID_DEBUG			0x1346
#define USB_ID_DEBUG_ADB		0x1346
#endif /* CONFIG_MACH_F09D */
/* FUJITSU:2012-03-06 F09D end */

#ifdef CONFIG_MACH_FJI12
#define ENABLE_NVREAD_IF
//#define ENABLE_PWSTRING_IF
#define USB_ID_MSC 				0x12f9
#define USB_ID_MSC_ADB	 		0x12f9
#define USB_ID_RNDIS 			0x12fa
#define USB_ID_RNDIS_ADB		0x12fa
#define USB_ID_DEBUG			0x12fb
#define USB_ID_DEBUG_ADB		0x12fb
#define USB_ID_RNDIS_DEBUG		0x12fc
#define USB_ID_RNDIS_DEBUG_ADB	0x12fc
#define USB_ID_ALADN			0x12f8
#endif /* CONFIG_MACH_FJI12 */

#ifdef CONFIG_MACH_IS3JP
#define ENABLE_NVREAD_IF
//#define ENABLE_PWSTRING_IF
#define USB_ID_MTP				0x135E
#define USB_ID_MTP_ADB			0x135E
#define USB_ID_PTP				0x135F
#define USB_ID_PTP_ADB			0x135F
#define USB_ID_RNDIS 			0x1360
#define USB_ID_RNDIS_ADB		0x1360
#define USB_ID_ALADN			0x1361
#define USB_ID_DEBUG			0x1362
#define USB_ID_DEBUG_ADB		0x1362
#endif

#endif /* __LINUX_USBID_H */
