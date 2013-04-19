/* Copyright (c) 2010 - 2011, Code Aurora Forum. All rights reserved.
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
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/
#ifndef _MACH_QDSP5_V2_AUDIO_ACDB_DEF_H
#define _MACH_QDSP5_V2_AUDIO_ACDB_DEF_H

/* Define ACDB device ID */
#define ACDB_ID_HANDSET_SPKR				1
#define ACDB_ID_HANDSET_MIC				2
#define ACDB_ID_HEADSET_MIC				3
#define ACDB_ID_HEADSET_SPKR_MONO			4
#define ACDB_ID_HEADSET_SPKR_STEREO			5
#define ACDB_ID_SPKR_PHONE_MIC				6
#define ACDB_ID_SPKR_PHONE_MONO				7
#define ACDB_ID_SPKR_PHONE_STEREO			8
#define ACDB_ID_BT_SCO_MIC				9
#define ACDB_ID_BT_SCO_SPKR				0x0A
#define ACDB_ID_BT_A2DP_SPKR				0x0B
#define ACDB_ID_BT_A2DP_TX				0x10
#define ACDB_ID_TTY_HEADSET_MIC				0x0C
#define ACDB_ID_TTY_HEADSET_SPKR			0x0D
#if defined(T_QCI_IS3)
#define ACDB_ID_SECONDARY_MIC				0x0E
#define ACDB_ID_CAMREC_MIC				0x0F
#endif
#define ACDB_ID_HEADSET_MONO_PLUS_SPKR_MONO_RX		0x11
#define ACDB_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX	0x14
#define ACDB_ID_FM_TX_LOOPBACK				0x17
#define ACDB_ID_FM_TX					0x18
#define ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX		0x19
#define ACDB_ID_LP_FM_HEADSET_SPKR_STEREO_RX		0x1A
#define ACDB_ID_I2S_RX					0x20
#define ACDB_ID_SPKR_PHONE_MIC_BROADSIDE		0x2B
#define ACDB_ID_HANDSET_MIC_BROADSIDE			0x2C
#define ACDB_ID_SPKR_PHONE_MIC_ENDFIRE			0x2D
#define ACDB_ID_HANDSET_MIC_ENDFIRE			0x2E
#define ACDB_ID_I2S_TX					0x30
#define ACDB_ID_HDMI					0x40
#define ACDB_ID_FM_RX					0x4F
/*Replace the max device ID,if any new device is added Specific to RTC only*/
/* FUJITSU:2011-12-01 bt(sco) ec off start */
#define ACDB_ID_BT_SCO_MIC_EC_OFF		0x50
#define ACDB_ID_BT_SCO_SPKR_EC_OFF		0x51
/* FUJITSU:2011-12-01 bt(sco) ec off end */
/* FUJITSU:2011-12-01 wm0010 start */
#define ACDB_ID_HANDSET_VOICE_SPKR			0x52
#define ACDB_ID_HANDSET_VOICE_MIC			0x53
#define ACDB_ID_SPKR_PHONE_VOICE_MONO		0x54
#define ACDB_ID_SPKR_PHONE_VOICE_MIC		0x55
#define ACDB_ID_HEADSET_VOICE_SPKR_STEREO	0x56
#define ACDB_ID_HEADSET_VOICE_MIC			0x57
/* FUJITSU:2011-12-01 wm0010 end */
/* FUJITSU:2011-12-01 vr_mode start */
#define ACDB_ID_SPKR_PHONE_MIC_FOR_VREC		0x58
/* FUJITSU:2011-12-01 vr_mode end */
/* FUJITSU:2011-12-01 start */
#define ACDB_ID_GUIDANCE_RX					0x59
#define ACDB_ID_GUIDANCE_TX					0x60
#define ACDB_ID_SLEEP_SUPPORT_TX			0x61
/* FUJITSU:2011-12-01 end */
/* FUJITSU:2011-12-01 camera sound start */
#define ACDB_ID_CAMERA_RX					0x62
/* FUJITSU:2011-12-01 camera sound end */

#define ACDB_ID_BACKMIC						0x63 /* FUJITSU:2012-04-23 back mic */

#define ACDB_ID_MAX                                 ACDB_ID_BACKMIC /* FUJITSU:2011-12-01 */ /* FUJITSU:2012-04-23 back mic */

/* ID used for virtual devices */
#define PSEUDO_ACDB_ID 					0xFFFF

#endif /* _MACH_QDSP5_V2_AUDIO_ACDB_DEF_H */
