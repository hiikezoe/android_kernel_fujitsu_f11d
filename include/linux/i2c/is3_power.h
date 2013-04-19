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
#ifndef __IS3_CHARGER_H__
#define __IS3_CHARGER_H__

struct smb329b_platform_data {
       unsigned charger_state_gpio;
	unsigned ovp_sw1_off_gpio;
	unsigned ovp_sw2_off_gpio;
	unsigned xcradle_state_gpio;
	unsigned usb_vbus_mpp;
};

void smb329b_vbus_draw(unsigned int mA);
int check_msm_temp_over_heat(void);

#endif
