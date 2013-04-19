/*
  fcsmd Driver

  Copyright(C) 2011-2012 FUJITSU LIMITED
 
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
*/

/* FUJITSU:2011-08-05 add start */
#include <linux/ioctl.h>

typedef struct {
	unsigned int smem_size;
	char        *buff;
} fcsmd_ioctl_cmd;

#define FCSMD_IOCTL_MAGIC 'f'

#define FCSMD_IOCTL_01   _IOR( FCSMD_IOCTL_MAGIC, 1, fcsmd_ioctl_cmd )
/* FUJITSU:2011-08-05 add end */
