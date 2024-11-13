/*
**             Copyright 2023 by Kvaser AB, Molndal, Sweden
**                         http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ==============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
** BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
** IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
** POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ==============================================================================
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
**
**
** IMPORTANT NOTICE:
** ==============================================================================
** This source code is made available for free, as an open license, by Kvaser AB,
** for use with its applications. Kvaser AB does not accept any liability
** whatsoever for any third party patent or other immaterial property rights
** violations that may result from any usage of this source code, regardless of
** the combination of source code and various applications that it can be used
** in, or with.
**
** -----------------------------------------------------------------------------
*/
#ifndef HYDRA_PARAM_DEFS_H_
#define HYDRA_PARAM_DEFS_H_

// Parameter identifiers (one byte), see doc\Parameter_types.txt for more info
#define PARAM_USER_FIRST                      1
#define PARAM_USER_LAST                       100
#define PARAM_USERPASSWORD                    123
#define PARAM_SERIAL_NUMBER                   124
#define PARAM_MANUFACTURING_DATE              125
#define PARAM_LICENSE_MASK                    126
#define PARAM_HW_REVISION                     127
#define PARAM_HW_TYPE                         128
#define PARAM_EAN_NUMBER                      129
#define PARAM_NUMBER_CHANNELS                 130
#define PARAM_KVASER_LICENSE_MASK             131
#define PARAM_CHANNEL_A_TRANS_TYPE            132
#define PARAM_CHANNEL_B_TRANS_TYPE            133
#define PARAM_HW_DESCRIPTION_0                134
#define PARAM_HW_DESCRIPTION_1                135
#define PARAM_HW_DESCRIPTION_2                136
#define PARAM_CHANNEL_C_TRANS_TYPE            137
#define PARAM_CHANNEL_D_TRANS_TYPE            138
#define PARAM_USB_VENDOR_ID                   139
#define PARAM_USB_PRODUCT_ID                  140
#define PARAM_NIC_GROUP_BRANDED               141
#define PARAM_EAN_NUMBER_PRODUCT              143
#define PARAM_CHANNEL_E_TRANS_TYPE            144
#define PARAM_CHANNEL_F_TRANS_TYPE            145
#define PARAM_CHANNEL_G_TRANS_TYPE            146
#define PARAM_CHANNEL_H_TRANS_TYPE            147
#define PARAM_MAX_BITRATE                     148
#define PARAM_HW_FLASH_SIZE                   240
#define PARAM_FW_FREE_SIGNALS                 241
#define PARAM_FW_LOWWATER_SIGNALS             242
#define PARAM_HW_CHIP_TEMPERATURE             243

// Parameter length, in bytes.
#define PARAM_SERIAL_NUMBER_LEN               4
#define PARAM_MANUFACTURING_DATE_LEN          4
#define PARAM_LICENSE_MASK_LEN                4
#define PARAM_HW_REVISION_LEN                 1
#define PARAM_HW_TYPE_LEN                     1
#define PARAM_EAN_NUMBER_LEN                  8
#define PARAM_NUMBER_CHANNELS_LEN             1
#define PARAM_KVASER_LICENSE_MASK_LEN         4
#define PARAM_CHANNEL_TRANS_TYPE_LEN          1
#define PARAM_HW_DESCRIPTION_PART_LEN         8
#define PARAM_USB_VENDOR_ID_LEN               2
#define PARAM_USB_PRODUCT_ID_LEN              2
#define PARAM_NIC_GROUP_BRANDED_LEN           1
#define PARAM_EAN_NUMBER_PRODUCT_LEN          8
#define PARAM_MAX_BITRATE_LEN                 4

#define PARAM_HW_FLASH_SIZE_LEN               4
#define PARAM_FW_FREE_SIGNALS_LEN             4
#define PARAM_FW_LOWWATER_SIGNALS_LEN         4
#define PARAM_HW_CHIP_TEMPERATURE_LEN         4

#endif // HYDRA_PARAM_DEFS_H_
