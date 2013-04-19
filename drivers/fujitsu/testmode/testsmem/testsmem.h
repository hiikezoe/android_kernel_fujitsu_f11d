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

#ifndef TESTSMEM_H
#define TESTSMEM_H
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mempool.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <mach/msm_smd.h>
#include <asm/atomic.h>
#define USB_MAX_OUT_BUF 4096
#define USB_MAX_IN_BUF  8192
#define HDLC_MAX 4096
#define HDLC_OUT_BUF_SIZE 8192
#define POOL_TYPE_COPY 1
#define POOL_TYPE_HDLC 0
#define POOL_TYPE_USB_STRUCT 2
#define MAX_DIAG_USB_REQUESTS 12
#define MSG_MASK_SIZE 8000
#define LOG_MASK_SIZE 1000
#define EVENT_MASK_SIZE 1000
#define REG_TABLE_SIZE 25
#define PKT_SIZE 4096
#define TRUE  1
#define FALSE 0
#define MA_SIO_BUFF_SIZE 1024
#define MA_CMD_BUFF_SIZE 256
#define MA_DISP_HIGHT14 14
#define MA_DISP_WIDTH15 15
#define MA_DISP_HIGHT28 28
#define MA_DISP_WIDTH30 30
#define TEST_REQUEST 0x00000001
#define TEST_RESP_END 0x00000002
#define TEST_RESP_CONT 0x00000004
#define TEST_RESP_REQ 0x00000008
#define TEST_RESP_WAIT 0x00000010
#define TEST_FORCE_END 0x00000020
#define SD_CMD_WAIT 0x00000001
#define SD_DISP_UPDATE_REQ 0x00000002
#define TS_SUCCESSFUL_RESULT 0x01
#define TS_CMD_179_COORDINATES_DATA_SIZE 6

typedef unsigned char byte;
typedef unsigned char boolean;

struct diag_master_table {
    uint16_t cmd_code;
    uint16_t subsys_id;
    uint16_t cmd_code_lo;
    uint16_t cmd_code_hi;
    int process_id;
};

struct testsmem_dev {
    unsigned int major;
    unsigned int minor_start;
    int num;
    struct cdev *cdev;
    char *name;
    struct class *testsmem_class;
    unsigned long  status;
    struct mutex testsmem_mutex;
};

typedef enum {
    MA_IF_DISP_TYPE_14x15,
    MA_IF_DISP_TYPE_28x30,
    MA_IF_DISP_TYPE_PAD = 0x7FFFFFFF
} ma_if_disp_format_type;

enum{
    ELECTRIC_PAD_COORDINATES = 0,
    ELECTRIC_PAD_READ,
    ELECTRIC_PAD_WRITE,
    ELECTRIC_PAD_COORDINATES_ONCE,
    ELECTRIC_PAD_EEPROM_WRITE,
    ELECTRIC_PAD_COORDINATES_WAIT,
    ELECTRIC_PAD_DOT_PLOT = 0x0F
};

typedef struct {
    unsigned long                valid;
    unsigned char                data[MA_DISP_WIDTH30];
} ma_if_disp_line_type;

typedef struct {
    ma_if_disp_format_type       format;
    unsigned long                valid;
    ma_if_disp_line_type         line[MA_DISP_HIGHT28];
} ma_if_disp_type;

typedef struct {
    unsigned long                size;
    unsigned char                data[MA_SIO_BUFF_SIZE];
} ma_if_sio_type;

typedef struct {
    unsigned long                cmd;
    unsigned char                command[MA_CMD_BUFF_SIZE];
} ma_if_command_type;

typedef struct{
    unsigned long                start_addr;
    unsigned long                end_addr;
    boolean                      random_data_flg;
    unsigned long                write_data1;
    unsigned long                write_data2;
    unsigned long                set_roop;
} ma_if_sdram_check_param_type;

typedef struct{
    byte                         err_flg;
    unsigned long                err_addr1;
    unsigned long                err_addr2;
    unsigned long                err_read_data1;
    unsigned long                err_read_data2;
    unsigned long                err_roop_cnt;
} ma_if_sdram_check_res_type;

typedef struct{
    byte                         test_data[32];
    byte*                        data_buff;
    byte                         result;
    unsigned int                 test_kind;
    unsigned long                start_address;
    unsigned long                data_size;

} ma_if_elec_pad_param_type;

typedef struct{
    byte                         led;
    byte                         result;
    byte                         dumy1;
    byte                         dumy2;
    byte                         dumy3;
    byte                         dumy4;
} ma_if_elec_pad_bl_led_param;

typedef struct{
    ma_if_sdram_check_param_type ts_144_sdram_chk_param;
    ma_if_sdram_check_res_type   ts_144_sdram_res_param;
    ma_if_elec_pad_param_type    ts_179_elec_pad_chk_param;
    ma_if_elec_pad_bl_led_param  ts_243_elec_pad_bl_led_param;
}ma_if_req_cmd_param;

typedef struct {
    boolean                      req_flg;
    unsigned long                req_cmd_no;
    unsigned long                which_req;
    ma_if_req_cmd_param          req_param;
} ma_if_ask_type;

typedef struct {
    ma_if_command_type           *cmd;
    ma_if_disp_type              *disp_ptr;
    ma_if_sio_type               *sio_ptr;
    unsigned long                *status;
    ma_if_ask_type               *req_cmd;
} ma_if_cmd_header_type;

typedef struct {
    ma_if_cmd_header_type        CmdHdr;
    ma_if_disp_type              DisplayBuf;
    ma_if_sio_type               SioBuf;
    ma_if_command_type           CmdBuf;
    unsigned long                Status;
    ma_if_ask_type               Req_cmd;
} test_maif_type;

#endif
