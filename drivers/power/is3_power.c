/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2012
/*----------------------------------------------------------------------------*/
/*
 * Battery/Charger driver for Qualcomm MSM chipsets and 
 * MAX17040_G+T and SUMB329B on IS3 Devices.
 *
 * is3_power.c
 * linux/i2c/is3_power.h
 * linux/i2c/is3_max17040gt.h
 * linux/i2c/is3_smb329b.h
 *
 *
 */
#define DRIVER_VER	"[20120717-01]"

#define DEBUG  0
#define DEBUG_MAX17040_GT	0
#define DEBUG_SMB329B		0
#define DEBUG_BATT_STATUS	0

#include <linux/slab.h>
#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <asm/atomic.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_battery.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/leds-pmic8058.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/input.h>

#include <linux/i2c/is3_power.h>
#include <linux/i2c/is3_max17040gt.h>
#include <linux/i2c/is3_smb329b.h>

#include "../../../arch/arm/mach-msm/proc_comm.h"

inline int qci_get_offline_charging(void);
inline int qci_get_recovery_charging(void);
extern void overheat_set_brightness (void);
extern void cooldown_set_brightness(void);

enum debug_show_type {
	TYPE_MAX17040,
	TYPE_SMB329B
};

#if DEBUG
    static int debug_level = 1;
#else
    #if (DEBUG_MAX17040_GT && DEBUG_SMB329B)
	static int debug_level = 1;
    #elif DEBUG_MAX17040_GT
	static int debug_level = 2;
    #elif DEBUG_SMB329B
	static int debug_level = 3;
    #else
	static int debug_level = 0;
    #endif
#endif

#define DBG_LIMIT(dl,t,x...) do {\
	                                          if ((dl==1 || dl==2) && (t==TYPE_MAX17040) ) {\
                                                 pr_debug(x);}\
						       if ((dl==1 || dl==3) && (t==TYPE_SMB329B) ) {\
                                                 pr_debug(x);}\
						    } while (0)

#if DEBUG_BATT_STATUS
#define DBG_BUILD(x...) do {pr_debug(x); } while (0)
#else
#define DBG_BUILD(x...) do {} while (0)
#endif

static struct workqueue_struct *batt_chg_work_queue = NULL;

#define MAX17040GT_DELAY_NORMAL		msecs_to_jiffies(30000)
#define MAX17040GT_DELAY_QUICK		msecs_to_jiffies(10000)
#define MAX17040GT_DELAY_RE_TRY		msecs_to_jiffies(1000)
#define MAX17040GT_DELAY_RESUME		msecs_to_jiffies(200)
#define MAX17040GT_DELAY_TEMP_MSM_OVERHEAT		msecs_to_jiffies(5000)
#define MAX17040GT_RESET_WAIT_MS		200
#define MAX17040GT_ENABLE_UPDATE_RCOMP		1

#define SMB329B_TEMP_DELAY	msecs_to_jiffies(5000)
#define SMB329B_WAIT_AP_DELAY		msecs_to_jiffies(5000)
#define SMB329B_OFFLINE_CHARGING_DELAY	msecs_to_jiffies(1000)
#define SMB329B_LED_BLINK_DELAY	msecs_to_jiffies(1500)
#define SMB320B_WAIT_OVP_SWITCH_READY_MS	200
#define SMB329B_FLOAT_VOLTAGE_4360	0x5A
#define SMB329B_FLOAT_VOLTAGE_4100	0x40
#define SMB329B_FLOAT_VOLTAGE_4200	0x4A
#define SMB329B_COMMAND_A_REG_DEFAULT	0xA0
#define SMB329B_PIN_CTRL_REG_CHG_ON		0x01
#define SMB329B_PIN_CTRL_REG_CHG_OFF		0x04
#define SMB320B_CHG_CURRENT_DEFAULT		0x1B
#define SMB320B_CHG_CURRENT_USB			0x1B
#define SMB320B_CHG_CURRENT_AC			0xFB
#define SMB320B_CHG_CURRENT_0_5_C		0x7B
#define SMB320B_ENABLE_AICL				0xF0
#define SMB320B_ENABLE_RE_CHARGE1		0x58	//SW re-charge, disable termianl current
#define SMB320B_ENABLE_RE_CHARGE2		0x98	//SW re-charge
#define SMB320B_ENABLE_RE_CHARGE3		0x18	//HW re-charge
#define SMB320B_RE_CHARGE_70_MV			0x3C
#define SMB320B_RE_CHARGE_140_MV			0x7C
#define RE_CHARGE_BATT_CAPACITY_NORMAL	99
#define RE_CHARGE_BATT_CAPACITY_CAMERA_CORDING		96
#define STOP_CHG_BATT_CAPACITY_ISDB_T		85
#define RE_CHARGE_BATT_CAPACITY_ISDB_T		80
#define SW_CHECK_OVC_VOLTAGE_4350	4400
#define SW_CHECK_OVC_VOLTAGE_4200	4250
#define SW_CHECK_CHG_TIMEOUT_RETRY_MIN	90

#define READ_RPC_MODE 	1
#define SW_CHECK_BATT_OVLO	1

#define CHG_ST_NTFY_CODE        0xE0    /* battery abnormal charge event */
#define HS_KEY_CHG_ST_NONE      0x155
#define HS_KEY_CHG_ST_OVP       0x156
#define HS_KEY_CHG_ST_OVC       0x157
#define HS_KEY_CHG_ST_OVD       0x158
#define HS_KEY_CHG_ST_EXP       0x159

enum battery_abnormal_events {
        K_CHG_ST_NONE = 0,
        K_CHG_ST_OVP,
        K_CHG_ST_OVC,
        K_CHG_ST_OVD,
        K_CHG_ST_EXP,
        K_TEMP_MSM_OVER_LAYER1,
        K_TEMP_MSM_OVER_LAYER2,
        K_TEMP_MSM_OVER_LAYER3,
        K_TEMP_MSM_OVER_RESUME_NORMAL
};

enum chg_flow_status {
	CHG_FLOW_STATUS_A,
	CHG_FLOW_STATUS_B,
	CHG_FLOW_STATUS_C,
	CHG_FLOW_STATUS_D,
	CHG_FLOW_STATUS_E,
	CHG_FLOW_STATUS_F,
	CHG_FLOW_STATUS_G,
	CHG_FLOW_STATUS_H,
	CHG_FLOW_STATUS_I,
	CHG_FLOW_STATUS_J,
	CHG_FLOW_STATUS_K,
	CHG_FLOW_STATUS_L,
	CHG_FLOW_STATUS_M,
	CHG_FLOW_STATUS_N,
	CHG_FLOW_STATUS_O,
	CHG_FLOW_STATUS_COUNT,
};

static char *chg_flow_status_text[] = {
              "A : without battery, charge stop",
              "B : battery existence, charge stop",
              "C : Preliminary charge timer finished, charge abnormality",
              "D : battery over voltage detected, charge abnormality",
              "E : preliminary charge(Pre-charge) 100mA",
              "F : Waiting charging for out of temperature range, power path operation",
              "G : Always check, Low input voltage detected, 4.0 or less",
              "H : Always check, Over input voltage detected, 6.4 or more",
              "I : Charging timer is finished, charge stop",
              "J : Adaptor ability judgement, 100mA => 1500mA",
              "K : high temperature range/low temperature range, Max 0.5C charging",
              "L : high speed charge (CC charge)",
              "M : high speed charge (CV charge), charging voltage less than 4.25V",
              "N : high speed charge",
              "O : full charge, charge finished",
		"Unknown",
};

static struct max17040gt_chip *max17040gt_chip;
static struct smb329b_chip *smb329b_chip;

enum chip_status {
	CHIP_STATUS_NONE,
	CHIP_STATUS_INIT_OK,
	CHIP_STATUS_INIT_FAIL,
};
static unsigned long max17040gt_delay_time = 3000;
static int max17040gt_ready = CHIP_STATUS_NONE;
static int smb329b_ready = CHIP_STATUS_NONE;
static int input_mA = 0;
static int vbus_connect = 0;
static int vbus_type = -1;
static int cradle_connect = 0;
static int verify_model_ready = 0;
static int wlock_en = 0;
static int thermal_lock_en = 0;
static int temp_work_en = 0;
static int temp_work_on = 0;
static int wait_exit_error_work_en = 0;
static int wait_exit_error_work_on = 0;
static int wait_exit_error_status = 0;
#if 1
   static int SMB320B_STOP_CHARGING_BATT_TEMP_DOWN = 0;
   static int SMB320B_STOP_CHARGING_BATT_TEMP_UP = 55;
   static int SMB320B_NO_START_CHARGING_BATT_TEMP_DOWN = 0;
   static int SMB320B_NO_START_CHARGING_BATT_TEMP_UP = 45;
   static int SMB320B_RESUME_CHARGING_BATT_TEMP_DOWN = 10;
   static int SMB320B_RESUME_CHARGING_BATT_TEMP_UP	= 45;
#else
   static int SMB320B_STOP_CHARGING_BATT_TEMP_DOWN = -3;
   static int SMB320B_STOP_CHARGING_BATT_TEMP_UP = 48;
   static int SMB320B_NO_START_CHARGING_BATT_TEMP_DOWN = -3;
   static int SMB320B_NO_START_CHARGING_BATT_TEMP_UP = 48;
   static int SMB320B_RESUME_CHARGING_BATT_TEMP_DOWN = 0;
   static int SMB320B_RESUME_CHARGING_BATT_TEMP_UP	= 45;
#endif
static int cradle_mode = 1;
static int change_max_current = 0;
static int debug_temp_value = -40;
static int debug_capacity_value = 0;
static int resume_temp_min = 0;
static int resume_temp_max = 0;
static int get_batt_first_info = 0;
static int rpc_ready = 0;
static int lock_batt_status = 0;
static unsigned long time_suspend;
static unsigned long time_resume;
#if USE_CHARGING_4350MV_BATTERY_PACK
static int float_voltage_mode = 1;
static int vbatt_max_voltage_mode = 0;
static int SMB329B_FLOAT_VOLTAGE_DEFAULT = SMB329B_FLOAT_VOLTAGE_4360;
static int SW_CHECK_OVC_VOLTAGE = SW_CHECK_OVC_VOLTAGE_4350;
static int SW_CHACK_OVC_RESUME_VOLTAGE = 4350;
#else
static int float_voltage_mode = 1;
static int vbatt_max_voltage_mode = 0;
static int SMB329B_FLOAT_VOLTAGE_DEFAULT = SMB329B_FLOAT_VOLTAGE_4200;
static int SW_CHECK_OVC_VOLTAGE = SW_CHECK_OVC_VOLTAGE_4200;
static int SW_CHACK_OVC_RESUME_VOLTAGE = 4200;
#endif
static u8 SMB320B_ENABLE_RE_CHARGE = SMB320B_ENABLE_RE_CHARGE3;
static u8 SMB320B_RE_CHARGE_VOLTAGE = SMB320B_RE_CHARGE_140_MV;
static int battery_info_mode = 1;
static int camera_recording = 0;
static int data_transmit = 0;
static int RE_CHARGE_BATT_CAPACITY = RE_CHARGE_BATT_CAPACITY_NORMAL;
static int batt_work_on = 0;
static int batt_work_disable = 0;
static int temp_msm_work_disable = 0;
static int temp_pa_work_disable = 0;
static int led_blink_work_on = 0;
static int debug_ovc = 0;
static int re_load_custom_model = 1;
static int power_source_switch_wait = 0;
static int isdb_t_on = 0;
static int isdb_t_stop_chg = 0;
static int send_keycode_work_disable = 0;

struct delayed_work thermal_msm_work;
struct delayed_work thermal_msm_work2;
static int thermal_msm_work_on = 0;
static int thermal_msm_set_brightness = 0;
static int thermal_msm_send_layer3_keycode = 0;
#define THERMAL_MSM_OVERHEAT_WAIT_TIME_3	msecs_to_jiffies(100)
#define THERMAL_MSM_OVERHEAT_WAIT_TIME_0_1_2	msecs_to_jiffies(3000)
#define PM_TEMP_MSM_OVER_HEAT_LAYER1_UP	45
#define PM_TEMP_MSM_OVER_HEAT_LAYER1_DOWN	40
#define PM_TEMP_MSM_OVER_HEAT_LAYER2_UP	48
#define PM_TEMP_MSM_OVER_HEAT_LAYER2_DOWN	44
#define PM_TEMP_MSM_OVER_HEAT_LAYER3	60

static struct mutex det_lock;
static struct mutex usb_type_lock;

//-----------------------------------------------------------
// BATTERY RPC
//-----------------------------------------------------------
#define BATTERY_RPC_PROG		0x30000089
#define BATTERY_RPC_VER_1_1	0x00010001
#define BATTERY_RPC_VER_2_1	0x00020001
#define BATTERY_RPC_VER_4_1     0x00040001
#define BATTERY_RPC_VER_5_1     0x00050001

#define BATTERY_RPC_CB_PROG	(BATTERY_RPC_PROG | 0x01000000)

#define CHG_RPC_PROG		0x3000001a
#define CHG_RPC_VER_1_1	0x00010001
#define CHG_RPC_VER_1_3	0x00010003
#define CHG_RPC_VER_2_2	0x00020002
#define CHG_RPC_VER_3_1       0x00030001
#define CHG_RPC_VER_4_1       0x00040001

#define BATTERY_REGISTER_PROC					2
#define BATTERY_MODIFY_CLIENT_PROC			4
#define BATTERY_DEREGISTER_CLIENT_PROC		5
#define BATTERY_READ_MV_PROC					12
#define BATTERY_ENABLE_DISABLE_FILTER_PROC	14
#define BATTERY_READ_MV_TEMP_PROC 			17
#define BATTERY_READ_CONN_PROC 				18
#define BATTERY_READ_PM_TEMP_PROC			19
#define BATTERY_READ_TEMP_PROC				20
#define BATTERY_READ_TEMP_RAW_PROC			21
#define BATTERY_READ_CRADLE_MV_PROC			22
#define BATTERY_READ_VBUS_MV_PROC			23
#define BATTERY_READ_PM_MSM_PROC				24
#define BATTERY_READ_PM_MSM_RAW_PROC		25
#define BATTERY_READ_PM_PA_PROC				26
#define BATTERY_READ_PM_PA_RAW_PROC			27
#define BATTERY_READ_CONNECT_PROC			28
#define BATTERY_READ_PM_MSM_OVERHEAT_LAYER_PROC			29
#define BATTERY_READ_THERMAL_TEST1_PROC			30
#define BATTERY_READ_THERMAL_TEST2_PROC			31

#define VBATT_FILTER			2

#define BATTERY_CB_TYPE_PROC		1
#define BATTERY_CB_ID_ALL_ACTIV	1
#define BATTERY_CB_ID_LOW_VOL		2

#define BATTERY_LOW		3400
#if USE_CHARGING_4350MV_BATTERY_PACK
#define BATTERY_HIGH		4350
#else
#define BATTERY_HIGH		4200
#endif

#define ONCRPC_CHG_GET_GENERAL_STATUS_PROC	12
#define ONCRPC_CHARGER_API_VERSIONS_PROC	0xffffffff

#define BATT_RPC_TIMEOUT    5000	/* 5 sec */

#define INVALID_BATT_HANDLE    -1

#define RPC_TYPE_REQ     0
#define RPC_TYPE_REPLY   1
#define RPC_REQ_REPLY_COMMON_HEADER_SIZE   (3 * sizeof(uint32_t))

enum {
	BATTERY_REGISTRATION_SUCCESSFUL = 0,
	BATTERY_DEREGISTRATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_MODIFICATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_INTERROGATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_CLIENT_TABLE_FULL = 1,
	BATTERY_REG_PARAMS_WRONG = 2,
	BATTERY_DEREGISTRATION_FAILED = 4,
	BATTERY_MODIFICATION_FAILED = 8,
	BATTERY_INTERROGATION_FAILED = 16,
	/* Client's filter could not be set because perhaps it does not exist */
	BATTERY_SET_FILTER_FAILED         = 32,
	/* Client's could not be found for enabling or disabling the individual
	 * client */
	BATTERY_ENABLE_DISABLE_INDIVIDUAL_CLIENT_FAILED  = 64,
	BATTERY_LAST_ERROR = 128,
};

enum {
	BATTERY_VOLTAGE_UP = 0,
	BATTERY_VOLTAGE_DOWN,
	BATTERY_VOLTAGE_ABOVE_THIS_LEVEL,
	BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
	BATTERY_VOLTAGE_LEVEL,
	BATTERY_ALL_ACTIVITY,
	VBATT_CHG_EVENTS,
	BATTERY_VOLTAGE_UNKNOWN,
};

/*
 * This enum contains defintions of the charger hardware status
 */
enum chg_charger_status_type {
	/* The charger is good      */
	CHARGER_STATUS_GOOD,
	/* The charger is bad       */
	CHARGER_STATUS_BAD,
	/* The charger is weak      */
	CHARGER_STATUS_WEAK,
	/* Invalid charger status.  */
	CHARGER_STATUS_INVALID
};

/*
 *This enum contains defintions of the charger hardware type
 */
enum chg_charger_hardware_type {
	/* The charger is removed                 */
	CHARGER_TYPE_NONE,
	/* The charger is a regular wall charger   */
	CHARGER_TYPE_WALL,
	/* The charger is a PC USB                 */
	CHARGER_TYPE_USB_PC,
	/* The charger is a wall USB charger       */
	CHARGER_TYPE_USB_WALL,
	/* The charger is a USB carkit             */
	CHARGER_TYPE_USB_CARKIT,
	/* Invalid charger hardware status.        */
	CHARGER_TYPE_INVALID
};

/*
 *  This enum contains defintions of the battery status
 */
enum chg_battery_status_type {
	/* The battery is good        */
	BATTERY_STATUS_GOOD,
	/* The battery is cold/hot    */
	BATTERY_STATUS_BAD_TEMP,
	/* The battery is bad         */
	BATTERY_STATUS_BAD,
	/* The battery is removed     */
	BATTERY_STATUS_REMOVED,		/* on v2.2 only */
	BATTERY_STATUS_INVALID_v1 = BATTERY_STATUS_REMOVED,
	/* Invalid battery status.    */
	BATTERY_STATUS_INVALID
};

/*
 *This enum contains defintions of the battery voltage level
 */
enum chg_battery_level_type {
	/* The battery voltage is dead/very low (less than 3.2V) */
	BATTERY_LEVEL_DEAD,
	/* The battery voltage is weak/low (between 3.2V and 3.4V) */
	BATTERY_LEVEL_WEAK,
	/* The battery voltage is good/normal(between 3.4V and 4.2V) */
	BATTERY_LEVEL_GOOD,
	/* The battery voltage is up to full (close to 4.2V) */
	BATTERY_LEVEL_FULL,
	/* Invalid battery voltage level. */
	BATTERY_LEVEL_INVALID
};

struct msm_battery_info {
	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 chg_api_version;
	u32 batt_technology;
	u32 batt_api_version;

	u32 avail_chg_sources;
	u32 current_chg_source;

	u32 batt_status;
	u32 batt_health;
	u32 charger_valid;
	u32 batt_valid;
	u32 batt_capacity; /* in percentage */
	u32 re_batt_capacity;

	u32 charger_status;
	u32 charger_type;
	u32 battery_status;
	u32 battery_level;
	u32 battery_voltage; /* in millie volts */
	u32 re_battery_voltage;
	u32 battery_temp;  /* in celsius */
       u32 battery_temp_raw;
	u32 battery_conn;
	u32 battery_cradle_mv;
	u32 battery_vbus_mv;

	u32 pm_temp_msm;
	u32 pm_temp_msm_raw;
	u32 pm_temp_pa;
	u32 pm_temp_pa_raw;
	int pm_temp_msm_over_layer;
	int pm_temp_msm_overheat;

	u32 chg_flow_status_now;

	u32(*calculate_capacity) (u32 voltage);

	s32 batt_handle;

	struct power_supply *msm_psy_ac;
	struct power_supply *msm_psy_usb;
	struct power_supply *msm_psy_batt;
	struct power_supply *current_ps;

	struct msm_rpc_client *batt_client;
	struct msm_rpc_endpoint *chg_ep;

	wait_queue_head_t wait_q;

	u32 vbatt_modify_reply_avail;

	struct early_suspend early_suspend;
};

static struct msm_battery_info msm_batt_info = {
	.batt_handle = INVALID_BATT_HANDLE,
	.charger_status = CHARGER_STATUS_BAD,
	.charger_type = CHARGER_TYPE_INVALID,
	.battery_status = BATTERY_STATUS_GOOD,
	.battery_level = BATTERY_LEVEL_FULL,
	.battery_voltage = BATTERY_HIGH,
	.batt_capacity = 100,
	.re_battery_voltage = 0,
	.re_batt_capacity = 0,
	.batt_status = POWER_SUPPLY_STATUS_DISCHARGING,
	.batt_health = POWER_SUPPLY_HEALTH_GOOD,
	.batt_valid  = 1,
#if MAX17040GT_ENABLE_UPDATE_RCOMP
	.battery_temp = 20,
	.pm_temp_msm = 20,
	.pm_temp_pa = 20,
#else
       .battery_temp = 27,
	.pm_temp_msm = 27,
	.pm_temp_pa = 27,
#endif
       .battery_temp_raw = 0,
       .pm_temp_msm_raw = 0,
	.pm_temp_pa_raw = 0,
	.pm_temp_msm_over_layer = 0,
	.pm_temp_msm_overheat = 0,
       .battery_conn = 1,
       .battery_cradle_mv = 0,
       .battery_vbus_mv = 0,
	.vbatt_modify_reply_avail = 0,
	.chg_flow_status_now = CHG_FLOW_STATUS_B,
};

struct msm_batt_get_val_ret_data {
       u32 battery_voltage;
	u32 battery_temp;
	u32 battery_temp_raw;
	u32 battery_conn;
	u32 battery_cradle_mv;
	u32 battery_vbus_mv;
};

static int msm_batt_get_val_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct msm_batt_get_val_ret_data *data_ptr, *buf_ptr;

	data_ptr = (struct msm_batt_get_val_ret_data *)data;
	buf_ptr = (struct msm_batt_get_val_ret_data *)buf;
       data_ptr->battery_voltage = be32_to_cpu(buf_ptr->battery_voltage);
	data_ptr->battery_temp = be32_to_cpu(buf_ptr->battery_temp);
	data_ptr->battery_temp_raw = be32_to_cpu(buf_ptr->battery_temp_raw);
	data_ptr->battery_conn = be32_to_cpu(buf_ptr->battery_conn);
	data_ptr->battery_cradle_mv = be32_to_cpu(buf_ptr->battery_cradle_mv);
	data_ptr->battery_vbus_mv = be32_to_cpu(buf_ptr->battery_vbus_mv);

	return 0;
}


static struct msm_batt_get_val_ret_data msm_batt_get_vbatt_mv_temp(void)
{
	int rc;

	struct msm_batt_get_val_ret_data rep;

      // printk(KERN_ERR "[IS3_POWER] Call_RPC(id=%d) --- start\n",BATTERY_READ_MV_TEMP_PROC);

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_READ_MV_TEMP_PROC,
			NULL, NULL,
			msm_batt_get_val_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt get temp. rc=%d\n", __func__, rc);
		rep.battery_voltage = 0;
		rep.battery_temp = 0;
		rep.battery_temp_raw = 0;
		rep.battery_conn = 1;
		rep.battery_cradle_mv = 0;
		rep.battery_vbus_mv = 0;
	}

	DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] battery_voltage=%d\n",(u16)rep.battery_voltage);
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] battery_temp=%d\n",(s16)rep.battery_temp);
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] battery_temp_raw=%d\n",(u16)rep.battery_temp_raw);
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] battery_conn=%d\n",(u16)rep.battery_conn);
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] battery_cradle_mv=%d\n",(u16)rep.battery_cradle_mv);
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] battery_vbus_mv=%d\n",(u16)rep.battery_vbus_mv);
       //printk(KERN_ERR "[IS3_POWER] Call_RPC(id=%d) --- end\n",BATTERY_READ_MV_TEMP_PROC);

	return rep;
}

struct msm_pm_get_val_ret_data {
       u32 pm_temp_msm;
	u32 pm_temp_msm_raw;
	u32 pm_temp_msm_overheat_layer;
	u32 pm_temp_pa;
	u32 pm_temp_pa_raw;
};

static int msm_pm_get_val_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct msm_pm_get_val_ret_data *data_ptr, *buf_ptr;

	data_ptr = (struct msm_pm_get_val_ret_data *)data;
	buf_ptr = (struct msm_pm_get_val_ret_data *)buf;
       data_ptr->pm_temp_msm = be32_to_cpu(buf_ptr->pm_temp_msm);
	data_ptr->pm_temp_msm_raw = be32_to_cpu(buf_ptr->pm_temp_msm_raw);
	data_ptr->pm_temp_msm_overheat_layer = be32_to_cpu(buf_ptr->pm_temp_msm_overheat_layer);
	data_ptr->pm_temp_pa = be32_to_cpu(buf_ptr->pm_temp_pa);
	data_ptr->pm_temp_pa_raw = be32_to_cpu(buf_ptr->pm_temp_pa_raw);

	return 0;
}


static struct msm_pm_get_val_ret_data msm_vbatt_get_pm_temp(void)
{
	int rc;

	struct msm_pm_get_val_ret_data rep;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_READ_PM_TEMP_PROC,
			NULL, NULL,
			msm_pm_get_val_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt get temp. rc=%d\n", __func__, rc);
		rep.pm_temp_msm = 0;
		rep.pm_temp_msm_raw = 0;
		rep.pm_temp_msm_overheat_layer = 0;
		rep.pm_temp_pa = 0;
		rep.pm_temp_pa_raw = 0;
	}

	DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] pm_temp_msm=%d\n",(u16)rep.pm_temp_msm);
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] pm_temp_msm_raw=%d\n",(u16)rep.pm_temp_msm_raw);
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] pm_temp_msm_overheat_layer=%d\n",(u16)rep.pm_temp_msm_overheat_layer);
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] pm_temp_pa=%d\n",(u16)rep.pm_temp_pa);
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] pm_temp_pa_raw=%d\n",(u16)rep.pm_temp_pa_raw);

	return rep;
}

struct msm_batt_get_ret_data {
       u32 battery_val;
};

static int msm_batt_get_conn_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct msm_batt_get_ret_data *data_ptr, *buf_ptr;

	data_ptr = (struct msm_batt_get_ret_data *)data;
	buf_ptr = (struct msm_batt_get_ret_data *)buf;
       data_ptr->battery_val = be32_to_cpu(buf_ptr->battery_val);

	return 0;
}

static u32 msm_batt_get_vbatt_val(u32 id)
{
	int rc;

	struct msm_batt_get_ret_data rep;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			id,
			NULL, NULL,
			msm_batt_get_conn_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt get temp. rc=%d\n", __func__, rc);
		if (id == BATTERY_READ_CONNECT_PROC ||
		    id == BATTERY_READ_CONN_PROC)
		{
		     return 1;
		}
		else
		{
		     return 0;
		}
	}

	DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] batt_id=%d, batt_val=%d\n",id,rep.battery_val);

	return rep.battery_val;
}

typedef struct
{
   s32 x;
   s32 y;
} AdcMapPtInt32toInt32Type;

#if USE_CHARGING_4350MV_BATTERY_PACK
static const AdcMapPtInt32toInt32Type adcMap_capacity[] =
{
   {3335 , 0 },
   {3560 , 5 },
   {3671 , 10 },
   {3698 , 15 },
   {3745 , 20 },
   {3753 , 25 },
   {3760 , 30 },
   {3770 , 35 },
   {3786 , 40 },
   {3803 , 45 },
   {3820 , 50 },
   {3857 , 55 },
   {3901 , 60 },
   {3943 , 65 },
   {3983 , 70 },
   {4028 , 75 },
   {4080 , 80 },
   {4135 , 85 },
   {4193 ,	 90 },
   {4255 ,	 95 },
   {4301 ,	 100 },
};
#else
static const AdcMapPtInt32toInt32Type adcMap_capacity[] =
{
   {3267 , 0 },
   {3550 , 5 },
   {3652 , 10 },
   {3657 , 15 },
   {3682 , 20 },
   {3720 , 25 },
   {3732 , 30 },
   {3741 , 35 },
   {3757 , 40 },
   {3772 , 45 },
   {3795 , 50 },
   {3815 , 55 },
   {3842 , 60 },
   {3875 , 65 },
   {3903 , 70 },
   {3937 , 75 },
   {3965 , 80 },
   {4012 , 85 },
   {4060 ,	 90 },
   {4095 ,	 95 },
   {4157 ,	 100 },
};
#endif

#define MAX_VBATT_ADC_VALUE	2084
#define MIN_VBATT_ADC_VALUE	750

static const AdcMapPtInt32toInt32Type adcMap_BattTherm[] =
{
   {2141 , -40 },
   {2117 , -35 },
   {2084 , -30 },
   {2042 , -25 },
   {1988 , -20 },
   {1921 , -15 },
   {1843 , -10 },
   {1753 ,  -5 },
   {1654 ,   0 },
   {1549 ,   5 },
   {1442 ,  10 },
   {1338 ,  15 },
   {1239 ,  20 },
   {1149  ,  25 },
   {1068  ,  30 },
   { 997 ,  35 },
   { 936 ,  40 },
   { 884 ,	45 },
   { 841 ,	50 },
   { 805 ,	55 },
   { 775 ,	60 },
   { 750 ,	65 },
   { 729 ,	70 },
   { 712 ,	75 },
   { 698 ,	80 },
   { 686 ,	85 },
   { 677 ,	90 },
   { 668 ,	95 },
   { 662 ,	100},
   { 656 ,	105},
   { 651 ,	110},
   { 647 ,	115},
   { 644 ,	120}
};

static const AdcMapPtInt32toInt32Type qci_adcMap_PmTherm[] =
{
   {2174 , -40 },
   {2163 , -35 },
   {2149 , -30 },
   {2130 , -25 },
   {2105 , -20 },
   {2074 , -15 },
   {2034 , -10 },
   {1984 ,  -5 },
   {1924 ,   0 },
   {1852 ,   5 },
   {1769 ,  10 },
   {1674 ,  15 },
   {1570 ,  20 },
   {1457 ,  25 },
   {1338 ,  30 },
   {1217 ,  35 },
   {1097 ,  40 },
   {1073 ,  41 },
   {1049 ,	  42 },
   {1025 ,	  43 },
   {1002 ,	  44 },
   {979 ,	45 },
   {956 ,	46 },
   {934 ,	47 },
   {911 ,	48 },
   {889 ,	49 },
   {867 ,	50 },
   {846 ,	51 },
   {825 ,	52 },
   {804 ,	53 },
   {783 ,	54 },
   {763 ,	55 },
   {743 ,	56 },
   {724 ,	57 },
   {705 ,	58 },
   {686 ,	59 },
   {668 ,	60 },
   {650 ,	61 },
   {581 ,	65 },
   {504 ,	70 },
   {437 ,	75 },
   {378 ,	80 },
   {326 ,	85 },
   {282 ,	90 },
   {244 ,	95 },
   {211 ,	100},
   {183 ,	105},
   {159 ,	110},
   {138 ,	115},
   {120 ,	120}
};

void AdcMapLinearInt32toInt32(
   const AdcMapPtInt32toInt32Type  *paPts,
   u32                           nTableSize,
   s32                            nInput,
   s32                           *pnOutput,
   u8                             mode
)
{
   u8 bDescending = 1;
   u32 nSearchIdx = 0;

   if ((paPts == NULL) || (pnOutput == NULL))
   {
      return;
   }

   if (mode == 1)
   {
       /*if (nInput > MAX_VBATT_ADC_VALUE || nInput < MIN_VBATT_ADC_VALUE)
       {
          *pnOutput = 0;
          return;
       }*/
   }

   /* Check if table is descending or ascending */
   if (nTableSize > 1)
   {
      if (paPts[0].x < paPts[1].x)
      {
         bDescending = 0;
      }
   }

   while (nSearchIdx < nTableSize)
   {
      if ( (bDescending == 1) && (paPts[nSearchIdx].x < nInput) )
      {
        /* table entry is less than measured value and table is descending, stop */
        break;
      }
      else if ( (bDescending == 0) && (paPts[nSearchIdx].x > nInput) )
      {
        /* table entry is greater than measured value and table is ascending, stop */
        break;
      }
      else
      {
        nSearchIdx++;
      }
   }

   if (nSearchIdx == 0)
   {
      *pnOutput = paPts[0].y;
   }
   else if (nSearchIdx == nTableSize)
   {
      *pnOutput = paPts[nTableSize-1].y;
   }
   else
   {
      /* result is between search_index and search_index-1 */
      /* interpolate linearly */
      *pnOutput = (
               ( (s32)
                   (
                    (paPts[nSearchIdx].y - paPts[nSearchIdx-1].y)
                     * (nInput - paPts[nSearchIdx-1].x)
                   )
                   / (paPts[nSearchIdx].x - paPts[nSearchIdx-1].x)
               )
               + paPts[nSearchIdx-1].y
             );
   }

   if (mode == 0)
   {
      if (*pnOutput < 0) *pnOutput = 0;
      else if (*pnOutput > 100) *pnOutput = 100;
   }

   return;
}

static struct msm_batt_get_val_ret_data update_batt_mv_temp_RPC(int mode,int vbatt_update)
{
       struct msm_batt_get_val_ret_data rep;

       if (mode == 0)
       {
           rep = msm_batt_get_vbatt_mv_temp();
       }
       else
       {
           if (vbatt_update == 1)
              rep.battery_voltage = msm_batt_get_vbatt_val(BATTERY_READ_MV_PROC);
	    else
		rep.battery_voltage = 0;
	    //rep.battery_temp = msm_batt_get_vbatt_val(BATTERY_READ_TEMP_PROC);
	    rep.battery_temp_raw = msm_batt_get_vbatt_val(BATTERY_READ_TEMP_RAW_PROC);
	    //rep.battery_conn = msm_batt_get_vbatt_val(BATTERY_READ_CONNECT_PROC);
           rep.battery_cradle_mv = msm_batt_get_vbatt_val(BATTERY_READ_CRADLE_MV_PROC);
	    rep.battery_vbus_mv = msm_batt_get_vbatt_val(BATTERY_READ_VBUS_MV_PROC);

	    AdcMapLinearInt32toInt32(
                    adcMap_BattTherm,
                    sizeof(adcMap_BattTherm)/sizeof(adcMap_BattTherm[0]),
                    rep.battery_temp_raw,
                    &rep.battery_temp,
                    1);

	    if (rep.battery_temp_raw > MAX_VBATT_ADC_VALUE ||
               rep.battery_temp_raw < MIN_VBATT_ADC_VALUE)
           {
                rep.battery_conn = 0;
           }
	    else
	    {
	         rep.battery_conn = 1;
	    }
       }

	return rep;
}

static struct msm_pm_get_val_ret_data update_pm_temp_RPC(int mode,int get)
{
       struct msm_pm_get_val_ret_data rep;

	if (mode == 0)
	{
	    rep = msm_vbatt_get_pm_temp();
	}
	else
	{
	    if (get == 0)
	    {
	         //rep.pm_temp_msm = msm_batt_get_vbatt_val(BATTERY_READ_PM_MSM_PROC);
	         rep.pm_temp_msm_raw = msm_batt_get_vbatt_val(BATTERY_READ_PM_MSM_RAW_PROC);
		  rep.pm_temp_msm_overheat_layer  = msm_batt_get_vbatt_val(BATTERY_READ_PM_MSM_OVERHEAT_LAYER_PROC);
		  rep.pm_temp_msm = msm_batt_get_vbatt_val(BATTERY_READ_PM_MSM_PROC);
	    }
	    else
	    {
	         //rep.pm_temp_pa = msm_batt_get_vbatt_val(BATTERY_READ_TEMP_PROC);
	         rep.pm_temp_pa_raw = msm_batt_get_vbatt_val(BATTERY_READ_PM_PA_RAW_PROC);

		  AdcMapLinearInt32toInt32(
                    qci_adcMap_PmTherm,
                    sizeof(qci_adcMap_PmTherm)/sizeof(qci_adcMap_PmTherm[0]),
                    rep.pm_temp_pa_raw,
                    &rep.pm_temp_pa,
                    2);
	    }
	}

	return rep;
}

static enum power_supply_property msm_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_power_supplied_to[] = {
	"battery",
};

static int msm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = msm_batt_info.current_chg_source & AC_CHG
			    ? 1 : 0;
		}
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = msm_batt_info.current_chg_source & USB_CHG
			    ? 1 : 0;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

u32 read_nv_item(void)
{
      u32 value = 0;
      u32 param1 = 2822;
      u32 param2 = 0;
      int ret = 0;

      ret=msm_proc_comm(PCOM_NV_READ, &param1, &param2);
      if(ret != 0){
		printk(KERN_ERR "[IS3_POWER] read_nv_item,ret=%d fail\n",ret);
      }
      else{
              printk(KERN_ERR "[IS3_POWER] read_nv_item(%d)=%d\n",param1 ,param2);
	       value = param2;
      }

      return value;
}

void write_nv_item(u32 data,u8 debug_level)
{
       u32 param1 = 2822;
	u32 param2;
	int ret = 0;

       param2 = data;

	ret=msm_proc_comm(PCOM_NV_WRITE, &param1, &param2);
	if(ret != 0){
		if (debug_level == 0)
		   DBG_LIMIT(debug_level,TYPE_MAX17040, "[MAX17040] write_nv_item(0x%x),ret=%d fail\n",param2,ret);
	       else
		   printk(KERN_ERR "[IS3_POWER] write_nv_item(0x%x),ret=%d fail\n",param2,ret);
	}
	else
	{
	       if (debug_level == 0)
		   DBG_LIMIT(debug_level,TYPE_MAX17040, "[MAX17040] write_nv_item(0x%x),ret=%d pass\n",param2,ret);
	       else
		   printk(KERN_ERR "[IS3_POWER] write_nv_item(0x%x),ret=%d pass\n",param2,ret);
	}
}


static struct power_supply msm_psy_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static struct power_supply msm_psy_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
       POWER_SUPPLY_PROP_TEMP,
       POWER_SUPPLY_PROP_TEMP_RAW,
       POWER_SUPPLY_PROP_TEMP_MSM,
       POWER_SUPPLY_PROP_TEMP_MSM_RAW,
       POWER_SUPPLY_PROP_TEMP_PA,
       POWER_SUPPLY_PROP_TEMP_PA_RAW,
       POWER_SUPPLY_PROP_CRADLE_MODE,
       POWER_SUPPLY_PROP_CRADLE_ONLINE,
       POWER_SUPPLY_PROP_DEBUG_LEVEL,
       POWER_SUPPLY_PROP_DEBUG_MAX17040,
       POWER_SUPPLY_PROP_DEBUG_SMB329B,
       POWER_SUPPLY_PROP_DEBUG_TEMP,
       POWER_SUPPLY_PROP_DEBUG_CAPACITY,
       POWER_SUPPLY_PROP_BATT_MODE,
       POWER_SUPPLY_PROP_OVP,
       POWER_SUPPLY_PROP_OC,
       POWER_SUPPLY_PROP_CAMERA_RECORDING,
       POWER_SUPPLY_PROP_DATA_TRANSMIT,
       POWER_SUPPLY_PROP_ISDB_T,
};

static int msm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	int v;
	int outv;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if ((msm_batt_info.batt_capacity >= 100 &&
		    (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
                   msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)) ||
                   msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
		{
		    //val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		    val->intval = POWER_SUPPLY_STATUS_FULL; //From AP to change to discharging.
		}
		else
		{
		    val->intval = msm_batt_info.batt_status;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = msm_batt_info.batt_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = msm_batt_info.batt_valid;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = msm_batt_info.batt_technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_batt_info.voltage_max_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_batt_info.voltage_min_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		//val->intval = msm_batt_info.battery_voltage * 1000;
		val->intval = msm_batt_info.battery_voltage;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = msm_batt_info.batt_capacity;
		break;
        case POWER_SUPPLY_PROP_TEMP:
              val->intval = msm_batt_info.battery_temp * 10;
              break;
        case POWER_SUPPLY_PROP_TEMP_RAW:
		val->intval = msm_batt_info.battery_temp_raw;
		break;
       case POWER_SUPPLY_PROP_TEMP_MSM:
              val->intval = msm_batt_info.pm_temp_msm * 10;
              break;
        case POWER_SUPPLY_PROP_TEMP_MSM_RAW:
		val->intval = msm_batt_info.pm_temp_msm_raw;
		break;
	case POWER_SUPPLY_PROP_TEMP_PA:
              val->intval = msm_batt_info.pm_temp_pa * 10;
              break;
        case POWER_SUPPLY_PROP_TEMP_PA_RAW:
		val->intval = msm_batt_info.pm_temp_pa_raw;
		break;
	case POWER_SUPPLY_PROP_CRADLE_MODE:
		val->intval = cradle_mode;
		break;
	case POWER_SUPPLY_PROP_CRADLE_ONLINE:
		val->intval = cradle_connect;
		break;
	case POWER_SUPPLY_PROP_DEBUG_LEVEL:
		val->intval = debug_level;
		break;
	case POWER_SUPPLY_PROP_DEBUG_MAX17040:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_DEBUG_SMB329B:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_DEBUG_TEMP:
		val->intval = debug_temp_value;
		break;
	case POWER_SUPPLY_PROP_DEBUG_CAPACITY:
		val->intval = debug_capacity_value;
		break;
	case POWER_SUPPLY_PROP_BATT_MODE:
		if (battery_info_mode == 1)
		    val->intval = 1;
		else
		    val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_OVP:
		if (smb329b_ready != CHIP_STATUS_NONE)
		{
		   outv = 0;
		   v =  gpio_get_value_cansleep(smb329b_chip->ovp_sw1_off_gpio);
	          //printk(KERN_ERR "[IS3_POWER] ovp_sw1_off_gpio = %d\n",v);
		   if (v != 0)
		      outv += 1;
		   v =  gpio_get_value_cansleep(smb329b_chip->ovp_sw2_off_gpio);
	          //printk(KERN_ERR "[IS3_POWER] ovp_sw2_off_gpio = %d\n",v);
		   if (v != 0)
		      outv += 2;
                 val->intval = outv;
		}
		else
		   val->intval = 0;
		break;
       case POWER_SUPPLY_PROP_OC:
                 val->intval = qci_get_offline_charging();
              break;
	case POWER_SUPPLY_PROP_CAMERA_RECORDING:
		   val->intval = camera_recording;
		break;
	case POWER_SUPPLY_PROP_DATA_TRANSMIT:
		   val->intval = data_transmit;
		break;
	case POWER_SUPPLY_PROP_ISDB_T:
		   val->intval = isdb_t_on;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int msm_batt_power_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
       int tmp;

	switch (psp) {
	case POWER_SUPPLY_PROP_CRADLE_MODE:
		if (val->intval == 0 || val->intval == 1)
		{
		   if (cradle_mode != val->intval && cradle_connect == 1 &&
                      (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
                       msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL))
		   {
		        smb329b_change_cradle_current(val->intval);
		   }
		   cradle_mode = val->intval;
		}
		break;
	case POWER_SUPPLY_PROP_DEBUG_LEVEL:
		debug_level = val->intval;
		break;
	case POWER_SUPPLY_PROP_DEBUG_MAX17040:
		tmp = debug_level;
		if (debug_level == 0 || debug_level == 3)
		   debug_level = 2;
		switch((int)val->intval)
		{
		    case 0:
				max17040gt_load_custom_model(max17040gt_chip->client,0);
				break;
		    case 1:
				max17040gt_load_custom_model(max17040gt_chip->client,1);
				break;
		    case 2:
				max17040gt_verify_model(max17040gt_chip->client,0);
				break;
		    case 3:
				max17040gt_initial_voltage(max17040gt_chip->client);
				break;
		    case 4:
				max17040gt_update_RCOMP(max17040gt_chip->client,20);
				break;
		    case 5:
				max17040gt_quick_start(max17040gt_chip->client);
				break;
		    case 6:
				max17040gt_get_version(max17040gt_chip->client);
				break;
		    case 7:
				max17040gt_reset(max17040gt_chip->client);
				break;
		    case 8:
				DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] rpc_ready=%d\n",rpc_ready);
				if (rpc_ready == 1)
		              {
		                   msm_batt_get_vbatt_val(BATTERY_READ_CONN_PROC);
		              }
				break;
		    case 9:
				DBG_LIMIT(debug_level,TYPE_MAX17040,"[BATT_RPC] rpc_ready=%d\n",rpc_ready);
				if (rpc_ready == 1)
		              {
		                   update_pm_temp_RPC(READ_RPC_MODE,0);
				     update_pm_temp_RPC(READ_RPC_MODE,1);
		              }
				break;
		    case 10:
				DBG_LIMIT(debug_level,TYPE_MAX17040,"[Thermistor] temp_msm_over_layer=%d\n",msm_batt_info.pm_temp_msm_over_layer);
				break;
                  case 100:
				 write_nv_item(0,1);
				 break;
		    case 101:
				 write_nv_item(1,1);
				 break;
		    case 102:
				 DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] read_nv = %d\n",read_nv_item());
				 break;
		    default:
				return 0;
		}
		debug_level = tmp;
		break;
	case POWER_SUPPLY_PROP_DEBUG_SMB329B:
		tmp = debug_level;
		if (debug_level == 0 || debug_level == 2)
		   debug_level = 3;
	       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] get value = %d\n",(int)val->intval);
		switch((int)val->intval)
		{
		      case 0:
		      case 1:
		      case 2:
		      case 3:
		      case 4:
		      case 5:
		      case 6:
		      case 7:
		      case 49:		//0x31
		      case 51:		//0x33
		      case 52:		//0x34
		      case 53:		//0x35
		      case 54:		//0x36
		      case 55:		//0x37
		      case 59:		//0x3B
		                 smb329b_read_reg((u8)val->intval,NULL);
		      break;
		      case 100:
                               smb329b_start_charging(500);
                            break;
		      case 101:
                               smb329b_start_charging(1000);
                            break;
                    case 102:
                               smb329b_stop_charging();
                            break;
		      case 103:
                               smb329b_change_float_voltage(SMB329B_FLOAT_VOLTAGE_DEFAULT);
                            break;
		      case 104:
                               smb329b_check_batt_is_full();
				break;
		      case 105:
                               debug_ovc = 1;
                            break;
		      case 106:
                               debug_ovc = 2;
                            break;
		      case 107:
				   smb329b_is_CV_mode();
				break;
		      case 341:
                               report_chg_err_key(CHG_ST_NTFY_CODE,K_CHG_ST_NONE);
                            break;
		      case 342:
                               report_chg_err_key(CHG_ST_NTFY_CODE,K_CHG_ST_OVP);
                            break;
			case 343:
                               report_chg_err_key(CHG_ST_NTFY_CODE,K_CHG_ST_OVC);
                            break;
			case 344:
                               report_chg_err_key(CHG_ST_NTFY_CODE,K_CHG_ST_OVD);
                            break;
                     case 345:
                               report_chg_err_key(CHG_ST_NTFY_CODE,K_CHG_ST_EXP);
                            break;
			case 347:
				   report_chg_err_key(CHG_ST_NTFY_CODE,K_TEMP_MSM_OVER_LAYER1);
				break;
			case 348:
				   report_chg_err_key(CHG_ST_NTFY_CODE,K_TEMP_MSM_OVER_LAYER2);
				break;
			case 349:
				   report_chg_err_key(CHG_ST_NTFY_CODE,K_TEMP_MSM_OVER_LAYER3);
				break;
			case 350:
				   report_chg_err_key(CHG_ST_NTFY_CODE,K_TEMP_MSM_OVER_RESUME_NORMAL);
				break;
		      default:
                            return 0;
		}
		debug_level = tmp;
		break;
	case POWER_SUPPLY_PROP_DEBUG_TEMP:
		debug_temp_value = (int)val->intval;
		if (debug_temp_value == 101) // for test temp_msm
	       {
	           msm_batt_get_vbatt_val(BATTERY_READ_THERMAL_TEST1_PROC);
	       }
	       else if (debug_temp_value == 102)
	       {
	           msm_batt_get_vbatt_val(BATTERY_READ_THERMAL_TEST2_PROC);
	       }
		break;
	case POWER_SUPPLY_PROP_DEBUG_CAPACITY:
		debug_capacity_value = (int)val->intval;
		break;
	case POWER_SUPPLY_PROP_CAMERA_RECORDING:
#if QCI_CHG_SOLUTION
		if (val->intval >= 0 && val->intval <= 1)
		    camera_recording = (int)val->intval;
	       else
#endif
		    camera_recording = 0;
		break;
	case POWER_SUPPLY_PROP_DATA_TRANSMIT:
#if QCI_CHG_SOLUTION
		if (val->intval >= 0 && val->intval <= 1)
		    data_transmit = (int)val->intval;
	       else
#endif
		    data_transmit = 0;
		break;
	case POWER_SUPPLY_PROP_ISDB_T:
#if ISDB_T_SOLUTION
		 if (val->intval >= 0 && val->intval <= 1)
		     isdb_t_on = (int)val->intval;
		 else
#endif
		     isdb_t_on = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_batt = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_batt_power_props,
	.num_properties = ARRAY_SIZE(msm_batt_power_props),
	.get_property = msm_batt_power_get_property,
	.set_property = msm_batt_power_set_property,
};


#define	be32_to_cpu_self(v)	(v = be32_to_cpu(v))

static void update_u_event(void)
{
       if (msm_batt_info.current_chg_source & AC_CHG)
           power_supply_changed( &msm_psy_ac);
	else if (msm_batt_info.current_chg_source & USB_CHG)
	    power_supply_changed( &msm_psy_usb);
       else
           power_supply_changed( &msm_psy_batt);

	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] update_u_event()\n");
}

static void msm_batt_update_psy_status(void)
{
       rpc_ready = 1;
}

struct batt_modify_client_req {

	u32 client_handle;

	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
};

struct batt_modify_client_rep {
	u32 result;
};

static int msm_batt_modify_client_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_modify_client_req *batt_modify_client_req =
		(struct batt_modify_client_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(batt_modify_client_req->client_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->cb_data);
	size += sizeof(u32);

	return size;
}

static int msm_batt_modify_client_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct  batt_modify_client_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_modify_client_rep *)data;
	buf_ptr = (struct batt_modify_client_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);

	return 0;
}

static int msm_batt_modify_client(u32 client_handle, u32 desired_batt_voltage,
	     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	int rc;

	struct batt_modify_client_req  req;
	struct batt_modify_client_rep rep;

	req.client_handle = client_handle;
	req.desired_batt_voltage = desired_batt_voltage;
	req.voltage_direction = voltage_direction;
	req.batt_cb_id = batt_cb_id;
	req.cb_data = cb_data;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_MODIFY_CLIENT_PROC,
			msm_batt_modify_client_arg_func, &req,
			msm_batt_modify_client_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: ERROR. failed to modify  Vbatt client\n",
		       __func__);
		return rc;
	}

	if (rep.result != BATTERY_MODIFICATION_SUCCESSFUL) {
		pr_err("%s: ERROR. modify client failed. result = %u\n",
		       __func__, rep.result);
		return -EIO;
	}

	return 0;
}

int check_msm_temp_over_heat(void)
{
     return msm_batt_info.pm_temp_msm_overheat;
}
EXPORT_SYMBOL_GPL(check_msm_temp_over_heat);

void qci_schedule_work(int mode,struct work_struct *work)
{
	bool schedule_status;
	char call_mode[256];
	int err = 0;

	switch(mode)
	{
	     case 0: strcpy(call_mode,"<smb329b_chip->cradle_work>"); break;
	     case 1: strcpy(call_mode,"<smb329b_chip->ovp1_work>"); break;
            case 2: strcpy(call_mode,"<smb329b_chip->ovp2_work>"); break;
	     case 3: strcpy(call_mode,"<smb329b_chip->vbus_work>"); break;
	     case 4: strcpy(call_mode,"<smb329b_chip->det_work2>"); break;
	     case 7: strcpy(call_mode,"<smb329b_chip->det_work>"); break;
	     default: strcpy(call_mode,"<unknow>");
	}

       schedule_status = queue_work(batt_chg_work_queue,work);
	if (schedule_status == 0 || err == 1)
	     printk(KERN_ERR "[IS3_POWER] schedule_work_status-fail = %d,%s\n",schedule_status,call_mode);
}


void qci_schedule_delayed_work(int mode,struct delayed_work *work,unsigned long delay)
{
	bool schedule_status;
	char call_mode[256];
	int err = 0;

	switch(mode)
	{
	     case 0: strcpy(call_mode,"<max17040gt_chip->work>"); break;
	     case 1: strcpy(call_mode,"<smb329b_chip->stop_charging_ic_work>"); break;
	     case 2: strcpy(call_mode,"<thermal_msm_work>"); break;
	     case 3: strcpy(call_mode,"<smb329b_chip->temp_work>"); break;
	     case 4: strcpy(call_mode,"<smb329b_chip->led_red_blink_work>"); break;
	     case 5: strcpy(call_mode,"<smb329b_chip->offline_charging_work>"); break;
	     case 6: strcpy(call_mode,"<smb329b_chip->wait_ap_update_work>"); break;
	     case 7: strcpy(call_mode,"<smb329b_chip->det_work>"); break;
	     case 8: strcpy(call_mode,"<max17040gt_chip->work_polling>"); break;
	     case 9: strcpy(call_mode,"<smb329b_chip->temp_work_polling>"); break;
	     case 10: strcpy(call_mode,"<smb329b_chip->offline_charging_work_polling>"); break;
	     case 11: strcpy(call_mode,"<smb329b_chip->wait_exit_error_work>"); break;
	     case 12: strcpy(call_mode,"<smb329b_chip->wait_exit_error_work_polling>"); break;
	     case 13: strcpy(call_mode,"<max17040gt_chip->msm_work>"); break;
	     case 14: strcpy(call_mode,"<max17040gt_chip->pa_work>"); break;
	     case 15: strcpy(call_mode,"<thermal_msm_work2>"); break;
	     case 16: strcpy(call_mode,"<smb329b_chip->det_delay_work>"); break;
	     case 17: strcpy(call_mode,"<smb329b_chip->send_keycode_work>"); break;
	     default: strcpy(call_mode,"<unknow>");
		            err = 1;
	}

       schedule_status = queue_delayed_work(batt_chg_work_queue,work, delay);
	if (schedule_status == 0 || err == 1)
	     printk(KERN_ERR "[IS3_POWER] schedule_delayed_work_status-fail = %d,%s\n",schedule_status,call_mode);
}

void qci_cancel_delayed_work(int mode,int *work_status,struct delayed_work *work,int sync)
{
     bool cancel_status;
     char call_mode[256];
     int err = 0;

     switch(mode)
     {
	     case 0: strcpy(call_mode,"<max17040gt_chip->work>"); break;
	     case 1: strcpy(call_mode,"<smb329b_chip->stop_charging_ic_work>"); break;
	     case 2: strcpy(call_mode,"<thermal_msm_work>"); break;
	     case 3: strcpy(call_mode,"<smb329b_chip->temp_work>"); break;
	     case 4: strcpy(call_mode,"<smb329b_chip->led_red_blink_work>"); break;
	     case 5: strcpy(call_mode,"<smb329b_chip->offline_charging_work>"); break;
	     case 6: strcpy(call_mode,"<smb329b_chip->wait_ap_update_work>"); break;
	     case 7: strcpy(call_mode,"<smb329b_chip->det_work>"); break;
	     case 8: strcpy(call_mode,"<max17040gt_chip->work_polling>"); break;
	     case 9: strcpy(call_mode,"<smb329b_chip->temp_work_polling>"); break;
	     case 10: strcpy(call_mode,"<smb329b_chip->offline_charging_work_polling>"); break;
	     case 11: strcpy(call_mode,"<smb329b_chip->wait_exit_error_work>"); break;
	     case 12: strcpy(call_mode,"<smb329b_chip->wait_exit_error_work_polling>"); break;
	     case 13: strcpy(call_mode,"<max17040gt_chip->msm_work>"); break;
	     case 14: strcpy(call_mode,"<max17040gt_chip->pa_work>"); break;
	     case 15: strcpy(call_mode,"<thermal_msm_work2>"); break;
	     case 16: strcpy(call_mode,"<smb329b_chip->det_delay_work>"); break;
	     case 17: strcpy(call_mode,"<smb329b_chip->send_keycode_work>"); break;
	     default: strcpy(call_mode,"<unknow>");
		            err = 1;
     }

     if (sync == 1)
	 cancel_status = cancel_delayed_work_sync(work);
     else
        cancel_status = cancel_delayed_work(work);
     if (cancel_status == 0 || err == 1)
     {
         printk(KERN_ERR "[IS3_POWER] cancel_work_status - fail , ws=%d,cs=%d,%s\n",*work_status,cancel_status,call_mode);
     }
     if (work_status != NULL)
     {
        *work_status = 0;
     }
}

void set_usb_type(int type)
{
     mutex_lock(&usb_type_lock);
     vbus_type = type;
     mutex_unlock(&usb_type_lock);
}

static unsigned long get_rtc_time_s(void)
{
        struct timespec ts;
	 struct rtc_time tm;

	 getnstimeofday(&ts);
        rtc_time_to_tm(ts.tv_sec, &tm);

	 printk(KERN_ERR "%d-%02d-%02d %02d:%02d:%02d.%09lu\n",
                                   tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                                   tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);

        return mktime(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                                 tm.tm_hour, tm.tm_min, tm.tm_sec);
}

int check_vbus_is_connect(void)
{
       int val;

	val =  gpio_get_value_cansleep(smb329b_chip->usb_vbus_mpp);
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] usb_vbus_mpp = %d\n",val);

	if (val == 0)
	{
	    vbus_connect = 1;
	    return 1;
	}
	else
	{
	    vbus_connect = 0;
	    return 0;
	}
}


int check_cradle_is_connect(void)
{
       int val;

	val =  gpio_get_value_cansleep(smb329b_chip->xcradle_state_gpio);
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] xcradle_state_gpio = %d\n",val);

	if (val == 0)
	{
	    cradle_connect = 1;
	    return 1;
	}
	else
	{
	    cradle_connect = 0;
	    return 0;
	}
}

void check_cradle_vbus_resume(void)
{
      if (!(msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL ||
            msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING))
	{
	    if (check_ovp1_status() == 0 && check_ovp2_status() == 0)
	    {
              if (check_cradle_is_connect() == 1)
              {
                   if (cradle_mode == 0)
                      smb329b_cradle_draw(500,0);
                   else
		        smb329b_cradle_draw(1000,0);
	            if (wlock_en == 0)
	            {
	               printk(KERN_ERR "[IS3_POWER] wake_lock(suspend)\n");
                      wake_lock(&smb329b_chip->wlock);
                      wlock_en = 1;
	            }
	        }
	        else if (check_vbus_is_connect() == 1)
	        {
	            if (vbus_type < 1)
	                smb329b_vbus_draw2(500,0);
	            else
		         smb329b_vbus_draw2(1000,0);
		     if (wlock_en == 0)
	            {
	               printk(KERN_ERR "[IS3_POWER] wake_lock(suspend)\n");
                      wake_lock(&smb329b_chip->wlock);
                      wlock_en = 1;
		     }
	         }
	     }
	 }
}

#define POWER_OFF_BATTERY_LEVEL  3400
static int msm_batt_suspend(struct device *dev)
{
	int rc;

       batt_work_disable = 1;
	qci_cancel_delayed_work(0,&batt_work_on,&max17040gt_chip->work,1);
	temp_msm_work_disable = 1;
	qci_cancel_delayed_work(13,&batt_work_on,&max17040gt_chip->msm_work,1);
	temp_pa_work_disable = 1;
	qci_cancel_delayed_work(14,&batt_work_on,&max17040gt_chip->pa_work,1);
	get_batt_first_info = 0;

	printk(KERN_ERR "[IS3_POWER] suspend - voltage=%d,capacity=%d\n",msm_batt_info.battery_voltage,msm_batt_info.batt_capacity);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				POWER_OFF_BATTERY_LEVEL, BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
				BATTERY_CB_ID_LOW_VOL, POWER_OFF_BATTERY_LEVEL);
		if (rc < 0) {
			pr_err("%s: msm_batt_modify_client. rc=%d\n",
			       __func__, rc);
			return 1;
		}
	} else {
		pr_err("%s: ERROR. invalid batt_handle\n", __func__);
		return 1;
	}

#ifndef CONFIG_HAS_EARLYSUSPEND
       time_suspend = get_rtc_time_s();
#endif

	return 0;
}

u32 batt_capacity_format_check(u32 value)
{
       if (value < 0)
	   return 0;
	else if (value > 100)
	   return 100;

	return value;
}

void resume_check_max17040gt_status2(void)
{
       u32 tmp;
	int adc_capacity;

	max17040gt_get_vcell(max17040gt_chip->client);
	max17040gt_get_soc(max17040gt_chip->client);
       msm_batt_info.battery_voltage = (u32)max17040gt_chip->vcell;
	tmp = (u32)max17040gt_chip->soc;
	if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL ||
            msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING)
	{
	     if (tmp > msm_batt_info.batt_capacity)
	        msm_batt_info.batt_capacity = tmp;
	}
	else
	{
	    adc_capacity = calculate_adc_capacity(msm_batt_info.battery_voltage);
	    if (abs(adc_capacity - (int)msm_batt_info.batt_capacity) > 40)
	    {
	         printk(KERN_ERR "[IS3_POWER] call max17040gt_quick_start - abs(adc_capacity=%d -batt_capacity=%d) > 40\n",
                             adc_capacity,(int)msm_batt_info.batt_capacity);
		  max17040gt_load_custom_model(max17040gt_chip->client,0);
	         max17040gt_quick_start(max17040gt_chip->client);
		  max17040gt_get_vcell(max17040gt_chip->client);
	         max17040gt_get_soc(max17040gt_chip->client);
                msm_batt_info.battery_voltage = (u32)max17040gt_chip->vcell;
	         tmp = (u32)max17040gt_chip->soc;
	    }
	    if (tmp < msm_batt_info.batt_capacity)
		msm_batt_info.batt_capacity = tmp;
	}
}
void resume_check_max17040gt_status1(void)
{
       if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_DISCHARGING && batt_work_on == 0)
       {
           if (time_suspend > time_resume)
           {
                time_resume = time_suspend;
           }
           else if (abs64(time_resume - time_suspend) > 3600)
	    {
		  if (max17040gt_verify_model(max17040gt_chip->client,vbatt_max_voltage_mode) != 0)
	         {
	              printk(KERN_ERR "[IS3_POWER] max17040gt_verify_model()-fail call  re_load_custom_model()\n");
		       max17040gt_load_custom_model(max17040gt_chip->client,0);
		  }
		  printk(KERN_ERR "[IS3_POWER] call max17040gt_quick_start()\n");
		  max17040gt_quick_start(max17040gt_chip->client);
		  max17040gt_update_RCOMP(max17040gt_chip->client,(int)msm_batt_info.battery_temp);
		  max17040gt_get_vcell(max17040gt_chip->client);
	         max17040gt_get_soc(max17040gt_chip->client);
                msm_batt_info.re_battery_voltage = (u32)max17040gt_chip->vcell;
	         msm_batt_info.re_batt_capacity = (u32)max17040gt_chip->soc;
		  time_resume = time_suspend;
           }
       }
}

static int msm_batt_resume(struct device *dev)
{
	int rc;
	unsigned long delay_time;

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
		if (rc < 0) {
			pr_err("%s: msm_batt_modify_client FAIL rc=%d\n",
			       __func__, rc);
			return 1;
		}
	} else {
		pr_err("%s: ERROR. invalid batt_handle\n", __func__);
		return 1;
	}

	time_resume= get_rtc_time_s();
       resume_check_max17040gt_status1();
	resume_check_max17040gt_status2();
	printk(KERN_ERR "[IS3_POWER] resume - voltage=%d,capacity=%d\n",msm_batt_info.battery_voltage,msm_batt_info.batt_capacity);
	if (check_vbus_is_connect() == 1 || check_cradle_is_connect() == 1)
	   delay_time = MAX17040GT_DELAY_QUICK;
       else
	   delay_time = MAX17040GT_DELAY_NORMAL;
	msm_batt_info.batt_capacity = batt_capacity_format_check(msm_batt_info.batt_capacity);
       get_batt_first_info = 1;
	update_u_event();
	batt_work_disable = 0;
	qci_schedule_delayed_work(0,&max17040gt_chip->work, delay_time);
       temp_msm_work_disable = 0;
	qci_schedule_delayed_work(13,&max17040gt_chip->msm_work, MAX17040GT_DELAY_RESUME);
	temp_pa_work_disable = 0;
	qci_schedule_delayed_work(14,&max17040gt_chip->pa_work, MAX17040GT_DELAY_RESUME);

       check_cradle_vbus_resume();

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void msm_batt_early_suspend(struct early_suspend *h)
{
	pr_debug("%s: \n", __func__);
	time_suspend = get_rtc_time_s();
}

void msm_batt_late_resume(struct early_suspend *h)
{
	pr_debug("%s: \n", __func__);
	time_resume= get_rtc_time_s();
	resume_check_max17040gt_status1();
}
#endif

static SIMPLE_DEV_PM_OPS(msm_batt_pm_ops,
		msm_batt_suspend, msm_batt_resume);

struct msm_batt_vbatt_filter_req {
	u32 batt_handle;
	u32 enable_filter;
	u32 vbatt_filter;
};

struct msm_batt_vbatt_filter_rep {
	u32 result;
};

static int msm_batt_filter_arg_func(struct msm_rpc_client *batt_client,

		void *buf, void *data)
{
	struct msm_batt_vbatt_filter_req *vbatt_filter_req =
		(struct msm_batt_vbatt_filter_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(vbatt_filter_req->batt_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->enable_filter);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->vbatt_filter);
	size += sizeof(u32);
	return size;
}

static int msm_batt_filter_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{

	struct msm_batt_vbatt_filter_rep *data_ptr, *buf_ptr;

	data_ptr = (struct msm_batt_vbatt_filter_rep *)data;
	buf_ptr = (struct msm_batt_vbatt_filter_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);
	return 0;
}

static int msm_batt_enable_filter(u32 vbatt_filter)
{
	int rc;
	struct  msm_batt_vbatt_filter_req  vbatt_filter_req;
	struct  msm_batt_vbatt_filter_rep  vbatt_filter_rep;

	vbatt_filter_req.batt_handle = msm_batt_info.batt_handle;
	vbatt_filter_req.enable_filter = 1;
	vbatt_filter_req.vbatt_filter = vbatt_filter;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_ENABLE_DISABLE_FILTER_PROC,
			msm_batt_filter_arg_func, &vbatt_filter_req,
			msm_batt_filter_ret_func, &vbatt_filter_rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: enable vbatt filter. rc=%d\n",
		       __func__, rc);
		return rc;
	}

	if (vbatt_filter_rep.result != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: FAIL: enable vbatt filter: result=%d\n",
		       __func__, vbatt_filter_rep.result);
		return -EIO;
	}

	pr_debug("%s: enable vbatt filter: OK\n", __func__);
	return rc;
}

struct batt_client_registration_req {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 more_data;
	u32 batt_error;
};

struct batt_client_registration_req_4_1 {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 batt_error;
};

struct batt_client_registration_rep {
	u32 batt_handle;
};

struct batt_client_registration_rep_4_1 {
	u32 batt_handle;
	u32 more_data;
	u32 err;
};

static int msm_batt_register_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_registration_req *batt_reg_req =
		(struct batt_client_registration_req *)data;

	u32 *req = (u32 *)buf;
	int size = 0;


	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->voltage_direction);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_cb_id);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->cb_data);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_error);
		size += sizeof(u32);

		return size;
	} else {
		*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->voltage_direction);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_cb_id);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->cb_data);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->more_data);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_error);
		size += sizeof(u32);

		return size;
	}

}

static int msm_batt_register_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_registration_rep *data_ptr, *buf_ptr;
	struct batt_client_registration_rep_4_1 *data_ptr_4_1, *buf_ptr_4_1;

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		data_ptr_4_1 = (struct batt_client_registration_rep_4_1 *)data;
		buf_ptr_4_1 = (struct batt_client_registration_rep_4_1 *)buf;

		data_ptr_4_1->batt_handle
			= be32_to_cpu(buf_ptr_4_1->batt_handle);
		data_ptr_4_1->more_data
			= be32_to_cpu(buf_ptr_4_1->more_data);
		data_ptr_4_1->err = be32_to_cpu(buf_ptr_4_1->err);
		return 0;
	} else {
		data_ptr = (struct batt_client_registration_rep *)data;
		buf_ptr = (struct batt_client_registration_rep *)buf;

		data_ptr->batt_handle = be32_to_cpu(buf_ptr->batt_handle);
		return 0;
	}
}

static int msm_batt_register(u32 desired_batt_voltage,
			     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	struct batt_client_registration_req batt_reg_req;
	struct batt_client_registration_req_4_1 batt_reg_req_4_1;
	struct batt_client_registration_rep batt_reg_rep;
	struct batt_client_registration_rep_4_1 batt_reg_rep_4_1;
	void *request;
	void *reply;
	int rc;

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		batt_reg_req_4_1.desired_batt_voltage = desired_batt_voltage;
		batt_reg_req_4_1.voltage_direction = voltage_direction;
		batt_reg_req_4_1.batt_cb_id = batt_cb_id;
		batt_reg_req_4_1.cb_data = cb_data;
		batt_reg_req_4_1.batt_error = 1;
		request = &batt_reg_req_4_1;
	} else {
		batt_reg_req.desired_batt_voltage = desired_batt_voltage;
		batt_reg_req.voltage_direction = voltage_direction;
		batt_reg_req.batt_cb_id = batt_cb_id;
		batt_reg_req.cb_data = cb_data;
		batt_reg_req.more_data = 1;
		batt_reg_req.batt_error = 0;
		request = &batt_reg_req;
	}

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1)
		reply = &batt_reg_rep_4_1;
	else
		reply = &batt_reg_rep;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_REGISTER_PROC,
			msm_batt_register_arg_func, request,
			msm_batt_register_ret_func, reply,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt register. rc=%d\n", __func__, rc);
		return rc;
	}

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		if (batt_reg_rep_4_1.more_data != 0
			&& batt_reg_rep_4_1.err
				!= BATTERY_REGISTRATION_SUCCESSFUL) {
			pr_err("%s: vBatt Registration Failed proc_num=%d\n"
					, __func__, BATTERY_REGISTER_PROC);
			return -EIO;
		}
		msm_batt_info.batt_handle = batt_reg_rep_4_1.batt_handle;
	} else
		msm_batt_info.batt_handle = batt_reg_rep.batt_handle;

	return 0;
}

struct batt_client_deregister_req {
	u32 batt_handle;
};

struct batt_client_deregister_rep {
	u32 batt_error;
};

static int msm_batt_deregister_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_deregister_req *deregister_req =
		(struct  batt_client_deregister_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(deregister_req->batt_handle);
	size += sizeof(u32);

	return size;
}

static int msm_batt_deregister_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_deregister_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_client_deregister_rep *)data;
	buf_ptr = (struct batt_client_deregister_rep *)buf;

	data_ptr->batt_error = be32_to_cpu(buf_ptr->batt_error);

	return 0;
}

static int msm_batt_deregister(u32 batt_handle)
{
	int rc;
	struct batt_client_deregister_req req;
	struct batt_client_deregister_rep rep;

	req.batt_handle = batt_handle;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_DEREGISTER_CLIENT_PROC,
			msm_batt_deregister_arg_func, &req,
			msm_batt_deregister_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt deregister. rc=%d\n", __func__, rc);
		return rc;
	}

	if (rep.batt_error != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: vbatt deregistration FAIL. error=%d, handle=%d\n",
		       __func__, rep.batt_error, batt_handle);
		return -EIO;
	}

	return 0;
}

static int msm_batt_cleanup(void)
{
	int rc = 0;

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {

		rc = msm_batt_deregister(msm_batt_info.batt_handle);
		if (rc < 0)
			pr_err("%s: FAIL: msm_batt_deregister. rc=%d\n",
			       __func__, rc);
	}

	msm_batt_info.batt_handle = INVALID_BATT_HANDLE;

	if (msm_batt_info.batt_client)
		msm_rpc_unregister_client(msm_batt_info.batt_client);

	if (msm_batt_info.msm_psy_ac)
		power_supply_unregister(msm_batt_info.msm_psy_ac);

	if (msm_batt_info.msm_psy_usb)
		power_supply_unregister(msm_batt_info.msm_psy_usb);
	if (msm_batt_info.msm_psy_batt)
		power_supply_unregister(msm_batt_info.msm_psy_batt);

	if (msm_batt_info.chg_ep) {
		rc = msm_rpc_close(msm_batt_info.chg_ep);
		if (rc < 0) {
			pr_err("%s: FAIL. msm_rpc_close(chg_ep). rc=%d\n",
			       __func__, rc);
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (msm_batt_info.early_suspend.suspend == msm_batt_early_suspend)
		unregister_early_suspend(&msm_batt_info.early_suspend);
#endif

	return rc;
}

int msm_batt_get_charger_api_version(void)
{
	int rc ;
	struct rpc_reply_hdr *reply;

	struct rpc_req_chg_api_ver {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_chg_api_ver;

	struct rpc_rep_chg_api_ver {
		struct rpc_reply_hdr hdr;
		u32 num_of_chg_api_versions;
		u32 *chg_api_versions;
	};

	u32 num_of_versions;

	struct rpc_rep_chg_api_ver *rep_chg_api_ver;


	req_chg_api_ver.more_data = cpu_to_be32(1);

	msm_rpc_setup_req(&req_chg_api_ver.hdr, CHG_RPC_PROG, CHG_RPC_VER_1_1,
			  ONCRPC_CHARGER_API_VERSIONS_PROC);

	rc = msm_rpc_write(msm_batt_info.chg_ep, &req_chg_api_ver,
			sizeof(req_chg_api_ver));
	if (rc < 0) {
		pr_err("%s: FAIL: msm_rpc_write. proc=0x%08x, rc=%d\n",
		       __func__, ONCRPC_CHARGER_API_VERSIONS_PROC, rc);
		return rc;
	}

	for (;;) {
		rc = msm_rpc_read(msm_batt_info.chg_ep, (void *) &reply, -1,
				BATT_RPC_TIMEOUT);
		if (rc < 0)
			return rc;
		if (rc < RPC_REQ_REPLY_COMMON_HEADER_SIZE) {
			pr_err("%s: LENGTH ERR: msm_rpc_read. rc=%d (<%d)\n",
			       __func__, rc, RPC_REQ_REPLY_COMMON_HEADER_SIZE);

			rc = -EIO;
			break;
		}
		/* we should not get RPC REQ or call packets -- ignore them */
		if (reply->type == RPC_TYPE_REQ) {
			pr_err("%s: TYPE ERR: type=%d (!=%d)\n",
			       __func__, reply->type, RPC_TYPE_REQ);
			kfree(reply);
			continue;
		}

		/* If an earlier call timed out, we could get the (no
		 * longer wanted) reply for it.	 Ignore replies that
		 * we don't expect
		 */
		if (reply->xid != req_chg_api_ver.hdr.xid) {
			pr_err("%s: XID ERR: xid=%d (!=%d)\n", __func__,
			       reply->xid, req_chg_api_ver.hdr.xid);
			kfree(reply);
			continue;
		}
		if (reply->reply_stat != RPCMSG_REPLYSTAT_ACCEPTED) {
			rc = -EPERM;
			break;
		}
		if (reply->data.acc_hdr.accept_stat !=
				RPC_ACCEPTSTAT_SUCCESS) {
			rc = -EINVAL;
			break;
		}

		rep_chg_api_ver = (struct rpc_rep_chg_api_ver *)reply;

		num_of_versions =
			be32_to_cpu(rep_chg_api_ver->num_of_chg_api_versions);

		rep_chg_api_ver->chg_api_versions =  (u32 *)
			((u8 *) reply + sizeof(struct rpc_reply_hdr) +
			sizeof(rep_chg_api_ver->num_of_chg_api_versions));

		rc = be32_to_cpu(
			rep_chg_api_ver->chg_api_versions[num_of_versions - 1]);

		pr_debug("%s: num_of_chg_api_versions = %u. "
			"The chg api version = 0x%08x\n", __func__,
			num_of_versions, rc);
		break;
	}
	kfree(reply);
	return rc;
}

static int msm_batt_cb_func(struct msm_rpc_client *client,
			    void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *req;
	u32 procedure;
	u32 accept_status;

	req = (struct rpc_request_hdr *)buffer;
	procedure = be32_to_cpu(req->procedure);

	switch (procedure) {
	case BATTERY_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;

	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		pr_err("%s: ERROR. procedure (%d) not supported\n",
		       __func__, procedure);
		break;
	}

	msm_rpc_start_accepted_reply(msm_batt_info.batt_client,
			be32_to_cpu(req->xid), accept_status);

	rc = msm_rpc_send_accepted_reply(msm_batt_info.batt_client, 0);
	if (rc)
		pr_err("%s: FAIL: sending reply. rc=%d\n", __func__, rc);

	if (accept_status == RPC_ACCEPTSTAT_SUCCESS)
		msm_batt_update_psy_status();

	return rc;
}

static int __devinit msm_batt_probe(struct platform_device *pdev)
{
	int rc;
	struct msm_psy_batt_pdata *pdata = pdev->dev.platform_data;

	if (pdev->id != -1) {
		dev_err(&pdev->dev,
			"%s: MSM chipsets Can only support one"
			" battery ", __func__);
		return -EINVAL;
	}

	if (pdata->avail_chg_sources & AC_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_ac);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_ac = &msm_psy_ac;
		msm_batt_info.avail_chg_sources |= AC_CHG;
	}

	if (pdata->avail_chg_sources & USB_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_usb);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_usb = &msm_psy_usb;
		msm_batt_info.avail_chg_sources |= USB_CHG;
	}

	if (!msm_batt_info.msm_psy_ac && !msm_batt_info.msm_psy_usb) {

		dev_err(&pdev->dev,
			"%s: No external Power supply(AC or USB)"
			"is avilable\n", __func__);
		msm_batt_cleanup();
		return -ENODEV;
	}

	msm_batt_info.voltage_max_design = pdata->voltage_max_design;
	msm_batt_info.voltage_min_design = pdata->voltage_min_design;
	msm_batt_info.batt_technology = pdata->batt_technology;
	msm_batt_info.calculate_capacity = pdata->calculate_capacity;

	if (!msm_batt_info.voltage_min_design)
		msm_batt_info.voltage_min_design = BATTERY_LOW;
	if (!msm_batt_info.voltage_max_design)
		msm_batt_info.voltage_max_design = BATTERY_HIGH;

	if (msm_batt_info.batt_technology == POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
		msm_batt_info.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION;

	rc = power_supply_register(&pdev->dev, &msm_psy_batt);
	if (rc < 0) {
		dev_err(&pdev->dev, "%s: power_supply_register failed"
			" rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}
	msm_batt_info.msm_psy_batt = &msm_psy_batt;

	rc = msm_batt_register(BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_register failed rc = %d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	rc =  msm_batt_enable_filter(VBATT_FILTER);

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_enable_filter failed rc = %d\n",
			__func__, rc);
		msm_batt_cleanup();
		return rc;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	msm_batt_info.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	msm_batt_info.early_suspend.suspend = msm_batt_early_suspend;
	msm_batt_info.early_suspend.resume = msm_batt_late_resume;
	register_early_suspend(&msm_batt_info.early_suspend);
#endif

	msm_batt_update_psy_status();

	return 0;
}

static int __devexit msm_batt_remove(struct platform_device *pdev)
{
	int rc;
	rc = msm_batt_cleanup();

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_cleanup  failed rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static struct platform_driver msm_batt_driver = {
	.probe = msm_batt_probe,
	.remove = __devexit_p(msm_batt_remove),
	.driver = {
		   .name = "msm-battery",
		   .owner = THIS_MODULE,
		   .pm = &msm_batt_pm_ops,
		   },
};

static int __devinit msm_batt_init_rpc(void)
{
	int rc;

	msm_batt_info.chg_ep =
		msm_rpc_connect_compatible(CHG_RPC_PROG, CHG_RPC_VER_4_1, 0);
	msm_batt_info.chg_api_version =  CHG_RPC_VER_4_1;
	if (msm_batt_info.chg_ep == NULL) {
		pr_err("%s: rpc connect CHG_RPC_PROG = NULL\n", __func__);
		return -ENODEV;
	}

	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_3_1, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_3_1;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_1_1, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_1_1;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_1_3, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_1_3;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_2_2, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_2_2;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		rc = PTR_ERR(msm_batt_info.chg_ep);
		pr_err("%s: FAIL: rpc connect for CHG_RPC_PROG. rc=%d\n",
		       __func__, rc);
		msm_batt_info.chg_ep = NULL;
		return rc;
	}

	/* Get the real 1.x version */
	if (msm_batt_info.chg_api_version == CHG_RPC_VER_1_1)
		msm_batt_info.chg_api_version =
			msm_batt_get_charger_api_version();

	/* Fall back to 1.1 for default */
	if (msm_batt_info.chg_api_version < 0)
		msm_batt_info.chg_api_version = CHG_RPC_VER_1_1;
	msm_batt_info.batt_api_version =  BATTERY_RPC_VER_4_1;

	msm_batt_info.batt_client =
		msm_rpc_register_client("battery", BATTERY_RPC_PROG,
					BATTERY_RPC_VER_4_1,
					1, msm_batt_cb_func);

	if (msm_batt_info.batt_client == NULL) {
		pr_err("%s: FAIL: rpc_register_client. batt_client=NULL\n",
		       __func__);
		return -ENODEV;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_1_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_1_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_2_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_2_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_5_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_5_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		rc = PTR_ERR(msm_batt_info.batt_client);
		pr_err("%s: ERROR: rpc_register_client: rc = %d\n ",
		       __func__, rc);
		msm_batt_info.batt_client = NULL;
		return rc;
	}

	rc = platform_driver_register(&msm_batt_driver);

	if (rc < 0)
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		       __func__, rc);

	return rc;
}

static int is2_smbus_read_byte(struct i2c_client *client, int reg,u8 *val)
{
	s32 ret;
	int count = 0;

       do
       {
	      ret = i2c_smbus_read_byte_data(client, reg);
	      count++;
       }while(ret < 0 && count<2);

	if (ret < 0) {
		dev_err(&client->dev, "i2c read fail: can't read from %02x: %d\n", reg, ret);
		*val = 0;
		return 1;
	} else
		*val = ret;

	return 0;
}

static int is2_smbus_write_byte(struct i2c_client *client, int reg,u8 val)
{
	s32 ret;
	int count = 0;

       do
       {
	     ret = i2c_smbus_write_byte_data(client, reg, val);
	     count++;
       }while(ret < 0 && count<2);

	if (ret < 0) {
		dev_err(&client->dev,"i2c write fail: can't write %02x to %02x: %d\n",val, reg, ret);
		return 1;
	}

	return 0;
}

static int i2c_transfer_write_16_byte(struct i2c_client *client, u8 *data, u16 len)
{
       struct i2c_msg msg;
	int ret;
	int count = 0;

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = len;
	msg.buf   = data;

       do
       {
	     ret = i2c_transfer(client->adapter, &msg, 1);
	     count++;
       }while(ret < 0 && count<2);

	if (ret >= 0)
           ret = 0;
	else
	{
	    dev_err(&client->dev,"i2c write fail: can't write, ret = %d\n",ret);
	    ret = 1;
	}

	return ret;
}

static int i2c_transfer_write_word(struct i2c_client *client,u8 addr, u8 msb, u8 lsb)
{
       struct i2c_msg msg;
	int ret;
	char data[3];
	int count = 0;

       data[0] = (char)addr;
	data[1] = (char)msb;
	data[2] = (char)lsb;

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 3;
	msg.buf   = (u8 *)&data[0];

       do
       {
	     ret = i2c_transfer(client->adapter, &msg, 1);
	     count++;
       }while(ret < 0 && count<2);

	if (ret >= 0)
           ret = 0;
	else
	{
	    dev_err(&client->dev,"i2c write fail: can't write, ret = %d\n",ret);
	    ret = 1;
	}

	return ret;
}

static int i2c_transfer_read_word(struct i2c_client *client,u8 addr, u8 *msb, u8 *lsb)
{
       u8 regAddr = addr;
	u8 rxData[2] = {0,0};
	int ret;
	int count = 0;

       struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &regAddr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 2,
			.buf = rxData,
		},
	};

       do
       {
	     ret = i2c_transfer(client->adapter, msgs, 2);
	     count++;
       }while(ret < 0 && count<2);

	if (ret >= 0)
	{
           ret = 0;
	    *msb = rxData[0];
           *lsb = rxData[1];
	}
	else
	{
	    dev_err(&client->dev,"i2c read fail: can't read, ret = %d\n",ret);
	    ret = 1;
	    *msb = 0;
	    *lsb = 0;
	}

	return ret;
}

static void set_led_red_enable(int enable)
{
      if (enable == 0)
      {
           pm8058_set_led_current(PMIC8058_ID_LED_0, 0);
           pm8058_set_led_current(PMIC8058_ID_LED_1, 0);
	    pm8058_set_led_current(PMIC8058_ID_LED_2, 0);
      }
      else
      {
           pm8058_set_led_current(PMIC8058_ID_LED_0, 2);
           pm8058_set_led_current(PMIC8058_ID_LED_1, 0);
	    pm8058_set_led_current(PMIC8058_ID_LED_2, 0);
      }
}

static int max17040gt_initial_voltage(struct i2c_client *client)
{
      int ret;
      u8 msb;
      u8 lsb;
      u16 vcell1;
      u16 vcell2;
      u16 desired_OCV;
      u16 OCV;

      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << initial_voltage - start >>\n");

      //s1 - Read First VCELL Sample
      ret  = i2c_transfer_read_word(client,MAX17040GT_VCELL_MSB,&msb,&lsb);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Read vcell1  fail.\n");
           goto out;
      }
      vcell1 = ((msb << 8) + lsb);
      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] read OCV = %d\n",vcell1);

      //S2 - Delay 125ms
      mdelay(125);

      //s3 - Read (2) VCELL Sample
      ret  = i2c_transfer_read_word(client,MAX17040GT_VCELL_MSB,&msb,&lsb);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Read vcell2  fail.\n");
           goto out;
      }
      vcell2 = ((msb << 8) + lsb);
      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] read OCV = %d\n",vcell2);

      //s4 - Unlock Model Access
      ret = i2c_transfer_write_word(client,MAX17040GT_UNLOCK_MODEL_MSB,0x4A,0x57);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Unlock Model Access  fail.\n");
           goto out;
      }

      //s5 - Read OCV
      ret  = i2c_transfer_read_word(client,MAX17040GT_OCV_MSB,&msb,&lsb);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Read OCV  fail.\n");
           goto out;
      }
      OCV = ((msb << 8) + lsb);
      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] read OCV = %d\n",OCV);

      //s6 - Determine maximum value of VCELL1, VCELL2, and OCV
      if ((vcell1 > vcell2) && (vcell1 > OCV))
      {
           desired_OCV = vcell1;
      }
      else if ((vcell2 > vcell1) && (vcell2 > OCV))
      {
           desired_OCV = vcell2;
      }
      else
      {
           desired_OCV = OCV;
      }

      //S7 - Write OCV
      ret = i2c_transfer_write_word(client,MAX17040GT_OCV_MSB,(desired_OCV >> 8),(desired_OCV & 0xFF));
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Write OCV  fail.\n");
           goto out;
      }

      //S8 - Lock Model Access
      ret = i2c_transfer_write_word(client,MAX17040GT_UNLOCK_MODEL_MSB,0x00,0x00);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Lock Model Access  fail.\n");
           goto out;
      }

      //S9 - Delay 125ms
      mdelay(125);

      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << initial_voltage - end >>\n");

      return 0;
	
out:
	return -1;
}

static int max17040gt_update_RCOMP(struct i2c_client *client,int temp)
{
      int ret;
      int INI_RCOMP = max17040gt_INI_RCOMP0;
      int TempCoHot = max17040gt_INI_TempCoUp_mul100;
      int TempCoCold = max17040gt_INI_TempCoDown_mul100;
      int TempUnit = max17040gt_INI_TempUnit;
      int NewRCOMP;

      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << update_RCOMP - start >>\n");

      if (temp > 20)
      {
          NewRCOMP = INI_RCOMP + ((temp - 20) * TempCoHot / TempUnit);
	   DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] cal(1) NewRCOMP=%d\n",NewRCOMP);
      }
      else if (temp <  20)
      {
          NewRCOMP = INI_RCOMP + ((temp - 20) * TempCoCold / TempUnit);
	   DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] cal(2) NewRCOMP=%d\n",NewRCOMP);
      }
      else
      {
          NewRCOMP = INI_RCOMP;
	   DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] cal(3) NewRCOMP=%d\n",NewRCOMP);
      }

      if (NewRCOMP > 0xFF)
      {
	    NewRCOMP = 0xFF;
      }
      else if (NewRCOMP < 0)
      {
           NewRCOMP = 0;
      }

      printk(KERN_ERR "[IS3_POWER][update_RCOMP] temp=%d, NewRCOMP=0x%x\n", temp, NewRCOMP);
      ret = i2c_transfer_write_word(client,MAX17040GT_RCOMP_MSB,(u8)NewRCOMP,0x00);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] restore config  (msb)  fail.\n");
	    goto out;
      }

      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << update_RCOMP - end >>\n");

      return 0;

out:
      return -1;
}

static int max17040gt_load_custom_model(struct i2c_client *client,int mode)
{
      int ret;
      u8 msb;
      u8 lsb;
      int i;
      int k;
      u8 mStartAddr;
      u8 original_OCV_1;
      u8 original_OCV_2;
      u8 data[17];
      int load_fail = 0;


      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << load_custom_model - start >>\n");
      i = 0;
      do
      {
          //s1 - Unlock Model Access
	   ret = i2c_transfer_write_word(client,MAX17040GT_UNLOCK_MODEL_MSB,0x4A,0x57);
	   if(ret != 0)
	   {
	        DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Unlock Model Access fail.\n");
	        goto out;
	   }

	   //s2 - Read OCV
	   ret  = i2c_transfer_read_word(client,MAX17040GT_OCV_MSB,&msb,&lsb);
          if(ret != 0)
	   {
	        DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Read OCV fail.\n");
	        goto out;
	   }
	   DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] original_ocv_1=0x%x, original_ocv_2=0x%x.\n",msb,lsb);
          i++;
      }while(msb==0xFF && lsb==0xFF && i < 20);

      original_OCV_1 = msb;
      original_OCV_2 = lsb;
      if (msb==0xFF && lsb==0xFF)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Unlock Model Access fail.\n");
           goto out;
      }

     // i = 0;
     //do
     //{
         //s3 - Write OCV
         ret = i2c_transfer_write_word(client,MAX17040GT_OCV_MSB,
                                                                  (max17040gt_INI_OCVTest >> 8),
                                                                  (max17040gt_INI_OCVTest & 0xFF));
         if(ret != 0)
         {
             DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Write OCV fail.\n");
	      goto out2;
         }

         //s4 - Write RCOMP to its Maximum Value
         ret = i2c_transfer_write_word(client,MAX17040GT_RCOMP_MSB,0xFF,0x00);
         if(ret != 0)
         {
              DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Write RCOMP to its Maximum Value fail.\n");
	       goto out2;
         }

         //s5 - Write the Model
         mStartAddr = MAX17040GT_START_MODEL;
	  if (mode == 0)
	  {
              for (i=0;i<64;i+=16)
              {
	            data[0] = (u8)mStartAddr;
	            DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] write => Model_data[0x%x]\n",mStartAddr);
                   for (k=0;k<16;k++)
                   {
                          data[k+1] = max17040gt_model_data[i+k];
			     DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] (%d) 0x%x-0x%x\n",i+k,mStartAddr+k,data[k+1]);
                   }
                   ret = i2c_transfer_write_16_byte(client,data,17);
	            if(ret != 0)
                   {
                       DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] write => fail\n");
	                goto out2;
	            }
		     mStartAddr += 16;
              }
         }
	  else
	  {
              for (i=0;i<64;i+=2)
              {
	            DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] write => Model_data[0x%x][0x%x,0x%x] .......\n",
                                                                               mStartAddr,
                                                                               max17040gt_model_data[i],
                                                                               max17040gt_model_data[i+1]);
                   ret = i2c_transfer_write_word(client,mStartAddr,max17040gt_model_data[i],max17040gt_model_data[i+1]);
	            if(ret != 0)
                   {
                       DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] write => fail\n");
	                goto out2;
                   }

		     mStartAddr+=2;
              }
	  }
         //s6 - Delay at least 150ms
         mdelay(150);

         //s7 - Write OCV
         ret = i2c_transfer_write_word(client,MAX17040GT_OCV_MSB,
                                                                       (max17040gt_INI_OCVTest >> 8),
                                                                       (max17040gt_INI_OCVTest & 0xFF));
         if(ret != 0)
         {
              DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Write INI_OCVTest fail.\n");
	       goto out2;
         }
         //s8 - Delay between 150ms and 600ms
         mdelay(300);

         //s9 - Read SOC Reqister and Compare to expected result
         ret  = i2c_transfer_read_word(client,MAX17040GT_SOC_MSB,&msb,&lsb);
         if(ret != 0)
         {
              DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Read SOC fail.\n");
	       goto out2;
         }
         DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] soc_1=0x%x,%d.\n",msb,msb);

      //   i++;
      //}while((msb < max17040gt_INI_SOCCheckA ||
      //                            msb > max17040gt_INI_SOCCheckB) &&
      //                            i < 2);

      if (msb >= max17040gt_INI_SOCCheckA && msb <= max17040gt_INI_SOCCheckB)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] model was loaded successfully\n");
      }
      else
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] model was not loaded successfully\n");
	    load_fail = 1;
      }

      //s10 - Restore CONFIG and OCV
      ret = i2c_transfer_write_word(client,MAX17040GT_RCOMP_MSB,max17040gt_INI_RCOMP0,0x00);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Restore INI_RCOMP0 fail.\n");
	    goto out2;
      }

      ret = i2c_transfer_write_word(client,MAX17040GT_OCV_MSB,original_OCV_1,original_OCV_2);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] restore original_OCV fail.\n");
           goto out2;
      }

out2:
      //s11 - Lock Mode Access
      ret = i2c_transfer_write_word(client,MAX17040GT_UNLOCK_MODEL_MSB,0x00,0x00);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Lock Mode Access fail.\n");
           goto out;
      }
      mdelay(150);

      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << load_custom_model - end >>\n");

      if (load_fail == 1)
	  return -1;

      return 0;

out:
      return -1;
}

static int max17040gt_verify_model(struct i2c_client *client,int type)
{
      int verify_ok = 1;
      int ret;
      u8 msb;
      u8 lsb;
      u16 RCOMP0;
      u16 OriginalOCV;
      u16 INI_OCVTest;
      u8 INI_SOCCheckA;
      u8 INI_SOCCheckB;

#if USE_CHARGING_4350MV_BATTERY_PACK
      if (type == 1)
      {
	  INI_OCVTest = max17040gt_INI_OCVTest_4200;
	  INI_SOCCheckA = max17040gt_INI_SOCCheckA_4200;
	  INI_SOCCheckB = max17040gt_INI_SOCCheckB_4200;
      }
      else
      {
	  INI_OCVTest = max17040gt_INI_OCVTest_4350;
	  INI_SOCCheckA = max17040gt_INI_SOCCheckA_4350;
	  INI_SOCCheckB = max17040gt_INI_SOCCheckB_4350;
      }
#else
      INI_OCVTest = max17040gt_INI_OCVTest;
      INI_SOCCheckA = max17040gt_INI_SOCCheckA;
      INI_SOCCheckB = max17040gt_INI_SOCCheckB;
#endif

      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << verify_model - start >>\n");

      //Unlock Model Access
      ret = i2c_transfer_write_word(client,MAX17040GT_UNLOCK_MODEL_MSB,0x4A,0x57);
      if(ret != 0)
      {
	     DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] unlock mode access fail.\n");
	     goto out;
      }

      //Read Original RCOMP
      ret  = i2c_transfer_read_word(client,MAX17040GT_RCOMP_MSB,&msb,&lsb);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] read Original RCOMP fail.\n");
	    goto out2;
      }
      RCOMP0 = (msb << 8) + lsb;
      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] RCOMP0 = 0x%x\n",RCOMP0);

      //Read Original OCV Register
      ret  = i2c_transfer_read_word(client,MAX17040GT_OCV_MSB,&msb,&lsb);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] read Original OCV fail.\n");
	    goto out2;
      }
      OriginalOCV = (msb << 8) + lsb;
      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] OriginalOCV = 0x%x\n",OriginalOCV);

      //Write OCV Register (OCVTest)
      ret = i2c_transfer_write_word(client,MAX17040GT_OCV_MSB,
                                                                 (INI_OCVTest >> 8),
                                                                 (INI_OCVTest && 0xFF));
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Write OCV Register (OCVTest) fail.\n");
	    goto out2;
      }

      //Write RCOMP Register
      ret = i2c_transfer_write_word(client,MAX17040GT_RCOMP_MSB,
                                                                 (RCOMP0 >> 8),
                                                                 (RCOMP0 & 0xFF));
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Write RCOMP fail.\n");
	    goto out2;
      }

      mdelay(150);

      //Read SOC Reqister and Compare to expected result
      ret  = i2c_transfer_read_word(client,MAX17040GT_SOC_MSB,&msb,&lsb);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] read SOC fail.\n");
	    goto out2;
      }
      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] soc_1=0x%x.\n",msb);
      if (msb >= INI_SOCCheckA && msb <= INI_SOCCheckB)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] verify_model_ready successfully\n");
	    verify_ok = 1;
      }
      else
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] verify_model_ready fail !!!\n");
           verify_ok = 0;
      }
      printk(KERN_ERR "[IS3_POWER][verify_model] Read soc_1=%d, OCVTest=%d, SOCCheckA=%d, SOCCheckB=%d",
                    msb,INI_OCVTest,INI_SOCCheckA,INI_SOCCheckB);

      //Write RCOMP Register
      ret = i2c_transfer_write_word(client,MAX17040GT_RCOMP_MSB,
                                                                 (RCOMP0 >> 8),
                                                                 (RCOMP0 & 0xFF));
      if(ret != 0)
      {
            DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Write RCOMP fail.\n");
            goto out2;
      }

      //Write OriginalOCV Register
      ret = i2c_transfer_write_word(client,MAX17040GT_OCV_MSB,
                                                                 (OriginalOCV >> 8),
                                                                 (OriginalOCV & 0xFF));
      if(ret != 0)
      {
            DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Write OriginalOCV fail.\n");
            goto out2;
      }

out2:
      //Lock Mode Access
      ret = i2c_transfer_write_word(client,MAX17040GT_UNLOCK_MODEL_MSB,0x00,0x00);
      if(ret != 0)
      {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] Lock Mode Access fail.\n");
           goto out;
      }

      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << verify_model - end >>\n");

      if (verify_ok == 0)
      {
           goto out;
      }

      return 0;

out:
      return -1;
}

static void max17040gt_quick_start(struct i2c_client *client)
{
       int ret;

	ret = i2c_transfer_write_word(client,MAX17040GT_MODE_MSB,0x40,0x00);
       if(ret != 0)
       {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] max17040gt_quick_start(ret =%d) fail.\n",ret);
           return;
       }

	mdelay(MAX17040GT_RESET_WAIT_MS);
}

static void max17040gt_reset(struct i2c_client *client)
{
       int ret;

	ret = i2c_transfer_write_word(client,MAX17040GT_CMD_MSB,0x54,0x00);
       if(ret != 0)
       {
           DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] max17040gt_reset(ret =%d) fail.\n",ret);
           return;
       }

	mdelay(MAX17040GT_RESET_WAIT_MS);
}

static void max17040gt_get_vcell(struct i2c_client *client)
{
	struct max17040gt_chip *chip_data = i2c_get_clientdata(client);
	u8 msb;
	u8 lsb;
	int ret;

       ret = i2c_transfer_read_word(client,MAX17040GT_VCELL_MSB,&msb,&lsb);
	if(ret == 0)
	{
	       chip_data->vcell = ((msb << 4) + (lsb >> 4)) * 125 / 100;
	       DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] get_vcell = %x,%x,%d,%d\n",
                                                      msb, lsb, ((msb << 4) + (lsb >> 4)), chip_data->vcell);
              return;
	}

	chip_data->vcell = 0;
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] get_vcell fail!!\n");
}

static int max17040gt_get_soc(struct i2c_client *client)
{
	struct max17040gt_chip *chip_data = i2c_get_clientdata(client);
	u8 msb;
	u8 lsb;
	int ret;

	ret = i2c_transfer_read_word(client,MAX17040GT_SOC_MSB,&msb,&lsb);
	if(ret == 0)
	{
	      if (max17040gt_INI_bit == 19)
	      {
	          chip_data->soc = ((msb << 8) + lsb) / 512;
		   DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] get_soc(cal1) = %x,%x,%d\n", msb, lsb, chip_data->soc);
	      }
	      else if (max17040gt_INI_bit == 18)
	      {
	          chip_data->soc = ((msb << 8) + lsb) / 256;
		   DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] get_soc(cal2) = %x,%x,%d\n", msb, lsb, chip_data->soc);
	      }
	      else
	      {
	          chip_data->soc = msb;
		   DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] get_soc(cal3) = %x,%x,%d\n", msb, lsb, chip_data->soc);
	      }


#if USE_CHARGING_4350MV_BATTERY_PACK
             if (vbatt_max_voltage_mode == 1)
	      {
	          chip_data->soc = chip_data->soc * 10 / 9;
             }
#endif

             return 1;
	}

	chip_data->soc = 0;

	DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] get_soc fail!!\n");

	return 0;
}

static int max17040gt_get_version(struct i2c_client *client)
{
	u8 msb;
	u8 lsb;
	int ret;

       ret = i2c_transfer_read_word(client,MAX17040GT_VER_MSB,&msb,&lsb);
	if(ret == 0)
	{
	      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] get_version = %x,%x\n", msb, lsb);
	}
       else
       {
	      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] get_version fail!!\n");
       }
	return ret;
}

static int max17040gt_get_RCOMP(struct i2c_client *client,u8 *msb,u8 *lsb)
{
	u8 msb_tmp;
	u8 lsb_tmp;
	int ret;

	ret = i2c_transfer_read_word(client,MAX17040GT_RCOMP_MSB,&msb_tmp,&lsb_tmp);
	if(ret == 0)
	{
	      *msb = msb_tmp;
	      *lsb = lsb_tmp;
	      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] get_RCOMP = %x,%x\n", msb_tmp, lsb_tmp);
	}
       else
       {
             *msb =0x97;
	      *lsb = 0x00;
	      DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] get_RCOMP fail!!\n");
       }
	return ret;
}

static u32 calculate_adc_capacity(u32 voltage)
{
       s32 capacity = 100;

       AdcMapLinearInt32toInt32(
                    adcMap_capacity,
                    sizeof(adcMap_capacity)/sizeof(adcMap_capacity[0]),
                    voltage,
                    &capacity,
                    0);

#if USE_CHARGING_4350MV_BATTERY_PACK
       if (vbatt_max_voltage_mode == 1)
       {
           capacity = capacity * 10 / 9;
       }
#endif

	return (u32)capacity;
}

void set_batt_status_not_charging(void)
{
     if (check_cradle_is_connect() || check_vbus_is_connect())
     {
           is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
           is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG, SMB329B_PIN_CTRL_REG_CHG_OFF);
           msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	    DBG_BUILD("[IS3_POWER_DEBUG-1] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
     }
     else
     {
           smb329b_stop_charging();
     }
}

void i2c_bus_check(void)
{
     u8 reg36_temp = 0;

     if (max17040gt_ready == CHIP_STATUS_INIT_OK)
     {
	  if (max17040gt_get_version(max17040gt_chip->client)  == 0)
         {
              max17040gt_ready = CHIP_STATUS_INIT_OK;
	  }
     }
     if (smb329b_ready != CHIP_STATUS_INIT_OK)
     {
         if (is2_smbus_read_byte(smb329b_chip->client, SMB329B_STATUS_B_REG, &reg36_temp) == 0)
         {
             smb329b_ready = CHIP_STATUS_INIT_OK;
         }
     }
}

static int first_ovp_check = 0;
static int check_ovp_again = 0;
void batt_chg_status_update(struct work_struct *work)
{
	struct max17040gt_chip *chip_data;
	u32 battery_voltage = msm_batt_info.battery_voltage;
	u32 battery_temp = msm_batt_info.battery_temp;
	u32 battery_temp_raw = msm_batt_info.battery_temp_raw;
	u32 battery_capacity = msm_batt_info.batt_capacity;
	u32 battery_conn = msm_batt_info.battery_conn;
	u32 battery_cradle_mv = msm_batt_info.battery_cradle_mv;
	u32 battery_vbus_mv = msm_batt_info.battery_vbus_mv;
	u32 battery_adc_voltage = 0;
	struct msm_batt_get_val_ret_data vbatt_data;
	int tmp;
	int tmp_max;
	int tmp_min;
	int ret = 0;
	u8 reg00_temp = 0;
	u8 reg02_temp = 0;
	u8 reg36_temp = 0;
	u8 reg37_temp = 0;
	u8 call_re_charge = 0;
	int status = 1;
	int check_chg_error = 0;
	int dmesg_smb329b_info = 0;
	int rechg_status;
	int call_retry;
	int call_update_u_event = 0;
	int update_batt_temp_to_gague_ic = 0;

       batt_work_on = 1;
       {
          chip_data = container_of(work, struct max17040gt_chip, work.work);

	   if (max17040gt_ready == CHIP_STATUS_INIT_OK)
	   {
	       if (get_batt_first_info == 1)
	       {
                 max17040gt_get_vcell(chip_data->client);
	          status = max17040gt_get_soc(chip_data->client);
	       }
	       battery_voltage = (u32)chip_data->vcell;
	       battery_capacity = (u32)chip_data->soc;
	   }
	   if (max17040gt_ready == CHIP_STATUS_INIT_FAIL ||
              status == 0)
	   {
	       if (status == 0) printk(KERN_ERR "[IS3_POWER] max17040gt read fail, change read ADC\n");
	       battery_info_mode = 0;
		vbatt_data = update_batt_mv_temp_RPC(READ_RPC_MODE,1);
	       battery_voltage = vbatt_data.battery_voltage;
              battery_adc_voltage = vbatt_data.battery_voltage;
	       battery_capacity = calculate_adc_capacity(battery_adc_voltage);
	   }
	   else
	   {
              battery_info_mode = 1;
		vbatt_data = update_batt_mv_temp_RPC(READ_RPC_MODE,0);
	   }
	   battery_temp = vbatt_data.battery_temp;
	   battery_temp_raw = vbatt_data.battery_temp_raw;
	   battery_conn = vbatt_data.battery_conn;
	   battery_cradle_mv = (u32)vbatt_data.battery_cradle_mv;
	   battery_vbus_mv = (u32)vbatt_data.battery_vbus_mv;

          if (msm_batt_info.re_battery_voltage != 0)
          {
              battery_voltage = msm_batt_info.re_battery_voltage;
		msm_batt_info.re_battery_voltage = 0;
		update_batt_temp_to_gague_ic = 1;
          }
	   if (msm_batt_info.re_batt_capacity!= 0)
          {
              battery_capacity = msm_batt_info.re_batt_capacity;
		msm_batt_info.re_batt_capacity = 0;
		update_batt_temp_to_gague_ic = 1;
          }
          battery_capacity = batt_capacity_format_check(battery_capacity);

          i2c_bus_check();


//for debug , start {
	   if (debug_temp_value >= -30 && debug_temp_value <= 65) // for test battery temp
	   {
	        battery_temp = debug_temp_value;
	   }
	   if (debug_capacity_value >= 200 && debug_capacity_value <= 300) // for test batt capacity
	   {
	        battery_capacity = debug_capacity_value - 200;
		 printk(KERN_ERR "[IS3_POWER] set battery_capacity = %d\n",battery_capacity);
	   }
	   if (debug_ovc == 1)  // for test Batt Over voltage
	   {
	        battery_voltage = SW_CHECK_OVC_VOLTAGE + 1;
	        printk(KERN_ERR "[IS3_POWER] set battery_voltage = %d\n",battery_voltage);
	   }
	   else if (debug_ovc == 2)  // for test Batt Over voltage
	   {
	        battery_voltage = SW_CHACK_OVC_RESUME_VOLTAGE - 1;
	        printk(KERN_ERR "[IS3_POWER] set battery_voltage = %d\n",battery_voltage);
	   }
//}end, for debug

          if (battery_conn == 0)
          {
              //chg_flow_path g : thermistor released
              if (msm_batt_info.chg_flow_status_now != CHG_FLOW_STATUS_A)
              {
                 if (lock_batt_status == 0) msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_A;
                 DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_A]);

		   if (lock_batt_status == 0) msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
                 if (lock_batt_status == 0) msm_batt_info.batt_valid = 0;
	          if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
                      msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
	          {
		        smb329b_stop_charging();
	          }
		   if (vbus_connect == 1 || cradle_connect == 1)
		   {
		       printk(KERN_ERR "[IS3_POWER] temp = %d\n",battery_temp);
		       printk(KERN_ERR "[IS3_POWER] temp_raw = %d\n",battery_temp_raw);
		       printk(KERN_ERR "[IS3_POWER] power source connect & battery disconnect !!!!");
		   }
	       }
          }
	   else
	   {
              if (!(msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
                   msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL))
              {
                   //chg_flow_path a : thermistor connected
                   if (msm_batt_info.chg_flow_status_now != CHG_FLOW_STATUS_B)
                   {
                      rechg_status = 1;
                      if (msm_batt_info.chg_flow_status_now == CHG_FLOW_STATUS_A &&
			     (check_vbus_is_connect() == 1 || check_cradle_is_connect() == 1))
                      {
                         printk(KERN_ERR "[IS3_POWER] call smb329b_batt_re_charging(status A -> B)\n");
			    rechg_status = smb329b_batt_re_charging(1,0);
                      }
                      if (lock_batt_status == 0 && rechg_status != 0)
			 {
			    msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_B;
                         DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_B]);
                      }
                   }
              }

		if (lock_batt_status == 0) msm_batt_info.batt_valid = 1;
	       if (battery_capacity == 100 && (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
                                                             msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL))
              {
                  if (float_voltage_mode == 1 && isdb_t_on == 0)
                  {
                      if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING &&
			      smb329b_check_batt_is_full())
                      {
                          if (lock_batt_status == 0)
                          {
                              if (msm_batt_info.chg_flow_status_now != CHG_FLOW_STATUS_O)
                              {
                                  printk(KERN_ERR "[IS3_POWER] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_O]);
                              }
				  msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_O;
                              msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
				  DBG_BUILD("[IS3_POWER_DEBUG-5] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	                   }
                      }
		        else if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
		        {
		            if (lock_batt_status == 0)
		            {
		                if (msm_batt_info.chg_flow_status_now != CHG_FLOW_STATUS_O)
                              {
                                  printk(KERN_ERR "[IS3_POWER] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_O]);
                              }
	                       msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_O;
		            }
		        }
                  }
		    else
		    {
		        if (lock_batt_status == 0)
		        {
		             if (msm_batt_info.chg_flow_status_now != CHG_FLOW_STATUS_N)
		             {
		                 printk(KERN_ERR "[IS3_POWER] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_N]);
				   msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_N;
		             }
			      if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
			      {
			          msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
				   DBG_BUILD("[IS3_POWER_DEBUG-6] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
			      }
		        }
		    }
	       }

             //chg_flow_path n : battery temperature lower than or equal to 0 Celsius,
             //                           higher than or equal to 45 Celsius.
	      if ((msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
                   msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL) &&
                   smb329b_chip->chg_current > 500)
	      {
                  if (data_transmit == 1 &&
			  change_max_current == 0)
		    {
		          printk(KERN_ERR "[IS3_POWER] camera_recording or data_transmit enable, set charging current to 0.5C\n");
		          smb329b_change_max_charging_current(1);
			   change_max_current = 1;
                  }
		    else
		    {
		          DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] camera_recording = %d\n",camera_recording);
		          tmp = (int)battery_temp;
                        if (((tmp <=10 && tmp >=0) || (tmp <=55 && tmp >=45)) &&
			      change_max_current == 0 &&
			      camera_recording == 0)
                        {
                           printk(KERN_ERR "[IS3_POWER] battery temp(%d) 0~10,45~55, set charging current to 0.5C\n",tmp);
		             smb329b_change_max_charging_current(1);
			      change_max_current = 1;
	                 }
		          else if (((tmp > 10 && tmp <45) && change_max_current == 1) ||
                                     (change_max_current == 1 && camera_recording == 1))
		          {
		             printk(KERN_ERR "[IS3_POWER] battery temp(%d)=11~44 or Camera_cording(%d)=1, set charging current to 1A\n",tmp,camera_recording);
                           smb329b_change_max_charging_current(0);
			      change_max_current = 0;
		          }
		    }
		}
	   }

#if MAX17040GT_ENABLE_UPDATE_RCOMP
          if (battery_temp != msm_batt_info.battery_temp)
          {
              update_batt_temp_to_gague_ic = 1;
          }
	   if (update_batt_temp_to_gague_ic == 1)
	   {
              DBG_LIMIT(debug_level,TYPE_MAX17040, "[MAX17040] update_RCOMP(%d,%d,%d,%d)\n",
                                                 (int)battery_temp,msm_batt_info.battery_temp,verify_model_ready,battery_conn);
              if (verify_model_ready == 1 && battery_conn == 1 && max17040gt_ready == CHIP_STATUS_INIT_OK)
              {
                  max17040gt_update_RCOMP(chip_data->client,(int)battery_temp);
              }
          }
#endif

          DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] chg_flow_status = %c, status=%d\n",
                                                                         (char)(msm_batt_info.chg_flow_status_now+0x41),
                                                                         msm_batt_info.batt_status);
	   //chg_flow_path q : battery remain is 99% or less, begin th re-charge
	   if (msm_batt_info.chg_flow_status_now == CHG_FLOW_STATUS_O && battery_conn == 1)
	   {
	        DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] check re-charge (%d,%d)\n",battery_capacity,RE_CHARGE_BATT_CAPACITY);

#if QCI_CHG_SOLUTION
               if (isdb_t_on == 1)
		    RE_CHARGE_BATT_CAPACITY = RE_CHARGE_BATT_CAPACITY_ISDB_T;
		 else if (camera_recording == 1)
		    RE_CHARGE_BATT_CAPACITY = RE_CHARGE_BATT_CAPACITY_CAMERA_CORDING;
		 else
                  RE_CHARGE_BATT_CAPACITY = RE_CHARGE_BATT_CAPACITY_NORMAL;
#else
                if (isdb_t_on == 1)
		    RE_CHARGE_BATT_CAPACITY = RE_CHARGE_BATT_CAPACITY_ISDB_T;
		 else
                  RE_CHARGE_BATT_CAPACITY = RE_CHARGE_BATT_CAPACITY_NORMAL;
#endif

               if (battery_capacity <= (RE_CHARGE_BATT_CAPACITY - 1))
	        {
                    printk(KERN_ERR "[IS3_POWER] call smb329b_batt_re_charging(%d)\n",RE_CHARGE_BATT_CAPACITY);
		      smb329b_batt_re_charging(0,0);
		      call_re_charge = 1;
		      battery_capacity = 100;
	        }
	   }

          if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
              msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
          {
		if (call_re_charge == 0)
              {
                   if (battery_capacity < msm_batt_info.batt_capacity)
                   {
                        if (((battery_capacity < 100 && battery_capacity >= 95) &&
			        msm_batt_info.batt_capacity  - battery_capacity >= 5) ||
			       (battery_capacity < 95 &&
			        msm_batt_info.batt_capacity  - battery_capacity >= 3) ||
			        (isdb_t_on == 1 && battery_capacity <= RE_CHARGE_BATT_CAPACITY_ISDB_T))
                        {
                             DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] new_capacity(%d) < old_capacity(%d) and over 2",battery_capacity,msm_batt_info.batt_capacity);
                             DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] isdb_t_on=%d\n",isdb_t_on);
				 DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] float_voltage_mode=%d\n",float_voltage_mode);
				 if ((isdb_t_on == 0 && float_voltage_mode == 1) ||
				      (isdb_t_on == 0 && float_voltage_mode == 0 && battery_voltage < 4100) ||
				      (isdb_t_on == 1 && battery_capacity <= RE_CHARGE_BATT_CAPACITY_ISDB_T))
				 {
				    printk(KERN_ERR "[IS3_POWER] call smb329b_batt_re_charging(%d,%d)(%d,%d)(%d)\n",
                                              battery_capacity,msm_batt_info.batt_capacity,float_voltage_mode,battery_voltage,isdb_t_on);
		                  if (smb329b_batt_re_charging(1,0) == 0)
		                  {
			              if (battery_capacity < 100)
			              {
			                 if (lock_batt_status == 0) msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
			              }
				       if (msm_batt_info.chg_flow_status_now != CHG_FLOW_STATUS_N)
                                   {
                                       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_N]);
                                   }
                                   if (lock_batt_status == 0) msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_N;
		                  }
			        }
                        }
		          else
		          {
		               battery_capacity = msm_batt_info.batt_capacity;
		          }
                    }
		}

              max17040gt_delay_time = MAX17040GT_DELAY_QUICK;
          }
	   else
	   {
	        if (battery_capacity > msm_batt_info.batt_capacity ||
	            msm_batt_info.batt_capacity - battery_capacity > 20)
	           battery_capacity = msm_batt_info.batt_capacity;

               if (msm_batt_info.batt_capacity <= 15)
                  max17040gt_delay_time = MAX17040GT_DELAY_QUICK;
               else
               {
                  if (msm_batt_info.batt_capacity - battery_capacity >= 2)
                      max17040gt_delay_time = MAX17040GT_DELAY_QUICK;
                  else if (check_vbus_is_connect() == 1 || check_cradle_is_connect() == 1)
		        max17040gt_delay_time = MAX17040GT_DELAY_QUICK;
                  else
                      max17040gt_delay_time = MAX17040GT_DELAY_NORMAL;
               }
	   }

	   if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
	       battery_capacity = 100;
	   battery_capacity = batt_capacity_format_check(battery_capacity);
 
          if ((abs(battery_cradle_mv - msm_batt_info.battery_cradle_mv) > 500 &&
		  battery_cradle_mv > 500) ||
		 msm_batt_info.battery_cradle_mv == 0 ||
		 battery_cradle_mv > 1863)
          {
              printk(KERN_INFO "[PM8058] CRADLE_STATE pin = %d mV\n",battery_cradle_mv);
          }
	   if ((abs(battery_vbus_mv - msm_batt_info.battery_vbus_mv) > 500 &&
                battery_vbus_mv > 500) ||
                msm_batt_info.battery_vbus_mv == 0 ||
		  battery_vbus_mv > 1863)
          {
              printk(KERN_INFO "[PM8058] USB_VBUS_STATE pin = %d mV\n",battery_vbus_mv);
          }

          msm_batt_info.battery_cradle_mv = battery_cradle_mv;
	   msm_batt_info.battery_vbus_mv = battery_vbus_mv;

          if (battery_voltage != msm_batt_info.battery_voltage ||
		 battery_capacity != msm_batt_info.batt_capacity ||
		 battery_temp != msm_batt_info.battery_temp ||
		 battery_temp_raw != msm_batt_info.battery_temp_raw ||
		 battery_conn != msm_batt_info.battery_conn)
          {
             if ( battery_capacity != msm_batt_info.batt_capacity)
             {
                 printk(KERN_INFO "Battery level = %d\n",battery_capacity);
		   if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
                     msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
                 {
		       dmesg_smb329b_info = 1;
		   }
             }
	      if ( battery_temp != msm_batt_info.battery_temp)
             {
                 printk(KERN_INFO "Battery temp = %d,%d\n",battery_temp,battery_temp_raw);
		   if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
                     msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
                 {
		       dmesg_smb329b_info = 1;
		   }
             }
	      if (dmesg_smb329b_info == 1)
	      {
	          is2_smbus_read_byte(smb329b_chip->client, SMB329B_CHG_CURRENT_REG, &reg00_temp);
		   printk(KERN_INFO "smb329b_reg00h = 0x%x\n",reg00_temp);
		   is2_smbus_read_byte(smb329b_chip->client, SMB329B_FLOAT_VOLTAGE_REG, &reg02_temp);
		   printk(KERN_INFO "smb329b_reg02h = 0x%x\n",reg02_temp);
		   is2_smbus_read_byte(smb329b_chip->client, SMB329B_STATUS_B_REG, &reg36_temp);
		   printk(KERN_INFO "smb329b_reg36h = 0x%x\n",reg36_temp);
	      }

             if (lock_batt_status == 0)
	      {
	          msm_batt_info.battery_voltage = battery_voltage;
                 msm_batt_info.batt_capacity = battery_capacity;
                 msm_batt_info.battery_temp = battery_temp;
	          msm_batt_info.battery_temp_raw= battery_temp_raw;
	          msm_batt_info.battery_conn = battery_conn;
             }

             if (temp_work_en == 0  &&
		    (wait_exit_error_work_en == 0 || (wait_exit_error_work_en == 1 && wait_exit_error_status == 0)))
	      {
	         if (msm_batt_info.batt_health != POWER_SUPPLY_HEALTH_DEAD)
	         {
                   tmp = (int)msm_batt_info.battery_temp;
	            DBG_LIMIT(debug_level,TYPE_MAX17040, "[MAX17040] battery temp = %d\n",tmp);
	            if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_DISCHARGING)
	            {
	                // path b : battery temperature lower than or equal to 0 Celsius,
	                //             higher than or equal to 45 Celsius
                       tmp_max = SMB320B_NO_START_CHARGING_BATT_TEMP_UP;
	                tmp_min = SMB320B_NO_START_CHARGING_BATT_TEMP_DOWN;
	            }
	            else
	            {
	                // path d : battery temperature lower than or equal to 0Celsius,
	                //             higher than or equal to 55 Celsius
	                tmp_max = SMB320B_STOP_CHARGING_BATT_TEMP_UP;
	                tmp_min = SMB320B_STOP_CHARGING_BATT_TEMP_DOWN;
                   }
	            if (tmp < tmp_min)
	            {
	                DBG_LIMIT(debug_level,TYPE_MAX17040, "[MAX17040] max17040gt_work - COLD\n");
		          if (lock_batt_status == 0) msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_COLD;
	            }
	            else if (tmp > tmp_max)
	            {
	                DBG_LIMIT(debug_level,TYPE_MAX17040, "[MAX17040] max17040gt_work - HEAT\n");
                       if (lock_batt_status == 0) msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
                   }
	            else
	            {
	                 DBG_LIMIT(debug_level,TYPE_MAX17040, "[MAX17040] max17040gt_work - GOOD\n");
		          if (lock_batt_status == 0) msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
	            }
	         }
	      }
             call_update_u_event = 1;
          }

          if (msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_COLD ||
		 msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_OVERHEAT ||
		 msm_batt_info.pm_temp_msm_over_layer >= 2)
          {
	       if ((msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
                    msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL ) && smb329b_ready == CHIP_STATUS_INIT_OK)
		{
		     if (lock_batt_status == 0)
		     {
		        msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		        if (temp_work_en == 0  &&  wait_exit_error_work_en == 0)
		        {
		            printk(KERN_ERR "[IS3_POWER] battery temp (%d) cold/overheat - resume(%d~%d) !!!\n",
						msm_batt_info.battery_temp,SMB320B_RESUME_CHARGING_BATT_TEMP_DOWN,SMB320B_RESUME_CHARGING_BATT_TEMP_UP);
			     printk(KERN_ERR "[IS3_POWER] temp_msm_over_layer = %d !!!\n",msm_batt_info.pm_temp_msm_over_layer);
                          printk(KERN_ERR "[IS3_POWER] call smb329b_chip->temp_work\n");
                          // turn off charging
                          ret = 0;
                          ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
                          ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG, SMB329B_PIN_CTRL_REG_CHG_OFF);
			     if (ret == 0)
			     {
			         resume_temp_min = SMB320B_RESUME_CHARGING_BATT_TEMP_DOWN;
			         resume_temp_max = SMB320B_RESUME_CHARGING_BATT_TEMP_UP;
		                qci_schedule_delayed_work(3,&smb329b_chip->temp_work, SMB329B_TEMP_DELAY);
		                temp_work_en = 1;
			     }
		        }
			 call_update_u_event = 1;
		     }
		 }
          }

#if USE_CHARGING_4350MV_BATTERY_PACK
#if SANYO_SOLUTION
          if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
               msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
          {
              tmp = (int)battery_temp;
		if (tmp > 50)
		{
		    if (float_voltage_mode != 0)
		    {
		        printk(KERN_ERR "[IS3_POWER] batt_temp(%d) > 50, change_float_voltage = 4100mV.\n",tmp);
		        smb329b_change_float_voltage(SMB329B_FLOAT_VOLTAGE_4100);
			 float_voltage_mode = 0;
		    }
		}
		else
		{
		    if (float_voltage_mode != 1)
		    {
		        printk(KERN_ERR "[IS3_POWER] batt_temp(%d) <= 50, change_float_voltage = default.\n",tmp);
		        smb329b_change_float_voltage(SMB329B_FLOAT_VOLTAGE_DEFAULT);
			 float_voltage_mode = 1;
		    }
		}
          }
#endif
#endif

          if (first_ovp_check >= 50 && check_ovp_again == 1)
          {
              check_ovp_again = 0;
		check_ovp1_status();
		check_ovp2_status();
          }

          if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
               msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
          {

              call_retry = 0;
		DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << work - start >>\n");

              if (smb329b_ready != CHIP_STATUS_INIT_OK)
              {
                 DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] not ready\n");
	          goto exit1;
              }

#if ISDB_T_SOLUTION
		if (isdb_t_on == 1 && msm_batt_info.batt_capacity > RE_CHARGE_BATT_CAPACITY_ISDB_T)
		{
		     if (isdb_t_stop_chg == 0 && (msm_batt_info.batt_capacity >= STOP_CHG_BATT_CAPACITY_ISDB_T || smb329b_is_CV_mode() == 1))
		     {
		          printk(KERN_ERR "[IS3_POWER][ISDB-T]  batt_capacity(%d) >= %d,  call stop_chg\n",msm_batt_info.batt_capacity,STOP_CHG_BATT_CAPACITY_ISDB_T);
		          isdb_t_stop_chg = 1;
		          is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG,SMB329B_COMMAND_A_REG_DEFAULT);
	                 is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG,SMB329B_PIN_CTRL_REG_CHG_OFF);
		     }

                   if (isdb_t_stop_chg == 1)
		        goto exit1;
		}
		else if (isdb_t_on == 0 && isdb_t_stop_chg == 1)
		{
		     printk(KERN_ERR "[IS3_POWER] ISDB-T OFF, call smb329b_batt_re_charging(isdb_t_on=%d,isdb_t_stop_chg=%d)\n",isdb_t_on ,isdb_t_stop_chg);

		     if (smb329b_batt_re_charging(1,0) == 0)
		     {
			    if (battery_capacity < 100)
			    {
			          if (lock_batt_status == 0) msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
			    }
			    if (msm_batt_info.chg_flow_status_now != CHG_FLOW_STATUS_N)
                         {
                                DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_N]);
                         }
                         if (lock_batt_status == 0) msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_N;
		     }
		     isdb_t_stop_chg = 0;
		     goto exit1;
		}
#endif

	       ret = is2_smbus_read_byte(smb329b_chip->client, SMB329B_STATUS_B_REG, &reg36_temp);
	       if (ret != 0) {
                  DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] read SMB329B_STATUS_B_REG_0x36 fail\n");
	           goto exit1;
	       }
	       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] SMB329B_STATUS_B_REG 0x36 = %x\n",reg36_temp);

              if (lock_batt_status == 0)
              {
#if SW_CHECK_BATT_OVLO
                   if (msm_batt_info.battery_voltage > SW_CHECK_OVC_VOLTAGE)
		     {
			    printk(KERN_ERR "[IS3_POWER][CHG_ERROR] BATTERY_OVLO(batt_voltage=%d > %d)\n",msm_batt_info.battery_voltage,SW_CHECK_OVC_VOLTAGE);
			    send_chg_err_to_ap(CHG_ST_NTFY_CODE,K_CHG_ST_OVC);
		     }
                   else if ((reg36_temp & SMB329B_CHG_STATUS_MASK) != 0)
#else
                   if ((reg36_temp & SMB329B_CHG_STATUS_MASK) != 0)
#endif
	            {
	                if (msm_batt_info.batt_status != POWER_SUPPLY_STATUS_FULL &&
				(check_cradle_is_connect() ||
				 check_vbus_is_connect()))
	                {
                           msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
	                }
                   }
                   else
                   {
                       if ((reg36_temp & SMB329B_CHG_ERROR_IRQ_BIT) != 0)
	                {
                           ret = is2_smbus_read_byte(smb329b_chip->client, SMB329B_STATUS_C_REG, &reg37_temp);
                           if (ret != 0) {
                              DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] read SMB329B_STATUS_B_REG_0x37 fail\n");
                              goto exit1;
                           }
                           DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] SMB329B_STATUS_B_REG 0x37 = %x\n",reg37_temp);

                           if ((reg36_temp & SMB329B_SAFEY_TIMER_STATUS_MASK) != 0)
	                    {
	                          if (isdb_t_on == 1 && msm_batt_info.batt_capacity >= (RE_CHARGE_BATT_CAPACITY_ISDB_T - 5))
                                    call_retry = 1;
				     else if (isdb_t_on == 0 && float_voltage_mode == 0 && msm_batt_info.battery_voltage >= 4050)
					 call_retry = 1;
	                          else if (isdb_t_on == 0 && float_voltage_mode == 1 && msm_batt_info.batt_capacity >= SW_CHECK_CHG_TIMEOUT_RETRY_MIN)
					 call_retry = 1;
				     else
	                          {
	                             printk(KERN_ERR "[IS3_POWER][CHG_ERROR] SAFEY_TIMER_STATUS(0x%x)(%d,%d,%d,%d)\n",
                                                                      reg36_temp & SMB329B_SAFEY_TIMER_STATUS_MASK,
                                                                      isdb_t_on,float_voltage_mode,msm_batt_info.battery_voltage,msm_batt_info.batt_capacity);
				        send_chg_err_to_ap(CHG_ST_NTFY_CODE,K_CHG_ST_OVD);
	                          }
	                    }
	                    else if ((reg36_temp & (SMB329B_AT_LEAST_ONE_CHARGE_CYCLE_BIT | SMB329B_CHG_EN_DIS_BIT)) != 0 ||
				    (reg36_temp & (SMB329B_CHG_CURRENT_L_TERMINATION_BIT | SMB329B_CHG_EN_DIS_BIT)) != 0)
	                    {
	                         if ((reg37_temp & SMB329B_BATTERY_OVLO_BIT) != 0)
				    {
#if SW_CHECK_BATT_OVLO
				        if (msm_batt_info.battery_voltage > SW_CHECK_OVC_VOLTAGE)
				        {
				              printk(KERN_ERR "[IS3_POWER][CHG_ERROR] BATTERY_OVLO(batt_voltage=%d > %d)\n",msm_batt_info.battery_voltage,SW_CHECK_OVC_VOLTAGE);
				              send_chg_err_to_ap(CHG_ST_NTFY_CODE,K_CHG_ST_OVC);
				        }
					 else
					 {
#endif
	                                   call_retry = 1;
#if SW_CHECK_BATT_OVLO
				        }
#endif
                                }
				    else if (((reg37_temp & SMB329B_WD_INTERRUPT_BIT) != 0 &&  msm_batt_info.batt_capacity >= SW_CHECK_CHG_TIMEOUT_RETRY_MIN)||
						   reg37_temp == 0)
				    {
				          call_retry = 1;
				    }
	                    }
			      else
			      {
	                       check_chg_error = 1;
			      }
			      if (call_retry == 1)
			      {
			          if (check_cradle_is_connect() || check_vbus_is_connect())
	                        {
	                             printk(KERN_ERR "[IS3_POWER] call smb329b_batt_re_charging(reg36=0x%x,reg37=0x%x,batt_voltage=%d,batt_level=%d)\n",
                                                                    reg36_temp,reg37_temp,msm_batt_info.battery_voltage,msm_batt_info.batt_capacity);
		                      if (smb329b_batt_re_charging(1,0) == 0)
		                      {
		                          ret = is2_smbus_read_byte(smb329b_chip->client, SMB329B_STATUS_B_REG, &reg36_temp);
                                        if (ret != 0) {
                                           DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] read SMB329B_STATUS_B_REG_0x36 fail\n");
	                                    goto exit1;
	                                 }
	                                 DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] SMB329B_STATUS_B_REG 0x36 = %x\n",reg36_temp);
				            if ((reg36_temp & SMB329B_CHG_ERROR_IRQ_BIT) != 0)
	                                 {
	                                     ret = is2_smbus_read_byte(smb329b_chip->client, SMB329B_STATUS_C_REG, &reg37_temp);
                                            if (ret != 0) {
                                               DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] read SMB329B_STATUS_B_REG_0x37 fail\n");
                                               goto exit1;
                                            }
                                            DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] SMB329B_STATUS_B_REG 0x37 = %x\n",reg37_temp);
				                check_chg_error = 1;
				            }
		                      }
	                        }
				   else
				   {
				       smb329b_stop_charging();
		                     goto exit1;
				   }
			      }

			      if (check_chg_error == 1)
			      {
			           printk(KERN_ERR "[IS3_POWER][CHG_ERROR] REG 0x37 = %x\n",reg37_temp);
				    if ((reg37_temp & SMB329B_BATTERY_MISSING_BIT) != 0)
	                         {
					  printk(KERN_ERR "[IS3_POWER][CHG_ERROR] BATTERY_MISSING\n");
                                     msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
	                              msm_batt_info.batt_valid = 0;
	                         }
				    else  if ((reg37_temp & SMB329B_BATTERY_OVLO_BIT) != 0)
				    {
				         printk(KERN_ERR "[IS3_POWER][CHG_ERROR] BATTERY_OVLO(batt_voltage=%d)\n",msm_batt_info.battery_voltage);
				         send_chg_err_to_ap(CHG_ST_NTFY_CODE,K_CHG_ST_OVC);
				    }
				    else  if ((reg37_temp & SMB329B_WD_INTERRUPT_BIT) != 0)
				    {
				         printk(KERN_ERR "[IS3_POWER][CHG_ERROR] WD_INTERRUPT - cal smb329b_batt_re_charging(reg37_temp=0x%x,no error)\n",reg37_temp);
				         smb329b_batt_re_charging(1,0);
				    }
				    else  if ((reg37_temp & SMB329B_INPUT_VOLO_BIT) != 0)
				    {
				         printk(KERN_ERR "[IS3_POWER][CHG_ERROR] INPUT_VOLO\n");
				         send_chg_err_to_ap(CHG_ST_NTFY_CODE,K_CHG_ST_OVP);
				    }
				    else  if ((reg37_temp & SMB329B_DCIN_LOWER_VBATT_BIT) != 0)
				    {
				         printk(KERN_ERR "[IS3_POWER][CHG_ERROR] DCIN_LOWER_VBATT\n");
                                     smb329b_stop_charging();
		                       goto exit1;
				    }
				    else if (reg37_temp != 0)
				    {
				         printk(KERN_ERR "[IS3_POWER][CHG_ERROR] OTHER - cal smb329b_batt_re_charging(reg37_temp=0x%x,no error)\n",reg37_temp);
				         smb329b_batt_re_charging(1,0);
				    }
				    else if (reg37_temp == 0)
				    {
				         if (check_cradle_is_connect() || check_vbus_is_connect())
	                              {
	                                 printk(KERN_ERR "[IS3_POWER] call smb329b_batt_re_charging(reg37_temp=0,no error)\n");
		                          smb329b_batt_re_charging(1,0);
	                              }
				    }
			      }
                       }
	                else if ((reg36_temp & (SMB329B_AT_LEAST_ONE_CHARGE_CYCLE_BIT | SMB329B_CHG_EN_DIS_BIT)) != 0)
	                {
	                    if (msm_batt_info.batt_status != POWER_SUPPLY_STATUS_FULL)
                               msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
	                }
	                else if ((reg36_temp & (SMB329B_CHG_CURRENT_L_TERMINATION_BIT | SMB329B_CHG_EN_DIS_BIT)) != 0)
	                {
	                    if (msm_batt_info.batt_status != POWER_SUPPLY_STATUS_FULL)
                               msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
	                }
	                else
	                {
	                    if (check_vbus_is_connect() == 1)
	                    {
	                        is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
                               is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG, SMB329B_PIN_CTRL_REG_CHG_OFF);
                               msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
				   DBG_BUILD("[IS3_POWER_DEBUG-10] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	                    }
	                    else if (cradle_connect == 1)
	                    {
		                 if (check_cradle_is_connect() == 0)
		                 {
		                     smb329b_cradle_draw(0,0);
		                     goto exit1;
		                 }
		                 else
		                 {
		                     is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
                                   is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG, SMB329B_PIN_CTRL_REG_CHG_OFF);
                                   msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
					DBG_BUILD("[IS3_POWER_DEBUG-11] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
		                 }
	                    }
	                    else
	                    {
		                  smb329b_stop_charging();
		                  goto exit1;
	                    }
	               }
                 }
                 call_update_u_event = 1;
             }

exit1:
	      DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << work - end >>\n");

          }
	   else if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING &&
                      msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_GOOD &&
                      (check_vbus_is_connect()== 1 || check_cradle_is_connect()== 1) &&
                      msm_batt_info.pm_temp_msm_over_layer < 2)
	   {
                DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] re-try charging!!");
		  smb329b_batt_re_charging(1,0);
	   }
	   else if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_DISCHARGING &&
                       (check_vbus_is_connect()== 1 || check_cradle_is_connect()== 1))
	   {
	         if (msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_GOOD &&
                     msm_batt_info.pm_temp_msm_over_layer < 2)
	         {
	             DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] (USB_VBUS_MPP status error) re-try charging!!");
		      smb329b_batt_re_charging(1,0);
	         }
		  else
		  {
		      if (lock_batt_status == 0)
                    {
		          msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		          call_update_u_event = 1;
		      }
		  }
	   }
	   if (first_ovp_check < 50)
	   {
	       first_ovp_check += jiffies_to_msecs(max17040gt_delay_time) / 1000;
	   }
          if (batt_work_disable == 0)
          {
	      qci_schedule_delayed_work(8,&chip_data->work, max17040gt_delay_time);
          }
       }
	if (call_update_u_event == 1)
	{
	    update_u_event();
	}
	batt_work_on = 0;
}

static void max17040gt_work(struct work_struct *work)
{
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] <<work - star>>\n");
       batt_chg_status_update(work);
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << work - end >>\n");
}

static uint32_t chg_key_code = 0;
static uint32_t chg_key_parm = 0;
static void smb329b_send_keycode_work(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_send_keycode_work - start>>\n");

       if (qci_get_chg_err_key_ready() == 1 && send_keycode_work_disable == 0)
           report_chg_err_key(chg_key_code,chg_key_parm);
	else if (send_keycode_work_disable == 0)
	    qci_schedule_delayed_work(17,&smb329b_chip->send_keycode_work, msecs_to_jiffies(100));

	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_send_keycode_work - end >>\n");
}

void send_none_to_ap(uint32_t key_code)
{
     printk(KERN_INFO "[IS3_POWER] send keycode=K_CHG_ST_NONE to AP\n");
     if (qci_get_chg_err_key_ready() == 1)
     {
        report_chg_err_key(key_code,K_CHG_ST_NONE);
     }
     else
     {
        send_keycode_work_disable = 1;
        cancel_delayed_work_sync(&smb329b_chip->send_keycode_work);
        chg_key_code = key_code;
        chg_key_parm = K_CHG_ST_NONE;
        send_keycode_work_disable = 0;
        qci_schedule_delayed_work(17,&smb329b_chip->send_keycode_work, msecs_to_jiffies(100));
     }
}

static void send_chg_err_to_ap(uint32_t key_code, uint32_t key_parm)
{
     if (check_cradle_is_connect() || check_vbus_is_connect())
     {
          qci_schedule_delayed_work(1,&smb329b_chip->stop_charging_ic_work, msecs_to_jiffies(500));
	   msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_DEAD;
          msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	   DBG_BUILD("[IS3_POWER_DEBUG-2] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);

	   if (qci_get_offline_charging() == 0 && qci_get_recovery_charging() == 0)
	   {
	       if (key_parm == K_CHG_ST_OVP)
	          printk(KERN_INFO "[IS3_POWER] send keycode=K_CHG_ST_OVP to AP\n");
              else if (key_parm == K_CHG_ST_OVC)
	          printk(KERN_INFO "[IS3_POWER] send keycode=K_CHG_ST_OVC to AP\n");
              else if (key_parm == K_CHG_ST_OVD)
	          printk(KERN_INFO "[IS3_POWER] send keycode=K_CHG_ST_OVD to AP\n");
              else if (key_parm == K_CHG_ST_EXP)
	          printk(KERN_INFO "[IS3_POWER] send keycode=K_CHG_ST_EXP to AP\n");
		if (qci_get_chg_err_key_ready() == 1)
              {
                 report_chg_err_key(key_code,key_parm);
              }
              else
              {
	          send_keycode_work_disable = 1;
		   cancel_delayed_work_sync(&smb329b_chip->send_keycode_work);
	          chg_key_code = key_code;
		   chg_key_parm = key_parm;
		   send_keycode_work_disable = 0;
	          qci_schedule_delayed_work(17,&smb329b_chip->send_keycode_work, msecs_to_jiffies(100));
              }
	   }
     }
     else
     {
          msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	   DBG_BUILD("[IS3_POWER_DEBUG-4] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	   stop_chg_clear_batt_info(1);
     }
     update_u_event();
}

void thermal_error_send_keycode_to_ap(uint32_t key_code, uint32_t key_parm)
{
      if (qci_get_offline_charging() == 0 && qci_get_recovery_charging() == 0)
      {
         if (key_parm == K_TEMP_MSM_OVER_LAYER1)
	     printk(KERN_INFO "[Thermistor] send keycode=K_TEMP_MSM_OVER_LAYER1 to AP\n");
         else if (key_parm == K_TEMP_MSM_OVER_LAYER2)
	     printk(KERN_INFO "[Thermistor] send keycode=K_TEMP_MSM_OVER_LAYER2 to AP\n");
         else if (key_parm == K_TEMP_MSM_OVER_LAYER3)
	    printk(KERN_INFO "[Thermistor] send keycode=K_TEMP_MSM_OVER_LAYER3 to AP\n");
         else if (key_parm == K_TEMP_MSM_OVER_RESUME_NORMAL)
	    printk(KERN_INFO "[Thermistor] send keycode=K_TEMP_MSM_OVER_RESUME_NORMAL to AP\n");

         if (qci_get_chg_err_key_ready() == 1)
         {
             report_chg_err_key(key_code,key_parm);
         }
         else
         {
             send_keycode_work_disable = 1;
	      cancel_delayed_work_sync(&smb329b_chip->send_keycode_work);
	      chg_key_code = key_code;
	      chg_key_parm = key_parm;
	      send_keycode_work_disable = 0;
	      qci_schedule_delayed_work(17,&smb329b_chip->send_keycode_work, msecs_to_jiffies(100));
         }
      }
}

static void thermal_msm_overheat_work(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[Thermistor] << thermal_msm_overheat_work - start>>\n");
       if (thermal_msm_send_layer3_keycode == 1)
       {
          printk(KERN_INFO "[Thermistor] send keycode=K_TEMP_MSM_OVER_LEYER3 to AP\n");
          thermal_error_send_keycode_to_ap(CHG_ST_NTFY_CODE,K_TEMP_MSM_OVER_LAYER3);
       }
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[Thermistor] << thermal_msm_overheat_work - end >>\n");
}

static void thermal_msm_overheat_work2(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[Thermistor] << thermal_msm_overheat_work2 - start>>\n");

       if (thermal_lock_en == 1)
	{
	    printk(KERN_ERR "[Thermistor] wake_unlock(suspend)\n");
	    wake_unlock(&smb329b_chip->thermal_lock);
	    thermal_lock_en = 0;
	}

	DBG_LIMIT(debug_level,TYPE_SMB329B,"[Thermistor] << thermal_msm_overheat_work2 - end >>\n");
}

void temp_msm_status_update(struct work_struct *work)
{
       u32 pm_temp_msm = msm_batt_info.pm_temp_msm;
	u32 pm_temp_msm_raw = msm_batt_info.pm_temp_msm_raw;
	u32 pm_temp_msm_over_layer = msm_batt_info.pm_temp_msm_over_layer;
       struct msm_pm_get_val_ret_data pm_data;
	unsigned long work_delay_time;
	int tmp;

       pm_data = update_pm_temp_RPC(READ_RPC_MODE,0);
	pm_temp_msm = pm_data.pm_temp_msm;
	pm_temp_msm_raw = pm_data.pm_temp_msm_raw;
	pm_temp_msm_over_layer = pm_data.pm_temp_msm_overheat_layer;
	//printk(KERN_ERR "[Thermistor] temp_msm = %d,%d\n",pm_temp_msm,pm_temp_msm_raw);

       tmp = (int)pm_temp_msm;
	work_delay_time = MAX17040GT_DELAY_NORMAL;
       if (tmp > PM_TEMP_MSM_OVER_HEAT_LAYER1_DOWN - 2)
	   work_delay_time = MAX17040GT_DELAY_TEMP_MSM_OVERHEAT;
	if (debug_temp_value == 101 || debug_temp_value == 102)
	   work_delay_time = MAX17040GT_DELAY_TEMP_MSM_OVERHEAT;

       //for Layer1 control backlight brightness
	if (thermal_msm_set_brightness == 0 && pm_temp_msm_over_layer >= 1)
	{
	     thermal_msm_set_brightness = 1;
	     msm_batt_info.pm_temp_msm_overheat = 1;
	     overheat_set_brightness();
            printk(KERN_INFO "[Thermistor] layer(%d) >= 1 , call overheat_set_brightness()\n",pm_temp_msm_over_layer);
	}
	else if (thermal_msm_set_brightness == 1 && pm_temp_msm_over_layer < 1)
	{
	     thermal_msm_set_brightness = 0;
	     msm_batt_info.pm_temp_msm_overheat = 0;
	     cooldown_set_brightness();
	     printk(KERN_INFO "[Thermistor] layer(%d) < 1 , call cooldown_set_brightness()\n",pm_temp_msm_over_layer);
	}

	if (thermal_msm_work_on == 0 && pm_temp_msm_over_layer >= 3)
	{
	      cancel_delayed_work_sync(&thermal_msm_work2);
	      if (thermal_lock_en == 1)
	      {
	           printk(KERN_ERR "[Thermistor] wake_unlock(suspend)\n");
		    wake_unlock(&smb329b_chip->thermal_lock);
		    thermal_lock_en = 0;
	      }

             printk(KERN_INFO "[Thermistor] layer(%d) >= 3 , call thermal_msm_overheat_work()\n",pm_temp_msm_over_layer);
             thermal_msm_send_layer3_keycode = 1;
	      if (thermal_lock_en == 0)
	      {
		     thermal_lock_en = 1;
		     printk(KERN_ERR "[Thermistor] wake_lock(suspend)\n");
		     wake_lock(&smb329b_chip->thermal_lock);
	      }
	      qci_schedule_delayed_work(2,&thermal_msm_work, THERMAL_MSM_OVERHEAT_WAIT_TIME_3);
	      thermal_msm_work_on = 1;
	}
	else if (thermal_msm_work_on == 1 && pm_temp_msm_over_layer < 3)
	{
             printk(KERN_INFO "[Thermistor] layer(%d) < 3 ,  cancel thermal_msm_overheat_work()\n",pm_temp_msm_over_layer);
	      thermal_msm_send_layer3_keycode = 0;
	      cancel_delayed_work_sync(&thermal_msm_work);
	      thermal_msm_work_on = 0;
	      if (thermal_lock_en == 1)
	      {
	           printk(KERN_ERR "[Thermistor] wake_unlock(suspend)\n");
		    wake_unlock(&smb329b_chip->thermal_lock);
		    thermal_lock_en = 0;
	      }
	}

       if ( pm_temp_msm != msm_batt_info.pm_temp_msm)
       {
            printk(KERN_INFO "[Thermistor] temp_msm = %d,%d, overheat_layer = %d\n",pm_temp_msm,pm_temp_msm_raw,pm_temp_msm_over_layer);
	}

       if (msm_batt_info.pm_temp_msm_over_layer != pm_temp_msm_over_layer)
       {
            if (pm_temp_msm_over_layer < 3)
            {
		   cancel_delayed_work_sync(&thermal_msm_work2);
		   if (thermal_lock_en == 1)
		   {
		         printk(KERN_ERR "[Thermistor] wake_unlock(suspend)\n");
			  wake_unlock(&smb329b_chip->thermal_lock);
			  thermal_lock_en = 0;
		   }

                 printk(KERN_INFO "[Thermistor] temp_msm_overheat_layer(%d->%d) , send keycode to ap\n",
                                   msm_batt_info.pm_temp_msm_over_layer,pm_temp_msm_over_layer);
                 if (pm_temp_msm_over_layer == 0)
                 {
                      thermal_error_send_keycode_to_ap(CHG_ST_NTFY_CODE,K_TEMP_MSM_OVER_RESUME_NORMAL);
			 if (thermal_lock_en == 0)
			 {
			     thermal_lock_en = 1;
			     printk(KERN_ERR "[Thermistor] wake_lock(suspend)\n");
			     wake_lock(&smb329b_chip->thermal_lock);
			 }
                 }
		   else if (pm_temp_msm_over_layer == 1)
		   {
		        thermal_error_send_keycode_to_ap(CHG_ST_NTFY_CODE,K_TEMP_MSM_OVER_LAYER1);
			 if (thermal_lock_en == 0)
			 {
			     thermal_lock_en = 1;
			     printk(KERN_ERR "[Thermistor] wake_lock(suspend)\n");
			     wake_lock(&smb329b_chip->thermal_lock);
			 }
		   }
		   else if (pm_temp_msm_over_layer == 2)
		   {
		        thermal_error_send_keycode_to_ap(CHG_ST_NTFY_CODE,K_TEMP_MSM_OVER_LAYER2);
			 if (thermal_lock_en == 0)
			 {
			     thermal_lock_en = 1;
			     printk(KERN_ERR "[Thermistor] wake_lock(suspend)\n");
			     wake_lock(&smb329b_chip->thermal_lock);
			 }
		   }
		   qci_schedule_delayed_work(15,&thermal_msm_work2, THERMAL_MSM_OVERHEAT_WAIT_TIME_0_1_2);
            }
       }

       msm_batt_info.pm_temp_msm = pm_temp_msm;
       msm_batt_info.pm_temp_msm_raw = pm_temp_msm_raw;
	msm_batt_info.pm_temp_msm_over_layer = pm_temp_msm_over_layer;

       if (temp_msm_work_disable == 0)
	   qci_schedule_delayed_work(13,&max17040gt_chip->msm_work, work_delay_time);
}

static void max17040gt_msm_work(struct work_struct *work)
{
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] <<msm_work - star>>\n");
       temp_msm_status_update(work);
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << msm_work - end >>\n");
}

void temp_pa_status_update(struct work_struct *work)
{
       u32 pm_temp_pa = msm_batt_info.pm_temp_pa;
	u32 pm_temp_pa_raw = msm_batt_info.pm_temp_pa_raw;
	struct msm_pm_get_val_ret_data pm_data;
	unsigned long work_delay_time;
	//int tmp;

       pm_data = update_pm_temp_RPC(READ_RPC_MODE,1);
	pm_temp_pa = pm_data.pm_temp_pa;
	pm_temp_pa_raw = pm_data.pm_temp_pa_raw;

	//tmp = (int)pm_temp_pa;
	work_delay_time = MAX17040GT_DELAY_NORMAL;

	if (debug_temp_value >= 200  && debug_temp_value < 300) // for test pa_temp
	{
	     pm_temp_pa = debug_temp_value - 200;
	     printk(KERN_ERR "[Thermistor] set pm_temp_pa = %d\n",pm_temp_pa);
	}

       if ( pm_temp_pa != msm_batt_info.pm_temp_pa)
       {
            printk(KERN_INFO "[Thermistor] temp_pa = %d,%d\n",pm_temp_pa,pm_temp_pa_raw);
	}

	msm_batt_info.pm_temp_pa = pm_temp_pa;
       msm_batt_info.pm_temp_pa_raw = pm_temp_pa_raw;

       if (temp_pa_work_disable == 0)
	   qci_schedule_delayed_work(14,&max17040gt_chip->pa_work, work_delay_time);
}

static void max17040gt_pa_work(struct work_struct *work)
{
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] <<pa_work - star>>\n");
       temp_pa_status_update(work);
	DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << pa_work - end >>\n");
}

static int __devinit max17040gt_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int rc = 0;
	u8 RCOMP_msb = 0;
	u8 RCOMP_lsb = 0;

       DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << probe - start >>\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
	{
		rc = -EIO;
		goto out;
	}

	max17040gt_chip = kzalloc(sizeof(*max17040gt_chip), GFP_KERNEL);
	if (!max17040gt_chip)
	{
		rc = -ENOMEM;
		goto out;
	}

	max17040gt_chip->client = client;

	i2c_set_clientdata(client, max17040gt_chip);

       if (max17040gt_get_version(client)  != 0)
       {
            max17040gt_ready = CHIP_STATUS_INIT_FAIL;
	     battery_info_mode = 0;
	     goto out1;
       }

	max17040gt_get_RCOMP(client,&RCOMP_msb,&RCOMP_lsb);
       printk(KERN_ERR "[IS3_POWER] Read RCOMP = (0x%x,0x%x) \n",RCOMP_msb,RCOMP_lsb);
	if (RCOMP_msb == 0x97 && RCOMP_lsb == 0)
	{
	     re_load_custom_model = 1;
	}
	else
	{
	     re_load_custom_model = 0;
	}

#if USE_CHARGING_4350MV_BATTERY_PACK
       //vbatt_max_voltage_mode = read_nv_item();
       vbatt_max_voltage_mode = MAX_CHARGING_VOLTAGE_MODE;
	if (vbatt_max_voltage_mode == 1)
	{
	    max17040gt_INI_RCOMP0 = max17040gt_INI_RCOMP0_4200;
	    max17040gt_INI_TempCoUp_mul100 = max17040gt_INI_TempCoUp_mul100_4200;
           max17040gt_INI_TempCoDown_mul100 = max17040gt_INI_TempCoDown_mul100_4200;
	    max17040gt_INI_TempUnit = max17040gt_INI_TempUnit_4200;
	    max17040gt_INI_OCVTest = max17040gt_INI_OCVTest_4200;
	    max17040gt_INI_SOCCheckA = max17040gt_INI_SOCCheckA_4200;
	    max17040gt_INI_SOCCheckB = max17040gt_INI_SOCCheckB_4200;
	    max17040gt_INI_bit = max17040gt_INI_bit_4200;
	    max17040gt_model_data = &max17040gt_model_data_4200[0];
           SMB329B_FLOAT_VOLTAGE_DEFAULT = SMB329B_FLOAT_VOLTAGE_4200;
	    SW_CHECK_OVC_VOLTAGE = SW_CHECK_OVC_VOLTAGE_4200;
	    SW_CHACK_OVC_RESUME_VOLTAGE = 4200;
	    printk(KERN_ERR "[IS3_POWER] battery_max_mode = 4200,ovc_voltage=%d\n",SW_CHECK_OVC_VOLTAGE);
	}
	else
	{
	    max17040gt_INI_RCOMP0 = max17040gt_INI_RCOMP0_4350;
	    max17040gt_INI_TempCoUp_mul100 = max17040gt_INI_TempCoUp_mul100_4350;
           max17040gt_INI_TempCoDown_mul100 = max17040gt_INI_TempCoDown_mul100_4350;
	    max17040gt_INI_TempUnit = max17040gt_INI_TempUnit_4350;
	    max17040gt_INI_OCVTest = max17040gt_INI_OCVTest_4350;
	    max17040gt_INI_SOCCheckA = max17040gt_INI_SOCCheckA_4350;
	    max17040gt_INI_SOCCheckB = max17040gt_INI_SOCCheckB_4350;
	    max17040gt_INI_bit = max17040gt_INI_bit_4350;
	    max17040gt_model_data = &max17040gt_model_data_4350[0];
	    SMB329B_FLOAT_VOLTAGE_DEFAULT = SMB329B_FLOAT_VOLTAGE_4360;
	    SW_CHECK_OVC_VOLTAGE = SW_CHECK_OVC_VOLTAGE_4350;
	    SW_CHACK_OVC_RESUME_VOLTAGE = 4350;
	    printk(KERN_ERR "[IS3_POWER] battery_max_mode = 4360,ovc_voltage=%d\n",SW_CHECK_OVC_VOLTAGE);
	    if (vbatt_max_voltage_mode != 0)
	    {
		//write_nv_item(0,1);
		vbatt_max_voltage_mode = 0;
	    }
	}
#endif

       if (re_load_custom_model == 1)
       {
          if (max17040gt_load_custom_model(client,0) == 0)
          {
                verify_model_ready = 1;
#if USE_CHARGING_4350MV_BATTERY_PACK
                printk(KERN_ERR "[IS3_POWER] load_custom_model(4350mv_batt_pack) ok.\n");
#else
                printk(KERN_ERR "[IS3_POWER] load_custom_model(4200mv_batt_pack) ok.\n");
#endif
	         max17040gt_quick_start(max17040gt_chip->client);
          }
          else
	   {
	         verify_model_ready = 0;
#if USE_CHARGING_4350MV_BATTERY_PACK
	         printk(KERN_ERR "[IS3_POWER] load_custom_model(4350mv_batt_pack) fail !!!\n");
#else
                printk(KERN_ERR "[IS3_POWER] load_custom_model(4200mv_batt_pack) fail !!!\n");
#endif
          }
       }
	else
	{
	   verify_model_ready = 1;
#if USE_CHARGING_4350MV_BATTERY_PACK
	   printk(KERN_ERR "[IS3_POWER] reboot not load_custom_model(4350mv_batt_pack) !!!\n");
#else
          printk(KERN_ERR "[IS3_POWER] reboot not load_custom_model(4200mv_batt_pack)!!!\n");
#endif
	}

	max17040gt_ready = CHIP_STATUS_INIT_OK;
	battery_info_mode = 1;

out1:

	INIT_DELAYED_WORK_DEFERRABLE(&max17040gt_chip->work, max17040gt_work);
	INIT_DELAYED_WORK_DEFERRABLE(&max17040gt_chip->msm_work, max17040gt_msm_work);
	INIT_DELAYED_WORK_DEFERRABLE(&max17040gt_chip->pa_work, max17040gt_pa_work);
	qci_schedule_delayed_work(0,&max17040gt_chip->work, MAX17040GT_DELAY_NORMAL);
	qci_schedule_delayed_work(13,&max17040gt_chip->msm_work, MAX17040GT_DELAY_NORMAL);
	qci_schedule_delayed_work(14,&max17040gt_chip->pa_work, MAX17040GT_DELAY_NORMAL);
       DBG_LIMIT(debug_level,TYPE_MAX17040,"[MAX17040] << probe - end >>\n");
	first_ovp_check = 30;

	return 0;

out:
       max17040gt_ready = CHIP_STATUS_INIT_FAIL;
	battery_info_mode = 0;
	return rc;
}

static int __devexit max17040gt_remove(struct i2c_client *client)
{
	struct max17040gt_chip *chip_data = i2c_get_clientdata(client);

       cancel_delayed_work(&chip_data->work);
	kfree(chip_data);
	return 0;
}

#define max17040gt_suspend NULL
#define max17040gt_resume NULL

static const struct i2c_device_id max17040gt_id[] = {
	{ "max17040gt", 0 },
	{ }
};

static struct i2c_driver max17040gt_i2c_driver = {
	.driver	= {
		.name	= "max17040gt",
	},
	.probe		= max17040gt_probe,
	.remove		= __devexit_p(max17040gt_remove),
	.suspend	= max17040gt_suspend,
	.resume		= max17040gt_resume,
	.id_table	= max17040gt_id,
};

static int smb329b_check_batt_is_full(void)
{
      u8 tmp = 0;

      is2_smbus_read_byte(smb329b_chip->client, SMB329B_STATUS_B_REG, &tmp);

      DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] check full read reg(0x36) = 0x%x\n",tmp);

      if (((tmp & SMB329B_CHG_CURRENT_L_TERMINATION_BIT) != 0) &&
	   ((tmp & SMB329B_CHG_ERROR_IRQ_BIT) == 0))
           return 1;
      else
           return 0;
}

static int smb329b_read_reg(u8 addr,u8 *val)
{
      int ret;
      u8 tmp;

      ret = is2_smbus_read_byte(smb329b_chip->client, addr, &tmp);
      if (ret != 0) {
          DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] read 0x%x fail\n",addr);
	   goto err;
      }

      if (val != NULL)
	  *val = tmp;

      DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] read reg(0x%x) = 0x%x\n",addr,tmp);

      return 0;

err:

      if (val != NULL)
	  *val = 0;

      return 0;
}

static int smb329b_start_charging_reg_set(void)
{
       int ret = 0;

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_start_charging_reg_set - start >>\n");

       ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_CLEAR_IRQ_REG,0);
	ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG, SMB329B_PIN_CTRL_REG_CHG_ON);
	ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_FLOAT_VOLTAGE_REG, SMB329B_FLOAT_VOLTAGE_DEFAULT);
	ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_INPUT_CURRENT_LIMIT_REG, SMB320B_ENABLE_AICL);
	ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_CONTROL_A_REG, SMB320B_ENABLE_RE_CHARGE);
	ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_OTG_CTRL_REG, SMB320B_RE_CHARGE_VOLTAGE);

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_start_charging_reg_set - end >>\n");
       return ret;
}

static int smb329b_batt_re_charging(int mode,int first)
{
       u16 reg00_cmd_val;
       u16 reg31_cmd_val = SMB329B_COMMAND_A_REG_DEFAULT;
	int ret;

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_batt_re_charging - start >>\n");

	if (check_vbus_is_connect() == 0 && check_cradle_is_connect() == 0)
	{
	     printk(KERN_ERR "[IS3_POWER][!!!] check vbus & cradle disconnect , return\n");
	     return 1;
	}
	if (msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_DEAD)
	{
	     printk(KERN_ERR "[IS3_POWER][!!!] chg error , return\n");
	     return 1;
	}

	mdelay(100);
	lock_batt_status = 1;

	if (cradle_connect == 1)
	{
	    if (cradle_mode == 0)
	    {
	        reg31_cmd_val |= SMB329B_USB_5_1_MODE_BIT;
               msm_batt_info.current_chg_source = USB_CHG;
	        reg00_cmd_val = SMB320B_CHG_CURRENT_USB;
		 printk(KERN_ERR "[IS3_POWER] set cradle(USB mode)\n");
	    }
	    else
	    {
	        reg31_cmd_val |= SMB329B_USB_AC_MODE_BIT;
	        msm_batt_info.current_chg_source = AC_CHG;
	        reg00_cmd_val = SMB320B_CHG_CURRENT_AC;
		 change_max_current = 0;
		 printk(KERN_ERR "[IS3_POWER] set cradle(AC mode)\n");
	    }
	}
	else
	{
	    if (vbus_type != 1)
	    {
	          reg31_cmd_val |= SMB329B_USB_5_1_MODE_BIT;
		   if (first == 0)
                 msm_batt_info.current_chg_source = USB_CHG;
	          reg00_cmd_val = SMB320B_CHG_CURRENT_USB;
		   printk(KERN_ERR "[IS3_POWER] set vbus(USB mode)\n");
	    }
	    else
	    {
	          reg31_cmd_val |= SMB329B_USB_AC_MODE_BIT;
	          msm_batt_info.current_chg_source = AC_CHG;
	          reg00_cmd_val = SMB320B_CHG_CURRENT_AC;
		   change_max_current = 0;
		   printk(KERN_ERR "[IS3_POWER] set vbus(AC mode)\n");
	    }
	}
	float_voltage_mode = 1;
	isdb_t_stop_chg = 0;

       ret = 0;
	ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
	ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG, SMB329B_PIN_CTRL_REG_CHG_OFF);

       mdelay(200);

	ret += smb329b_start_charging_reg_set();
	ret += is2_smbus_write_byte(smb329b_chip->client, SMB329B_COMMAND_A_REG, reg31_cmd_val);
	ret += is2_smbus_write_byte(smb329b_chip->client, SMB329B_CHG_CURRENT_REG, reg00_cmd_val);

       if (ret == 0)
       {
          if (smb329b_is_charging() == 1)
	   {
	        if (mode == 0)
	        {
                   msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
		     DBG_BUILD("[IS3_POWER_DEBUG-12] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	        }
               else
	        {
	           if (msm_batt_info.batt_capacity >= 100 && smb329b_check_batt_is_full())
	           {
		        msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
		        DBG_BUILD("[IS3_POWER_DEBUG-13] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	           }
		    else
		    {
		        msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
			 DBG_BUILD("[IS3_POWER_DEBUG-14] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
		    }
	        }
	        msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_N;
               DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_N]);
          }
	   else
	   {
               set_batt_status_not_charging();
	   }
	}
	else
	{
	     set_batt_status_not_charging();
	}

	//update_u_event();
	lock_batt_status = 0;

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_batt_re_charging - end >>\n");

       return 0;
}

//chg_flow_path c : battery temperature higher than 10 Celsius,
//                          higher than or equal to 55 Celsius.
static int smb329b_batt_temp_resume_restart_charging(void)
{
       u16 reg00_cmd_val;
       u16 reg31_cmd_val = SMB329B_COMMAND_A_REG_DEFAULT;
	int charging_fail = 1;
	int ret = 0;

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_batt_temp_resume_restart_charging - start >>\n");

	mdelay(100);

	if (cradle_connect == 1)
	{
	    if (cradle_mode == 0)
	    {
	        reg31_cmd_val |= SMB329B_USB_5_1_MODE_BIT;
               msm_batt_info.current_chg_source = USB_CHG;
	        reg00_cmd_val = SMB320B_CHG_CURRENT_USB;
		 printk(KERN_ERR "[IS3_POWER] set cradle(USB mode)\n");
	    }
	    else
	    {
	        reg31_cmd_val |= SMB329B_USB_AC_MODE_BIT;
	        msm_batt_info.current_chg_source = AC_CHG;
	        reg00_cmd_val = SMB320B_CHG_CURRENT_AC;
		 change_max_current = 0;
		 printk(KERN_ERR "[IS3_POWER] set cradle(AC mode)\n");
	    }
	}
	else
	{
	    if (vbus_type != 1)
	    {
	          reg31_cmd_val |= SMB329B_USB_5_1_MODE_BIT;
                 msm_batt_info.current_chg_source = USB_CHG;
	          reg00_cmd_val = SMB320B_CHG_CURRENT_USB;
		   printk(KERN_ERR "[IS3_POWER] set vbus(USB mode)\n");
	    }
	    else
	    {
	          reg31_cmd_val |= SMB329B_USB_AC_MODE_BIT;
	          msm_batt_info.current_chg_source = AC_CHG;
	          reg00_cmd_val = SMB320B_CHG_CURRENT_0_5_C;
		   printk(KERN_ERR "[IS3_POWER] set vbus(AC mode)\n");
	    }
	}
	float_voltage_mode = 1;
	isdb_t_stop_chg = 0;

       ret = 0;
	ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
	ret += smb329b_start_charging_reg_set();
	ret += is2_smbus_write_byte(smb329b_chip->client, SMB329B_COMMAND_A_REG, reg31_cmd_val);
	ret += is2_smbus_write_byte(smb329b_chip->client, SMB329B_CHG_CURRENT_REG, reg00_cmd_val);

       if (ret == 0)
       {
          if (smb329b_is_charging() == 1)
	   {
	        charging_fail = 0;
          }
	   else
	   {
               mdelay(200);
	        smb329b_batt_re_charging(1,0);
	        if (smb329b_is_charging() == 1)
	        {
	             charging_fail = 0;
               }
	   }
	}

	if (charging_fail == 0)
	{
	     if (msm_batt_info.batt_capacity >= 100 && smb329b_check_batt_is_full())
	     {
	         msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
		  DBG_BUILD("[IS3_POWER_DEBUG-15] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	     }
	     else
	     {
	        if (msm_batt_info.batt_status != POWER_SUPPLY_STATUS_FULL)
	        {
                   msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
		     DBG_BUILD("[IS3_POWER_DEBUG-16] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	        }
	     }
	     msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_N;
            DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_N]);
	}
	else
	{
	    set_batt_status_not_charging();
	}

	update_u_event();

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_batt_temp_resume_restart_charging - end >>\n");
       return 0;
}

static void smb329b_temp_work(struct work_struct *work)
{
       int tmp;
	int call_re_charging = 0;

       temp_work_on = 1;
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_temp_work - start>>\n");

	tmp = (int)msm_batt_info.battery_temp;
       DBG_LIMIT(debug_level,TYPE_SMB329B, "[SMB329B] battery temp = %d\n",tmp);
	if (tmp >= resume_temp_min &&
	    tmp <= resume_temp_max)
	{
	     call_re_charging++;
	}
	if (msm_batt_info.pm_temp_msm_over_layer < 2)
	{
	     call_re_charging++;
	}
	if (call_re_charging >= 2)
	{
            msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
	     printk(KERN_ERR "[IS3_POWER] smb329b_temp_work - (%d <= %d <= %d)\n",resume_temp_min,tmp,resume_temp_max);
	     printk(KERN_ERR "[IS3_POWER] smb329b_temp_work - msm_over_layer({now}%d < 2)\n",msm_batt_info.pm_temp_msm_over_layer);
	     printk(KERN_ERR "[IS3_POWER] smb329b_temp_work - call restart_charging()\n");
	     smb329b_batt_temp_resume_restart_charging();
	}

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] vbus_connect=%d, cradle_connect=%d, batt_health=%d\n",
                                                      vbus_connect,
                                                      cradle_connect,
                                                      msm_batt_info.batt_health);
	if ((vbus_connect == 1 || cradle_connect == 1) &&
	    ((msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_OVERHEAT ||
	      msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_COLD) ||
	      msm_batt_info.pm_temp_msm_over_layer >= 2))
	{
	    DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] call smb329b_chip->temp_work\n");
           qci_schedule_delayed_work(9,&smb329b_chip->temp_work, SMB329B_TEMP_DELAY);
	}
	else
	{
	    temp_work_en = 0;
	}

	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_temp_work - end >>\n");
	temp_work_on = 0;
}

static void smb329b_wait_exit_error_work(struct work_struct *work)
{
       int tmp;
	int call_re_charging = 0;

       wait_exit_error_work_on = 1;
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_wait_exit_error_work - start>>\n");

	tmp = (int)msm_batt_info.battery_temp;
       DBG_LIMIT(debug_level,TYPE_SMB329B, "[SMB329B] battery temp = %d\n",tmp);
	if (tmp >= resume_temp_min &&
	    tmp <= resume_temp_max)
	{
            call_re_charging++;
	     wait_exit_error_status = 0;
	}
	else
	{
	     wait_exit_error_status = 1;
	}
	if (msm_batt_info.pm_temp_msm_over_layer < 2)
	{
	     call_re_charging++;
	}
	if (msm_batt_info.battery_voltage <= SW_CHACK_OVC_RESUME_VOLTAGE)
	{
            call_re_charging++;
	}
	if (call_re_charging >= 3)
	{
	     msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
	     printk(KERN_ERR "[IS3_POWER] smb329b_wait_exit_error_work - temp_resume(%d <= {now}%d <= %d)\n",resume_temp_min,tmp,resume_temp_max);
            printk(KERN_ERR "[IS3_POWER] smb329b_wait_exit_error_work - msm_over_layer({now}%d < 2)\n",msm_batt_info.pm_temp_msm_over_layer);
	     printk(KERN_ERR "[IS3_POWER] smb329b_wait_exit_error_work - batt_voltage_resume({now}%d <= %d)\n",msm_batt_info.battery_voltage,SW_CHACK_OVC_RESUME_VOLTAGE);
            printk(KERN_ERR "[IS3_POWER] smb329b_wait_exit_error_work - call restart_charging()\n");
	     smb329b_batt_temp_resume_restart_charging();
	}

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] vbus_connect=%d, cradle_connect=%d, batt_health=%d, batt_voltage=%d\n",
                                                      vbus_connect,
                                                      cradle_connect,
                                                      msm_batt_info.batt_health,
                                                      msm_batt_info.battery_voltage);

	if ((check_vbus_is_connect()== 1 || check_cradle_is_connect()== 1) &&
	    ((msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_OVERHEAT ||
	      msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_COLD) ||
	    msm_batt_info.battery_voltage > SW_CHACK_OVC_RESUME_VOLTAGE ||
	    msm_batt_info.pm_temp_msm_over_layer >= 2))
	{
	    DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] call smb329b_chip->wait_exit_error_work\n");
           qci_schedule_delayed_work(12,&smb329b_chip->wait_exit_error_work, SMB329B_TEMP_DELAY);
	}
	else
	{
	   wait_exit_error_work_en = 0;
	}

	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_wait_exit_error_work - end >>\n");
	wait_exit_error_work_on = 0;
}

static void smb329b_wait_ap_update_work(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_wait_ap_update_work - start>>\n");

       check_cradle_is_connect();
       if (vbus_connect == 0 && cradle_connect == 0)
       {
           if (wlock_en == 1)
	    {
	       printk(KERN_ERR "[IS3_POWER] wake_unlock(suspend)\n");
	       wake_unlock(&smb329b_chip->wlock);
	       wlock_en = 0;
	    }
       }

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_wait_ap_update_work - end >>\n");
}

static void smb329b_stop_charging_ic_work(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_stop_charging_ic_work - start>>\n");

       is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
       is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG, SMB329B_PIN_CTRL_REG_CHG_OFF);

	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_stop_charging_ic_work - end >>\n");
}

static int led_rad_blink_on = -1;
static int led_rad_on = -1;
static int chg_error = 0;
static int led_blink_en = 0;
static void smb329b_offline_charging_work(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_offline_charging_work - start>>\n");

       if (vbus_connect == 1 || cradle_connect == 1)
       {
             if (msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_GOOD)
             {
                  if (chg_error != 0)
	           {
	              led_rad_on = -1;
                     chg_error = 0;
	           }
		    if (led_blink_en != 0)
		    {
			 led_blink_en = 0;
		        qci_cancel_delayed_work(4,&led_blink_work_on,&smb329b_chip->led_red_blink_work,1);
			 mdelay(100);
		    }
                  if (msm_batt_info.batt_capacity >= 100)
                  {
                       if (led_rad_on != 0 && chg_error == 0)
	                {
	                    set_led_red_enable(0);
                           led_rad_on = 0;
	                }
                  }
		    else
		    {
	               if (led_rad_on != 1 && chg_error == 0)
	               {
	                   set_led_red_enable(1);
                          led_rad_on = 1;
	               }
	           }
	      }
	      else if (msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_DEAD)
	      {
	           if (chg_error != 1)
	           {
	               led_rad_on = -1;
	               chg_error = 1;
	           }
	           if (led_blink_en != 1)
	           {
	               led_rad_blink_on = -1;
			 qci_schedule_delayed_work(4,&smb329b_chip->led_red_blink_work, SMB329B_LED_BLINK_DELAY);
			 led_blink_en = 1;
	           }
	      }
	      else
	      {
	           if (chg_error != 1)
	           {
	               led_rad_on = -1;
	               chg_error = 1;
	           }
		    if (led_blink_en != 0)
		    {
			 led_blink_en = 0;
			 qci_cancel_delayed_work(4,&led_blink_work_on,&smb329b_chip->led_red_blink_work,1);
			 mdelay(100);
		    }
	           if (led_rad_on != 0)
	           {
	              set_led_red_enable(0);
		       led_rad_on = 0;
	           }
	      }
	}
	else
	{
	      if (chg_error != 0)
	      {
	         led_rad_on = -1;
	         chg_error = 0;
	      }
	      if (led_blink_en != 0)
	      {
		    led_blink_en = 0;
		    qci_cancel_delayed_work(4,&led_blink_work_on,&smb329b_chip->led_red_blink_work,1);
		    mdelay(100);
		}
	      if (led_rad_on != 0 && chg_error == 0)
	      {
	           set_led_red_enable(0);
		    led_rad_on = 0;
	      }
	}

       qci_schedule_delayed_work(10,&smb329b_chip->offline_charging_work, SMB329B_OFFLINE_CHARGING_DELAY);
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_offline_charging_work - end >>\n");
}

static void smb329b_led_red_blink_work(struct work_struct *work)
{
       led_blink_work_on = 1;
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_led_red_blink_work - start>>\n");

       if (led_rad_blink_on != 0)
       {
           set_led_red_enable(0);
	    led_rad_blink_on = 0;
       }
	else if (led_rad_blink_on != 1)
	{
	    set_led_red_enable(1);
	    led_rad_blink_on = 1;
	}

       schedule_delayed_work(&smb329b_chip->led_red_blink_work, SMB329B_LED_BLINK_DELAY);
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_led_red_blink_work - end >>\n");
	led_blink_work_on = 0;
}

static void stop_chg_clear_batt_info(int send_keycode)
{
      if (msm_batt_info.chg_flow_status_now != CHG_FLOW_STATUS_A)
      {
            msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_B;
            DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_B]);
      }
      if (msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_OVERHEAT ||
	    msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_COLD)
      {
           if (msm_batt_info.battery_temp <= SMB320B_NO_START_CHARGING_BATT_TEMP_UP &&
		 msm_batt_info.battery_temp >= SMB320B_NO_START_CHARGING_BATT_TEMP_DOWN)
           {
               msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
           }
      }
      else if (msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_DEAD)
      {
            msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
      }
      msm_batt_info.current_chg_source = 0;
      input_mA = 0;
      smb329b_chip->chg_current = 0;
      power_source_switch_wait = 0;
      isdb_t_stop_chg = 0;

      if (send_keycode == 1)
      {
         if (qci_get_offline_charging() == 0 && qci_get_recovery_charging() == 0)
         {
             send_none_to_ap(CHG_ST_NTFY_CODE);
         }
      }
}

static int smb329b_is_charging(void)
{
       int ret = 0;
	int isCharging = 0;
	u8 tmp = 0;

       mdelay(100);
       ret = is2_smbus_read_byte(smb329b_chip->client, SMB329B_STATUS_B_REG, &tmp);
	if (ret != 0) {
		goto out;
	}

	if ((tmp & SMB329B_CHG_STATUS_MASK) != 0)
	{
		isCharging = 1;
	}
	else if (((tmp & (SMB329B_AT_LEAST_ONE_CHARGE_CYCLE_BIT | SMB329B_CHG_EN_DIS_BIT)) != 0) &&
	             ((tmp & SMB329B_CHG_ERROR_IRQ_BIT) == 0))
	{
		isCharging = 1;
	}
	else if (((tmp & (SMB329B_CHG_CURRENT_L_TERMINATION_BIT | SMB329B_CHG_EN_DIS_BIT)) != 0) &&
	            ((tmp & SMB329B_CHG_ERROR_IRQ_BIT) == 0))
	{
		isCharging = 1;
	}

	if (isCharging == 1 && check_vbus_is_connect() == 0 && check_cradle_is_connect() == 0)
	{
	       isCharging = 0;
	}

	if (isCharging == 0)
	{
	       printk(KERN_ERR "[IS3_POWER] smb329b_is_charging - SMB329B_STATUS_B_REG 0x36 = 0x%x\n",tmp);
	}

out:
	return isCharging;
}

static int smb329b_is_CV_mode(void)
{
       int ret = 0;
	int isCVmode = 0;
	u8 tmp = 0;

       ret = is2_smbus_read_byte(smb329b_chip->client, SMB329B_STATUS_B_REG, &tmp);
	if (ret != 0) {
		goto out;
	}

	if (((tmp & (SMB329B_CHG_STATUS_MASK | SMB329B_CHG_EN_DIS_BIT)) == (SMB329B_CHG_STATUS_MASK | SMB329B_CHG_EN_DIS_BIT)) &&
	       ((tmp & SMB329B_CHG_ERROR_IRQ_BIT) == 0))
	{
		isCVmode = 1;
	}

	if (isCVmode == 1 && check_vbus_is_connect() == 0 && check_cradle_is_connect() == 0)
	{
	       isCVmode = 0;
	}

	if (isCVmode == 0)
	{
	    printk(KERN_ERR "[IS3_POWER] smb329b_is_CV_mode - SMB329B_STATUS_B_REG 0x36 = 0x%x\n",tmp);
	}

out:
	return isCVmode;
}

void wait_exit_error_resume_set(int mode)
{
       lock_batt_status = 1;
       is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
       is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG, SMB329B_PIN_CTRL_REG_CHG_OFF);
	msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	DBG_BUILD("[IS3_POWER_DEBUG-17] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	lock_batt_status = 0;
	if (mode == 0)
	{
	}
	else
	{
	   if (wait_exit_error_work_en == 0)
	   {
	      if (temp_work_en == 1)
	      {
	         qci_cancel_delayed_work(3,&temp_work_on,&smb329b_chip->temp_work,0);
	         temp_work_en = 0;
	      }
	      printk(KERN_ERR "[IS3_POWER] call  smb329b_chip->wait_exit_error_work  !!!\n");
             resume_temp_min = SMB320B_NO_START_CHARGING_BATT_TEMP_DOWN;
	      resume_temp_max = SMB320B_NO_START_CHARGING_BATT_TEMP_UP;
	      wait_exit_error_work_en = 1;
	      wait_exit_error_status = 0;
	      qci_schedule_delayed_work(11,&smb329b_chip->wait_exit_error_work, SMB329B_TEMP_DELAY);
	   }
	}
	update_u_event();
}

//chg_flow_path e : VBUS higher than 4.5V or more, battery voltage 3.05V or more
static int smb329b_start_charging(int chg_current)
{
	u16 reg31_cmd_val = SMB329B_COMMAND_A_REG_DEFAULT;
	u16 reg00_cmd_val = SMB320B_CHG_CURRENT_DEFAULT;
	int charging_fail = 1;
	int ret = 0;
	unsigned long work_delay_tmp = MAX17040GT_DELAY_QUICK;

	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_start_charging - start >>\n");

       cancel_delayed_work_sync(&smb329b_chip->wait_ap_update_work);
       cancel_delayed_work_sync(&smb329b_chip->stop_charging_ic_work);
	mdelay(100);
	if (wlock_en == 0)
	{
	    printk(KERN_ERR "[IS3_POWER] wake_lock(suspend)\n");
	    wake_lock(&smb329b_chip->wlock);
	    wlock_en = 1;
	}

	if (power_source_switch_wait == 1)
	{
	     mdelay(SMB320B_WAIT_OVP_SWITCH_READY_MS);
	}

       if (msm_batt_info.chg_flow_status_now == CHG_FLOW_STATUS_A)
       {
            printk(KERN_ERR "[IS3_POWER][!!!] CHG_FLOW_STATUS_A return\n");
	     send_chg_err_to_ap(CHG_ST_NTFY_CODE,K_CHG_ST_EXP);
	     return 0;
       }
	else if ((msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING ||
                    msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL) &&
                    smb329b_chip->chg_current == chg_current)
	{
	     printk(KERN_ERR "[IS3_POWER][!!!] smb329b_start_charging (status = %d) - current not change %d,%d return\n",
                                                          msm_batt_info.batt_status,smb329b_chip->chg_current,chg_current);
	     return 0;
	}
	else if (msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_COLD ||
		     msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_OVERHEAT)
	{
	     printk(KERN_ERR "[IS3_POWER][!!!] battery temp (%d,%d) overRange return\n",
                                                               msm_batt_info.battery_temp,
                                                               msm_batt_info.batt_health);
	     wait_exit_error_resume_set(1);
	     return 0;
	}
	else if (msm_batt_info.pm_temp_msm_over_layer >= 2)
	{
	     printk(KERN_ERR "[IS3_POWER][!!!] pm_temp_msm_over_layer (%d) >= 2\n",msm_batt_info.pm_temp_msm_over_layer);
	     wait_exit_error_resume_set(1);
	     return 0;
	}
	else if (msm_batt_info.battery_voltage > SW_CHECK_OVC_VOLTAGE)
	{
            printk(KERN_ERR "[IS3_POWER][!!!] battery voltage = %d > OVC(%d)!!!\n",msm_batt_info.battery_voltage,SW_CHECK_OVC_VOLTAGE);
	     wait_exit_error_resume_set(1);
	     return 0;
	}
       batt_work_disable = 1;
	qci_cancel_delayed_work(0,&batt_work_on,&max17040gt_chip->work,1);
	lock_batt_status = 1;

	if (chg_current < 500)
	{
		reg31_cmd_val &= ~SMB329B_USB_5_1_MODE_BIT;
		if (check_cradle_is_connect() == 1)
		   msm_batt_info.current_chg_source = USB_CHG;
		smb329b_chip->chg_current = 100;
		printk(KERN_ERR "[IS3_POWER] set chg(USB-100 mode)\n");
	}
	else if (chg_current == 500)
	{
		reg31_cmd_val |= SMB329B_USB_5_1_MODE_BIT;
		if (check_cradle_is_connect() == 1)
		   msm_batt_info.current_chg_source = USB_CHG;
		reg00_cmd_val = SMB320B_CHG_CURRENT_USB;
		smb329b_chip->chg_current = 500;
		printk(KERN_ERR "[IS3_POWER] set chg(USB-500 mode)\n");
	}
	else
	{
		reg31_cmd_val |= SMB329B_USB_AC_MODE_BIT;
		msm_batt_info.current_chg_source = AC_CHG;
		reg00_cmd_val = SMB320B_CHG_CURRENT_AC;
		smb329b_chip->chg_current = 1000;
		change_max_current = 0;
		printk(KERN_ERR "[IS3_POWER] set chg(AC mode)\n");
	}
	float_voltage_mode = 1;
	isdb_t_stop_chg = 0;

       ret = 0;
       ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
	if (power_source_switch_wait == 1)
	{
	     ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG,SMB329B_PIN_CTRL_REG_CHG_OFF);
	     power_source_switch_wait = 0;
	     mdelay(100);
	}

	ret += smb329b_start_charging_reg_set();
	ret += is2_smbus_write_byte(smb329b_chip->client, SMB329B_COMMAND_A_REG, reg31_cmd_val);
	ret += is2_smbus_write_byte(smb329b_chip->client, SMB329B_CHG_CURRENT_REG, reg00_cmd_val);

       if (ret == 0)
       {
          if (smb329b_is_charging() == 1)
	   {
	        charging_fail = 0;
          }
	   else
	   {
	        mdelay(200);
               smb329b_batt_re_charging(1,1);
	        if (smb329b_is_charging() == 1)
	        {
	             charging_fail = 0;
               }
	   }
	}

	if (charging_fail == 0)
	{
	     if (msm_batt_info.batt_capacity >= 100 && smb329b_check_batt_is_full())
	     {
	         msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
		  DBG_BUILD("[IS3_POWER_DEBUG-20] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	     }
	     else
	     {
	        if (msm_batt_info.batt_status != POWER_SUPPLY_STATUS_FULL)
	        {
                   msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
		     DBG_BUILD("[IS3_POWER_DEBUG-21] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	        }
	     }
	     msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_N;
            DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_N]);
	     printk(KERN_ERR "[IS3_POWER] start_charging_set - ok\n");
	}
	else
	{
	    set_batt_status_not_charging();
	    printk(KERN_ERR "[IS3_POWER] start_charging_set - fail\n");
	    work_delay_tmp = MAX17040GT_DELAY_RE_TRY;
	}

	update_u_event();
	batt_work_disable = 0;
	qci_schedule_delayed_work(0,&max17040gt_chip->work, work_delay_tmp);

	lock_batt_status = 0;

	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_start_charging - end >>\n");

	return 0;
}

static int smb329b_change_float_voltage(u8 val)
{
       int ret = 0;
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_change_float_voltage - start >>\n");

	ret += is2_smbus_write_byte(smb329b_chip->client, SMB329B_FLOAT_VOLTAGE_REG, val);

	if (ret != 0)
	   printk(KERN_ERR "[IS3_POWER] smb329b_change_float_voltage(0x%x) - fail!!!\n",val);

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_change_float_voltage - end >>\n");

	return ret;
}

static int smb329b_change_max_charging_current(int mode)
{
       u16 reg00_cmd_val;
	int ret = 0;

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_change_max_charging_current(mode=%d) - start >>\n",mode);

	if (mode == 1)
       {
               reg00_cmd_val = SMB320B_CHG_CURRENT_0_5_C;
       }
       else
       {
               reg00_cmd_val = SMB320B_CHG_CURRENT_AC;
       }
	ret += is2_smbus_write_byte(smb329b_chip->client, SMB329B_CHG_CURRENT_REG, reg00_cmd_val);

	if (ret != 0)
	   printk(KERN_ERR "[IS3_POWER] smb329b_change_max_charging_current(%d) - fail!!!\n",mode);

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_change_max_charging_current - end >>\n");

	return ret;
}

static int smb329b_change_cradle_current(int mode)
{
       u16 reg00_cmd_val;
       u16 reg31_cmd_val = SMB329B_COMMAND_A_REG_DEFAULT;
	int ret = 0;

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_change_cradle_current - start >>\n");

	if (mode == 0)
	{
	      reg31_cmd_val |= SMB329B_USB_5_1_MODE_BIT;
	      msm_batt_info.current_chg_source = USB_CHG;
	      reg00_cmd_val = SMB320B_CHG_CURRENT_USB;
	}
	else
	{
	      reg31_cmd_val |= SMB329B_USB_AC_MODE_BIT;
	      msm_batt_info.current_chg_source = AC_CHG;
	      reg00_cmd_val = SMB320B_CHG_CURRENT_AC;
	      change_max_current = 0;
	}
	float_voltage_mode = 1;
	isdb_t_stop_chg = 0;

	ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
	ret += is2_smbus_write_byte(smb329b_chip->client, SMB329B_COMMAND_A_REG, reg31_cmd_val);
	ret += is2_smbus_write_byte(smb329b_chip->client, SMB329B_CHG_CURRENT_REG, reg00_cmd_val);

	if (ret != 0)
	   printk(KERN_ERR "[IS3_POWER] smb329b_change_cradle_current(%d) - fail!!!\n",mode);

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_change_cradle_current - end >>\n");
       return ret;
}

void wait_temp_resume_set(void)
{
      lock_batt_status = 1;
      is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
      is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG, SMB329B_PIN_CTRL_REG_CHG_OFF);
      msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
      DBG_BUILD("[IS3_POWER_DEBUG-22] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
      lock_batt_status = 0;
      if (temp_work_en == 0 && wait_exit_error_work_en == 0)
      {
	    printk(KERN_ERR "[IS3_POWER][ovp_path_switch] call smb329b_chip->temp_work !!!\n");
	    resume_temp_min = SMB320B_NO_START_CHARGING_BATT_TEMP_DOWN;
	    resume_temp_max = SMB320B_NO_START_CHARGING_BATT_TEMP_UP;
	    qci_schedule_delayed_work(3,&smb329b_chip->temp_work, SMB329B_TEMP_DELAY);
	    temp_work_en = 1;
	}
	update_u_event();
}

static int smb329b_ovp_path_switch_restart_charging(void)
{
       u16 reg00_cmd_val;
       u16 reg31_cmd_val = SMB329B_COMMAND_A_REG_DEFAULT;
       int charging_fail = 1;
	int ret = 0;

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_restart_charging - start >>\n");

        if (msm_batt_info.chg_flow_status_now == CHG_FLOW_STATUS_A)
       {
            printk(KERN_ERR "[IS3_POWER][ovp_path_switch][!!!] CHG_FLOW_STATUS_A return\n");
	     return 0;
       }
	else if (msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_COLD ||
	            msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_OVERHEAT)
	{
	     printk(KERN_ERR "[IS3_POWER][ovp_path_switch][!!!] battery temp (%d) overRange return\n",msm_batt_info.battery_temp);
            wait_temp_resume_set();
	     return 0;
	}
	else if (msm_batt_info.pm_temp_msm_over_layer >= 2)
	{
	     printk(KERN_ERR "[IS3_POWER][ovp_path_switch][!!!] temp_msm_over_layer (%d) >=2 return\n",msm_batt_info.pm_temp_msm_over_layer);
            wait_temp_resume_set();
	     return 0;
	}

	mdelay(100);
	lock_batt_status = 1;

	if (vbus_type != 1)
	{
	      reg31_cmd_val |= SMB329B_USB_5_1_MODE_BIT;
	      msm_batt_info.current_chg_source = USB_CHG;
	      reg00_cmd_val = SMB320B_CHG_CURRENT_USB;
	      smb329b_chip->chg_current = 500;
	}
	else
	{
	      reg31_cmd_val |= SMB329B_USB_AC_MODE_BIT;
	      msm_batt_info.current_chg_source = AC_CHG;
	      reg00_cmd_val = SMB320B_CHG_CURRENT_AC;
	      smb329b_chip->chg_current = 1000;
	      change_max_current = 0;
	}
	float_voltage_mode = 1;
	isdb_t_stop_chg = 0;

	ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
       ret += is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG,SMB329B_PIN_CTRL_REG_CHG_OFF);
       mdelay(SMB320B_WAIT_OVP_SWITCH_READY_MS);

	smb329b_start_charging_reg_set();
	ret += is2_smbus_write_byte(smb329b_chip->client, SMB329B_COMMAND_A_REG, reg31_cmd_val);
	ret += is2_smbus_write_byte(smb329b_chip->client, SMB329B_CHG_CURRENT_REG, reg00_cmd_val);

       if (ret == 0)
       {
	   if (smb329b_is_charging() == 1)
	   {
	        charging_fail = 0;
          }
	   else
	   {
               mdelay(200);
	        smb329b_batt_re_charging(1,0);
	        if (smb329b_is_charging() == 1)
	        {
	             charging_fail = 0;
               }
	   }
	}

	if (charging_fail == 0)
	{
	     if (msm_batt_info.batt_capacity >= 100 && smb329b_check_batt_is_full())
	     {
	         msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
		  DBG_BUILD("[IS3_POWER_DEBUG-24] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	     }
	     else
	     {
	        if (msm_batt_info.batt_status != POWER_SUPPLY_STATUS_FULL)
	        {
                   msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
		     DBG_BUILD("[IS3_POWER_DEBUG-25] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	        }
	     }
	     msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_N;
            DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_N]);
	}
	else
	{
	    set_batt_status_not_charging();
	}

	update_u_event();
       lock_batt_status = 0;
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_restart_charging - end >>\n");
       return 0;
}

//chg_flow_path h & i : adaptor remove detection
static int smb329b_stop_charging(void)
{
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_stop_charging - start >>\n");

       check_vbus_is_connect();
	check_cradle_is_connect();
	if (msm_batt_info.chg_flow_status_now != CHG_FLOW_STATUS_A)
	{
	   if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_DISCHARGING)
          {
              printk(KERN_ERR "[IS3_POWER][!!!] smb329b_stop_charging - staus discharging\n");
		stop_chg_clear_batt_info(1);

              if (wlock_en == 0)
	       {
	           printk(KERN_ERR "[IS3_POWER] wake_lock(suspend)\n");
	           wake_lock(&smb329b_chip->wlock);
	           wlock_en = 1;
		    qci_schedule_delayed_work(6,&smb329b_chip->wait_ap_update_work, SMB329B_WAIT_AP_DELAY);
	       }

		update_u_event();
		DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] cancel_delayed_work\n");
		if (temp_work_en == 1)
		{
	           qci_cancel_delayed_work(3,&temp_work_on,&smb329b_chip->temp_work,0);
		    temp_work_en = 0;
		}
		if (wait_exit_error_work_en == 1)
		{
		    qci_cancel_delayed_work(12,&wait_exit_error_work_on,&smb329b_chip->wait_exit_error_work,0);
		    wait_exit_error_work_en = 0;
		}
	       return 0;
	   }
	   else if (cradle_connect == 1 || vbus_connect == 1)
	   {
              if (cradle_connect == 0 || vbus_connect == 1)
              {
                    printk(KERN_ERR "[IS3_POWER][!!!] smb329b_stop_charging - cradle disconnect and usb connect return\n");
                    if (smb329b_ovp_path_switch_restart_charging() != 0)
                    {
                         printk(KERN_ERR "[IS3_POWER][!!!] smb329b_ovp_path_switch_restart_charging - fail");
			    return -1;
                    }
              }
	       else
	       {
	           printk(KERN_ERR "[IS3_POWER][!!!] smb329b_stop_charging - cradle disconnect or usb disconnect return\n");
	       }
	       return 0;
	   }
	   else if (smb329b_is_charging() == 1)
	   {
              printk(KERN_ERR "[IS3_POWER][!!!] smb329b_stop_charging - (usb or cradle) not disconnect return\n");
	       return 0;
	   }
	}

       lock_batt_status = 1;
	stop_chg_clear_batt_info(1);
	msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	DBG_BUILD("[IS3_POWER_DEBUG-26] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);

       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] cancel_delayed_work\n");
       if (temp_work_en == 1)
       {
	   qci_cancel_delayed_work(3,&temp_work_on,&smb329b_chip->temp_work,0);
	   temp_work_en = 0;
       }
	if (wait_exit_error_work_en == 1)
	{
	     qci_cancel_delayed_work(12,&wait_exit_error_work_on,&smb329b_chip->wait_exit_error_work,0);
	     wait_exit_error_work_en = 0;
	}
       batt_work_disable = 1;
	qci_cancel_delayed_work(0,&batt_work_on,&max17040gt_chip->work,1);

	is2_smbus_write_byte(smb329b_chip->client,SMB329B_COMMAND_A_REG,SMB329B_COMMAND_A_REG_DEFAULT);
	is2_smbus_write_byte(smb329b_chip->client,SMB329B_PIN_CTRL_REG,SMB329B_PIN_CTRL_REG_CHG_OFF);
	printk(KERN_ERR "[IS3_POWER] smb329b_stop_charging - ok\n");

	qci_schedule_delayed_work(6,&smb329b_chip->wait_ap_update_work, SMB329B_WAIT_AP_DELAY);

       update_u_event();
	batt_work_disable = 0;
	qci_schedule_delayed_work(0,&max17040gt_chip->work, MAX17040GT_DELAY_NORMAL);

       lock_batt_status = 0;

	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_stop_charging - end >>\n");
	return 0;
}

void set_chg_status(void)
{
      int err = 0;

      mutex_lock(&det_lock);

      if (smb329b_ready == CHIP_STATUS_NONE || get_batt_first_info == 0)
      {
            err = 1;
            printk(KERN_ERR "[IS3_POWER] << smb329b_det_work -%d,%d >>\n",smb329b_ready,get_batt_first_info);
            mdelay(100);
      }
      if (err == 1)
      {
            printk(KERN_ERR "[IS3_POWER] << smb329b_det_work(wait100) -%d,%d >>\n",smb329b_ready,get_batt_first_info);
      }

      if (smb329b_ready == CHIP_STATUS_INIT_OK)
      {
	     printk(KERN_ERR "[IS3_POWER] smb329b_draw = %d\n",input_mA);

	     if (input_mA == 0)
		   smb329b_stop_charging();
            else
                 smb329b_start_charging(input_mA);

	     if (input_mA > 0)
            {
	          if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING &&
	               (check_cradle_is_connect() == 0 && check_vbus_is_connect() == 0))
                 {
                     smb329b_stop_charging();
	          }
	     }
	}

	mutex_unlock(&det_lock);
}

static void smb329b_det_work(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_det_work - start >>\n");
       set_chg_status();
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_det_work - end >>\n");
}

static void smb329b_det_delay_work(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_det_delay_work - start >>\n");
       set_chg_status();
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_det_delay_work - end >>\n");
}

void set_vbus_status2(void)
{
       set_usb_type(0);
	if (check_cradle_is_connect() == 0)
	{
	    msm_batt_info.current_chg_source = USB_CHG;
	    update_u_event();
	}
}

static void smb329b_det_work3(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_det_work3 - start >>\n");
       set_vbus_status2();
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_det_work3 - end >>\n");
}

void set_vbus_status(void)
{
       set_usb_type(1);
	if (check_cradle_is_connect() == 0)
	{
	   msm_batt_info.current_chg_source = AC_CHG;
	   smb329b_vbus_draw2(1000,0);
	}
}

static void smb329b_det_work2(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_det_work2 - start >>\n");
       set_vbus_status();
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_det_work2 - end >>\n");
}

void smb329b_vbus_draw(unsigned int mA)
{
       if (check_vbus_is_connect() == 1)
       {
	   if (mA > 500 && vbus_type < 1)
	   {
	        printk(KERN_ERR "[IS3_POWER] << smb329b_vbus_draw2 -- call det_work2(AC) >>\n");
               qci_schedule_work(4,&smb329b_chip->det_work2);
	   }
	   else if (mA > 0 && mA <= 500)
	   {
	        printk(KERN_ERR "[IS3_POWER] << smb329b_vbus_draw2 -- call det_work3(USB) >>\n");
               qci_schedule_work(4,&smb329b_chip->det_work3);
	   }
	}
}
EXPORT_SYMBOL_GPL(smb329b_vbus_draw);

static void smb329b_vbus_draw2(unsigned int mA, int delay)
{
       input_mA = mA;
	if (mA == 0)
	{
             vbus_connect = 0;
	}
	else
	{
             vbus_connect = 1;
	}
	if (delay == 0)
	{
	   printk(KERN_ERR "[IS3_POWER] << smb329b_vbus_draw2 -- call det_work >>\n");
          qci_schedule_work(7,&smb329b_chip->det_work);
	}
	else
	{
	   printk(KERN_ERR "[IS3_POWER] << smb329b_vbus_draw2 -- call det_delay_work >>\n");
          qci_schedule_delayed_work(16,&smb329b_chip->det_delay_work,msecs_to_jiffies(500));
	}
}

void check_vbus_status(void)
{
       int val;

	val =  gpio_get_value_cansleep(smb329b_chip->usb_vbus_mpp);
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] usb_vbus_mpp = %d\n",val);

	if (val == 1)
	{
             printk(KERN_ERR ">>>>>> USB disconnect\n");
	      set_usb_type(-1);
	      smb329b_vbus_draw2(0,0);
	}
	else
	{
	      printk(KERN_ERR ">>>>>> USB connect\n");

	      if (vbus_type < 1)
	      {
	          if (vbus_type != 0)
	             set_usb_type(0);
		   if (check_cradle_is_connect() == 0)
	             smb329b_vbus_draw2(500,0);
		   else
                    printk(KERN_ERR "[IS3_POWER][!!!] check_vbus_status() - cradle on return\n");
	      }
	      else
	      {
	          if (vbus_type != 1)
	             set_usb_type(1);
		   if (check_cradle_is_connect() == 0)
		      smb329b_vbus_draw2(1000,0);
		   else
		      printk(KERN_ERR "[IS3_POWER][!!!] check_vbus_status() - cradle on return\n");
	      }
	}
}

static void smb329b_vbus_work(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_vbus_work - start >>\n");
       check_vbus_status();
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_vbus_work - end >>\n");
}

static irqreturn_t smb329b_vbus_det_irq(int irq, void *dev_id)
{
       printk(KERN_ERR "[IS3_POWER] smb329b_vbus_det_irq()\n");
       qci_schedule_work(3,&smb329b_chip->vbus_work);

	return IRQ_HANDLED;
}

static void smb329b_cradle_draw(unsigned int mA, int delay)
{
       input_mA = mA;
	if (mA == 0)
	{
             cradle_connect = 0;
	}
	else
	{
             cradle_connect = 1;
	}
	if (delay == 0)
	{
	   printk(KERN_ERR "[IS3_POWER] << smb329b_cradle_draw -- call det_work >>\n");
          qci_schedule_work(7,&smb329b_chip->det_work);
	}
	else
	{
	   printk(KERN_ERR "[IS3_POWER] << smb329b_cradle_draw -- call det_delay_work >>\n");
          qci_schedule_delayed_work(16,&smb329b_chip->det_delay_work,msecs_to_jiffies(500));
	}
}

void check_cradle_status(void)
{
       int val;

	val =  gpio_get_value_cansleep(smb329b_chip->xcradle_state_gpio);
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] xcradle_state_gpio = %d\n",val);

	if (val == 1)
	{
             printk(KERN_ERR ">>>>>> cradle disconnect\n");
	      smb329b_cradle_draw(0,0);
	}
	else
	{
	      printk(KERN_ERR ">>>>>> cradle connect\n");

             if (vbus_connect == 1)
		  power_source_switch_wait = 1;

	      if (cradle_mode == 0)
	          smb329b_cradle_draw(500,0);
	      else
		   smb329b_cradle_draw(1000,0);
	}
}

static void smb329b_cradle_work(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_cradle_work - start >>\n");
       check_cradle_status();
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_cradle_work - end >>\n");
}

static irqreturn_t smb329b_cradle_det_irq(int irq, void *dev_id)
{
	printk(KERN_ERR "[IS3_POWER] smb329b_cradle_det_irq()\n");
	qci_schedule_work(0,&smb329b_chip->cradle_work);

	return IRQ_HANDLED;
}

static int check_ovp1_status(void)
{
      int val1;

      val1 =  gpio_get_value_cansleep(smb329b_chip->ovp_sw1_off_gpio);

      if (val1 != 0)
      {
            mdelay(10);
	     val1 =  gpio_get_value_cansleep(smb329b_chip->ovp_sw1_off_gpio);
      }

      printk(KERN_ERR "[IS3_POWER] ovp_sw1_off_gpio = %d,voltage=%d\n",val1,msm_batt_info.battery_cradle_mv);

      if (val1 != 0)
      {
	    printk(KERN_ERR "[IS3_POWER] smb329b_ovp1_det_irq - OVP\n");
	    msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
           DBG_BUILD("[IS3_POWER_DEBUG-27] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	    send_chg_err_to_ap(CHG_ST_NTFY_CODE,K_CHG_ST_OVP);
	    return 1;
      }

      return 0;
}

static void smb329b_ovp1_work(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_ovp1_work - start >>\n");
       check_ovp1_status();
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_ovp1_work - end >>\n");
}

static irqreturn_t smb329b_ovp1_det_irq(int irq, void *dev_id)
{
	printk(KERN_ERR "[IS3_POWER] smb329b_ovp1_det_irq()\n");
       qci_schedule_work(1,&smb329b_chip->ovp1_work);

	return IRQ_HANDLED;
}

static int check_ovp2_status(void)
{
       int val2;

	val2 =  gpio_get_value_cansleep(smb329b_chip->ovp_sw2_off_gpio);

       if (val2 != 0)
       {
            mdelay(10);
	     val2 =  gpio_get_value_cansleep(smb329b_chip->ovp_sw2_off_gpio);
       }

	printk(KERN_ERR "[IS3_POWER] ovp_sw2_off_gpio = %d,voltage=%d\n",val2,msm_batt_info.battery_vbus_mv);

	if (val2 != 0)
	{
	    printk(KERN_ERR "[IS3_POWER] smb329b_ovp2_det_irq - OVP\n");
	    msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	    DBG_BUILD("[IS3_POWER_DEBUG-28] msm_batt_info.batt_status=%d\n",msm_batt_info.batt_status);
	    send_chg_err_to_ap(CHG_ST_NTFY_CODE,K_CHG_ST_OVP);
	    return 1;
	}

	return 0;
}

static void smb329b_ovp2_work(struct work_struct *work)
{
       DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_ovp2_work - start >>\n");
       check_ovp2_status();
	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << smb329b_ovp2_work - end >>\n");
}

static irqreturn_t smb329b_ovp2_det_irq(int irq, void *dev_id)
{
	printk(KERN_ERR "[IS3_POWER] smb329b_ovp2_det_irq()\n");
       qci_schedule_work(2,&smb329b_chip->ovp2_work);

	return IRQ_HANDLED;
}

static int __devinit smb329b_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	int ret = 0;
	const struct smb329b_platform_data *pdata;
	struct msm_batt_get_val_ret_data vbatt_data;
	int i = 0;
	int ovp_err = 0;

	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << probe - start >>\n");

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		ret = -EINVAL;
		goto out;
	}

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		ret = -EIO;
		goto out;
	}

	smb329b_chip = kzalloc(sizeof(*smb329b_chip), GFP_KERNEL);
	if (!smb329b_chip) {
		ret = -ENOMEM;
		goto out;
	}

	smb329b_chip->client = client;
	smb329b_chip->charger_state_gpio = pdata->charger_state_gpio;
	smb329b_chip->ovp_sw1_off_gpio = pdata->ovp_sw1_off_gpio;
	smb329b_chip->ovp_sw2_off_gpio = pdata->ovp_sw2_off_gpio;
	smb329b_chip->xcradle_state_gpio = pdata->xcradle_state_gpio;
	smb329b_chip->usb_vbus_mpp = pdata->usb_vbus_mpp;

       if (!(max17040gt_ready != CHIP_STATUS_NONE && rpc_ready == 1))
       {
          i = 1;
          mdelay(100);
       }

	if (max17040gt_ready != CHIP_STATUS_INIT_OK)
	   printk(KERN_ERR "[IS3_POWER] wait max17040gt_ready timout (%d).\n",i);
       else
	   printk(KERN_ERR "[IS3_POWER] wait max17040gt_ready ok (%d).\n",i);
	if (rpc_ready  != 1)
	   printk(KERN_ERR "[IS3_POWER] wait rpc_ready timout (%d).\n",i);
       else
	   printk(KERN_ERR "[IS3_POWER] wait rpc_ready ok (%d).\n",i);

	ret = gpio_request(smb329b_chip->xcradle_state_gpio, "smb329b_xcradle_state");
	if (ret) {
		printk(KERN_ERR "[IS3_POWER] xcradle_state_gpio_request = %d",ret);
		goto out2;
	}
	ret = gpio_request(smb329b_chip->usb_vbus_mpp, "smb329b_usb_vbus");
	if (ret) {
		printk(KERN_ERR "[IS3_POWER] usb_vbus_mpp_request = %d",ret);
		goto out2;
	}
	ret = gpio_request(smb329b_chip->ovp_sw1_off_gpio, "smb329b_ovp_sw1");
	if (ret) {
		printk(KERN_ERR "[IS3_POWER] ovp_sw1_off_gpio_request = %d",ret);
		goto out2;
	}
	ret = gpio_request(smb329b_chip->ovp_sw2_off_gpio, "smb329b_ovp_sw2");
	if (ret) {
		printk(KERN_ERR "[IS3_POWER] ovp_sw2_off_gpio = %d",ret);
		goto out2;
	}

	i2c_set_clientdata(client, smb329b_chip);

	wake_lock_init(&smb329b_chip->wlock,WAKE_LOCK_SUSPEND, "charging_active");
	wake_lock_init(&smb329b_chip->thermal_lock,WAKE_LOCK_SUSPEND, "thermal_error_active");

       ret = 0;
	ret += is2_smbus_write_byte(client, SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
       ret += is2_smbus_write_byte(client,SMB329B_PIN_CTRL_REG, SMB329B_PIN_CTRL_REG_CHG_OFF);
	if (ret != 0) {
	     ret = 0;
	     ret += is2_smbus_write_byte(client, SMB329B_COMMAND_A_REG, SMB329B_COMMAND_A_REG_DEFAULT);
            ret += is2_smbus_write_byte(client,SMB329B_PIN_CTRL_REG, SMB329B_PIN_CTRL_REG_CHG_OFF);
	}
	if (ret != 0) {
	   smb329b_ready = CHIP_STATUS_INIT_FAIL;
	   goto out1;
	}
	mdelay(100);
	// first get battery info
       vbatt_data = update_batt_mv_temp_RPC(READ_RPC_MODE,1);	
	if (re_load_custom_model == 1)
	{
	   printk(KERN_ERR "[IS3_POWER] call max17040gt_quick_start()\n");
	   max17040gt_quick_start(max17040gt_chip->client);
#if MAX17040GT_ENABLE_UPDATE_RCOMP
          if (vbatt_data.battery_temp != 20 && vbatt_data.battery_conn == 1)
          {
             max17040gt_update_RCOMP(max17040gt_chip->client,(int)vbatt_data.battery_temp);
          }
#else
          max17040gt_update_RCOMP(max17040gt_chip->client,(int)msm_batt_info.battery_temp);
#endif
	}
	max17040gt_get_vcell(max17040gt_chip->client);
	max17040gt_get_soc(max17040gt_chip->client);
       msm_batt_info.battery_voltage = (u32)max17040gt_chip->vcell;

       msm_batt_info.batt_capacity = (u32)max17040gt_chip->soc;
	msm_batt_info.batt_capacity = batt_capacity_format_check(msm_batt_info.batt_capacity);
	if ((vbatt_max_voltage_mode != 1 && msm_batt_info.batt_capacity == 100 && msm_batt_info.battery_voltage <= 4250) ||
	     (vbatt_max_voltage_mode == 1 && msm_batt_info.batt_capacity == 100 && msm_batt_info.battery_voltage <= 4020) )
	{
	     printk(KERN_ERR "[IS3_POWER] call max17040gt_quick_start(), vbatt_max_voltage_mode=%d\n",vbatt_max_voltage_mode);
	     printk(KERN_ERR "[IS3_POWER] call max17040gt_quick_start(), battery_voltage=%d,%d\n",
                                                msm_batt_info.battery_voltage,msm_batt_info.batt_capacity);
            max17040gt_load_custom_model(max17040gt_chip->client,0);
	     max17040gt_quick_start(max17040gt_chip->client);
#if MAX17040GT_ENABLE_UPDATE_RCOMP
            if (vbatt_data.battery_temp != 20 && vbatt_data.battery_conn == 1)
            {
               max17040gt_update_RCOMP(max17040gt_chip->client,(int)vbatt_data.battery_temp);
            }
#else
             max17040gt_update_RCOMP(max17040gt_chip->client,(int)msm_batt_info.battery_temp);
#endif
	     max17040gt_get_vcell(max17040gt_chip->client);
	     max17040gt_get_soc(max17040gt_chip->client);
            msm_batt_info.battery_voltage = (u32)max17040gt_chip->vcell;
            msm_batt_info.batt_capacity = (u32)max17040gt_chip->soc;
	     msm_batt_info.batt_capacity = batt_capacity_format_check(msm_batt_info.batt_capacity);
	}

	msm_batt_info.battery_conn = vbatt_data.battery_conn;
	msm_batt_info.battery_temp = vbatt_data.battery_temp;
	msm_batt_info.battery_temp_raw= vbatt_data.battery_temp_raw;
	if (vbatt_data.battery_conn == 0)
	{
	   msm_batt_info.chg_flow_status_now = CHG_FLOW_STATUS_A;
	   DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] CHG_FLOW_STATUS = %s\n",chg_flow_status_text[CHG_FLOW_STATUS_A]);
	}
	get_batt_first_info = 1;

	printk(KERN_ERR "[IS3_POWER][DEF] battery_voltage=%d\n",msm_batt_info.battery_voltage);
	printk(KERN_ERR "[IS3_POWER][DEF] batt_capacity=%d\n",msm_batt_info.batt_capacity);
	printk(KERN_ERR "[IS3_POWER][DEF] battery_temp=%d,%d\n",msm_batt_info.battery_temp,msm_batt_info.battery_temp_raw);
	printk(KERN_ERR "[IS3_POWER][DEF] battery_conn=%d\n",msm_batt_info.battery_conn);

       INIT_WORK(&smb329b_chip->det_work, smb329b_det_work);
	INIT_WORK(&smb329b_chip->det_work2, smb329b_det_work2);
	INIT_WORK(&smb329b_chip->det_work3, smb329b_det_work3);
	INIT_WORK(&smb329b_chip->cradle_work, smb329b_cradle_work);
	INIT_WORK(&smb329b_chip->vbus_work, smb329b_vbus_work);
	INIT_WORK(&smb329b_chip->ovp1_work, smb329b_ovp1_work);
	INIT_WORK(&smb329b_chip->ovp2_work, smb329b_ovp2_work);
	INIT_DELAYED_WORK_DEFERRABLE(&smb329b_chip->det_delay_work, smb329b_det_delay_work);
	INIT_DELAYED_WORK_DEFERRABLE(&smb329b_chip->temp_work, smb329b_temp_work);
	INIT_DELAYED_WORK_DEFERRABLE(&smb329b_chip->wait_exit_error_work, smb329b_wait_exit_error_work);
	INIT_DELAYED_WORK_DEFERRABLE(&smb329b_chip->wait_ap_update_work, smb329b_wait_ap_update_work);
	INIT_DELAYED_WORK_DEFERRABLE(&smb329b_chip->stop_charging_ic_work, smb329b_stop_charging_ic_work);
	INIT_DELAYED_WORK_DEFERRABLE(&smb329b_chip->send_keycode_work,smb329b_send_keycode_work);
	INIT_DELAYED_WORK_DEFERRABLE(&thermal_msm_work, thermal_msm_overheat_work);
	INIT_DELAYED_WORK_DEFERRABLE(&thermal_msm_work2, thermal_msm_overheat_work2);
	if (qci_get_offline_charging() == 1 || qci_get_recovery_charging() == 1)
	{
	   INIT_DELAYED_WORK_DEFERRABLE(&smb329b_chip->offline_charging_work, smb329b_offline_charging_work);
	   INIT_DELAYED_WORK_DEFERRABLE(&smb329b_chip->led_red_blink_work, smb329b_led_red_blink_work);
	   SMB320B_ENABLE_RE_CHARGE = SMB320B_ENABLE_RE_CHARGE3;
	}

	smb329b_ready = CHIP_STATUS_INIT_OK;

	smb329b_chip->irq = gpio_to_irq(smb329b_chip->xcradle_state_gpio);
	if (smb329b_chip->irq < 0) {
		printk(KERN_ERR "[IS3_POWER] Unable to get irq number \n");
		goto out2;
	}
	ret = request_irq(smb329b_chip->irq,\
		                     &smb329b_cradle_det_irq,\
		                     IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,\
		                     "xcradle_state",\
		                     smb329b_chip);
	if (ret)
	{
	      printk(KERN_ERR "[IS3_POWER] request_irq = %d",ret);
             goto out2;
	}
	ret = enable_irq_wake(smb329b_chip->irq);
	if (ret)
	{
		printk("%s: enable_irq_wake(%d) failed for (%d)\n",  __func__, smb329b_chip->irq, ret);
		goto out2;
	}

	smb329b_chip->irq3 = gpio_to_irq(smb329b_chip->usb_vbus_mpp);
	if (smb329b_chip->irq3 < 0) {
		printk(KERN_ERR "[IS3_POWER] Unable to get irq3 number \n");
		goto out2;
	}
	ret = request_irq(smb329b_chip->irq3,\
		                     &smb329b_vbus_det_irq,\
		                     IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,\
		                     "vbus_state",\
		                     smb329b_chip);
	if (ret)
	{
	      printk(KERN_ERR "[IS3_POWER] request_irq = %d",ret);
             goto out2;
	}

	smb329b_chip->irq1 = gpio_to_irq(smb329b_chip->ovp_sw1_off_gpio);
	if (smb329b_chip->irq1 < 0) {
		printk(KERN_ERR "[IS3_POWER] Unable to get irq1 number \n");
		goto out2;
	}
	ret = request_irq(smb329b_chip->irq1,\
		                     &smb329b_ovp1_det_irq,\
		                     IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,\
		                     "xcradle_state",\
		                     smb329b_chip);
	if (ret)
	{
	      printk(KERN_ERR "[IS3_POWER] request_irq1 = %d",ret);
             goto out2;
	}
	ret = enable_irq_wake(smb329b_chip->irq1);
	if (ret)
	{
		printk("%s: enable_irq1_wake(%d) failed for (%d)\n",  __func__, smb329b_chip->irq1, ret);
		goto out2;
	}

	smb329b_chip->irq2 = gpio_to_irq(smb329b_chip->ovp_sw2_off_gpio);
	if (smb329b_chip->irq2 < 0) {
		printk(KERN_ERR "[IS3_POWER] Unable to get irq2 number \n");
		goto out2;
	}
	ret = request_irq(smb329b_chip->irq2,\
		                     &smb329b_ovp2_det_irq,\
		                     IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,\
		                     "xcradle_state",\
		                     smb329b_chip);
	if (ret)
	{
	      printk(KERN_ERR "[IS3_POWER] request_irq2 = %d",ret);
             goto out2;
	}
	ret = enable_irq_wake(smb329b_chip->irq2);
	if (ret)
	{
		printk("%s: enable_irq2_wake(%d) failed for (%d)\n",  __func__, smb329b_chip->irq2, ret);
		goto out2;
	}

	if (msm_batt_info.chg_flow_status_now != CHG_FLOW_STATUS_A ||
	    msm_batt_info.battery_conn != 0)
       {
            ovp_err += check_ovp1_status();
	     ovp_err += check_ovp2_status();
	     if (ovp_err != 0)
	     {
	        first_ovp_check = 10;
	        check_ovp_again = 1;
		 batt_work_disable = 1;
	        qci_cancel_delayed_work(0,&batt_work_on,&max17040gt_chip->work,1);
		 batt_work_disable = 0;
	        qci_schedule_delayed_work(0,&max17040gt_chip->work, MAX17040GT_DELAY_QUICK);
	     }
            else
            {
               if (check_cradle_is_connect() == 1)
               {
                   printk(KERN_ERR ">>>>>> cradle connect\n");
		     first_ovp_check = 10;
	            if (check_vbus_is_connect() == 1)
	            {
	                power_source_switch_wait = 1;
	            }
	            if (cradle_mode == 0)
	                smb329b_cradle_draw(500,0);
	            else
		         smb329b_cradle_draw(1000,0);
               }
	        else if (check_vbus_is_connect() == 1)
	        {
	            printk(KERN_ERR ">>>>>> USB connect\n");
		     first_ovp_check = 10;
	            if (vbus_type < 1)
	                smb329b_vbus_draw2(500,0);
	            else
		         smb329b_vbus_draw2(1000,0);
	        }
	     }
	}

out2:
	smb329b_ready = CHIP_STATUS_INIT_OK;

out1:

	if (qci_get_offline_charging() == 1 || qci_get_recovery_charging() == 1)
	{
	    qci_schedule_delayed_work(5,&smb329b_chip->offline_charging_work, SMB329B_OFFLINE_CHARGING_DELAY);
	}

	DBG_LIMIT(debug_level,TYPE_SMB329B,"[SMB329B] << probe - end >>\n");

	return 0;

out:
	smb329b_ready = CHIP_STATUS_INIT_FAIL;
	free_irq(smb329b_chip->irq, smb329b_chip);
	gpio_free(smb329b_chip->xcradle_state_gpio);
	return ret;
}

static int __devexit smb329b_remove(struct i2c_client *client)
{
	struct smb329b_chip *chip_data = i2c_get_clientdata(client);

	free_irq(chip_data->irq, chip_data);
	gpio_free(chip_data->xcradle_state_gpio);
	kfree(chip_data);
	return 0;
}

static const struct i2c_device_id smb329b_id[] = {
	{"smb329b", 0},
	{},
};

static struct i2c_driver smb329b_driver = {
	.driver = {
		   .name = "smb329b",
	},
	.probe = smb329b_probe,
	.remove = __devexit_p(smb329b_remove),
	.id_table = smb329b_id,
};

static int __init is3_power_init(void)
{
	int rc;

	pr_debug("%s %s : enter\n", __func__,DRIVER_VER);

	rc = msm_batt_init_rpc();

	if (rc < 0) {
		pr_err("%s: FAIL: msm_batt_init_rpc.  rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	pr_info("%s: Charger/Battery = 0x%08x/0x%08x (RPC version)\n",
		__func__, msm_batt_info.chg_api_version,
		msm_batt_info.batt_api_version);

	mutex_init(&det_lock);
	mutex_init(&usb_type_lock);
	max17040gt_delay_time = MAX17040GT_DELAY_NORMAL;
	batt_chg_work_queue = create_workqueue("batt_chg_wq");

	rc = i2c_add_driver(&max17040gt_i2c_driver);
       if (rc < 0) {
		pr_err("%s: FAIL: i2c_add_driver.  rc=%d\n", __func__, rc);
		return rc;
	}

	rc = i2c_add_driver(&smb329b_driver);
       if (rc < 0) {
		pr_err("%s: FAIL: i2c_add_driver.  rc=%d\n", __func__, rc);
		return rc;
	}

	return 0;
}

static void __exit is3_power_exit(void)
{
	platform_driver_unregister(&msm_batt_driver);
       if (batt_chg_work_queue)
            destroy_workqueue(batt_chg_work_queue);
       i2c_del_driver(&smb329b_driver);
       i2c_del_driver(&max17040gt_i2c_driver);
}

module_init(is3_power_init);
module_exit(is3_power_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Battery/Charger driver for Qualcomm MSM chipsets and MAX17040_G+T and SUMB329B on IS3 Devices.");
MODULE_VERSION("1.0");
