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
#ifndef __IS3_SMB329B_H__
#define __IS3_SMB329B_H__

#define ISDB_T_SOLUTION	0
#define QCI_CHG_SOLUTION	0
#define SANYO_SOLUTION        1

//-----------------------------------------------------------
// SMB329B REGISTERS
//-----------------------------------------------------------
#define SMB329B_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))

#define SMB329B_CHG_CURRENT_REG			0x00
#define SMB329B_FAST_CHG_CURRENT_MASK	SMB329B_MASK(3, 5)
#define SMB329B_PRE_CHG_CURRENT_MASK	SMB329B_MASK(2, 3)
#define SMB329B_TERM_CHG_CURRENT_MASK	SMB329B_MASK(2, 1)

#define SMB329B_INPUT_CURRENT_LIMIT_REG			0x01
#define SMB329B_AC_IN_CURRENT_LIMIT_MASK		SMB329B_MASK(3, 5)
#define SMB329B_USB_IN_CURRENT_LIMIT_BIT			BIT(4)
#define SMB329B_AUTO_INPUT_CURRENT_LIMIT_BIT	BIT(2)
#define SMB329B_AUTO_INPUT_CURRENT_LIMIT_MASK 	SMB329B_MASK(2, 0)

#define SMB329B_FLOAT_VOLTAGE_REG				0x02
#define SMB329B_INTERNAL_TEMPERATURE_LIMIT_BIT	BIT(7)
#define SMB329B_FLOAT_VOLTAGE_MASK				SMB329B_MASK(7, 0)

#define SMB329B_CONTROL_A_REG					0x03
#define SMB329B_AUTO_RECHARGE_BIT			BIT(7)
#define SMB329B_CURR_TERMINATION_BIT			BIT(6)
#define SMB329B_PRE_TO_FAST_V_MASK			SMB329B_MASK(3, 3)
#define SMB329B_SYSON_UVLO_BIT				BIT(2)
#define SMB329B_AUTO_POWER_SOURCE_DET_BIT	BIT(1)
#define SMB329B_TRICKLE_CHG_BIT				BIT(0)

#define SMB329B_CONTROL_B_REG						0x04
#define SMB329B_STAT_OUTPUT_MODE_BIT			BIT(7)
#define SMB329B_BATT_OV_BIT						BIT(6)
#define SMB329B_AUTO_PRE_TO_FAST_THRESHOLD_BIT	BIT(4)
#define SMB329B_SAFETY_TIMER_BIT					BIT(3)
#define SMB329B_OTG_WD_TIMER_BIT					BIT(2)
#define SMB329B_CHG_WD_TIMER_BIT					BIT(1)
#define SMB329B_IRQ_OUT_BIT						BIT(0)

#define SMB329B_PIN_CTRL_REG					0x05
#define SMB329B_AUTO_CHG_CTRL_BIT			BIT(6)
#define SMB329B_DEAD_BATT_V_THRESHOLD_BIT	BIT(5)
#define SMB329B_SAFETY_TIMER_OPERATION_BIT	BIT(4)
#define SMB329B_EN_CTRL_MASK					SMB329B_MASK(2, 2)
#define SMB329B_OTG_USB5_AC_PIN_CTRL_MASK	SMB329B_MASK(2, 0)

#define SMB329B_OTG_CTRL_REG						0x06
#define SMB329B_BATT_MISSING_DET_BIT				BIT(7)
#define SMB329B_AUTO_RECHARGE_THRESHOLD_BIT	BIT(6)
#define SMB329B_SYSON_CTRL_BIT					BIT(5)
#define SMB329B_OTG_CURRENT_LIMIT_MASK			SMB329B_MASK(2, 3)
#define SMB329B_OTG_MODE_UVLO_THRESHOLD_MASK	SMB329B_MASK(3, 0)

#define SMB329B_CLEAR_IRQ_REG	0x30

#define SMB329B_COMMAND_A_REG			0x31
#define SMB329B_VOLATILE_WRITE_BIT		BIT(7)
#define SMB329B_POR_BIT					BIT(6)
#define SMB329B_FAST_CHG_SETTINGS_BIT	BIT(5)
#define SMB329B_BATT_CHG_EN_BIT			BIT(4)
#define SMB329B_USB_5_1_MODE_BIT			BIT(3)
#define SMB329B_USB_AC_MODE_BIT			BIT(2)
#define SMB329B_OTG_MODE_BIT				BIT(1)
#define SMB329B_STAT_OUTPUT_BIT			BIT(0)

#define SMB329B_COMMAND_BATT_REG	0x33
#define SMB329B_BATT_TOO_HOT_BIT		BIT(1)
#define SMB329B_BATT_TOO_COLD_BIT	BIT(0)

#define SMB329B_POWER_SOURCE_DET_STATUS_REG		0x34
#define SMB329B_AC_IN_CURRENT_LIMIT_STATUS_MASK	SMB329B_MASK(4, 4)
#define SMB329B_POWER_SOURCE_DET_FUN_STATUS_BIT	BIT(2)
#define SMB329B_POWER_SOURCE_DET_STATUS_BIT		BIT(1)
#define SMB329B_POWER_SOURCE_TYPE_BIT				BIT(0)

#define SMB329B_BATT_STATUS_A_REG					0x35
#define SMB329B_STATE_OTG_USB5_AC_PIN_BIT			BIT(6)
#define SMB329B_USB_5_1_MODE_STATUS_BIT				BIT(5)
#define SMB329B_USB_AC_MODE_STATUS_BIT				BIT(4)
#define SMB329B_RECHG_IRQ_STATUS_BIT					BIT(3)
#define SMB329B_INTERNAL_TEMP_LIMIT_REACHED_BIT		BIT(2)
#define SMB329B_OTG_MODE_IN_PROGRESS_BIT			BIT(1)
#define SMB329B_OTG_BATT_UVLO_BIT					BIT(0)

#define SMB329B_STATUS_B_REG						0x36
#define SMB329B_AT_LEAST_ONE_CHARGE_CYCLE_BIT	BIT(7)
#define SMB329B_CHG_CURRENT_L_TERMINATION_BIT	BIT(6)
#define SMB329B_SAFEY_TIMER_STATUS_MASK			SMB329B_MASK(2, 4)
#define SMB329B_CHG_ERROR_IRQ_BIT					BIT(3)
#define SMB329B_CHG_STATUS_MASK					SMB329B_MASK(2, 1)
#define SMB329B_CHG_EN_DIS_BIT					BIT(0)

#define SMB329B_STATUS_C_REG						0x37
#define SMB329B_WD_INTERRUPT_BIT					BIT(7)
#define SMB329B_OTG_CURRENT_LIMIT_REACHED_BIT	BIT(6)
#define SMB329B_BATTERY_MISSING_BIT				BIT(4)
#define SMB329B_BATTERY_OVLO_BIT					BIT(3)
#define SMB329B_INPUT_VOLO_BIT					BIT(2)
#define SMB329B_DCIN_LOWER_VBATT_BIT				BIT(1)
#define SMB329B_TRICKLE_CHG_MODE_BIT				BIT(0)

struct smb329b_chip {
	struct i2c_client *client;
	struct delayed_work work;
	struct work_struct det_work;
	struct work_struct det_work2;
	struct work_struct det_work3;
	struct delayed_work det_delay_work;
	struct work_struct cradle_work;
	struct work_struct vbus_work;
	struct work_struct ovp1_work;
	struct work_struct ovp2_work;
	struct delayed_work temp_work;
	struct delayed_work wait_exit_error_work;
	struct delayed_work wait_ap_update_work;
	struct delayed_work offline_charging_work;
	struct delayed_work led_red_blink_work;
	struct delayed_work stop_charging_ic_work;
	struct delayed_work send_keycode_work;
	struct wake_lock	wlock;
	struct wake_lock	thermal_lock;
	int irq;
	int irq1;
	int irq2;
	int irq3;

	int chg_current;

	unsigned charger_state_gpio;
	unsigned ovp_sw1_off_gpio;
	unsigned ovp_sw2_off_gpio;
	unsigned xcradle_state_gpio;
	unsigned usb_vbus_mpp;
};

static int check_ovp1_status(void);
static int check_ovp2_status(void);
static void send_chg_err_to_ap(uint32_t key_code, uint32_t key_parm);
static void stop_chg_clear_batt_info(int send_keycode);
static u32 calculate_adc_capacity(u32 voltage);
static int smb329b_check_batt_is_full(void);
static int smb329b_read_reg(u8 addr,u8 *val);
static int smb329b_start_charging_reg_set(void);
static int smb329b_batt_re_charging(int mode,int first);
static int smb329b_batt_temp_resume_restart_charging(void);
static void smb329b_temp_work(struct work_struct *work);
static void smb329b_wait_exit_error_work(struct work_struct *work);
static void smb329b_wait_ap_update_work(struct work_struct *work);
static void smb329b_stop_charging_ic_work(struct work_struct *work);
static void smb329b_send_keycode_work(struct work_struct *work);
static void smb329b_offline_charging_work(struct work_struct *work);
static void smb329b_led_red_blink_work(struct work_struct *work);
static int smb329b_is_charging(void);
static int smb329b_is_CV_mode(void);
static int smb329b_stop_charging(void);
static int smb329b_start_charging(int chg_current);
static int smb329b_change_float_voltage(u8 val);
static int smb329b_change_max_charging_current(int mode);
static int smb329b_change_cradle_current(int mode);
static int smb329b_ovp_path_switch_restart_charging(void);
static void smb329b_det_work(struct work_struct *work);
static void smb329b_det_delay_work(struct work_struct *work);
static void smb329b_det_work2(struct work_struct *work);
static void smb329b_det_work3(struct work_struct *work);
static void smb329b_vbus_work(struct work_struct *work);
static void smb329b_cradle_work(struct work_struct *work);
static void smb329b_ovp1_work(struct work_struct *work);
static void smb329b_ovp2_work(struct work_struct *work);
static int smb329b_stop_charging(void);
static void smb329b_vbus_draw2(unsigned int mA, int delay);
static void smb329b_cradle_draw(unsigned int mA, int delay);
static irqreturn_t smb329b_vbus_det_irq(int irq, void *dev_id);
static irqreturn_t smb329b_cradle_det_irq(int irq, void *dev_id);
static irqreturn_t smb329b_ovp1_det_irq(int irq, void *dev_id);
static irqreturn_t smb329b_ovp2_det_irq(int irq, void *dev_id);

#endif
