/******************************************************************************
 * Copyright (C) 2019 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fap_2p2s.c
 * @brief FAP-2P2S module
 *
 * Module for control of FAP-2P2S power supplies. It implements the controller
 * for load current, current share between both parallel arms and current share
 * between 8 IGBT's.
 *
 * @author gabriel.brunheira
 * @date 17/04/2019
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/can/can_bkp.h"
#include "communication_drivers/control/control.h"
#include "communication_drivers/control/wfmref/wfmref.h"
#include "communication_drivers/event_manager/event_manager.h"
#include "communication_drivers/iib/iib_data.h"
#include "communication_drivers/iib/iib_module.h"
#include "communication_drivers/ps_modules/fap_2p2s/fap_2p2s.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/**
 * Controller defines
 */
#define I_LOAD_1                    g_controller_ctom.net_signals[0]  // HRADC0
#define I_LOAD_2                    g_controller_ctom.net_signals[1]  // HRADC1
#define I_ARM_1                     g_controller_ctom.net_signals[2]  // HRADC2
#define I_ARM_2                     g_controller_ctom.net_signals[3]  // HRADC3

#define I_LOAD_MEAN                 g_controller_ctom.net_signals[4]
#define I_LOAD_ERROR                g_controller_ctom.net_signals[5]
#define DUTY_MEAN                   g_controller_ctom.net_signals[6]

#define I_LOAD_DIFF                 g_controller_ctom.net_signals[7]

#define I_ARMS_DIFF                 g_controller_ctom.net_signals[8]
#define DUTY_ARMS_DIFF              g_controller_ctom.net_signals[9]

#define I_MOD_1                     g_controller_ctom.net_signals[10]
#define I_MOD_2                     g_controller_ctom.net_signals[11]
#define I_MOD_3                     g_controller_ctom.net_signals[12]
#define I_MOD_4                     g_controller_ctom.net_signals[13]

#define I_IGBTS_DIFF_MOD_1          g_controller_ctom.net_signals[14]
#define I_IGBTS_DIFF_MOD_2          g_controller_ctom.net_signals[15]
#define I_IGBTS_DIFF_MOD_3          g_controller_ctom.net_signals[16]
#define I_IGBTS_DIFF_MOD_4          g_controller_ctom.net_signals[17]

#define DUTY_IGBTS_DIFF_MOD_1       g_controller_ctom.net_signals[18]
#define DUTY_IGBTS_DIFF_MOD_2       g_controller_ctom.net_signals[19]
#define DUTY_IGBTS_DIFF_MOD_3       g_controller_ctom.net_signals[20]
#define DUTY_IGBTS_DIFF_MOD_4       g_controller_ctom.net_signals[21]

#define I_IGBT_1_MOD_1              g_controller_mtoc.net_signals[0]  // ANI0
#define I_IGBT_2_MOD_1              g_controller_mtoc.net_signals[1]  // ANI1
#define I_IGBT_1_MOD_2              g_controller_mtoc.net_signals[2]  // ANI2
#define I_IGBT_2_MOD_2              g_controller_mtoc.net_signals[3]  // ANI3
#define I_IGBT_1_MOD_3              g_controller_mtoc.net_signals[4]  // ANI4
#define I_IGBT_2_MOD_3              g_controller_mtoc.net_signals[5]  // ANI5
#define I_IGBT_1_MOD_4              g_controller_mtoc.net_signals[6]  // ANI6
#define I_IGBT_2_MOD_4              g_controller_mtoc.net_signals[7]  // ANI7

#define V_DCLINK_MOD_1              g_controller_mtoc.net_signals[8]  // IIB 1
#define V_DCLINK_MOD_2              g_controller_mtoc.net_signals[9]  // IIB 2
#define V_DCLINK_MOD_3              g_controller_mtoc.net_signals[10] // IIB 3
#define V_DCLINK_MOD_4              g_controller_mtoc.net_signals[11] // IIB 4

#define DUTY_CYCLE_IGBT_1_MOD_1     g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_IGBT_2_MOD_1     g_controller_ctom.output_signals[1]
#define DUTY_CYCLE_IGBT_1_MOD_2     g_controller_ctom.output_signals[2]
#define DUTY_CYCLE_IGBT_2_MOD_2     g_controller_ctom.output_signals[3]
#define DUTY_CYCLE_IGBT_1_MOD_3     g_controller_ctom.output_signals[4]
#define DUTY_CYCLE_IGBT_2_MOD_3     g_controller_ctom.output_signals[5]
#define DUTY_CYCLE_IGBT_1_MOD_4     g_controller_ctom.output_signals[6]
#define DUTY_CYCLE_IGBT_2_MOD_4     g_controller_ctom.output_signals[7]

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    IGBT_1_Mod_1_Overcurrent,
    IGBT_2_Mod_1_Overcurrent,
    IGBT_1_Mod_2_Overcurrent,
    IGBT_2_Mod_2_Overcurrent,
    IGBT_1_Mod_3_Overcurrent,
    IGBT_2_Mod_3_Overcurrent,
    IGBT_1_Mod_4_Overcurrent,
    IGBT_2_Mod_4_Overcurrent,
    Welded_Contactor_Mod_1_Fault,
    Welded_Contactor_Mod_2_Fault,
    Welded_Contactor_Mod_3_Fault,
    Welded_Contactor_Mod_4_Fault,
    Opened_Contactor_Mod_1_Fault,
    Opened_Contactor_Mod_2_Fault,
    Opened_Contactor_Mod_3_Fault,
    Opened_Contactor_Mod_4_Fault,
    DCLink_Mod_1_Overvoltage,
    DCLink_Mod_2_Overvoltage,
    DCLink_Mod_3_Overvoltage,
    DCLink_Mod_4_Overvoltage,
    DCLink_Mod_1_Undervoltage,
    DCLink_Mod_2_Undervoltage,
    DCLink_Mod_3_Undervoltage,
    DCLink_Mod_4_Undervoltage,
    IIB_Mod_1_Itlk,
    IIB_Mod_2_Itlk,
    IIB_Mod_3_Itlk,
    IIB_Mod_4_Itlk,
    ARM_1_Overcurrent,
    ARM_2_Overcurrent
} hard_interlocks_t;

typedef enum
{
    DCCT_1_Fault,
    DCCT_2_Fault,
    DCCT_High_Difference,
    Load_Feedback_1_Fault,
    Load_Feedback_2_Fault,
    Arms_High_Difference,
    IGBTs_Current_High_Difference,
    Complementary_PS_Itlk,
} soft_interlocks_t;

typedef enum
{
    High_Sync_Input_Frequency = 0x00000001
} alarms_t;

static volatile iib_fap_module_t iib_fap_2p2s[4];

static void init_iib();

static void handle_can_data(volatile uint8_t *data, volatile unsigned long id);

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC ACDC operation.
*
*/
static void adcp_channel_config(void)
{
    /// Module 1 IGBT 1 current: 10 V = 200 A
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 200.0/2048.0;
    g_analog_ch_0.Value = &(I_IGBT_1_MOD_1.f);

    /// Module 1 IGBT 2 current: 10 V = 200 A
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 200.0/2048.0;
    g_analog_ch_1.Value = &(I_IGBT_2_MOD_1.f);

    /// Module 2 IGBT 1 current: 10 V = 200 A
    g_analog_ch_2.Enable = 1;
    g_analog_ch_2.Gain = 200.0/2048.0;
    g_analog_ch_2.Value = &(I_IGBT_1_MOD_2.f);

    /// Module 2 IGBT 2 current: 10 V = 200 A
    g_analog_ch_3.Enable = 1;
    g_analog_ch_3.Gain = 200.0/2048.0;
    g_analog_ch_3.Value = &(I_IGBT_2_MOD_2.f);

    /// Module 3 IGBT 1 current: 10 V = 200 A
    g_analog_ch_4.Enable = 1;
    g_analog_ch_4.Gain = 200.0/2048.0;
    g_analog_ch_4.Value = &(I_IGBT_1_MOD_3.f);

    /// Module 3 IGBT 2 current: 10 V = 200 A
    g_analog_ch_5.Enable = 1;
    g_analog_ch_5.Gain = 200.0/2048.0;
    g_analog_ch_5.Value = &(I_IGBT_2_MOD_3.f);

    /// Module 4 IGBT 1 current: 10 V = 200 A
    g_analog_ch_6.Enable = 1;
    g_analog_ch_6.Gain = 200.0/2048.0;
    g_analog_ch_6.Value = &(I_IGBT_1_MOD_4.f);

    /// Module 4 IGBT 2 current: 10 V = 200 A
    g_analog_ch_7.Enable = 1;
    g_analog_ch_7.Gain = 200.0/2048.0;
    g_analog_ch_7.Value = &(I_IGBT_2_MOD_4.f);
}

/**
* @brief Initialize BSMP servers.
*
* Setup BSMP servers for FBP operation.
*
*/
static void bsmp_init_server(void)
{
    create_bsmp_var(31, 0, 4, false, g_ipc_ctom.ps_module[0].ps_soft_interlock.u8);
    create_bsmp_var(32, 0, 4, false, g_ipc_ctom.ps_module[0].ps_hard_interlock.u8);

    create_bsmp_var(33, 0, 4, false, I_LOAD_MEAN.u8);
    create_bsmp_var(34, 0, 4, false, I_LOAD_1.u8);
    create_bsmp_var(35, 0, 4, false, I_LOAD_2.u8);

    create_bsmp_var(36, 0, 4, false, I_ARM_1.u8);
    create_bsmp_var(37, 0, 4, false, I_ARM_2.u8);

    create_bsmp_var(38, 0, 4, false, I_IGBT_1_MOD_1.u8);
    create_bsmp_var(39, 0, 4, false, I_IGBT_2_MOD_1.u8);
    create_bsmp_var(40, 0, 4, false, I_IGBT_1_MOD_2.u8);
    create_bsmp_var(41, 0, 4, false, I_IGBT_2_MOD_2.u8);
    create_bsmp_var(42, 0, 4, false, I_IGBT_1_MOD_3.u8);
    create_bsmp_var(43, 0, 4, false, I_IGBT_2_MOD_3.u8);
    create_bsmp_var(44, 0, 4, false, I_IGBT_1_MOD_4.u8);
    create_bsmp_var(45, 0, 4, false, I_IGBT_2_MOD_4.u8);

    create_bsmp_var(46, 0, 4, false, I_MOD_1.u8);
    create_bsmp_var(47, 0, 4, false, I_MOD_2.u8);
    create_bsmp_var(48, 0, 4, false, I_MOD_3.u8);
    create_bsmp_var(49, 0, 4, false, I_MOD_4.u8);

    create_bsmp_var(50, 0, 4, false, V_DCLINK_MOD_1.u8);
    create_bsmp_var(51, 0, 4, false, V_DCLINK_MOD_2.u8);
    create_bsmp_var(52, 0, 4, false, V_DCLINK_MOD_3.u8);
    create_bsmp_var(53, 0, 4, false, V_DCLINK_MOD_4.u8);

    create_bsmp_var(54, 0, 4, false, DUTY_MEAN.u8);
    create_bsmp_var(55, 0, 4, false, DUTY_ARMS_DIFF.u8);
    create_bsmp_var(56, 0, 4, false, DUTY_CYCLE_IGBT_1_MOD_1.u8);
    create_bsmp_var(57, 0, 4, false, DUTY_CYCLE_IGBT_2_MOD_1.u8);
    create_bsmp_var(58, 0, 4, false, DUTY_CYCLE_IGBT_1_MOD_2.u8);
    create_bsmp_var(59, 0, 4, false, DUTY_CYCLE_IGBT_2_MOD_2.u8);
    create_bsmp_var(60, 0, 4, false, DUTY_CYCLE_IGBT_1_MOD_3.u8);
    create_bsmp_var(61, 0, 4, false, DUTY_CYCLE_IGBT_2_MOD_3.u8);
    create_bsmp_var(62, 0, 4, false, DUTY_CYCLE_IGBT_1_MOD_4.u8);
    create_bsmp_var(63, 0, 4, false, DUTY_CYCLE_IGBT_2_MOD_4.u8);

    create_bsmp_var(64, 0, 4, false, iib_fap_2p2s[0].Vin.u8);
    create_bsmp_var(65, 0, 4, false, iib_fap_2p2s[0].Vout.u8);
    create_bsmp_var(66, 0, 4, false, iib_fap_2p2s[0].IoutA1.u8);
    create_bsmp_var(67, 0, 4, false, iib_fap_2p2s[0].IoutA2.u8);
    create_bsmp_var(68, 0, 4, false, iib_fap_2p2s[0].TempIGBT1.u8);
    create_bsmp_var(69, 0, 4, false, iib_fap_2p2s[0].TempIGBT2.u8);
    create_bsmp_var(70, 0, 4, false, iib_fap_2p2s[0].DriverVoltage.u8);
    create_bsmp_var(71, 0, 4, false, iib_fap_2p2s[0].Driver1Current.u8);
    create_bsmp_var(72, 0, 4, false, iib_fap_2p2s[0].Driver2Current.u8);
    create_bsmp_var(73, 0, 4, false, iib_fap_2p2s[0].TempL.u8);
    create_bsmp_var(74, 0, 4, false, iib_fap_2p2s[0].TempHeatSink.u8);
    create_bsmp_var(75, 0, 4, false, iib_fap_2p2s[0].GroundLeakage.u8);
    create_bsmp_var(76, 0, 4, false, iib_fap_2p2s[0].BoardTemperature.u8);
    create_bsmp_var(77, 0, 4, false, iib_fap_2p2s[0].RelativeHumidity.u8);
    create_bsmp_var(78, 0, 4, false, iib_fap_2p2s[0].InterlocksRegister.u8);
    create_bsmp_var(79, 0, 4, false, iib_fap_2p2s[0].AlarmsRegister.u8);

    create_bsmp_var(80, 0, 4, false, iib_fap_2p2s[1].Vin.u8);
    create_bsmp_var(81, 0, 4, false, iib_fap_2p2s[1].Vout.u8);
    create_bsmp_var(82, 0, 4, false, iib_fap_2p2s[1].IoutA1.u8);
    create_bsmp_var(83, 0, 4, false, iib_fap_2p2s[1].IoutA2.u8);
    create_bsmp_var(84, 0, 4, false, iib_fap_2p2s[1].TempIGBT1.u8);
    create_bsmp_var(85, 0, 4, false, iib_fap_2p2s[1].TempIGBT2.u8);
    create_bsmp_var(86, 0, 4, false, iib_fap_2p2s[1].DriverVoltage.u8);
    create_bsmp_var(87, 0, 4, false, iib_fap_2p2s[1].Driver1Current.u8);
    create_bsmp_var(88, 0, 4, false, iib_fap_2p2s[1].Driver2Current.u8);
    create_bsmp_var(89, 0, 4, false, iib_fap_2p2s[1].TempL.u8);
    create_bsmp_var(90, 0, 4, false, iib_fap_2p2s[1].TempHeatSink.u8);

    //create_bsmp_var(91, 0, 4, false, iib_fap_2p2s[1].GroundLeakage.u8);
    create_bsmp_var(91, 0, 4, false, g_ipc_ctom.ps_module[0].ps_alarms.u8);

    create_bsmp_var(92, 0, 4, false, iib_fap_2p2s[1].BoardTemperature.u8);
    create_bsmp_var(93, 0, 4, false, iib_fap_2p2s[1].RelativeHumidity.u8);
    create_bsmp_var(94, 0, 4, false, iib_fap_2p2s[1].InterlocksRegister.u8);
    create_bsmp_var(95, 0, 4, false, iib_fap_2p2s[1].AlarmsRegister.u8);

    create_bsmp_var(96, 0, 4, false, iib_fap_2p2s[2].Vin.u8);
    create_bsmp_var(97, 0, 4, false, iib_fap_2p2s[2].Vout.u8);
    create_bsmp_var(98, 0, 4, false, iib_fap_2p2s[2].IoutA1.u8);
    create_bsmp_var(99, 0, 4, false, iib_fap_2p2s[2].IoutA2.u8);
    create_bsmp_var(100, 0, 4, false, iib_fap_2p2s[2].TempIGBT1.u8);
    create_bsmp_var(101, 0, 4, false, iib_fap_2p2s[2].TempIGBT2.u8);
    create_bsmp_var(102, 0, 4, false, iib_fap_2p2s[2].DriverVoltage.u8);
    create_bsmp_var(103, 0, 4, false, iib_fap_2p2s[2].Driver1Current.u8);
    create_bsmp_var(104, 0, 4, false, iib_fap_2p2s[2].Driver2Current.u8);
    create_bsmp_var(105, 0, 4, false, iib_fap_2p2s[2].TempL.u8);
    create_bsmp_var(106, 0, 4, false, iib_fap_2p2s[2].TempHeatSink.u8);
    create_bsmp_var(107, 0, 4, false, iib_fap_2p2s[2].GroundLeakage.u8);
    create_bsmp_var(108, 0, 4, false, iib_fap_2p2s[2].BoardTemperature.u8);
    create_bsmp_var(109, 0, 4, false, iib_fap_2p2s[2].RelativeHumidity.u8);
    create_bsmp_var(110, 0, 4, false, iib_fap_2p2s[2].InterlocksRegister.u8);
    create_bsmp_var(111, 0, 4, false, iib_fap_2p2s[2].AlarmsRegister.u8);

    create_bsmp_var(112, 0, 4, false, iib_fap_2p2s[3].Vin.u8);
    create_bsmp_var(113, 0, 4, false, iib_fap_2p2s[3].Vout.u8);
    create_bsmp_var(114, 0, 4, false, iib_fap_2p2s[3].IoutA1.u8);
    create_bsmp_var(115, 0, 4, false, iib_fap_2p2s[3].IoutA2.u8);
    create_bsmp_var(116, 0, 4, false, iib_fap_2p2s[3].TempIGBT1.u8);
    create_bsmp_var(117, 0, 4, false, iib_fap_2p2s[3].TempIGBT2.u8);
    create_bsmp_var(118, 0, 4, false, iib_fap_2p2s[3].DriverVoltage.u8);
    create_bsmp_var(119, 0, 4, false, iib_fap_2p2s[3].Driver1Current.u8);
    create_bsmp_var(120, 0, 4, false, iib_fap_2p2s[3].Driver2Current.u8);
    create_bsmp_var(121, 0, 4, false, iib_fap_2p2s[3].TempL.u8);
    create_bsmp_var(122, 0, 4, false, iib_fap_2p2s[3].TempHeatSink.u8);
    create_bsmp_var(123, 0, 4, false, iib_fap_2p2s[3].GroundLeakage.u8);
    create_bsmp_var(124, 0, 4, false, iib_fap_2p2s[3].BoardTemperature.u8);
    create_bsmp_var(125, 0, 4, false, iib_fap_2p2s[3].RelativeHumidity.u8);
    create_bsmp_var(126, 0, 4, false, iib_fap_2p2s[3].InterlocksRegister.u8);
    create_bsmp_var(127, 0, 4, false, iib_fap_2p2s[3].AlarmsRegister.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fap_2p2s_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
    init_iib();

    init_wfmref(&WFMREF[0], WFMREF_SELECTED_PARAM[0].u16,
                WFMREF_SYNC_MODE_PARAM[0].u16, ISR_CONTROL_FREQ.f,
                WFMREF_FREQUENCY_PARAM[0].f, WFMREF_GAIN_PARAM[0].f,
                WFMREF_OFFSET_PARAM[0].f, &g_wfmref_data.data[0][0].f,
                SIZE_WFMREF, &g_ipc_ctom.ps_module[0].ps_reference.f);

    init_scope(&g_ipc_mtoc.scope[0], ISR_CONTROL_FREQ.f,
               SCOPE_FREQ_SAMPLING_PARAM[0].f, &(g_buf_samples_ctom[0].f),
               SIZE_BUF_SAMPLES_CTOM, SCOPE_SOURCE_PARAM[0].p_f,
               (void *) 0);
}

static void init_iib()
{
    iib_fap_2p2s[0].CanAddress = 1;
    iib_fap_2p2s[1].CanAddress = 2;
    iib_fap_2p2s[2].CanAddress = 3;
    iib_fap_2p2s[3].CanAddress = 4;

    init_iib_module_can_data(&g_iib_module_can_data, &handle_can_data);
}

static void handle_can_data(volatile uint8_t *data, volatile unsigned long id)
{
    volatile uint8_t module;

    volatile unsigned long can_module = id;
    volatile unsigned long id_var = 0;

    switch(can_module)
    {
    	case 10:
    	case 11:
    	case 12:
    	case 13:
    	case 14:
    	case 15:
    	case 16:
    	case 17:
    	{
    		module = 0;
    		id_var = (id - 10);
    		break;
    	}
    	case 20:
    	case 21:
    	case 22:
    	case 23:
    	case 24:
    	case 25:
    	case 26:
    	case 27:
    	{
    		module = 1;
    		id_var = (id - 20);
    		break;
    	}
    	case 30:
    	case 31:
    	case 32:
    	case 33:
    	case 34:
    	case 35:
    	case 36:
    	case 37:
    	{
    		module = 2;
    		id_var = (id - 30);
    		break;
    	}
    	case 40:
    	case 41:
    	case 42:
    	case 43:
    	case 44:
    	case 45:
    	case 46:
    	case 47:
    	{
    		module = 3;
    		id_var = (id - 40);
    		break;
    	}

    	default:
    	{
    		break;
    	}
    }

    switch(id_var)
    {
        case 0:
        {
            memcpy((void *)iib_fap_2p2s[module].Vin.u8, (const void *)&data[0], (size_t)4);
            memcpy((void *)(&V_DCLINK_MOD_1.f + module), (const void *)&data[0], (size_t)4);
            memcpy((void *)iib_fap_2p2s[module].Vout.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 1:
        {
        	memcpy((void *)iib_fap_2p2s[module].IoutA1.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fap_2p2s[module].IoutA2.u8, (const void *)&data[4], (size_t)4);

        	break;
        }
        case 2:
        {
        	memcpy((void *)iib_fap_2p2s[module].DriverVoltage.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fap_2p2s[module].GroundLeakage.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 3:
        {
        	memcpy((void *)iib_fap_2p2s[module].Driver1Current.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fap_2p2s[module].Driver2Current.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 4:
        {
            memcpy((void *)iib_fap_2p2s[module].TempIGBT1.u8, (const void *)&data[0], (size_t)4);
            memcpy((void *)iib_fap_2p2s[module].TempIGBT2.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 5:
        {
        	memcpy((void *)iib_fap_2p2s[module].TempL.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fap_2p2s[module].TempHeatSink.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 6:
        {
        	memcpy((void *)iib_fap_2p2s[module].BoardTemperature.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fap_2p2s[module].RelativeHumidity.u8, (const void *)&data[4], (size_t)4);

        	break;
        }
        case 7:
        {
        	memcpy((void *)iib_fap_2p2s[module].InterlocksRegister.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fap_2p2s[module].AlarmsRegister.u8, (const void *)&data[4], (size_t)4);

        	if(iib_fap_2p2s[module].InterlocksRegister.u32 > 0)
        	{
        		set_hard_interlock(0, IIB_Mod_1_Itlk + module);
        	}

        	else
        	{
        		iib_fap_2p2s[module].InterlocksRegister.u32 = 0;
        	}

            break;
        }
        default:
        {
            break;
        }
    }
}
