/******************************************************************************
 * Copyright (C) 2018 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fap.c
 * @brief FAP module
 *
 * Module for control of FAP power supplies. It implements the controller for
 * load current.
 *
 * @author gabriel.brunheira
 * @date 09/08/2018
 *
 */

#include <stdint.h>
#include <stdbool.h>

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
#include "communication_drivers/ps_modules/fap_4p/fap_4p.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/**
 * Controller defines
 */
#define I_LOAD_1                g_controller_ctom.net_signals[0]    // HRADC0
#define I_LOAD_2                g_controller_ctom.net_signals[1]    // HRADC1
#define V_LOAD                  g_controller_ctom.net_signals[2]    // HRADC2

#define I_LOAD_MEAN             g_controller_ctom.net_signals[3]
#define I_LOAD_ERROR            g_controller_ctom.net_signals[4]
#define DUTY_MEAN               g_controller_ctom.net_signals[5]

#define I_LOAD_DIFF             g_controller_ctom.net_signals[6]

#define I_MOD_1                 g_controller_ctom.net_signals[7]
#define I_MOD_2                 g_controller_ctom.net_signals[8]
#define I_MOD_3                 g_controller_ctom.net_signals[9]
#define I_MOD_4                 g_controller_ctom.net_signals[10]

#define I_MOD_MEAN              g_controller_ctom.net_signals[11]

#define I_MOD_1_DIFF            g_controller_ctom.net_signals[12]
#define I_MOD_2_DIFF            g_controller_ctom.net_signals[13]
#define I_MOD_3_DIFF            g_controller_ctom.net_signals[14]
#define I_MOD_4_DIFF            g_controller_ctom.net_signals[15]

#define I_IGBTS_DIFF_MOD_1      g_controller_ctom.net_signals[16]
#define I_IGBTS_DIFF_MOD_2      g_controller_ctom.net_signals[17]
#define I_IGBTS_DIFF_MOD_3      g_controller_ctom.net_signals[18]
#define I_IGBTS_DIFF_MOD_4      g_controller_ctom.net_signals[19]

#define DUTY_SHARE_MODULES_1    g_controller_ctom.net_signals[20]
#define DUTY_SHARE_MODULES_2    g_controller_ctom.net_signals[21]
#define DUTY_SHARE_MODULES_3    g_controller_ctom.net_signals[22]
#define DUTY_SHARE_MODULES_4    g_controller_ctom.net_signals[23]

#define DUTY_DIFF_MOD_1         g_controller_ctom.net_signals[24]
#define DUTY_DIFF_MOD_2         g_controller_ctom.net_signals[25]
#define DUTY_DIFF_MOD_3         g_controller_ctom.net_signals[26]
#define DUTY_DIFF_MOD_4         g_controller_ctom.net_signals[27]

#define I_IGBT_1_MOD_1          g_controller_mtoc.net_signals[0]    // ANI0
#define I_IGBT_2_MOD_1          g_controller_mtoc.net_signals[1]    // ANI1
#define I_IGBT_1_MOD_2          g_controller_mtoc.net_signals[2]    // ANI2
#define I_IGBT_2_MOD_2          g_controller_mtoc.net_signals[3]    // ANI3
#define I_IGBT_1_MOD_3          g_controller_mtoc.net_signals[4]    // ANI4
#define I_IGBT_2_MOD_3          g_controller_mtoc.net_signals[5]    // ANI5
#define I_IGBT_1_MOD_4          g_controller_mtoc.net_signals[6]    // ANI6
#define I_IGBT_2_MOD_4          g_controller_mtoc.net_signals[7]    // ANI7

#define V_DCLINK_MOD_1          g_controller_mtoc.net_signals[8]    // IIB 1
#define V_DCLINK_MOD_2          g_controller_mtoc.net_signals[9]    // IIB 2
#define V_DCLINK_MOD_3          g_controller_mtoc.net_signals[10]   // IIB 3
#define V_DCLINK_MOD_4          g_controller_mtoc.net_signals[11]   // IIB 4

#define IIB_V_IN_GLITCH             g_controller_mtoc.net_signals[12] // 0x0000C018
#define IIB_V_OUT_GLITCH            g_controller_mtoc.net_signals[13] // 0x0000C01A
#define IIB_I_IGBT_1_GLITCH         g_controller_mtoc.net_signals[14] // 0x0000C01C
#define IIB_I_IGBT_2_GLITCH         g_controller_mtoc.net_signals[15] // 0x0000C01E
#define IIB_TEMP_IGBT_1_GLITCH      g_controller_mtoc.net_signals[16] // 0x0000C020
#define IIB_TEMP_IGBT_2_GLITCH      g_controller_mtoc.net_signals[17] // 0x0000C022
#define IIB_V_DRIVER_GLITCH         g_controller_mtoc.net_signals[18] // 0x0000C024
#define IIB_I_DRIVER_1_GLITCH       g_controller_mtoc.net_signals[19] // 0x0000C026
#define IIB_I_DRIVER_2_GLITCH       g_controller_mtoc.net_signals[20] // 0x0000C028
#define IIB_TEMP_L_GLITCH           g_controller_mtoc.net_signals[21] // 0x0000C02A
#define IIB_TEMP_HEATSINK_GLITCH    g_controller_mtoc.net_signals[22] // 0x0000C02C
#define IIB_I_LEAKAGE_GLITCH        g_controller_mtoc.net_signals[23] // 0x0000C02E
#define IIB_TEMP_BOARD_GLITCH       g_controller_mtoc.net_signals[24] // 0x0000C030
#define IIB_RH_BOARD_GLITCH         g_controller_mtoc.net_signals[25] // 0x0000C032
#define IIB_ITLK_GLITCH             g_controller_mtoc.net_signals[26] // 0x0000C034
#define IIB_ALARM_GLITCH            g_controller_mtoc.net_signals[27] // 0x0000C036

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
    Load_Overvoltage,
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
    IIB_Mod_4_Itlk
} hard_interlocks_t;

typedef enum
{
    DCCT_1_Fault,
    DCCT_2_Fault,
    DCCT_High_Difference,
    Load_Feedback_1_Fault,
    Load_Feedback_2_Fault,
    IGBTs_Current_High_Difference
} soft_interlocks_t;

static volatile iib_fap_module_t iib_fap_4p[4];

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

    create_bsmp_var(36, 0, 4, false, V_LOAD.u8);

    create_bsmp_var(37, 0, 4, false, I_IGBT_1_MOD_1.u8);
    create_bsmp_var(38, 0, 4, false, I_IGBT_2_MOD_1.u8);
    create_bsmp_var(39, 0, 4, false, I_IGBT_1_MOD_2.u8);
    create_bsmp_var(40, 0, 4, false, I_IGBT_2_MOD_2.u8);
    create_bsmp_var(41, 0, 4, false, I_IGBT_1_MOD_3.u8);
    create_bsmp_var(42, 0, 4, false, I_IGBT_2_MOD_3.u8);
    create_bsmp_var(43, 0, 4, false, I_IGBT_1_MOD_4.u8);
    create_bsmp_var(44, 0, 4, false, I_IGBT_2_MOD_4.u8);

    create_bsmp_var(45, 0, 4, false, V_DCLINK_MOD_1.u8);
    create_bsmp_var(46, 0, 4, false, V_DCLINK_MOD_2.u8);
    create_bsmp_var(47, 0, 4, false, V_DCLINK_MOD_3.u8);
    create_bsmp_var(48, 0, 4, false, V_DCLINK_MOD_4.u8);

    create_bsmp_var(49, 0, 4, false, DUTY_MEAN.u8);
    create_bsmp_var(50, 0, 4, false, DUTY_CYCLE_IGBT_1_MOD_1.u8);
    create_bsmp_var(51, 0, 4, false, DUTY_CYCLE_IGBT_2_MOD_1.u8);
    create_bsmp_var(52, 0, 4, false, DUTY_CYCLE_IGBT_1_MOD_2.u8);
    create_bsmp_var(53, 0, 4, false, DUTY_CYCLE_IGBT_2_MOD_2.u8);
    create_bsmp_var(54, 0, 4, false, DUTY_CYCLE_IGBT_1_MOD_3.u8);
    create_bsmp_var(55, 0, 4, false, DUTY_CYCLE_IGBT_2_MOD_3.u8);
    create_bsmp_var(56, 0, 4, false, DUTY_CYCLE_IGBT_1_MOD_4.u8);
    create_bsmp_var(57, 0, 4, false, DUTY_CYCLE_IGBT_2_MOD_4.u8);

    create_bsmp_var(58, 0, 4, false, iib_fap_4p[0].Vin.u8);
    create_bsmp_var(59, 0, 4, false, iib_fap_4p[0].Vout.u8);
    create_bsmp_var(60, 0, 4, false, iib_fap_4p[0].IoutA1.u8);
    create_bsmp_var(61, 0, 4, false, iib_fap_4p[0].IoutA2.u8);
    create_bsmp_var(62, 0, 4, false, iib_fap_4p[0].TempIGBT1.u8);
    create_bsmp_var(63, 0, 4, false, iib_fap_4p[0].TempIGBT2.u8);
    create_bsmp_var(64, 0, 4, false, iib_fap_4p[0].DriverVoltage.u8);
    create_bsmp_var(65, 0, 4, false, iib_fap_4p[0].Driver1Current.u8);
    create_bsmp_var(66, 0, 4, false, iib_fap_4p[0].Driver2Current.u8);
    create_bsmp_var(67, 0, 4, false, iib_fap_4p[0].TempL.u8);
    create_bsmp_var(68, 0, 4, false, iib_fap_4p[0].TempHeatSink.u8);
    create_bsmp_var(69, 0, 4, false, iib_fap_4p[0].GroundLeakage.u8);
    create_bsmp_var(70, 0, 4, false, iib_fap_4p[0].BoardTemperature.u8);
    create_bsmp_var(71, 0, 4, false, iib_fap_4p[0].RelativeHumidity.u8);
    create_bsmp_var(72, 0, 4, false, iib_fap_4p[0].InterlocksRegister.u8);
    create_bsmp_var(73, 0, 4, false, iib_fap_4p[0].AlarmsRegister.u8);

    create_bsmp_var(74, 0, 4, false, iib_fap_4p[1].Vin.u8);
    create_bsmp_var(75, 0, 4, false, iib_fap_4p[1].Vout.u8);
    create_bsmp_var(76, 0, 4, false, iib_fap_4p[1].IoutA1.u8);
    create_bsmp_var(77, 0, 4, false, iib_fap_4p[1].IoutA2.u8);
    create_bsmp_var(78, 0, 4, false, iib_fap_4p[1].TempIGBT1.u8);
    create_bsmp_var(79, 0, 4, false, iib_fap_4p[1].TempIGBT2.u8);
    create_bsmp_var(80, 0, 4, false, iib_fap_4p[1].DriverVoltage.u8);
    create_bsmp_var(81, 0, 4, false, iib_fap_4p[1].Driver1Current.u8);
    create_bsmp_var(82, 0, 4, false, iib_fap_4p[1].Driver2Current.u8);
    create_bsmp_var(83, 0, 4, false, iib_fap_4p[1].TempL.u8);
    create_bsmp_var(84, 0, 4, false, iib_fap_4p[1].TempHeatSink.u8);
    create_bsmp_var(85, 0, 4, false, iib_fap_4p[1].GroundLeakage.u8);
    create_bsmp_var(86, 0, 4, false, iib_fap_4p[1].BoardTemperature.u8);
    create_bsmp_var(87, 0, 4, false, iib_fap_4p[1].RelativeHumidity.u8);
    create_bsmp_var(88, 0, 4, false, iib_fap_4p[1].InterlocksRegister.u8);
    create_bsmp_var(89, 0, 4, false, iib_fap_4p[1].AlarmsRegister.u8);

    create_bsmp_var(90, 0, 4, false, iib_fap_4p[2].Vin.u8);
    create_bsmp_var(91, 0, 4, false, iib_fap_4p[2].Vout.u8);
    create_bsmp_var(92, 0, 4, false, iib_fap_4p[2].IoutA1.u8);
    create_bsmp_var(93, 0, 4, false, iib_fap_4p[2].IoutA2.u8);
    create_bsmp_var(94, 0, 4, false, iib_fap_4p[2].TempIGBT1.u8);
    create_bsmp_var(95, 0, 4, false, iib_fap_4p[2].TempIGBT2.u8);
    create_bsmp_var(96, 0, 4, false, iib_fap_4p[2].DriverVoltage.u8);
    create_bsmp_var(97, 0, 4, false, iib_fap_4p[2].Driver1Current.u8);
    create_bsmp_var(98, 0, 4, false, iib_fap_4p[2].Driver2Current.u8);
    create_bsmp_var(99, 0, 4, false, iib_fap_4p[2].TempL.u8);
    create_bsmp_var(100, 0, 4, false, iib_fap_4p[2].TempHeatSink.u8);
    create_bsmp_var(101, 0, 4, false, iib_fap_4p[2].GroundLeakage.u8);
    create_bsmp_var(102, 0, 4, false, iib_fap_4p[2].BoardTemperature.u8);
    create_bsmp_var(103, 0, 4, false, iib_fap_4p[2].RelativeHumidity.u8);
    create_bsmp_var(104, 0, 4, false, iib_fap_4p[2].InterlocksRegister.u8);
    create_bsmp_var(105, 0, 4, false, iib_fap_4p[2].AlarmsRegister.u8);

    create_bsmp_var(106, 0, 4, false, iib_fap_4p[3].Vin.u8);
    create_bsmp_var(107, 0, 4, false, iib_fap_4p[3].Vout.u8);
    create_bsmp_var(108, 0, 4, false, iib_fap_4p[3].IoutA1.u8);
    create_bsmp_var(109, 0, 4, false, iib_fap_4p[3].IoutA2.u8);
    create_bsmp_var(110, 0, 4, false, iib_fap_4p[3].TempIGBT1.u8);
    create_bsmp_var(111, 0, 4, false, iib_fap_4p[3].TempIGBT2.u8);
    create_bsmp_var(112, 0, 4, false, iib_fap_4p[3].DriverVoltage.u8);
    create_bsmp_var(113, 0, 4, false, iib_fap_4p[3].Driver1Current.u8);
    create_bsmp_var(114, 0, 4, false, iib_fap_4p[3].Driver2Current.u8);
    create_bsmp_var(115, 0, 4, false, iib_fap_4p[3].TempL.u8);
    create_bsmp_var(116, 0, 4, false, iib_fap_4p[3].TempHeatSink.u8);
    create_bsmp_var(117, 0, 4, false, iib_fap_4p[3].GroundLeakage.u8);
    create_bsmp_var(118, 0, 4, false, iib_fap_4p[3].BoardTemperature.u8);
    create_bsmp_var(119, 0, 4, false, iib_fap_4p[3].RelativeHumidity.u8);
    create_bsmp_var(120, 0, 4, false, iib_fap_4p[3].InterlocksRegister.u8);
    create_bsmp_var(121, 0, 4, false, iib_fap_4p[3].AlarmsRegister.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fap_4p_system_config()
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
    iib_fap_4p[0].CanAddress = 1;
    iib_fap_4p[1].CanAddress = 2;
    iib_fap_4p[2].CanAddress = 3;
    iib_fap_4p[3].CanAddress = 4;

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
            memcpy(iib_fap_4p[module].Vin.u8, &data[0], 4);
            memcpy( (&V_DCLINK_MOD_1.f + module) , &data[0], 4);
            memcpy(iib_fap_4p[module].Vout.u8, &data[4], 4);

            if( (iib_fap_4p[module].Vin.f < -20.0) ||
                (iib_fap_4p[module].Vin.f > 150.0) )
            {
                IIB_V_IN_GLITCH.f = iib_fap_4p[module].Vin.f;
            }

            if( (iib_fap_4p[module].Vout.f < -20.0) ||
                (iib_fap_4p[module].Vout.f > 150.0) )
            {
                IIB_V_OUT_GLITCH.f = iib_fap_4p[module].Vout.f;
            }

            break;
        }
        case 1:
        {
        	memcpy(iib_fap_4p[module].IoutA1.u8, &data[0], 4);
        	memcpy(iib_fap_4p[module].IoutA2.u8, &data[4], 4);

            if( (iib_fap_4p[module].IoutA1.f < -20.0) ||
                (iib_fap_4p[module].IoutA1.f > 200.0) )
            {
                IIB_I_IGBT_1_GLITCH.f = iib_fap_4p[module].IoutA1.f;
            }

            if( (iib_fap_4p[module].IoutA2.f < -20.0) ||
                (iib_fap_4p[module].IoutA2.f > 200.0) )
            {
                IIB_I_IGBT_2_GLITCH.f = iib_fap_4p[module].IoutA2.f;
            }

            break;
        }
        case 2:
        {
        	memcpy(iib_fap_4p[module].DriverVoltage.u8, &data[0], 4);
        	memcpy(iib_fap_4p[module].GroundLeakage.u8, &data[4], 4);

        	if( (iib_fap_4p[module].DriverVoltage.f < -20.0) ||
                (iib_fap_4p[module].DriverVoltage.f > 50.0) )
            {
                IIB_V_DRIVER_GLITCH.f = iib_fap_4p[module].DriverVoltage.f;
            }

            if( (iib_fap_4p[module].GroundLeakage.f < -20.0) ||
                (iib_fap_4p[module].GroundLeakage.f > 50.0) )
            {
                IIB_I_LEAKAGE_GLITCH.f = iib_fap_4p[module].GroundLeakage.f;
            }

            break;
        }
        case 3:
        {
        	memcpy(iib_fap_4p[module].Driver1Current.u8, &data[0], 4);
        	memcpy(iib_fap_4p[module].Driver2Current.u8, &data[4], 4);

            if( (iib_fap_4p[module].Driver1Current.f < -50.0) ||
                (iib_fap_4p[module].Driver1Current.f > 50.0) )
            {
                IIB_I_DRIVER_1_GLITCH.f = iib_fap_4p[module].Driver1Current.f;
            }

            if( (iib_fap_4p[module].Driver2Current.f < -50.0) ||
                (iib_fap_4p[module].Driver2Current.f > 50.0) )
            {
                IIB_I_DRIVER_2_GLITCH.f = iib_fap_4p[module].Driver2Current.f;
            }

            break;
        }
        case 4:
        {
            memcpy(iib_fap_4p[module].TempIGBT1.u8, &data[0], 4);
            memcpy(iib_fap_4p[module].TempIGBT2.u8, &data[4], 4);

            if( (iib_fap_4p[module].TempIGBT1.f < -50.0) ||
                (iib_fap_4p[module].TempIGBT1.f > 150.0) )
            {
                IIB_TEMP_IGBT_1_GLITCH.f = iib_fap_4p[module].TempIGBT1.f;
            }

            if( (iib_fap_4p[module].TempIGBT2.f < -50.0) ||
                (iib_fap_4p[module].TempIGBT2.f > 150.0) )
            {
                IIB_TEMP_IGBT_1_GLITCH.f = iib_fap_4p[module].TempIGBT2.f;
            }

            break;
        }
        case 5:
        {
        	memcpy(iib_fap_4p[module].TempL.u8, &data[0], 4);
        	memcpy(iib_fap_4p[module].TempHeatSink.u8, &data[4], 4);

            if( (iib_fap_4p[module].TempL.f < -10.0) ||
                (iib_fap_4p[module].TempL.f > 100.0) )
            {
                IIB_TEMP_L_GLITCH.f = iib_fap_4p[module].TempL.f;
            }

            if( (iib_fap_4p[module].TempHeatSink.f < -10.0) ||
                (iib_fap_4p[module].TempHeatSink.f > 100.0) )
            {
                IIB_TEMP_HEATSINK_GLITCH.f = iib_fap_4p[module].TempHeatSink.f;
            }

            break;
        }
        case 6:
        {
        	memcpy(iib_fap_4p[module].BoardTemperature.u8, &data[0], 4);
        	memcpy(iib_fap_4p[module].RelativeHumidity.u8, &data[4], 4);

            if( (iib_fap_4p[module].BoardTemperature.f < -10.0) ||
                (iib_fap_4p[module].BoardTemperature.f > 150.0) )
            {
                IIB_TEMP_BOARD_GLITCH.f = iib_fap_4p[module].BoardTemperature.f;
            }

            if( (iib_fap_4p[module].RelativeHumidity.f < -10.0) ||
                (iib_fap_4p[module].RelativeHumidity.f > 100.0) )
            {
                IIB_RH_BOARD_GLITCH.f = iib_fap_4p[module].RelativeHumidity.f;
            }

            break;
        }
        case 7:
        {
        	memcpy(iib_fap_4p[module].InterlocksRegister.u8, &data[0], 4);
        	memcpy(iib_fap_4p[module].AlarmsRegister.u8, &data[4], 4);


            if(iib_fap_4p[module].InterlocksRegister.u32 > 0x000FFFFF)
            {
                IIB_ITLK_GLITCH.u32 = iib_fap_4p[module].InterlocksRegister.u32;
            }

            else if(iib_fap_4p[module].InterlocksRegister.u32 > 0)
        	{
        		set_hard_interlock(0, IIB_Mod_1_Itlk + module);
        	}

        	else
        	{
        		iib_fap_4p[module].InterlocksRegister.u32 = 0;
        	}

            if(iib_fap_4p[module].AlarmsRegister.u32 > 0x00003FFF)
            {
                IIB_ALARM_GLITCH.u32 = iib_fap_4p[module].AlarmsRegister.u32;
            }

        	break;
        }
        default:
        {
            break;
        }
    }
}
