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

#define IIB_ITLK_REG_MOD_1      g_controller_mtoc.net_signals[12]
#define IIB_ITLK_REG_MOD_2      g_controller_mtoc.net_signals[13]
#define IIB_ITLK_REG_MOD_3      g_controller_mtoc.net_signals[14]
#define IIB_ITLK_REG_MOD_4      g_controller_mtoc.net_signals[15]

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

static volatile iib_fap_module_t iib_fap_4p[4];
volatile hard_interlocks_t hard_interlocks;

static void init_iib();
static void handle_can_data(uint8_t *data);
static void update_iib_structure(iib_fap_module_t *module, uint8_t data_id,
                                                               float data_val);
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

    create_bsmp_var(33, 0, 4, false, I_LOAD_1.u8);
    create_bsmp_var(34, 0, 4, false, I_LOAD_2.u8);
    create_bsmp_var(35, 0, 4, false, I_LOAD_MEAN.u8);

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
    create_bsmp_var(129, 0, 4, false, iib_fap_4p[3].RelativeHumidity.u8);
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

    init_wfmref(&WFMREF[0], WFMREF[0].wfmref_selected.u16, WFMREF[0].sync_mode.enu,
                ISR_CONTROL_FREQ.f,
                TIMESLICER_FREQ[TIMESLICER_WFMREF].f,
                WFMREF[0].gain.f, WFMREF[0].offset.f, &g_wfmref_data.data[0][0].f,
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

static void handle_can_data(uint8_t *data)
{
    uint8_t iib_address;
    uint8_t data_id;

    convert_to_bytes_t converter;

    iib_address     = data[0];
    data_id         = data[1];

    converter.u8[0] = data[4];
    converter.u8[1] = data[5];
    converter.u8[2] = data[6];
    converter.u8[3] = data[7];

    update_iib_structure(&iib_fap_4p[iib_address - 1], data_id, converter.f);
}

static void update_iib_structure(iib_fap_module_t *module, uint8_t data_id,
                                                        float data_val)
{
    uint8_t id = data_id;
    convert_to_bytes_t converter;

    switch (id)
    {
        case 0:
            converter.f = data_val;

            if (module->CanAddress == 1) {
                IIB_ITLK_REG_MOD_1.u32 = converter.u32;
                set_hard_interlock(0, IIB_Mod_1_Itlk);
            }

            if (module->CanAddress == 2) {
                IIB_ITLK_REG_MOD_2.u32 = converter.u32;
                set_hard_interlock(0, IIB_Mod_2_Itlk);
            }

            if (module->CanAddress == 3) {
                IIB_ITLK_REG_MOD_3.u32 = converter.u32;
                set_hard_interlock(0, IIB_Mod_3_Itlk);
            }

            if (module->CanAddress == 4) {
                IIB_ITLK_REG_MOD_4.u32 = converter.u32;
                set_hard_interlock(0, IIB_Mod_4_Itlk);
            }

            break;

        case 1:
            // TODO: Handle alarm message
            break;
        case 2:
            module->Vin.f = data_val;

            /// Copy to shared memory for C28 access
            *(&V_DCLINK_MOD_1.f + module->CanAddress - 1) = data_val;
            break;

        case 3:
            module->Vout.f = data_val;
            break;

        case 4:
            module->IoutA1.f = data_val;
            break;

        case 5:
            module->IoutA2.f = data_val;
            break;

        case 6:
            module->TempIGBT1.f = data_val;
            break;

        case 7:
            module->TempIGBT2.f = data_val;
            break;

        case 8:
            module->DriverVoltage.f = data_val;
            break;

        case 9:
            module->Driver1Current.f = data_val;
            break;

        case 10:
            module->Driver2Current.f = data_val;
            break;

        case 11:
            module->TempL.f = data_val;
            break;

        case 12:
            module->TempHeatSink.f = data_val;
            break;

        case 13:
            module->GroundLeakage.f = data_val;
            break;

        default:
            break;
    }
}
