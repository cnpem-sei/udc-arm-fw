/******************************************************************************
 * Copyright (C) 2017 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fac_dcdc.c
 * @brief FAC DC/DC Stage module
 *
 * Module for control of DC/DC module of FAC power supplies. It implements the
 * controller for load current.
 *
 * @author gabriel.brunheira
 * @date 01/05/2018
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
#include "communication_drivers/ps_modules/fac_2p4s_dcdc/fac_2p4s_dcdc.h"
#include "communication_drivers/ps_modules/ps_modules.h"

#define I_LOAD_1            g_controller_ctom.net_signals[0]
#define I_LOAD_2            g_controller_ctom.net_signals[1]
#define I_ARM_1             g_controller_ctom.net_signals[2]
#define I_ARM_2             g_controller_ctom.net_signals[3]

#define I_LOAD_MEAN         g_controller_ctom.net_signals[5]
#define I_LOAD_ERROR        g_controller_ctom.net_signals[6]
#define DUTY_MEAN           g_controller_ctom.net_signals[7]

#define I_ARMS_DIFF         g_controller_ctom.net_signals[8]
#define DUTY_DIFF           g_controller_ctom.net_signals[9]

#define I_LOAD_DIFF         g_controller_ctom.net_signals[10]

#define V_LOAD              g_controller_mtoc.net_signals[0]

#define V_CAPBANK_MOD_1     g_controller_mtoc.net_signals[1]
#define V_CAPBANK_MOD_2     g_controller_mtoc.net_signals[2]
#define V_CAPBANK_MOD_3     g_controller_mtoc.net_signals[3]
#define V_CAPBANK_MOD_4     g_controller_mtoc.net_signals[4]
#define V_CAPBANK_MOD_5     g_controller_mtoc.net_signals[5]
#define V_CAPBANK_MOD_6     g_controller_mtoc.net_signals[6]
#define V_CAPBANK_MOD_7     g_controller_mtoc.net_signals[7]
#define V_CAPBANK_MOD_8     g_controller_mtoc.net_signals[8]

#define V_OUT_MOD_1         g_controller_mtoc.net_signals[9]
#define V_OUT_MOD_2         g_controller_mtoc.net_signals[10]
#define V_OUT_MOD_3         g_controller_mtoc.net_signals[11]
#define V_OUT_MOD_4         g_controller_mtoc.net_signals[12]
#define V_OUT_MOD_5         g_controller_mtoc.net_signals[13]
#define V_OUT_MOD_6         g_controller_mtoc.net_signals[14]
#define V_OUT_MOD_7         g_controller_mtoc.net_signals[15]
#define V_OUT_MOD_8         g_controller_mtoc.net_signals[16]

#define IIB_ITLK_REG_1      g_controller_mtoc.net_signals[17]
#define IIB_ITLK_REG_2      g_controller_mtoc.net_signals[18]
#define IIB_ITLK_REG_3      g_controller_mtoc.net_signals[19]
#define IIB_ITLK_REG_4      g_controller_mtoc.net_signals[20]
#define IIB_ITLK_REG_5      g_controller_mtoc.net_signals[21]
#define IIB_ITLK_REG_6      g_controller_mtoc.net_signals[22]
#define IIB_ITLK_REG_7      g_controller_mtoc.net_signals[23]
#define IIB_ITLK_REG_8      g_controller_mtoc.net_signals[24]

//#define TEMP_INDUCTORS      g_controller_mtoc.net_signals[10]
//#define TEMP_IGBT           g_controller_mtoc.net_signals[11]

#define DUTY_CYCLE_MOD_1    g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_MOD_2    g_controller_ctom.output_signals[1]
#define DUTY_CYCLE_MOD_3    g_controller_ctom.output_signals[2]
#define DUTY_CYCLE_MOD_4    g_controller_ctom.output_signals[3]
#define DUTY_CYCLE_MOD_5    g_controller_ctom.output_signals[4]
#define DUTY_CYCLE_MOD_6    g_controller_ctom.output_signals[5]
#define DUTY_CYCLE_MOD_7    g_controller_ctom.output_signals[6]
#define DUTY_CYCLE_MOD_8    g_controller_ctom.output_signals[7]

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Load_Overvoltage,
    Module_1_CapBank_Overvoltage,
    Module_2_CapBank_Overvoltage,
    Module_3_CapBank_Overvoltage,
    Module_4_CapBank_Overvoltage,
    Module_5_CapBank_Overvoltage,
    Module_6_CapBank_Overvoltage,
    Module_7_CapBank_Overvoltage,
    Module_8_CapBank_Overvoltage,
    Module_1_CapBank_Undervoltage,
    Module_2_CapBank_Undervoltage,
    Module_3_CapBank_Undervoltage,
    Module_4_CapBank_Undervoltage,
    Module_5_CapBank_Undervoltage,
    Module_6_CapBank_Undervoltage,
    Module_7_CapBank_Undervoltage,
    Module_8_CapBank_Undervoltage,
    Module_1_Output_Overvoltage,
    Module_2_Output_Overvoltage,
    Module_3_Output_Overvoltage,
    Module_4_Output_Overvoltage,
    Module_5_Output_Overvoltage,
    Module_6_Output_Overvoltage,
    Module_7_Output_Overvoltage,
    Module_8_Output_Overvoltage,
    IIB_1_Itlk,
    IIB_2_Itlk,
    IIB_3_Itlk,
    IIB_4_Itlk,
    IIB_5_Itlk,
    IIB_6_Itlk,
    IIB_7_Itlk,
    IIB_8_Itlk
} hard_interlocks_t;

volatile iib_fac_os_t fac_os[8];

static void init_iib();
static void handle_can_data(uint8_t *data);
static void update_iib_structure(iib_fac_os_t *module, uint8_t data_id,
                                                               float data_val);

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC-2P4S DCDC operation. Note that module 3
* and module 4 don't follow the sequence of ADCP channels, in order to keep
* cables installation on EXB more intuitive.
*/
static void adcp_channel_config(void)
{
    // Module 1 DCLink Voltage: 10 V = 300 V
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 300.0/2048.0;
    g_analog_ch_0.Value = &(V_CAPBANK_MOD_1.f);

    // Module 2 DCLink Voltage: 10 V = 300 V
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 300.0/2048.0;
    g_analog_ch_1.Value = &(V_CAPBANK_MOD_2.f);

    // Module 3 DCLink Voltage: 10 V = 300 V
    g_analog_ch_3.Enable = 1;
    g_analog_ch_3.Gain = 300.0/2048.0;
    g_analog_ch_3.Value = &(V_CAPBANK_MOD_3.f);

    // Module 4 DCLink Voltage: 10 V = 300 V
    g_analog_ch_2.Enable = 1;
    g_analog_ch_2.Gain = 300.0/2048.0;
    g_analog_ch_2.Value = &(V_CAPBANK_MOD_4.f);

    // Module 5 DCLink Voltage: 10 V = 300 V
    g_analog_ch_4.Enable = 1;
    g_analog_ch_4.Gain = 300.0/2048.0;
    g_analog_ch_4.Value = &(V_CAPBANK_MOD_5.f);

    // Module 6 DCLink Voltage: 10 V = 300 V
    g_analog_ch_5.Enable = 1;
    g_analog_ch_5.Gain = 300.0/2048.0;
    g_analog_ch_5.Value = &(V_CAPBANK_MOD_6.f);

    // Module 7 DCLink Voltage: 10 V = 300 V
    g_analog_ch_6.Enable = 1;
    g_analog_ch_6.Gain = 300.0/2048.0;
    g_analog_ch_6.Value = &(V_CAPBANK_MOD_7.f);

    // Module 8 DCLink Voltage: 10 V = 300 V
    g_analog_ch_7.Enable = 1;
    g_analog_ch_7.Gain = 300.0/2048.0;
    g_analog_ch_7.Value = &(V_CAPBANK_MOD_8.f);
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

    create_bsmp_var(38, 0, 4, false, V_LOAD.u8);

    create_bsmp_var(39, 0, 4, false, V_CAPBANK_MOD_1.u8);
    create_bsmp_var(40, 0, 4, false, V_CAPBANK_MOD_2.u8);
    create_bsmp_var(41, 0, 4, false, V_CAPBANK_MOD_3.u8);
    create_bsmp_var(42, 0, 4, false, V_CAPBANK_MOD_4.u8);
    create_bsmp_var(43, 0, 4, false, V_CAPBANK_MOD_5.u8);
    create_bsmp_var(44, 0, 4, false, V_CAPBANK_MOD_6.u8);
    create_bsmp_var(45, 0, 4, false, V_CAPBANK_MOD_7.u8);
    create_bsmp_var(46, 0, 4, false, V_CAPBANK_MOD_8.u8);

    create_bsmp_var(47, 0, 4, false, V_OUT_MOD_1.u8);
    create_bsmp_var(48, 0, 4, false, V_OUT_MOD_2.u8);
    create_bsmp_var(49, 0, 4, false, V_OUT_MOD_3.u8);
    create_bsmp_var(50, 0, 4, false, V_OUT_MOD_4.u8);
    create_bsmp_var(51, 0, 4, false, V_OUT_MOD_5.u8);
    create_bsmp_var(52, 0, 4, false, V_OUT_MOD_6.u8);
    create_bsmp_var(53, 0, 4, false, V_OUT_MOD_7.u8);
    create_bsmp_var(54, 0, 4, false, V_OUT_MOD_8.u8);

    create_bsmp_var(55, 0, 4, false, DUTY_CYCLE_MOD_1.u8);
    create_bsmp_var(56, 0, 4, false, DUTY_CYCLE_MOD_2.u8);
    create_bsmp_var(57, 0, 4, false, DUTY_CYCLE_MOD_3.u8);
    create_bsmp_var(58, 0, 4, false, DUTY_CYCLE_MOD_4.u8);
    create_bsmp_var(59, 0, 4, false, DUTY_CYCLE_MOD_5.u8);
    create_bsmp_var(60, 0, 4, false, DUTY_CYCLE_MOD_6.u8);
    create_bsmp_var(61, 0, 4, false, DUTY_CYCLE_MOD_7.u8);
    create_bsmp_var(62, 0, 4, false, DUTY_CYCLE_MOD_8.u8);

    create_bsmp_var(63, 0, 4, false, fac_os[0].Iin.u8);
    create_bsmp_var(64, 0, 4, false, fac_os[0].Iout.u8);
    create_bsmp_var(65, 0, 4, false, fac_os[0].VdcLink.u8);
    create_bsmp_var(66, 0, 4, false, fac_os[0].TempL.u8);
    create_bsmp_var(67, 0, 4, false, fac_os[0].TempHeatSink.u8);

    create_bsmp_var(70, 0, 4, false, fac_os[1].Iin.u8);
    create_bsmp_var(71, 0, 4, false, fac_os[1].Iout.u8);
    create_bsmp_var(72, 0, 4, false, fac_os[1].VdcLink.u8);
    create_bsmp_var(73, 0, 4, false, fac_os[1].TempL.u8);
    create_bsmp_var(74, 0, 4, false, fac_os[1].TempHeatSink.u8);

    create_bsmp_var(77, 0, 4, false, fac_os[2].Iin.u8);
    create_bsmp_var(78, 0, 4, false, fac_os[2].Iout.u8);
    create_bsmp_var(79, 0, 4, false, fac_os[2].VdcLink.u8);
    create_bsmp_var(80, 0, 4, false, fac_os[2].TempL.u8);
    create_bsmp_var(81, 0, 4, false, fac_os[2].TempHeatSink.u8);

    create_bsmp_var(84, 0, 4, false, fac_os[3].Iin.u8);
    create_bsmp_var(85, 0, 4, false, fac_os[3].Iout.u8);
    create_bsmp_var(86, 0, 4, false, fac_os[3].VdcLink.u8);
    create_bsmp_var(87, 0, 4, false, fac_os[3].TempL.u8);
    create_bsmp_var(88, 0, 4, false, fac_os[3].TempHeatSink.u8);

    create_bsmp_var(91, 0, 4, false, fac_os[4].Iin.u8);
    create_bsmp_var(92, 0, 4, false, fac_os[4].Iout.u8);
    create_bsmp_var(93, 0, 4, false, fac_os[4].VdcLink.u8);
    create_bsmp_var(94, 0, 4, false, fac_os[4].TempL.u8);
    create_bsmp_var(95, 0, 4, false, fac_os[4].TempHeatSink.u8);

    create_bsmp_var(98, 0, 4, false, fac_os[5].Iin.u8);
    create_bsmp_var(99, 0, 4, false, fac_os[5].Iout.u8);
    create_bsmp_var(100, 0, 4, false, fac_os[5].VdcLink.u8);
    create_bsmp_var(101, 0, 4, false, fac_os[5].TempL.u8);
    create_bsmp_var(102, 0, 4, false, fac_os[5].TempHeatSink.u8);

    create_bsmp_var(105, 0, 4, false, fac_os[6].Iin.u8);
    create_bsmp_var(106, 0, 4, false, fac_os[6].Iout.u8);
    create_bsmp_var(107, 0, 4, false, fac_os[6].VdcLink.u8);
    create_bsmp_var(108, 0, 4, false, fac_os[6].TempL.u8);
    create_bsmp_var(109, 0, 4, false, fac_os[6].TempHeatSink.u8);

    create_bsmp_var(112, 0, 4, false, fac_os[7].Iin.u8);
    create_bsmp_var(113, 0, 4, false, fac_os[7].Iout.u8);
    create_bsmp_var(114, 0, 4, false, fac_os[7].VdcLink.u8);
    create_bsmp_var(115, 0, 4, false, fac_os[7].TempL.u8);
    create_bsmp_var(116, 0, 4, false, fac_os[7].TempHeatSink.u8);

    create_bsmp_var(119, 0, 4, false, IIB_ITLK_REG_1.u8);
    create_bsmp_var(120, 0, 4, false, IIB_ITLK_REG_2.u8);
    create_bsmp_var(121, 0, 4, false, IIB_ITLK_REG_3.u8);
    create_bsmp_var(122, 0, 4, false, IIB_ITLK_REG_4.u8);
    create_bsmp_var(123, 0, 4, false, IIB_ITLK_REG_5.u8);
    create_bsmp_var(124, 0, 4, false, IIB_ITLK_REG_6.u8);
    create_bsmp_var(125, 0, 4, false, IIB_ITLK_REG_7.u8);
    create_bsmp_var(126, 0, 4, false, IIB_ITLK_REG_8.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fac_2p4s_dcdc_system_config()
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
    fac_os[0].CanAddress = 1;
    fac_os[1].CanAddress = 2;
    fac_os[2].CanAddress = 3;
    fac_os[3].CanAddress = 4;
    fac_os[4].CanAddress = 5;
    fac_os[5].CanAddress = 6;
    fac_os[6].CanAddress = 7;
    fac_os[7].CanAddress = 8;

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

    update_iib_structure(&fac_os[iib_address-1], data_id, converter.f);
}

static void update_iib_structure(iib_fac_os_t *module, uint8_t data_id,
                                                               float data_val)
{
    uint8_t id;
    id = data_id;

    convert_to_bytes_t converter;

    switch (id) {
        case 0:
            converter.f = data_val;
            if (module->CanAddress == 1) {
                IIB_ITLK_REG_1.u32 = converter.u32;
                set_hard_interlock(0, IIB_1_Itlk);
            }
            if (module->CanAddress == 2) {
                IIB_ITLK_REG_2.u32 = converter.u32;
                set_hard_interlock(0, IIB_2_Itlk);
            }
            if (module->CanAddress == 3) {
                IIB_ITLK_REG_3.u32 = converter.u32;
                set_hard_interlock(0, IIB_3_Itlk);
            }
            if (module->CanAddress == 4) {
                IIB_ITLK_REG_4.u32 = converter.u32;
                set_hard_interlock(0, IIB_4_Itlk);
            }
            if (module->CanAddress == 5) {
                IIB_ITLK_REG_5.u32 = converter.u32;
                set_hard_interlock(0, IIB_5_Itlk);
            }
            if (module->CanAddress == 6) {
                IIB_ITLK_REG_6.u32 = converter.u32;
                set_hard_interlock(0, IIB_6_Itlk);
            }
            if (module->CanAddress == 7) {
                IIB_ITLK_REG_7.u32 = converter.u32;
                set_hard_interlock(0, IIB_7_Itlk);
            }
            if (module->CanAddress == 8) {
                IIB_ITLK_REG_8.u32 = converter.u32;
                set_hard_interlock(0, IIB_8_Itlk);
            }

            break;
        case 1:
            //TODO: Handle alarm message
            break;
        case 2:
            module->Iin.f = data_val;
            break;

        case 3:
            module->Iout.f = data_val;
            break;

        case 4:
            module->VdcLink.f = data_val;
            break;

        case 5:
            //module->TempIGBT1.f = data_val;
            break;

        case 6:
            //module->TempIGBT2.f = data_val;
            break;

        case 7:
            module->TempL.f = data_val;
            break;

        case 8:
            module->TempHeatSink.f = data_val;
            break;

        default:
            break;
    }
}
