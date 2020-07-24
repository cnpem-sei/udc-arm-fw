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
 * @file fac_2s_dcdc.c
 * @brief FAC-2S DC/DC Stage module
 *
 * Module for control of two DC/DC modules of FAC power supplies for focusing
 * quadrupoles from booster. It implements the controller for load current.
 *
 * @author gabriel.brunheira
 * @date 27/02/2019
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
#include "communication_drivers/ps_modules/fac_2s_dcdc/fac_2s_dcdc.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/// DSP Net Signals
#define I_LOAD_1                        g_controller_ctom.net_signals[0]    // HRADC0
#define I_LOAD_2                        g_controller_ctom.net_signals[1]    // HRADC1
#define V_CAPBANK_MOD_1                 g_controller_ctom.net_signals[2]    // HRADC2
#define V_CAPBANK_MOD_2                 g_controller_ctom.net_signals[3]    // HRADC3

#define I_LOAD_REFERENCE_WFMREF         g_controller_ctom.net_signals[4]

#define I_LOAD_MEAN                     g_controller_ctom.net_signals[5]
#define I_LOAD_ERROR                    g_controller_ctom.net_signals[6]

#define DUTY_I_LOAD_PI                  g_controller_ctom.net_signals[7]
#define DUTY_REFERENCE_FF               g_controller_ctom.net_signals[8]
#define DUTY_MEAN                       g_controller_ctom.net_signals[9]

#define V_OUT_DIFF                      g_controller_ctom.net_signals[10]
#define DUTY_DIFF                       g_controller_ctom.net_signals[11]

#define V_CAPBANK_MOD_1_FILTERED        g_controller_ctom.net_signals[12]
#define V_CAPBANK_MOD_2_FILTERED        g_controller_ctom.net_signals[13]

#define IN_FF_V_CAPBANK_MOD_1           g_controller_ctom.net_signals[14]
#define IN_FF_V_CAPBANK_MOD_2           g_controller_ctom.net_signals[15]

#define I_LOAD_DIFF                     g_controller_ctom.net_signals[16]
#define V_LOAD                          g_controller_ctom.net_signals[17]

#define DUTY_CYCLE_MOD_1                g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_MOD_2                g_controller_ctom.output_signals[1]

/// ARM Net Signals
#define V_OUT_MOD_1                     g_controller_mtoc.net_signals[0]
#define V_OUT_MOD_2                     g_controller_mtoc.net_signals[1]

#define IIB_ITLK_REG_1                  g_controller_mtoc.net_signals[2]
#define IIB_ITLK_REG_2                  g_controller_mtoc.net_signals[3]

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Load_Overvoltage,
    Module_1_CapBank_Overvoltage,
    Module_2_CapBank_Overvoltage,
    Module_1_CapBank_Undervoltage,
    Module_2_CapBank_Undervoltage,
    Module_1_Output_Overvoltage,
    Module_2_Output_Overvoltage,
    IIB_1_Itlk,
    IIB_2_Itlk,
    External_Interlock,
    Rack_Interlock
} hard_interlocks_t;

typedef enum
{
    DCCT_1_Fault,
    DCCT_2_Fault,
    DCCT_High_Difference,
    Load_Feedback_1_Fault,
    Load_Feedback_2_Fault
} soft_interlocks_t;

volatile iib_fac_os_t fac_2s_dcdc_os[2];

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
    g_analog_ch_0.Enable = 0;
    g_analog_ch_1.Enable = 0;
    g_analog_ch_2.Enable = 0;
    g_analog_ch_3.Enable = 0;
    g_analog_ch_4.Enable = 0;
    g_analog_ch_5.Enable = 0;
    g_analog_ch_6.Enable = 0;
    g_analog_ch_7.Enable = 0;
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

    create_bsmp_var(37, 0, 4, false, V_OUT_MOD_1.u8);
    create_bsmp_var(38, 0, 4, false, V_OUT_MOD_2.u8);

    create_bsmp_var(39, 0, 4, false, V_CAPBANK_MOD_1.u8);
    create_bsmp_var(40, 0, 4, false, V_CAPBANK_MOD_2.u8);

    create_bsmp_var(41, 0, 4, false, DUTY_CYCLE_MOD_1.u8);
    create_bsmp_var(42, 0, 4, false, DUTY_CYCLE_MOD_2.u8);
    create_bsmp_var(43, 0, 4, false, DUTY_DIFF.u8);

    create_bsmp_var(44, 0, 4, false, fac_2s_dcdc_os[0].Iin.u8);
    create_bsmp_var(45, 0, 4, false, fac_2s_dcdc_os[0].Iout.u8);
    create_bsmp_var(46, 0, 4, false, fac_2s_dcdc_os[0].VdcLink.u8);
    create_bsmp_var(47, 0, 4, false, fac_2s_dcdc_os[0].TempL.u8);
    create_bsmp_var(48, 0, 4, false, fac_2s_dcdc_os[0].TempHeatSink.u8);

    create_bsmp_var(51, 0, 4, false, fac_2s_dcdc_os[1].Iin.u8);
    create_bsmp_var(52, 0, 4, false, fac_2s_dcdc_os[1].Iout.u8);
    create_bsmp_var(53, 0, 4, false, fac_2s_dcdc_os[1].VdcLink.u8);
    create_bsmp_var(54, 0, 4, false, fac_2s_dcdc_os[1].TempL.u8);
    create_bsmp_var(55, 0, 4, false, fac_2s_dcdc_os[1].TempHeatSink.u8);

    create_bsmp_var(58, 0, 4, false, IIB_ITLK_REG_1.u8);
    create_bsmp_var(59, 0, 4, false, IIB_ITLK_REG_2.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fac_2s_dcdc_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
    init_iib();

    init_wfmref(&WFMREF[0], WFMREF[0].wfmref_selected.u16, WFMREF[0].sync_mode.enu,
                ISR_CONTROL_FREQ.f, TIMESLICER_FREQ[TIMESLICER_WFMREF].f,
                WFMREF[0].gain.f, WFMREF[0].offset.f, &g_wfmref_data.data[0][0].f,
                SIZE_WFMREF, &g_ipc_ctom.ps_module[0].ps_reference.f);

    init_scope(&g_ipc_mtoc.scope[0], ISR_CONTROL_FREQ.f,
               SCOPE_FREQ_SAMPLING_PARAM[0].f, &(g_buf_samples_ctom[0].f),
               SIZE_BUF_SAMPLES_CTOM, SCOPE_SOURCE_PARAM[0].p_f,
               (void *) 0);
}

static void init_iib()
{
    fac_2s_dcdc_os[0].CanAddress = 1;
    fac_2s_dcdc_os[1].CanAddress = 2;

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

    update_iib_structure(&fac_2s_dcdc_os[iib_address-1], data_id, converter.f);
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
                set_hard_interlock(module->CanAddress - 1, IIB_1_Itlk);
            }
            if (module->CanAddress == 2) {
                IIB_ITLK_REG_2.u32 = converter.u32;
                set_hard_interlock(module->CanAddress - 1, IIB_2_Itlk);
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


