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
 * @file fac_dcdc_ema.c
 * @brief FAC DC/DC Stage module for dipole magnet from EMA beamline.
 *
 * Module for control of DC/DC module of FAC power supply for dipole magnet used
 * at EMA beamline from Sirius. It implements the controller for load current.
 *
 * @author gabriel.brunheira
 * @date 08/03/2019
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
#include "communication_drivers/ps_modules/fac_dcdc_ema/fac_dcdc_ema.h"
#include "communication_drivers/ps_modules/ps_modules.h"

#define I_LOAD                      g_controller_ctom.net_signals[0]  // HRADC0
#define V_CAPBANK                   g_controller_ctom.net_signals[1]  // HRADC1

#define I_LOAD_REFERENCE_WFMREF     g_controller_ctom.net_signals[2]
#define I_LOAD_ERROR                g_controller_ctom.net_signals[3]

#define DUTY_I_LOAD_PI              g_controller_ctom.net_signals[4]
#define DUTY_REFERENCE_FF           g_controller_ctom.net_signals[5]
#define DUTY_NOMINAL                g_controller_ctom.net_signals[6]

#define V_CAPBANK_FILTERED          g_controller_ctom.net_signals[7]

#define DUTY_CYCLE                  g_controller_ctom.output_signals[0]

#define IIB_ITLK_REG                g_controller_mtoc.net_signals[1]


/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    CapBank_Overvoltage,
    CapBank_Undervoltage,
    Emergency_Button,
    Load_Waterflow,
    Load_Overtemperature,
    IIB_Itlk,
} hard_interlocks_t;

typedef enum
{
    DCCT_Fault,
    Load_Feedback_Fault,
} soft_interlocks_t;

volatile iib_fac_os_t iib_output_stage;
volatile hard_interlocks_t hard_interlocks;

static void init_iib_modules();
static void handle_can_data(uint8_t *data);
static void update_iib_structure(iib_fac_os_t *module, uint8_t data_id,
                                                               float data_val);

static void handle_interlock_message(uint8_t *data);
static void handle_alarm_message(uint8_t *data);

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC ACDC operation.
*
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

    create_bsmp_var(33, 0, 4, false, I_LOAD.u8);
    create_bsmp_var(34, 0, 4, false, V_CAPBANK.u8);

    create_bsmp_var(35, 0, 4, false, DUTY_CYCLE.u8);

    // Output Module
    create_bsmp_var(36, 0, 4, false, iib_output_stage.Iin.u8);
    create_bsmp_var(37, 0, 4, false, iib_output_stage.Iout.u8);
    create_bsmp_var(38, 0, 4, false, iib_output_stage.VdcLink.u8);
    create_bsmp_var(39, 0, 4, false, iib_output_stage.TempIGBT1.u8);
    create_bsmp_var(40, 0, 4, false, iib_output_stage.TempIGBT2.u8);
    create_bsmp_var(41, 0, 4, false, iib_output_stage.TempL.u8);
    create_bsmp_var(42, 0, 4, false, iib_output_stage.TempHeatSink.u8);
    create_bsmp_var(43, 0, 4, false, iib_output_stage.Driver1Error.u8);
    create_bsmp_var(44, 0, 4, false, iib_output_stage.Driver2Error.u8);

    create_bsmp_var(45, 0, 4, false, IIB_ITLK_REG.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fac_dcdc_ema_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
    init_iib_modules();

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

static void init_iib_modules()
{
    iib_output_stage.CanAddress = 1;

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

    update_iib_structure(&iib_output_stage, data_id, converter.f);
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
            IIB_ITLK_REG.u32 = converter.u32;
            set_hard_interlock(0, IIB_Itlk);
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
            module->TempIGBT1.f = data_val;
            break;

        case 6:
            module->TempIGBT2.f = data_val;
            break;

        case 7:
            module->TempL.f = data_val;
            break;

        case 8:
            module->TempHeatSink.f = data_val;
            break;

        case 9:
            module->Driver1Error.f = data_val;
            break;

        case 10:
            module->Driver2Error.f = data_val;
            break;

        default:
            break;
    }
}

