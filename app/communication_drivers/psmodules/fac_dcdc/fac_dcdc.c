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


#include <communication_drivers/psmodules/fac_dcdc/fac_dcdc.h>
#include <communication_drivers/psmodules/ps_modules.h>
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/control/control.h"
#include "communication_drivers/iib/iib_data.h"

#include "fac_dcdc.h"

#define I_LOAD_1            g_controller_ctom.net_signals[0]
#define I_LOAD_2            g_controller_ctom.net_signals[1]
#define V_CAPBANK           g_controller_ctom.net_signals[2]

#define I_LOAD_MEAN         g_controller_ctom.net_signals[4]
#define I_LOAD_ERROR        g_controller_ctom.net_signals[5]
#define I_LOAD_DIFF         g_controller_ctom.net_signals[10]

#define v_LOAD              g_controller_mtoc.net_signals[0]
#define TEMP_INDUCTORS      g_controller_mtoc.net_signals[1]
#define TEMP_IGBT           g_controller_mtoc.net_signals[2]

#define DUTY_CYCLE          g_controller_ctom.output_signals[0]

volatile iib_output_stage_t iib_output_stage[2];

static void init_iib_modules();
static void handle_can_message(uint8_t *data);
static void update_iib_structure(iib_output_stage_t *module, uint8_t data_id,
                                                               float data_val);

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
    create_bsmp_var(25, 0, 4, false, g_ipc_ctom.ps_module[0].ps_soft_interlock.u8);
    create_bsmp_var(26, 0, 4, false, g_ipc_ctom.ps_module[0].ps_hard_interlock.u8);
    create_bsmp_var(27, 0, 4, false, I_LOAD_1.u8);
    create_bsmp_var(28, 0, 4, false, I_LOAD_2.u8);
    create_bsmp_var(29, 0, 4, false, v_LOAD.u8);
    create_bsmp_var(30, 0, 4, false, V_CAPBANK.u8);
    create_bsmp_var(31, 0, 4, false, TEMP_INDUCTORS.u8);
    create_bsmp_var(32, 0, 4, false, TEMP_IGBT.u8);
    create_bsmp_var(33, 0, 4, false, DUTY_CYCLE.u8);

    // Output Module 1
    create_bsmp_var(34, 0, 4, false, iib_output_stage[0].Iin.u8);
    create_bsmp_var(35, 0, 4, false, iib_output_stage[0].Iout.u8);
    create_bsmp_var(36, 0, 4, false, iib_output_stage[0].VdcLink.u8);
    create_bsmp_var(37, 0, 4, false, iib_output_stage[0].TempIGBT1.u8);
    create_bsmp_var(38, 0, 4, false, iib_output_stage[0].TempIGBT2.u8);
    create_bsmp_var(39, 0, 4, false, iib_output_stage[0].TempL.u8);
    create_bsmp_var(40, 0, 4, false, iib_output_stage[0].TempHeatSink.u8);
    create_bsmp_var(41, 0, 4, false, iib_output_stage[0].Driver1Error.u8);
    create_bsmp_var(42, 0, 4, false, iib_output_stage[0].Driver2Error.u8);

    // Output Module 2
    create_bsmp_var(43, 0, 4, false, iib_output_stage[1].Iin.u8);
    create_bsmp_var(44, 0, 4, false, iib_output_stage[1].Iout.u8);
    create_bsmp_var(45, 0, 4, false, iib_output_stage[1].VdcLink.u8);
    create_bsmp_var(46, 0, 4, false, iib_output_stage[1].TempIGBT1.u8);
    create_bsmp_var(47, 0, 4, false, iib_output_stage[1].TempIGBT2.u8);
    create_bsmp_var(48, 0, 4, false, iib_output_stage[1].TempL.u8);
    create_bsmp_var(49, 0, 4, false, iib_output_stage[1].TempHeatSink.u8);
    create_bsmp_var(50, 0, 4, false, iib_output_stage[1].Driver1Error.u8);
    create_bsmp_var(51, 0, 4, false, iib_output_stage[1].Driver2Error.u8);

}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fac_dcdc_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
    init_iib_modules();
}

static void init_iib_modules()
{
    iib_output_stage[0].CanAddress = 1;
    iib_output_stage[1].CanAddress = 2;
}
static void handle_can_message(uint8_t *data)
{
    uint8_t iib_address;
    uint8_t data_id;

    float_to_bytes_t converter;

    iib_address     = data[0];
    data_id         = data[1];

    converter.u8[0] = data[4];
    converter.u8[1] = data[5];
    converter.u8[2] = data[6];
    converter.u8[3] = data[7];

    update_iib_structure(&iib_output_stage[iib_address - 1], data_id,
                                                                 converter.f);
}
static void update_iib_structure(iib_output_stage_t *module, uint8_t data_id,
                                                               float data_val)
{
    uint8_t id;
    id = data_id;

    switch (id) {
        case 0:
            module->Iin.f = data_val;
            break;

        case 1:
            module->Iout.f = data_val;
            break;

        case 2:
            module->VdcLink.f = data_val;
            break;

        case 3:
            module->TempIGBT1.f = data_val;
            break;

        case 5:
            module->TempIGBT2.f = data_val;
            break;

        case 6:
            module->TempL.f = data_val;
            break;

        case 7:
            module->TempHeatSink.f = data_val;
            break;

        case 8:
            module->Driver1Error.f = data_val;
            break;

        case 9:
            module->Driver2Error.f = data_val;
            break;

        default:
            break;
    }
}
