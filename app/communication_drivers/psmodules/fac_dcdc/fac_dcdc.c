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

volatile iib_output_stage_t iib_output_stage[NUMBER_OF_IIB_MODULES];

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
    create_bsmp_var(34, 0, 4, false, g_iib_output_module.iib_signals[0].u8);
    create_bsmp_var(35, 0, 4, false, g_iib_output_module.iib_signals[1].u8);
    create_bsmp_var(36, 0, 4, false, g_iib_output_module.iib_signals[2].u8);
    create_bsmp_var(37, 0, 4, false, g_iib_output_module.iib_signals[3].u8);
    create_bsmp_var(38, 0, 4, false, g_iib_output_module.iib_signals[4].u8);
    create_bsmp_var(39, 0, 4, false, g_iib_output_module.iib_signals[5].u8);
    create_bsmp_var(40, 0, 4, false, g_iib_output_module.iib_signals[6].u8);
    create_bsmp_var(41, 0, 4, false, g_iib_output_module.iib_signals[7].u8);
    create_bsmp_var(42, 0, 4, false, g_iib_output_module.iib_signals[8].u8);
    create_bsmp_var(43, 0, 4, false, g_iib_output_module.iib_signals[9].u8);

    // Output Module 2
    create_bsmp_var(44, 0, 4, false, g_iib_output_module.iib_signals[10].u8);
    create_bsmp_var(45, 0, 4, false, g_iib_output_module.iib_signals[11].u8);
    create_bsmp_var(46, 0, 4, false, g_iib_output_module.iib_signals[12].u8);
    create_bsmp_var(47, 0, 4, false, g_iib_output_module.iib_signals[13].u8);
    create_bsmp_var(48, 0, 4, false, g_iib_output_module.iib_signals[14].u8);
    create_bsmp_var(49, 0, 4, false, g_iib_output_module.iib_signals[15].u8);
    create_bsmp_var(50, 0, 4, false, g_iib_output_module.iib_signals[16].u8);
    create_bsmp_var(51, 0, 4, false, g_iib_output_module.iib_signals[17].u8);
    create_bsmp_var(52, 0, 4, false, g_iib_output_module.iib_signals[18].u8);
    create_bsmp_var(53, 0, 4, false, g_iib_output_module.iib_signals[19].u8);

}

static void check_can_message(uint8_t message[], uint8_t size)
{
    uint8_t iib_address = 0;

    iib_address = message[0];

    if (iib_address == iib_output_stage[0].CanAddress) {

        update_output_stage_data(&iib_output_stage[0], message, size);

    } else if (iib_address == iib_output_stage[1].CanAddress) {
        update_output_stage_data(&iib_output_stage[0], message, size);
    }
}

static void update_output_stage_data(iib_output_stage_t *stage,
                                               uint8_t message[], uint8_t size)
{
    float_to_bytes_t converter;
    uint8_t data_id;
    data_id = message[1];

    converter.u8[0] = message[4];
    converter.u8[1] = message[5];
    converter.u8[2] = message[6];
    converter.u8[3] = message[7];

    switch (data_id) {
        case 0:
            stage->Iin = converter.f;
            break;

        case 1:
            stage->Iout = converter.f;
            break;

        case 2:
            stage->VdcLink = converter.f;
            break;

        case 3:
            stage->TempIGBT1 = converter.u8[0];
            break;

        case 5:
            stage->TempIGBT2 = converter.u8[0];
            break;

        case 6:
            stage->TempL = converter.u8[0];
            break;

        case 7:
            stage->TempHeatSink = converter.u8[0];
            break;

        case 8:
            stage->Driver1Error = converter.u8[0];
            break;

        case 9:
            stage->Driver2Error = converter.u8[0];
            break;

        default:
            break;
    }
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
    init_iib_framwork(&g_iib_input_module);
    init_iib_framwork(&g_iib_output_module);
    bsmp_init_server();
}
