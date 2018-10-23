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

#include <communication_drivers/psmodules/fap/fap.h>
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

/**
 * Controller defines
 */
#define I_LOAD_1                g_controller_ctom.net_signals[0]    // HRADC0
#define I_LOAD_2                g_controller_ctom.net_signals[1]    // HRADC1
#define V_DCLINK                g_controller_ctom.net_signals[2]    // HRADC2

#define I_LOAD_MEAN             g_controller_ctom.net_signals[3]
#define I_LOAD_ERROR            g_controller_ctom.net_signals[4]
#define I_LOAD_DIFF             g_controller_ctom.net_signals[16]

#define I_IGBTS_DIFF            g_controller_ctom.net_signals[6]
#define I_IGBT_1                g_controller_mtoc.net_signals[0]    // ANI0
#define I_IGBT_2                g_controller_mtoc.net_signals[1]    // ANI1

#define DUTY_MEAN               g_controller_ctom.net_signals[5]
#define DUTY_DIFF               g_controller_ctom.net_signals[7]

#define DUTY_CYCLE_IGBT_1       g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_IGBT_2       g_controller_ctom.output_signals[1]

static volatile iib_fap_module_t iib_fap[4];

static void init_iib_modules();
static void handle_can_message(uint8_t *data);
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
    // IGBT 1 current: 10 V = 200 A
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 200.0/2048.0;
    g_analog_ch_0.Value = &(I_IGBT_1.f);

    // IGBT 2 current: 10 V = 200 A
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 200.0/2048.0;
    g_analog_ch_1.Value = &(I_IGBT_2.f);

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
    create_bsmp_var(29, 0, 4, false, V_DCLINK.u8);
    create_bsmp_var(30, 0, 4, false, I_IGBT_1.u8);
    create_bsmp_var(31, 0, 4, false, I_IGBT_2.u8);
    create_bsmp_var(32, 0, 4, false, DUTY_CYCLE_IGBT_1.u8);
    create_bsmp_var(33, 0, 4, false, DUTY_CYCLE_IGBT_2.u8);
    create_bsmp_var(34, 0, 4, false, DUTY_DIFF.u8);

    create_bsmp_var(35, 0, 4, false, iib_fap[0].Vin.u8);
    create_bsmp_var(36, 0, 4, false, iib_fap[0].Vout.u8);
    create_bsmp_var(37, 0, 4, false, iib_fap[0].IoutA1.u8);
    create_bsmp_var(38, 0, 4, false, iib_fap[0].IoutA2.u8);
    create_bsmp_var(39, 0, 4, false, iib_fap[0].TempIGBT1.u8);
    create_bsmp_var(40, 0, 4, false, iib_fap[0].TempIGBT2.u8);
    create_bsmp_var(41, 0, 4, false, iib_fap[0].DriveVoltage.u8);
    create_bsmp_var(42, 0, 4, false, iib_fap[0].Drive1Current.u8);
    create_bsmp_var(43, 0, 4, false, iib_fap[0].Drive2Current.u8);
    create_bsmp_var(44, 0, 4, false, iib_fap[0].TempL.u8);
    create_bsmp_var(45, 0, 4, false, iib_fap[0].TempHeatSink.u8);
    create_bsmp_var(46, 0, 4, false, iib_fap[0].ExternalItlk.u8);
    create_bsmp_var(47, 0, 4, false, iib_fap[0].LeakageCurrent.u8);
    create_bsmp_var(48, 0, 4, false, iib_fap[0].Rack.u8);

    create_bsmp_var(49, 0, 4, false, iib_fap[1].Vin.u8);
    create_bsmp_var(50, 0, 4, false, iib_fap[1].Vout.u8);
    create_bsmp_var(51, 0, 4, false, iib_fap[1].IoutA1.u8);
    create_bsmp_var(52, 0, 4, false, iib_fap[1].IoutA2.u8);
    create_bsmp_var(53, 0, 4, false, iib_fap[1].TempIGBT1.u8);
    create_bsmp_var(54, 0, 4, false, iib_fap[1].TempIGBT2.u8);
    create_bsmp_var(55, 0, 4, false, iib_fap[1].DriveVoltage.u8);
    create_bsmp_var(56, 0, 4, false, iib_fap[1].Drive1Current.u8);
    create_bsmp_var(57, 0, 4, false, iib_fap[1].Drive2Current.u8);
    create_bsmp_var(58, 0, 4, false, iib_fap[1].TempL.u8);
    create_bsmp_var(59, 0, 4, false, iib_fap[1].TempHeatSink.u8);
    create_bsmp_var(60, 0, 4, false, iib_fap[1].ExternalItlk.u8);
    create_bsmp_var(61, 0, 4, false, iib_fap[1].LeakageCurrent.u8);
    create_bsmp_var(62, 0, 4, false, iib_fap[1].Rack.u8);

    create_bsmp_var(63, 0, 4, false, iib_fap[2].Vin.u8);
    create_bsmp_var(64, 0, 4, false, iib_fap[2].Vout.u8);
    create_bsmp_var(65, 0, 4, false, iib_fap[2].IoutA1.u8);
    create_bsmp_var(66, 0, 4, false, iib_fap[2].IoutA2.u8);
    create_bsmp_var(67, 0, 4, false, iib_fap[2].TempIGBT1.u8);
    create_bsmp_var(68, 0, 4, false, iib_fap[2].TempIGBT2.u8);
    create_bsmp_var(69, 0, 4, false, iib_fap[2].DriveVoltage.u8);
    create_bsmp_var(70, 0, 4, false, iib_fap[2].Drive1Current.u8);
    create_bsmp_var(71, 0, 4, false, iib_fap[2].Drive2Current.u8);
    create_bsmp_var(72, 0, 4, false, iib_fap[2].TempL.u8);
    create_bsmp_var(73, 0, 4, false, iib_fap[2].TempHeatSink.u8);
    create_bsmp_var(74, 0, 4, false, iib_fap[2].ExternalItlk.u8);
    create_bsmp_var(75, 0, 4, false, iib_fap[2].LeakageCurrent.u8);
    create_bsmp_var(76, 0, 4, false, iib_fap[2].Rack.u8);

    create_bsmp_var(77, 0, 4, false, iib_fap[3].Vin.u8);
    create_bsmp_var(78, 0, 4, false, iib_fap[3].Vout.u8);
    create_bsmp_var(79, 0, 4, false, iib_fap[3].IoutA1.u8);
    create_bsmp_var(80, 0, 4, false, iib_fap[3].IoutA2.u8);
    create_bsmp_var(81, 0, 4, false, iib_fap[3].TempIGBT1.u8);
    create_bsmp_var(82, 0, 4, false, iib_fap[3].TempIGBT2.u8);
    create_bsmp_var(83, 0, 4, false, iib_fap[3].DriveVoltage.u8);
    create_bsmp_var(84, 0, 4, false, iib_fap[3].Drive1Current.u8);
    create_bsmp_var(85, 0, 4, false, iib_fap[3].Drive2Current.u8);
    create_bsmp_var(86, 0, 4, false, iib_fap[3].TempL.u8);
    create_bsmp_var(87, 0, 4, false, iib_fap[3].TempHeatSink.u8);
    create_bsmp_var(88, 0, 4, false, iib_fap[3].ExternalItlk.u8);
    create_bsmp_var(89, 0, 4, false, iib_fap[3].LeakageCurrent.u8);
    create_bsmp_var(90, 0, 4, false, iib_fap[3].Rack.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fap_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
    init_iib_modules();
}

static void init_iib_modules()
{
    iib_fap[0].CanAddress = 1;
    iib_fap[1].CanAddress = 2;
    iib_fap[2].CanAddress = 3;
    iib_fap[3].CanAddress = 4;
}

static void handle_can_data(uint8_t *data)
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

    update_iib_structure(&iib_fap[iib_address - 1], data_id, converter.f);

}

static void update_iib_structure(iib_fap_module_t *module, uint8_t data_id,
                                                               float data_val)
{
    uint8_t id = data_id;

    switch (id)
    {
        case 1:
            module->Vin.f = data_val;
            break;
        case 2:
            module->Vout.f = data_val;
            break;
        case 3:
            module->IoutA1.f = data_val;
            break;
        case 4:
            module->IoutA2.f = data_val;
            break;
        case 5:
            module->DriveVoltage.f = data_val;
            break;
        case 6:
            module->Drive1Current.f = data_val;
            break;
        case 7:
            module->Drive2Current.f = data_val;
            break;
        case 8:
            module->TempIGBT1.f = data_val;
            break;
        case 9:
            module->TempIGBT2.f = data_val;
            break;
        case 10:
            module->Driver1Error.f = data_val;
            break;
        case 11:
            module->Driver2Error.f = data_val;
            break;
        case 12:
            module->TempL.f = data_val;
            break;
        case 13:
            module->TempHeatSink.f = data_val;
            break;
        case 14:
            module->Relay.f = data_val;
            break;
        case 15:
            module->LeakageCurrent.f = data_val;
            break;
        case 16:
            module->Rack.f = data_val;
            break;
        default:
            break;
    }
}
