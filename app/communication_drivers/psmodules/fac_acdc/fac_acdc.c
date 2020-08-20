/******************************************************************************
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
 * @file fac_acdc.c
 * @brief FAC AC/DC Stage module
 *
 * Module for control of a AC/DC module of FAC power supplies It implements the
 * controllers for input current and capacitor bank voltage of a AC/DC module.
 *
 * @author gabriel.brunheira
 * @date 23/04/2018
 *
 */

#include <communication_drivers/psmodules/fac_acdc/fac_acdc.h>
#include <stdint.h>
#include <stdbool.h>

#include <communication_drivers/psmodules/ps_modules.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/control/control.h"
#include "communication_drivers/event_manager/event_manager.h"
#include "communication_drivers/iib/iib_data.h"
#include "communication_drivers/iib/iib_module.h"

#define V_CAPBANK                       g_controller_ctom.net_signals[0]    // HRADC0
#define IOUT_RECT                       g_controller_ctom.net_signals[1]    // HRADC1

#define VOUT_RECT                       g_controller_mtoc.net_signals[0]
#define TEMP_HEATSINK                   g_controller_mtoc.net_signals[1]
#define TEMP_INDUCTORS                  g_controller_mtoc.net_signals[2]

#define DUTY_CYCLE                      g_controller_ctom.output_signals[0]

#define IIB_ITLK_REG_IS                 g_controller_mtoc.net_signals[3]
#define IIB_ITLK_REG_CD                 g_controller_mtoc.net_signals[4]

/**
 * Interlocks defines
 */
typedef enum
{
    CapBank_Overvoltage,
    Rectifier_Overvoltage,
    Rectifier_Undervoltage,
    Rectifier_Overcurrent,
    AC_Mains_Contactor_Fault,
    IGBT_Driver_Fault,
    IIB_Itlk
} hard_interlocks_t;

volatile iib_input_stage_t iib_input_stage;
volatile iib_command_drawer_t iib_command_drawer;

static void init_iib_modules();
static void handle_can_data(uint8_t *data);
static void update_iib_structure_is(uint8_t data_id, float data_val);
static void update_iib_structure_cd(uint8_t data_id, float data_val);

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
    create_bsmp_var(27, 0, 4, false, V_CAPBANK.u8);
    create_bsmp_var(28, 0, 4, false, VOUT_RECT.u8);
    create_bsmp_var(29, 0, 4, false, IOUT_RECT.u8);
    create_bsmp_var(30, 0, 4, false, TEMP_HEATSINK.u8);
    create_bsmp_var(31, 0, 4, false, TEMP_INDUCTORS.u8);
    create_bsmp_var(32, 0, 4, false, DUTY_CYCLE.u8);

    create_bsmp_var(33, 0, 4, false, iib_input_stage.Iin.u8);
    create_bsmp_var(34, 0, 4, false, iib_input_stage.VdcLink.u8);
    create_bsmp_var(35, 0, 4, false, iib_input_stage.TempL.u8);
    create_bsmp_var(36, 0, 4, false, iib_input_stage.TempHeatsink.u8);

    create_bsmp_var(37, 0, 4, false, iib_command_drawer.Vout.u8);
    create_bsmp_var(38, 0, 4, false, iib_command_drawer.VcapBank.u8);
    create_bsmp_var(39, 0, 4, false, iib_command_drawer.TempL.u8);
    create_bsmp_var(40, 0, 4, false, iib_command_drawer.TempHeatSink.u8);
    create_bsmp_var(41, 0, 4, false, iib_command_drawer.GroundLeakage.u8);

    create_bsmp_var(42, 0, 4, false, IIB_ITLK_REG_IS.u8);
    create_bsmp_var(43, 0, 4, false, IIB_ITLK_REG_CD.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fac_acdc_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
    init_iib_modules();
    init_buffer(&g_ipc_mtoc.buf_samples[0], &(g_buf_samples_ctom[0].f), SIZE_BUF_SAMPLES_CTOM);
}

static void init_iib_modules()
{
    iib_input_stage.CanAddress = 1;
    iib_command_drawer.CanAddress = 2;

    init_iib_module(&g_iib_module, &handle_can_data);
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

    if (iib_address == 1) update_iib_structure_is(data_id, converter.f);
    if (iib_address == 2) update_iib_structure_cd(data_id, converter.f);
}

static void update_iib_structure_is(uint8_t data_id, float data_val)
{
    uint8_t id;
    id = data_id;

    float_to_bytes_t converter;

    switch(id) {
        case 0:
            converter.f = data_val;
            IIB_ITLK_REG_IS.u32 = converter.u32;
            set_hard_interlock(0, IIB_Itlk);
            break;
        case 1:
            // TODO: Handle alarm message
            break;
        case 2:
            iib_input_stage.Iin.f = data_val;
            break;

        case 3:
            iib_input_stage.VdcLink.f = data_val;
            break;

        case 4:
            iib_input_stage.TempL.f = data_val;
            break;

        case 5:
            iib_input_stage.TempHeatsink.f = data_val;
            break;

        default:
            break;
    }
}

static void update_iib_structure_cd(uint8_t data_id, float data_val)
{
    uint8_t id;
    id = data_id;

    float_to_bytes_t converter;

    switch(id) {
        case 0:
            converter.f = data_val;
            IIB_ITLK_REG_CD.u32 = converter.u32;
            set_hard_interlock(0, IIB_Itlk);
            break;
        case 1:
            // TODO: Handle alarm data
            break;
        case 2:
            iib_command_drawer.Vout.f = data_val;
            break;

        case 3:
            iib_command_drawer.VcapBank.f = data_val;
            break;

        case 4:
            iib_command_drawer.TempL.f = data_val;
            break;

        case 5:
            iib_command_drawer.TempHeatSink.f = data_val;
            break;

        case 6:
            iib_command_drawer.GroundLeakage.f = data_val;
            break;

        default:
            break;
    }
}
