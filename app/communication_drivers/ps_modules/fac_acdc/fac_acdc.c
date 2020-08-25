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
#include "communication_drivers/event_manager/event_manager.h"
#include "communication_drivers/iib/iib_data.h"
#include "communication_drivers/iib/iib_module.h"
#include "communication_drivers/ps_modules/fac_acdc/fac_acdc.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/**
 * IIB Defines
 */
#define IIB_IS_ADDRESS      1
#define IIB_CMD_ADDRESS     2

/**
 * Controller defines
 */

/// DSP Net Signals
#define V_CAPBANK                       g_controller_ctom.net_signals[0]    // HRADC0
#define IOUT_RECT                       g_controller_ctom.net_signals[1]    // HRADC1

#define DUTY_CYCLE                      g_controller_ctom.output_signals[0]

/// ARM Net Signals
#define VOUT_RECT                       g_controller_mtoc.net_signals[0]

/**
 * Interlocks defines
 */
typedef enum
{
    CapBank_Overvoltage,
    Rectifier_Overvoltage,
    Rectifier_Undervoltage,
    Rectifier_Overcurrent,
    Welded_Contactor_Fault,
    Opened_Contactor_Fault,
    IIB_IS_Itlk,
    IIB_Cmd_Itlk
} hard_interlocks_t;

/*typedef enum
{
} soft_interlocks_t;*/

volatile iib_fac_is_t iib_fac_is;
volatile iib_fac_cmd_t iib_fac_cmd;

static void init_iib_modules();

static void handle_can_data(uint8_t *data);
static void handle_can_interlock(uint8_t *data);
static void handle_can_alarm(uint8_t *data);

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

    create_bsmp_var(33, 0, 4, false, V_CAPBANK.u8);
    create_bsmp_var(34, 0, 4, false, IOUT_RECT.u8);

    create_bsmp_var(35, 0, 4, false, DUTY_CYCLE.u8);

    create_bsmp_var(36, 0, 4, false, iib_fac_is.Iin.u8);
    create_bsmp_var(37, 0, 4, false, iib_fac_is.Vin.u8);
    create_bsmp_var(38, 0, 4, false, iib_fac_is.TempIGBT.u8);
    create_bsmp_var(39, 0, 4, false, iib_fac_is.DriverVoltage.u8);
    create_bsmp_var(40, 0, 4, false, iib_fac_is.DriverCurrent.u8);
    create_bsmp_var(41, 0, 4, false, iib_fac_is.TempL.u8);
    create_bsmp_var(42, 0, 4, false, iib_fac_is.TempHeatsink.u8);
    create_bsmp_var(43, 0, 4, false, iib_fac_is.BoardTemperature.u8);
    create_bsmp_var(44, 0, 4, false, iib_fac_is.RelativeHumidity.u8);
    create_bsmp_var(45, 0, 4, false, iib_fac_is.InterlocksRegister.u8);
    create_bsmp_var(46, 0, 4, false, iib_fac_is.AlarmsRegister.u8);

    create_bsmp_var(47, 0, 4, false, iib_fac_cmd.Vout.u8);
    create_bsmp_var(48, 0, 4, false, iib_fac_cmd.VcapBank.u8);
    create_bsmp_var(49, 0, 4, false, iib_fac_cmd.TempRectInductor.u8);
    create_bsmp_var(50, 0, 4, false, iib_fac_cmd.TempRectHeatSink.u8);
    create_bsmp_var(51, 0, 4, false, iib_fac_cmd.ExternalBoardsVoltage.u8);
    create_bsmp_var(52, 0, 4, false, iib_fac_cmd.AuxiliaryBoardCurrent.u8);
    create_bsmp_var(53, 0, 4, false, iib_fac_cmd.IDBBoardCurrent.u8);
    create_bsmp_var(54, 0, 4, false, iib_fac_cmd.GroundLeakage.u8);
    create_bsmp_var(55, 0, 4, false, iib_fac_cmd.BoardTemperature.u8);
    create_bsmp_var(56, 0, 4, false, iib_fac_cmd.RelativeHumidity.u8);
    create_bsmp_var(57, 0, 4, false, iib_fac_cmd.InterlocksRegister.u8);
    create_bsmp_var(58, 0, 4, false, iib_fac_cmd.AlarmsRegister.u8);
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

    init_scope(&g_ipc_mtoc.scope[0], ISR_CONTROL_FREQ.f,
               SCOPE_FREQ_SAMPLING_PARAM[0].f, &(g_buf_samples_ctom[0].f),
               SIZE_BUF_SAMPLES_CTOM, SCOPE_SOURCE_PARAM[0].p_f,
               (void *) 0);
}

static void init_iib_modules()
{
    iib_fac_is.CanAddress = IIB_IS_ADDRESS;
    iib_fac_cmd.CanAddress = IIB_CMD_ADDRESS;

    init_iib_module_can_data(&g_iib_module_can_data, &handle_can_data);
    init_iib_module_can_interlock(&g_iib_module_can_interlock, &handle_can_interlock);
    init_iib_module_can_alarm(&g_iib_module_can_alarm, &handle_can_alarm);
}

static void handle_can_data(uint8_t *data)
{
    uint8_t iib_address;
    uint8_t data_id;

    iib_address = data[0];
    data_id    = data[1];

    switch(iib_address)
    {
        /// Data from FAC IS
        case IIB_IS_ADDRESS:
        {
            switch(data_id)
            {
                case 0:
                {
                    memcpy(iib_fac_is.Vin.u8, &data[4], 4);
                    VOUT_RECT.f = iib_fac_is.Vin.f;
                    break;
                }
                case 1:
                {
                    memcpy(iib_fac_is.Iin.u8, &data[4], 4);
                    break;
                }
                case 2:
                {
                    memcpy(iib_fac_is.TempIGBT.u8, &data[4], 4);
                    break;
                }
                case 3:
                {
                    memcpy(iib_fac_is.DriverVoltage.u8, &data[4], 4);
                    break;
                }
                case 4:
                {
                    memcpy(iib_fac_is.DriverCurrent.u8, &data[4], 4);
                    break;
                }
                case 5:
                {
                    memcpy(iib_fac_is.TempL.u8, &data[4], 4);
                    break;
                }
                case 6:
                {
                    memcpy(iib_fac_is.TempHeatsink.u8, &data[4], 4);
                    break;
                }
                case 7:
                {
                    memcpy(iib_fac_is.BoardTemperature.u8, &data[4], 4);
                    break;
                }
                case 8:
                {
                    memcpy(iib_fac_is.RelativeHumidity.u8, &data[4], 4);
                    break;
                }

                default:
                {
                    break;
                }
            }

            break;
        }

        /// Data from FAC Cmd
        case IIB_CMD_ADDRESS:
        {
            switch(data_id)
            {
                case 0:
                {
                    memcpy(iib_fac_cmd.VcapBank.u8, &data[4], 4);
                    break;
                }
                case 1:
                {
                    memcpy(iib_fac_cmd.Vout.u8, &data[4], 4);
                    break;
                }
                case 2:
                {
                    memcpy(iib_fac_cmd.ExternalBoardsVoltage.u8, &data[4], 4);
                    break;
                }
                case 3:
                {
                    memcpy(iib_fac_cmd.AuxiliaryBoardCurrent.u8, &data[4], 4);
                    break;
                }
                case 4:
                {
                    memcpy(iib_fac_cmd.IDBBoardCurrent.u8, &data[4], 4);
                    break;
                }
                case 5:
                {
                    memcpy(iib_fac_cmd.GroundLeakage.u8, &data[4], 4);
                    break;
                }
                case 6:
                {
                    memcpy(iib_fac_cmd.TempRectInductor.u8, &data[4], 4);
                    break;
                }
                case 7:
                {
                    memcpy(iib_fac_cmd.TempRectHeatSink.u8, &data[4], 4);
                    break;
                }
                case 8:
                {
                    memcpy(iib_fac_cmd.BoardTemperature.u8, &data[4], 4);
                    break;
                }
                case 9:
                {
                    memcpy(iib_fac_cmd.RelativeHumidity.u8, &data[4], 4);
                    break;
                }

                default:
                {
                    break;
                }
            }

            break;
        }

        default:
        {
            break;
        }
    }
}

static void handle_can_interlock(uint8_t *data)
{
    uint8_t iib_address;
    uint8_t data_id;

    iib_address = data[0];
    data_id    = data[1];

    switch(iib_address)
    {
        /// Interlocks from FAC IS
        case IIB_IS_ADDRESS:
        {
            switch(data_id)
            {
               case 0:
               {
                   if(g_can_reset_flag[0])
                   {
                       memcpy(iib_fac_is.InterlocksRegister.u8, &data[4], 4);
                       set_hard_interlock(0, IIB_IS_Itlk);
                   }
                   break;
               }

               case 1:
               {
                   g_can_reset_flag[0] = 1;
                   iib_fac_is.InterlocksRegister.u32 = 0;
                   break;
               }

               default:
               {
                   break;
               }
            }
        }

        /// Interlocks from FAC Cmd
        case IIB_CMD_ADDRESS:
        {
            switch(data_id)
            {
               case 0:
               {
                   if(g_can_reset_flag[1])
                   {
                       memcpy(iib_fac_cmd.InterlocksRegister.u8, &data[4], 4);
                       set_hard_interlock(0, IIB_Cmd_Itlk);
                   }
                   break;
               }

               case 1:
               {
                   g_can_reset_flag[1] = 1;
                   iib_fac_cmd.InterlocksRegister.u32 = 0;
                   break;
               }

               default:
               {
                   break;
               }
            }
        }

        default:
        {
            break;
        }
    }
}

static void handle_can_alarm(uint8_t *data)
{
    uint8_t iib_address;
    uint8_t data_id;

    iib_address = data[0];
    data_id    = data[1];

    switch(iib_address)
    {
        /// Alarms from FAC IS
        case IIB_IS_ADDRESS:
        {
            switch(data_id)
            {
               case 0:
               {
                   memcpy(iib_fac_is.AlarmsRegister.u8, &data[4], 4);
                   break;
               }

               case 1:
               {
                   iib_fac_is.AlarmsRegister.u32 = 0;
                   break;
               }

               default:
               {
                   break;
               }
            }
        }

        /// Alarms from FAC Cmd
        case IIB_CMD_ADDRESS:
        {
            switch(data_id)
            {
               case 0:
               {
                   memcpy(iib_fac_cmd.AlarmsRegister.u8, &data[4], 4);
                   break;
               }

               case 1:
               {
                   iib_fac_cmd.AlarmsRegister.u32 = 0;
                   break;
               }

               default:
               {
                   break;
               }
            }
        }

        default:
        {
            break;
        }
    }
}
