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
 * @file fac_2s_acdc.c
 * @brief FAC-2S AC/DC Stage module
 *
 * Module for control of two AC/DC modules of FAC power supplies for focusing
 * quadrupoles from booster. It implements the individual controllers for input
 * current and capacitor bank voltage of each AC/DC module.
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

#include "driverlib/gpio.h"
#include "board_drivers/hardware_def.h"

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/can/can_bkp.h"
#include "communication_drivers/control/control.h"
#include "communication_drivers/event_manager/event_manager.h"
#include "communication_drivers/iib/iib_data.h"
#include "communication_drivers/iib/iib_module.h"
#include "communication_drivers/ps_modules/fac_2s_acdc/fac_2s_acdc.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/**
 * IIB Defines
 */
#define IIB_IS_ADDRESS_MOD_A      1
#define IIB_IS_ADDRESS_MOD_B      2
#define IIB_CMD_ADDRESS_MOD_A     3
#define IIB_CMD_ADDRESS_MOD_B     4

/**
 * Controller defines
 */
#define MOD_A_ID                    0x0
#define MOD_B_ID                    0x1

/// DSP Net Signals
#define V_CAPBANK_MOD_A                     g_controller_ctom.net_signals[0]  // HRADC0
#define I_OUT_RECT_MOD_A                    g_controller_ctom.net_signals[1]  // HRADC1
#define V_CAPBANK_MOD_B                     g_controller_ctom.net_signals[2]  // HRADC2
#define I_OUT_RECT_MOD_B                    g_controller_ctom.net_signals[3]  // HRADC3

#define V_CAPBANK_FILTERED_2HZ_MOD_A        g_controller_ctom.net_signals[4]
#define V_CAPBANK_FILTERED_2Hz_4HZ_MOD_A    g_controller_ctom.net_signals[5]
#define V_CAPBANK_ERROR_MOD_A               g_controller_ctom.net_signals[6]

#define I_OUT_RECT_REF_MOD_A                g_controller_ctom.net_signals[7]
#define I_OUT_RECT_ERROR_MOD_A              g_controller_ctom.net_signals[8]
#define I_OUT_RECT_RESS_2HZ_MOD_A           g_controller_ctom.net_signals[9]
#define I_OUT_RECT_RESS_2HZ_4HZ_MOD_A       g_controller_ctom.net_signals[10]

#define V_CAPBANK_FILTERED_2HZ_MOD_B        g_controller_ctom.net_signals[11]
#define V_CAPBANK_FILTERED_2Hz_4HZ_MOD_B    g_controller_ctom.net_signals[12]
#define V_CAPBANK_ERROR_MOD_B               g_controller_ctom.net_signals[13]

#define I_OUT_RECT_REF_MOD_B                g_controller_ctom.net_signals[14]
#define I_OUT_RECT_ERROR_MOD_B              g_controller_ctom.net_signals[15]
#define I_OUT_RECT_RESS_2HZ_MOD_B           g_controller_ctom.net_signals[16]
#define I_OUT_RECT_RESS_2HZ_4HZ_MOD_B       g_controller_ctom.net_signals[17]

#define DUTY_CYCLE_MOD_A                    g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_MOD_B                    g_controller_ctom.output_signals[1]

/// ARM Net Signals
#define V_OUT_RECT_MOD_A                     g_controller_mtoc.net_signals[0]
#define V_OUT_RECT_MOD_B                     g_controller_mtoc.net_signals[1]

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

volatile iib_fac_is_t fac_2s_acdc_is[2];
volatile iib_fac_cmd_t fac_2s_acdc_cmd[2];

static void init_iib_modules();

static void handle_can_data(uint8_t *data);
static void handle_can_interlock(uint8_t *data);
static void handle_can_alarm(uint8_t *data);

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC-2P4S ACDC operation.
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
* Setup BSMP servers for FAC-2P4S AC/DC operation.
*
*/
static void bsmp_init_server(void)
{
    /**
     * Create module A specific variables
     */
    create_bsmp_var(31, MOD_A_ID, 4, false, g_ipc_ctom.ps_module[MOD_A_ID].ps_soft_interlock.u8);
    create_bsmp_var(32, MOD_A_ID, 4, false, g_ipc_ctom.ps_module[MOD_A_ID].ps_hard_interlock.u8);

    create_bsmp_var(33, MOD_A_ID, 4, false, V_CAPBANK_MOD_A.u8);
    create_bsmp_var(34, MOD_A_ID, 4, false, I_OUT_RECT_MOD_A.u8);

    create_bsmp_var(35, MOD_A_ID, 4, false, DUTY_CYCLE_MOD_A.u8);

    create_bsmp_var(36, MOD_A_ID, 4, false, fac_2s_acdc_is[MOD_A_ID].Iin.u8);
    create_bsmp_var(37, MOD_A_ID, 4, false, fac_2s_acdc_is[MOD_A_ID].Vin.u8);
    create_bsmp_var(38, MOD_A_ID, 4, false, fac_2s_acdc_is[MOD_A_ID].TempIGBT.u8);
    create_bsmp_var(39, MOD_A_ID, 4, false, fac_2s_acdc_is[MOD_A_ID].DriverVoltage.u8);
    create_bsmp_var(40, MOD_A_ID, 4, false, fac_2s_acdc_is[MOD_A_ID].DriverCurrent.u8);
    create_bsmp_var(41, MOD_A_ID, 4, false, fac_2s_acdc_is[MOD_A_ID].TempL.u8);
    create_bsmp_var(42, MOD_A_ID, 4, false, fac_2s_acdc_is[MOD_A_ID].TempHeatsink.u8);
    create_bsmp_var(43, MOD_A_ID, 4, false, fac_2s_acdc_is[MOD_A_ID].BoardTemperature.u8);
    create_bsmp_var(44, MOD_A_ID, 4, false, fac_2s_acdc_is[MOD_A_ID].RelativeHumidity.u8);
    create_bsmp_var(45, MOD_A_ID, 4, false, fac_2s_acdc_is[MOD_A_ID].InterlocksRegister.u8);
    create_bsmp_var(46, MOD_A_ID, 4, false, fac_2s_acdc_is[MOD_A_ID].AlarmsRegister.u8);

    create_bsmp_var(47, MOD_A_ID, 4, false, fac_2s_acdc_cmd[MOD_A_ID].Vout.u8);
    create_bsmp_var(48, MOD_A_ID, 4, false, fac_2s_acdc_cmd[MOD_A_ID].VcapBank.u8);
    create_bsmp_var(49, MOD_A_ID, 4, false, fac_2s_acdc_cmd[MOD_A_ID].TempRectInductor.u8);
    create_bsmp_var(50, MOD_A_ID, 4, false, fac_2s_acdc_cmd[MOD_A_ID].TempRectHeatSink.u8);
    create_bsmp_var(51, MOD_A_ID, 4, false, fac_2s_acdc_cmd[MOD_A_ID].ExternalBoardsVoltage.u8);
    create_bsmp_var(52, MOD_A_ID, 4, false, fac_2s_acdc_cmd[MOD_A_ID].AuxiliaryBoardCurrent.u8);
    create_bsmp_var(53, MOD_A_ID, 4, false, fac_2s_acdc_cmd[MOD_A_ID].IDBBoardCurrent.u8);
    create_bsmp_var(54, MOD_A_ID, 4, false, fac_2s_acdc_cmd[MOD_A_ID].GroundLeakage.u8);
    create_bsmp_var(55, MOD_A_ID, 4, false, fac_2s_acdc_cmd[MOD_A_ID].BoardTemperature.u8);
    create_bsmp_var(56, MOD_A_ID, 4, false, fac_2s_acdc_cmd[MOD_A_ID].RelativeHumidity.u8);
    create_bsmp_var(57, MOD_A_ID, 4, false, fac_2s_acdc_cmd[MOD_A_ID].InterlocksRegister.u8);
    create_bsmp_var(58, MOD_A_ID, 4, false, fac_2s_acdc_cmd[MOD_A_ID].AlarmsRegister.u8);

    /**
     * Create module B specific variables
     */

    /// Module A BSMP server already initialized
    bsmp_init(MOD_B_ID);

    /// Both modules share these variables
    modify_bsmp_var(0, MOD_B_ID, g_ipc_ctom.ps_module[0].ps_status.u8);
    modify_bsmp_var(1, MOD_B_ID, g_ipc_ctom.ps_module[0].ps_setpoint.u8);
    modify_bsmp_var(2, MOD_B_ID, g_ipc_ctom.ps_module[0].ps_reference.u8);

    create_bsmp_var(31, MOD_B_ID, 4, false, g_ipc_ctom.ps_module[MOD_B_ID].ps_soft_interlock.u8);
    create_bsmp_var(32, MOD_B_ID, 4, false, g_ipc_ctom.ps_module[MOD_B_ID].ps_hard_interlock.u8);

    create_bsmp_var(33, MOD_B_ID, 4, false, V_CAPBANK_MOD_B.u8);
    create_bsmp_var(34, MOD_B_ID, 4, false, I_OUT_RECT_MOD_B.u8);

    create_bsmp_var(35, MOD_B_ID, 4, false, DUTY_CYCLE_MOD_B.u8);

    create_bsmp_var(36, MOD_B_ID, 4, false, fac_2s_acdc_is[MOD_B_ID].Iin.u8);
    create_bsmp_var(37, MOD_B_ID, 4, false, fac_2s_acdc_is[MOD_B_ID].Vin.u8);
    create_bsmp_var(38, MOD_B_ID, 4, false, fac_2s_acdc_is[MOD_B_ID].TempIGBT.u8);
    create_bsmp_var(39, MOD_B_ID, 4, false, fac_2s_acdc_is[MOD_B_ID].DriverVoltage.u8);
    create_bsmp_var(40, MOD_B_ID, 4, false, fac_2s_acdc_is[MOD_B_ID].DriverCurrent.u8);
    create_bsmp_var(41, MOD_B_ID, 4, false, fac_2s_acdc_is[MOD_B_ID].TempL.u8);
    create_bsmp_var(42, MOD_B_ID, 4, false, fac_2s_acdc_is[MOD_B_ID].TempHeatsink.u8);
    create_bsmp_var(43, MOD_B_ID, 4, false, fac_2s_acdc_is[MOD_B_ID].BoardTemperature.u8);
    create_bsmp_var(44, MOD_B_ID, 4, false, fac_2s_acdc_is[MOD_B_ID].RelativeHumidity.u8);
    create_bsmp_var(45, MOD_B_ID, 4, false, fac_2s_acdc_is[MOD_B_ID].InterlocksRegister.u8);
    create_bsmp_var(46, MOD_B_ID, 4, false, fac_2s_acdc_is[MOD_B_ID].AlarmsRegister.u8);

    create_bsmp_var(47, MOD_B_ID, 4, false, fac_2s_acdc_cmd[MOD_B_ID].Vout.u8);
    create_bsmp_var(48, MOD_B_ID, 4, false, fac_2s_acdc_cmd[MOD_B_ID].VcapBank.u8);
    create_bsmp_var(49, MOD_B_ID, 4, false, fac_2s_acdc_cmd[MOD_B_ID].TempRectInductor.u8);
    create_bsmp_var(50, MOD_B_ID, 4, false, fac_2s_acdc_cmd[MOD_B_ID].TempRectHeatSink.u8);
    create_bsmp_var(51, MOD_B_ID, 4, false, fac_2s_acdc_cmd[MOD_B_ID].ExternalBoardsVoltage.u8);
    create_bsmp_var(52, MOD_B_ID, 4, false, fac_2s_acdc_cmd[MOD_B_ID].AuxiliaryBoardCurrent.u8);
    create_bsmp_var(53, MOD_B_ID, 4, false, fac_2s_acdc_cmd[MOD_B_ID].IDBBoardCurrent.u8);
    create_bsmp_var(54, MOD_B_ID, 4, false, fac_2s_acdc_cmd[MOD_B_ID].GroundLeakage.u8);
    create_bsmp_var(55, MOD_B_ID, 4, false, fac_2s_acdc_cmd[MOD_B_ID].BoardTemperature.u8);
    create_bsmp_var(56, MOD_B_ID, 4, false, fac_2s_acdc_cmd[MOD_B_ID].RelativeHumidity.u8);
    create_bsmp_var(57, MOD_B_ID, 4, false, fac_2s_acdc_cmd[MOD_B_ID].InterlocksRegister.u8);
    create_bsmp_var(58, MOD_B_ID, 4, false, fac_2s_acdc_cmd[MOD_B_ID].AlarmsRegister.u8);
}

/**
* @brief System configuration for FAC-2P4S AC/DC.
*
* Initialize specific parameters e configure peripherals for FAC-2P4S AC/DC
* operation.
*
*/
void fac_2s_acdc_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
    init_iib_modules();

    init_scope(&g_ipc_mtoc.scope[MOD_A_ID], ISR_CONTROL_FREQ.f,
               SCOPE_FREQ_SAMPLING_PARAM[MOD_A_ID].f, &(g_buf_samples_ctom[0].f),
               SIZE_BUF_SAMPLES_CTOM/2, SCOPE_SOURCE_PARAM[MOD_A_ID].p_f,
               (void *) 0);

    init_scope(&g_ipc_mtoc.scope[MOD_B_ID], ISR_CONTROL_FREQ.f,
               SCOPE_FREQ_SAMPLING_PARAM[MOD_B_ID].f,
               &(g_buf_samples_ctom[SIZE_BUF_SAMPLES_CTOM/2].f),
               SIZE_BUF_SAMPLES_CTOM/2, SCOPE_SOURCE_PARAM[MOD_B_ID].p_f,
               (void *) 0);
}

static void init_iib_modules()
{
    fac_2s_acdc_is[MOD_A_ID].CanAddress = IIB_IS_ADDRESS_MOD_A;
    fac_2s_acdc_is[MOD_B_ID].CanAddress = IIB_IS_ADDRESS_MOD_B;
    fac_2s_acdc_cmd[MOD_A_ID].CanAddress = IIB_CMD_ADDRESS_MOD_A;
    fac_2s_acdc_cmd[MOD_B_ID].CanAddress = IIB_CMD_ADDRESS_MOD_B;

    init_iib_module_can_data(&g_iib_module_can_data, &handle_can_data);
    init_iib_module_can_interlock(&g_iib_module_can_interlock, &handle_can_interlock);
    init_iib_module_can_alarm(&g_iib_module_can_alarm, &handle_can_alarm);
}

static void handle_can_data(uint8_t *data)
{
    static uint8_t iib_address;
    static uint8_t data_id;
    static uint8_t module;

    iib_address = data[0];
    data_id     = data[1];
    module      = (data[0] - 1) % 2;

    switch(iib_address)
    {
        case IIB_IS_ADDRESS_MOD_A:
        case IIB_IS_ADDRESS_MOD_B:
        {
            switch(data_id)
            {
                case 0:
                {
                    memcpy(fac_2s_acdc_is[module].Vin.u8, &data[4], 4);
                    memcpy( (&V_OUT_RECT_MOD_A.f + module) , &data[4], 4);
                    break;
                }
                case 1:
                {
                    memcpy(fac_2s_acdc_is[module].Iin.u8, &data[4], 4);
                    break;
                }
                case 2:
                {
                    memcpy(fac_2s_acdc_is[module].TempIGBT.u8, &data[4], 4);
                    break;
                }
                case 3:
                {
                    memcpy(fac_2s_acdc_is[module].DriverVoltage.u8, &data[4], 4);
                    break;
                }
                case 4:
                {
                    memcpy(fac_2s_acdc_is[module].DriverCurrent.u8, &data[4], 4);
                    break;
                }
                case 5:
                {
                    memcpy(fac_2s_acdc_is[module].TempL.u8, &data[4], 4);
                    break;
                }
                case 6:
                {
                    memcpy(fac_2s_acdc_is[module].TempHeatsink.u8, &data[4], 4);
                    break;
                }
                case 7:
                {
                    memcpy(fac_2s_acdc_is[module].BoardTemperature.u8, &data[4], 4);
                    break;
                }
                case 8:
                {
                    memcpy(fac_2s_acdc_is[module].RelativeHumidity.u8, &data[4], 4);
                    break;
                }

                default:
                {
                    break;
                }
            }

            break;
        }

        case IIB_CMD_ADDRESS_MOD_A:
        case IIB_CMD_ADDRESS_MOD_B:
        {
            switch(data_id)
            {
                case 0:
                {
                    memcpy(fac_2s_acdc_cmd[module].VcapBank.u8, &data[4], 4);
                    break;
                }
                case 1:
                {
                    memcpy(fac_2s_acdc_cmd[module].Vout.u8, &data[4], 4);
                    break;
                }
                case 2:
                {
                    memcpy(fac_2s_acdc_cmd[module].ExternalBoardsVoltage.u8, &data[4], 4);
                    break;
                }
                case 3:
                {
                    memcpy(fac_2s_acdc_cmd[module].AuxiliaryBoardCurrent.u8, &data[4], 4);
                    break;
                }
                case 4:
                {
                    memcpy(fac_2s_acdc_cmd[module].IDBBoardCurrent.u8, &data[4], 4);
                    break;
                }
                case 5:
                {
                    memcpy(fac_2s_acdc_cmd[module].GroundLeakage.u8, &data[4], 4);
                    break;
                }
                case 6:
                {
                    memcpy(fac_2s_acdc_cmd[module].TempRectInductor.u8, &data[4], 4);
                    break;
                }
                case 7:
                {
                    memcpy(fac_2s_acdc_cmd[module].TempRectHeatSink.u8, &data[4], 4);
                    break;
                }
                case 8:
                {
                    memcpy(fac_2s_acdc_cmd[module].BoardTemperature.u8, &data[4], 4);
                    break;
                }
                case 9:
                {
                    memcpy(fac_2s_acdc_cmd[module].RelativeHumidity.u8, &data[4], 4);
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
    static uint8_t iib_address;
    static uint8_t data_id;
    static uint8_t module;

    iib_address = data[0];
    data_id     = data[1];
    module      = (data[0] - 1) % 2;

    switch(iib_address)
    {
        case IIB_IS_ADDRESS_MOD_A:
        case IIB_IS_ADDRESS_MOD_B:
        {
            switch(data_id)
            {
               case 0:
               {
                   if(g_can_reset_flag[iib_address-1])
                   {
                       memcpy(fac_2s_acdc_is[module].InterlocksRegister.u8, &data[4], 4);
                       set_hard_interlock(module, IIB_IS_Itlk);
                   }
                   break;
               }

               case 1:
               {
                   g_can_reset_flag[iib_address-1] = 1;
                   fac_2s_acdc_is[module].InterlocksRegister.u32 = 0;
                   break;
               }

               default:
               {
                   break;
               }
            }
        }

        case IIB_CMD_ADDRESS_MOD_A:
        case IIB_CMD_ADDRESS_MOD_B:
        {
            switch(data_id)
            {
               case 0:
               {
                   if(g_can_reset_flag[iib_address-1])
                   {
                       memcpy(fac_2s_acdc_cmd[module].InterlocksRegister.u8, &data[4], 4);
                       set_hard_interlock(module, IIB_Cmd_Itlk);
                   }
                   break;
               }

               case 1:
               {
                   g_can_reset_flag[iib_address-1] = 1;
                   fac_2s_acdc_cmd[module].InterlocksRegister.u32 = 0;
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
    static uint8_t iib_address;
    static uint8_t data_id;
    static uint8_t module;

    iib_address = data[0];
    data_id     = data[1];
    module      = (data[0] - 1) % 2;

    switch(iib_address)
    {
    case IIB_IS_ADDRESS_MOD_A:
    case IIB_IS_ADDRESS_MOD_B:
        {
            switch(data_id)
            {
               case 0:
               {
                   memcpy(fac_2s_acdc_is[module].AlarmsRegister.u8, &data[4], 4);
                   break;
               }

               case 1:
               {
                   fac_2s_acdc_is[module].AlarmsRegister.u32 = 0;
                   break;
               }

               default:
               {
                   break;
               }
            }
        }

        case IIB_CMD_ADDRESS_MOD_A:
        case IIB_CMD_ADDRESS_MOD_B:
        {
            switch(data_id)
            {
               case 0:
               {
                   memcpy(fac_2s_acdc_cmd[module].AlarmsRegister.u8, &data[4], 4);
                   break;
               }

               case 1:
               {
                   fac_2s_acdc_cmd[module].AlarmsRegister.u32 = 0;
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
