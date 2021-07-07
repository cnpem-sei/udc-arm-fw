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
#include "communication_drivers/can/can_bkp.h"
#include "communication_drivers/control/control.h"
#include "communication_drivers/control/wfmref/wfmref.h"
#include "communication_drivers/event_manager/event_manager.h"
#include "communication_drivers/iib/iib_data.h"
#include "communication_drivers/iib/iib_module.h"
#include "communication_drivers/ps_modules/fap/fap.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/**
 * Controller defines
 */

/// DSP Net Signals
#define I_LOAD_1                g_controller_ctom.net_signals[0]    // HRADC0
#define I_LOAD_2                g_controller_ctom.net_signals[1]    // HRADC1
#define V_DCLINK                g_controller_ctom.net_signals[2]    // HRADC2

#define I_LOAD_MEAN             g_controller_ctom.net_signals[3]
#define I_LOAD_ERROR            g_controller_ctom.net_signals[4]
#define I_LOAD_DIFF             g_controller_ctom.net_signals[16]

#define I_IGBTS_DIFF            g_controller_ctom.net_signals[6]

#define DUTY_MEAN               g_controller_ctom.net_signals[5]
#define DUTY_DIFF               g_controller_ctom.net_signals[7]

#define DUTY_CYCLE_IGBT_1       g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_IGBT_2       g_controller_ctom.output_signals[1]

/// ARM Net Signals
#define I_IGBT_1                g_controller_mtoc.net_signals[0]    // ANI0
#define I_IGBT_2                g_controller_mtoc.net_signals[1]    // ANI1

#define V_LOAD                  g_controller_mtoc.net_signals[2]

#define IIB_V_IN_GLITCH             g_controller_mtoc.net_signals[12] // 0x0000C018
#define IIB_V_OUT_GLITCH            g_controller_mtoc.net_signals[13] // 0x0000C01A
#define IIB_I_IGBT_1_GLITCH         g_controller_mtoc.net_signals[14] // 0x0000C01C
#define IIB_I_IGBT_2_GLITCH         g_controller_mtoc.net_signals[15] // 0x0000C01E
#define IIB_TEMP_IGBT_1_GLITCH      g_controller_mtoc.net_signals[16] // 0x0000C020
#define IIB_TEMP_IGBT_2_GLITCH      g_controller_mtoc.net_signals[17] // 0x0000C022
#define IIB_V_DRIVER_GLITCH         g_controller_mtoc.net_signals[18] // 0x0000C024
#define IIB_I_DRIVER_1_GLITCH       g_controller_mtoc.net_signals[19] // 0x0000C026
#define IIB_I_DRIVER_2_GLITCH       g_controller_mtoc.net_signals[20] // 0x0000C028
#define IIB_TEMP_L_GLITCH           g_controller_mtoc.net_signals[21] // 0x0000C02A
#define IIB_TEMP_HEATSINK_GLITCH    g_controller_mtoc.net_signals[22] // 0x0000C02C
#define IIB_I_LEAKAGE_GLITCH        g_controller_mtoc.net_signals[23] // 0x0000C02E
#define IIB_TEMP_BOARD_GLITCH       g_controller_mtoc.net_signals[24] // 0x0000C030
#define IIB_RH_BOARD_GLITCH         g_controller_mtoc.net_signals[25] // 0x0000C032
#define IIB_ITLK_GLITCH             g_controller_mtoc.net_signals[26] // 0x0000C034
#define IIB_ALARM_GLITCH            g_controller_mtoc.net_signals[27] // 0x0000C036

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Load_Overvoltage,
    DCLink_Overvoltage,
    DCLink_Undervoltage,
    Welded_Contactor_Fault,
    Opened_Contactor_Fault,
    IGBT_1_Overcurrent,
    IGBT_2_Overcurrent,
    IIB_Itlk
} hard_interlocks_t;

typedef enum
{
    DCCT_1_Fault,
    DCCT_2_Fault,
    DCCT_High_Difference,
    Load_Feedback_1_Fault,
    Load_Feedback_2_Fault,
    IGBTs_Current_High_Difference,
} soft_interlocks_t;

static volatile iib_fap_module_t iib_fap;

static void init_iib();

static void handle_can_data(uint8_t *data, unsigned long id);

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAP operation.
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
    create_bsmp_var(31, 0, 4, false, g_ipc_ctom.ps_module[0].ps_soft_interlock.u8);
    create_bsmp_var(32, 0, 4, false, g_ipc_ctom.ps_module[0].ps_hard_interlock.u8);

    create_bsmp_var(33, 0, 4, false, I_LOAD_MEAN.u8);
    create_bsmp_var(34, 0, 4, false, I_LOAD_1.u8);
    create_bsmp_var(35, 0, 4, false, I_LOAD_2.u8);

    create_bsmp_var(36, 0, 4, false, V_DCLINK.u8);
    create_bsmp_var(37, 0, 4, false, I_IGBT_1.u8);
    create_bsmp_var(38, 0, 4, false, I_IGBT_2.u8);

    create_bsmp_var(39, 0, 4, false, DUTY_CYCLE_IGBT_1.u8);
    create_bsmp_var(40, 0, 4, false, DUTY_CYCLE_IGBT_2.u8);
    create_bsmp_var(41, 0, 4, false, DUTY_DIFF.u8);

    create_bsmp_var(42, 0, 4, false, iib_fap.Vin.u8);
    create_bsmp_var(43, 0, 4, false, iib_fap.Vout.u8);
    create_bsmp_var(44, 0, 4, false, iib_fap.IoutA1.u8);
    create_bsmp_var(45, 0, 4, false, iib_fap.IoutA2.u8);
    create_bsmp_var(46, 0, 4, false, iib_fap.TempIGBT1.u8);
    create_bsmp_var(47, 0, 4, false, iib_fap.TempIGBT2.u8);
    create_bsmp_var(48, 0, 4, false, iib_fap.DriverVoltage.u8);
    create_bsmp_var(49, 0, 4, false, iib_fap.Driver1Current.u8);
    create_bsmp_var(50, 0, 4, false, iib_fap.Driver2Current.u8);
    create_bsmp_var(51, 0, 4, false, iib_fap.TempL.u8);
    create_bsmp_var(52, 0, 4, false, iib_fap.TempHeatSink.u8);
    create_bsmp_var(53, 0, 4, false, iib_fap.GroundLeakage.u8);
    create_bsmp_var(54, 0, 4, false, iib_fap.BoardTemperature.u8);
    create_bsmp_var(55, 0, 4, false, iib_fap.RelativeHumidity.u8);
    create_bsmp_var(56, 0, 4, false, iib_fap.InterlocksRegister.u8);
    create_bsmp_var(57, 0, 4, false, iib_fap.AlarmsRegister.u8);
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
    init_iib();

    init_wfmref(&WFMREF[0], WFMREF_SELECTED_PARAM[0].u16,
                WFMREF_SYNC_MODE_PARAM[0].u16, ISR_CONTROL_FREQ.f,
                WFMREF_FREQUENCY_PARAM[0].f, WFMREF_GAIN_PARAM[0].f,
                WFMREF_OFFSET_PARAM[0].f, &g_wfmref_data.data[0][0].f,
                SIZE_WFMREF, &g_ipc_ctom.ps_module[0].ps_reference.f);

    init_scope(&g_ipc_mtoc.scope[0], ISR_CONTROL_FREQ.f,
               SCOPE_FREQ_SAMPLING_PARAM[0].f, &(g_buf_samples_ctom[0].f),
               SIZE_BUF_SAMPLES_CTOM, SCOPE_SOURCE_PARAM[0].p_f,
               (void *) 0);
}

static void init_iib()
{
    iib_fap.CanAddress = 1;

    init_iib_module_can_data(&g_iib_module_can_data, &handle_can_data);
}

static void handle_can_data(uint8_t *data, unsigned long id)
{
    switch(id)
    {
        case 10:
        {
            memcpy(iib_fap.Vin.u8, &data[0], 4);
            memcpy(iib_fap.Vout.u8, &data[4], 4);
            V_LOAD.f = iib_fap.Vout.f;

            if( (iib_fap.Vin.f < -20.0) ||
                (iib_fap.Vin.f > 150.0) )
            {
                IIB_V_IN_GLITCH.f = iib_fap.Vin.f;
            }

            if( (iib_fap.Vout.f < -20.0) ||
                (iib_fap.Vout.f > 150.0) )
            {
                IIB_V_OUT_GLITCH.f = iib_fap.Vout.f;
            }

            break;
        }
        case 11:
        {
            memcpy(iib_fap.IoutA1.u8, &data[0], 4);
            memcpy(iib_fap.IoutA2.u8, &data[4], 4);

            if( (iib_fap.IoutA1.f < -20.0) ||
                (iib_fap.IoutA1.f > 350.0) )
            {
                IIB_I_IGBT_1_GLITCH.f = iib_fap.IoutA1.f;
            }

            if( (iib_fap.IoutA2.f < -20.0) ||
                (iib_fap.IoutA2.f > 350.0) )
            {
                IIB_I_IGBT_2_GLITCH.f = iib_fap.IoutA2.f;
            }

            break;
        }
        case 12:
        {
        	memcpy(iib_fap.DriverVoltage.u8, &data[0], 4);
        	memcpy(iib_fap.GroundLeakage.u8, &data[4], 4);

        	if( (iib_fap.DriverVoltage.f < -20.0) ||
                (iib_fap.DriverVoltage.f > 50.0) )
            {
                IIB_V_DRIVER_GLITCH.f = iib_fap.DriverVoltage.f;
            }

            if( (iib_fap.GroundLeakage.f < -20.0) ||
                (iib_fap.GroundLeakage.f > 50.0) )
            {
                IIB_I_LEAKAGE_GLITCH.f = iib_fap.GroundLeakage.f;
            }

            break;
        }
        case 13:
        {
        	memcpy(iib_fap.Driver1Current.u8, &data[0], 4);
        	memcpy(iib_fap.Driver2Current.u8, &data[4], 4);

        	if( (iib_fap.Driver1Current.f < -50.0) ||
                (iib_fap.Driver1Current.f > 50.0) )
            {
                IIB_I_DRIVER_1_GLITCH.f = iib_fap.Driver1Current.f;
            }

            if( (iib_fap.Driver2Current.f < -50.0) ||
                (iib_fap.Driver2Current.f > 50.0) )
            {
                IIB_I_DRIVER_2_GLITCH.f = iib_fap.Driver2Current.f;
            }

            break;
        }
        case 14:
        {
            memcpy(iib_fap.TempIGBT1.u8, &data[0], 4);
            memcpy(iib_fap.TempIGBT2.u8, &data[4], 4);

            if( (iib_fap.TempIGBT1.f < -50.0) ||
                (iib_fap.TempIGBT1.f > 150.0) )
            {
                IIB_TEMP_IGBT_1_GLITCH.f = iib_fap.TempIGBT1.f;
            }

            if( (iib_fap.TempIGBT2.f < -50.0) ||
                (iib_fap.TempIGBT2.f > 150.0) )
            {
                IIB_TEMP_IGBT_1_GLITCH.f = iib_fap.TempIGBT2.f;
            }

            break;
        }
        case 15:
        {
        	memcpy(iib_fap.TempL.u8, &data[0], 4);
        	memcpy(iib_fap.TempHeatSink.u8, &data[4], 4);

        	if( (iib_fap.TempL.f < -10.0) ||
                (iib_fap.TempL.f > 100.0) )
            {
                IIB_TEMP_L_GLITCH.f = iib_fap.TempL.f;
            }

            if( (iib_fap.TempHeatSink.f < -10.0) ||
                (iib_fap.TempHeatSink.f > 100.0) )
            {
                IIB_TEMP_HEATSINK_GLITCH.f = iib_fap.TempHeatSink.f;
            }

            break;
        }
        case 16:
        {
        	memcpy(iib_fap.BoardTemperature.u8, &data[0], 4);
        	memcpy(iib_fap.RelativeHumidity.u8, &data[4], 4);

        	if( (iib_fap.BoardTemperature.f < -10.0) ||
                (iib_fap.BoardTemperature.f > 150.0) )
            {
                IIB_TEMP_BOARD_GLITCH.f = iib_fap.BoardTemperature.f;
            }

            if( (iib_fap.RelativeHumidity.f < -10.0) ||
                (iib_fap.RelativeHumidity.f > 100.0) )
            {
                IIB_RH_BOARD_GLITCH.f = iib_fap.RelativeHumidity.f;
            }

            break;
        }
        case 17:
        {
        	memcpy(iib_fap.InterlocksRegister.u8, &data[0], 4);
        	memcpy(iib_fap.AlarmsRegister.u8, &data[4], 4);

        	if(iib_fap.InterlocksRegister.u32 > 0x000FFFFF)
            {
                IIB_ITLK_GLITCH.u32 = iib_fap.InterlocksRegister.u32;
            }

            else if(iib_fap.InterlocksRegister.u32 > 0)
        	{
        		set_hard_interlock(0, IIB_Itlk);
        	}
        	else
        	{
        		iib_fap.InterlocksRegister.u32 = 0;
        	}

        	if(iib_fap.AlarmsRegister.u32 > 0x00003FFF)
            {
                IIB_ALARM_GLITCH.u32 = iib_fap.AlarmsRegister.u32;
            }

        	break;
        }
        default:
        {
            break;
        }
    }
}
