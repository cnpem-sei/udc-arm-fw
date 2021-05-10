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
#include "communication_drivers/can/can_bkp.h"
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

#define I_LOAD_MEAN                     g_controller_ctom.net_signals[4]
#define I_LOAD_ERROR                    g_controller_ctom.net_signals[5]

#define DUTY_I_LOAD_PI                  g_controller_ctom.net_signals[6]
#define DUTY_REF_FF                     g_controller_ctom.net_signals[7]

#define V_CAPBANK_MOD_1_FILTERED        g_controller_ctom.net_signals[8]
#define V_CAPBANK_MOD_2_FILTERED        g_controller_ctom.net_signals[9]

#define IN_FF_V_CAPBANK                 g_controller_ctom.net_signals[10]

#define I_LOAD_DIFF                     g_controller_ctom.net_signals[11]

#define WFMREF_IDX                      g_controller_ctom.net_signals[30]

#define DUTY_CYCLE_MOD_1                g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_MOD_2                g_controller_ctom.output_signals[1]

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Module_1_CapBank_Overvoltage,
    Module_2_CapBank_Overvoltage,
    Module_1_CapBank_Undervoltage,
    Module_2_CapBank_Undervoltage,
    IIB_Mod_1_Itlk,
    IIB_Mod_2_Itlk,
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

static volatile iib_fac_os_t iib_fac_2s_dcdc[2];

static void init_iib_modules();

static void handle_can_data(uint8_t *data, unsigned long id);

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

    create_bsmp_var(36, 0, 4, false, V_CAPBANK_MOD_1.u8);
    create_bsmp_var(37, 0, 4, false, V_CAPBANK_MOD_2.u8);

    create_bsmp_var(38, 0, 4, false, DUTY_CYCLE_MOD_1.u8);
    create_bsmp_var(39, 0, 4, false, DUTY_CYCLE_MOD_2.u8);

    create_bsmp_var(40, 0, 4, false, iib_fac_2s_dcdc[0].VdcLink.u8);
    create_bsmp_var(41, 0, 4, false, iib_fac_2s_dcdc[0].Iin.u8);
    create_bsmp_var(42, 0, 4, false, iib_fac_2s_dcdc[0].Iout.u8);
    create_bsmp_var(43, 0, 4, false, iib_fac_2s_dcdc[0].TempIGBT1.u8);
    create_bsmp_var(44, 0, 4, false, iib_fac_2s_dcdc[0].TempIGBT2.u8);
    create_bsmp_var(45, 0, 4, false, iib_fac_2s_dcdc[0].TempL.u8);
    create_bsmp_var(46, 0, 4, false, iib_fac_2s_dcdc[0].TempHeatSink.u8);
    create_bsmp_var(47, 0, 4, false, iib_fac_2s_dcdc[0].DriverVoltage.u8);
    create_bsmp_var(48, 0, 4, false, iib_fac_2s_dcdc[0].Driver1Current.u8);
    create_bsmp_var(49, 0, 4, false, iib_fac_2s_dcdc[0].Driver2Current.u8);
    create_bsmp_var(50, 0, 4, false, iib_fac_2s_dcdc[0].BoardTemperature.u8);
    create_bsmp_var(51, 0, 4, false, iib_fac_2s_dcdc[0].RelativeHumidity.u8);
    create_bsmp_var(52, 0, 4, false, iib_fac_2s_dcdc[0].InterlocksRegister.u8);
    create_bsmp_var(53, 0, 4, false, iib_fac_2s_dcdc[0].AlarmsRegister.u8);

    create_bsmp_var(54, 0, 4, false, iib_fac_2s_dcdc[1].VdcLink.u8);
    create_bsmp_var(55, 0, 4, false, iib_fac_2s_dcdc[1].Iin.u8);
    create_bsmp_var(56, 0, 4, false, iib_fac_2s_dcdc[1].Iout.u8);
    create_bsmp_var(57, 0, 4, false, iib_fac_2s_dcdc[1].TempIGBT1.u8);
    create_bsmp_var(58, 0, 4, false, iib_fac_2s_dcdc[1].TempIGBT2.u8);
    create_bsmp_var(59, 0, 4, false, iib_fac_2s_dcdc[1].TempL.u8);
    create_bsmp_var(60, 0, 4, false, iib_fac_2s_dcdc[1].TempHeatSink.u8);
    create_bsmp_var(61, 0, 4, false, iib_fac_2s_dcdc[1].DriverVoltage.u8);
    create_bsmp_var(62, 0, 4, false, iib_fac_2s_dcdc[1].Driver1Current.u8);
    create_bsmp_var(63, 0, 4, false, iib_fac_2s_dcdc[1].Driver2Current.u8);
    create_bsmp_var(64, 0, 4, false, iib_fac_2s_dcdc[1].BoardTemperature.u8);
    create_bsmp_var(65, 0, 4, false, iib_fac_2s_dcdc[1].RelativeHumidity.u8);
    create_bsmp_var(66, 0, 4, false, iib_fac_2s_dcdc[1].InterlocksRegister.u8);
    create_bsmp_var(67, 0, 4, false, iib_fac_2s_dcdc[1].AlarmsRegister.u8);
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
    init_iib_modules();

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

static void init_iib_modules()
{
    iib_fac_2s_dcdc[0].CanAddress = 1;
    iib_fac_2s_dcdc[1].CanAddress = 2;

    init_iib_module_can_data(&g_iib_module_can_data, &handle_can_data);
}

static void handle_can_data(uint8_t *data, unsigned long id)
{
    uint8_t module;

    unsigned long can_module = id;
    unsigned long id_var = 0;

    switch(can_module)
    {
    	case 10:
    	case 11:
    	case 12:
    	case 13:
    	case 14:
    	case 15:
    	case 16:
    	case 17:
    	{
    		module = 0;
    		id_var = (id - 10);
    		break;
    	}
    	case 20:
    	case 21:
    	case 22:
    	case 23:
    	case 24:
    	case 25:
    	case 26:
    	case 27:
    	{
    		module = 1;
    		id_var = (id - 20);
    		break;
    	}

    	default:
    	{
    		break;
    	}
    }

    switch(id_var)
    {
        case 0:
        {
            memcpy(iib_fac_2s_dcdc[module].VdcLink.u8, &data[0], 4);
            memcpy(iib_fac_2s_dcdc[module].DriverVoltage.u8, &data[4], 4);
            break;
        }
        case 1:
        {
            memcpy(iib_fac_2s_dcdc[module].Iin.u8, &data[0], 4);
            memcpy(iib_fac_2s_dcdc[module].Iout.u8, &data[4], 4);
            break;
        }
        case 2:
        {
        	memcpy(iib_fac_2s_dcdc[module].Driver1Current.u8, &data[0], 4);
        	memcpy(iib_fac_2s_dcdc[module].Driver2Current.u8, &data[4], 4);
            break;
        }
        case 3:
        {
            memcpy(iib_fac_2s_dcdc[module].TempIGBT1.u8, &data[0], 4);
            memcpy(iib_fac_2s_dcdc[module].TempIGBT2.u8, &data[4], 4);
            break;
        }
        case 4:
        {
        	memcpy(iib_fac_2s_dcdc[module].TempL.u8, &data[0], 4);
        	memcpy(iib_fac_2s_dcdc[module].TempHeatSink.u8, &data[4], 4);
            break;
        }
        case 5:
        {
        	memcpy(iib_fac_2s_dcdc[module].BoardTemperature.u8, &data[0], 4);
        	memcpy(iib_fac_2s_dcdc[module].RelativeHumidity.u8, &data[4], 4);
            break;
        }
        case 6:
        {
        	memcpy(iib_fac_2s_dcdc[module].GroundLeakage.u8, &data[0], 4);
            break;
        }
        case 7:
        {
        	memcpy(iib_fac_2s_dcdc[module].InterlocksRegister.u8, &data[0], 4);
        	memcpy(iib_fac_2s_dcdc[module].AlarmsRegister.u8, &data[4], 4);

        	if(iib_fac_2s_dcdc[module].InterlocksRegister.u32 > 0)
        	{
        		set_hard_interlock(0, IIB_Mod_1_Itlk + module);
        	}
        	else
        	{
        		iib_fac_2s_dcdc[module].InterlocksRegister.u32 = 0;
        	}
            break;
        }
        default:
        {
            break;
        }
    }
}
