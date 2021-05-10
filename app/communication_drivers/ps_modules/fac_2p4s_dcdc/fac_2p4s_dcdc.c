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
#include "communication_drivers/ps_modules/fac_2p4s_dcdc/fac_2p4s_dcdc.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/**
 * BSMP servers defines
 */
#define BSMP_SERVER_MOD_1_2     0
#define BSMP_SERVER_MOD_3_4     1
#define BSMP_SERVER_MOD_5_6     2
#define BSMP_SERVER_MOD_7_8     3

/**
 * Controller defines
 */
/// DSP Net Signals
#define I_LOAD_1                        g_controller_ctom.net_signals[0]  // HRADC0
#define I_LOAD_2                        g_controller_ctom.net_signals[1]  // HRADC1
#define I_ARM_1                         g_controller_ctom.net_signals[2]  // HRADC2
#define I_ARM_2                         g_controller_ctom.net_signals[3]  // HRADC3

#define I_LOAD_MEAN                     g_controller_ctom.net_signals[4]
#define I_LOAD_ERROR                    g_controller_ctom.net_signals[5]
#define DUTY_I_LOAD_PI                  g_controller_ctom.net_signals[6]

#define I_ARMS_DIFF                     g_controller_ctom.net_signals[7]
#define DUTY_DIFF                       g_controller_ctom.net_signals[8]

#define I_LOAD_DIFF                     g_controller_ctom.net_signals[9]

#define DUTY_REF_FF                     g_controller_ctom.net_signals[10]

#define V_CAPBANK_ARM_1_FILTERED        g_controller_ctom.net_signals[11]
#define V_CAPBANK_ARM_2_FILTERED        g_controller_ctom.net_signals[12]

#define IN_FF_V_CAPBANK_ARM_1           g_controller_ctom.net_signals[13]
#define IN_FF_V_CAPBANK_ARM_2           g_controller_ctom.net_signals[14]

#define WFMREF_IDX                      g_controller_ctom.net_signals[31]

#define DUTY_CYCLE_MOD_1    g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_MOD_2    g_controller_ctom.output_signals[1]
#define DUTY_CYCLE_MOD_3    g_controller_ctom.output_signals[2]
#define DUTY_CYCLE_MOD_4    g_controller_ctom.output_signals[3]
#define DUTY_CYCLE_MOD_5    g_controller_ctom.output_signals[4]
#define DUTY_CYCLE_MOD_6    g_controller_ctom.output_signals[5]
#define DUTY_CYCLE_MOD_7    g_controller_ctom.output_signals[6]
#define DUTY_CYCLE_MOD_8    g_controller_ctom.output_signals[7]

/// ARM Net Signals
#define V_CAPBANK_MOD_1     g_controller_mtoc.net_signals[0]
#define V_CAPBANK_MOD_2     g_controller_mtoc.net_signals[1]
#define V_CAPBANK_MOD_3     g_controller_mtoc.net_signals[2]
#define V_CAPBANK_MOD_4     g_controller_mtoc.net_signals[3]
#define V_CAPBANK_MOD_5     g_controller_mtoc.net_signals[4]
#define V_CAPBANK_MOD_6     g_controller_mtoc.net_signals[5]
#define V_CAPBANK_MOD_7     g_controller_mtoc.net_signals[6]
#define V_CAPBANK_MOD_8     g_controller_mtoc.net_signals[7]

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Module_1_CapBank_Overvoltage,
    Module_2_CapBank_Overvoltage,
    Module_3_CapBank_Overvoltage,
    Module_4_CapBank_Overvoltage,
    Module_5_CapBank_Overvoltage,
    Module_6_CapBank_Overvoltage,
    Module_7_CapBank_Overvoltage,
    Module_8_CapBank_Overvoltage,
    Module_1_CapBank_Undervoltage,
    Module_2_CapBank_Undervoltage,
    Module_3_CapBank_Undervoltage,
    Module_4_CapBank_Undervoltage,
    Module_5_CapBank_Undervoltage,
    Module_6_CapBank_Undervoltage,
    Module_7_CapBank_Undervoltage,
    Module_8_CapBank_Undervoltage,
    IIB_Mod_1_Itlk,
    IIB_Mod_2_Itlk,
    IIB_Mod_3_Itlk,
    IIB_Mod_4_Itlk,
    IIB_Mod_5_Itlk,
    IIB_Mod_6_Itlk,
    IIB_Mod_7_Itlk,
    IIB_Mod_8_Itlk
} hard_interlocks_t;

typedef enum
{
    DCCT_1_Fault,
    DCCT_2_Fault,
    DCCT_High_Difference,
    Load_Feedback_1_Fault,
    Load_Feedback_2_Fault,
    ARM_1_Overcurrent,
    ARM_2_Overcurrent,
    Arms_High_Difference,
    Complementary_PS_Itlk
} soft_interlocks_t;

volatile iib_fac_os_t iib_fac_2p4s_dcdc[8];

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
    // Module 1 DCLink Voltage: 10 V = 300 V
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 300.0/2048.0;
    g_analog_ch_0.Value = &(V_CAPBANK_MOD_1.f);

    // Module 2 DCLink Voltage: 10 V = 300 V
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 300.0/2048.0;
    g_analog_ch_1.Value = &(V_CAPBANK_MOD_2.f);

    // Module 3 DCLink Voltage: 10 V = 300 V
    g_analog_ch_3.Enable = 1;
    g_analog_ch_3.Gain = 300.0/2048.0;
    g_analog_ch_3.Value = &(V_CAPBANK_MOD_3.f);

    // Module 4 DCLink Voltage: 10 V = 300 V
    g_analog_ch_2.Enable = 1;
    g_analog_ch_2.Gain = 300.0/2048.0;
    g_analog_ch_2.Value = &(V_CAPBANK_MOD_4.f);

    // Module 5 DCLink Voltage: 10 V = 300 V
    g_analog_ch_4.Enable = 1;
    g_analog_ch_4.Gain = 300.0/2048.0;
    g_analog_ch_4.Value = &(V_CAPBANK_MOD_5.f);

    // Module 6 DCLink Voltage: 10 V = 300 V
    g_analog_ch_5.Enable = 1;
    g_analog_ch_5.Gain = 300.0/2048.0;
    g_analog_ch_5.Value = &(V_CAPBANK_MOD_6.f);

    // Module 7 DCLink Voltage: 10 V = 300 V
    g_analog_ch_6.Enable = 1;
    g_analog_ch_6.Gain = 300.0/2048.0;
    g_analog_ch_6.Value = &(V_CAPBANK_MOD_7.f);

    // Module 8 DCLink Voltage: 10 V = 300 V
    g_analog_ch_7.Enable = 1;
    g_analog_ch_7.Gain = 300.0/2048.0;
    g_analog_ch_7.Value = &(V_CAPBANK_MOD_8.f);
}

/**
* @brief Initialize BSMP servers.
*
* Setup BSMP servers for FBP operation.
*
*/
static void bsmp_init_server(void)
{
    uint8_t server, var_id;

    // First BSMP server already initialized
    bsmp_init(BSMP_SERVER_MOD_3_4);
    bsmp_init(BSMP_SERVER_MOD_5_6);
    bsmp_init(BSMP_SERVER_MOD_7_8);

    for(server = 0; server < 4; server++)
    {
        /// Mirror common variables from first server all others
        if(server > 0)
        {
            for(var_id = 0; var_id < 31; var_id++)
            {
                modify_bsmp_var(var_id, server,
                                bsmp[BSMP_SERVER_MOD_1_2].vars.list[var_id]->data);
            }
        }

        create_bsmp_var(31, server, 4, false, g_ipc_ctom.ps_module[0].ps_soft_interlock.u8);
        create_bsmp_var(32, server, 4, false, g_ipc_ctom.ps_module[0].ps_hard_interlock.u8);

        create_bsmp_var(33, server, 4, false, I_LOAD_MEAN.u8);
        create_bsmp_var(34, server, 4, false, I_LOAD_1.u8);
        create_bsmp_var(35, server, 4, false, I_LOAD_2.u8);

        create_bsmp_var(36, server, 4, false, I_ARM_1.u8);
        create_bsmp_var(37, server, 4, false, I_ARM_2.u8);

        create_bsmp_var(38, server, 4, false, V_CAPBANK_MOD_1.u8);
        create_bsmp_var(39, server, 4, false, V_CAPBANK_MOD_2.u8);
        create_bsmp_var(40, server, 4, false, V_CAPBANK_MOD_3.u8);
        create_bsmp_var(41, server, 4, false, V_CAPBANK_MOD_4.u8);
        create_bsmp_var(42, server, 4, false, V_CAPBANK_MOD_5.u8);
        create_bsmp_var(43, server, 4, false, V_CAPBANK_MOD_6.u8);
        create_bsmp_var(44, server, 4, false, V_CAPBANK_MOD_7.u8);
        create_bsmp_var(45, server, 4, false, V_CAPBANK_MOD_8.u8);

        create_bsmp_var(46, server, 4, false, DUTY_CYCLE_MOD_1.u8);
        create_bsmp_var(47, server, 4, false, DUTY_CYCLE_MOD_2.u8);
        create_bsmp_var(48, server, 4, false, DUTY_CYCLE_MOD_3.u8);
        create_bsmp_var(49, server, 4, false, DUTY_CYCLE_MOD_4.u8);
        create_bsmp_var(50, server, 4, false, DUTY_CYCLE_MOD_5.u8);
        create_bsmp_var(51, server, 4, false, DUTY_CYCLE_MOD_6.u8);
        create_bsmp_var(52, server, 4, false, DUTY_CYCLE_MOD_7.u8);
        create_bsmp_var(53, server, 4, false, DUTY_CYCLE_MOD_8.u8);

        create_bsmp_var(54, server, 4, false, iib_fac_2p4s_dcdc[server*2].VdcLink.u8);
        create_bsmp_var(55, server, 4, false, iib_fac_2p4s_dcdc[server*2].Iin.u8);
        create_bsmp_var(56, server, 4, false, iib_fac_2p4s_dcdc[server*2].Iout.u8);
        create_bsmp_var(57, server, 4, false, iib_fac_2p4s_dcdc[server*2].TempIGBT1.u8);
        create_bsmp_var(58, server, 4, false, iib_fac_2p4s_dcdc[server*2].TempIGBT2.u8);
        create_bsmp_var(59, server, 4, false, iib_fac_2p4s_dcdc[server*2].TempL.u8);
        create_bsmp_var(60, server, 4, false, iib_fac_2p4s_dcdc[server*2].TempHeatSink.u8);
        create_bsmp_var(61, server, 4, false, iib_fac_2p4s_dcdc[server*2].DriverVoltage.u8);
        create_bsmp_var(62, server, 4, false, iib_fac_2p4s_dcdc[server*2].Driver1Current.u8);
        create_bsmp_var(63, server, 4, false, iib_fac_2p4s_dcdc[server*2].Driver2Current.u8);
        create_bsmp_var(64, server, 4, false, iib_fac_2p4s_dcdc[server*2].BoardTemperature.u8);
        create_bsmp_var(65, server, 4, false, iib_fac_2p4s_dcdc[server*2].RelativeHumidity.u8);
        create_bsmp_var(66, server, 4, false, iib_fac_2p4s_dcdc[server*2].InterlocksRegister.u8);
        create_bsmp_var(67, server, 4, false, iib_fac_2p4s_dcdc[server*2].AlarmsRegister.u8);

        create_bsmp_var(68, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].VdcLink.u8);
        create_bsmp_var(69, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].Iin.u8);
        create_bsmp_var(70, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].Iout.u8);
        create_bsmp_var(71, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].TempIGBT1.u8);
        create_bsmp_var(72, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].TempIGBT2.u8);
        create_bsmp_var(73, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].TempL.u8);
        create_bsmp_var(74, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].TempHeatSink.u8);
        create_bsmp_var(75, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].DriverVoltage.u8);
        create_bsmp_var(76, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].Driver1Current.u8);
        create_bsmp_var(77, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].Driver2Current.u8);
        create_bsmp_var(78, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].BoardTemperature.u8);
        create_bsmp_var(79, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].RelativeHumidity.u8);
        create_bsmp_var(80, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].InterlocksRegister.u8);
        create_bsmp_var(81, server, 4, false, iib_fac_2p4s_dcdc[server*2+1].AlarmsRegister.u8);
    }
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fac_2p4s_dcdc_system_config()
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
    uint8_t i;

    for(i = 0; i < 8; i++)
    {
        iib_fac_2p4s_dcdc[i].CanAddress = i+1;
    }

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
    	case 30:
    	case 31:
    	case 32:
    	case 33:
    	case 34:
    	case 35:
    	case 36:
    	case 37:
    	{
    		module = 2;
    		id_var = (id - 30);
    		break;
    	}
    	case 40:
    	case 41:
    	case 42:
    	case 43:
    	case 44:
    	case 45:
    	case 46:
    	case 47:
    	{
    		module = 3;
    		id_var = (id - 40);
    		break;
    	}
    	case 50:
    	case 51:
    	case 52:
    	case 53:
    	case 54:
    	case 55:
    	case 56:
    	case 57:
    	{
    		module = 4;
    		id_var = (id - 50);
    		break;
    	}
    	case 60:
    	case 61:
    	case 62:
    	case 63:
    	case 64:
    	case 65:
    	case 66:
    	case 67:
    	{
    		module = 5;
    		id_var = (id - 60);
    		break;
    	}
    	case 70:
    	case 71:
    	case 72:
    	case 73:
    	case 74:
    	case 75:
    	case 76:
    	case 77:
    	{
    		module = 6;
    		id_var = (id - 70);
    		break;
    	}
    	case 80:
    	case 81:
    	case 82:
    	case 83:
    	case 84:
    	case 85:
    	case 86:
    	case 87:
    	{
    		module = 7;
    		id_var = (id - 80);
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
            memcpy(iib_fac_2p4s_dcdc[module].VdcLink.u8, &data[0], 4);
            memcpy(iib_fac_2p4s_dcdc[module].DriverVoltage.u8, &data[4], 4);
            break;
        }
        case 1:
        {
            memcpy(iib_fac_2p4s_dcdc[module].Iin.u8, &data[0], 4);
            memcpy(iib_fac_2p4s_dcdc[module].Iout.u8, &data[4], 4);
            break;
        }
        case 2:
        {
        	memcpy(iib_fac_2p4s_dcdc[module].Driver1Current.u8, &data[0], 4);
        	memcpy(iib_fac_2p4s_dcdc[module].Driver2Current.u8, &data[4], 4);
            break;
        }
        case 3:
        {
            memcpy(iib_fac_2p4s_dcdc[module].TempIGBT1.u8, &data[0], 4);
            memcpy(iib_fac_2p4s_dcdc[module].TempIGBT2.u8, &data[4], 4);
            break;
        }
        case 4:
        {
        	memcpy(iib_fac_2p4s_dcdc[module].TempL.u8, &data[0], 4);
        	memcpy(iib_fac_2p4s_dcdc[module].TempHeatSink.u8, &data[4], 4);
            break;
        }
        case 5:
        {
        	memcpy(iib_fac_2p4s_dcdc[module].BoardTemperature.u8, &data[0], 4);
        	memcpy(iib_fac_2p4s_dcdc[module].RelativeHumidity.u8, &data[4], 4);
            break;
        }
        case 6:
        {
        	memcpy(iib_fac_2p4s_dcdc[module].GroundLeakage.u8, &data[0], 4);
            break;
        }
        case 7:
        {
        	memcpy(iib_fac_2p4s_dcdc[module].InterlocksRegister.u8, &data[0], 4);
        	memcpy(iib_fac_2p4s_dcdc[module].AlarmsRegister.u8, &data[4], 4);

        	if(iib_fac_2p4s_dcdc[module].InterlocksRegister.u32 > 0)
        	{
        		set_hard_interlock(0, IIB_Mod_1_Itlk + module);
        	}
        	else
        	{
        		iib_fac_2p4s_dcdc[module].InterlocksRegister.u32 = 0;
        	}
            break;
        }
        default:
        {
            break;
        }
    }
}
