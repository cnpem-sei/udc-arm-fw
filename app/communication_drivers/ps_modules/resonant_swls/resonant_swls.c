/******************************************************************************
 * Copyright (C) 2022 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file resonant_swls.h
 * @brief Resonant converter module for SWLS
 *
 * Module for control of resonant convert power supply designed for the
 * superconducting Wavelength Shifter. It implements the controller for load
 * current using frequency modulation strategy, instead of pulse-width
 * modulation (PWM).
 *
 * @author gabriel.brunheira
 * @date 18/07/2022
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

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
#include "communication_drivers/ps_modules/resonant_swls/resonant_swls.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/**
 * Controller defines
 */

/// DSP Net Signals
#define I_LOAD_1                g_controller_ctom.net_signals[0]  // HRADC0
#define I_LOAD_2                g_controller_ctom.net_signals[1]  // HRADC1
#define V_DCLINK                g_controller_ctom.net_signals[2]  // HRADC2

#define I_LOAD_MEAN             g_controller_ctom.net_signals[2]
#define I_LOAD_ERROR            g_controller_ctom.net_signals[3]
#define I_LOAD_DIFF             g_controller_ctom.net_signals[4]

#define FREQ_MODULATED          g_controller_ctom.net_signals[5]

#define FREQ_MODULATED_COMPENS  g_controller_ctom.output_signals[0]

/**
 * Interlocks defines
 */
typedef enum
{
	Load_Overcurrent,
	DCLink_Overvoltage,
	DCLink_Undervoltage,
	Welded_Contactor_K1_Fault,
	Welded_Contactor_K2_Fault,
	Opened_Contactor_K1_Fault,
	Opened_Contactor_K2_Fault,
	External_Itlk,
	IIB_Itlk
} hard_interlocks_t;

typedef enum
{
    DCCT_1_Fault,
    DCCT_2_Fault,
    DCCT_High_Difference
} soft_interlocks_t;

typedef enum
{
    High_Sync_Input_Frequency = 0x00000001
} alarms_t;

static volatile iib_resonant_swls_module_t iib_resonant_swls;

static void init_iib();

static void handle_can_data(volatile uint8_t *data, volatile unsigned long id);

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAP operation.
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
    create_bsmp_var(33, 0, 4, false, g_ipc_ctom.ps_module[0].ps_alarms.u8);

    create_bsmp_var(34, 0, 4, false, I_LOAD_MEAN.u8);
    create_bsmp_var(35, 0, 4, false, I_LOAD_1.u8);
    create_bsmp_var(36, 0, 4, false, I_LOAD_2.u8);
    create_bsmp_var(37, 0, 4, false, I_LOAD_ERROR.u8);
    create_bsmp_var(38, 0, 4, false, V_DCLINK.u8);
    create_bsmp_var(39, 0, 4, false, FREQ_MODULATED.u8);
    create_bsmp_var(40, 0, 4, false, FREQ_MODULATED_COMPENS.u8);

    create_bsmp_var(41, 0, 4, false, iib_resonant_swls.Vin.u8);
    create_bsmp_var(42, 0, 4, false, iib_resonant_swls.Vout.u8);
    create_bsmp_var(43, 0, 4, false, iib_resonant_swls.Iin.u8);
    create_bsmp_var(44, 0, 4, false, iib_resonant_swls.Iout.u8);
    create_bsmp_var(45, 0, 4, false, iib_resonant_swls.TempHeatSinkTransformerPfc.u8);
    create_bsmp_var(46, 0, 4, false, iib_resonant_swls.TempOutputInductor.u8);
    create_bsmp_var(47, 0, 4, false, iib_resonant_swls.TempHeatSinkDiodes.u8);
    create_bsmp_var(48, 0, 4, false, iib_resonant_swls.TempHeatSinkClamp.u8);
    create_bsmp_var(49, 0, 4, false, iib_resonant_swls.DriverAuxVoltage.u8);
    create_bsmp_var(50, 0, 4, false, iib_resonant_swls.Driver1Current.u8);
    create_bsmp_var(51, 0, 4, false, iib_resonant_swls.AuxCurrent.u8);
    create_bsmp_var(52, 0, 4, false, iib_resonant_swls.GroundLeakage.u8);
    create_bsmp_var(53, 0, 4, false, iib_resonant_swls.BoardTemperature.u8);
    create_bsmp_var(54, 0, 4, false, iib_resonant_swls.RelativeHumidity.u8);
    create_bsmp_var(55, 0, 4, false, iib_resonant_swls.InterlocksRegister.u8);
    create_bsmp_var(56, 0, 4, false, iib_resonant_swls.AlarmsRegister.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void resonant_swls_system_config()
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
    iib_resonant_swls.CanAddress = 1;

    init_iib_module_can_data(&g_iib_module_can_data, &handle_can_data);
}

static void handle_can_data(volatile uint8_t *data, volatile unsigned long id)
{
    switch(id)
    {
        case 10:
        {
            memcpy((void *)iib_resonant_swls.Vin.u8, (const void *)&data[0], (size_t)4);
            memcpy((void *)iib_resonant_swls.Vout.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 11:
        {
            memcpy((void *)iib_resonant_swls.Iin.u8, (const void *)&data[0], (size_t)4);
            memcpy((void *)iib_resonant_swls.Iout.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 12:
        {
        	memcpy((void *)iib_resonant_swls.TempHeatSinkTransformerPfc.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_resonant_swls.TempOutputInductor.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 13:
        {
        	memcpy((void *)iib_resonant_swls.TempHeatSinkDiodes.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_resonant_swls.TempHeatSinkClamp.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 14:
        {
            memcpy((void *)iib_resonant_swls.DriverAuxVoltage.u8, (const void *)&data[0], (size_t)4);
            memcpy((void *)iib_resonant_swls.GroundLeakage.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 15:
        {
        	memcpy((void *)iib_resonant_swls.Driver1Current.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_resonant_swls.AuxCurrent.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 16:
        {
        	memcpy((void *)iib_resonant_swls.BoardTemperature.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_resonant_swls.RelativeHumidity.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 17:
        {
        	memcpy((void *)iib_resonant_swls.InterlocksRegister.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_resonant_swls.AlarmsRegister.u8, (const void *)&data[4], (size_t)4);

            if(iib_resonant_swls.InterlocksRegister.u32 > 0)
        	{
        		set_hard_interlock(0, IIB_Itlk);
        	}

            else
        	{
        		iib_resonant_swls.InterlocksRegister.u32 = 0;
        	}

        	break;
        }
        default:
        {
            break;
        }
    }
}
