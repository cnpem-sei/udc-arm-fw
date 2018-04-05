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
 * @file ps_parameters.c
 * @brief Power supply parameters bank module.
 * 
 * This module implements a data structure for initialization and configuration
 * of parameters for operation of the power supplies applications.
 *
 * @author gabriel.brunheira
 * @date 23/02/2018
 *
 */

#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"

#include "hardware_def.h"
#include "ps_parameters.h"

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/i2c_onboard/eeprom.h"
#include "communication_drivers/i2c_onboard/i2c_onboard.h"


static const uint16_t param_addresses[NUM_MAX_PARAMETERS] =
{
    [PS_Model] = 0x0040,
    [Num_PS_Modules] = 0x0042,

    [RS485_Baudrate] = 0x0044,
    [RS485_Address] = 0x0048,
    [RS485_Termination] = 0x0050,
    [UDCNet_Address] = 0x0052,
    [Ethernet_IP] = 0x0054,
    [Ethernet_Subnet_Mask] = 0x0058,
    [Command_Interface] = 0x005C,

    [Freq_ISR_Controller] = 0x0080,
    [Freq_TimeSlicer] = 0x0084,
    [Max_Ref] = 0x00A0,
    [Min_Ref] = 0x00A4,
    [Max_Ref_OpenLoop] = 0x00A8,
    [Min_Ref_OpenLoop] = 0x00AC,
    [Max_SlewRate_SlowRef] = 0x00B0,
    [Max_SlewRate_SigGen_Amp] = 0x00B4,
    [Max_SlewRate_SigGen_Offset] = 0x00B8,
    [Max_SlewRate_WfmRef] = 0x00BC,

    [PWM_Freq] = 0x00C0,
    [PWM_DeadTime] = 0x00C4,
    [PWM_Max_Duty] = 0x00C8,
    [PWM_Min_Duty] = 0x00CC,
    [PWM_Max_Duty_OpenLoop] = 0x00D0,
    [PWM_Min_Duty_OpenLoop] = 0x00D4,
    [PWM_Lim_Duty_Share] = 0x00D8,

    [HRADC_Num_Boards] = 0x00E0,
    [HRADC_Freq_SPICLK] = 0x00E2,
    [HRADC_Freq_Sampling] = 0x00E4,
    [HRADC_Enable_Heater] = 0x00E8,
    [HRADC_Enable_Monitor] = 0x00F0,
    [HRADC_Type_Transducer] = 0x00F8,
    [HRADC_Gain_Transducer] = 0x0100,
    [HRADC_Offset_Transducer] = 0x0110,

    [SigGen_Type] = 0x0120,
    [SigGen_Num_Cycles] = 0x0122,
    [SigGen_Freq] = 0x0124,
    [SigGen_Amplitude] = 0x0128,
    [SigGen_Offset] = 0x012C,
    [SigGen_Aux_Param] = 0x0130,

    [WfmRef_ID_WfmRef] = 0x0140,
    [WfmRef_SyncMode] = 0x0142,
    [WfmRef_Gain] = 0x0144,
    [WfmRef_Offset] = 0x148,

    [Analog_Var_Max] = 0x160,
    [Analog_Var_Min] = 0x260,
};

static uint8_t data_eeprom[32];
static unsigned long ulLoop;

//#pragma DATA_SECTION(ps_parameters_bank,"SHARERAMS0_1");
volatile param_t g_parameters[NUM_MAX_PARAMETERS];

void init_param(param_id_t id, param_type_t type, uint16_t num_elements, uint8_t *p_param)
{
    uint8_t n;

    if(num_elements > 0)
    {
        g_parameters[id].id = id;
        g_parameters[id].type = type;
        g_parameters[id].num_elements = num_elements;
        g_parameters[id].eeprom_add.u16 = param_addresses[id];
        g_parameters[id].p_val.u8 = p_param;

        switch(g_parameters[id].type)
        {
            case is_uint8_t:
            {
                g_parameters[id].size_type = 1;
                for(n = 0; n < num_elements; n++)
                {
                    *(g_parameters[id].p_val.u8 + n) = 0;
                }
                break;
            }

            case is_uint16_t:
            {
                g_parameters[id].size_type = 2;
                for(n = 0; n < num_elements; n++)
                {
                    *(g_parameters[id].p_val.u16 + n) = 0;
                }
                break;
            }

            case is_uint32_t:
            {
                g_parameters[id].size_type = 4;
                for(n = 0; n < num_elements; n++)
                {
                    *(g_parameters[id].p_val.u32 + n) = 0;
                }
                break;
            }

            case is_float:
            {
                g_parameters[id].size_type = 4;
                for(n = 0; n < num_elements; n++)
                {
                    *(g_parameters[id].p_val.f + n) = 0.0;
                }
                break;
            }

            default:
            {
                break;
            }
        }
    }
}

uint8_t set_param(param_id_t id, uint16_t n, float val)
{
    if(n < g_parameters[id].num_elements)
    {
        switch(g_parameters[id].type)
        {
            case is_uint8_t:
            {
                *(g_parameters[id].p_val.u8 + n) = (uint8_t) val;
                break;
            }

            case is_uint16_t:
            {
                *(g_parameters[id].p_val.u16 + n) = (uint16_t) val;
                break;
            }

            case is_uint32_t:
            {
                *(g_parameters[id].p_val.u32 + n) = (uint32_t) val;
                break;
            }

            case is_float:
            {
                *(g_parameters[id].p_val.f + n) = val;
                break;
            }

            default:
            {
                return 0;
            }
        }

        return 1;
    }
    else
    {
        return 0;
    }
}

float get_param(param_id_t id, uint16_t n)
{
    if(n < g_parameters[id].num_elements)
    {
        switch(g_parameters[id].type)
        {
            case is_uint8_t:
            {
                return (float) *(g_parameters[id].p_val.u8 + n);
            }

            case is_uint16_t:
            {
                return (float) *(g_parameters[id].p_val.u16 + n);
            }

            case is_uint32_t:
            {
                return (float) *(g_parameters[id].p_val.u32 + n);
            }

            case is_float:
            {
                return *(g_parameters[id].p_val.f + n);
            }

            default:
            {
                return NAN;
            }
        }
    }
    else
    {
        return NAN;
    }
}

uint8_t save_param_eeprom(param_id_t id, uint16_t n)
{
    static uint8_t size_type;
    static u_uint16_t u_add;

    // Check wheter index is inside parameter range
    if(n < g_parameters[id].num_elements)
    {
        size_type = g_parameters[id].size_type;

        // Increment element position on parameter address and prepare for EEPROM
        u_add.u16 = g_parameters[id].eeprom_add.u16 + size_type*n;
        data_eeprom[0] = u_add.u8[1];
        data_eeprom[1] = u_add.u8[0];

        // Prepare EEPROM data
        memcpy(&data_eeprom[2], (g_parameters[id].p_val.u8 + size_type*n),
               size_type);

        // Send new parameter to EEPROM
        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF);
        write_i2c(I2C_SLV_ADDR_EEPROM, 2+size_type, data_eeprom);
        SysCtlDelay(375000);                // Wait 5 ms for EEPROM write cycle
        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON);

        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t load_param_eeprom(param_id_t id, uint16_t n)
{
    static uint8_t size_type;
    static u_uint16_t u_add;

    // Check wheter index is inside parameter range
    if(n < g_parameters[id].num_elements)
    {
        size_type = g_parameters[id].size_type;

        // Increment element position on parameter address and prepare for EEPROM
        u_add.u16 = g_parameters[id].eeprom_add.u16 + size_type*n;
        data_eeprom[0] = u_add.u8[1];
        data_eeprom[1] = u_add.u8[0];

        read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, size_type, data_eeprom);

        memcpy( (g_parameters[id].p_val.u8 + size_type*n), &data_eeprom[0],
                size_type);

        return 1;
    }
    else
    {
        return 0;
    }
}

void init_parameters_bank(void)
{
    init_param(PS_Model, is_uint16_t, 1, (uint8_t *) &g_ipc_mtoc.ps_model);

    init_param(Num_PS_Modules, is_uint16_t, 1,
               (uint8_t *) &g_ipc_mtoc.num_ps_modules);

    /**
     *  Communication parameters
     */
    init_param(RS485_Baudrate, is_float, 1,
                &g_ipc_mtoc.communication.rs485_baud.u8[0]);

    init_param(RS485_Address, is_uint16_t, NUM_MAX_PS_MODULES,
                &g_ipc_mtoc.communication.rs485_address[0].u8[0]);

    init_param(RS485_Termination, is_uint16_t, 1,
                &g_ipc_mtoc.communication.rs485_termination.u8[0]);

    init_param(UDCNet_Address, is_uint16_t, 1,
                    &g_ipc_mtoc.communication.udcnet_address.u8[0]);

    init_param(Ethernet_IP, is_uint8_t, 4,
                &g_ipc_mtoc.communication.ethernet_ip[0]);

    init_param(Ethernet_Subnet_Mask, is_uint8_t, 4,
                &g_ipc_mtoc.communication.ethernet_mask[0]);

    init_param(Command_Interface, is_uint16_t, 1,
               (uint8_t *) &g_ipc_mtoc.communication.command_interface);

    /**
     * Controller parameters
     */
    init_param(Freq_ISR_Controller, is_float, 1,
                &g_ipc_mtoc.control.freq_isr_control.u8[0]);

    init_param(Freq_TimeSlicer, is_float, NUM_MAX_TIMESLICERS,
                &g_ipc_mtoc.control.freq_timeslicer[0].u8[0]);

    init_param(Max_Ref, is_float, 1, &g_ipc_mtoc.control.max_ref.u8[0]);

    init_param(Min_Ref, is_float, 1, &g_ipc_mtoc.control.min_ref.u8[0]);

    init_param(Max_Ref_OpenLoop, is_float, 1,
                &g_ipc_mtoc.control.max_ref_openloop.u8[0]);

    init_param(Min_Ref_OpenLoop, is_float, 1,
                &g_ipc_mtoc.control.min_ref_openloop.u8[0]);

    init_param(Max_SlewRate_SlowRef, is_float, 1,
                &g_ipc_mtoc.control.slewrate_slowref.u8[0]);

    init_param(Max_SlewRate_SigGen_Amp, is_float, 1,
                &g_ipc_mtoc.control.slewrate_siggen_amp.u8[0]);

    init_param(Max_SlewRate_SigGen_Offset, is_float, 1,
                &g_ipc_mtoc.control.slewrate_siggen_offset.u8[0]);

    init_param(Max_SlewRate_WfmRef, is_float, 1,
                &g_ipc_mtoc.control.slewrate_wfmref.u8[0]);

    /**
     * PWM parameters
     */
    init_param(PWM_Freq, is_float, 1, &g_ipc_mtoc.pwm.freq_pwm.u8[0]);

    init_param(PWM_DeadTime, is_float, 1, &g_ipc_mtoc.pwm.dead_time.u8[0]);

    init_param(PWM_Max_Duty, is_float, 1, &g_ipc_mtoc.pwm.max_duty.u8[0]);

    init_param(PWM_Min_Duty, is_float, 1, &g_ipc_mtoc.pwm.min_duty.u8[0]);

    init_param(PWM_Max_Duty_OpenLoop, is_float, 1,
                &g_ipc_mtoc.pwm.max_duty_openloop.u8[0]);

    init_param(PWM_Min_Duty_OpenLoop, is_float, 1,
                &g_ipc_mtoc.pwm.min_duty_openloop.u8[0]);

    init_param(PWM_Lim_Duty_Share, is_float, 1,
                &g_ipc_mtoc.pwm.lim_duty_share.u8[0]);

    /**
     * HRADC parameters
     */
    init_param(HRADC_Num_Boards, is_uint16_t, 1,
                &g_ipc_mtoc.hradc.num_hradc.u8[0]);

    init_param(HRADC_Freq_SPICLK, is_uint16_t, 1,
                &g_ipc_mtoc.hradc.freq_spiclk.u8[0]);

    init_param(HRADC_Freq_Sampling, is_float, 1,
                &g_ipc_mtoc.hradc.freq_hradc_sampling.u8[0]);

    init_param(HRADC_Enable_Heater, is_uint16_t, NUM_MAX_HRADC,
                &g_ipc_mtoc.hradc.enable_heater[0].u8[0]);

    init_param(HRADC_Enable_Monitor, is_uint16_t, NUM_MAX_HRADC,
                &g_ipc_mtoc.hradc.enable_monitor[0].u8[0]);

    init_param(HRADC_Type_Transducer, is_uint16_t, NUM_MAX_HRADC,
                &g_ipc_mtoc.hradc.type_transducer_output[0].u8[0]);

    init_param(HRADC_Gain_Transducer, is_float, NUM_MAX_HRADC,
                &g_ipc_mtoc.hradc.gain_transducer[0].u8[0]);

    init_param(HRADC_Offset_Transducer, is_float, NUM_MAX_HRADC,
                &g_ipc_mtoc.hradc.offset_transducer[0].u8[0]);


    /**
     * SigGen parameters
     */
    init_param(SigGen_Type, is_uint16_t, 1, &g_ipc_mtoc.siggen.type.u8[0]);

    init_param(SigGen_Num_Cycles, is_uint16_t, 1,
                &g_ipc_mtoc.siggen.num_cycles.u8[0]);

    init_param(SigGen_Freq, is_float, 1, &g_ipc_mtoc.siggen.freq.u8[0]);

    init_param(SigGen_Amplitude, is_float, 1,
                &g_ipc_mtoc.siggen.amplitude.u8[0]);

    init_param(SigGen_Offset, is_float, 1, &g_ipc_mtoc.siggen.offset.u8[0]);

    init_param(SigGen_Aux_Param, is_float, NUM_SIGGEN_AUX_PARAM,
                &g_ipc_mtoc.siggen.aux_param[0].u8[0]);


    /**
     * WfmRef parameters
     */
    init_param(WfmRef_ID_WfmRef, is_uint16_t, 1,
                &g_ipc_mtoc.wfmref.wfmref_selected.u8[0]);

    init_param(WfmRef_SyncMode, is_uint16_t, 1,
                &g_ipc_mtoc.wfmref.sync_mode.u8[0]);

    init_param(WfmRef_Gain, is_float, 1, &g_ipc_mtoc.wfmref.gain.u8[0]);

    init_param(WfmRef_Offset, is_float, 1, &g_ipc_mtoc.wfmref.offset.u8[0]);


    /**
     * Analog variables parameters
     */
    init_param(Analog_Var_Max, is_float, NUM_MAX_ANALOG_VAR,
                &g_ipc_mtoc.analog_vars.max[0].u8[0]);

    init_param(Analog_Var_Min, is_float, NUM_MAX_ANALOG_VAR,
                &g_ipc_mtoc.analog_vars.min[0].u8[0]);
}

void save_param_bank(void)
{
    param_id_t id;
    uint16_t n;

    for(id = 0; id < NUM_PARAMETERS; id++)
    {
        for(n = 0; n < g_parameters[id].num_elements; n++)
        {
            save_param_eeprom(id, n);
        }
    }
}

void load_param_bank(void)
{
    param_id_t id;
    uint16_t n;

    for(id = 0; id < NUM_PARAMETERS; id++)
    {
        for(n = 0; n < g_parameters[id].num_elements; n++)
        {
            load_param_eeprom(id, n);
        }
    }
}
