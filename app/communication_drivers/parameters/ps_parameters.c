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
#include "driverlib/sysctl.h"

#include "board_drivers/hardware_def.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/i2c_onboard/i2c_onboard.h"
#include "communication_drivers/i2c_offboard_isolated/i2c_offboard_isolated.h"
#include "communication_drivers/parameters/ps_parameters.h"

static const unsigned char * default_ps_name = "Initialization mode - invalid parameter bank                    ";
static const uint8_t default_ip[4] = {10, 0, 25, 43};
static const uint8_t default_ethernet_mask[4] = {255, 255, 255, 0};

static const uint16_t param_addresses_onboard_eeprom[NUM_MAX_PARAMETERS] =
{
    [PS_Name] = 0x0000,
    [PS_Model] = 0x0040,
    [Num_PS_Modules] = 0x0042,

    [RS485_Baudrate] = 0x0044,
    [RS485_Address] = 0x0048,
    [RS485_Termination] = 0x0050,
    [UDCNet_Address] = 0x0052,
    [Ethernet_IP] = 0x0054,
    [Ethernet_Subnet_Mask] = 0x0058,
    [Command_Interface] = 0x005C,
    [Buzzer_Volume] = 0x005E,

    [Freq_ISR_Controller] = 0x0080,
    [Freq_TimeSlicer] = 0x0084,
    [Control_Loop_State] = 0x0094,
    [Max_Ref] = 0x00A0,
    [Min_Ref] = 0x00B0,
    [Max_Ref_OpenLoop] = 0x00C0,
    [Min_Ref_OpenLoop] = 0x00D0,

    [PWM_Freq] = 0x00E0,
    [PWM_DeadTime] = 0x00E4,
    [PWM_Max_Duty] = 0x00E8,
    [PWM_Min_Duty] = 0x00EC,
    [PWM_Max_Duty_OpenLoop] = 0x00F0,
    [PWM_Min_Duty_OpenLoop] = 0x00F4,
    [PWM_Lim_Duty_Share] = 0x00F8,

    [HRADC_Num_Boards] = 0x0100,
    [HRADC_Freq_SPICLK] = 0x0102,
    [HRADC_Freq_Sampling] = 0x0104,
    [HRADC_Enable_Heater] = 0x0108,
    [HRADC_Enable_Monitor] = 0x0110,
    [HRADC_Type_Transducer] = 0x0118,
    [HRADC_Gain_Transducer] = 0x0120,
    [HRADC_Offset_Transducer] = 0x0130,

    [SigGen_Type] = 0x0140,
    [SigGen_Num_Cycles] = 0x0142,
    [SigGen_Freq] = 0x0144,
    [SigGen_Amplitude] = 0x0148,
    [SigGen_Offset] = 0x014C,
    [SigGen_Aux_Param] = 0x0150,

    [WfmRef_Selected] = 0x01C0,
    [WfmRef_SyncMode] = 0x01C8,
    [WfmRef_Frequency] = 0x1D0,
    [WfmRef_Gain] = 0x01E0,
    [WfmRef_Offset] = 0x1F0,

    [Analog_Var_Max] = 0x200,
    [Analog_Var_Min] = 0x300,

    [Hard_Interlocks_Debounce_Time] = 0x400,
    [Hard_Interlocks_Reset_Time] = 0x480,
    [Soft_Interlocks_Debounce_Time] = 0x500,
    [Soft_Interlocks_Reset_Time] = 0x580,

    [Scope_Sampling_Frequency] = 0x740,
    [Scope_Source] = 0x750,

    [Password] = 0x1FFD,
    [Enable_Onboard_EEPROM] = 0x1FFF
};

static const uint16_t param_addresses_offboard_eeprom[NUM_MAX_PARAMETERS] =
{
     [PS_Name] = 0x0000,
     [PS_Model] = 0x0040,
     [Num_PS_Modules] = 0x0042,

     [RS485_Baudrate] = 0x0044,
     [RS485_Address] = 0x0048,
     [RS485_Termination] = 0x0050,
     [UDCNet_Address] = 0x0052,
     [Ethernet_IP] = 0x0054,
     [Ethernet_Subnet_Mask] = 0x0058,
     [Command_Interface] = 0x005C,
     [Buzzer_Volume] = 0x005E,

     [Freq_ISR_Controller] = 0x0080,
     [Freq_TimeSlicer] = 0x0084,
     [Control_Loop_State] = 0x0094,
     [Max_Ref] = 0x00A0,
     [Min_Ref] = 0x00B0,
     [Max_Ref_OpenLoop] = 0x00C0,
     [Min_Ref_OpenLoop] = 0x00D0,

     [PWM_Freq] = 0x00E0,
     [PWM_DeadTime] = 0x00E4,
     [PWM_Max_Duty] = 0x00E8,
     [PWM_Min_Duty] = 0x00EC,
     [PWM_Max_Duty_OpenLoop] = 0x00F0,
     [PWM_Min_Duty_OpenLoop] = 0x00F4,
     [PWM_Lim_Duty_Share] = 0x00F8,

     [HRADC_Num_Boards] = 0x0100,
     [HRADC_Freq_SPICLK] = 0x0102,
     [HRADC_Freq_Sampling] = 0x0104,
     [HRADC_Enable_Heater] = 0x0108,
     [HRADC_Enable_Monitor] = 0x0110,
     [HRADC_Type_Transducer] = 0x0118,
     [HRADC_Gain_Transducer] = 0x0120,
     [HRADC_Offset_Transducer] = 0x0130,

     [SigGen_Type] = 0x0140,
     [SigGen_Num_Cycles] = 0x0142,
     [SigGen_Freq] = 0x0144,
     [SigGen_Amplitude] = 0x0148,
     [SigGen_Offset] = 0x014C,
     [SigGen_Aux_Param] = 0x0150,

     [WfmRef_Selected] = 0x01C0,
     [WfmRef_SyncMode] = 0x01C8,
     [WfmRef_Frequency] = 0x1D0,
     [WfmRef_Gain] = 0x01E0,
     [WfmRef_Offset] = 0x1F0,

     [Analog_Var_Max] = 0x200,
     [Analog_Var_Min] = 0x300,

     [Hard_Interlocks_Debounce_Time] = 0x400,
     [Hard_Interlocks_Reset_Time] = 0x480,
     [Soft_Interlocks_Debounce_Time] = 0x500,
     [Soft_Interlocks_Reset_Time] = 0x580,

     [Scope_Sampling_Frequency] = 0x740,
     [Scope_Source] = 0x750,
};

static uint8_t data_eeprom[32];

#pragma DATA_SECTION(g_param_bank,"SHARERAMS0_1");
volatile param_bank_t g_param_bank;

/**
 * Private functions
 */
static void init_param_bank_info(void)
{
    init_param(PS_Name, is_uint8_t, SIZE_PS_NAME, &PS_NAME);

    init_param(PS_Model, is_uint16_t, 1, (uint8_t *) &PS_MODEL);

    init_param(Num_PS_Modules, is_uint16_t, 1, (uint8_t *) &NUM_PS_MODULES);

    /**
     *  Communication parameters
     */
    init_param(Command_Interface, is_uint16_t, 1,
                   (uint8_t *) &COMMAND_INTERFACE);

    init_param(RS485_Baudrate, is_float, 1, &RS485_BAUDRATE.u8[0]);

    init_param(RS485_Address, is_uint16_t, NUM_MAX_PS_MODULES,
               &RS485_ADDRESS[0].u8[0]);

    init_param(RS485_Termination, is_uint16_t, 1, &RS485_TERMINATION.u8[0]);

    init_param(UDCNet_Address, is_uint16_t, 1, &UDCNET_ADDRESS.u8[0]);

    init_param(Ethernet_IP, is_uint8_t, 4, &ETHERNET_IP[0]);

    init_param(Ethernet_Subnet_Mask, is_uint8_t, 4, &ETHERNET_MASK[0]);

    init_param(Buzzer_Volume, is_uint16_t, 2, &BUZZER_VOLUME.u8[0]);

    /**
     * Controller parameters
     */
    init_param(Freq_ISR_Controller, is_float, 1, &ISR_CONTROL_FREQ.u8[0]);

    init_param(Freq_TimeSlicer, is_float, NUM_MAX_TIMESLICERS,
               &TIMESLICER_FREQ[0].u8[0]);

    init_param(Control_Loop_State, is_uint16_t, 1, &LOOP_STATE.u8[0]);

    init_param(Max_Ref, is_float, 4, &MAX_REF[0].u8[0]);

    init_param(Min_Ref, is_float, 4, &MIN_REF[0].u8[0]);

    init_param(Max_Ref_OpenLoop, is_float, 4, &MAX_REF_OL[0].u8[0]);

    init_param(Min_Ref_OpenLoop, is_float, 4, &MIN_REF_OL[0].u8[0]);

    /*init_param(Max_SlewRate_SlowRef, is_float, 1, &MAX_SLEWRATE_SLOWREF.u8[0]);

    init_param(Max_SlewRate_SigGen_Amp, is_float, 1,
               &MAX_SLEWRATE_SIGGEN_AMP.u8[0]);

    init_param(Max_SlewRate_SigGen_Offset, is_float, 1,
               &MAX_SLEWRATE_SIGGEN_OFFSET.u8[0]);

    init_param(Max_SlewRate_WfmRef, is_float, 1, &MAX_SLEWRATE_WFMREF.u8[0]);*/

    /**
     * PWM parameters
     */
    init_param(PWM_Freq, is_float, 1, &PWM_FREQ.u8[0]);

    init_param(PWM_DeadTime, is_float, 1, &PWM_DEAD_TIME.u8[0]);

    init_param(PWM_Max_Duty, is_float, 1, &PWM_MAX_DUTY.u8[0]);

    init_param(PWM_Min_Duty, is_float, 1, &PWM_MIN_DUTY.u8[0]);

    init_param(PWM_Max_Duty_OpenLoop, is_float, 1, &PWM_MAX_DUTY_OL.u8[0]);

    init_param(PWM_Min_Duty_OpenLoop, is_float, 1, &PWM_MIN_DUTY_OL.u8[0]);

    init_param(PWM_Lim_Duty_Share, is_float, 1, &PWM_LIM_DUTY_SHARE.u8[0]);

    /**
     * HRADC parameters
     */
    init_param(HRADC_Num_Boards, is_uint16_t, 1, &NUM_HRADC_BOARDS.u8[0]);

    init_param(HRADC_Freq_SPICLK, is_uint16_t, 1, &HRADC_SPI_CLK.u8[0]);

    init_param(HRADC_Freq_Sampling, is_float, 1, &HRADC_FREQ_SAMP.u8[0]);

    init_param(HRADC_Enable_Heater, is_uint16_t, NUM_MAX_HRADC,
               &HRADC_HEATER_ENABLE[0].u8[0]);

    init_param(HRADC_Enable_Monitor, is_uint16_t, NUM_MAX_HRADC,
               &HRADC_MONITOR_ENABLE[0].u8[0]);

    init_param(HRADC_Type_Transducer, is_uint16_t, NUM_MAX_HRADC,
               &TRANSDUCER_OUTPUT_TYPE[0].u8[0]);

    init_param(HRADC_Gain_Transducer, is_float, NUM_MAX_HRADC,
               &TRANSDUCER_GAIN[0].u8[0]);

    init_param(HRADC_Offset_Transducer, is_float, NUM_MAX_HRADC,
               &TRANSDUCER_OFFSET[0].u8[0]);


    /**
     * SigGen parameters
     */
    init_param(SigGen_Type, is_uint16_t, 1, &SIGGEN_TYPE_PARAM.u8[0]);

    init_param(SigGen_Num_Cycles, is_uint16_t, 1, &SIGGEN_NUM_CYCLES_PARAM.u8[0]);

    init_param(SigGen_Freq, is_float, 1, &SIGGEN_FREQ_PARAM.u8[0]);

    init_param(SigGen_Amplitude, is_float, 1, &SIGGEN_AMP_PARAM.u8[0]);

    init_param(SigGen_Offset, is_float, 1, &SIGGEN_OFFSET_PARAM.u8[0]);

    init_param(SigGen_Aux_Param, is_float, NUM_SIGGEN_AUX_PARAM,
                &SIGGEN_AUX_PARAM[0].u8[0]);


    /**
     * WfmRef parameters
     */
    init_param(WfmRef_Selected, is_uint16_t, 4, &WFMREF_SELECTED_PARAM[0].u8[0]);

    init_param(WfmRef_SyncMode, is_uint16_t, 4, &WFMREF_SYNC_MODE_PARAM[0].u8[0]);

    init_param(WfmRef_Frequency, is_float, 4, &WFMREF_FREQUENCY_PARAM[0].u8[0]);

    init_param(WfmRef_Gain, is_float, 4, &WFMREF_GAIN_PARAM[0].u8[0]);

    init_param(WfmRef_Offset, is_float, 4, &WFMREF_OFFSET_PARAM[0].u8[0]);


    /**
     * Analog variables parameters
     */
    init_param(Analog_Var_Max, is_float, NUM_MAX_ANALOG_VAR,
                &ANALOG_VARS_MAX[0].u8[0]);

    init_param(Analog_Var_Min, is_float, NUM_MAX_ANALOG_VAR,
                &ANALOG_VARS_MIN[0].u8[0]);

    /**
     * Interlocks parameters
     */
    init_param(Hard_Interlocks_Debounce_Time, is_uint32_t,
               NUM_MAX_HARD_INTERLOCKS,
               &HARD_INTERLOCKS_DEBOUNCE_TIME[0].u8[0]);

    init_param(Hard_Interlocks_Reset_Time, is_uint32_t, NUM_MAX_HARD_INTERLOCKS,
               &HARD_INTERLOCKS_RESET_TIME[0].u8[0]);

    init_param(Soft_Interlocks_Debounce_Time, is_uint32_t,
               NUM_MAX_SOFT_INTERLOCKS,
               &SOFT_INTERLOCKS_DEBOUNCE_TIME[0].u8[0]);

    init_param(Soft_Interlocks_Reset_Time, is_uint32_t, NUM_MAX_SOFT_INTERLOCKS,
               &SOFT_INTERLOCKS_RESET_TIME[0].u8[0]);

    /**
     * Scope parameters
     */
    init_param(Scope_Sampling_Frequency, is_float, NUM_MAX_SCOPES,
                &SCOPE_FREQ_SAMPLING_PARAM[0].u8[0]);

    init_param(Scope_Source, is_p_float, NUM_MAX_SCOPES,
               &SCOPE_SOURCE_PARAM[0].u8[0]);
}

static uint8_t save_param_onboard_eeprom(param_id_t id, uint16_t n)
{
    static uint8_t size_type;
    static u_uint16_t u_add;

    // Check wheter index is inside parameter range
    if(n < g_param_bank.param_info[id].num_elements)
    {
        size_type = g_param_bank.param_info[id].size_type;

        // Increment element position on parameter address and prepare for EEPROM
        u_add.u16 = param_addresses_onboard_eeprom[id] + size_type*n;
        data_eeprom[0] = u_add.u8[1];
        data_eeprom[1] = u_add.u8[0];

        // Prepare EEPROM data
        memcpy(&data_eeprom[2], (g_param_bank.param_info[id].p_val.u8 + size_type*n),
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

static uint8_t load_param_onboard_eeprom(param_id_t id, uint16_t n)
{
    static uint8_t size_type;
    static u_uint16_t u_add;

    // Check wheter index is inside parameter range
    if(n < g_param_bank.param_info[id].num_elements)
    {
        size_type = g_param_bank.param_info[id].size_type;

        // Increment element position on parameter address and prepare for EEPROM
        u_add.u16 = param_addresses_onboard_eeprom[id] + size_type*n;
        data_eeprom[0] = u_add.u8[1];
        data_eeprom[1] = u_add.u8[0];

        read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, size_type, data_eeprom);

        memcpy( (g_param_bank.param_info[id].p_val.u8 + size_type*n), &data_eeprom[0],
                size_type);

        return 1;
    }
    else
    {
        return 0;
    }
}

static uint8_t save_param_offboard_eeprom(param_id_t id, uint16_t n)
{
    static uint8_t size_type;
    static u_uint16_t u_add;

    // Check wheter index is inside parameter range
    if(n < g_param_bank.param_info[id].num_elements)
    {
        size_type = g_param_bank.param_info[id].size_type;

        // Increment element position on parameter address and prepare for EEPROM
        u_add.u16 = param_addresses_offboard_eeprom[id] + size_type*n;
        data_eeprom[0] = u_add.u8[1];
        data_eeprom[1] = u_add.u8[0];

        // Prepare EEPROM data
        memcpy(&data_eeprom[2], (g_param_bank.param_info[id].p_val.u8 + size_type*n),
               size_type);

        // Send new parameter to EEPROM
        write_i2c_offboard_isolated(I2C_SLV_ADDR_EEPROM, 2+size_type, data_eeprom);
        SysCtlDelay(375000);                // Wait 5 ms for EEPROM write cycle

        return 1;
    }
    else
    {
        return 0;
    }
}

static uint8_t load_param_offboard_eeprom(param_id_t id, uint16_t n)
{
    static uint8_t size_type;
    static u_uint16_t u_add;

    // Check wheter index is inside parameter range
    if(n < g_param_bank.param_info[id].num_elements)
    {
        size_type = g_param_bank.param_info[id].size_type;

        // Increment element position on parameter address and prepare for EEPROM
        u_add.u16 = param_addresses_offboard_eeprom[id] + size_type*n;
        data_eeprom[0] = u_add.u8[1];
        data_eeprom[1] = u_add.u8[0];

        read_i2c_offboard_isolated(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, size_type, data_eeprom);

        memcpy( (g_param_bank.param_info[id].p_val.u8 + size_type*n), &data_eeprom[0],
                size_type);

        return 1;
    }
    else
    {
        return 0;
    }
}

static void save_param_bank_onboard_eeprom(void)
{
    param_id_t id;
    uint16_t n;

    for(id = 0; id < NUM_PARAMETERS; id++)
    {
        for(n = 0; n < g_param_bank.param_info[id].num_elements; n++)
        {
            save_param_onboard_eeprom(id, n);
        }
    }
}

static void load_param_bank_onboard_eeprom(void)
{
    param_id_t id;
    uint16_t n;

    for(id = 0; id < NUM_PARAMETERS; id++)
    {
        for(n = 0; n < g_param_bank.param_info[id].num_elements; n++)
        {
            load_param_onboard_eeprom(id, n);
        }
    }
}

static void save_param_bank_offboard_eeprom(void)
{
    param_id_t id;
    uint16_t n;

    for(id = 0; id < NUM_PARAMETERS; id++)
    {
        for(n = 0; n < g_param_bank.param_info[id].num_elements; n++)
        {
            save_param_offboard_eeprom(id, n);
        }
    }
}

static void load_param_bank_offboard_eeprom(void)
{
    param_id_t id;
    uint16_t n;

    for(id = 0; id < NUM_PARAMETERS; id++)
    {
        for(n = 0; n < g_param_bank.param_info[id].num_elements; n++)
        {
            load_param_offboard_eeprom(id, n);
        }
    }
}

static void load_param_bank_default(void)
{
    static uint16_t n;

    for(n = 0; n < SIZE_PS_NAME; n++)
    {
        set_param(PS_Name, n, (float) default_ps_name[n]);
    }
    /*set_param(PS_Name, 0, 'A');
    set_param(PS_Name, 1, 'B');
    set_param(PS_Name, 2, 'C');*/

    set_param(PS_Model, 0, Uninitialized);
    set_param(Num_PS_Modules, 0, 1);

    set_param(Command_Interface, 0, 0);
    set_param(RS485_Baudrate, 0, 115200);
    set_param(RS485_Address, 0, 1);
    set_param(RS485_Address, 1, 30);
    set_param(RS485_Address, 2, 30);
    set_param(RS485_Address, 3, 30);
    set_param(RS485_Termination, 0, 1);

    for(n = 0; n < 4; n++)
    {
        set_param(Ethernet_IP, n, default_ip[n]);
        set_param(Ethernet_Subnet_Mask, n, default_ethernet_mask[n]);
    }

    set_param(Buzzer_Volume, 0, 1.0);
}

static uint8_t check_param_bank_offboard_eeprom(void)
{
    if(get_param(PS_Model,0) > 100)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

static uint8_t check_param_bank_onboard_eeprom(void)
{
    if(get_param(PS_Model,0) > 100)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/**
 * Public functions
 */
void init_param(param_id_t id, param_type_t type, uint16_t num_elements, uint8_t *p_param)
{
    uint8_t n;

    if(num_elements > 0)
    {
        g_param_bank.param_info[id].id = id;
        g_param_bank.param_info[id].type = type;
        g_param_bank.param_info[id].num_elements = num_elements;
        g_param_bank.param_info[id].eeprom_add.u16 = param_addresses_onboard_eeprom[id];
        g_param_bank.param_info[id].p_val.u8 = p_param;

        switch(g_param_bank.param_info[id].type)
        {
            case is_uint8_t:
            {
                g_param_bank.param_info[id].size_type = 1;
                for(n = 0; n < num_elements; n++)
                {
                    *(g_param_bank.param_info[id].p_val.u8 + n) = 0;
                }
                break;
            }

            case is_uint16_t:
            {
                g_param_bank.param_info[id].size_type = 2;
                for(n = 0; n < num_elements; n++)
                {
                    *(g_param_bank.param_info[id].p_val.u16 + n) = 0;
                }
                break;
            }

            case is_uint32_t:
            {
                g_param_bank.param_info[id].size_type = 4;
                for(n = 0; n < num_elements; n++)
                {
                    *(g_param_bank.param_info[id].p_val.u32 + n) = 0;
                }
                break;
            }

            case is_float:
            {
                g_param_bank.param_info[id].size_type = 4;
                for(n = 0; n < num_elements; n++)
                {
                    *(g_param_bank.param_info[id].p_val.f + n) = 0.0;
                }
                break;
            }

            case is_p_float:
            {
                g_param_bank.param_info[id].size_type = 4;
                for(n = 0; n < num_elements; n++)
                {
                    *(g_param_bank.param_info[id].p_val.p_f + n) = 0x0;
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
    if(n < g_param_bank.param_info[id].num_elements)
    {
        switch(g_param_bank.param_info[id].type)
        {
            case is_uint8_t:
            {
                *(g_param_bank.param_info[id].p_val.u8 + n) = (uint8_t) val;
                break;
            }

            case is_uint16_t:
            {
                *(g_param_bank.param_info[id].p_val.u16 + n) = (uint16_t) val;
                break;
            }

            case is_uint32_t:
            {
                *(g_param_bank.param_info[id].p_val.u32 + n) = (uint32_t) val;
                break;
            }

            case is_float:
            {
                *(g_param_bank.param_info[id].p_val.f + n) = val;
                break;
            }

            case is_p_float:
            {
                *(g_param_bank.param_info[id].p_val.p_f + n) = (uint32_t) val;
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
    if(n < g_param_bank.param_info[id].num_elements)
    {
        switch(g_param_bank.param_info[id].type)
        {
            case is_uint8_t:
            {
                return (float) *(g_param_bank.param_info[id].p_val.u8 + n);
            }

            case is_uint16_t:
            {
                return (float) *(g_param_bank.param_info[id].p_val.u16 + n);
            }

            case is_uint32_t:
            {
                return (float) *(g_param_bank.param_info[id].p_val.u32 + n);
            }

            case is_float:
            {
                return *(g_param_bank.param_info[id].p_val.f + n);
            }

            case is_p_float:
            {
                return (uint32_t) (*(g_param_bank.param_info[id].p_val.p_f + n));
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

uint8_t save_param_eeprom(param_id_t id, uint16_t n, param_memory_t type_memory)
{
    switch(type_memory)
    {
        case Offboard_EEPROM:
        {
            return save_param_offboard_eeprom(id, n);
        }

        case Onboard_EEPROM:
        {
            return save_param_onboard_eeprom(id, n);
        }

        default:
        {
            return 0;
        }
    }
}

uint8_t load_param_eeprom(param_id_t id, uint16_t n, param_memory_t type_memory)
{
    switch(type_memory)
    {
        case Offboard_EEPROM:
        {
            return load_param_offboard_eeprom(id, n);
        }

        case Onboard_EEPROM:
        {
            return load_param_onboard_eeprom(id, n);
        }

        default:
        {
            return 0;
        }
    }
}

void save_param_bank(param_memory_t type_memory)
{
    switch(type_memory)
    {
        case Offboard_EEPROM:
        {
            save_param_bank_offboard_eeprom();
            break;
        }

        case Onboard_EEPROM:
        {
            save_param_bank_onboard_eeprom();
            break;
        }

        default:
        {
            break;
        }
    }
}

void load_param_bank(param_memory_t type_memory)
{
    switch(type_memory)
    {
        case Offboard_EEPROM:
        {
            load_param_bank_offboard_eeprom();
            break;
        }

        case Onboard_EEPROM:
        {
            load_param_bank_onboard_eeprom();
            break;
        }

        case Default_Initialization:
        {
            load_param_bank_default();
            break;
        }

        default:
        {

            break;
        }
    }
}

uint8_t check_param_bank(param_memory_t type_memory)
{
    switch(type_memory)
    {
        case Offboard_EEPROM:
        {
            return check_param_bank_offboard_eeprom();
        }

        case Onboard_EEPROM:
        {
            return check_param_bank_onboard_eeprom();
        }

        default:
        {
            return 0;
        }
    }
}

void init_parameters_bank(void)
{
    init_param(Password, is_uint16_t, 1,
               (uint8_t *) &PASSWORD);

    load_param_onboard_eeprom(Password, 0);

    init_param(Enable_Onboard_EEPROM, is_uint8_t, 1,
               (uint8_t *) &ENABLE_ONBOARD_EEPROM);

    load_param_onboard_eeprom(Enable_Onboard_EEPROM, 0);

    init_param_bank_info();

    load_param_bank_offboard_eeprom();

    if(check_param_bank(Offboard_EEPROM))
    {
        //load_dsp_modules_eeprom(Offboard_EEPROM);
        return;
    }

    else
    {
        if(get_param(Enable_Onboard_EEPROM, 0) == 0)
        {
            load_param_bank_onboard_eeprom();
            if(check_param_bank(Onboard_EEPROM))
            {
                //load_dsp_modules_eeprom(Onboard_EEPROM);
                return;
            }
        }

        load_param_bank_default();
    }
}
