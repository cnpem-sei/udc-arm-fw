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
 * @file ps_parameters.h
 * @brief Power supply parameters bank module.
 * 
 * This module implements a data structure for initialization and configuration
 * of parameters for operation of the power supplies applications.
 *
 * @author gabriel.brunheira
 * @date 23/02/2018
 *
 */

#ifndef PS_PARAMETERS_H_
#define PS_PARAMETERS_H_

#include <stdint.h>
#include <math.h>
#include "communication_drivers/common/structs.h"
#include "communication_drivers/control/siggen/siggen.h"
#include "communication_drivers/ps_modules/ps_modules.h"
#include "communication_drivers/scope/scope.h"

#define SIZE_PS_NAME            64

#define NUM_MAX_TIMESLICERS     4

#define NUM_MAX_ANALOG_VAR      64
#define NUM_MAX_DIGITAL_VAR     12
#define NUM_MAX_HRADC           4

#define NUM_MAX_HARD_INTERLOCKS     32
#define NUM_MAX_SOFT_INTERLOCKS     32

#define NUM_PARAMETERS          54
#define NUM_MAX_PARAMETERS      64
#define NUM_MAX_FLOATS          200

typedef enum
{
    PS_Name,
    PS_Model,
    Num_PS_Modules,

    Command_Interface,
    RS485_Baudrate,
    RS485_Address,
    RS485_Termination,
    UDCNet_Address,
    Ethernet_IP,
    Ethernet_Subnet_Mask,
    Buzzer_Volume,

    Freq_ISR_Controller,
    Freq_TimeSlicer,
    Max_Ref,
    Min_Ref,
    Max_Ref_OpenLoop,
    Min_Ref_OpenLoop,
    Max_SlewRate_SlowRef,
    Max_SlewRate_SigGen_Amp,
    Max_SlewRate_SigGen_Offset,
    Max_SlewRate_WfmRef,

    PWM_Freq,
    PWM_DeadTime,
    PWM_Max_Duty,
    PWM_Min_Duty,
    PWM_Max_Duty_OpenLoop,
    PWM_Min_Duty_OpenLoop,
    PWM_Lim_Duty_Share,

    HRADC_Num_Boards,
    HRADC_Freq_SPICLK,
    HRADC_Freq_Sampling,
    HRADC_Enable_Heater,
    HRADC_Enable_Monitor,
    HRADC_Type_Transducer,
    HRADC_Gain_Transducer,
    HRADC_Offset_Transducer,

    SigGen_Type,
    SigGen_Num_Cycles,
    SigGen_Freq,
    SigGen_Amplitude,
    SigGen_Offset,
    SigGen_Aux_Param,

    WfmRef_ID_WfmRef,
    WfmRef_SyncMode,
    WfmRef_Gain,
    WfmRef_Offset,

    Analog_Var_Max,
    Analog_Var_Min,

    Hard_Interlocks_Debounce_Time,
    Hard_Interlocks_Reset_Time,
    Soft_Interlocks_Debounce_Time,
    Soft_Interlocks_Reset_Time,

    Scope_Sampling_Frequency,
    Scope_Source
} param_id_t;

typedef enum
{
    is_uint8_t,
    is_uint16_t,
    is_uint32_t,
    is_float,
    is_p_float,
} param_type_t;

typedef union
{
    uint8_t     *u8;
    uint16_t    *u16;
    uint32_t    *u32;
    float       *f;
    float       **p_f;
} p_param_t;

typedef struct
{
    param_id_t      id;
    param_type_t    type;
    uint8_t         size_type;
    uint16_t        num_elements;
    u_uint16_t      eeprom_add;
    p_param_t       p_val;
} param_t;

typedef struct
{
    u_float_t       rs485_baud;
    u_uint16_t      rs485_address[NUM_MAX_PS_MODULES];
    u_uint16_t      rs485_termination;
    u_uint16_t      udcnet_address;
    uint8_t         ethernet_ip[4];
    uint8_t         ethernet_mask[4];
    u_uint16_t      buzzer_volume;
    ps_interface_t  command_interface;
} param_communication_t;

typedef struct
{
    u_float_t   freq_isr_control;
    u_float_t   freq_timeslicer[NUM_MAX_TIMESLICERS];
    u_float_t   max_ref;
    u_float_t   min_ref;
    u_float_t   max_ref_openloop;
    u_float_t   min_ref_openloop;
    u_float_t   slewrate_slowref;
    u_float_t   slewrate_siggen_amp;
    u_float_t   slewrate_siggen_offset;
    u_float_t   slewrate_wfmref;
} param_control_t;

typedef struct
{
    u_float_t   freq_pwm;
    u_float_t   dead_time;
    u_float_t   max_duty;
    u_float_t   min_duty;
    u_float_t   max_duty_openloop;
    u_float_t   min_duty_openloop;
    u_float_t   lim_duty_share;
} param_pwm_t;

typedef struct
{
    u_uint16_t  num_hradc;
    u_uint16_t  freq_spiclk;
    u_float_t   freq_hradc_sampling;
    u_uint16_t  enable_heater[NUM_MAX_HRADC];
    u_uint16_t  enable_monitor[NUM_MAX_HRADC];
    u_uint16_t  type_transducer_output[NUM_MAX_HRADC];
    u_float_t   gain_transducer[NUM_MAX_HRADC];
    u_float_t   offset_transducer[NUM_MAX_HRADC];
} param_hradc_t;

typedef struct
{
    u_float_t   max[NUM_MAX_ANALOG_VAR];
    u_float_t   min[NUM_MAX_ANALOG_VAR];
} param_analog_vars_t;

typedef struct
{
    u_uint32_t  hard_itlks_debounce_time[NUM_MAX_HARD_INTERLOCKS];
    u_uint32_t  hard_itlks_reset_time[NUM_MAX_HARD_INTERLOCKS];
    u_uint32_t  soft_itlks_debounce_time[NUM_MAX_SOFT_INTERLOCKS];
    u_uint32_t  soft_itlks_reset_time[NUM_MAX_SOFT_INTERLOCKS];
} param_interlocks_t;

typedef struct
{
    u_uint16_t  type[NUM_MAX_PS_MODULES];
    u_uint16_t  num_cycles[NUM_MAX_PS_MODULES];
    u_float_t   freq[NUM_MAX_PS_MODULES];
    u_float_t   amplitude[NUM_MAX_PS_MODULES];
    u_float_t   offset[NUM_MAX_PS_MODULES];
    u_float_t   aux_param_0[NUM_MAX_PS_MODULES];
    u_float_t   aux_param_1[NUM_MAX_PS_MODULES];
    u_float_t   aux_param_2[NUM_MAX_PS_MODULES];
    u_float_t   aux_param_3[NUM_MAX_PS_MODULES];
} param_siggen_t;

typedef struct
{
    u_float_t   sync_mode[NUM_MAX_PS_MODULES];
    u_float_t   gain[NUM_MAX_PS_MODULES];
    u_float_t   offset[NUM_MAX_PS_MODULES];
} param_wfmref_t;


typedef struct
{
    u_float_t   freq_sampling[NUM_MAX_SCOPES];
    u_p_float_t p_source[NUM_MAX_SCOPES];
} param_scope_t;

typedef struct
{
    param_t                 param_info[NUM_MAX_PARAMETERS];
    uint8_t                 ps_name[SIZE_PS_NAME];
    uint16_t                ps_model;
    uint16_t                num_ps_modules;
    param_communication_t   communication;
    param_control_t         control;
    param_pwm_t             pwm;
    param_hradc_t           hradc;
    param_siggen_t          siggen;
    param_wfmref_t          wfmref;
    param_analog_vars_t     analog_vars;
    param_interlocks_t      interlocks;
    param_scope_t           scope;
} param_bank_t;

//extern volatile param_t g_parameters[NUM_MAX_PARAMETERS];

extern volatile param_bank_t g_param_bank;

extern void init_param(param_id_t id, param_type_t type, uint16_t num_elements, uint8_t *p_param);
extern uint8_t set_param(param_id_t id, uint16_t n, float val);
extern uint8_t save_param_eeprom(param_id_t id, uint16_t n);
extern float get_param(param_id_t id, uint16_t n);
extern uint8_t load_param_eeprom(param_id_t id, uint16_t n);

extern void init_parameters_bank(void);
extern void save_param_bank(void);
extern void load_param_bank(void);

#endif /* PS_PARAMETERS_H_ */


