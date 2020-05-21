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
 * @file ipc.h
 * @brief Interprocessor Communication module
 *
 * This module is responsible for definition of interprocessor communication
 * functionalities, between ARM and C28 cores.
 *
 * @author gabriel.brunheira
 * @date 22/11/2017
 *
 */

#ifndef IPC_LIB_H_
#define IPC_LIB_H_

#include <stdint.h>
#include "board_drivers/version.h"
#include "communication_drivers/control/dsp.h"
#include "communication_drivers/control/siggen/siggen.h"
#include "communication_drivers/control/wfmref/wfmref.h"
#include "communication_drivers/common/structs.h"
#include "communication_drivers/common/timeslicer.h"
#include "communication_drivers/parameters/ps_parameters.h"
#include "communication_drivers/ps_modules/ps_modules.h"
#include "communication_drivers/scope/scope.h"


/**
 * Shared resources defines
 */

#define SIZE_BUF_SAMPLES_CTOM   4096
#define SIZE_BUF_SAMPLES_MTOC   4096

/**
 * IPC Message Defines
 */

#define IPC_MTOC_LOWPRIORITY_MSG    0x00000001  // IPC1
#define IPC_CTOM_LOWPRIORITY_MSG    0x00000001  // IPC1
#define SYNC_PULSE                  0x00000002  // IPC2
#define HARD_INTERLOCK              0x00000004  // IPC3
#define SOFT_INTERLOCK              0x00000008  // IPC4

#define MSG_ID_MTOC                 g_ipc_mtoc.msg_id
#define MSG_ID_CTOM                 g_ipc_ctom.msg_id

typedef enum
{
    Turn_On = 1,
    Turn_Off,
    Open_Loop,
    Close_Loop,
    Operating_Mode,
    Reset_Interlocks,
    Unlock_UDC,
    Lock_UDC,
    Cfg_Source_Scope,
    Cfg_Freq_Scope,
    Cfg_Duration_Scope,
    Enable_Scope,
    Disable_Scope,
    Reset_Scope,
    Set_SlowRef,
    Set_SlowRef_All_PS,
    Cfg_WfmRef,
    Update_WfmRef,
    Reset_WfmRef,
    Cfg_SigGen,
    Set_SigGen,
    Enable_SigGen,
    Disable_SigGen,
    Reset_Counters,
    Set_Param,
    Set_DSP_Coeffs,
    Cfg_TimeSlicer,
    Set_Command_Interface,
    CtoM_Message_Error
} ipc_mtoc_lowpriority_msg_t;

typedef enum
{
    Enable_HRADC_Boards,
    Disable_HRADC_Boards,
    MtoC_Message_Error
} ipc_ctom_lowpriority_msg_t;

#define GET_IPC_CTOM_LOWPRIORITY_MSG    (ipc_ctom_lowpriority_msg_t) (g_ipc_mtoc.msg_ctom >> 4 ) & 0x0000FFFF

typedef enum
{
    No_Error_CtoM,
    Error1,
    Error2,
    Error3,
    Error4
} error_ctom_t;

typedef enum
{
    No_Error_MtoC,
    Invalid_Argument,
    Invalid_OpMode,
    IPC_LowPriority_Full,
    HRADC_Config_Error
} error_mtoc_t;

/**
 * IPC structures definitions
 */
typedef volatile struct
{
    char            udc_c28_version[2*SIZE_VERSION]; // C28 char = 2 bytes
    uint32_t        msg_mtoc;
    uint16_t        msg_id;
    error_mtoc_t    error_mtoc;
    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
    } counter_set_slowref;
    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
    } counter_sync_pulse;
    ps_module_t     ps_module[NUM_MAX_PS_MODULES];
    siggen_t        siggen[NUM_MAX_PS_MODULES];
    wfmref_t        wfmref[NUM_MAX_PS_MODULES];
    scope_t         scope[NUM_MAX_SCOPES];
} ipc_ctom_t;

typedef volatile struct
{
    uint32_t                msg_ctom;
    uint16_t                msg_id;
    uint16_t                error_ctom;
    uint8_t                 ps_name[SIZE_PS_NAME];
    uint16_t                ps_model;
    uint16_t                num_ps_modules;
    ps_module_t             ps_module[NUM_MAX_PS_MODULES];
    siggen_t                siggen[NUM_MAX_PS_MODULES];
    wfmref_t                wfmref[NUM_MAX_PS_MODULES];
    scope_t                 scope[NUM_MAX_SCOPES];
    dsp_module_t            dsp_module;
    //param_control_t         control;
    //param_pwm_t             pwm;
    //param_hradc_t           hradc;
    //param_analog_vars_t     analog_vars;
    //param_communication_t   communication;
    //param_interlocks_t      interlocks;
} ipc_mtoc_t;

extern volatile u_float_t g_buf_samples_ctom[SIZE_BUF_SAMPLES_CTOM];

extern volatile ipc_ctom_t g_ipc_ctom;
extern volatile ipc_mtoc_t g_ipc_mtoc;

extern void init_ipc(void);
extern void send_ipc_msg(uint16_t msg_id, uint32_t flag);
extern void send_ipc_lowpriority_msg(uint16_t msg_id,
                                     ipc_mtoc_lowpriority_msg_t msg);
extern uint32_t low_priority_msg_to_reg(ipc_mtoc_lowpriority_msg_t msg);

extern uint32_t ipc_mtoc_translate (uint32_t ulShareAddress);
extern uint32_t ipc_ctom_translate (uint32_t ulShareAddress);
extern uint16_t ipc_mtoc_busy (uint32_t ulFlags);

extern void get_firmwares_version(void);

#endif /* IPC_LIB_H_ */
