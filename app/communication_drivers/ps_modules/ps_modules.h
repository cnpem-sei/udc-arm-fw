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
 * @file ps_modules.h
 * @brief Power supplies modules.
 * 
 * Main source file for power supply modules. It includes macros and enumerates
 * related to operation of power supplies from ELP group on Sirius Project.
 *
 * @author gabriel.brunheira
 * @date 25/10/2017
 *
 */

#ifndef PS_MODULES_H_
#define PS_MODULES_H_

#include <stdint.h>
//#include <string.h>

/**
 * TODO: update macros for interlock check
 */
#define CHECK_SOFTINTERLOCK(itlk)   !(IPC_CtoM_Msg.PSModule.SoftInterlocks & itlk)
#define CHECK_INTERLOCK(itlk)       !(IPC_CtoM_Msg.PSModule.HardInterlocks & itlk)
#define CHECK_INTERLOCKS            !(IPC_CtoM_Msg.PSModule.HardInterlocks)

#define tCLOSED_LOOP        0
#define tOPEN_LOOP          1

#define INACTIVE            0
#define ACTIVE              1

#define LOCKED              0
#define UNLOCKED            1

#define NUM_MAX_PS_MODULES  4


typedef enum
{
    Off,
    Interlock,
    Initializing,
    SlowRef,
    SlowRefSync,
    Cycle,
    RmpWfm,
    MigWfm,
    FastRef
} ps_state_t;


typedef enum
{
    Remote,
    Local,
    PCHost
} ps_interface_t;

typedef enum
{
    Empty,
    FBP,
    FBP_DCLink,
    FAC_ACDC,
    FAC_DCDC,
    FAC_2S_ACDC,
    FAC_2S_DCDC,
    FAC_2P4S_ACDC,
    FAC_2P4S_DCDC,
    FAP,
    FAP_4P,
    FAC_DCDC_EMA,
    FAP_2P2S,
    FAP_IMAS,
    FAC_2P_ACDC_IMAS,
    FAC_2P_DCDC_IMAS,
    Uninitialized = 0x1F      // Empty EEPROM
} ps_model_t;

typedef struct
{
    ps_state_t      state      : 4;    // 3:0      Operation state
    uint16_t        openloop   : 1;    // 4        Control loop state
    ps_interface_t  interface  : 2;    // 6:5      Communication interface
    uint16_t        active     : 1;    // 7        Power supply active?
    ps_model_t      model      : 5;    // 12:8     Power supply model
    uint16_t        unlocked   : 1;    // 13       Unlocked?
    uint16_t        reserved   : 2;    // 15:14    Reserved for future use
} ps_status_bits_t;

typedef union
{
    uint8_t             u8[2];
    uint16_t            all;
    ps_status_bits_t    bit;
} ps_status_t;

typedef struct
{
    ps_status_t     ps_status;

    union {
        volatile uint8_t    u8[4];
        volatile uint32_t   u32;
        volatile float      f;
    } ps_setpoint;

    union {
        volatile uint32_t   u32;
        volatile uint8_t    u8[4];
        volatile float      f;
    } ps_reference;

    union {
        volatile uint32_t   u32;
        volatile uint8_t    u8[4];
    } ps_hard_interlock;

    union {
        volatile uint32_t   u32;
        volatile uint8_t    u8[4];
    } ps_soft_interlock;

    void            (*turn_on)(void);
    void            (*turn_off)(void);
    void            (*isr_soft_interlock)(void);
    void            (*isr_hard_interlock)(void);
    void            (*reset_interlocks)(void);
} ps_module_t;

#endif /* PS_MODULES_H_ */
