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
 * @file iib_data.h
 * @brief Brief description of module
 * 
 * Detailed description
 *
 * @author allef.silva
 * @date 2 de out de 2018
 *
 */

#ifndef IIB_DATA_H_
#define IIB_DATA_H_

#include <stdint.h>

#define NUM_MAX_IIB_SIGNALS     32

typedef volatile struct
{

    union
    {
        volatile uint32_t   u32;
        volatile uint8_t    u8[4];
        volatile float      f;
    } iib_signals[NUM_MAX_IIB_SIGNALS];

    union
    {
        volatile uint32_t   u32;
        volatile uint8_t    u8[4];
        volatile float      f;
    } iib_itlk_lim[NUM_MAX_IIB_SIGNALS];

    union
    {
        volatile uint32_t   u32;
        volatile uint8_t    u8[4];
        volatile float      f;
    } iib_alm_lim[NUM_MAX_IIB_SIGNALS];

} iib_framwork_t;

typedef volatile struct {

    union {
        float       f;
        uint8_t     u8[4];
    } Iin;

    union {
        float       f;
        uint8_t     u8[4];
    } VdcLink;

    union {
        float       f;
        uint8_t     u8[4];
    } TempHeatsink;

    union {
        float       f;
        uint8_t     u8[4];
    } TempL;

    union {
        float       f;
        uint8_t     u8[4];
    } Driver1Error;

    union {
        float       f;
        uint8_t     u8[4];
    } Driver2Error;

    uint8_t CanAddress;

} iib_input_stage_t;

typedef volatile struct {

    union {
        float       f;
        uint8_t     u8[4];
    } Iin;

    union {
        float       f;
        uint8_t     u8[4];
    } Iout;

    union {
        float       f;
        uint8_t     u8[4];
    } VdcLink;

    union {
        float       f;
        uint8_t     u8[4];
    } TempIGBT1;

    union {
        float       f;
        uint8_t     u8[4];
    } TempIGBT2;

    union {
        float       f;
        uint8_t     u8[4];
    } TempL;

    union {
        float       f;
        uint8_t     u8[4];
    } TempHeatSink;

    union {
        float       f;
        uint8_t     u8[4];
    } Driver1Error;

    union {
        float       f;
        uint8_t     u8[4];
    } Driver2Error;

    uint8_t CanAddress;

} iib_output_stage_t;

typedef volatile struct {

    union {
        float       f;
        uint8_t     u8[4];
    } TempHeatSink;

    union {
        float       f;
        uint8_t     u8[4];
    } TempL;

    union {
        float       f;
        uint8_t     u8[4];
    } VcapBank;

    union {
        float       f;
        uint8_t     u8[4];
    } Vout;

    union {
        float       f;
        uint8_t     u8[4];
    } GroundLeakage;

    uint8_t CanAddress;

} iib_command_drawer_t;

typedef volatile struct {

    union {
        float       f;
        uint8_t     u8[4];
    } IoutRectf1;

    union {
        float       f;
        uint8_t     u8[4];
    } IoutRectf2;

    union {
        float       f;
        uint8_t     u8[4];
    } VoutRectf1;

    union {
        float       f;
        uint8_t     u8[4];
    } VoutRectf2;

    union {
        float       f;
        uint8_t     u8[4];
    } LeakageCurrent;

    union {
        float       f;
        uint8_t     u8[4];
    } TempHeatSink;

    union {
        float       f;
        uint8_t     u8[4];
    } TempWater;

    union {
        float       f;
        uint8_t     u8[4];
    } TempModule1;

    union {
        float       f;
        uint8_t     u8[4];
    } TempModule2;

    union {
        float       f;
        uint8_t     u8[4];
    } TempL1;

    union {
        float       f;
        uint8_t     u8[4];
    } TempL2;

    union {
        float       f;
        uint8_t     u8[4];
    } AcPhaseFault;

    union {
        float       f;
        uint8_t     u8[4];
    } AcOverCurrent;

    union {
        float       f;
        uint8_t     u8[4];
    } AcTransformerOverTemp;

    union {
        float       f;
        uint8_t     u8[4];
    } WaterFluxInterlock;

    uint8_t CanAddress;

} iib_rectifier_module_t;

typedef volatile struct {

    union {
        float       f;
        uint8_t     u8[4];
    } Vin;

    union {
        float       f;
        uint8_t     u8[4];
    } Vout;

    union {
        float       f;
        uint8_t     u8[4];
    } IoutA1;

    union {
        float       f;
        uint8_t     u8[4];
    } IoutA2;

    union {
        float       f;
        uint8_t     u8[4];
    } DriveVoltage;

    union {
        float       f;
        uint8_t     u8[4];
    } Drive1Current;

    union {
        float       f;
        uint8_t     u8[4];
    } Drive2Current;

    union {
        float       f;
        uint8_t     u8[4];
    } TempIGBT1;

    union {
        float       f;
        uint8_t     u8[4];
    } TempIGBT2;

    union {
        float       f;
        uint8_t     u8[4];
    } Driver1Error;

    union {
        float       f;
        uint8_t     u8[4];
    } Driver2Error;

    union {
        float       f;
        uint8_t     u8[4];
    } TempL;

    union {
        float       f;
        uint8_t     u8[4];
    } TempHeatSink;

    union {
        float       f;
        uint8_t     u8[4];
    } Relay;

    union {
        float       f;
        uint8_t     u8[4];
    } LeakageCurrent;

    union {
        float       f;
        uint8_t     u8[4];
    } Rack;

    union {
        float       f;
        uint8_t     u8[4];
    } ExternalItlk;

    union {
        float       f;
        uint8_t     u8[4];
    } GroundLeakage;

    union {
        float       f;
        uint8_t     u8[4];
    } BoardTemperature;

    union {
        float       f;
        uint8_t     u8[4];
    } RelativeHumidity;

    uint8_t CanAddress;

} iib_fap_module_t;

typedef union {
    float       f;
    uint8_t     u8[4];
    uint32_t    u32;
} float_to_bytes_t;

#endif /* IIB_DATA_H_ */
