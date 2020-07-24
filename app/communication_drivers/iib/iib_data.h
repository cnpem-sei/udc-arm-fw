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
    } iib_itlk[NUM_MAX_IIB_SIGNALS];

    union
    {
        volatile uint32_t   u32;
        volatile uint8_t    u8[4];
        volatile float      f;
    } iib_alarm[NUM_MAX_IIB_SIGNALS];

} iib_framwork_t;

typedef volatile struct {

    union {
        float       f;
        uint8_t     u8[4];
    } Iin;

    union {
        float       f;
        uint8_t     u8[4];
    } Vin;

    union {
        float       f;
        uint8_t     u8[4];
    } DriverVoltage;

    union {
        float       f;
        uint8_t     u8[4];
    } DriverCurrent;

    union {
        float       f;
        uint8_t     u8[4];
    } TempIGBT;

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
    } BoardTemperature;

    union {
        float       f;
        uint8_t     u8[4];
    } RelativeHumidity;

    union {
        float       f;
        uint8_t     u8[4];
        uint32_t    u32;
    } InterlocksRegister;

    union {
        float       f;
        uint8_t     u8[4];
        uint32_t    u32;
    } AlarmsRegister;

    uint8_t CanAddress;

} iib_fac_is_t;

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
    } DriverVoltage;

    union {
        float       f;
        uint8_t     u8[4];
    } Driver1Current;

    union {
        float       f;
        uint8_t     u8[4];
    } Driver2Current;

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
    } BoardTemperature;

    union {
        float       f;
        uint8_t     u8[4];
    } RelativeHumidity;

    union {
        float       f;
        uint8_t     u8[4];
        uint32_t    u32;
    } InterlocksRegister;

    union {
        float       f;
        uint8_t     u8[4];
        uint32_t    u32;
    } AlarmsRegister;

    uint8_t CanAddress;

} iib_fac_os_t;

typedef volatile struct {

    union {
        float       f;
        uint8_t     u8[4];
    } TempRectHeatSink;

    union {
        float       f;
        uint8_t     u8[4];
    } TempRectInductor;

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
    } ExternalBoardsVoltage;

    union {
        float       f;
        uint8_t     u8[4];
    } AuxiliaryBoardCurrent;

    union {
        float       f;
        uint8_t     u8[4];
    } IDBBoardCurrent;

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

    union {
        float       f;
        uint8_t     u8[4];
        uint32_t    u32;
    } InterlocksRegister;

    union {
        float       f;
        uint8_t     u8[4];
        uint32_t    u32;
    } AlarmsRegister;

    uint8_t CanAddress;

} iib_fac_cmd_t;

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
    } DriverVoltage;

    union {
        float       f;
        uint8_t     u8[4];
    } Driver1Current;

    union {
        float       f;
        uint8_t     u8[4];
    } Driver2Current;

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
    } GroundLeakage;

    union {
        float       f;
        uint8_t     u8[4];
    } BoardTemperature;

    union {
        float       f;
        uint8_t     u8[4];
    } RelativeHumidity;

    union {
        float       f;
        uint8_t     u8[4];
        uint32_t    u32;
    } InterlocksRegister;

    union {
        float       f;
        uint8_t     u8[4];
        uint32_t    u32;
    } AlarmsRegister;

    uint8_t CanAddress;

} iib_fap_module_t;

typedef union {
    float       f;
    uint8_t     u8[4];
    uint32_t    u32;
} convert_to_bytes_t;

#endif /* IIB_DATA_H_ */
