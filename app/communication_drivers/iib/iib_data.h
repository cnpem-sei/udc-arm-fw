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

    float Iin;
    uint8_t IinAlarmSts;
    uint8_t IinItlkSts;
    float VdcLink;
    uint8_t VdcLinkAlarmSts;
    uint8_t VdcLinkItlkSts;
    uint8_t TempHeatsink;
    uint8_t TempHeatsinkAlarmSts;
    uint8_t TempHeatsinkItlkSts;
    uint8_t TempL;
    uint8_t TempLAlarmSts;
    uint8_t TempLItlkSts;
    uint8_t Driver1Error;
    uint8_t Driver1ErrorItlk;
    uint8_t Driver2Error;
    uint8_t Driver2ErrorItlk;
    uint8_t CanAddress;

} iib_input_stage_t;

typedef volatile struct {
    float Iin;
    uint8_t IinAlarmSts;
    uint8_t IinItlkSts;
    float Iout;
    uint8_t IoutAlarmSts;
    uint8_t IoutItlkSts;
    float VdcLink;
    uint8_t VdcLinkAlarmSts;
    uint8_t VdcLinkItlkSts;
    uint8_t TempIGBT1;
    uint8_t TempIGBT1AlarmSts;
    uint8_t TempIGBT1ItlkSts;
    uint8_t TempIGBT1HwrItlk;
    uint8_t TempIGBT1HwrItlkSts;
    uint8_t TempIGBT2;
    uint8_t TempIGBT2AlarmSts;
    uint8_t TempIGBT2ItlkSts;
    uint8_t TempIGBT2HwrItlk;
    uint8_t TempIGBT2HwrItlkSts;
    uint8_t TempL;
    uint8_t TempLAlarmSts;
    uint8_t TempLItlkSts;
    uint8_t TempHeatSink;
    uint8_t TempHeatSinkAlarmSts;
    uint8_t TempHeatSinkItlkSts;
    uint8_t Driver1Error;
    uint8_t Driver1ErrorItlk;
    uint8_t Driver2Error;
    uint8_t Driver2ErrorItlk;
    uint8_t CanAddress;

} iib_output_stage_t;

typedef volatile struct {

    uint8_t TempHeatSink;
    uint8_t TempHeatSinkAlarmSts;
    uint8_t TempHeatSinkItlkSts;
    uint8_t TempL;
    uint8_t TempLAlarmSts;
    uint8_t TempLItlkSts;
    float VcapBank;
    uint8_t VcapBankAlarmSts;
    uint8_t VcapBankItlkSts;
    float Vout;
    uint8_t VoutAlarmSts;
    uint8_t VoutItlkSts;
    uint8_t ExtItlkSts;
    uint8_t ExtItlk2Sts;
    uint8_t CanAddress;

} iib_command_drawer_t;

typedef volatile struct {

    float IoutRectf1;
    uint8_t IoutRectf1AlarmSts;
    uint8_t IoutRectf1ItlkSts;
    float IoutRectf2;
    uint8_t IoutRectf2AlarmSts;
    uint8_t IoutRectf2ItlkSts;
    float VoutRectf1;
    uint8_t VoutRectf1AlarmSts;
    uint8_t VoutRectf1ItlkSts;
    float VoutRectf2;
    uint8_t VoutRectf2AlarmSts;
    uint8_t VoutRectf2ItlkSts;
    float LeakageCurrent;
    uint8_t LeakageCurrentAlarmSts;
    uint8_t LeakageCurrentItlkSts;
    uint8_t TempHeatSink;
    uint8_t TempHeatSinkAlarmSts;
    uint8_t TempHeatSinkItlkSts;
    uint8_t TempWater;
    uint8_t TempWaterAlarmSts;
    uint8_t TempWaterItlkSts;
    uint8_t TempModule1;
    uint8_t TempModule1AlarmSts;
    uint8_t TempModule1ItlkSts;
    uint8_t TempModule2;
    uint8_t TempModule2AlarmSts;
    uint8_t TempModule2ItlkSts;
    uint8_t TempL1;
    uint8_t TempL1AlarmSts;
    uint8_t TempL1ItlkSts;
    uint8_t TempL2;
    uint8_t TempL2AlarmSts;
    uint8_t TempL2ItlkSts;
    uint8_t AcPhaseFault;
    uint8_t AcPhaseFaultSts;
    uint8_t AcOverCurrent;
    uint8_t AcOverCurrentSts;
    uint8_t AcTransformerOverTemp;
    uint8_t AcTransformerOverTempSts;
    uint8_t WaterFluxInterlock;
    uint8_t WaterFluxInterlockSts;
    uint8_t CanAddress;

} iib_rectifier_module_t;

typedef volatile struct {
    float Vin;
    uint8_t VinAlarmSts;
    uint8_t VinItlkSts;
    float Vout;
    uint8_t VoutAlarmSts;
    uint8_t VoutItlkSts;
    float IoutA1;
    uint8_t IoutA1AlarmSts;
    uint8_t IoutA1ItlkSts;
    float IoutA2;
    uint8_t IoutA2AlarmSts;
    uint8_t IoutA2ItlkSts;
    uint8_t TempIGBT1;
    uint8_t TempIGBT1AlarmSts;
    uint8_t TempIGBT1ItlkSts;
    uint8_t TempIGBT1HwrItlk;
    uint8_t TempIGBT1HwrItlkSts;
    uint8_t TempIGBT2;
    uint8_t TempIGBT2AlarmSts;
    uint8_t TempIGBT2ItlkSts;
    uint8_t TempIGBT2HwrItlk;
    uint8_t TempIGBT2HwrItlkSts;
    float DriveVoltage;
    float Drive1Current;
    float Drive2Current;
    uint8_t Driver1Error;
    uint8_t Driver1ErrorItlk;
    uint8_t Driver2Error;
    uint8_t Driver2ErrorItlk;
    uint8_t TempL;
    uint8_t TempLAlarmSts;
    uint8_t TempLItlkSts;
    uint8_t TempHeatSink;
    uint8_t TempHeatSinkAlarmSts;
    uint8_t TempHeatSinkItlkSts;
    uint8_t Relay;
    uint8_t ExternalItlk;
    uint8_t ExternalItlkSts;
    uint8_t LeakageCurrent;
    uint8_t LeakageCurrentSts;
    uint8_t Rack;
    uint8_t RackSts;
    uint8_t CanAddress;
} iib_fap_module_t;

typedef union {
    float       f;
    uint8_t     u8[4];
} float_to_bytes_t;

extern volatile iib_framwork_t g_iib_input_module;
extern volatile iib_framwork_t g_iib_output_module;
extern volatile iib_framwork_t g_iib_command_module;

extern void init_iib_framwork();

/**
 * TODO: Put here your defines. Just what need 
 * to be accessed by other modules.
 */


/**
 * TODO: Put here your functions prototypes. Just what need 
 * to be accessed by other modules.
 */


#endif /* IIB_DATA_H_ */
