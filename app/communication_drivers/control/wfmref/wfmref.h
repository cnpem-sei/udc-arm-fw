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
 * @file wfmref.h
 * @brief Waveform references module
 * 
 * This module implements waveform references functionality.
 *
 * @author gabriel.brunheira
 * @date 22 de nov de 2017
 *
 */

#ifndef WFMREF_H_
#define WFMREF_H_

#include <stdint.h>
#include "communication_drivers/common/structs.h"

#define SIZE_WFMREF             4096
#define SIZE_WFMREF_FBP         SIZE_WFMREF/4
#define NUM_WFMREF_CURVES       2

#define WFMREF                  g_ipc_mtoc.wfmref

#define TIMESLICER_WFMREF       0

typedef enum
{
    SampleBySample,
    SampleBySample_OneCycle,
    OneShot
} sync_mode_t;

typedef union
{
    u_float_t data[NUM_WFMREF_CURVES][SIZE_WFMREF];
    u_float_t data_fbp[4][NUM_WFMREF_CURVES][SIZE_WFMREF_FBP];
} u_wfmref_data_t;

typedef volatile struct
{
    uint16_t        counter;
    uint16_t        max_count;
    float           freq_lerp;
    u_float_t       freq_base;
    float           inv_decimation;
    float           fraction;
    float           out;
} wfmref_lerp_t;

typedef volatile struct
{
    buf_t           wfmref_data[NUM_WFMREF_CURVES];

    union
    {
        uint8_t     u8[2];
        uint16_t    u16;
    } wfmref_selected;

    union
    {
        uint8_t     u8[2];
        uint16_t    u16;
        sync_mode_t enu;
    } sync_mode;

    wfmref_lerp_t   lerp;

    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
        float       f;
    } gain;

    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
        float       f;
    } offset;

    float *p_out;
} wfmref_t;

extern volatile u_wfmref_data_t g_wfmref_data;

extern void init_wfmref(wfmref_t *p_wfmref, uint16_t wfmref_selected,
                        sync_mode_t sync_mode, float freq_lerp, float freq_wfmref,
                        float gain, float offset, float *p_start, uint16_t size,
                        float *p_out);
#endif /* WFMREF_H_ */
