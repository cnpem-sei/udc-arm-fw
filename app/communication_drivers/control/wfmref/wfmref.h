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
#include "../../common/structs.h"

typedef enum
{
    //SampleBySample,
    SampleBySample_OneCycle,
    //OneShot
} sync_mode_t;

typedef volatile struct
{
    buf_t           wfmref_data;

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
} wfmref_t;


#endif /* WFMREF_H_ */
