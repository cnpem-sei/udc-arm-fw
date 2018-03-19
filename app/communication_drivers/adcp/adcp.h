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
 * @file adcp.h
 * @brief ADCP module.
 *
 * @author joao.rosa
 *
 * @date 14/07/2015
 *
 */

#ifndef ADCP_H_
#define ADCP_H_

#include <stdint.h>
#include <stdarg.h>

typedef struct
{
    uint8_t Enable;
    float Gain;
    volatile float *Value;
}adcp_ch_t;

extern void adcp_init(void);

extern void adcp_read(void);
extern void adcp_get_samples(void);

//extern void ReadAdcP(adcpvar_t *ReadAd);
//extern void ClearAdcFilter(void);

extern adcp_ch_t g_analog_ch_0;
extern adcp_ch_t g_analog_ch_1;
extern adcp_ch_t g_analog_ch_2;
extern adcp_ch_t g_analog_ch_3;
extern adcp_ch_t g_analog_ch_4;
extern adcp_ch_t g_analog_ch_5;
extern adcp_ch_t g_analog_ch_6;
extern adcp_ch_t g_analog_ch_7;

#endif /* ADCP_H_ */
