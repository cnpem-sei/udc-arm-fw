/******************************************************************************
 * Copyright (C) 2020 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file timeslicer.h
 * @brief Time Slicer Module
 * 
 * This module implements functions for time slicer functionality. It allows
 * the decimation of a periodic task within another periodic task.
 *
 * @author gabriel.brunheira
 * @date 06/04/2020
 *
 */

#ifndef TIMESLICER_H_
#define TIMESLICER_H_

#include <stdint.h>
#include "communication_drivers/common/structs.h"

#define RUN_TIMESLICER(timeslicer)
#define END_TIMESLICER(timeslicer)

#define RUN_TIMESLICER_NEW(timeslicer)  if(timeslicer.counter++ == timeslicer.freq_ratio){
#define END_TIMESLICER_NEW(timeslicer)  timeslicer.counter = 1;}

#define RESET_TIMESLICER(timeslicer)    timeslicer.counter = timeslicer.ratio

typedef volatile struct
{
    u_float_t   freq_base;
    u_float_t   freq_sampling;
    uint16_t    freq_ratio;
    uint16_t    counter;
} timeslicer_t;

extern void init_timeslicer(timeslicer_t *p_ts, float freq_base);
extern void cfg_timeslicer(timeslicer_t *p_ts, float freq_sampling);
extern void reset_timeslicer(timeslicer_t *p_ts);

#endif /* TIMESLICER_H_ */
