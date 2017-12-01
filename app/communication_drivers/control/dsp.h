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
 * @file dsp.h
 * @brief Digital Signal Processing Module
 *
 * This module implements digital signal processing funcionalities, including
 * digital filters, PI controller, and other useful blocks for digital control
 * implementations. It replaces legacy ELP_DCL module.
 * 
 * Ref.: Figoli, David; "Implementing a Digital Power Supply with TMS320C28x
 * Digital Signal Controllers.pdf", Texas Instruments, 2005
 *
 * @author gabriel
 * @date 27/11/2017
 *
 * TODO: insert comments
 *
 */

#ifndef DSP_H_
#define DSP_H_

#include <stdint.h>

#define SATURATE(var, max, min)     if(var > max) var = max;    \
                                    if(var < min) var = min;

#define USE_MODULE              0
#define BYPASS_MODULE           1
#define NUM_MAX_MATRIX_SIZE     16

typedef enum
{
    DSP_Error,
    DSP_SRLim,
    DSP_LPF,
    DSP_PI,
    DSP_IIR_2P2Z,
    DSP_IIR_3P3Z,
    DSP_VdcLink_FeedForward,
    DSP_Matrix,
    DSP_Vect_Product
} dsp_class_t;

typedef volatile struct
{
    volatile float *pos;
    volatile float *neg;
    volatile float *error;
} dsp_error_t;

typedef volatile struct
{
    uint16_t bypass;
    float delta_max;
    volatile float *in;
    volatile float *out;
} dsp_srlim_t;

typedef volatile struct
{
    float k;
    float a;
    float in_old;
    volatile float *in;
    volatile float *out;
} dsp_lpf_t;

typedef volatile struct
{
    float kp;
    float ki;
    float freq_sampling;
    float u_max;
    float u_min;
    float u_prop;
    float u_int;
    volatile float *in;
    volatile float *out;
} dsp_pi_t;

typedef volatile struct
{
    float u_max;
    float u_min;
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float w1;
    float w2;
    volatile float *in;
    volatile float *out;
} dsp_iir_2p2z_t;

typedef volatile struct
{
    float u_max;
    float u_min;
    float b0;
    float b1;
    float b2;
    float b3;
    float a1;
    float a2;
    float a3;
    float w1;
    float w2;
    float w3;
    volatile float *in;
    volatile float *out;
} dsp_iir_3p3z_t;

typedef volatile struct
{
    float vdc_nom;
    float vdc_min;
    volatile float *vdc_meas;
    volatile float *in;
    volatile float *out;
} dsp_vdclink_ff_t;

typedef volatile struct
{
    uint16_t    num_rows;
    uint16_t    num_cols;
    float       data[NUM_MAX_MATRIX_SIZE][NUM_MAX_MATRIX_SIZE];
} dsp_matrix_t;

typedef volatile struct
{
    dsp_matrix_t    matrix;
    volatile float  *in;
    volatile float  *out;
} dsp_vect_product_t;

#endif /* DSP_H_ */
