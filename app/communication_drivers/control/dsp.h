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

#define NUM_MAX_MATRIX_SIZE     12
#define NUM_MAX_COEFFS_DSP      NUM_MAX_MATRIX_SIZE

#define NUM_DSP_CLASSES         8

#define NUM_COEFFS_DSP_SRLIM        1
#define NUM_COEFFS_DSP_LPF          1
#define NUM_COEFFS_DSP_PI           4
#define NUM_COEFFS_DSP_IIR_2P2Z     7
#define NUM_COEFFS_DSP_IIR_3P3Z     9
#define NUM_COEFFS_DSP_VDCLINK_FF   2
#define NUM_COEFFS_DSP_MATRIX       (2 + NUM_MAX_MATRIX_SIZE*NUM_MAX_MATRIX_SIZE)

typedef enum
{
    DSP_Error,
    DSP_SRLim,
    DSP_LPF,
    DSP_PI,
    DSP_IIR_2P2Z,
    DSP_IIR_3P3Z,
    DSP_VdcLink_FeedForward,
    DSP_Vect_Product
} dsp_class_t;

typedef volatile struct
{
    dsp_class_t dsp_class;
    uint16_t    id;
} dsp_module_t;

typedef volatile struct
{
    volatile float *pos;
    volatile float *neg;
    volatile float *error;
} dsp_error_t;

typedef volatile struct
{
    union
    {
        float f[NUM_COEFFS_DSP_SRLIM];
        struct
        {
            float max_slewrate;
        } s;
    } coeffs;

    uint16_t bypass;
    float freq_sampling;
    float delta_max;
    volatile float *in;
    volatile float *out;
} dsp_srlim_t;

typedef volatile struct
{
    union
    {
        float f[NUM_COEFFS_DSP_LPF];
        struct
        {
            float freq_cut;
        } s;
    } coeffs;

    float freq_sampling;
    float k;
    float a;
    float in_old;
    volatile float *in;
    volatile float *out;
} dsp_lpf_t;

typedef volatile struct
{
    union
    {
        float f[NUM_COEFFS_DSP_PI];
        struct
        {
            float kp;
            float ki;
            float u_max;
            float u_min;
        } s;
    } coeffs;

    float freq_sampling;
    float u_prop;
    float u_int;
    volatile float *in;
    volatile float *out;
} dsp_pi_t;

typedef volatile struct
{
    union
    {
        float f[NUM_COEFFS_DSP_IIR_2P2Z];
        struct
        {
            float b0;
            float b1;
            float b2;
            float a1;
            float a2;
            float u_max;
            float u_min;
        } s;
    } coeffs;

    float w1;
    float w2;
    volatile float *in;
    volatile float *out;
} dsp_iir_2p2z_t;

typedef volatile struct
{
    union
    {
        float f[NUM_COEFFS_DSP_IIR_3P3Z];
        struct
        {
            float b0;
            float b1;
            float b2;
            float b3;
            float a1;
            float a2;
            float a3;
            float u_max;
            float u_min;
        } s;
    } coeffs;

    float w1;
    float w2;
    float w3;
    volatile float *in;
    volatile float *out;
} dsp_iir_3p3z_t;

typedef volatile struct
{
    union
    {
        float f[NUM_COEFFS_DSP_VDCLINK_FF];
        struct
        {
            float vdc_nom;
            float vdc_min;
        } s;
    } coeffs;

    volatile float *vdc_meas;
    volatile float *in;
    volatile float *out;
} dsp_vdclink_ff_t;

typedef volatile struct
{
    union
    {
        float f[NUM_COEFFS_DSP_MATRIX];
        struct
        {
            float    num_rows;
            float    num_cols;
            float    data[NUM_MAX_MATRIX_SIZE][NUM_MAX_MATRIX_SIZE];
        } s;
    } coeffs;
} dsp_matrix_t;

typedef volatile struct
{
    dsp_matrix_t    matrix;
    volatile float  *in;
    volatile float  *out;
} dsp_vect_product_t;


extern void init_dsp_error(dsp_error_t *p_error, volatile float *pos,
                             volatile float *neg, volatile float *error);
extern void reset_dsp_error(dsp_error_t *p_error);
extern void run_dsp_error(dsp_error_t *p_error);


extern void init_dsp_srlim(dsp_srlim_t *p_srlim, float max_slewrate,
                             float freq_sampling, volatile float *in,
                             volatile float *out);
extern void cfg_dsp_srlim(dsp_srlim_t *p_srlim, float max_slewrate);
extern void bypass_dsp_srlim(dsp_srlim_t *p_srlim, uint16_t bypass);
extern void reset_dsp_srlim(dsp_srlim_t *p_srlim);
extern void run_dsp_srlim(dsp_srlim_t *p_srlim, uint16_t bypass);


extern void init_dsp_lpf(dsp_lpf_t *p_lpf, float freq_cut, float freq_sampling,
                         volatile float *in, volatile float *out);
extern void cfg_dsp_lpf(dsp_lpf_t *p_lpf, float freq_cut);
extern void reset_dsp_lpf(dsp_lpf_t *p_lpf);
extern void run_dsp_lpf(dsp_lpf_t *p_lpf);


extern void init_dsp_pi(dsp_pi_t *p_pi, float kp, float ki, float freq_sampling,
                        float u_max, float u_min, volatile float *in,
                        volatile float *out);
extern void cfg_dsp_pi(dsp_pi_t *p_pi, float kp, float ki, float u_max,
                       float u_min);
extern void reset_dsp_pi(dsp_pi_t *p_pi);
extern void run_dsp_pi(dsp_pi_t *p_pi);


extern void init_dsp_iir_2p2z(dsp_iir_2p2z_t *p_iir, float b0, float b1,
                              float b2, float a1, float a2, float u_max,
                              float u_min, volatile float *in,
                              volatile float *out);
extern void init_dsp_notch_2p2z(dsp_iir_2p2z_t *p_iir, float alpha,
                                float freq_cut, float freq_sampling,
                                float u_max, float u_min, volatile float *in,
                                volatile float *out);
extern void cfg_dsp_iir_2p2z(dsp_iir_2p2z_t *p_iir, float b0, float b1, float b2,
                             float a1, float a2, float u_max, float u_min);
extern void reset_dsp_iir_2p2z(dsp_iir_2p2z_t *p_iir);
extern void run_dsp_iir_2p2z(dsp_iir_2p2z_t *p_iir);


extern void init_dsp_iir_3p3z(dsp_iir_3p3z_t *p_iir, float b0, float b1,
                              float b2, float b3, float a1, float a2, float a3,
                              float u_max, float u_min, volatile float *in,
                              volatile float *out);
extern void cfg_dsp_iir_3p3z(dsp_iir_3p3z_t *p_iir, float b0, float b1, float b2,
                             float b3, float a1, float a2, float a3, float u_max,
                             float u_min);
extern void reset_dsp_iir_3p3z(dsp_iir_3p3z_t *p_iir);
extern void run_dsp_iir_3p3z(dsp_iir_3p3z_t *p_iir);


extern void init_dsp_vdclink_ff(dsp_vdclink_ff_t *p_ff, float vdc_nom,
                                float vdc_min, volatile float *vdc_meas,
                                volatile float *in, volatile float *out);
extern void cfg_dsp_vdclink_ff(dsp_vdclink_ff_t *p_ff, float vdc_nom,
                               float vdc_min);
extern void reset_dsp_vdclink_ff(dsp_vdclink_ff_t *p_ff);
extern void run_dsp_vdclink_ff(dsp_vdclink_ff_t *p_ff);


extern void init_dsp_vect_product(dsp_vect_product_t *p_vect_product,
                                  uint16_t num_rows, uint16_t num_cols,
                                  volatile float matrix[num_rows][num_cols],
                                  volatile float *in, volatile float *out);
extern void reset_dsp_vect_product(dsp_vect_product_t *p_vect_product);
extern void run_dsp_vect_product(dsp_vect_product_t *p_vect_product);

#endif /* DSP_H_ */
