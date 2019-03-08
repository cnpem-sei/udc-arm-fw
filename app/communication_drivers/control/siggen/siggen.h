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
 * @file siggen.h
 * @brief Signal generator module
 *
 * This module implements a real-time parametric digital signal generator. It
 * supports some broadly used signals, like sinusoidals, trapezoids, squares,
 * triangular, etc.
 *
 * @author gabriel.brunheira
 * @date 25/10/2017
 *
 */


#ifndef SIGGEN_H_
#define SIGGEN_H_

#include <stdint.h>

#define NUM_SIGGEN_AUX_PARAM    4
#define NUM_SIGGEN_AUX_VAR      8

/**
 * TODO: Implement square, triangular and prbs
 */
typedef enum
{
    Sine,
    DampedSine,
    Trapezoidal,
    DampedSquaredSine,
    Square
} siggen_type_t;

typedef volatile struct siggen_t siggen_t;

struct siggen_t
{
    union
    {
        uint8_t     u8[2];
        uint16_t    u16;
    } enable;

    union
    {
        uint8_t         u8[2];
        uint16_t        u16;
        siggen_type_t   enu;
    } type;

    union
    {
        uint8_t     u8[2];
        uint16_t    u16;
    } num_cycles;

    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
        float       f;
    } freq;

    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
        float       f;
    } amplitude;

    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
        float       f;
    } offset;

    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
        float       f;
    } n;

    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
        float       f;
    } num_samples;

    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
        float       f;
    } aux_param[NUM_SIGGEN_AUX_PARAM];

    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
        float       f;
    } aux_var[NUM_SIGGEN_AUX_VAR];

    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
        float       f;
    } freq_sampling;

    volatile float  *p_out;
    void            (*p_run_siggen)(siggen_t *p_siggen);
};

#endif
