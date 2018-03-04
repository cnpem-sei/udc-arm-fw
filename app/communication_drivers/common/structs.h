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
 * @file structs.h
 * @brief Common structs definitions.
 * 
 * Definition of structs and its functions, used on different applications.
 *
 * @author gabriel.brunheira
 * @date 25/10/2017
 *
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <stdint.h>


typedef enum
{
    Idle,
    Buffering,
    Postmortem
} buf_status_t;

typedef struct
{
    buf_status_t    status;
    volatile float  *p_buf_start;
    volatile float  *p_buf_end;
    volatile float  *p_buf_idx;
} buf_t;

typedef union
{
    uint8_t     u8[2];
    uint16_t    u16;
}  u_uint16_t;

typedef union
{
    uint8_t     u8[4];
    uint32_t    u32;
}  u_uint32_t;

typedef union
{
    uint8_t     u8[4];
    float       f;
}  u_float_t;

#endif /* STRUCTS_H_ */
