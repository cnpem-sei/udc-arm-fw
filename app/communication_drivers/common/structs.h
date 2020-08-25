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
    uint32_t    u32;
    float       f;
} u_float_t;

typedef union
{
    uint8_t     u8[4];
    uint32_t    u32;
    float       *p_f;
} u_p_float_t;

typedef union
{
    uint8_t     u8[4];
    uint32_t    u32;
    void        *p;
} u_pointer_t;

typedef enum
{
    Disabled,
    Idle,
    Buffering,
    Postmortem
} buf_status_t;

typedef struct
{
    buf_status_t    status;
    u_p_float_t     p_buf_start;
    u_p_float_t     p_buf_end;
    u_p_float_t     p_buf_idx;
} buf_t;

/**
 * Initialization for an instance of ```buf_t```. It requires a pre-defined
 * ```float``` array, addressed by ```p_buf_start```
 *
 * @param p_buf pointer to buffer structure
 * @param p_buf_start pointer to the first element of the pre-defined array
 * @param size number of elements on the array used by the buffer
 */
extern void init_buffer(buf_t *p_buf, volatile float *p_buf_start,
                        uint16_t size);

/**
 * Set values from buffer to 0 and reset index pointer
 *
 * @param p_buf pointer to buffer structure
 */
extern void reset_buffer(buf_t *p_buf);

/**
 * Enable specified buffer
 *
 * @param p_buf pointer to buffer structure
 */
extern void enable_buffer(buf_t *p_buf);
/**
 * Disable specified buffer
 *
 * @param p_buf pointer to buffer structure
 */
extern void disable_buffer(buf_t *p_buf);

/**
 * Return number of elements of specified buffer
 *
 * @param p_buf pointer to buffer structure
 * @return buffer size
 */
extern uint16_t size_buffer(buf_t *p_buf);

/**
 * Return position of current index pointer on specified buffer
 *
 * @param p_buf pointer to buffer structure
 * @return index position
 */
extern uint16_t idx_buffer(buf_t *p_buf);

/**
 * Insert new data to buffer. If buffer is enabled and full (```idx == end```),
 * it wraps around to the beginning, and keeps inserting new values. If buffer
 * is disabled, it stops to insert as soon as the buffer is filled.
 *
 * @param p_buf pointer to buffer structure
 * @param data new data to insert to buffer
 * @return indicate whether buffer is full
 */
extern uint16_t insert_buffer(buf_t *p_buf, float data);

/**
 * Test to indicate whether the buffer contains any sample outside the limits
 * determined by the
 *
 * @param p_buf pointer to buffer structure
 * @param value reference value for test
 * @param tol tolerance value for test
 * @return
 */
extern uint16_t test_buffer_limits(buf_t *p_buf, float value, float tol);

#endif /* STRUCTS_H_ */
