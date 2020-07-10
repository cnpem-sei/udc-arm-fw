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
 * @file eeprom.c
 * @brief EEPROM module.
 *
 * @author joao.rosa
 *
 * @date 15/07/2015
 *
 */

#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"

#include "i2c_onboard.h"
#include "hardware_def.h"
#include "eeprom.h"

#include "communication_drivers/i2c_offboard_isolated/i2c_offboard_isolated.h"

#include "communication_drivers/common/structs.h"
#include "communication_drivers/control/dsp.h"

//***********************************************************************************
//  The memory address is compose of 13bits (2 bytes)
//  Byte MSB with 5 bits [---1 1111] and the LSB with 8 bits [1111 1111]
//  The 8 MSB bits (5 in the MSB byte + 3 in the LSB byte) correspond to the page address and the lasts 5 LSB bits are used to point one of 32bytes available in a memory page.
//
//  Memory page [8 bits], Byte in the page [5 bits]
//
//***********************************************************************************

const uint16_t dsp_modules_eeprom_add[NUM_DSP_CLASSES] =
{
    [DSP_Error]         = 0x1FE0,
    [DSP_SRLim]         = 0x0C00,
    [DSP_LPF]           = 0x0C20,
    [DSP_PI]            = 0x0C40,
    [DSP_IIR_2P2Z]      = 0x0CE0,
    [DSP_IIR_3P3Z]      = 0x0DE0,
    [DSP_VdcLink_FeedForward] = 0x0EE0,
    [DSP_Vect_Product]  = 0x1FE0,
};

const uint16_t num_coeffs_dsp_module[NUM_DSP_CLASSES] =
{
    [DSP_Error]         = 0,
    [DSP_SRLim]         = NUM_COEFFS_DSP_SRLIM,
    [DSP_LPF]           = NUM_COEFFS_DSP_LPF,
    [DSP_PI]            = NUM_COEFFS_DSP_PI,
    [DSP_IIR_2P2Z]      = NUM_COEFFS_DSP_IIR_2P2Z,
    [DSP_IIR_3P3Z]      = NUM_COEFFS_DSP_IIR_3P3Z,
    [DSP_VdcLink_FeedForward] = NUM_COEFFS_DSP_VDCLINK_FF,
    [DSP_Vect_Product]  = NUM_COEFFS_DSP_MATRIX
};

const uint16_t num_dsp_modules[NUM_DSP_CLASSES] =
{
    [DSP_Error]         = NUM_MAX_DSP_ERROR,
    [DSP_SRLim]         = NUM_MAX_DSP_SRLIM,
    [DSP_LPF]           = NUM_MAX_DSP_LPF,
    [DSP_PI]            = NUM_MAX_DSP_PI,
    [DSP_IIR_2P2Z]      = NUM_MAX_DSP_IIR_2P2Z,
    [DSP_IIR_3P3Z]      = NUM_MAX_DSP_IIR_3P3Z,
    [DSP_VdcLink_FeedForward] = NUM_MAX_DSP_VDCLINK_FF,
    [DSP_Vect_Product]  = NUM_MAX_DSP_VECT_PRODUCT
};

static uint8_t data_eeprom[64];
volatile unsigned long ulLoop;

//***********************************************************************************

uint8_t save_dsp_coeffs_onboard_eeprom(dsp_class_t dsp_class, uint16_t id)
{
    static u_uint16_t u_add;
    static uint8_t *p_val, size_coeffs;

    size_coeffs = 4*num_coeffs_dsp_module[dsp_class];

    // Increment element position on parameter address and prepare for EEPROM
    u_add.u16 = dsp_modules_eeprom_add[dsp_class] + id*size_coeffs;
    data_eeprom[0] = u_add.u8[1];
    data_eeprom[1] = u_add.u8[0];

    // Perform typecast of pointer to coefficients avoid local copy of them
    switch(dsp_class)
    {
        case DSP_SRLim:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_srlim[id].coeffs.f;
            break;
        }

        case DSP_LPF:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_lpf[id].coeffs.f;
            break;
        }

        case DSP_PI:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_pi[id].coeffs.f;
            break;
        }
        case DSP_IIR_2P2Z:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_iir_2p2z[id].coeffs.f;
            break;
        }

        case DSP_IIR_3P3Z:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f;
            break;
        }

        case DSP_VdcLink_FeedForward:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_ff[id].coeffs.f;
            break;
        }

        default:
        {
            return 0;
        }
    }

    // Prepare EEPROM data
    memcpy(&data_eeprom[2], p_val, size_coeffs);

    if( size_coeffs > 32 )
    {
        // Send new parameter to EEPROM
        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF);
        write_i2c(I2C_SLV_ADDR_EEPROM, 2+32, data_eeprom);
        for (ulLoop=0;ulLoop<100000;ulLoop++){};
        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON);

        u_add.u16 += 32;
        data_eeprom[0] = u_add.u8[1];
        data_eeprom[1] = u_add.u8[0];
        memcpy(&data_eeprom[2], &data_eeprom[34], size_coeffs-32);

        // Send new parameter to EEPROM
        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF);
        write_i2c(I2C_SLV_ADDR_EEPROM, 2+(size_coeffs-32), data_eeprom);
        for (ulLoop=0;ulLoop<100000;ulLoop++){};
        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON);
    }
    else
    {
        // Send new parameter to EEPROM
        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF);
        write_i2c(I2C_SLV_ADDR_EEPROM, 2+size_coeffs, data_eeprom);
        for (ulLoop=0;ulLoop<100000;ulLoop++){};
        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON);
    }

    return 1;
}

uint8_t save_dsp_coeffs_offboard_eeprom(dsp_class_t dsp_class, uint16_t id)
{
    static u_uint16_t u_add;
    static uint8_t *p_val, size_coeffs;

    size_coeffs = 4*num_coeffs_dsp_module[dsp_class];

    // Increment element position on parameter address and prepare for EEPROM
    u_add.u16 = dsp_modules_eeprom_add[dsp_class] + id*size_coeffs;
    data_eeprom[0] = u_add.u8[1];
    data_eeprom[1] = u_add.u8[0];

    // Perform typecast of pointer to coefficients avoid local copy of them
    switch(dsp_class)
    {
        case DSP_SRLim:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_srlim[id].coeffs.f;
            break;
        }

        case DSP_LPF:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_lpf[id].coeffs.f;
            break;
        }

        case DSP_PI:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_pi[id].coeffs.f;
            break;
        }
        case DSP_IIR_2P2Z:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_iir_2p2z[id].coeffs.f;
            break;
        }

        case DSP_IIR_3P3Z:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f;
            break;
        }

        case DSP_VdcLink_FeedForward:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_ff[id].coeffs.f;
            break;
        }

        default:
        {
            return 0;
        }
    }

    // Prepare EEPROM data
    memcpy(&data_eeprom[2], p_val, size_coeffs);

    if( size_coeffs > 32 )
    {
        // Send new parameter to EEPROM
        write_i2c_offboard_isolated(I2C_SLV_ADDR_EEPROM, 2+32, data_eeprom);
        for (ulLoop=0;ulLoop<100000;ulLoop++){};

        u_add.u16 += 32;
        data_eeprom[0] = u_add.u8[1];
        data_eeprom[1] = u_add.u8[0];
        memcpy(&data_eeprom[2], &data_eeprom[34], size_coeffs-32);

        // Send new parameter to EEPROM
        write_i2c_offboard_isolated(I2C_SLV_ADDR_EEPROM, 2+(size_coeffs-32), data_eeprom);
        for (ulLoop=0;ulLoop<100000;ulLoop++){};
    }
    else
    {
        // Send new parameter to EEPROM
        write_i2c_offboard_isolated(I2C_SLV_ADDR_EEPROM, 2+size_coeffs, data_eeprom);
        for (ulLoop=0;ulLoop<100000;ulLoop++){};
    }

    return 1;
}

uint8_t load_dsp_coeffs_onboard_eeprom(dsp_class_t dsp_class, uint16_t id)
{
    static u_uint16_t u_add;
    static uint8_t *p_val, size_coeffs;

    size_coeffs = 4*num_coeffs_dsp_module[dsp_class];

    // Increment element position on parameter address and prepare for EEPROM
    u_add.u16 = dsp_modules_eeprom_add[dsp_class] + id*size_coeffs;
    data_eeprom[0] = u_add.u8[1];
    data_eeprom[1] = u_add.u8[0];

    // Perform typecast of pointer to coefficients avoid local copy of them
    switch(dsp_class)
    {
        case DSP_SRLim:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_srlim[id].coeffs.f;
            break;
        }

        case DSP_LPF:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_lpf[id].coeffs.f;
            break;
        }

        case DSP_PI:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_pi[id].coeffs.f;
            break;
        }
        case DSP_IIR_2P2Z:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_iir_2p2z[id].coeffs.f;
            break;
        }

        case DSP_IIR_3P3Z:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f;
            break;
        }

        case DSP_VdcLink_FeedForward:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_ff[id].coeffs.f;
            break;
        }

        default:
        {
            return 0;
        }
    }

    if( size_coeffs > 32 )
    {
        // Send new parameter to EEPROM
        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF);
        read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 32, data_eeprom);
        for (ulLoop=0;ulLoop<100000;ulLoop++){};
        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON);

        memcpy(p_val, &data_eeprom, 32);

        u_add.u16 += 32;
        data_eeprom[0] = u_add.u8[1];
        data_eeprom[1] = u_add.u8[0];

        // Send new parameter to EEPROM
        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF);
        read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, size_coeffs-32, data_eeprom);
        for (ulLoop=0;ulLoop<100000;ulLoop++){};
        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON);

        memcpy(p_val+32, &data_eeprom, size_coeffs-32);
    }
    else
    {
        read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, size_coeffs, data_eeprom);
        memcpy( p_val, &data_eeprom, size_coeffs);
    }

    return 1;
}

uint8_t load_dsp_coeffs_offboard_eeprom(dsp_class_t dsp_class, uint16_t id)
{
    static u_uint16_t u_add;
    static uint8_t *p_val, size_coeffs;

    size_coeffs = 4*num_coeffs_dsp_module[dsp_class];

    // Increment element position on parameter address and prepare for EEPROM
    u_add.u16 = dsp_modules_eeprom_add[dsp_class] + id*size_coeffs;
    data_eeprom[0] = u_add.u8[1];
    data_eeprom[1] = u_add.u8[0];

    // Perform typecast of pointer to coefficients avoid local copy of them
    switch(dsp_class)
    {
        case DSP_SRLim:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_srlim[id].coeffs.f;
            break;
        }

        case DSP_LPF:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_lpf[id].coeffs.f;
            break;
        }

        case DSP_PI:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_pi[id].coeffs.f;
            break;
        }
        case DSP_IIR_2P2Z:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_iir_2p2z[id].coeffs.f;
            break;
        }

        case DSP_IIR_3P3Z:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f;
            break;
        }

        case DSP_VdcLink_FeedForward:
        {
            p_val = (uint8_t *) &g_controller_mtoc.dsp_modules.dsp_ff[id].coeffs.f;
            break;
        }

        default:
        {
            return 0;
        }
    }

    if( size_coeffs > 32 )
    {
        // Send new parameter to EEPROM
        read_i2c_offboard_isolated(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 32, data_eeprom);
        for (ulLoop=0;ulLoop<100000;ulLoop++){};

        memcpy(p_val, &data_eeprom, 32);

        u_add.u16 += 32;
        data_eeprom[0] = u_add.u8[1];
        data_eeprom[1] = u_add.u8[0];

        // Send new parameter to EEPROM
        read_i2c_offboard_isolated(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, size_coeffs-32, data_eeprom);
        for (ulLoop=0;ulLoop<100000;ulLoop++){};

        memcpy(p_val+32, &data_eeprom, size_coeffs-32);
    }
    else
    {
        read_i2c_offboard_isolated(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, size_coeffs, data_eeprom);
        memcpy( p_val, &data_eeprom, size_coeffs);
    }

    return 1;
}

uint8_t save_dsp_coeffs_eeprom(dsp_class_t dsp_class, uint16_t id, param_memory_t type_memory)
{
    switch(type_memory)
    {
        case Offboard_EEPROM:
        {
            return save_dsp_coeffs_offboard_eeprom(dsp_class, id);
        }

        case Onboard_EEPROM:
        {
            return save_dsp_coeffs_onboard_eeprom(dsp_class, id);
        }

        default:
        {
            return 0;
        }
    }
}

uint8_t load_dsp_coeffs_eeprom(dsp_class_t dsp_class, uint16_t id, param_memory_t type_memory)
{
    switch(type_memory)
    {
        case Offboard_EEPROM:
        {
            return load_dsp_coeffs_offboard_eeprom(dsp_class, id);
        }

        case Onboard_EEPROM:
        {
            return load_dsp_coeffs_onboard_eeprom(dsp_class, id);
        }

        default:
        {
            return 0;
        }
    }
}

void save_dsp_modules_eeprom(param_memory_t type_memory)
{
    dsp_class_t dsp_class;
    uint16_t    id;

    for(dsp_class = 0; dsp_class < NUM_DSP_CLASSES; dsp_class++)
    {
        for(id = 0; id < num_dsp_modules[dsp_class]; id++)
        {
            save_dsp_coeffs_eeprom(dsp_class, id, type_memory);
        }
    }
}

void load_dsp_modules_eeprom(param_memory_t type_memory)
{
    dsp_class_t dsp_class;
    uint16_t    id;

    for(dsp_class = 0; dsp_class < NUM_DSP_CLASSES; dsp_class++)
    {
        for(id = 0; id < num_dsp_modules[dsp_class]; id++)
        {
            load_dsp_coeffs_eeprom(dsp_class, id, type_memory);
        }
    }
}
