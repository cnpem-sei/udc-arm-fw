/******************************************************************************
 * Copyright (C) 2018 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file version.h
 * @brief Firmware version module
 * 
 * This module contains information about current build version of UDC ARM core.
 *
 * @author gabriel.brunheira
 * @date 08/02/2018
 *
 */

#ifndef VERSION_H_
#define VERSION_H_

#include <stdint.h>

#define SIZE_VERSION 16

typedef struct
{
    char udc_arm[SIZE_VERSION];
    char udc_c28[SIZE_VERSION];
    char hradc0_cpld[SIZE_VERSION];
    char hradc1_cpld[SIZE_VERSION];
    char hradc2_cpld[SIZE_VERSION];
    char hradc3_cpld[SIZE_VERSION];
    char iib_arm[SIZE_VERSION];
    char ihm_pic[SIZE_VERSION];
} cores_version_t;

typedef union
{
    cores_version_t     cores;
    uint8_t             u8[8*SIZE_VERSION];
} firmwares_version_t;

extern volatile firmwares_version_t firmwares_version;
extern const char * udc_arm_version;

#endif /* VERSION_H_ */
