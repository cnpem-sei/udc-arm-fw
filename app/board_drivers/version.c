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
 * @file version.c
 * @brief Firmware version module
 * 
 * This module contains information about current build version of UDC ARM core.
 *
 * @author gabriel.brunheira
 * @date 08/02/2018
 *
 */

#include "version.h"

volatile firmwares_version_t firmwares_version;

const char * udc_arm_version = "V0.41u2020-07-27";
