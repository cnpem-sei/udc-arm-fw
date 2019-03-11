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
 * @file fac_dcdc_ema.h
 * @brief FAC DC/DC Stage module for dipole magnet from EMA beamline.
 *
 * Module for control of DC/DC module of FAC power supply for dipole magnet used
 * at EMA beamline from Sirius. It implements the controller for load current.
 *
 * @author gabriel.brunheira
 * @date 08/03/2019
 *
 */

#ifndef FAC_DCDC_EMA_H_
#define FAC_DCDC_EMA_H_

void fac_dcdc_ema_system_config(void);

#endif /* FAC_DCDC_EMA_H_ */
