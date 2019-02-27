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
 * @file fac_2s_dcdc.c
 * @brief FAC-2S DC/DC Stage module
 *
 * Module for control of two DC/DC modules of FAC power supplies for focusing
 * quadrupoles from booster. It implements the controller for load current.
 *
 * @author gabriel.brunheira
 * @date 27/02/2019
 *
 */

#ifndef FAC_2S_DCDC_H_
#define FAC_2S_DCDC_H_

void fac_2s_dcdc_system_config(void);

#endif /* FAC_2S_DCDC_H_ */
