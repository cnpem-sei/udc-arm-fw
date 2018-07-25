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
 * @file fac_2p4s_acdc.h
 * @brief FAC-2P4S AC/DC Stage module
 *
 * Module for control of two AC/DC modules of FAC power supplies for dipoles
 * from booster. It implements the individual controllers for input current and
 * capacitor bank voltage of each AC/DC module.
 *
 * @author gabriel.brunheira
 * @date 21/07/2018
 *
 */

#ifndef FAC_2P4S_ACDC_H_
#define FAC_2P4S_ACDC_H_

void fac_2p4s_acdc_system_config(void);

#endif /* FAC_2P4S_ACDC_H_ */
