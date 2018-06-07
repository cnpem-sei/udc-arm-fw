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
 * @file fbp_dclink.h
 * @brief System setup for operation as FBP DC Link
 *
 * @author gabriel.brunheira
 * @date 06/06/2018
 *
 */

#ifndef FBP_DCLINK_H_
#define FBP_DCLINK_H_

void  set_digital_potentiometer(float perc);
float get_digital_potentiometer(void);

void fbp_dclink_system_config(void);

#endif /* FBP_DCLINK_H_ */
