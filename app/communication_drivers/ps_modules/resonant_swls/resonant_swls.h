/******************************************************************************
 * Copyright (C) 2022 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file resonant_swls.h
 * @brief Resonant converter module for SWLS
 *
 * Module for control of resonant convert power supply designed for the
 * superconducting Wavelength Shifter. It implements the controller for load
 * current using frequency modulation strategy, instead of pulse-width
 * modulation (PWM).
 *
 * @author gabriel.brunheira
 * @date 18/07/2022
 *
 */

#ifndef RESONANT_SWLS_H_
#define RESONANT_SWLS_H_

void resonant_swls_system_config(void);

#endif /* RESONANT_SWLS_H_ */
