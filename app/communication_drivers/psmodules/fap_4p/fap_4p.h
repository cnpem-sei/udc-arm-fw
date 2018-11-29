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
 * @file fap_4p.h
 * @brief FAP-4P module
 *
 * Module for control of FAP-4P power supplies. It implements the controller for
 * load current and current share between 8 IGBT's
 *
 * @author gabriel.brunheira
 * @date 29/11/2018
 *
 */

#ifndef FAP_4P_H_
#define FAP_4P_H_

void fap_4p_system_config(void);

#endif /* FAP_4P_H_ */
