/******************************************************************************
 * Copyright (C) 2019 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fap_2p2s.h
 * @brief FAP-2P2S module
 *
 * Module for control of FAP-2P2S power supplies. It implements the controller
 * for load current, current share between both parallel arms and current share
 * between 8 IGBT's.
 *
 * @author gabriel.brunheira
 * @date 17/04/2019
 *
 */

#ifndef FAP_2P2S_H_
#define FAP_2P2S_H_

void fap_2p2s_system_config(void);

#endif /* FAP_2P2S_H_ */
