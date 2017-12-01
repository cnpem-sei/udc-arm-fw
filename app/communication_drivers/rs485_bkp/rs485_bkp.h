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
 * @file rs485_bkp.h
 * @brief Backplane RS485 module.
 *
 * Module to process data in RS485 bus for backplane.
 *
 * @author joao.rosa
 *
 * @date 17/06/2015
 *
 */

#ifndef RS485_BKP_H_
#define RS485_BKP_H_

extern void init_rs485_bkp(void);
extern void rs485_bkp_process_data(void);
extern void rs485_bkp_tx_handler(void);

#endif /* RS485_BKP_H_ */
