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
 * @file ethernet_uip.h
 * @brief Ethernet module.
 *
 * @author joao.rosa
 *
 * @date 07/08/2014
 *
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#ifndef ETHERNET_UIP_H_
#define ETHERNET_UIP_H_

extern void ethernet_init(void);

extern void ethernet_process_data(void);

extern void ip_address_read(uint8_t *ip_addr0, uint8_t *ip_addr1, uint8_t *ip_addr2, uint8_t *ip_addr3);
extern void ip_address_write(uint8_t ip_addr0, uint8_t ip_addr1, uint8_t ip_addr2, uint8_t ip_addr3);
extern void ip_mask_read(uint8_t *ip_mask0, uint8_t *ip_mask1, uint8_t *ip_mask2, uint8_t *ip_mask3);
extern void ip_mask_write(uint8_t ip_mask0, uint8_t ip_mask1, uint8_t ip_mask2, uint8_t ip_mask3);
extern uint16_t eth_port_read(void);
extern void eth_port_write(uint16_t EthP);
extern uint64_t mac_address_read(void);

#endif /* ETHERNET_UIP_H_ */
