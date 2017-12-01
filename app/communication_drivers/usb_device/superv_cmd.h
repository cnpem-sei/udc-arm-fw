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
 * @file superv_cmd.h
 * @brief USB Supervisory.
 *
 * @author joao.rosa
 *
 * @date 16/09/2013
 *
 */

#ifndef SUPERV_CMD_H_
#define SUPERV_CMD_H_

#include <stdint.h>

 typedef struct
 {
	 uint8_t buffer_rx [265];
	 int16_t counter;
     uint8_t Start;
     uint8_t Ndado;
 }buffer_usb_t;

  typedef struct
 {
	 uint8_t CMD;
     uint8_t NDADO;
     uint8_t DADO[100];
     uint8_t CKS;

 }protocolo_usb_t;

 extern buffer_usb_t DadoUsb;
 extern protocolo_usb_t MensagemUsb;

extern void clear_buffer_usb(void);
extern void mensagem_usb(void);

extern void set_new_data(void);

#endif /* SUPERV_CMD_H_ */
