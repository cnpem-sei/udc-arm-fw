/*
 * superv_cmd.c
 *
 *  Created on: 16/09/2013
 *      Author: joao.rosa
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "usb_serial_structs.h"
#include "usb_dev_serial.h"



/*********************************************************************************************************
 *                          Comandos da comunicação com o com o supervisório via usb
 *
 *
 *
 *********************************************************************************************************/
#include "superv_cmd.h"

buffer_usb_t DadoUsb;
protocolo_usb_t MensagemUsb;

uint8_t test1;

static uint8_t NewData = 0;

/******************************************************************************************
 *
 * As sub rotinas contidas abaixo se referem a comunicação
 *
 *
 *******************************************************************************************/

void
SetNewData(void)
{
	NewData = 1;
}

// Sub rotina para teste de Checksum
uint8_t CheckSumTestUsb(void)
{
   unsigned char result = 0;

   while (DadoUsb.counter > 0){
      result += DadoUsb.buffer_rx[DadoUsb.counter - 1];
      DadoUsb.counter--;
   }

   if(result == 0) return(1);
   else return(0);
}

// Sub rotina para alocação dos dados recebidos
void SeparaDadoUsb(void){

	uint8_t count = 0;

	MensagemUsb.CMD = DadoUsb.buffer_rx[0];
	MensagemUsb.NDADO = DadoUsb.buffer_rx[1];

   for(count = 0; count < MensagemUsb.NDADO; count++){

	   MensagemUsb.DADO[count] = DadoUsb.buffer_rx[count + 2];

   }

   MensagemUsb.DADO[count] = 0;

   DadoUsb.counter = 0;
   DadoUsb.Ndado = 0;

}

// Sub rotina para calculo do Checksum para envio da mensagem
void CheckSumCalcUsb(void){
	uint8_t count;

	MensagemUsb.CKS = MensagemUsb.CMD;
	MensagemUsb.CKS += MensagemUsb.NDADO;

   if(MensagemUsb.NDADO > 0)
   {
		for(count = 0; count < MensagemUsb.NDADO; count++)
		{
			MensagemUsb.CKS += MensagemUsb.DADO[count];
		}
   }

   MensagemUsb.CKS = 0x100 - MensagemUsb.CKS;

}

/**********************************************************************************************************************
 *
 * Subrotina destinada a enviar dados para o display via UART2
 *
 **********************************************************************************************************************/
void SendUsb(void){

   uint8_t count;

   CheckSumCalcUsb();            // Rotina de calculo de CheckSum

   USBBufferWrite(&g_sTxBuffer, &MensagemUsb.CMD, 1);
   USBBufferWrite(&g_sTxBuffer, &MensagemUsb.NDADO, 1);

   if(MensagemUsb.NDADO > 0)
   {
	   for(count = 0; count < MensagemUsb.NDADO; count++)
	   {
		  USBBufferWrite(&g_sTxBuffer, &MensagemUsb.DADO[count], 1);
	   }
   }

   USBBufferWrite(&g_sTxBuffer, &MensagemUsb.CKS, 1);

}

// Sub rotina que limba o buffer de entrada para UART2
void ClearBufferUsb(void){
	unsigned char ucChar;
	while (USBBufferDataAvailable(&g_sRxBuffer))// Testa se há bytes no buffer de entrada
   {
	   USBBufferRead(&g_sRxBuffer, &ucChar, 1);      // Retira os dados
   }
}

/********************************************************************************************************************
 *
 * Processa os comandos enviados pelo display
 *
 ********************************************************************************************************************/
void ProcessCmdUsb(){
	uint8_t mani1, mani2, mani3 = 0;
	uint8_t Rx[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
	uint32_t uLong = 0;

  switch(MensagemUsb.CMD){

     // Comandos de consulta de parâmetros, pede que seja enviado o parametro requerido
     // Endereço RS-485
     case 0x00:
			   MensagemUsb.CMD = 0x00;
			   MensagemUsb.NDADO = 0x01;
			   //MensagemUsb.DADO[0] = Parametros.End;
			   SendUsb(); // Envia mensagem para a usb
               break;
     // Modelo da fonte
     case 0x01:
    	       MensagemUsb.CMD = 0x01;
    	       MensagemUsb.NDADO = 0x01;
    	       //MensagemUsb.DADO[0] = Parametros.MdFnt;
    	       SendUsb(); // Envia mensagem para a usb
               break;
     // Data e hora
     case 0x02:
    	       // Chamar sub rotina de coleta de dados do RTC
    	       //Read_Rtc_Clock(true); // Faz a leitura do RTC
    	       MensagemUsb.CMD = 0x02;
    	       MensagemUsb.NDADO = 0x06;
    	       //MensagemUsb.DADO[0] = Rtc.RTCano;
    	       //MensagemUsb.DADO[1] = Rtc.RTCmes;
    	       //MensagemUsb.DADO[2] = Rtc.RTCdia;
    	       //MensagemUsb.DADO[3] = Rtc.RTChora;
    	       //MensagemUsb.DADO[4] = Rtc.RTCmin;
    	       //MensagemUsb.DADO[5] = Rtc.RTCseg;
    	       SendUsb(); // Envia mensagem para a usb
               break;
     //
     case 0x03:

               break;
     // Configuração do interlock
     case 0x04:
    	       MensagemUsb.CMD = 0x04;
    	       MensagemUsb.NDADO = 0x02;
    	       //MensagemUsb.DADO[0] = Parametros.ItlkAnalog;
    	       //MensagemUsb.DADO[1] = Parametros.ItlkStatInput;
    	       SendUsb(); // Envia mensagem para a usb
               break;
     // Configuração de Alarme
     case 0x05:
    	       MensagemUsb.CMD = 0x05;
    	       MensagemUsb.NDADO = 0x02;
    	       //MensagemUsb.DADO[0] = Parametros.AlrmAnlog;
    	       //MensagemUsb.DADO[1] = Parametros.AlrmStatInput;
    	       SendUsb(); // Envia mensagem para a usb
               break;
     // Status local/remoto
     case 0x06:
    	       MensagemUsb.CMD = 0x06;
    	       MensagemUsb.NDADO = 0x01;
    	       //MensagemUsb.DADO[0] = Parametros.LocRem;
    	       SendUsb(); // Envia mensagem para a usb
               break;
     // Senha
     case 0x07:
    	       MensagemUsb.CMD = 0x07;
    	       MensagemUsb.NDADO = 0x03;
    	       //MensagemUsb.DADO[0] = Parametros.Senha >> 16;
    	       //MensagemUsb.DADO[1] = Parametros.Senha >> 8;
    	       //MensagemUsb.DADO[2] = Parametros.Senha;
    	       SendUsb(); // Envia mensagem para a usb
               break;
     // Setpoint das para geração de alarme ou interlock por meio das medidas analogicas
     case 0x08:
    	       MensagemUsb.CMD = 0x08;
    	       MensagemUsb.NDADO = 0x0F;
/*
    	       MensagemUsb.DADO[0] = Parametros.SetPointAn1 >> 8;
    	       MensagemUsb.DADO[1] = Parametros.SetPointAn1;

    	       MensagemUsb.DADO[2] = Parametros.SetPointAn2 >> 8;
    	       MensagemUsb.DADO[3] = Parametros.SetPointAn2;

    	       MensagemUsb.DADO[4] = Parametros.SetPointAn3 >> 8;
    	       MensagemUsb.DADO[5] = Parametros.SetPointAn3;

    	       MensagemUsb.DADO[6] = Parametros.SetPointAn4 >> 8;
    	       MensagemUsb.DADO[7] = Parametros.SetPointAn4;

    	       MensagemUsb.DADO[8] = Parametros.SetPointAn5 >> 8;
    	       MensagemUsb.DADO[9] = Parametros.SetPointAn5;

    	       MensagemUsb.DADO[10] = Parametros.SetPointAn6 >> 8;
    	       MensagemUsb.DADO[11] = Parametros.SetPointAn6;

    	       MensagemUsb.DADO[12] = Parametros.SetPointAn7 >> 8;
    	       MensagemUsb.DADO[13] = Parametros.SetPointAn7;

    	       MensagemUsb.DADO[14] = Parametros.SetPointAn8 >> 8;
    	       MensagemUsb.DADO[15] = Parametros.SetPointAn8; */

    	       SendUsb(); // Envia mensagem para a usb
               break;

	   // Numero de série
	   case 0x09:
		   	   MensagemUsb.CMD = 0x09;
		   	   MensagemUsb.NDADO = 0x08;
		   	   //MensagemUsb.DADO[0] = Parametros.NSerie >> 56;
		   	   //MensagemUsb.DADO[1] = Parametros.NSerie >> 48;
		   	   //MensagemUsb.DADO[2] = Parametros.NSerie >> 40;
		   	   //MensagemUsb.DADO[3] = Parametros.NSerie >> 32;
		   	   //MensagemUsb.DADO[4] = Parametros.NSerie >> 24;
		   	   //MensagemUsb.DADO[5] = Parametros.NSerie >> 16;
		   	   //MensagemUsb.DADO[6] = Parametros.NSerie >> 8;
		   	   //MensagemUsb.DADO[7] = Parametros.NSerie;

		   	   SendUsb(); // Envia mensagem para a usb
			   break;

     // Comandos de leitura, pede para retornar os dados requeridos
     // Corrente de saída
     case 0x10:
    	 	   //shm_getIout();
    	       MensagemUsb.CMD = 0x10;
    	       MensagemUsb.NDADO = 0x03;
			   //MensagemUsb.DADO[0] = 0x03 & LeituraVarDin.IoutReadI >> 16;
			   //MensagemUsb.DADO[1] = LeituraVarDin.IoutReadI >> 8;
			   //MensagemUsb.DADO[2] = LeituraVarDin.IoutReadI;

			   SendUsb(); // Envia mensagem para a usb
    	 	   break;
     //
     case 0x11:


               break;
     // Entadas On/Off
     case 0x12:
    	 	   // Chama subrotina que faz a leitura das entradas digitais de status
    	 	   MensagemUsb.CMD = 0x12;
    	 	   MensagemUsb.NDADO = 0x02;

    	 	   //MensagemUsb.DADO[0] = LeituraVarDin.SobreCorrente;
    	 	   //MensagemUsb.DADO[1] = LeituraVarDin.Fusivel;

    	 	   SendUsb(); // Envia mensagem para a usb

               break;
     // Log de eventos
     case 0x13:

               break;
     // Post-Mortem
     case 0x14:

               break;
     // Status do interlock
     case 0x15:

               break;
     // Sem definição
     case 0x16:

               break;
     // Sem definição
     case 0x17:

               break;
     // Sem definição
     case 0x18:

               break;

     // Comandos de atuação
     // Seta corrente de saída
	 case 0x20:
		 	   uLong = 0x00000003 & MensagemUsb.DADO[0];
		 	   uLong = uLong << 8;
		 	   uLong |= MensagemUsb.DADO[1];
		 	   uLong = uLong << 8;
		 	   uLong |= MensagemUsb.DADO[2];
			   //shm_setPiRef( uLong );
		       break;
	 // Liga/desliga saida da fonte
     case 0x22:
    	 	   //ShmSetStatusFonteOp(MensagemUsb.DADO[0]);
			   break;

     // Liga/desliga malha de realimentação da Fonte
	 case 0x23:
		       //ShmSetStatusMalha(MensagemUsb.DADO[0]);
			   break;

     // Comandos de alteração de parâmetro - essas funções são utilizadas para tratar o ACK
     // Os valores recebidos devem ser ajustados
     // Endereço RS-485
     case 0x30:
               //Parametros.End = MensagemUsb.DADO[0];
               // Chama subrotina para salvar o novo dado na memória não volátil
               break;
     // Data e hora
     case 0x32:
    	       //Rtc.RTCano = MensagemUsb.DADO[0];
    	       //Rtc.RTCmes = MensagemUsb.DADO[1];
    	       //Rtc.RTCdia = MensagemUsb.DADO[2];
    	       //Rtc.RTChora = MensagemUsb.DADO[3];
    	       //Rtc.RTCmin = MensagemUsb.DADO[4];
    	       //Rtc.RTCseg = MensagemUsb.DADO[5];
			   // Chamar subrotina de ajuste dos dados no RTC
    	       //Write_Rtc_Clock();
			   // Retornar ACK para PIC32 sinalizando que a tarefa foi executada com sucesso
               break;
     // Ganho controlador Proporcional
     case 0x33:
    	       //ShmSetControlKp(atof(MensagemUsb.DADO));
               // Chama subrotina para salvar o novo dado na memória FLASH
               // Retornar ACK para PIC32 sinalizando que a tarefa foi executada com sucesso
               break;
     // Ganho controlador Integral
     case 0x34:
    	 	   //ShmSetControlKi(atof(MensagemUsb.DADO));
               break;
     // Configuração do alarme
     case 0x35:

               break;
     // Alteração local/remoto
     case 0x36:
    	 	   //Parametros.LocRem = MensagemUsb.DADO[0];
    	 	  // Retornar ACK para PIC32 sinalizando que a tarefa foi executada com sucesso
               break;
     // Salva nova Senha
     case 0x37:
    	       //Parametros.Senha = MensagemUsb.DADO[0];
    	       mani2 = MensagemUsb.DADO[1];
    	       mani3 = MensagemUsb.DADO[2];
    	       //Parametros.Senha = Parametros.Senha << 8;
    	       //Parametros.Senha |= mani2;
    	       //Parametros.Senha = Parametros.Senha << 8;
    	       //Parametros.Senha |= mani3;
    	       // Chamar função que salva a nova senha na memória FLASH
    	       // Retornar ACK para PIC32 sinalizando que a tarefa foi executada com sucesso
               break;
     // Salva novo ajuste de setpoint para ADC
     case 0x38:
    	       /*Parametros.SetPointAn1 = MensagemUsb.DADO[0];
			   Parametros.SetPointAn1 = Parametros.SetPointAn1 << 8;
			   Parametros.SetPointAn1 |= MensagemUsb.DADO[1];

			   Parametros.SetPointAn2 = MensagemUsb.DADO[2];
			   Parametros.SetPointAn2 = Parametros.SetPointAn2 << 8;
			   Parametros.SetPointAn2 |= MensagemUsb.DADO[3];

			   Parametros.SetPointAn3 = MensagemUsb.DADO[4];
			   Parametros.SetPointAn3 = Parametros.SetPointAn3 << 8;
			   Parametros.SetPointAn3 |= MensagemUsb.DADO[5];

			   Parametros.SetPointAn4 = MensagemUsb.DADO[6];
			   Parametros.SetPointAn4 = Parametros.SetPointAn4 << 8;
			   Parametros.SetPointAn4 |= MensagemUsb.DADO[7];

			   Parametros.SetPointAn5 = MensagemUsb.DADO[8];
			   Parametros.SetPointAn5 = Parametros.SetPointAn5 << 8;
			   Parametros.SetPointAn5 |= MensagemUsb.DADO[9];

			   Parametros.SetPointAn6 = MensagemUsb.DADO[10];
			   Parametros.SetPointAn6 = Parametros.SetPointAn6 << 8;
			   Parametros.SetPointAn6 |= MensagemUsb.DADO[11];

			   Parametros.SetPointAn7 = MensagemUsb.DADO[12];
			   Parametros.SetPointAn7 = Parametros.SetPointAn4 << 8;
			   Parametros.SetPointAn7 |= MensagemUsb.DADO[13];

			   Parametros.SetPointAn8 = MensagemUsb.DADO[14];
			   Parametros.SetPointAn8 = Parametros.SetPointAn8 << 8;
			   Parametros.SetPointAn8 |= MensagemUsb.DADO[15];*/
			   // Chama função que grava o setpoint na memória FLASH
			   // Chamar função que envia os setpoints para o ADCP
			   // Retornar ACK para PIC32 sinalizando que a tarefa foi executada com sucesso
               break;

     // Comandos de consulta de curva, deve retornar os dados recebidos
     // Os valores recebidos devem ser ajustados
     // Curvas armazenadas
     case 0x40:

               break;
     // Visualizar curva
     case 0x41:

               break;
     // Visualizar parametro da curva
     case 0x42:

               break;

     // Comandos de ajuste para curva - essas funções são utilizadas para tratar o ACK
     // Seleciona curva
     case 0x50:

               break;
     // Inicia curva
     case 0x51:

               break;
     // Ajusta parametro da curva
     case 0x52:

               break;

     default:

    	 	   break;


     }

}

void MensagUsb(void)
{
   if(NewData)// Testa a integridade da Mensagem
   {
	   SeparaDadoUsb(); // Chama sub rotina que tira os dados do buffer e aloca na estrutura
	   ProcessCmdUsb(); // Chama Sub rotina para interpretação dos dados recebidos
	   NewData = 0;
	   ClearBufferUsb();  // Limpa buffer antes de voltar para a operação normal
   }


   //MessageUart.NewDataSupervisorio = 0; // Apaga flag de nova mensagem do supervisório
}


