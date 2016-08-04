/*
 * ihm.c
 *
 *  Created on: 17/06/2015
 *      Author: joao.rosa
 */


#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/debug.h"

#include "set_pinout_udc_v2.0.h"
//#include "set_pinout_ctrl_card.h"

#include "../rs485/rs485.h"

#include "../i2c_onboard/rtc.h"

#include "../shared_memory/ctrl_law.h"
#include "../shared_memory/main_var.h"

#include "../ethernet/ethernet_uip.h"

#include "ihm.h"

#include <stdint.h>


#define STX 			0x02
#define SERIAL_BUF_SIZE	256


 typedef struct
 {
	 uint8_t buffer_rx [265];
	 int16_t counter;
     uint8_t Start;
     uint8_t Ndado;
     uint8_t csum;
 }buffer_t;

  typedef struct
 {
	 uint8_t CMD;
     uint8_t PDADO;
     uint8_t NDADO;
     uint8_t DADO[256];
     uint8_t ACK;
     uint8_t CKS;

 }protocolo_t;

buffer_t Dado;
protocolo_t Mensagem;


//*****************************************************************************

static uint8_t NewData = 0;

// Local = 1 Remote = 0
static uint8_t LocRem = 0;

//*****************************************************************************

// Partir o float em bytes
union
{
   float f;
   char c[4];
} floatNchars;

/******************************************************************************************
 *
 * As sub rotinas contidas abaixo se referem a comunicação com o microcontrolador da IHM
 *
 *
 *******************************************************************************************/

// Sub rotina para alocação dos dados recebidos
void SeparaDado(void){

	uint8_t count = 0;

   Mensagem.CMD = Dado.buffer_rx[0];
   Mensagem.PDADO = Dado.buffer_rx[1];
   Mensagem.NDADO = Dado.buffer_rx[2];

   for(count = 0; count < Mensagem.NDADO; count++){

      Mensagem.DADO[count] = Dado.buffer_rx[count + 3];

   }

   Mensagem.ACK = Dado.buffer_rx[count + 3];

   Dado.counter = 0;
   Dado.Ndado = 0;

}

/**********************************************************************************************************************
 *
 * Subrotina destinada a enviar dados para o display via UART2
 *
 **********************************************************************************************************************/
void SendDisplay(void){

	unsigned int i;

	// Prepare answer
	Mensagem.CKS = 0;

	Mensagem.CKS -= Mensagem.CMD;
	Mensagem.CKS -= Mensagem.PDADO;
	Mensagem.CKS -= Mensagem.NDADO;
	Mensagem.CKS -= Mensagem.ACK;

	// Send packet
	UARTCharPutNonBlocking(DISPLAY_UART_BASE,Mensagem.CMD);
	UARTCharPutNonBlocking(DISPLAY_UART_BASE,Mensagem.PDADO);
	UARTCharPutNonBlocking(DISPLAY_UART_BASE,Mensagem.NDADO);


	for(i = 0; i < Mensagem.NDADO; ++i)
	{
		// Wait until have space in the TX buffer
		while(!UARTSpaceAvail(DISPLAY_UART_BASE));
		// CheckSum calc
		Mensagem.CKS -= Mensagem.DADO[i];
		// Send Byte
		UARTCharPutNonBlocking(DISPLAY_UART_BASE, Mensagem.DADO[i]);
	}
	// Wait until have space in the TX buffer
	while(!UARTSpaceAvail(DISPLAY_UART_BASE));
	// Send Byte
	UARTCharPutNonBlocking(DISPLAY_UART_BASE,Mensagem.ACK);
	UARTCharPutNonBlocking(DISPLAY_UART_BASE, Mensagem.CKS);

}

/********************************************************************************************************************
 *
 * Processa os comandos enviados pelo display
 *
 ********************************************************************************************************************/
void ProcessCmd(){
	uint8_t mani1, mani2, mani3 = 0;
	uint16_t uInt = 0;
	uint64_t var64 = 0;
	float Var = 0;

  switch(Mensagem.CMD){

     // Comandos de consulta de parâmetros, pede que seja enviado o parametro requerido
     // Endereço RS-485
     case 0x00:
    	 	   Mensagem.CMD = 0x00;
    	       Mensagem.PDADO = 0x00;
    	       Mensagem.NDADO = 0x01;
    	       //Mensagem.DADO[0] = Parametros.End;
    	       Mensagem.DADO[0] = ReadRS485Address();
    	       Mensagem.ACK = 0x00;
    	       SendDisplay(); // Envia mensagem para o Display
               break;
     // Modelo da fonte
     case 0x01:
    	       Mensagem.CMD = 0x01;
    	       Mensagem.PDADO = 0x00;
    	       Mensagem.NDADO = 0x01;
    	       //Mensagem.DADO[0] = Parametros.MdFnt;
    	       Mensagem.ACK = 0x00;
    	       SendDisplay(); // Envia mensagem para o Display
               break;
     // Data e hora
     case 0x02:
    	       // Chamar sub rotina de coleta de dados do RTC
    	       //Read_Rtc_Clock(true); // Faz a leitura do RTC
    	       Mensagem.CMD = 0x02;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x07;
			   //Mensagem.DADO[0] = Rtc.RTCano;
			   //Mensagem.DADO[1] = Rtc.RTCmes;
			   //Mensagem.DADO[2] = Rtc.RTCdia;
			   //Mensagem.DADO[3] = Rtc.RTChora;
			   //Mensagem.DADO[4] = Rtc.RTCmin;
			   //Mensagem.DADO[5] = Rtc.RTCseg;
			   //Mensagem.DADO[6] = Rtc.RTCdiasem;

			   var64 = DataHourRead();

			   Mensagem.DADO[0] = var64 >> 48;
			   Mensagem.DADO[1] = var64 >> 40;
			   Mensagem.DADO[2] = var64 >> 32;
			   Mensagem.DADO[3] = var64 >> 16;
			   Mensagem.DADO[4] = var64 >> 8;
			   Mensagem.DADO[5] = var64;
			   Mensagem.DADO[6] = var64 >> 24;
			   Mensagem.ACK = 0x00;
			   SendDisplay(); // Envia mensagem para o Display
               break;
     // Valor do PI
     case 0x03:

               break;
     // Configuração do interlock
     case 0x04:
    	       Mensagem.CMD = 0x04;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x02;
			   //Mensagem.DADO[0] = Parametros.ItlkAnalog;
			   //Mensagem.DADO[1] = Parametros.ItlkStatInput;
			   Mensagem.ACK = 0x00;
			   SendDisplay(); // Envia mensagem para o Display
               break;
     // Configuração de Alarme
     case 0x05:
    	       Mensagem.CMD = 0x05;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x02;
			   //Mensagem.DADO[0] = Parametros.AlrmAnlog;
			   //Mensagem.DADO[1] = Parametros.AlrmStatInput;
			   Mensagem.ACK = 0x00;
			   SendDisplay(); // Envia mensagem para o Display
               break;
     // Status local/remoto
     case 0x06:
    	       Mensagem.CMD = 0x06;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x01;
			   Mensagem.DADO[0] = LocRem; // Read ajust status
			   Mensagem.ACK = 0x00;
			   SendDisplay(); // Envia mensagem para o Display
               break;
     // Senha
     case 0x07:
    	       Mensagem.CMD = 0x07;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x03;
			   //Mensagem.DADO[0] = Parametros.Senha >> 16;
			   //Mensagem.DADO[1] = Parametros.Senha >> 8;
			   //Mensagem.DADO[2] = Parametros.Senha;
			   Mensagem.ACK = 0x00;
			   SendDisplay(); // Envia mensagem para o Display
               break;
     // Setpoint das para geração de alarme ou interlock por meio das medidas analogicas
     case 0x08:
    	       Mensagem.CMD = 0x08;
    	 	   Mensagem.PDADO = 0x00;
    	 	   Mensagem.NDADO = 0x0F;

    	 	   //Mensagem.DADO[0] = Parametros.SetPointAn1 >> 8;
    	 	   //Mensagem.DADO[1] = Parametros.SetPointAn1;

    	 	   //Mensagem.DADO[2] = Parametros.SetPointAn2 >> 8;
    	 	   //Mensagem.DADO[3] = Parametros.SetPointAn2;

    	 	   //Mensagem.DADO[4] = Parametros.SetPointAn3 >> 8;
    	 	   //Mensagem.DADO[5] = Parametros.SetPointAn3;

    	 	   //Mensagem.DADO[6] = Parametros.SetPointAn4 >> 8;
    	 	   //Mensagem.DADO[7] = Parametros.SetPointAn4;

    	 	   //Mensagem.DADO[8] = Parametros.SetPointAn5 >> 8;
    	 	   //Mensagem.DADO[9] = Parametros.SetPointAn5;

    	 	   //Mensagem.DADO[10] = Parametros.SetPointAn6 >> 8;
    	 	   //Mensagem.DADO[11] = Parametros.SetPointAn6;

    	 	   //Mensagem.DADO[12] = Parametros.SetPointAn7 >> 8;
    	 	   //Mensagem.DADO[13] = Parametros.SetPointAn7;

    	 	   //Mensagem.DADO[14] = Parametros.SetPointAn8 >> 8;
    	 	   //Mensagem.DADO[15] = Parametros.SetPointAn8;

    	 	   Mensagem.ACK = 0x00;
    	 	   SendDisplay(); // Envia mensagem para o Display
               break;

	   // Numero de série
	   case 0x09:
			   Mensagem.CMD = 0x09;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x08;
			   //Mensagem.DADO[0] = Parametros.NSerie >> 56;
			   //Mensagem.DADO[1] = Parametros.NSerie >> 48;
			   //Mensagem.DADO[2] = Parametros.NSerie >> 40;
			   //Mensagem.DADO[3] = Parametros.NSerie >> 32;
			   //Mensagem.DADO[4] = Parametros.NSerie >> 24;
			   //Mensagem.DADO[5] = Parametros.NSerie >> 16;
			   //Mensagem.DADO[6] = Parametros.NSerie >> 8;
			   //Mensagem.DADO[7] = Parametros.NSerie;
			   Mensagem.ACK = 0x00;

			   SendDisplay(); // Envia mensagem para o Display
			   break;

     // Comandos de leitura, pede para retornar os dados requeridos
     // Corrente de saída
     case 0x10:
    	 	   Mensagem.CMD = 0x10;
    	 	   Mensagem.PDADO = 0x00;
    	 	   Mensagem.NDADO = 0x04;
			   //floatNchars.f = LeituraVarDin.IoutReadF;
    	 	   floatNchars.f = IOutputRead();
			   Mensagem.DADO[0] = floatNchars.c[0];
			   Mensagem.DADO[1] = floatNchars.c[1];
			   Mensagem.DADO[2] = floatNchars.c[2];
			   Mensagem.DADO[3] = floatNchars.c[3];
    	 	   Mensagem.ACK = 0x00;

    	 	   SendDisplay(); // Envia mensagem para o Display
    	 	   break;

     // Leitura da tensão na saída
     case 0x11:
    	 	   Mensagem.CMD = 0x11;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x02;
			   //Mensagem.DADO[0] = LeituraVarDin.IoutMod1;
			   //Mensagem.DADO[1] = LeituraVarDin.IoutMod2;
			   Mensagem.DADO[0] = 4;
			   Mensagem.DADO[1] = 6;
			   Mensagem.ACK = 0x00;

			   SendDisplay(); // Envia mensagem para o Display
               break;

     // Leitura da corrente na entrada
     case 0x12:
    	 	   Mensagem.CMD = 0x12;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x04;

			   //Mensagem.DADO[0] = LeituraVarDin.ItlkFaseFault;
			   //Mensagem.DADO[1] = LeituraVarDin.ItlkOverTemp;
			   //Mensagem.DADO[2] = LeituraVarDin.ItlkDcctFault;
			   //Mensagem.DADO[3] = LeituraVarDin.ItlkLeakageToGround;

			   Mensagem.ACK = 0x00;
			   SendDisplay(); // Envia mensagem para o Display

               break;
     // Temperatura da placa
     case 0x13:
    	       //ReadAdcP();
    	       Mensagem.CMD = 0x13;
    	       Mensagem.PDADO = 0x00;
    	       Mensagem.NDADO = 0x02;
    	       Mensagem.DADO[0] = 0x01;
    	       Mensagem.DADO[1] = 0x55;

    	       Mensagem.ACK = 0x00;
    	       SendDisplay(); // Envia mensagem para o Display

               break;
     // Leitura do status da malha de realimentação
     case 0x14:
			   Mensagem.CMD = 0x14;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x01;
			   Mensagem.DADO[0] = CtrlLoopRead();

			   Mensagem.ACK = 0x00;
			   SendDisplay(); // Envia mensagem para o Display

               break;
     // Status da chave de saida
     case 0x15:
    	       Mensagem.CMD = 0x15;
    	 	   Mensagem.PDADO = 0x00;
    	 	   Mensagem.NDADO = 0x01;
    	 	   Mensagem.DADO[0] = OutputStsRead();

    	 	   Mensagem.ACK = 0x00;
    	 	   SendDisplay(); // Envia mensagem para o Display
               break;
     // Envia Setpoint de corrente atual
     case 0x16:
			   Mensagem.CMD = 0x16;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x04;
			   floatNchars.f = ISetpointRead();
			   Mensagem.DADO[0] = floatNchars.c[0];
			   Mensagem.DADO[1] = floatNchars.c[1];
			   Mensagem.DADO[2] = floatNchars.c[2];
			   Mensagem.DADO[3] = floatNchars.c[3];
			   Mensagem.ACK = 0x00;

			   SendDisplay(); // Envia mensagem para o Display
               break;
     // Envia o valor do ganho proporcional atual da malha de controle
     case 0x17:
    	       Mensagem.CMD = 0x17;
    	 	   Mensagem.PDADO = 0x00;
    	 	   Mensagem.NDADO = 0x04;
    	 	   floatNchars.f = KpRead();
    	 	   Mensagem.DADO[0] = floatNchars.c[0];
    	 	   Mensagem.DADO[1] = floatNchars.c[1];
    	 	   Mensagem.DADO[2] = floatNchars.c[2];
    	 	   Mensagem.DADO[3] = floatNchars.c[3];
    	       Mensagem.ACK = 0x00;

    	       SendDisplay(); // Envia mensagem para o Display
               break;
     // Envia o valor do ganho integral atual da malha de controle
     case 0x18:
    	       Mensagem.CMD = 0x18;
    	       Mensagem.PDADO = 0x00;
    	       Mensagem.NDADO = 0x04;
			   floatNchars.f = KiRead();
			   Mensagem.DADO[0] = floatNchars.c[0];
			   Mensagem.DADO[1] = floatNchars.c[1];
			   Mensagem.DADO[2] = floatNchars.c[2];
			   Mensagem.DADO[3] = floatNchars.c[3];
			   Mensagem.ACK = 0x00;

			   SendDisplay(); // Envia mensagem para o Display
               break;
	 // Envia Setpoint de tensão atual
	 case 0x19:
			   Mensagem.CMD = 0x19;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x04;
			   floatNchars.f = ISetpointRead();
			   Mensagem.DADO[0] = floatNchars.c[0];
			   Mensagem.DADO[1] = floatNchars.c[1];
			   Mensagem.DADO[2] = floatNchars.c[2];
			   Mensagem.DADO[3] = floatNchars.c[3];
			   Mensagem.ACK = 0x00;

			   SendDisplay(); // Envia mensagem para o Display
			   break;
	 // Envia o valor do ganho proporcional atual da malha de tensão
	 case 0x1A:
			   Mensagem.CMD = 0x1A;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x04;
			   floatNchars.f = KpRead();
			   Mensagem.DADO[0] = floatNchars.c[0];
			   Mensagem.DADO[1] = floatNchars.c[1];
			   Mensagem.DADO[2] = floatNchars.c[2];
			   Mensagem.DADO[3] = floatNchars.c[3];
			   Mensagem.ACK = 0x00;

			   SendDisplay(); // Envia mensagem para o Display
			   break;
	 // Envia o valor do ganho integral atual da malha de tensão
	 case 0x1B:
			   Mensagem.CMD = 0x1B;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x04;
			   floatNchars.f = KiRead();
			   Mensagem.DADO[0] = floatNchars.c[0];
			   Mensagem.DADO[1] = floatNchars.c[1];
			   Mensagem.DADO[2] = floatNchars.c[2];
			   Mensagem.DADO[3] = floatNchars.c[3];
			   Mensagem.ACK = 0x00;

			   SendDisplay(); // Envia mensagem para o Display
			   break;
	 // Envia o valor do ganho proporcional atual da malha de corrente
	 case 0x1C:
			   Mensagem.CMD = 0x1C;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x04;
			   floatNchars.f = KpRead();
			   Mensagem.DADO[0] = floatNchars.c[0];
			   Mensagem.DADO[1] = floatNchars.c[1];
			   Mensagem.DADO[2] = floatNchars.c[2];
			   Mensagem.DADO[3] = floatNchars.c[3];
			   Mensagem.ACK = 0x00;

			   SendDisplay(); // Envia mensagem para o Display
			   break;
	 // Envia o valor do ganho integral atual da malha de corrente
	 case 0x1D:
			   Mensagem.CMD = 0x1D;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x04;
			   floatNchars.f = KiRead();
			   Mensagem.DADO[0] = floatNchars.c[0];
			   Mensagem.DADO[1] = floatNchars.c[1];
			   Mensagem.DADO[2] = floatNchars.c[2];
			   Mensagem.DADO[3] = floatNchars.c[3];
			   Mensagem.ACK = 0x00;

			   SendDisplay(); // Envia mensagem para o Display
			   break;
     // Comandos de atuação
     // Seta corrente de saída
	 case 0x20:
		       if(LocRem) // Testa se está em Local(0x01)
		       {
		    	   floatNchars.c[0] = Mensagem.DADO[0];
				   floatNchars.c[1] = Mensagem.DADO[1];
				   floatNchars.c[2] = Mensagem.DADO[2];
				   floatNchars.c[3] = Mensagem.DADO[3];

				   //SoftStarterIRef(floatNchars.f);
				   ISetpointWrite(floatNchars.f);

		       }
		 	   break;

	 // Seta tensão de saída
	 case 0x21:
			   if(LocRem) // Testa se está em Local(0x01)
			   {
				   floatNchars.c[0] = Mensagem.DADO[0];
				   floatNchars.c[1] = Mensagem.DADO[1];
				   floatNchars.c[2] = Mensagem.DADO[2];
				   floatNchars.c[3] = Mensagem.DADO[3];

				   //SoftStarterIRef(floatNchars.f);
				   ISetpointWrite(floatNchars.f);

			   }
			   break;

     // Liga/desliga saida da fonte
	 case 0x22:
		 	   if(LocRem) // Testa se está em Local(0x01)
		 	   {
		 		   //ShmSetStatusFonteOp(Mensagem.DADO[0]);

		 		   OutputStsWrite(Mensagem.DADO[0]);

		 	   }
    	 	   // Retornar ACK para PIC32 sinalizando que a tarefa foi executada com sucesso
			   break;

     // Liga/desliga malha de realimentação da Fonte
	 case 0x23:
		 	   if(LocRem) // Testa se está em Local(0x01)
			   {

		 		  CtrlLoopWrite(Mensagem.DADO[0]);
			   }
			   break;

     // Altera o Status do ajuste para Local ou Remoto
	 case 0x24:
		 	   LocRem = Mensagem.DADO[0];
			   break;

	 // Altera o ganho proporcional da malha de controle "Kp"
	 case 0x27:
		 	   if(LocRem) // Testa se está em Local(0x01)
			   {
		 		   floatNchars.c[0] = Mensagem.DADO[0];
				   floatNchars.c[1] = Mensagem.DADO[1];
				   floatNchars.c[2] = Mensagem.DADO[2];
				   floatNchars.c[3] = Mensagem.DADO[3];

				   KpWrite(floatNchars.f);

				   //ShmSetControlKp(floatNchars.f);
			   }
		 	   break;
	 //	Altera o ganho integral da malha de controle "Ki"
	 case 0x28:
		 	   if(LocRem) // Testa se está em Local(0x01)
		 	   {
		 		   floatNchars.c[0] = Mensagem.DADO[0];
				   floatNchars.c[1] = Mensagem.DADO[1];
				   floatNchars.c[2] = Mensagem.DADO[2];
				   floatNchars.c[3] = Mensagem.DADO[3];

				   KiWrite(floatNchars.f);

				   //ShmSetControlKi(floatNchars.f);
		 	   }
		 	   break;
	 // Altera o ganho proporcional da malha de tensão "Kp"
	 case 0x29:
			   if(LocRem) // Testa se está em Local(0x01)
			   {
				   floatNchars.c[0] = Mensagem.DADO[0];
				   floatNchars.c[1] = Mensagem.DADO[1];
				   floatNchars.c[2] = Mensagem.DADO[2];
				   floatNchars.c[3] = Mensagem.DADO[3];

				   KpWrite(floatNchars.f);

				   //ShmSetControlKp(floatNchars.f);
			   }
			   break;
	 //	Altera o ganho integral da malha de tensão "Ki"
	 case 0x2A:
			   if(LocRem) // Testa se está em Local(0x01)
			   {
				   floatNchars.c[0] = Mensagem.DADO[0];
				   floatNchars.c[1] = Mensagem.DADO[1];
				   floatNchars.c[2] = Mensagem.DADO[2];
				   floatNchars.c[3] = Mensagem.DADO[3];

				   KiWrite(floatNchars.f);

				   //ShmSetControlKi(floatNchars.f);
			   }
			   break;
	 // Altera o ganho proporcional da malha de corrente "Kp"
	 case 0x2B:
			   if(LocRem) // Testa se está em Local(0x01)
			   {
				   floatNchars.c[0] = Mensagem.DADO[0];
				   floatNchars.c[1] = Mensagem.DADO[1];
				   floatNchars.c[2] = Mensagem.DADO[2];
				   floatNchars.c[3] = Mensagem.DADO[3];

				   KpWrite(floatNchars.f);

				   //ShmSetControlKp(floatNchars.f);
			   }
			   break;
	 //	Altera o ganho integral da malha de corrente "Ki"
	 case 0x2C:
			   if(LocRem) // Testa se está em Local(0x01)
			   {
				   floatNchars.c[0] = Mensagem.DADO[0];
				   floatNchars.c[1] = Mensagem.DADO[1];
				   floatNchars.c[2] = Mensagem.DADO[2];
				   floatNchars.c[3] = Mensagem.DADO[3];

				   KiWrite(floatNchars.f);

				   //ShmSetControlKi(floatNchars.f);
			   }
			   break;

     // Comandos de alteração de parâmetro - essas funções são utilizadas para tratar o ACK
     // Os valores recebidos devem ser ajustados
     // Endereço RS-485
     case 0x30:

    	 	   if(LocRem) // Testa se está em Local(0x01)
    	 	   {
    	 		   SetRS485Address(Mensagem.DADO[0]);
    	 	   }

               // Chama subrotina para salvar o novo dado na memória não volátil
               // Retornar ACK para PIC32 sinalizando que a tarefa foi executada com sucesso
               break;
     // Data e hora
     case 0x32:
    	 	   /*if(Parametros.LocRem) // Testa se está em Local(0x01)
    	 	   {
    	 		   Rtc.RTCano = Mensagem.DADO[0];
				   Rtc.RTCmes = Mensagem.DADO[1];
				   Rtc.RTCdia = Mensagem.DADO[2];
				   Rtc.RTChora = Mensagem.DADO[3];
				   Rtc.RTCmin = Mensagem.DADO[4];
				   Rtc.RTCseg = Mensagem.DADO[5];
				   // Chamar subrotina de ajuste dos dados no RTC
				   Write_Rtc_Clock();
				   // Retornar ACK para PIC32 sinalizando que a tarefa foi executada com sucesso
    	 	   }*/
               break;
     // Valor PI
     case 0x33:

               // Chama subrotina para salvar o novo dado na memória FLASH
               // Retornar ACK para PIC32 sinalizando que a tarefa foi executada com sucesso
               break;
     // Configuração do interlock
     case 0x34:

               break;
     // Configuração do alarme
     case 0x35:

               break;
     //
     case 0x36:

    	 	  // Retornar ACK para PIC32 sinalizando que a tarefa foi executada com sucesso
               break;
     // Salva nova Senha
     case 0x37:
    	       //Parametros.Senha = Mensagem.DADO[0];
    	       mani2 = Mensagem.DADO[1];
    	       mani3 = Mensagem.DADO[2];
    	       //Parametros.Senha = Parametros.Senha << 8;
    	       //Parametros.Senha |= mani2;
    	       //Parametros.Senha = Parametros.Senha << 8;
    	       //Parametros.Senha |= mani3;
    	       // Chamar função que salva a nova senha na memória FLASH
    	       // Retornar ACK para PIC32 sinalizando que a tarefa foi executada com sucesso
               break;
     // Salva novo ajuste de setpoint para ADC
     case 0x38:
    	       /*Parametros.SetPointAn1 = Mensagem.DADO[0];
			   Parametros.SetPointAn1 = Parametros.SetPointAn1 << 8;
			   Parametros.SetPointAn1 |= Mensagem.DADO[1];

			   Parametros.SetPointAn2 = Mensagem.DADO[2];
			   Parametros.SetPointAn2 = Parametros.SetPointAn2 << 8;
			   Parametros.SetPointAn2 |= Mensagem.DADO[3];

			   Parametros.SetPointAn3 = Mensagem.DADO[4];
			   Parametros.SetPointAn3 = Parametros.SetPointAn3 << 8;
			   Parametros.SetPointAn3 |= Mensagem.DADO[5];

			   Parametros.SetPointAn4 = Mensagem.DADO[6];
			   Parametros.SetPointAn4 = Parametros.SetPointAn4 << 8;
			   Parametros.SetPointAn4 |= Mensagem.DADO[7];

			   Parametros.SetPointAn5 = Mensagem.DADO[8];
			   Parametros.SetPointAn5 = Parametros.SetPointAn5 << 8;
			   Parametros.SetPointAn5 |= Mensagem.DADO[9];

			   Parametros.SetPointAn6 = Mensagem.DADO[10];
			   Parametros.SetPointAn6 = Parametros.SetPointAn6 << 8;
			   Parametros.SetPointAn6 |= Mensagem.DADO[11];

			   Parametros.SetPointAn7 = Mensagem.DADO[12];
			   Parametros.SetPointAn7 = Parametros.SetPointAn4 << 8;
			   Parametros.SetPointAn7 |= Mensagem.DADO[13];

			   Parametros.SetPointAn8 = Mensagem.DADO[14];
			   Parametros.SetPointAn8 = Parametros.SetPointAn8 << 8;
			   Parametros.SetPointAn8 |= Mensagem.DADO[15];*/
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
     // Envia o valor de tensão do DC link em 12 bits
     case 0x60:
    	       Mensagem.CMD = 0x60;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x04;

			   //floatNchars.f = LeituraVarDin.Vin;
			   Mensagem.DADO[0] = floatNchars.c[0];
			   Mensagem.DADO[1] = floatNchars.c[1];
			   Mensagem.DADO[2] = floatNchars.c[2];
			   Mensagem.DADO[3] = floatNchars.c[3];

			   Mensagem.ACK = 0x00;

			   SendDisplay(); // Envia mensagem para o Display
    	 	   break;
	 // Envia o valor de tensão na carga em 12 bits
	 case 0x61:
		 	   Mensagem.CMD = 0x61;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x04;

			   //floatNchars.f = LeituraVarDin.Vout;
			   Mensagem.DADO[0] = floatNchars.c[0];
			   Mensagem.DADO[1] = floatNchars.c[1];
			   Mensagem.DADO[2] = floatNchars.c[2];
			   Mensagem.DADO[3] = floatNchars.c[3];

			   Mensagem.ACK = 0x00;

			   SendDisplay(); // Envia mensagem para o Display
			   break;
	 // Envia o valor da temperatura no dissipador do módulo de potencia em 8 bits
	 case 0x62:
		 	   Mensagem.CMD = 0x62;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x01;
			   //Mensagem.DADO[0] = LeituraVarDin.TempDig;
			   Mensagem.DADO[0] = 47;
			   Mensagem.ACK = 0x00;

			   SendDisplay(); // Envia mensagem para o Display
			   break;
	 // Envia o valor lido no ch3 do conversor AD
	 case 0x63:

			   break;
	 // Envia o valor lido no ch4 do conversor AD
	 case 0x64:

			   break;
	 // Envia o valor lido no ch5 do conversor AD
     case 0x65:

			   break;
     // Envia o valor lido no ch6 do conversor AD
	 case 0x66:

			   break;
	 // Envia o valor lido no ch7 do conversor AD
     case 0x67:

			   break;
	 // Envia o endereço IP
     case 0x70:
    	 	   Mensagem.CMD = 0x70;
    	 	   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x04;

			   IPAddressRead(&Mensagem.DADO[0], &Mensagem.DADO[1], &Mensagem.DADO[2], &Mensagem.DADO[3]);

			   Mensagem.ACK = 0x00;

    	 	   SendDisplay(); // Envia mensagem para o Display
    	 	   break;

     case 0x71:

			   IPAddressWrite(Mensagem.DADO[0], Mensagem.DADO[1], Mensagem.DADO[2], Mensagem.DADO[3]);

			   break;

     // Recebe os dados de ajuste de frequencia e amplitude do gerador de frequencia
     case 0x80:
    	 	   /*if(Parametros.LocRem) // Testa se está em Local(0x01)
    	 	   {
    	 		   // Frequencia
				   uInt = Mensagem.DADO[0];
				   uInt = uInt << 8;
				   uInt |= Mensagem.DADO[1];

				   // Amplitude
				   floatNchars.c[0] = Mensagem.DADO[4];
				   floatNchars.c[1] = Mensagem.DADO[5];
				   floatNchars.c[2] = Mensagem.DADO[6];
				   floatNchars.c[3] = Mensagem.DADO[7];

				   ShmSetFreqGen(Var, floatNchars.f);
    	 	   }*/
    	 	   break;
     // Liga/desliga o gerador de frequencia
     case 0x81:
    	 	   /*if(Parametros.LocRem) // Testa se está em Local(0x01)
    	 	   {
    	 		   ShmSetStsFreqGen(Mensagem.DADO[0]);
    	 	   }*/
    	 	   break;
     // Liga/desliga a função "Sweep" para levantar a resposta em frequencia da fonte
     case 0x82:
    	 	   /*if(Parametros.LocRem) // Testa se está em Local(0x01)
    	 	   {
    	 		   if(Mensagem.DADO[0] && !LeituraVarDin.RunningFunction)InitSweep();
				   else if(!Mensagem.DADO[0]) StopSweep();
    	 	   }*/
    	 	   break;
     // Obtem o status e a frequencia atual do gerador de sweep
     case 0x83:
    	       Mensagem.CMD = 0x83;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x03;

			   //Mensagem.DADO[0] = ShmGetStsFreqGen();

    	 	   //uInt = ShmGetFreqSet();

    	 	   Mensagem.DADO[1] = uInt >> 8;
    	 	   Mensagem.DADO[2] = uInt;

			   Mensagem.ACK = 0x00;
			   SendDisplay(); // Envia mensagem para o Display

    	 	   break;
     // Liga/desliga a função para teste de linearidade
     case 0x85:
    	 	   /*if(Parametros.LocRem) // Testa se está em Local(0x01)
    	 	   {
    	 		   if(Mensagem.DADO[0] && !LeituraVarDin.RunningFunction)SoftStarterStartRamp();
    	 		   else if(!Mensagem.DADO[0]) SoftStarterStopRamp();
    	 	   }*/

    	 	   break;
     // Envia o status de funcionamento da função linearidade
     case 0x86:
    	       Mensagem.CMD = 0x86;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x01;

			   //Mensagem.DADO[0] = LeituraVarDin.Ramp1h;

			   Mensagem.ACK = 0x00;
			   SendDisplay(); // Envia mensagem para o Display
    	 	   break;

     // Recebe a configuração de amplitude do Step para o ensaio de resposta ao degrau
     case 0x90:
    	 	   /*if(Parametros.LocRem) // Testa se está em Local(0x01)
			   {
    	 		   ConfigStepResponse(Mensagem.DADO[0]);
			   }*/
    	 	   break;

     // Recebe o comando para ligar/desligar o ensaio de resposta ao degrau
     case 0x91:
    	 	   /*if(Parametros.LocRem) // Testa se está em Local(0x01)
			   {
				   if(Mensagem.DADO[0] && !LeituraVarDin.RunningFunction) InitStepResponse();
				   else if(!Mensagem.DADO[0]) StopTimerStepResponse();
			   }*/
		       break;

	 // Envia os dados de status do ensaio de resposta ao degrau
	 case 0x92:
		 	   Mensagem.CMD = 0x92;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x02;

			   //Mensagem.DADO[0] = LeituraVarDin.RunningStepResponse;
			   //Mensagem.DADO[1] = ReadConfigStepResponse();

			   Mensagem.ACK = 0x00;
			   SendDisplay(); // Envia mensagem para o Display
			   break;

	 // Envia os dados referentes ao status do alarme
	 case 0xA0:


		   	   break;

	 // Reseta interlock e alarme
     case 0xC0:
    	 	   InterlockAlarmReset();
    	       break;

	 // Envia os dados referentes ao status do interlock
	 case 0xE0:
		 	   Mensagem.CMD = 0xE0;
			   Mensagem.PDADO = 0x00;
			   Mensagem.NDADO = 0x04;

			   Mensagem.DADO[0] = InterlockSts(0);
			   Mensagem.DADO[1] = InterlockSts(1);
			   Mensagem.DADO[2] = InterlockSts(2);
			   Mensagem.DADO[3] = InterlockSts(3);

			   Mensagem.ACK = 0x00;
			   SendDisplay(); // Envia mensagem para o Display

			   break;

     default:

    	 	   break;


     }

}



void
IHMIntHandler(void)
{
    unsigned long ulStatus;

    // Get the interrrupt status.
    ulStatus = UARTIntStatus(DISPLAY_UART_BASE, true);

    if(0x00000040 == ulStatus)
    {
    	NewData = 1;
    	// Loop while there are characters in the receive FIFO.
    	while(UARTCharsAvail(DISPLAY_UART_BASE) && Dado.counter < SERIAL_BUF_SIZE)
    	{
    		Dado.buffer_rx[Dado.counter] = UARTCharGet(DISPLAY_UART_BASE);
    		Dado.csum += Dado.buffer_rx[Dado.counter++];
    	}

    }

    // Clear the asserted interrupts.
    UARTIntClear(DISPLAY_UART_BASE, ulStatus);

}

void
DisplayProcessData(void)
{
	if(NewData)
	{
		// Checksum is not zero
		if(Dado.csum)
			goto exit;

		SeparaDado(); // Chama sub rotina que tira os dados do buffer e aloca na estrutura
		ProcessCmd(); // Chama Sub rotina para interpretação dos dados recebidos

	exit:
		Dado.counter = 0;
		Dado.csum  = 0;

		// Clear new data flag
		NewData = 0;
		//GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, OFF);
	}
}

void
InitDisplay(void)
{
	// Configura UART0 com baud de 8Mbps, operação 8-N-1 devido as limitações do conversor usb/serial controle
	UARTConfigSetExpClk(DISPLAY_UART_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 1000000,
						(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
						UART_CONFIG_PAR_NONE));

	UARTFIFOEnable(DISPLAY_UART_BASE);

	//Habilita interrupção pela UART (RS-485 BKP)
	IntRegister(DISPLAY_INT, IHMIntHandler);
	UARTIntEnable(DISPLAY_UART_BASE, UART_INT_RX | UART_INT_RT);

	//Seta níveis de prioridade entre as interrupções
	IntPrioritySet(DISPLAY_INT, 2);

	IntEnable(DISPLAY_INT);
}

uint8_t LocRemUpdate(void)
{
	return LocRem;
}
