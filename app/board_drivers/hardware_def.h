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
 * @file hardware_def.h
 * @brief Brief description of module
 *
 * Enable basic peripherals for UDC board.
 *
 * @author joao.rosa
 * @date 13/01/2017
 *
 */

#include <stdint.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/gpio.h"

#ifndef HARDWARE_DEF_H_
#define HARDWARE_DEF_H_


//#define HARDWARE_VERSION  0x20
#define HARDWARE_VERSION    0x21

/**
 * Select DRS boards version
 */
#define UDC_V2_0    0
#define UDC_V2_1    1

#define HRADC_v2_0  0
#define HRADC_v2_1  1

/**
 * Core selector for some initialization
 */
#define M3_MASTER   0
#define C28_MASTER  1

/******************************************************************************
 * GPIO for the operation LED
 *****************************************************************************/
#define ON					~0
#define OFF					0
#define LED_OP_BASE         GPIO_PORTP_BASE
#define LED_OP_PIN          GPIO_PIN_5

/******************************************************************************
 * SDCard SSI port
 *****************************************************************************/
#define SDC_SSI_BASE            SSI3_BASE
#define SDC_SSI_SYSCTL_PERIPH   SYSCTL_PERIPH_SSI3

/******************************************************************************
 * GPIO for SDCard SSI pins
 *****************************************************************************/
#define SDC_GPIO_PORT_BASE      GPIO_PORTR_BASE
#define SDC_GPIO_SYSCTL_PERIPH  SYSCTL_PERIPH_GPIOR
#define SDC_SSI_CLK             GPIO_PIN_2
#define SDC_SSI_TX              GPIO_PIN_0
#define SDC_SSI_RX              GPIO_PIN_1

#define SDC_SSI_PINS            (SDC_SSI_TX | SDC_SSI_RX | SDC_SSI_CLK)

/******************************************************************************
 * GPIO for the SDCard chip select
 *****************************************************************************/
#define SDCARD_CS_PERIPH   SYSCTL_PERIPH_GPIOR
#define SDCARD_CS_BASE     GPIO_PORTR_BASE
#define SDCARD_CS_PIN      GPIO_PIN_3

/******************************************************************************
 * GPIO for the debug pin CON9 GPIO0
 *****************************************************************************/
#define DEBUG_GPIO0_CORE    GPIO_PIN_M_CORE_SELECT // GPIO_PIN_C_CORE_SELECT
#define DEBUG_GPIO1_CORE    GPIO_PIN_C_CORE_SELECT // GPIO_PIN_M_CORE_SELECT

#define DEBUG_GPIO0_BASE    GPIO_PORTG_BASE
#define DEBUG_GPIO0_PIN     GPIO_PIN_6

#define DEBUG_GPIO1_BASE    GPIO_PORTP_BASE
#define DEBUG_GPIO1_PIN     GPIO_PIN_7

#define SET_DEBUG_GPIO0     GPIOPinWrite(DEBUG_GPIO0_BASE, DEBUG_GPIO0_PIN, ON);
#define CLEAR_DEBUG_GPIO0   GPIOPinWrite(DEBUG_GPIO0_BASE, DEBUG_GPIO0_PIN, OFF);

#define SET_DEBUG_GPIO1     GPIOPinWrite(DEBUG_GPIO1_BASE, DEBUG_GPIO1_PIN, ON);
#define CLEAR_DEBUG_GPIO1   GPIOPinWrite(DEBUG_GPIO1_BASE, DEBUG_GPIO1_PIN, OFF);

/******************************************************************************
 * Macros for onboard I2C
 *****************************************************************************/
#define	I2C_ONBOARD_MASTER_BASE		I2C0_MASTER_BASE
#define I2C_ONBOARD_BASE	  		GPIO_PORTN_BASE
#define I2C_ONBOARD_PINS 			(GPIO_PIN_0 | GPIO_PIN_1)
#define I2C_ONBOARD_SCL				GPIO_PN0_I2C0SCL
#define I2C_ONBOARD_SDA 			GPIO_PN1_I2C0SDA
#define I2C_ONBOARD_SYSCTL			SYSCTL_PERIPH_I2C0

/******************************************************************************
 * Macros for WP EEPROM
 *****************************************************************************/
#define EEPROM_WP_BASE		GPIO_PORTP_BASE
#define EEPROM_WP_PIN		GPIO_PIN_4

/******************************************************************************
 * Macros for offboard I2C isolated
 *****************************************************************************/
#define	I2C_OFFBOARD_ISO_MASTER_BASE	I2C1_MASTER_BASE
#define I2C_OFFBOARD_ISO_BASE			GPIO_PORTP_BASE
#define I2C_OFFBOARD_ISO_PINS 			(GPIO_PIN_0 | GPIO_PIN_1)
#define I2C_OFFBOARD_ISO_SCL			GPIO_PP0_I2C1SCL
#define I2C_OFFBOARD_ISO_SDA 			GPIO_PP1_I2C1SDA
#define I2C_OFFBOARD_ISO_SYSCTL			SYSCTL_PERIPH_I2C1

/******************************************************************************
 * Macros for RS-485 communication
 *****************************************************************************/
#define RS485_BASE			GPIO_PORTN_BASE
#define	RS485_PINS			(GPIO_PIN_2 | GPIO_PIN_3)
#define RS485_RX			GPIO_PN2_U1RX
#define	RS485_TX			GPIO_PN3_U1TX
#define RS485_SYSCTL       	SYSCTL_PERIPH_UART1

#define RS485_RD_BASE		GPIO_PORTP_BASE
#define RS485_RD_PIN		GPIO_PIN_2

#define RS485_UART_BASE		UART1_BASE
#define RS485_INT			INT_UART1

/******************************************************************************
 * Macros for RS-485 backplane communication
 *****************************************************************************/
#define RS485_BKP_BASE		GPIO_PORTN_BASE
#define	RS485_BKP_PINS		(GPIO_PIN_6 | GPIO_PIN_7)
#define RS485_BKP_RX		GPIO_PN6_U4RX
#define	RS485_BKP_TX		GPIO_PN7_U4TX
#define RS485_BKP_SYSCTL    SYSCTL_PERIPH_UART4

#define RS485_BKP_RD_BASE	GPIO_PORTP_BASE
#define RS485_BKP_RD_PIN	GPIO_PIN_3

#define RS485_BKP_UART_BASE		UART4_BASE
#define RS485_BKP_INT			INT_UART4

/******************************************************************************
 * Macros for display communication
 *****************************************************************************/
#define DISPLAY_BASE		GPIO_PORTN_BASE
#define	DISPLAY_PINS		(GPIO_PIN_4 | GPIO_PIN_5)
#define DISPLAY_RX			GPIO_PN5_U3RX
#define	DISPLAY_TX			GPIO_PN4_U3TX
#define DISPLAY_SYSCTL    	SYSCTL_PERIPH_UART3

#define DISPLAY_UART_BASE	UART3_BASE
#define DISPLAY_INT			INT_UART3

/******************************************************************************
 * Macros for UART FT230 communication (USB to Uart) (Hardware ver. 2.1)
 *****************************************************************************/
#define FT230_BASE			GPIO_PORTQ_BASE
#define	FT230_PINS			(GPIO_PIN_2 | GPIO_PIN_3)
#define FT230_RX			GPIO_PQ2_U0RX
#define	FT230_TX			GPIO_PQ3_U0TX
#define FT230_SYSCTL    	SYSCTL_PERIPH_UART0

#define FT230_UART_BASE		UART0_BASE
#define FT230_INT			INT_UART0

/******************************************************************************
 * Macros for ADCP communication
 *****************************************************************************/
#define ADCP_BASE			GPIO_PORTD_BASE
#define ADCP_SPI_PINS		(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)
#define ADCP_SPI_CLK		GPIO_PD2_SSI0CLK
#define ADCP_SPI_FSS		GPIO_PD3_SSI0FSS
#define ADCP_SPI_RX			GPIO_PD1_SSI0RX
#define ADCP_SPI_TX			GPIO_PD0_SSI0TX
#define	ADCP_SPI_SYSCTL		SYSCTL_PERIPH_SSI0

#define	ADCP_SPI_BASE		SSI0_BASE
#define	ADCP_SPI_TX_UDMA	UDMA_CHANNEL_SSI0TX
#define	ADCP_SPI_RX_UDMA	UDMA_CHANNEL_SSI0RX

#define ADCP_SPI_INT		INT_SSI0

/******************************************************************************
 * Macros for CAN communication
 *****************************************************************************/
#define CAN_BASE			GPIO_PORTE_BASE
#define CAN_PINS			(GPIO_PIN_6 | GPIO_PIN_7)
#define CAN_RX				GPIO_PE6_CAN0RX
#define	CAN_TX				GPIO_PE7_CAN0TX

/******************************************************************************
 * Macros for flash memory communication
 *****************************************************************************/
#define FLASH_MEM_E_BASE	GPIO_PORTE_BASE
#define FLASH_MEM_E_PINS	(GPIO_PIN_2 | GPIO_PIN_3)
#define FLASH_MEM_F_BASE	GPIO_PORTF_BASE
#define FLASH_MEM_F_PINS	(GPIO_PIN_2 | GPIO_PIN_3)
#define FLASH_MEM_CLK		GPIO_PF2_SSI1CLK
#define FLASH_MEM_FSS		GPIO_PF3_SSI1FSS
#define FLASH_MEM_RX		GPIO_PE2_SSI1RX
#define FLASH_MEM_TX		GPIO_PE3_SSI1TX
#define	FLASH_MEM_SYSCTL	SYSCTL_PERIPH_SSI1

#define	FLASH_MEM_BASE		SSI1_BASE

/******************************************************************************
 * Macros for int arm
 *****************************************************************************/
#define INT_ARM_BASE		GPIO_PORTE_BASE
#define	INT_ARM_PIN			GPIO_PIN_4

/******************************************************************************
 * Macros for RTC timer
 *****************************************************************************/
#define RTC_TIMER_BASE		GPIO_PORTG_BASE
#define RTC_TIMER_PIN		GPIO_PIN_3

/**
* @brief Perform pinout configurations
*
* Enable peripherals and perform pinout configurations for ARM core and C28
*
* @return void
*/
extern void pinout_setup();

#endif /* HARDWARE_DEF_H_ */
