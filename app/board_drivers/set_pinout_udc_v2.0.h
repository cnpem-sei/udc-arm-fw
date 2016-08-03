//###########################################################################
// FILE:   set_pinout_f28m35x.h
// TITLE:  GPIO setup for the F28M35x controlCARD.
//###########################################################################
// $TI Release: F28M36x Support Library v202 $
// $Release Date: Tue Apr  8 12:36:34 CDT 2014 $
//###########################################################################

#ifndef __SET_PINOUT_H__
#define __SET_PINOUT_H__

//*****************************************************************************
// Define board model
//*****************************************************************************


//*****************************************************************************
// SDCard SSI port
//*****************************************************************************
#define SDC_SSI_BASE            SSI3_BASE
#define SDC_SSI_SYSCTL_PERIPH   SYSCTL_PERIPH_SSI3

//*****************************************************************************
// GPIO for SDCard SSI pins
//*****************************************************************************
#define SDC_GPIO_PORT_BASE      GPIO_PORTR_BASE
#define SDC_GPIO_SYSCTL_PERIPH  SYSCTL_PERIPH_GPIOR
#define SDC_SSI_CLK             GPIO_PIN_2
#define SDC_SSI_TX              GPIO_PIN_0
#define SDC_SSI_RX              GPIO_PIN_1

#define SDC_SSI_PINS            (SDC_SSI_TX | SDC_SSI_RX | SDC_SSI_CLK)

//*****************************************************************************
// GPIO for the SDCard chip select
//*****************************************************************************
#define SDCARD_CS_PERIPH   SYSCTL_PERIPH_GPIOR
#define SDCARD_CS_BASE     GPIO_PORTR_BASE
#define SDCARD_CS_PIN      GPIO_PIN_3

//*****************************************************************************
// GPIO for the Operation LED
//*****************************************************************************
#define ON					~0
#define OFF					0
#define LED_OP_BASE         GPIO_PORTP_BASE
#define LED_OP_PIN          GPIO_PIN_5

//*****************************************************************************
// GPIO for the Debug Pin CON9 GPIO0
//*****************************************************************************
#define DEBUG_BASE         GPIO_PORTG_BASE
#define DEBUG_PIN          GPIO_PIN_6

//*****************************************************************************
// MACROS for onboard I2C
//*****************************************************************************
#define	I2C_ONBOARD__MASTER_BASE	I2C0_MASTER_BASE
#define I2C_ONBOARD_BASE	  		GPIO_PORTN_BASE
#define I2C_ONBOARD_PINS 			(GPIO_PIN_0 | GPIO_PIN_1)
#define I2C_ONBOARD_SCL				GPIO_PN0_I2C0SCL
#define I2C_ONBOARD_SDA 			GPIO_PN1_I2C0SDA
#define I2C_ONBOARD_SYSCTL			SYSCTL_PERIPH_I2C0

//*****************************************************************************
// MACROS for offboard I2C
//*****************************************************************************
#define	I2C_OFFBOARD__MASTER_BASE	I2C1_MASTER_BASE
#define I2C_OFFBOARD_BASE			GPIO_PORTP_BASE
#define I2C_OFFBOARD_PINS 			(GPIO_PIN_0 | GPIO_PIN_1)
#define I2C_OFFBOARD_SCL			GPIO_PP0_I2C1SCL
#define I2C_OFFBOARD_SDA 			GPIO_PP1_I2C1SDA
#define I2C_OFFBOARD_SYSCTL			SYSCTL_PERIPH_I2C1

//*****************************************************************************
// MACROS for RS-485 communication
//*****************************************************************************
#define RS485_BASE			GPIO_PORTN_BASE
#define	RS485_PINS			(GPIO_PIN_2 | GPIO_PIN_3)
#define RS485_RX			GPIO_PN2_U1RX
#define	RS485_TX			GPIO_PN3_U1TX
#define RS485_SYSCTL       	SYSCTL_PERIPH_UART1

#define RS485_RD_BASE		GPIO_PORTP_BASE
#define RS485_RD_PIN		GPIO_PIN_2

#define RS485_UART_BASE		UART1_BASE
#define RS485_INT			INT_UART1

//*****************************************************************************
// MACROS for DISPLAY communication
//*****************************************************************************
#define DISPLAY_BASE		GPIO_PORTN_BASE
#define	DISPLAY_PINS		(GPIO_PIN_4 | GPIO_PIN_5)
#define DISPLAY_RX			GPIO_PN5_U3RX
#define	DISPLAY_TX			GPIO_PN4_U3TX
#define DISPLAY_SYSCTL    	SYSCTL_PERIPH_UART3

#define DISPLAY_UART_BASE	UART3_BASE
#define DISPLAY_INT			INT_UART3

//*****************************************************************************
// MACROS for RS-485 BACKPLANE communication
//*****************************************************************************
#define RS485_BKP_BASE		GPIO_PORTN_BASE
#define	RS485_BKP_PINS		(GPIO_PIN_6 | GPIO_PIN_7)
#define RS485_BKP_RX		GPIO_PN6_U4RX
#define	RS485_BKP_TX		GPIO_PN7_U4TX
#define RS485_BKP_SYSCTL    SYSCTL_PERIPH_UART4

#define RS485_BKP_RD_BASE	GPIO_PORTP_BASE
#define RS485_BKP_RD_PIN	GPIO_PIN_3

#define RS485_BKP_UART_BASE		UART4_BASE
#define RS485_BKP_INT			INT_UART4

//*****************************************************************************
// MACROS for WP EEPROM
//*****************************************************************************
#define EEPROM_WP_BASE		GPIO_PORTP_BASE
#define EEPROM_WP_PIN		GPIO_PIN_4

//*****************************************************************************
// MACROS for INT ARM
//*****************************************************************************
#define INT_ARM_BASE		GPIO_PORTE_BASE
#define	INT_ARM_PIN			GPIO_PIN_4

//*****************************************************************************
// MACROS for RTC TIMER
//*****************************************************************************
#define RTC_TIMER_BASE		GPIO_PORTG_BASE
#define RTC_TIMER_PIN		GPIO_PIN_3

//*****************************************************************************
// MACROS for CAN communication
//*****************************************************************************
#define CAN_BASE			GPIO_PORTE_BASE
#define CAN_PINS			(GPIO_PIN_6 | GPIO_PIN_7)
#define CAN_RX				GPIO_PE6_CAN0RX
#define	CAN_TX				GPIO_PE7_CAN0TX

//*****************************************************************************
// MACROS for USB communication
//*****************************************************************************
#define USB_5V_BASE			GPIO_PORTF_BASE
#define USB_5V_PIN			GPIO_PIN_6

#define USB_PN_BASE			GPIO_PORTG_BASE
#define USB_PN_PINS			(GPIO_PIN_2 | GPIO_PIN_5)

//*****************************************************************************
// MACROS for ADCP communication
//*****************************************************************************
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

//*****************************************************************************
// MACROS for FLASH MEMORY communication
//*****************************************************************************
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


//*****************************************************************************
//
// Public function prototypes.
//
//*****************************************************************************
extern void PinoutSet(void);


#endif // __SET_PINOUT_H__
