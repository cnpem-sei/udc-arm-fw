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
 * @file hardware_def.c
 * @brief Hardware Abstraction Layer for UDC board.
 *
 * Enable basic peripherals for UDC board.
 *
 * @author joao.rosa
 *
 * @date 14/02/2017
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_epi.h"
#include "inc/hw_ssi.h"
#include "inc/hw_ethernet.h"
#include "inc/hw_udma.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/can.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/ethernet.h"
#include "driverlib/epi.h"
#include "driverlib/uart.h"
#include "driverlib/usb.h"
#include "driverlib/ethernet.h"
#include "driverlib/udma.h"

#include "hardware_def.h"

/**
* @brief Enable Concerto Peripherals
*
* This enables the basic peripherals for C28 and ARM operation.
*
* @return void
*/
static void enable_peripherals(void);

/**
* @brief Setup Port control to peripherals
*
* Configure pins for the basic peripherals for C28 and ARM.
*
* @return void
*/
static void port_control_setup(void);

/**
* @brief Setup pins control for each core
*
* Configure what pins are controlled by each core
*
* @return void
*/
static void pin_core_setup(void);


void pinout_setup(void)
{

    /************************************************************************
     * Enable GPIO clock
     ***********************************************************************/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOR);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOS);

	/***************************************************************************
	 * Unlock the register to change the commit register value for Port B pin 7.
	 * This disables the NMI functionality on the pin and allows other muxing
	 * options to be used.
	 **************************************************************************/
	HWREG(GPIO_PORTB_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;

	/***************************************************************************
	 * Write to commit register
	 **************************************************************************/
	HWREG(GPIO_PORTB_BASE+GPIO_O_CR) |= 0x000000FF;

	pin_core_setup();

	port_control_setup();

	enable_peripherals();
}

void enable_peripherals(void)
{

    /***************************************************************************
     * Disable clock for Watchdog timers.
     **************************************************************************/
    SysCtlPeripheralDisable(SYSCTL_PERIPH_WDOG1);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_WDOG0);

    /***************************************************************************
     * Enable clock for EPI0.
     **************************************************************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EPI0);

    /***************************************************************************
     * Enable clock for RS485.
     **************************************************************************/
    SysCtlPeripheralEnable(RS485_SYSCTL);
    SysCtlPeripheralEnable(DISPLAY_SYSCTL);
    SysCtlPeripheralEnable(RS485_BKP_SYSCTL);

    /***************************************************************************
     * Enable clock for FT230 if UDC is v2.1.
     **************************************************************************/
    if(HARDWARE_VERSION == 0x21) SysCtlPeripheralEnable(FT230_SYSCTL);

    /***************************************************************************
     * Enable clock for onboard I2C
     **************************************************************************/
    SysCtlPeripheralEnable(I2C_ONBOARD_SYSCTL);
    SysCtlPeripheralEnable(I2C_OFFBOARD_ISO_SYSCTL);

    /***************************************************************************
     * Enable clock for ADCP
     **************************************************************************/
    SysCtlPeripheralEnable(ADCP_SPI_SYSCTL);

    /***************************************************************************
     * Enable clock for SSI1
     **************************************************************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    /***************************************************************************
     * Enable clock for CAN0
     **************************************************************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    /***************************************************************************
     * Enable clock for uDMA
     **************************************************************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    /***************************************************************************
     * Enable clock for ethernet
     **************************************************************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH);

    /***************************************************************************
     * Enable clock for TIMER0
     **************************************************************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    /***************************************************************************
     * Enable clock for TIMER1
     **************************************************************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

}


static void port_control_setup(void)
{

    /**********************************************************************
     * UART Setup for RS-485
     *********************************************************************/
    GPIOPinTypeUART(RS485_BASE, RS485_PINS);
    GPIOPinConfigure(RS485_RX);
    GPIOPinConfigure(RS485_TX);

    /**********************************************************************
     * UART setup for display
     *********************************************************************/
    GPIOPinTypeUART(DISPLAY_BASE, DISPLAY_PINS);
    GPIOPinConfigure(DISPLAY_RX);
    GPIOPinConfigure(DISPLAY_TX);

    /**********************************************************************
     * UART Setup for backplane
     *********************************************************************/
    GPIOPinTypeUART(RS485_BKP_BASE, RS485_BKP_PINS);
    GPIOPinConfigure(RS485_BKP_RX);
    GPIOPinConfigure(RS485_BKP_TX);

    /**********************************************************************
     * I2C onboard setup
     *********************************************************************/
    GPIOPinTypeI2C(I2C_ONBOARD_BASE, I2C_ONBOARD_PINS);
    GPIOPinConfigure(I2C_ONBOARD_SCL);
    GPIOPinConfigure(I2C_ONBOARD_SDA);

    /**********************************************************************
     * I2C offboard setup
     *********************************************************************/
    GPIOPinTypeI2C(I2C_OFFBOARD_ISO_BASE, I2C_OFFBOARD_ISO_PINS);
    GPIOPinConfigure(I2C_OFFBOARD_ISO_SCL);
    GPIOPinConfigure(I2C_OFFBOARD_ISO_SDA);

    /**********************************************************************
     * ADCP setup
     *********************************************************************/
    GPIOPinTypeSSI(ADCP_BASE, ADCP_SPI_PINS);
    GPIOPinConfigure(ADCP_SPI_CLK);
    GPIOPinConfigure(ADCP_SPI_FSS);
    GPIOPinConfigure(ADCP_SPI_RX);
    GPIOPinConfigure(ADCP_SPI_TX);

    /**********************************************************************
     * SPI flash setup
     *********************************************************************/
    GPIOPinTypeSSI(GPIO_PORTE_BASE , GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_TYPE_STD_WPU);
    GPIOPinConfigure(GPIO_PF2_SSI1CLK);
    GPIOPinConfigure(GPIO_PF3_SSI1FSS);
    GPIOPinConfigure(GPIO_PE2_SSI1RX);
    GPIOPinConfigure(GPIO_PE3_SSI1TX);

    /**********************************************************************
     * CAN bus setup
     *********************************************************************/
    GPIOPinConfigure(CAN_RX);
    GPIOPinConfigure(CAN_TX);
    GPIOPinTypeCAN(CAN_BASE, CAN_PINS);

    /**********************************************************************
     * SSI SdCard setup
     *********************************************************************/
    GPIOPinTypeSSI(SDC_GPIO_PORT_BASE, SDC_SSI_TX | SDC_SSI_RX | SDC_SSI_CLK);
    GPIOPinTypeGPIOOutput(SDCARD_CS_BASE, SDCARD_CS_PIN);
    GPIOPadConfigSet(SDC_GPIO_PORT_BASE, SDC_SSI_PINS,
                     GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(SDCARD_CS_BASE, SDCARD_CS_PIN,
                     GPIO_PIN_TYPE_STD_WPU);
    GPIOPinConfigure(GPIO_PR0_SSI3TX);
    GPIOPinConfigure(GPIO_PR1_SSI3RX);
    GPIOPinConfigure(GPIO_PR2_SSI3CLK);
    GPIOPinWrite(SDCARD_CS_BASE, SDCARD_CS_PIN, SDCARD_CS_PIN);

    /**********************************************************************
     * Ethernet setup
     *********************************************************************/
    GPIODirModeSet(GPIO_PORTK_BASE,
                   GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7,
                   GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTK_BASE,
                     GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7,
                     GPIO_PIN_TYPE_STD);
    GPIOPinConfigure(GPIO_PK4_MIITXEN);
    GPIOPinConfigure(GPIO_PK5_MIITXCK);
    GPIOPinConfigure(GPIO_PK7_MIICRS);
    GPIODirModeSet(GPIO_PORTL_BASE,
                   GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                   GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                   GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTL_BASE,
                     GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                     GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                     GPIO_PIN_TYPE_STD);
    GPIOPinConfigure(GPIO_PL0_MIIRXD3);
    GPIOPinConfigure(GPIO_PL1_MIIRXD2);
    GPIOPinConfigure(GPIO_PL2_MIIRXD1);
    GPIOPinConfigure(GPIO_PL3_MIIRXD0);
    GPIOPinConfigure(GPIO_PL4_MIICOL);
    GPIOPinConfigure(GPIO_PL5_MIIPHYRSTN);
    GPIOPinConfigure(GPIO_PL6_MIIPHYINTRN);
    GPIOPinConfigure(GPIO_PL7_MIIMDC);

    GPIODirModeSet(GPIO_PORTM_BASE,
                   GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                   GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                   GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTM_BASE,
                     GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                     GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                     GPIO_PIN_TYPE_STD);
    GPIOPinConfigure(GPIO_PM0_MIIMDIO);
    GPIOPinConfigure(GPIO_PM1_MIITXD3);
    GPIOPinConfigure(GPIO_PM2_MIITXD2);
    GPIOPinConfigure(GPIO_PM3_MIITXD1);
    GPIOPinConfigure(GPIO_PM4_MIITXD0);
    GPIOPinConfigure(GPIO_PM5_MIIRXDV);
    GPIOPinConfigure(GPIO_PM6_MIIRXER);
    GPIOPinConfigure(GPIO_PM7_MIIRXCK);

    if(HARDWARE_VERSION == 0x20)
    {
        GPIOPinTypeGPIOOutput(LED_OP_BASE, LED_OP_PIN);
        GPIOPinWrite(LED_OP_BASE, LED_OP_PIN, OFF);
    }
    else if(HARDWARE_VERSION == 0x21)
    {
        //UART FT230
        GPIOPinTypeUART(FT230_BASE, FT230_PINS);
        GPIOPinConfigure(FT230_RX);
        GPIOPinConfigure(FT230_TX);

    }


    /**********************************************************************
     * RS485_RD setup
     *********************************************************************/
    GPIOPinTypeGPIOOutput(RS485_RD_BASE, RS485_RD_PIN); //RD RS485
    GPIOPinWrite(RS485_RD_BASE, RS485_RD_PIN, OFF);

    /**********************************************************************
     * Debug pin setup
     *********************************************************************/
    GPIOPinTypeGPIOOutput(DEBUG_BASE, DEBUG_PIN);
    GPIOPinWrite(DEBUG_BASE, DEBUG_PIN, ON);

    /**********************************************************************
     * Backplane RS-485 setup
     *********************************************************************/
    GPIOPinTypeGPIOOutput(RS485_BKP_RD_BASE, RS485_BKP_RD_PIN); //RD RS485 BACKPLANE
    GPIOPinWrite(RS485_BKP_RD_BASE, RS485_BKP_RD_PIN, OFF);

    /**********************************************************************
     * WP EEPROM setup
     *********************************************************************/
    GPIOPinTypeGPIOOutput(EEPROM_WP_BASE, EEPROM_WP_PIN); //WP EEPROM
    GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write ptrotection in to the EEPROM

    /**********************************************************************
     * GPIO P7 setup
     *********************************************************************/
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_7, OFF);

    /**********************************************************************
     * ARM interrupt setup
     *********************************************************************/
    GPIOPinTypeGPIOInput(INT_ARM_BASE, INT_ARM_PIN); //Int ARM

    /**********************************************************************
     * RTC timer setup
     *********************************************************************/
    GPIOPinTypeGPIOInput(RTC_TIMER_BASE, RTC_TIMER_PIN); // RTC TIMER

}

static void pin_core_setup(void)
{

    if(HARDWARE_VERSION == 0x20)
    {
        /**********************************************************************
         * Pins to be controlled by C28
         *********************************************************************/
        GPIOPinConfigureCoreSelect(GPIO_PORTA_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT); //GPIO0 ao GPIO7 sуo utilizados para os 8 ch de PWM (C28)
                                                                                   //GPIO0 - PA0 щ utilizado para o PWM_1A
                                                                                   //GPIO1 - PA1 щ utilizado para o PWM_1B
                                                                                   //GPIO2 - PA2 щ utilizado para o PWM_2A
                                                                                   //GPIO3 - PA3 щ utilizado para o PWM_2B
                                                                                   //GPIO4 - PA4 щ utilizado para o PWM_3A
                                                                                   //GPIO5 - PA5 щ utilizado para o PWM_3B
                                                                                   //GPIO6 - PA6 щ utilizado para o PWM_4A
                                                                                   //GPIO7 - PA7 щ utilizado para o PWM_4B

        GPIOPinConfigureCoreSelect(GPIO_PORTB_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT); //GPIO8 ao GPIO15 sуo utilizados para os 8 ch de PWM (C28)
                                                                                   //GPIO8 - PB0 щ utilizado para o PWM_5A
                                                                                   //GPIO9 - PB1 щ utilizado para o PWM_5B
                                                                                   //GPIO10 - PB2 щ utilizado para o PWM_6A
                                                                                   //GPIO11 - PB3 щ utilizado para o PWM_6B
                                                                                   //GPIO12 - PB4 щ utilizado para o PWM_7A
                                                                                   //GPIO13 - PB5 щ utilizado para o PWM_7B
                                                                                   //GPIO14 - PB6 щ utilizado para o PWM_8A
                                                                                   //GPIO15 - PB7 щ utilizado para o PWM_8B

        GPIOPinConfigureCoreSelect(GPIO_PORTD_BASE, 0xF0, GPIO_PIN_C_CORE_SELECT); //GPIO20 ao GPIO23 sуo utilizados para o MCBSP (C28)
                                                                                   //GPIO16 - PD0 щ utilizado para o SPI_ADC_MOSI_I (ARM)
                                                                                   //GPIO17 - PD1 щ utilizado para o SPI_ADC_MISO_I (ARM)
                                                                                   //GPIO18 - PD2 щ utilizado para o SPI_ADC_SCLK_I (ARM)
                                                                                   //GPIO19 - PD3 щ utilizado para o SPI_ADC_SPISTE_I (ARM)
                                                                                   //GPIO20 - PD4 щ utilizado para o MCBSP_MDX (C28)
                                                                                   //GPIO21 - PD5 щ utilizado para o MCBSP_MDR (C28)
                                                                                   //GPIO22 - PD6 щ utilizado para o MCBSP_MCLKX (C28)
                                                                                   //GPIO23 - PD7 щ utilizado para o MCBSP_MFSX (C28)

        GPIOPinConfigureCoreSelect(GPIO_PORTE_BASE, 0x20, GPIO_PIN_C_CORE_SELECT); //GPIO29 щ utilizado para a interrupчуo externa do C28 (C28)
                                                                                   //GPIO24 - PE0 щ utilizado para o SDRAM_AD8 (ARM)
                                                                                   //GPIO25 - PE1 щ utilizado para o SDRAM_AD9 (ARM)
                                                                                   //GPIO26 - PE2 щ utilizado para o SPI_FLASH_MISO (ARM)
                                                                                   //GPIO27 - PE3 щ utilizado para o SPI_FLASH_MOSI (ARM)
                                                                                   //GPIO28 - PE4 щ utilizado para o INT_ARM (ARM)
                                                                                   //GPIO29 - PE5 щ utilizado para o INT_C28 (C28)
                                                                                   //GPIO30 - PE6 щ utilizado para o CAN_RX (ARM)
                                                                                   //GPIO31 - PE7 щ utilizado para o CAN_TX (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTF_BASE, 0x83, GPIO_PIN_C_CORE_SELECT); //GPIO32, GPIO33 e GPIO39 sуo utilizados pelo C28 (C28)
                                                                                   //GPIO32 - PF0 щ utilizado para o EPWMSYNCI (C28)
                                                                                   //GPIO33 - PF1 щ utilizado para o EPWMSYNCO (C28)
                                                                                   //GPIO34 - PF2 щ utilizado para o SPI_FLASH_CLK (ARM)
                                                                                   //GPIO35 - PF3 щ utilizado para o SPI_FLASH_CS (ARM)
                                                                                   //GPIO36 - PF4 щ utilizado para o SDRAM_AD12 (ARM)
                                                                                   //GPIO37 - PF5 щ utilizado para o SDRAM_AD15 (ARM)
                                                                                   //GPIO38 - PF6 щ utilizado para o USB0_GPIO38 (ARM)
                                                                                   //GPIO39 - PF7 щ utilizado para o HRADC_INT_STS (C28)

        GPIOPinConfigureCoreSelect(GPIO_PORTG_BASE, 0x00, GPIO_PIN_C_CORE_SELECT); //GPIO46 щ utilizado pelo C28 (C28)
                                                                                   //GPIO40 - PG0 щ utilizado para o SDRAM_BA0D13 (ARM)
                                                                                   //GPIO41 - PG1 щ utilizado para o SDRAM_BA1D14 (ARM)
                                                                                   //GPIO42 - PG2 щ utilizado para o USB0_GPIO42 (ARM)
                                                                                   //GPIO43 - PG3 щ utilizado para o RTC-TIMER (ARM)
                                                                                   //GPIO44 - PG4 щ utilizado para o ADC_AD_I (ARM)
                                                                                   //GPIO45 - PG5 щ utilizado para o USB0_GPIO45 (ARM)
                                                                                   //GPIO46 - PG6 щ utilizado para o GPIO0 (ARM)
                                                                                   //GPIO47 - PG7 щ utilizado para o SDRAM_CLK (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTH_BASE, 0x80, GPIO_PIN_C_CORE_SELECT); //GPIO55 щ utilizado pelo C28 (C28)
                                                                                   //GPIO48 - PH0 щ utilizado para a SDRAM_AD6 (ARM)
                                                                                   //GPIO49 - PH1 щ utilizado para a SDRAM_AD7 (ARM)
                                                                                   //GPIO50 - PH2 щ utilizado para o SDRAM_AD1 (ARM)
                                                                                   //GPIO51 - PH3 щ utilizado para o SDRAM_AD0 (ARM)
                                                                                   //GPIO52 - PH4 щ utilizado para a SDRAM_AD10 (ARM)
                                                                                   //GPIO53 - PH5 щ utilizado para a SDRAM_AD11 (ARM)
                                                                                   //GPIO54 - PH6 щ utilizado para o MCU_GPIO78_ENET (ARM)
                                                                                   //GPIO55 - PH7 щ utilizado para o INT_GENERAL/SYNC_IN (C28)

        GPIOPinConfigureCoreSelect(GPIO_PORTJ_BASE, 0x00, GPIO_PIN_C_CORE_SELECT); //Nenhum pino щ utilizado pelo C28 (C28)
                                                                                   //GPIO56 - PJ0 щ utilizado para o SDRAM_DQML (ARM)
                                                                                   //GPIO57 - PJ1 щ utilizado para o SDRAM_DQMH (ARM)
                                                                                   //GPIO58 - PJ2 щ utilizado para o SDRAM_CASn (ARM)
                                                                                   //GPIO59 - PJ3 щ utilizado para o SDRAM_RASn (ARM)
                                                                                   //GPIO60 - PJ4 щ utilizado para o SDRAM_WEn (ARM)
                                                                                   //GPIO61 - PJ5 щ utilizado para o SDRAM_CSn (ARM)
                                                                                   //GPIO62 - PJ6 щ utilizado para o SDRAM_CLKE (ARM)
                                                                                   //GPIO63 - PJ7 щ utilizado para o VOLT_IN_TRIP (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTC_BASE, 0x0F, GPIO_PIN_C_CORE_SELECT); //GPIO64 ao GPIO67 щ utilizado pelo C28 (C28)
                                                                                   //GPIO64 - PC0 щ utilizado para o GPDO4 (C28)
                                                                                   //GPIO65 - PC1 щ utilizado para o GPDO2 (C28)
                                                                                   //GPIO66 - PC2 щ utilizado para o GPDO3 (C28)
                                                                                   //GPIO67 - PC3 щ utilizado para o GPDO1 (C28)
                                                                                   //GPIO68 - PC4 щ utilizado para o SDRAM_AD2 (ARM)
                                                                                   //GPIO69 - PC5 щ utilizado para o SDRAM_AD3 (ARM)
                                                                                   //GPIO70 - PC6 щ utilizado para o SDRAM_AD4 (ARM)
                                                                                   //GPIO71 - PC7 щ utilizado para o SDRAM_AD5 (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTK_BASE, 0x0F, GPIO_PIN_C_CORE_SELECT); //GPIO72 ao GPIO75 щ utilizado pelo C28 (C28)
                                                                                   //GPIO72 - PK0 щ utilizado para o SPI_MOSI_C28 (C28)
                                                                                   //GPIO73 - PK1 щ utilizado para o SPI_MISO_C28 (C28)
                                                                                   //GPIO74 - PK2 щ utilizado para o SPI_CLK_C28 (C28)
                                                                                   //GPIO75 - PK3 щ utilizado para o SPI_STE_C28 (C28)
                                                                                   //GPIO76 - PK4 щ utilizado para o MCU_GPIO76_ENET (ARM)
                                                                                   //GPIO77 - PK5 щ utilizado para o MCU_GPIO77_ENET (ARM)
                                                                                   //GPIO78 - PK6 щ utilizado para o MCU_GPIO78_ENET (ARM)
                                                                                   //GPIO79 - PK7 щ utilizado para o MCU_GPIO79_ENET (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTL_BASE, 0x00, GPIO_PIN_C_CORE_SELECT); //Nenhum pino щ utilizado pelo C28 (C28)
                                                                                   //GPIO80 - PL0 щ utilizado para o MCU_GPIO80_ENET (ARM)
                                                                                   //GPIO81 - PL1 щ utilizado para o MCU_GPIO81_ENET (ARM)
                                                                                   //GPIO82 - PL2 щ utilizado para o MCU_GPIO82_ENET (ARM)
                                                                                   //GPIO83 - PL3 щ utilizado para o MCU_GPIO83_ENET (ARM)
                                                                                   //GPIO84 - PL4 щ utilizado para o MCU_GPIO84_ENET (ARM)
                                                                                   //GPIO85 - PL5 щ utilizado para o MCU_GPIO85_ENET (ARM)
                                                                                   //GPIO86 - PL6 щ utilizado para o MCU_GPIO86_ENET (ARM)
                                                                                   //GPIO87 - PL7 щ utilizado para o MCU_GPIO87_ENET (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTM_BASE, 0x00, GPIO_PIN_C_CORE_SELECT); //Nenhum pino щ utilizado pelo C28 (C28)
                                                                                   //GPIO88 - PM0 щ utilizado para o MCU_GPIO88_ENET (ARM)
                                                                                   //GPIO89 - PM1 щ utilizado para o MCU_GPIO89_ENET (ARM)
                                                                                   //GPIO90 - PM2 щ utilizado para o MCU_GPIO90_ENET (ARM)
                                                                                   //GPIO91 - PM3 щ utilizado para o MCU_GPIO91_ENET (ARM)
                                                                                   //GPIO92 - PM4 щ utilizado para o MCU_GPIO92_ENET (ARM)
                                                                                   //GPIO93 - PM5 щ utilizado para o MCU_GPIO93_ENET (ARM)
                                                                                   //GPIO94 - PM6 щ utilizado para o MCU_GPIO94_ENET (ARM)
                                                                                   //GPIO95 - PM7 щ utilizado para o MCU_GPIO95_ENET (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTN_BASE, 0x00, GPIO_PIN_C_CORE_SELECT); //Nenhum pino щ utilizado pelo C28 (C28)
                                                                                   //GPIO96 - PN0 щ utilizado para o I2C_INT-CLK (ARM)
                                                                                   //GPIO97 - PN1 щ utilizado para o I2C_INT-SDA (ARM)
                                                                                   //GPIO98 - PN2 щ utilizado para o RS-485_RX (ARM)
                                                                                   //GPIO99 - PN3 щ utilizado para o RS-485_TX (ARM)
                                                                                   //GPIO100 - PN4 щ utilizado para o DISPLAY_TX (ARM)
                                                                                   //GPIO101 - PN5 щ utilizado para o DISPLAY_RX (ARM)
                                                                                   //GPIO102 - PN6 щ utilizado para o UART_M3_RX (ARM)
                                                                                   //GPIO103 - PN7 щ utilizado para o UART_M3_TX (ARM)


        GPIOPinConfigureCoreSelect(GPIO_PORTP_BASE, 0xC0, GPIO_PIN_C_CORE_SELECT); //GPIO110 e GPIO111 щ utilizado pelo C28 (C28)
                                                                                   //GPIO104 - PP0 щ utilizado para o BP_I2C_SCL (ARM)
                                                                                   //GPIO105 - PP1 щ utilizado para o BP_I2C_SDA (ARM)
                                                                                   //GPIO106 - PP2 щ utilizado para o RS-485_RD (ARM)
                                                                                   //GPIO107 - PP3 щ utilizado para o UART_M3_RD (ARM)
                                                                                   //GPIO108 - PP4 щ utilizado para o EEPROM-WP (ARM)
                                                                                   //GPIO109 - PP5 щ utilizado para o OperationARM (ARM)
                                                                                   //GPIO110 - PP6 щ utilizado para o OperationDSP (C28)
                                                                                   //GPIO111 - PP7 щ utilizado para o GPIO1 (C28)

        GPIOPinConfigureCoreSelect(GPIO_PORTQ_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT); //Todo o port щ utilizado pelo C28 (C28)
                                                                                   //GPIO112 - PQ0 щ utilizado para o GPDI12B (C28)
                                                                                   //GPIO113 - PQ1 щ utilizado para o GPDI11B (C28)
                                                                                   //GPIO114 - PQ2 щ utilizado para o GPDI10B (C28)
                                                                                   //GPIO115 - PQ3 щ utilizado para o GPDI9B (C28)
                                                                                   //GPIO116 - PQ4 щ utilizado para o GPDI6B (C28)
                                                                                   //GPIO117 - PQ5 щ utilizado para o SCI_RD (C28)
                                                                                   //GPIO118 - PQ6 щ utilizado para o SCITXDA (C28)
                                                                                   //GPIO119 - PQ7 щ utilizado para o SCIRXDA (C28)

        GPIOPinConfigureCoreSelect(GPIO_PORTR_BASE, 0xF0, GPIO_PIN_C_CORE_SELECT); //GPIO124 ao GPIO127 щ utilizado pelo C28 (C28)
                                                                                   //GPIO120 - PR0 щ utilizado para o SPI_SD_DI (ARM)
                                                                                   //GPIO121 - PR1 щ utilizado para o SPI_SD_DO (ARM)
                                                                                   //GPIO122 - PR2 щ utilizado para o SPI_SD_CLK (ARM)
                                                                                   //GPIO123 - PR3 щ utilizado para o SPI_SD_CS (ARM)
                                                                                   //GPIO124 - PR4 щ utilizado para o GPDI3B (C28)
                                                                                   //GPIO125 - PR5 щ utilizado para o GPDI4B (C28)
                                                                                   //GPIO126 - PR6 щ utilizado para o GPDI1B (C28)
                                                                                   //GPIO127 - PR7 щ utilizado para o GPDI2B (C28)

        GPIOPinConfigureCoreSelect(GPIO_PORTS_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT); //Todo o port щ utilizado pelo C28 (C28)
                                                                                   //GPIO128 - PS0 щ utilizado para o GPIO_CS1 (C28)
                                                                                   //GPIO129 - PS1 щ utilizado para o GPIO_CS2 (C28)
                                                                                   //GPIO130 - PS2 щ utilizado para o PWM_SOC (C28)
                                                                                   //GPIO131 - PS3 щ utilizado para o GPIO_CONFIG (C28)
                                                                                   //GPIO132 - PS4 щ utilizado para o STATUS_ADC0 (C28)
                                                                                   //GPIO133 - PS5 щ utilizado para o STATUS_ADC1 (C28)
                                                                                   //GPIO134 - PS6 щ utilizado para o STATUS_ADC2 (C28)
                                                                                   //GPIO135 - PS7 щ utilizado para o STATUS_ADC3 (C28)
    }

    else if(HARDWARE_VERSION == 0x21)
    {
        // Pinos a serem controlados pelo C28
        GPIOPinConfigureCoreSelect(GPIO_PORTA_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT); //GPIO0 ao GPIO7 sуo utilizados para os 8 ch de PWM (C28)
                                                                                   //GPIO0 - PA0 щ utilizado para o PWM_1A
                                                                                   //GPIO1 - PA1 щ utilizado para o PWM_1B
                                                                                   //GPIO2 - PA2 щ utilizado para o PWM_2A
                                                                                   //GPIO3 - PA3 щ utilizado para o PWM_2B
                                                                                   //GPIO4 - PA4 щ utilizado para o PWM_3A
                                                                                   //GPIO5 - PA5 щ utilizado para o PWM_3B
                                                                                   //GPIO6 - PA6 щ utilizado para o PWM_4A
                                                                                   //GPIO7 - PA7 щ utilizado para o PWM_4B

        GPIOPinConfigureCoreSelect(GPIO_PORTB_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT); //GPIO8 ao GPIO15 sуo utilizados para os 8 ch de PWM (C28)
                                                                                   //GPIO8 - PB0 щ utilizado para o PWM_5A
                                                                                   //GPIO9 - PB1 щ utilizado para o PWM_5B
                                                                                   //GPIO10 - PB2 щ utilizado para o PWM_6A
                                                                                   //GPIO11 - PB3 щ utilizado para o PWM_6B
                                                                                   //GPIO12 - PB4 щ utilizado para o PWM_7A
                                                                                   //GPIO13 - PB5 щ utilizado para o PWM_7B
                                                                                   //GPIO14 - PB6 щ utilizado para o PWM_8A
                                                                                   //GPIO15 - PB7 щ utilizado para o PWM_8B

        GPIOPinConfigureCoreSelect(GPIO_PORTD_BASE, 0xF0, GPIO_PIN_C_CORE_SELECT); //GPIO20 ao GPIO23 sуo utilizados para o MCBSP (C28)
                                                                                   //GPIO16 - PD0 щ utilizado para o SPI_ADC_MOSI_I (ARM)
                                                                                   //GPIO17 - PD1 щ utilizado para o SPI_ADC_MISO_I (ARM)
                                                                                   //GPIO18 - PD2 щ utilizado para o SPI_ADC_SCLK_I (ARM)
                                                                                   //GPIO19 - PD3 щ utilizado para o SPI_ADC_SPISTE_I (ARM)
                                                                                   //GPIO20 - PD4 щ utilizado para o MCBSP_MDX (C28)
                                                                                   //GPIO21 - PD5 щ utilizado para o MCBSP_MDR (C28)
                                                                                   //GPIO22 - PD6 щ utilizado para o MCBSP_MCLKX (C28)
                                                                                   //GPIO23 - PD7 щ utilizado para o MCBSP_MFSX (C28)

        GPIOPinConfigureCoreSelect(GPIO_PORTE_BASE, 0x20, GPIO_PIN_C_CORE_SELECT); //GPIO29 щ utilizado para a interrupчуo externa do C28 (C28)
                                                                                   //GPIO24 - PE0 щ utilizado para o SDRAM_AD8 (ARM)
                                                                                   //GPIO25 - PE1 щ utilizado para o SDRAM_AD9 (ARM)
                                                                                   //GPIO26 - PE2 щ utilizado para o SPI_FLASH_MISO (ARM)
                                                                                   //GPIO27 - PE3 щ utilizado para o SPI_FLASH_MOSI (ARM)
                                                                                   //GPIO28 - PE4 щ utilizado para o INT_ARM (ARM)
                                                                                   //GPIO29 - PE5 щ utilizado para o INT_C28 (C28)
                                                                                   //GPIO30 - PE6 щ utilizado para o CAN_RX (ARM)
                                                                                   //GPIO31 - PE7 щ utilizado para o CAN_TX (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTF_BASE, 0xC3, GPIO_PIN_C_CORE_SELECT); //GPIO32, GPIO33 e GPIO39 sуo utilizados pelo C28 (C28)
                                                                                   //GPIO32 - PF0 щ utilizado para o PWM_SOC (C28)
                                                                                   //GPIO33 - PF1 щ utilizado para o EPWMSYNCO (C28)
                                                                                   //GPIO34 - PF2 щ utilizado para o SPI_FLASH_CLK (ARM)
                                                                                   //GPIO35 - PF3 щ utilizado para o SPI_FLASH_CS (ARM)
                                                                                   //GPIO36 - PF4 щ utilizado para o SDRAM_AD12 (ARM)
                                                                                   //GPIO37 - PF5 щ utilizado para o SDRAM_AD15 (ARM)
                                                                                   //GPIO38 - PF6 щ utilizado para o EPWMSYNCI (C28)
                                                                                   //GPIO39 - PF7 щ utilizado para o HRADC_INT_STS (C28)

        GPIOPinConfigureCoreSelect(GPIO_PORTG_BASE, 0x24, GPIO_PIN_C_CORE_SELECT); //GPIO42 e 45 sуo utilizados pelo C28 (C28)
                                                                                   //GPIO40 - PG0 щ utilizado para o SDRAM_BA0D13 (ARM)
                                                                                   //GPIO41 - PG1 щ utilizado para o SDRAM_BA1D14 (ARM)
                                                                                   //GPIO42 - PG2 щ utilizado para o STATUS_ADC0 (C28)
                                                                                   //GPIO43 - PG3 щ utilizado para o RTC-TIMER (ARM)
                                                                                   //GPIO44 - PG4 щ utilizado para o ADC_AD_I (ARM)
                                                                                   //GPIO45 - PG5 щ utilizado para o STATUS_ADC2 (C28)
                                                                                   //GPIO46 - PG6 щ utilizado para o GPIO0 (ARM)
                                                                                   //GPIO47 - PG7 щ utilizado para o SDRAM_CLK (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTH_BASE, 0x80, GPIO_PIN_C_CORE_SELECT); //GPIO55 щ utilizado pelo C28 (C28)
                                                                                   //GPIO48 - PH0 щ utilizado para a SDRAM_AD6 (ARM)
                                                                                   //GPIO49 - PH1 щ utilizado para a SDRAM_AD7 (ARM)
                                                                                   //GPIO50 - PH2 щ utilizado para o SDRAM_AD1 (ARM)
                                                                                   //GPIO51 - PH3 щ utilizado para o SDRAM_AD0 (ARM)
                                                                                   //GPIO52 - PH4 щ utilizado para a SDRAM_AD10 (ARM)
                                                                                   //GPIO53 - PH5 щ utilizado para a SDRAM_AD11 (ARM)
                                                                                   //GPIO54 - PH6 щ utilizado para o MCU_GPIO78_ENET (ARM)
                                                                                   //GPIO55 - PH7 щ utilizado para o INT_GENERAL/SYNC_IN (C28)

        GPIOPinConfigureCoreSelect(GPIO_PORTJ_BASE, 0x00, GPIO_PIN_C_CORE_SELECT); //Nenhum pino щ utilizado pelo C28 (C28)
                                                                                   //GPIO56 - PJ0 щ utilizado para o SDRAM_DQML (ARM)
                                                                                   //GPIO57 - PJ1 щ utilizado para o SDRAM_DQMH (ARM)
                                                                                   //GPIO58 - PJ2 щ utilizado para o SDRAM_CASn (ARM)
                                                                                   //GPIO59 - PJ3 щ utilizado para o SDRAM_RASn (ARM)
                                                                                   //GPIO60 - PJ4 щ utilizado para o SDRAM_WEn (ARM)
                                                                                   //GPIO61 - PJ5 щ utilizado para o SDRAM_CSn (ARM)
                                                                                   //GPIO62 - PJ6 щ utilizado para o SDRAM_CLKE (ARM)
                                                                                   //GPIO63 - PJ7 щ utilizado para o VOLT_IN_TRIP (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTC_BASE, 0x0F, GPIO_PIN_C_CORE_SELECT); //GPIO64 ao GPIO67 щ utilizado pelo C28 (C28)
                                                                                   //GPIO64 - PC0 щ utilizado para o GPDO4 (C28)
                                                                                   //GPIO65 - PC1 щ utilizado para o GPDO2 (C28)
                                                                                   //GPIO66 - PC2 щ utilizado para o GPDO3 (C28)
                                                                                   //GPIO67 - PC3 щ utilizado para o GPDO1 (C28)
                                                                                   //GPIO68 - PC4 щ utilizado para o SDRAM_AD2 (ARM)
                                                                                   //GPIO69 - PC5 щ utilizado para o SDRAM_AD3 (ARM)
                                                                                   //GPIO70 - PC6 щ utilizado para o SDRAM_AD4 (ARM)
                                                                                   //GPIO71 - PC7 щ utilizado para o SDRAM_AD5 (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTK_BASE, 0x0F, GPIO_PIN_C_CORE_SELECT); //GPIO72 ao GPIO75 щ utilizado pelo C28 (C28)
                                                                                   //GPIO72 - PK0 щ utilizado para o SPI_MOSI_C28 (C28)
                                                                                   //GPIO73 - PK1 щ utilizado para o SPI_MISO_C28 (C28)
                                                                                   //GPIO74 - PK2 щ utilizado para o SPI_CLK_C28 (C28)
                                                                                   //GPIO75 - PK3 щ utilizado para o SPI_STE_C28 (C28)
                                                                                   //GPIO76 - PK4 щ utilizado para o MCU_GPIO76_ENET (ARM)
                                                                                   //GPIO77 - PK5 щ utilizado para o MCU_GPIO77_ENET (ARM)
                                                                                   //GPIO78 - PK6 щ utilizado para o MCU_GPIO78_ENET (ARM)
                                                                                   //GPIO79 - PK7 щ utilizado para o MCU_GPIO79_ENET (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTL_BASE, 0x00, GPIO_PIN_C_CORE_SELECT); //Nenhum pino щ utilizado pelo C28 (C28)
                                                                                   //GPIO80 - PL0 щ utilizado para o MCU_GPIO80_ENET (ARM)
                                                                                   //GPIO81 - PL1 щ utilizado para o MCU_GPIO81_ENET (ARM)
                                                                                   //GPIO82 - PL2 щ utilizado para o MCU_GPIO82_ENET (ARM)
                                                                                   //GPIO83 - PL3 щ utilizado para o MCU_GPIO83_ENET (ARM)
                                                                                   //GPIO84 - PL4 щ utilizado para o MCU_GPIO84_ENET (ARM)
                                                                                   //GPIO85 - PL5 щ utilizado para o MCU_GPIO85_ENET (ARM)
                                                                                   //GPIO86 - PL6 щ utilizado para o MCU_GPIO86_ENET (ARM)
                                                                                   //GPIO87 - PL7 щ utilizado para o MCU_GPIO87_ENET (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTM_BASE, 0x00, GPIO_PIN_C_CORE_SELECT); //Nenhum pino щ utilizado pelo C28 (C28)
                                                                                   //GPIO88 - PM0 щ utilizado para o MCU_GPIO88_ENET (ARM)
                                                                                   //GPIO89 - PM1 щ utilizado para o MCU_GPIO89_ENET (ARM)
                                                                                   //GPIO90 - PM2 щ utilizado para o MCU_GPIO90_ENET (ARM)
                                                                                   //GPIO91 - PM3 щ utilizado para o MCU_GPIO91_ENET (ARM)
                                                                                   //GPIO92 - PM4 щ utilizado para o MCU_GPIO92_ENET (ARM)
                                                                                   //GPIO93 - PM5 щ utilizado para o MCU_GPIO93_ENET (ARM)
                                                                                   //GPIO94 - PM6 щ utilizado para o MCU_GPIO94_ENET (ARM)
                                                                                   //GPIO95 - PM7 щ utilizado para o MCU_GPIO95_ENET (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTN_BASE, 0x00, GPIO_PIN_C_CORE_SELECT); //Nenhum pino щ utilizado pelo C28 (C28)
                                                                                   //GPIO96 - PN0 щ utilizado para o I2C_INT-CLK (ARM)
                                                                                   //GPIO97 - PN1 щ utilizado para o I2C_INT-SDA (ARM)
                                                                                   //GPIO98 - PN2 щ utilizado para o RS-485_RX (ARM)
                                                                                   //GPIO99 - PN3 щ utilizado para o RS-485_TX (ARM)
                                                                                   //GPIO100 - PN4 щ utilizado para o DISPLAY_TX (ARM)
                                                                                   //GPIO101 - PN5 щ utilizado para o DISPLAY_RX (ARM)
                                                                                   //GPIO102 - PN6 щ utilizado para o UART_M3_RX (ARM)
                                                                                   //GPIO103 - PN7 щ utilizado para o UART_M3_TX (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTP_BASE, 0xE0, GPIO_PIN_C_CORE_SELECT); //GPIO109 , 110 e 111 sуo utilizados pelo C28 (C28)
                                                                                   //GPIO104 - PP0 щ utilizado para o BP_I2C_SCL (ARM)
                                                                                   //GPIO105 - PP1 щ utilizado para o BP_I2C_SDA (ARM)
                                                                                   //GPIO106 - PP2 щ utilizado para o RS-485_RD (ARM)
                                                                                   //GPIO107 - PP3 щ utilizado para o UART_M3_RD (ARM)
                                                                                   //GPIO108 - PP4 щ utilizado para o EEPROM-WP (ARM)
                                                                                   //GPIO109 - PP5 щ utilizado para o GPDI9B (C28)
                                                                                   //GPIO110 - PP6 щ utilizado para o GPDI10B (C28)
                                                                                   //GPIO111 - PP7 щ utilizado para o GPIO1 (C28)

        // GPIO1 for ARM
        //GPIOPinConfigureCoreSelect(GPIO_PORTP_BASE, 0x60, GPIO_PIN_C_CORE_SELECT); //GPIO109 , 110 e 111 sуo utilizados pelo C28 (C28)
                                                                                   //GPIO104 - PP0 щ utilizado para o BP_I2C_SCL (ARM)
                                                                                   //GPIO105 - PP1 щ utilizado para o BP_I2C_SDA (ARM)
                                                                                   //GPIO106 - PP2 щ utilizado para o RS-485_RD (ARM)
                                                                                   //GPIO107 - PP3 щ utilizado para o UART_M3_RD (ARM)
                                                                                   //GPIO108 - PP4 щ utilizado para o EEPROM-WP (ARM)
                                                                                   //GPIO109 - PP5 щ utilizado para o GPDI9B (C28)
                                                                                   //GPIO110 - PP6 щ utilizado para o GPDI10B (C28)
                                                                                   //GPIO111 - PP7 щ utilizado para o GPIO1 (ARM)

        GPIOPinConfigureCoreSelect(GPIO_PORTQ_BASE, 0xF3, GPIO_PIN_C_CORE_SELECT); //GPIO114 e 115 sуo utilizados pelo ARM (ARM)
                                                                                   //GPIO112 - PQ0 щ utilizado para o GPDI12B (C28)
                                                                                   //GPIO113 - PQ1 щ utilizado para o GPDI11B (C28)
                                                                                   //GPIO114 - PQ2 щ utilizado para o USB_UART_RX (ARM)
                                                                                   //GPIO115 - PQ3 щ utilizado para o USB_UART_TX (ARM)
                                                                                   //GPIO116 - PQ4 щ utilizado para o GPDI6B (C28)
                                                                                   //GPIO117 - PQ5 щ utilizado para o SCI_RD (C28)
                                                                                   //GPIO118 - PQ6 щ utilizado para o SCITXDA (C28)
                                                                                   //GPIO119 - PQ7 щ utilizado para o SCIRXDA (C28)

        GPIOPinConfigureCoreSelect(GPIO_PORTR_BASE, 0xF0, GPIO_PIN_C_CORE_SELECT); //GPIO124 ao GPIO127 щ utilizado pelo C28 (C28)
                                                                                   //GPIO120 - PR0 щ utilizado para o SPI_SD_DI (ARM)
                                                                                   //GPIO121 - PR1 щ utilizado para o SPI_SD_DO (ARM)
                                                                                   //GPIO122 - PR2 щ utilizado para o SPI_SD_CLK (ARM)
                                                                                   //GPIO123 - PR3 щ utilizado para o SPI_SD_CS (ARM)
                                                                                   //GPIO124 - PR4 щ utilizado para o GPDI3B (C28)
                                                                                   //GPIO125 - PR5 щ utilizado para o GPDI4B (C28)
                                                                                   //GPIO126 - PR6 щ utilizado para o GPDI1B (C28)
                                                                                   //GPIO127 - PR7 щ utilizado para o GPDI2B (C28)

        GPIOPinConfigureCoreSelect(GPIO_PORTS_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT); //Todo o port щ utilizado pelo C28 (C28)
                                                                                   //GPIO128 - PS0 щ utilizado para o 4KHz (C28)
                                                                                   //GPIO129 - PS1 щ utilizado para o GPIO_CS2 (C28)
                                                                                   //GPIO130 - PS2 щ utilizado para o GPIO_CS1 (C28)
                                                                                   //GPIO131 - PS3 щ utilizado para o GPIO_CONFIG (C28)
                                                                                   //GPIO132 - PS4 щ utilizado para o 16Hz (C28)
                                                                                   //GPIO133 - PS5 щ utilizado para o STATUS_ADC1 (C28)
                                                                                   //GPIO134 - PS6 щ utilizado para o SOUND_CUSTOM (C28)
                                                                                   //GPIO135 - PS7 щ utilizado para o STATUS_ADC3 (C28)
    }

}
