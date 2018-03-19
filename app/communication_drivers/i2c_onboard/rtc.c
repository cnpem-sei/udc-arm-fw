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
 * @file rtc.c
 * @brief RTC module.
 *
 * @author joao.rosa
 *
 * @date 15/07/2015
 *
 */

#include <stdint.h>

#include "i2c_onboard.h"
#include "rtc.h"

#define I2C_SLV_ADDR_RTC	0x68 // Endereço 7 bits

uint8_t data[10];

uint64_t	DataHour;

uint64_t data_hour_read(void)
{
	return DataHour;
}

void rtc_write_data_hour(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t dayweek, uint8_t day, uint8_t month, uint8_t year)
{
	data[0] = 0x01; //Register
	data[1] = seconds;
	data[2] = minutes;
	data[3] = hours;
	data[4] = dayweek;
	data[5] = day;
	data[6] = month;
	data[7] = year;

	write_i2c(I2C_SLV_ADDR_RTC, 0x08, data);
}

void rtc_read_data_hour(void)
{
	data[0] = 0x01;  // Register
	read_i2c(I2C_SLV_ADDR_RTC, SINGLE_ADDRESS, 0x07, data);

	DataHour = data[6]; // Year
	DataHour = DataHour << 8;

	DataHour |= data[5]; // Month
	DataHour = DataHour << 8;

	DataHour |= data[4]; // Day
	DataHour = DataHour << 8;

	DataHour |= data[3]; // Day week
	DataHour = DataHour << 8;

	DataHour |= data[2]; // Hours
	DataHour = DataHour << 8;

	DataHour |= data[1]; // Minutes
	DataHour = DataHour << 8;

	DataHour |= data[0]; // Seconds

	// 0x00 00 00 00 00 00 00 00  64bits
	//     |Y |M |D |DW|H |M |S
}

void rtc_clear_ht(void)
{
	uint8_t tst;
	data[0] = 0x0C;  // Register
	read_i2c(I2C_SLV_ADDR_RTC, SINGLE_ADDRESS, 0x02, data);

	tst = data[0];
	tst &= 0b01000000;

	if(tst)
	{
		data[1] = data[0] & 0b10111111;
		data[0] = 0x0C;

		write_i2c(I2C_SLV_ADDR_RTC, 0x02, data); // Send command to clear the HT bit

	}
}

void rtc_stop_clock(void)
{
	data[0] = 0x01;
	data[1] = 0x80;
	write_i2c(I2C_SLV_ADDR_RTC, 0x02, data); // Send command to stop the RTC

	data[0] = 0x01;
	data[1] = 0x00;
	write_i2c(I2C_SLV_ADDR_RTC, 0x02, data); // Send command to start the RTC
}

uint8_t rtc_status_of(void)
{
	uint8_t tst;
	data[0] = 0x0F; // Register
	read_i2c(I2C_SLV_ADDR_RTC, SINGLE_ADDRESS, 0x02, data);

	tst = data[0];
	tst &= 0b00000100;
	return tst;
}

void rtc_check_of(void)
{
	// Check RTC Oscillator

	unsigned long ulLoop;

	if(rtc_status_of())
	{
		// Send a message to the system to indicate corruption in the RTC data

		for (ulLoop=0;ulLoop<80000000;ulLoop++){}; // Wait 4 seconds

		data[1] = data[0] & 0b11111011;
		data[0] = 0x0F;

		write_i2c(I2C_SLV_ADDR_RTC, 0x02, data); // Send command to clear the OF bit

		// Test if the RTC is working
		if(rtc_status_of())
		{
		    rtc_stop_clock(); // Stop and start the RTC oscillator to try turn on the RTC
			if(rtc_status_of()) ; // Test RTC to check the oscillator, if a problem is found than a message needs to be send to indicate a problem with RTC
		}

	}
}

uint8_t rtc_battery_check(void)
{
	// Check if the battery is OK, Low Battery (LB) bit
	// The battery is cehcked once a day at 23:59:59 in continuous work or during power-up process

	uint8_t tst;

	data[0] = 0x0F; // Register
	read_i2c(I2C_SLV_ADDR_RTC, SINGLE_ADDRESS, 0x02, data);

	tst = data[0];
	tst &= 0b00010000;

	return tst;
}

void rtc_init(void)
{
    rtc_clear_ht();
    rtc_check_of();
}
