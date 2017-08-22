/*
 * hradc.h
 *
 *  Created on: 22/06/2015
 *      Author: joao.rosa
 */

#include <stdint.h>

#ifndef HRADC_H_
#define HRADC_H_

/* Defines for HRADC */
#define HEATER_DISABLE 		0
#define HEATER_ENABLE 		1
#define RAILS_DISABLE 		0
#define RAILS_ENABLE 		1

#define UFM_BOARDDATA_SIZE 	24

/* Enumerate definition for configurable analog inputs */
typedef enum {
		Vin_bipolar,
		Vin_unipolar_p,
		Vin_unipolar_n,
		Iin_bipolar,
		Iin_unipolar_p,
		Iin_unipolar_n,
		Vref_bipolar_p,
		Vref_bipolar_n,
		GND,
		Vref_unipolar_p,
		Vref_unipolar_n,
		GND_unipolar,
		Temp,
		Reserved0,
		Reserved1,
		Reserved2
} eInputType;

typedef enum {
		HRADC_Sampling,
		HRADC_UFM
} eHRADCOpMode;

typedef enum {
	FBP,
	FAx
} eHRADCVar;

/* HRADC Board Data from UFM */
typedef volatile struct
{
	uint64_t 	SerialNumber;			// Unique identification
	uint16_t	CalibDate[5]; 			// Date of last calibration [DD/MM/AAAA/hh/mm]
	eHRADCVar	Variant;				// Board variant (FBP/FAx/...)
	float		Rburden;				// Nominal value of burden resistor
	float 		CalibTemp;	  			// Ambient temperature during last calibration
	float		gain_Vin_bipolar;		// Calibration gain		Vin bipolar
	float		offset_Vin_bipolar;		// Calibration offset	Vin bipolar
	float		gain_Iin_bipolar;		// Calibration gain		Iin bipolar
	float		offset_Iin_bipolar;		// Calibration offset	Iin bipolar
	float		Vref_bipolar_p;			// + Voltage reference 	Bipolar
	float		Vref_bipolar_n;			// - Voltage reference 	Bipolar
	float		GND_bipolar;			// GND					Bipolar
} tHRADC_BoardData;

typedef volatile union
{
	tHRADC_BoardData	t;
	uint16_t 				u[UFM_BOARDDATA_SIZE];
} uHRADC_BoardData;


/* HRADC Board Information */
typedef volatile struct
{
	uint16_t 			ID;						// Backplane position
	eHRADCOpMode		OpMode;					// Operation mode;
	eInputType			AnalogInput;			// Current analog input
	uint16_t 			enable_Heater;			// Current temperature controller status
	uint16_t 			enable_RailsMonitor;	// Rails monitor status
	uint16_t			size_SamplesBuffer;		// Size of samples buffer
	uint16_t			index_SamplesBuffer;	// Current buffer pointer position
	uint32_t			StatusReg;				// Configuration/Status Register
	volatile uint32_t	*SamplesBuffer;			// Pointer to samples buffer
	volatile float		*gain;					// Pointer to gain value of current analog input
	volatile float		*offset;				// Pointer to offset value of current analog input
	uHRADC_BoardData	BoardData;				// Calibration database
} HRADC_struct;

typedef volatile struct
{
	float			freq_Sampling;
	uint16_t 		enable_Sampling;
	uint16_t 		n_HRADC_boards;
	HRADC_struct 	HRADC_boards[4];
} HRADCs_struct;

/* Prototype statements for variables */
extern volatile HRADCs_struct HRADCs_Info;
/*extern volatile HRADC_struct  HRADC0_board;
extern volatile HRADC_struct  HRADC1_board;
extern volatile HRADC_struct  HRADC2_board;
extern volatile HRADC_struct  HRADC3_board;*/

#endif /* HRADC_H_ */
