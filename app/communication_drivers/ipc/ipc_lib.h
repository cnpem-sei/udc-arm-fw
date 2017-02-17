/*
 * 		FILE: 		ipc_lib.h
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	05/11/2015
 * 		MODIFIED:	05/11/2015
 *
 * 		AUTHOR: 	Ricieri  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code for interprocessor communications (IPC)
 *
 *		TODO:
 */

#include "stdint.h"
#include "../shared_memory/structs.h"

#ifndef IPC_LIB_H_
#define IPC_LIB_H_

#define DP_MODULE_MAX_COEFF   16

#define N_SWEEP_FREQS			37
#define N_MAX_REF				4
#define N_MAX_DIG				8
#define	N_MAX_AI				8

/*
 *
 * MtoC Message Defines
 */
#define IPC_PS_ON_OFF			0x00000011 //IPC1+IPC5
#define OPERATING_MODE			0x00000021 //IPC1+IPC6
#define OPEN_CLOSE_LOOP			0x00000041 //IPC1+IPC7
#define SLOWREF_UPDATE			0x00000081 //IPC1+IPC8
#define SIGGEN_ENA_DIS			0x00000101 //IPC1+IPC9
#define SIGGEN_CONFIG			0x00000201 //IPC1+IPC10
#define DPMODULES_CONFIG		0x00000401 //IPC1+IPC11
#define SAMPLES_BUFFER_ONOFF	0x00000801 //IPC1+IPC12
#define RESET_INTERLOCKS		0x00001001 //IPC1+IPC13
#define RESET_WFMREF			0x00002001 //IPC1+IPC14
//...//
#define HRADC_SAMPLING_DISABLE	0x08000001 //IPC1+IPC28
#define HRADC_SAMPLING_ENABLE	0x10000001 //IPC1+IPC29
#define HRADC_OPMODE			0x20000001 //IPC1+IPC30
#define HRADC_CONFIG			0x40000001 //IPC1+IPC31
#define CTOM_MESSAGE_ERROR		0x80000001 //IPC1+IPC32

#define WFMREF_SYNC				0x00000002 //IPC2
#define SOFT_INTERLOCK		0x00000004 //IPC3
#define HARD_INTERLOCK		0x00000008 //IPC4

/*
 * CtoM Message Defines
 */
#define IPC5				0x00000011 //IPC1+IPC5
#define IPC6				0x00000021 //IPC1+IPC6
#define IPC7				0x00000041 //IPC1+IPC7
#define IPC8				0x00000081 //IPC1+IPC8
#define IPC9				0x00000101 //IPC1+IPC9
#define IPC10				0x00000201 //IPC1+IPC10
#define IPC11				0x00000401 //IPC1+IPC11
#define MTOC_MESSAGE_ERROR	0x80000001 //IPC1+IPC32
#define SOFT_INTERLOCK_CTOM	0x00000002 //IPC2
#define HARD_INTERLOCK_CTOM	0x00000004 //IPC3
#define IPC4				0x00000008 //IPC4

/*
 * CtoM Hard Interlocks Defines
 */
#define	LOAD_OVERCURRENT		0x00000001
#define	EXTERNAL_INTERLOCK		0x00000002
#define	AC_FAULT				0x00000004
#define	ACDC_FAULT				0x00000008
#define	DCDC_FAULT				0x00000010
#define	LOAD_OVERVOLTAGE		0x00000020
#define PRECHARGERS_FAULT		0x00000040
#define OUTPUT_CAP_CHARGE_FAULT	0x00000080
#define	EMERGENCY_BUTTON		0x00000100
#define OUT_OVERVOLTAGE			0x00000200
#define IN_OVERVOLTAGE			0x00000400
#define ARM1_OVERCURRENT		0x00000800
#define ARM2_OVERCURRENT		0x00001000
#define IN_OVERCURRENT			0x00002000
#define DRIVER1_FAULT			0x00004000
#define DRIVER2_FAULT			0x00008000
#define OUT1_OVERCURRENT		0x00010000
#define OUT2_OVERCURRENT		0x00020000
#define OUT1_OVERVOLTAGE		0x00040000
#define	OUT2_OVERVOLTAGE		0x00080000
#define LEAKAGE_OVERCURRENT		0x00100000
#define AC_OVERCURRENT			0x00200000

#define IGBT1_OVERTEMP			0x00000001
#define IGBT2_OVERTEMP			0x00000002
#define L1_OVERTEMP				0x00000004
#define L2_OVERTEMP				0x00000008
#define HEATSINK_OVERTEMP		0x00000010
#define WATER_OVERTEMP			0x00000020
#define RECTFIER1_OVERTEMP		0x00000040
#define RECTFIER2_OVERTEMP		0x00000080
#define AC_TRANSF_OVERTEMP		0x00000100
#define WATER_FLUX_FAULT		0x00000200
#define OVER_HUMIDITY_FAULT		0x00000400


typedef enum {NO_ERROR_CTOM, ERROR1, ERROR2, ERROR3, ERROR4} 	eCTOMerror;
typedef enum {NO_ERROR_MTOC, INVALID_SLOWREF_UPDATE, INVALID_DP_MODULE, IPC_LOW_PRIORITY_MSG_FULL, HRADC_CONFIG_ERROR} 	eMTOCerror;
typedef enum {OneShot, SampleBySample, SampleBySample_Continuous} eSyncMode;

//######################## MTOC ###############################

typedef enum
{
	FBP_100kHz,
	FBP_Parallel_100kHz,
	FAC_ACDC_10kHz,
	FAC_DCDC_20kHz,
	FAC_Full_ACDC_10kHz,
	FAC_Full_DCDC_20kHz,
	FAP_ACDC,
	FAP_DCDC_20kHz,
	TEST_HRPWM,
	TEST_HRADC,
	JIGA_HRADC,
	FAP_DCDC_15kHz_225A,
	FBPx4_100kHz
}ePSModel;

typedef enum
{
	SlowRef,
	FastRef,
	WfmRef,
	SigGen
}ePSOpMode;

typedef enum
{
	ELP_Error,
	ELP_SRLim,
	ELP_LPF,
	ELP_PI_dawu,
	ELP_IIR_2P2Z,
	ELP_IIR_3P3Z,
	DCL_PID,
	DCL_PI,
	DCL_DF13,
	DCL_DF22,
	DCL_23
} eDPclass;

typedef enum
{
	Buffer_Idle,
	Buffer_All,
	Buffer_Block0,
	Buffer_Block1,
} eBlockBusy;

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

typedef struct
{
	union
	{
		uint8_t  u8[4];
		float 	*f;
	} PtrBufferStart;

	union
	{
		uint8_t  u8[4];
		float 	*f;
	} PtrBufferEnd;

	union
	{
		uint8_t  u8[4];
		float 	*f;
	} PtrBufferK;

	union
	{
		eBlockBusy enu;
		uint16_t u16;
	}BufferBusy;
} tBuffer;

typedef struct
{
	union
	{
	uint8_t  u8[2];
	uint16_t u16;
	ePSModel enu;
	}Model;
	union
	{
	 uint8_t  u8[2];
	 uint16_t u16;
	}OnOff;
	union
	{
	uint16_t u16;
	ePSOpMode enu;
	}OpMode;
	union
	{
	 uint8_t  u8[2];
     uint16_t u16;
	}OpenLoop;
	union
	{
	 uint8_t u8[2];
	 uint16_t u16;
	}LocalRemote;
	union
	{
	 uint8_t  u8[4];
	 uint32_t u32;
	}SoftInterlocks;
	union
	{
	 uint8_t  u8[4];
	 uint32_t u32;
	}HardInterlocks;
	union
	{
	 uint8_t  u8[2];
	 uint16_t u16;
	}BufferOnOff;
	union
    {
	 float   f;
	 uint32_t u32;
    }ISlowRef;
    eCTOMerror ErrorCtoM;
}tPSModuleMtoC;

typedef struct
{
	tBuffer   BufferInfo;
	union
	{
	 uint8_t  u8[4];
	 uint32_t u32;
	 float    f;
	}Gain;
	union
	{
	 uint8_t  u8[4];
	 uint32_t u32;
	 float    f;
	}Offset;
	union
	{
	 uint8_t  u8[2];
	 uint16_t u16;
	 eSigGenType enu;
	}SyncMode;
}tWfmRef;

typedef struct
{
	union
	{
		uint8_t  u8[2];
		uint16_t u16;
	}Enable;
	union{
		uint8_t  u8[2];
		uint16_t u16;
		eSigGenType enu;
	}Type;
	union
	{
		uint8_t  u8[2];
		uint16_t u16;
	}Ncycles;
	union
	{
		uint8_t  u8[4];
		uint32_t  u32;
		float 	 f;
	}PhaseStart;
	union
	{
		uint8_t  u8[4];
		uint32_t  u32;
		float 	 f;
	}PhaseEnd;

	union
	{
		uint8_t  u8[4];
		float 	 f;
	}Freq;

	union
	{
		uint8_t  u8[4];
		float 	 f;
	}Amplitude[N_SWEEP_FREQS];

	union
	{
		uint8_t  u8[4];
		float 	 f;
	}Offset;

	union
	{
		uint8_t  u8[4];
		float 	 f;
	}Aux;

}tSigGen;

typedef struct
{
	union
	{
		uint8_t u8[2];
		uint16_t u16;
	}ID;
	union
	{
		uint8_t u8[2];
		eDPclass enu;
	}DPclass;
	union
	{
		uint8_t u8[4];
		float	f;
	} Coeffs[DP_MODULE_MAX_COEFF];

}tDPModule;

typedef struct
{
	union
	{
		uint8_t u8[2];
		uint16_t u16;
	}ID;
	union
	{
		uint8_t  u8[4];
		uint32_t u32;
		float	 f;
	} FreqSampling;
	union
	{
		uint8_t      u8[2];
		uint16_t     u16;
		eHRADCOpMode enu;
	}OpMode;
	union
	{
		uint8_t   u8[2];
		uint16_t   u16;
		eInputType enu;
	}InputType;
	union
	{
		uint8_t u8[2];
		uint16_t u16;
	}EnableHeater;
	union
	{
		uint8_t u8[2];
		uint16_t u16;
	}EnableMonitor;
} tHRADCConfig;


typedef struct
{
	 tPSModuleMtoC	PSModule;
     tWfmRef		WfmRef;
     tSigGen		SigGen;
     tDPModule		DPModule;
     tHRADCConfig	HRADCConfig;
}tIPC_MTOC_MSG_RAM;

//######################## CTOM ###############################

typedef struct
{
	union
	{
	 uint8_t u8[2];
	 uint16_t u16;
	}OnOff;
	union
	{
	uint8_t u8[2];
	ePSOpMode enu;
	}OpMode;
	union
	{
	 uint8_t u8[2];
     uint16_t u16;
	}OpenLoop;
	union
	{
	 uint8_t u8[4];
	 uint32_t u32;
	}SoftInterlocks;
	union
	{
	 uint8_t u8[4];
	 uint32_t u32;
	}HardInterlocks;
	union
	{
	 uint8_t u8[2];
	 uint16_t u16;
	}BufferOnOff;
	union
	{
     float f;
     uint8_t u8[4];
	}IRef;
    eMTOCerror ErrorMtoC;
}tPSModuleCtoM;

typedef struct
{
	 tPSModuleCtoM 	PSModule;
	 tWfmRef       	WfmRef;
	 tBuffer 		SamplesBuffer;
}tIPC_CTOM_MSG_RAM;

/*
typedef volatile struct
{
		float		Ref_max[N_MAX_REF];
		float		SR_ref_max[N_MAX_REF];

		Uint16			HRADC_K_decim;
		float			HRADC_Transducer_InputRated[N_MAX_HRADC];
		float			HRADC_Transducer_OutputRated[N_MAX_HRADC];
		enum_AN_INPUT	HRADC_Transducer_OutputType[N_MAX_HRADC];
		Uint16			HRADC_EnableHeater[N_MAX_HRADC];
		Uint16			HRADC_EnableRails[N_MAX_HRADC];

		ePWMConfig	PWM_Config;
		Uint16		N_PowerModules;
		double		Freq_PWM[N_MAX_PWM_MODULES];
		float		T_dead_min[N_MAX_PWM_MODULES];
		float		T_dead[N_MAX_PWM_MODULES];
		float		d_min_lim[2*N_MAX_PWM_MODULES];
		float		d_min[2*N_MAX_PWM_MODULES];
		float		d_max_lim[2*N_MAX_PWM_MODULES];
		float		d_max[2*N_MAX_PWM_MODULES];
		float		dD_max_lim[2*N_MAX_PWM_MODULES];
		float		dD_max[2*N_MAX_PWM_MODULES];

		Uint16		T_Off;
		Uint16		T_Stg;

		Uint16		C_Dig[N_MAX_DIG];
		Uint16		N_Dig[N_MAX_DIG];

		float		V_min_NC[N_MAX_AI];
		float		V_max_NC[N_MAX_AI];
		float		V_min_CR[N_MAX_AI];
		float		V_max_CR[N_MAX_AI];
		float		V_min_SC[N_MAX_AI];
		float		V_max_SC[N_MAX_AI];
		Uint16		N_NC[N_MAX_AI];
		Uint16		N_CR[N_MAX_AI];
		Uint16		N_SC[N_MAX_AI];
		Uint16		K_decim[N_MAX_AI];
} tIPC_MTOC_PARAM_RAM;

typedef volatile struct
{
	void (*PS_turnOn)(void);
	void (*PS_turnOff)(void);
} tIPC_MTOC_PS_FUNCS;

*/

extern void IPCInit(void);
extern void CtoMIPC1IntHandler(void);
extern void CtoMIPC2IntHandler(void);
extern void CtoMIPC3IntHandler(void);
extern void SendIpcFlag(unsigned long int flag);
extern inline unsigned long IPC_MtoC_Translate (unsigned long ulShareAddress);
extern inline unsigned long IPC_CtoM_Translate (unsigned long ulShareAddress);
extern unsigned short IPCMtoCBusy (unsigned long ulFlags);

extern tIPC_CTOM_MSG_RAM IPC_CtoM_Msg;
extern tIPC_MTOC_MSG_RAM IPC_MtoC_Msg;
//extern tIPC_MTOC_PARAM_RAM	IPC_MtoC_Param;
//extern tIPC_MTOC_PS_FUNCS	IPC_MtoC_PS_funcs;

#endif /* IPC_LIB_H_ */
