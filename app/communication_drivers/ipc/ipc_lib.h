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

#define DP_MODULE_MAX_COEFF   16

#define N_SWEEP_FREQS			37
#define N_MAX_REF				4
#define N_MAX_DIG				8
#define	N_MAX_AI				8

/*
 *
 * MtoC Message Defines
 */
#define IPC_PS_ON_OFF		 0x00000011 //IPC1+IPC5
#define OPERATING_MODE		 0x00000021 //IPC1+IPC6
#define OPEN_CLOSE_LOOP		 0x00000041 //IPC1+IPC7
#define SLOWREF_UPDATE		 0x00000081 //IPC1+IPC8
#define SIGGEN_ENA_DIS		 0x00000101 //IPC1+IPC9
#define SIGGEN_CONFIG		 0x00000201 //IPC1+IPC10
#define DPMODULES_CONFIG	 0x00000401 //IPC1+IPC11
#define SAMPLES_BUFFER_ONOFF 0x00000801 //IPC1+IPC12
#define RESET_INTERLOCKS	 0x00001001 //IPC1+IPC13
#define CTOM_MESSAGE_ERROR	 0x80000001 //IPC1+IPC32
#define WFMREF_SYNC			 0x00000002 //IPC2
#define SOFT_INTERLOCK		 0x00000004 //IPC3
#define HARD_INTERLOCK		 0x00000008 //IPC4

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


typedef enum {NO_ERROR_CTOM,ERROR1, ERROR2, ERROR3, ERROR4} 	eCTOMerror;
typedef enum {NO_ERROR_MTOC,INVALID_SLOWREF_UPDATE, INVALID_DP_MODULE, ERROR7, ERROR8} 	eMTOCerror;

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
	TEST_HRADC
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

typedef struct
{
	float	   *PtrBufferStart;
	float	   *PtrBufferEnd;
	float	   *PtrBufferK;
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
	 tPSModuleMtoC PSModule;
     tWfmRef       WfmRef;
     tSigGen       SigGen;
     tDPModule     DPModule;
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

