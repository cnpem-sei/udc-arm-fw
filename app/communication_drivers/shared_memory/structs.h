/*
 * structs.h
 *
 *  Created on: 23/07/2015
 *      Author: joao.rosa
 */

#include <stdint.h>

/* Library-wide limits */

#define N_MAX_NET_SIGNALS	32
#define N_MAX_TS_COUNTERS	4

#define N_MAX_ELP_ERROR		4
#define N_MAX_ELP_SRLim		3
#define N_MAX_ELP_PI_DAWU	6
#define	N_MAX_ELP_IIR_2P2Z	6
#define N_MAX_ELP_IIR_3P3Z  3
#define N_MAX_ELP_LPF		4
#define N_MAX_TI_PID		3
#define N_MAX_TI_PI			3
#define N_MAX_TI_DF13		3
#define N_MAX_TI_DF22		3
#define N_MAX_TI_DF23		3

//##################################################
#define N_MAX_PWM_MODULES			8
//##################################################


#ifndef STRUCTS_H_
#define STRUCTS_H_

typedef enum {
       Iout_Ref,
       Iout_RefBuffer,
       Iout_FOFB
} enum_Iout_Ref_mode;

typedef struct {

	//float 		freq;
	//float 		amplitude;
	//float 		offset;
	//uint16_t	status;

	union
	{
	   float f;
	   uint8_t u[4];
	}freq;

	union
	{
		float f;
		uint8_t u[4];
	}amplitude;

	union
	{
		float f;
		uint8_t u[4];
	}offset;

	union
	{
		uint16_t d;
		uint8_t u[2];
	}status;

} sinGen_t;

typedef struct {
       enum_Iout_Ref_mode  Iout_Ref_mode;

       float  Iout_Ref;
       float  *Iout_RefBuffer;
       float  size_Iout_RefBuffer;
       float Kp;
       float Ki;
       float  u_max;
       uint16_t	ctrl_loop;
       uint16_t	ps_sts;
} shm_m2c_param_ctrl_t;

typedef struct
{
	float Iout;
	uint16_t ps_sts;
} shm_c2m_output_var_t;

extern shm_m2c_param_ctrl_t shm_m2c_param_ctrl;

extern shm_c2m_output_var_t shm_c2m_output_var;

typedef volatile struct
{
	volatile float *pos;
	volatile float *neg;
	volatile float *error;
} tELP_Error;

typedef volatile struct
{
	uint16_t bypass;
	float dI_max;
	volatile float *in;
	volatile float *out;
} tELP_SRLim;

typedef volatile struct
{
	float K;
	float a;
	float in_old;
	volatile float *in;
	volatile float *out;
} tELP_LPF;

typedef volatile struct
{
	float Kp;
	float Ki;
	float FreqSample;
	float Umax;
	float Umin;
    float up;
    float ui;
	volatile float *in;
	volatile float *out;
} tELP_PI_dawu;

typedef volatile struct
{
	float Umax;
	float Umin;
	float b0;
	float b1;
	float b2;
	float a1;
	float a2;
	float w1;
	float w2;
	volatile float *in;
	volatile float *out;
} tELP_IIR_2P2Z;

typedef volatile struct
{
	float Umax;
	float Umin;
	float b0;
	float b1;
	float b2;
	float b3;
	float a1;
	float a2;
	float a3;
	float w1;
	float w2;
	float w3;
	volatile float *in;
	volatile float *out;
} tELP_IIR_3P3Z;

typedef volatile struct {
	float Kp;		// proportional gain
	float Ki;		// integral gain
	float Kd;		// derivative gain
	float Kr;		// set point weight
	float c1;		// D filter coefficient 1
	float c2;		// D filter coefficient 2
	float d2;		// D filter storage 1
	float d3;		// D filter storage 2
	float i10;		// I storage
	float i14;		// sat storage
	float Umax;		// upper saturation limit
	float Umin;		// lower saturation limit
} PID;

typedef volatile struct {
	float Kp;		// [0] proportional gain
 	float Ki;		// [2] integral gain
 	float i10;		// [4] I storage
	float Umax;		// [6] upper saturation limit
	float Umin;		// [8] lower saturation limit
	float i6;		// [A] saturation storage
} PI;

typedef volatile struct {
	// coefficients
	float b0;	// [0] b0
	float b1;	// [2] b1
	float b2;	// [4] b2
	float b3;	// [6] b3
	float a0;	// [8] a0
	float a1;	// [A] a1
	float a2;	// [C] a2
	float a3;	// [E] a3

	//data
	float d0;	// [10] e(k)
	float d1;	// [12] e(k-1)
	float d2;	// [14] e(k-2)
	float d3;	// [16] e(k-3)
	float d4;	// [18] u(k)
	float d5;	// [1A] u(k-1)
	float d6;	// [1C] u(k-2)
	float d7;	// [1E] u(k-3)
} DF13;

typedef volatile struct {
	float b0;	// [0] b0
	float b1;	// [2] b1
	float b2;	// [4] b2
	float a1;	// [6] a1
	float a2;	// [8] a2
	float x1;	// [A] x1
	float x2;	// [C] x2
} DF22;

typedef volatile struct {
	float b0;	// [0] b0
	float b1;	// [2] b1
	float b2;	// [4] b2
	float b3;	// [6] b3
	float a1;	// [8] a1
	float a2;	// [A] a2
	float a3;	// [C] a3
	float x1;	// [E] x1
	float x2;	// [10] x2
	float x3;	// [12] x3
} DF23;

typedef enum
{
	Sine,
	Square,
	Triangle,
	FreqSweep,
	DampedSine
} eSigGenType;

/*typedef struct {
	uint16_t 		OnOff;
	eSigGenType	Type;
	eSigGenType	TypeNew;
	float 		FreqSignal;
	float 		FreqSignalNew;
	float 		FreqSample;
	float 		Gain;
	float 		GainNew;
	float 		Offset;
	float 		OffsetNew;
	float 		n;
	float 		nSamples;
	float 		w;
	float 		*out;
} tELP_SigGen;*/

typedef volatile struct tELP_SigGen tELP_SigGen;

struct tELP_SigGen {
	uint16_t		Enable;
	eSigGenType		Type;
	uint16_t		nCycles;
	float			PhaseStart;
	float			PhaseEnd;
	float 			FreqSample;
	float			Aux;
	float 			w;
	float 			n;
	float 			nSamples;
	volatile float 	*ptr_FreqSignal;
	volatile float 	*ptr_Amp;
	volatile float 	*ptr_Offset;
	volatile float	*ptr_Aux;
	volatile float 	*out;
	void			(*Run_ELP_SigGen)(tELP_SigGen *ptr_sg);
};

/* Collection of Digital Control Library (DCL) modules used by Framework */

typedef volatile struct
{
	tELP_Error		ELP_Error[N_MAX_ELP_ERROR];
	tELP_SRLim		ELP_SRLim[N_MAX_ELP_SRLim];
	tELP_LPF		ELP_LPF[N_MAX_ELP_LPF];
	tELP_PI_dawu 	ELP_PI_dawu[N_MAX_ELP_PI_DAWU];
	tELP_IIR_2P2Z	ELP_IIR_2P2Z[N_MAX_ELP_IIR_2P2Z];
	tELP_IIR_3P3Z	ELP_IIR_3P3Z[N_MAX_ELP_IIR_3P3Z];
	PID				TI_PID[N_MAX_TI_PID];
	PI				TI_PI[N_MAX_TI_PI];
	DF13			TI_DF13[N_MAX_TI_DF13];
	DF22			TI_DF22[N_MAX_TI_DF22];
	DF23			TI_DF23[N_MAX_TI_DF23];
} tDP_Library;


/*
 * Time-Slice Manager module.
 *
 * Module responsible for managing the execution of task within
 * the framework ISR with different sample rates
 */

typedef volatile struct
{
	uint16_t Counters[N_MAX_TS_COUNTERS];
	uint16_t FreqRatios[N_MAX_TS_COUNTERS];
} tTimeSlicer;


/*
 * DP Framework entity. This struct groups information regardind a
 * particular DP Framework implementation, including:
 *
 * 		- Pointer to reference signal
 * 		- Set of netlists for modules interconnection
 * 		- Set of DCL modules
 * 		- Time-Slice Manager module
 */

/*typedef volatile struct
{
	volatile float	*Ref;
	float 			NetSignals[N_MAX_NET_SIGNALS];
	float			DutySignals[2*N_MAX_PWM_MODULES];
	tDP_Library		DPlibrary;
	//tELP_SigGen		SigGen;
	tTimeSlicer		TSManager;
} tDP_Framework;*/

typedef volatile struct
{
	volatile float	*Ref;

	union
	{
		uint8_t	u8[4];
		float 	f;
	} NetSignals[N_MAX_NET_SIGNALS];

	union
	{
		uint8_t	u8[4];
		float	f;
	} DutySignals[2*N_MAX_PWM_MODULES];

	tDP_Library		DPlibrary;
	tTimeSlicer		TSManager;

} tDP_Framework;

extern tDP_Framework DP_Framework_MtoC;

extern tDP_Framework DP_Framework;

#endif /* STRUCTS_H_ */
