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
 * @file control.c
 * @brief Brief description of module
 * 
 * Detailed description
 *
 * @author gabriel.brunheira
 * @date 27/11/2017
 *
 * TODO: insert comments
 */

#include <math.h>
#include "control.h"

#pragma DATA_SECTION(g_controller_mtoc,"SHARERAMS0_0");
#pragma DATA_SECTION(g_controller_ctom,"SHARERAMS1_0");

volatile control_framework_t g_controller_ctom;
volatile control_framework_t g_controller_mtoc;

void init_control_framework(volatile control_framework_t *p_controller)
{
    uint16_t i;

    for(i = 0; i < NUM_MAX_NET_SIGNALS; i++)
    {
        p_controller->net_signals[i].f = 0.0;
    }

    for(i = 0; i < NUM_MAX_OUTPUT_SIGNALS; i++)
    {
        p_controller->output_signals[i].f = 0.0;
    }
}

uint8_t set_dsp_coeffs(volatile control_framework_t *p_controller,
                       dsp_class_t dsp_class, uint16_t id, float *p_coeffs)
{
    switch(dsp_class)
    {
        case DSP_SRLim:
        {
            //memcpy(&p_controller->dsp_modules.dsp_srlim[id].coeffs.f, p_coeffs,
            //       4*NUM_COEFFS_DSP_SRLIM);
            cfg_dsp_srlim(&p_controller->dsp_modules.dsp_srlim[id], *(p_coeffs));
            return 1;
        }

        case DSP_LPF:
        {
            //memcpy(&p_controller->dsp_modules.dsp_lpf[id].coeffs.f, p_coeffs,
            //       4*NUM_COEFFS_DSP_LPF);
            cfg_dsp_lpf(&p_controller->dsp_modules.dsp_lpf[id], *(p_coeffs));
            return 1;
        }

        case DSP_PI:
        {
            // TODO: this memcpy is going to Fault_ISR when size >
            //memcpy(&p_controller->dsp_modules.dsp_pi[id].coeffs.f, p_coeffs,
            //       8);
            cfg_dsp_pi(&p_controller->dsp_modules.dsp_pi[id], *(p_coeffs),
                       *(p_coeffs+1), *(p_coeffs+2), *(p_coeffs+3));
            return 1;
       }
       case DSP_IIR_2P2Z:
       {
            //memcpy(&p_controller->dsp_modules.dsp_iir_2p2z[id].coeffs.f,
            //       p_coeffs, 4*NUM_COEFFS_DSP_IIR_2P2Z);
            cfg_dsp_iir_2p2z(&p_controller->dsp_modules.dsp_iir_2p2z[id],
                             *(p_coeffs), *(p_coeffs+1), *(p_coeffs+2),
                             *(p_coeffs+3), *(p_coeffs+4), *(p_coeffs+5),
                             *(p_coeffs+6));
            return 1;
        }

        case DSP_IIR_3P3Z:
        {
            //memcpy(&p_controller->dsp_modules.dsp_iir_3p3z[id].coeffs.f,
            //       p_coeffs, 4*NUM_COEFFS_DSP_IIR_3P3Z);
            cfg_dsp_iir_3p3z(&p_controller->dsp_modules.dsp_iir_3p3z[id],
                             *(p_coeffs), *(p_coeffs+1), *(p_coeffs+2),
                             *(p_coeffs+3), *(p_coeffs+4), *(p_coeffs+5),
                             *(p_coeffs+6), *(p_coeffs+7), *(p_coeffs+8));
            return 1;
        }

        case DSP_VdcLink_FeedForward:
        {
            //memcpy(&p_controller->dsp_modules.dsp_ff[id].coeffs.f, p_coeffs,
            //       4*NUM_COEFFS_DSP_VDCLINK_FF);
            cfg_dsp_vdclink_ff(&p_controller->dsp_modules.dsp_ff[id],
                               *(p_coeffs), *(p_coeffs+1));
            return 1;
        }

        default:
            return 0;
    }
}

float get_dsp_coeff(volatile control_framework_t *p_controller,
                    dsp_class_t dsp_class, uint16_t id, uint16_t coeff)
{
    switch(dsp_class)
    {
        case DSP_SRLim:
        {
            return p_controller->dsp_modules.dsp_srlim[id].coeffs.f[coeff];
        }

        case DSP_LPF:
        {
            return p_controller->dsp_modules.dsp_lpf[id].coeffs.f[coeff];
        }

        case DSP_PI:
        {
            return p_controller->dsp_modules.dsp_pi[id].coeffs.f[coeff];
        }
        case DSP_IIR_2P2Z:
        {
            return p_controller->dsp_modules.dsp_iir_2p2z[id].coeffs.f[coeff];
        }

        case DSP_IIR_3P3Z:
        {
            return p_controller->dsp_modules.dsp_iir_3p3z[id].coeffs.f[coeff];
        }

        case DSP_VdcLink_FeedForward:
        {
            return p_controller->dsp_modules.dsp_ff[id].coeffs.f[coeff];
        }

        default:
            return NAN;
    }
}
