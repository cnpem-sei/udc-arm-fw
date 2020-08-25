/******************************************************************************
 * Copyright (C) 2019 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file wfmref.c
 * @brief Waveform references module
 * 
 * This module implements waveform references functionality.
 *
 * @author gabriel.brunheira
 * @date 25/09/2019
 *
 */
#include <math.h>
#include "communication_drivers/common/structs.h"
#include "communication_drivers/control/wfmref/wfmref.h"
#include "communication_drivers/ipc/ipc_lib.h"

#pragma DATA_SECTION(g_wfmref_data,"SHARERAMS2345")
volatile u_wfmref_data_t g_wfmref_data;

void init_wfmref(wfmref_t *p_wfmref, uint16_t wfmref_selected,
                 sync_mode_t sync_mode, float freq_lerp, float freq_wfmref,
                 float gain, float offset, float *p_start, uint16_t size,
                 float *p_out)
{
    uint16_t i;

    p_wfmref->wfmref_selected.u16 = wfmref_selected;
    p_wfmref->sync_mode.u16 = sync_mode;
    p_wfmref->gain.f = gain;
    p_wfmref->offset.f = offset;
    p_wfmref->p_out = (float *) ipc_mtoc_translate((uint32_t) p_out);

    for(i = 0; i < NUM_WFMREF_CURVES; i++)
    {
        init_buffer(&p_wfmref->wfmref_data[i], p_start + i * size, size);

        /// Convert WfmRef pointers to C28 memory mapping
        p_wfmref->wfmref_data[i].p_buf_start.p_f =
                (float *) ipc_mtoc_translate((uint32_t) p_wfmref->wfmref_data[i].p_buf_start.p_f);

        p_wfmref->wfmref_data[i].p_buf_idx.p_f =
                        (float *) ipc_mtoc_translate((uint32_t) (p_wfmref->wfmref_data[i].p_buf_end.p_f + 1));

        p_wfmref->wfmref_data[i].p_buf_end.p_f =
                (float *) ipc_mtoc_translate((uint32_t) p_wfmref->wfmref_data[i].p_buf_end.p_f);
    }

    p_wfmref->lerp.counter = 0;
    p_wfmref->lerp.max_count = (uint16_t) roundf(freq_lerp / freq_wfmref);
    p_wfmref->lerp.freq_lerp = freq_lerp;
    p_wfmref->lerp.freq_base.f = freq_wfmref;
    p_wfmref->lerp.inv_decimation = freq_wfmref / freq_lerp;
    p_wfmref->lerp.out = 0.0;
}
