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
 * @file sdram_mem.c
 * @brief SDRAM module.
 *
 * @author joao.rosa
 *
 * @date 23/01/2017
 *
 */

#include <stdint.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "inc/hw_epi.h"
#include "inc/hw_gpio.h"
#include "inc/hw_nvic.h"

#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/epi.h"

#include "sdram_mem.h"


//*****************************************************************************
//
// Size of memory which is used to run this example. In this case it's
// 64MB/512Mb.
//
//*****************************************************************************
#define MEM_SIZE 0x04000000			//Size in bytes (64MBytes)
//#define MEM_SIZE 0x00800000		//Size in bytes (8MBytes)
#define MEM_SIZ2 (MEM_SIZE>>1)

// Global Variables used in EPI example.
short *XMEM_p;
volatile unsigned long test_fail_cnt;
volatile unsigned long test_cnt;

//*****************************************************************************
//
// This function performs simple read/write accesses to memory.
//
//*****************************************************************************
uint8_t sdram_read_write(void)
{
    unsigned char  sdram_rdb;
    unsigned short sdram_rds;
    unsigned long  sdram_rdl;
    unsigned char  sdram_wdb;
    unsigned short sdram_wds;
    unsigned long  sdram_wdl;
    int i;
    char  *XMEM_pb;
    short *XMEM_ps;
    long  *XMEM_pl;

    //Write data bytes
   	XMEM_pb = (char *)0x60000000;
   	sdram_wdb=0x01;
    for (i=0; i < 4; i++)
    {
        *XMEM_pb++ = sdram_wdb++;
    }

    //Write data short
   	XMEM_ps   = (short *)XMEM_pb;
   	sdram_wds = 0x0605;
    for (i=0; i < 2; i++)
    {
        *XMEM_ps++ = sdram_wds;
        sdram_wds += 0x0202;
    }

    //Write data long
   	XMEM_pl   = (long *)XMEM_ps;
   	sdram_wdl = 0x0C0B0A09;
    for (i=0; i < 2; i++)
    {
        *XMEM_pl++ = sdram_wdl;
        sdram_wdl += 0x04040404;
    }

    //Read bytes
    XMEM_pb = (char *)0x60000000;
    sdram_wdb = 0x01;
    for (i=0; i < 16; i++)
    {
   	    sdram_rdb     = *XMEM_pb;
        if( sdram_rdb!= sdram_wdb)
        {
	        return(1);
        }
        XMEM_pb++;
        sdram_wdb++;
    }

    //Read data short
    XMEM_ps = (short *)0x60000000;
    sdram_wds=0x0201;
    for (i=0; i < 8; i++)
    {
        sdram_rds     = *XMEM_ps;
        if( sdram_rds != sdram_wds)
        {
	        return(1);
        }
        XMEM_ps++;
        sdram_wds += 0x0202;
    }

    //Read data long
   	XMEM_pl= (long *)0x60000000;
   	sdram_wdl=0x04030201;
    for (i=0; i < 4; i++)
    {
   	    sdram_rdl      = *XMEM_pl;
        if( sdram_rdl != sdram_wdl)
        {
		    return(1);
        }
        XMEM_pl++;
        sdram_wdl += 0x04040404;
    }
    return(0);
}

void sdram_init(void)
{
    // Set clock divider to 1 (divide by 2)
    EPIDividerSet(EPI0_BASE, 1);

    // Enable SDRAM mode
    EPIModeSet(EPI0_BASE, EPI_MODE_SDRAM);

    // System clock frq between 50-100MHz
    // Sleep mode disabled.
    // Refresh Count = 0x2EE (default value)
    EPIConfigSDRAMSet(EPI0_BASE, (EPI_SDRAM_CORE_FREQ_50_100 |
EPI_SDRAM_FULL_POWER | EPI_SDRAM_SIZE_512MBIT), 750);

    // External RAM Size (to set proper address range) 256MB
    // External RAM Address = 0x60000000
    EPIAddressMapSet(EPI0_BASE, (EPI_ADDR_RAM_SIZE_256MB | EPI_ADDR_RAM_BASE_6));

    // Wait for EPI SDRAM init sequence to complete
    while(HWREG(EPI0_BASE + EPI_O_STAT) & EPI_STAT_INITSEQ)
    {
    }
}
