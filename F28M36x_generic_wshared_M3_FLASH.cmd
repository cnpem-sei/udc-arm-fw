/*
//###########################################################################
// FILE:    F28M36x_generic_wshared_M3_FLASH.cmd
// TITLE:   Linker Command File for F28M36H63C2 examples that run from FLASH
//          Keep in mind that C0 and C1 are protected by the code
//          security module.
//          What this means is in most cases you will want to move to
//          another memory map file which has more memory defined.
//###########################################################################
// $TI Release: F28M36x Support Library v206 $
// $Release Date: Thu Mar  5 10:16:50 CST 2015 $
// $Copyright: Copyright (C) 2012-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################
*/

--retain=g_pfnVectors

/* The following command line options are set as part of the CCS project.    */
/* If you are building using the command line, or for some reason want to    */
/* define them here, you can uncomment and modify these lines as needed.     */
/* If you are using CCS for building, it is probably better to make any such */
/* modifications in your CCS project and leave this file alone.              */
/*                                                                           */
/* --heap_size=0                                                             */
/* --stack_size=256                                                          */
/* --library=rtsv7M3_T_le_eabi.lib                                           */

/* The following options allow the user to program Z1 and Z2 DCSM security   */
/* values, include CSM PSWD, ECSL PSWD, GRABSECT, GRABRAM, and FLASH EXEONLY */
/* The driverlib/dcsm_z1_secvalues.s and driverlib/dcsm_z2_secvalues.s files */
/* must be included in the Flash project for the below 2 lines to take       */
/* effect.                                                                   */
--retain=dcsm_z1_secvalues.obj(.z1secvalues,.z1_csm_rsvd)
--retain=dcsm_z2_secvalues.obj(.z2secvalues,.z2_csm_rsvd)

/* System memory map */

MEMORY
{
    /* Flash Block 0, Sector 0 Z1 CSM */
    CSM_ECSL_Z1     : origin = 0x00200000, length = 0x0024
    CSM_RSVD_Z1     : origin = 0x00200024, length = 0x000C
    
    /* Flash Block 0, Sector 0 */
    RESETISR (RX)   : origin = 0x00200030, length = 0x0008   /* Reset ISR is mapped to boot to Flash location */
    INTVECS (RX)    : origin = 0x00201000, length = 0x0258
    FLASHLOAD (RX)  : origin = 0x00201258, length = 0x6DA8   /* For storing code in Flash to copy to RAM at runtime */
    
    /* Flash Block 0, Sector 1 to Flash Block 0, Sector 13 */
    FLASH (RX)      : origin = 0x00208000, length = 0xF7E00  /* Sector 1 thru Sector 13 (minus Z2 CSM) */
    
    /* Flash Block 0, Sector 13 Z2 CSM*/
    CSM_RSVD_Z2     : origin = 0x002FFE0, length = 0x01DC
    CSM_ECSL_Z2     : origin = 0x002FFFDC, length = 0x0024
    
    /* RAM */
    C0 (RWX)        : origin = 0x20000000, length = 0x2000
    C1 (RWX)        : origin = 0x20002000, length = 0x2000
    BOOT_RSVD (RX)  : origin = 0x20004000, length = 0x0FF8
    C2 (RWX)        : origin = 0x200051B0, length = 0x0E50
    C3 (RWX)        : origin = 0x20006000, length = 0x2000
    S0_0 (RWX)        : origin = 0x20008000, length = 0x1000
    S0_1 (RWX)        : origin = 0x20009000, length = 0x1000
    S1_0 (RWX)        : origin = 0x2000A000, length = 0x1000
    S1_1 (RWX)        : origin = 0x2000B000, length = 0x1000
    //S2 (RWX)        : origin = 0x2000C000, length = 0x2000
    //S3 (RWX)        : origin = 0x2000E000, length = 0x2000
    //S4 (RWX)        : origin = 0x20010000, length = 0x2000
    //S5 (RWX)        : origin = 0x20012000, length = 0x2000
    //S23 (RWX)       : origin = 0x2000C000, length = 0x4000
    //S45 (RWX)       : origin = 0x20010000, length = 0x4000
    //S6 (RWX)        : origin = 0x20014000, length = 0x2000
    //S7 (RWX)        : origin = 0x20016000, length = 0x2000
    S2345 (RWX)     : origin = 0x2000C000, length = 0x8000
    S67 (RWX)       : origin = 0x20014000, length = 0x4000
    C0408  (RWX)        : origin = 0x20018000, length = 0xA000
    //C4  (RWX)        : origin = 0x20018000, length = 0x2000
    //C5  (RWX)        : origin = 0x2001A000, length = 0x2000
    //C6  (RWX)        : origin = 0x2001C000, length = 0x2000
    //C7  (RWX)        : origin = 0x2001E000, length = 0x2000
    //C8  (RWX)        : origin = 0x20020000, length = 0x2000
    //C9  (RWX)        : origin = 0x20022000, length = 0x2000
    //C10 (RWX)        : origin = 0x20024000, length = 0x2000
    //C11 (RWX)        : origin = 0x20026000, length = 0x2000
    //C12 (RWX)        : origin = 0x20028000, length = 0x2000
    C0913 (RWX)        : origin = 0x20022000, length = 0xA000
    //C13 (RWX)        : origin = 0x2002A000, length = 0x2000
    //C14 (RWX)        : origin = 0x2002C000, length = 0x2000
    //C15 (RWX)        : origin = 0x2002E000, length = 0x2000
    C1415 (RWX)		 : origin = 0x2002C000, length = 0x4000
    
    CTOMRAM (RX)    : origin = 0x2007F000, length = 0x0800
    MTOCRAM (RWX)   : origin = 0x2007F800, length = 0x0800
	
	OTPSECLOCK       : origin = 0x00681000, length = 0x0004
    OTP_Reserved1    : origin = 0x00681004, length = 0x0004
    OTP_Reserved2    : origin = 0x00681008, length = 0x0004
    OTP_Z2_FLASH_START_ADDR             : origin = 0x0068100C, length = 0x0004
    OTP_EMACID       : origin = 0x00681010, length = 0x0008
    OTP_Reserved3    : origin = 0x00681018, length = 0x0004
    CUSTOMER_OTP_MAIN_OSC_CLK_FREQ      : origin = 0x0068101C, length = 0x0004
    OTP_Reserved4    : origin = 0x00681020, length = 0x0004
    OTP_BOOT_MODE_GPIO_CONFIGURE        : origin = 0x00681024, length = 0x0004
    OTP_Reserved5    : origin = 0x00681028, length = 0x0004
    OTP_ENTRY_POINT  : origin = 0x0068102C, length = 0x0004
    OTP_Reserved6    : origin = 0x00681030, length = 0x0010
}

/* Section allocation in memory */

SECTIONS
{
    .intvecs:   > INTVECS, ALIGN(8)
    .resetisr:  > RESETISR, ALIGN(8)
    .text   :   > FLASH, crc_table(AppCrc, algorithm=CRC32_PRIME), ALIGN(8)
    .const  :   > FLASH, crc_table(AppCrc, algorithm=CRC32_PRIME), ALIGN(8)
    .cinit  :   > FLASH, crc_table(AppCrc, algorithm=CRC32_PRIME), ALIGN(8)
    .pinit  :   > FLASH, crc_table(AppCrc, algorithm=CRC32_PRIME), ALIGN(8)

    .vtable :   >  C0 | C1 | C2 | C3
    .data   :   >  C2 | C3
    .bss    :   >  C0408// | C5 | C6 | C7 | C8
    .sysmem :   >  C0 | C1 | C2 | C3
    .stack  :   >  C0 | C1 | C2 | C3
    
    .TI.crctab : > FLASH, ALIGN(8)
    
    .z1secvalues  :   >  CSM_ECSL_Z1, ALIGN(8)
    .z1_csm_rsvd  :   >  CSM_RSVD_Z1, ALIGN(8)
    .z2secvalues  :   >  CSM_ECSL_Z2, ALIGN(8)
    .z2_csm_rsvd  :   >  CSM_RSVD_Z2, ALIGN(8)
    
    
     ramfuncs            : LOAD = FLASHLOAD,
                           RUN = C0,
                           LOAD_START(RamfuncsLoadStart),
                           LOAD_SIZE(RamfuncsLoadSize),
                           LOAD_END(RamfuncsLoadEnd),
                           RUN_START(RamfuncsRunStart),
                           RUN_SIZE(RamfuncsRunSize),
                           RUN_END(RamfuncsRunEnd),
                           crc_table(AppCrc, algorithm=CRC32_PRIME),
                           PAGE = 0, ALIGN(8)
 
    SHARERAMS0_0    : > S0_0   // g_controller_mtoc
    SHARERAMS0_1    : > S0_1   // g_param_bank
    SHARERAMS1_0  	: > S1_0   // g_controller_ctom
    SHARERAMS1_1  	: > S1_1   // HRADCs_Info
    //SHARERAMS2  	: > S2
    //SHARERAMS3  	: > S3
    //SHARERAMS4  	: > S4
    //SHARERAMS5  	: > S5
    //SHARERAMS6  	: > S6
    //SHARERAMS7  	: > S7
    //SHARERAMS23  	: > S23   // g_wfmref
    //SHARERAMS45  	: > S45   // g_buf_samples_ctom
    SHARERAMS2345   : > S2345 // g_wfmref_data
    SHARERAMS67  	: > S67   // g_buf_samples_ctom

 
 	SERIALBUFFER : > C0913
	ETHERNETBUFFER : > C1415

 	MTOC_MSG_RAM : > MTOCRAM
    CTOM_MSG_RAM : TYPE = DSECT > CTOMRAM
/*
    GROUP : > MTOCRAM
    {
        PUTBUFFER  
        PUTWRITEIDX
        GETREADIDX  
    }

    GROUP : > CTOMRAM 
    {
        GETBUFFER : TYPE = DSECT
        GETWRITEIDX : TYPE = DSECT
        PUTREADIDX : TYPE = DSECT
    }    
*/
}

__STACK_TOP = __stack + 256;
