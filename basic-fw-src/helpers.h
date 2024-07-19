/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#ifndef HELPERS_H
#define HELPERS_H

#include <inttypes.h>

/*------------------------------------------------------------------*/
/*------------------------[ M A C R O S ]---------------------------*/
/*------------------------------------------------------------------*/

#define stringify(s)		#s

/**
 * Move to gp register from system register.
 */
#define arch_mrs(variable, register)	\
		__asm__ volatile("mrs %0, "stringify(register): "=r"(variable))


/**
 * Move to system register from gp register.
 */
#define arch_msr(register, variable)	\
		__asm__ volatile("msr "stringify(register) ", %0":: "r"(variable))

/*------------------------------------------------------------------*/
/*---------------------------[ M M U ]------------------------------*/
/*------------------------------------------------------------------*/
static inline void set_ttbr0(uint64_t address){
	arch_msr(ttbr0_el1, address);
}

static inline void set_ttbr1(uint64_t* address){
	arch_msr(ttbr1_el1, address);
}

static inline void raw_wait_func(uint64_t num){
    uint64_t w = 1;
    for (uint64_t i = 0; i < num*8000000; i++) {
        w = w * 1372;
        w = w / 183;
        w = w % 20;
    }
}

#endif /* HELPERS_H */
