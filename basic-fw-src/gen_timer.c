/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#include "gen_timer.h"

uint64_t gen_timer_get_freq(void){
    uint64_t _freq;

    asm volatile("mrs %0, cntfrq_el0": "=r" (_freq));

    return _freq;
}

uint64_t gen_timer_get_vtimestamp(void){
    uint64_t _timestamp;

    asm volatile("mrs %0, cntvct_el0": "=r" (_timestamp));

    return _timestamp;
}

uint64_t gen_timer_get_ptimestamp(void){
    uint64_t _timestamp;

    asm volatile("mrs %0, cntpct_el0": "=r" (_timestamp));

    return _timestamp;
}

void gen_timer_arm_vtimer_global(uint64_t value){
	asm volatile("msr CNTV_CVAL_EL0, %0":: "r" (value));
}

void gen_timer_arm_ptimer_global(uint64_t value){
	asm volatile("msr CNTP_CVAL_EL0, %0":: "r" (value));
}

void gen_timer_arm_vtimer_rel(uint64_t value){
    asm volatile("msr CNTV_TVAL_EL0, %0":: "r" (value));
}

void gen_timer_arm_ptimer_rel(uint64_t value){
    asm volatile("msr CNTP_TVAL_EL0, %0":: "r" (value));
}

void gen_timer_enable_vtimer(void){
	asm volatile("msr CNTV_CTL_EL0, %0":: "r" (0x1));
}

void gen_timer_enable_ptimer(void){
	asm volatile("msr CNTP_CTL_EL0, %0":: "r" (0x1));
}

void gen_timer_disable_vtimer(void){
	asm volatile("msr CNTV_CTL_EL0, %0":: "r" (0x2));
}

void gen_timer_disable_ptimer(void){
	asm volatile("msr CNTP_CTL_EL0, %0":: "r" (0x2));
}
