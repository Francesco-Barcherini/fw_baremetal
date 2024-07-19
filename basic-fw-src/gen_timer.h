/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#ifndef __GEN_TIMER_H__
#define __GEN_TIMER_H__

#include <inttypes.h>

uint64_t gen_timer_get_freq(void);

uint64_t gen_timer_get_vtimestamp(void);

uint64_t gen_timer_get_ptimestamp(void);

void gen_timer_arm_vtimer_global(uint64_t value);

void gen_timer_arm_ptimer_global(uint64_t value);

void gen_timer_arm_vtimer_rel(uint64_t value);

void gen_timer_arm_ptimer_rel(uint64_t value);

void gen_timer_enable_vtimer(void);

void gen_timer_enable_ptimer(void);

void gen_timer_disable_vtimer(void);

void gen_timer_disable_ptimer(void);

#endif /* __GEN_TIMER_H__ */