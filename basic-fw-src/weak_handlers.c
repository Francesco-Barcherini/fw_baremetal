/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#include "handlers.h"

#define __weak              __attribute__((weak))

void __weak sync_handler(void){}

void __weak irq_handler(void){}

void __weak fiq_handler(void){}

void __weak serror_handler(void){}
