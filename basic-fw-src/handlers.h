/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#ifndef __HANDLERS_H__
#define __HANDLERS_H__

void sync_handler(void);

void irq_handler(void);

void fiq_handler(void);

void serror_handler(void);

#endif /* __HANDLERS_H__ */