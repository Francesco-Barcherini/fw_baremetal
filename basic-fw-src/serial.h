/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <inttypes.h>

void _itoa(char* buffer, int base, uint64_t value);

int puts_no_lock(const char *str);

int puts(const char *str);

int put(const char str);

#endif
