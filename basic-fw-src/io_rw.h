/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#ifndef __IO_RW_H__
#define __IO_RW_H__

#include <inttypes.h>

static inline void io_write_8(uint8_t value, uintptr_t addr){
	*(volatile uint8_t*)addr = value;
}

static inline uint8_t io_read_8(uintptr_t addr){
	return *(volatile uint8_t*)addr;
}

static inline void io_write_16(uint16_t value, uintptr_t addr){
	*(volatile uint16_t*)addr = value;
}

static inline uint16_t io_read_16(uintptr_t addr){
	return *(volatile uint16_t*)addr;
}

static inline void io_write_32(uint32_t value, uintptr_t addr){
	*(volatile uint32_t*)addr = value;
}

static inline uint32_t io_read_32(uintptr_t addr){
	return *(volatile uint32_t*)addr;
}

static inline void io_write_64(uint64_t value, uintptr_t addr){
	*(volatile uint64_t*)addr = value;
}

static inline uint64_t io_read_64(uintptr_t addr){
	return *(volatile uint64_t*)addr;
}

#endif /* __IO_RW_H__ */