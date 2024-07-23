/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#ifndef __REG_H__
#define __REG_H__

/* UART */

#define UART_BASE_REG					(uintptr_t)(0xE6540000)
#define UART_SIZE						(0x60)

/* GIC Macros */

#define GICD_BASE_ADDRESS				0xf9010000
#define GICD_SIZE						0x10000

#define GICC_BASE_ADDRESS				0xf9020000
#define GICC_SIZE						0x20000

#define GICR_BASE_ADDRESS				0x2f100000
#define GICR_SIZE						0x200000

#endif
