/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#define psci_cpu_on(cpu_id, start_address) asm volatile("mov x0, %0\n" \
					"mov x1, %1\n" \
					"mov x2, %2\n" \
					"smc #0":: \
					"r" (0xC4000003), \
					"r" (cpu_id), \
					"r" (start_address))

#define psci_cpu_off() asm volatile("mov x0, %0\n" \
						"smc #0":: "r" (0x84000002));

#define psci_system_off() asm volatile("mov x0, %0\n" \
					"smc #0":: \
					"r" (0x84000008))
