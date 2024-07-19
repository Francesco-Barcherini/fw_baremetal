/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

.macro  pushm, reg1, reg2
	stp     \reg1, \reg2, [sp, #-16]!
.endm

.macro  popm, reg1, reg2
	ldp     \reg1, \reg2, [sp], #16
.endm

.section .entry, "ax"

.global	__start
__start:
	/* Install vector table */
	adr x0, _vector_table
	msr VBAR_EL1, x0

	/* Disable trapping FPU usage */
	mov x0, #(3 << 20)
	msr cpacr_el1, x0 

	/* Disable address allignment check */
	mrs x0, sctlr_el1
	bic x0, x0, #(1 << 1)
	msr sctlr_el1, x0

	b _start

	/* We should never reach here */
	b .

.macro	interrupt_routine label
	pushm x0, x1
	pushm x2, x3
	pushm x4, x5
	pushm x6, x7
	pushm x8, x9
	pushm x10, x11
	pushm x12, x13
	pushm x14, x15
	pushm x16, x17
	pushm x18, x19
	pushm x20, x21
	pushm x22, x23
	pushm x24, x25
	pushm x26, x27
	pushm x28, x29
	pushm x30, xzr
	mrs x0, spsr_el1
	mrs x1, elr_el1
	pushm x0, x1
	bl	\label
	popm x0, x1
	msr spsr_el1, x0
	msr elr_el1, x1
	popm x30, xzr
	popm x28, x29
	popm x26, x27
	popm x24, x25
	popm x22, x23
	popm x20, x21
	popm x18, x19
	popm x16, x17
	popm x14, x15
	popm x12, x13
	popm x10, x11
	popm x8, x9
	popm x6, x7
	popm x4, x5
	popm x2, x3
	popm x0, x1
	eret
.endm

sync_handler_asm:
	interrupt_routine sync_handler

irq_handler_asm:
	interrupt_routine irq_handler

fiq_handler_asm:
	interrupt_routine fiq_handler

serror_handler_asm:
	interrupt_routine serror_handler


.macro	vector_entry label
	.align	7
	b	\label
.endm

.align	11
	.globl _vector_table
_vector_table:
	vector_entry	sync_handler_asm	/* Synchronous EL1t */
	vector_entry	irq_handler_asm		/* IRQ EL1t */
	vector_entry	fiq_handler_asm		/* FIQ EL1t */
	vector_entry	serror_handler_asm	/* Error EL1t */

	vector_entry	sync_handler_asm	/* Synchronous EL1h */
	vector_entry	irq_handler_asm		/* IRQ EL1h */
	vector_entry	fiq_handler_asm		/* FIQ EL1h */
	vector_entry	serror_handler_asm	/* Error EL1h */

	vector_entry	.					/* Synchronous 64-bit EL0 */
	vector_entry	.					/* IRQ 64-bit EL0 */
	vector_entry	.					/* FIQ 64-bit EL0 */
	vector_entry	.					/* Error 64-bit EL0 */

	vector_entry	.					/* Synchronous 32-bit EL0 */
	vector_entry	.					/* IRQ 32-bit EL0 */
	vector_entry	.					/* FIQ 32-bit EL0 */
	vector_entry	.					/* Error 32-bit EL0 */
