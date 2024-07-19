#define STACK_SIZE_PER_CPU			0x200000

.section .entry, "ax"

.globl	_start
_start:
	/* check CPU ID */
	mrs x0, mpidr_el1
	and x0, x0, #0xF			/* CPU number */
	mov x20, x0 /* Place cpu number in x20 */

	/* configure stack */
	ldr x0, =stacktop
	mov x1, #STACK_SIZE_PER_CPU
	add x2, x20, #1
	madd x0, x1, x2, x0 /* x0 = x0 + (x1 * x2) */
	msr SPSel, #0  // switch to SP_EL0
	mov sp, x0

	msr SPSel, #1  // switch to SP_ELn
	sub x0, x0, #0x5000
	mov sp, x0
	msr SPSel, #0  // switch to SP_ELn

	cmp x20, xzr
	bne other_cpu

boot_cpu:
	/* Clean bss section */
	ldr x0, =_bss_start
	ldr x1, =_bss_end

bss_loop:
	cmp x0, x1
	b.eq bss_end_loop
	str xzr, [x0], #8
	b bss_loop
bss_end_loop:

	mrs x0, CurrentEL

	mov x1, sp

	/* Enable Interrupts */
	msr DAIFclr, #0b0010

	b main

other_cpu:
	/* Enable Interrupts */
	msr DAIFclr, #0b0010

	b secondary_cores_main
