
#include <handlers.h>
#include <serial.h>
#include <gicv3.h>
#include <cbfw_printf.h>
#include <gen_timer.h>

void sync_handler(void){
	// bprintf("HERE WE ARE------SYNC GUEST\n");
	while(1);
}

int64_t my_handler(uint64_t iar){

	/*****************************/
	/* PUT YOU HANDLER CODE HERE */
	/*****************************/

	return 0;
}


void irq_handler(void){
	/* Dispatch the interrupt */
	arm_gic_dispatch_interrupt(my_handler);
}

void fiq_handler(void){
	// bprintf("HERE WE ARE------FIQ GUEST\n");
}

void serror_handler(void){
	// bprintf("HERE WE ARE------SERROR GUEST\n");
}
