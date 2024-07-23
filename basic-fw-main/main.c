#include <reg.h>
#include <inttypes.h>
#include <serial.h>
#include <gicv3.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <cbfw_printf.h>
#include <psci.h>
#include <gen_timer.h>
#include "ttbl.h"

#define RAM_SIZE 0x2000000

#define ARMV8_PMCR_P (1 << 1)

// Entry point of the guest
extern void __start();
volatile uint64_t timestamp_t0;

int size = 10000;
int arr[10000];

void secondary_cores_main(void){

    bprintf("Hello. I am the Guest 1 from a secondary core\n");

    arm_gic_init();

    /***************************/
    /* PUT YOUR USER CODE HERE */
    /*   FOR THE SECOND CORE   */
    /***************************/

    while(1) asm volatile("wfi");
}

// static void setup_mmu()
// {
//     if(create_pagetable_flat_single_section(0, RAM_SIZE) < 0){
//         bprintf("ERROR: create pagetable\n");
//     }

//     if(pagetable_add_device_section(GICD_BASE_ADDRESS, GICD_SIZE) < 0){
//         bprintf("ERROR: add device section GICD\n");
//     }

//     if(pagetable_add_device_section(GICC_BASE_ADDRESS, GICC_SIZE) < 0){
//         bprintf("ERROR: add device section GICC\n");
//     }

//     if(pagetable_add_device_section(UART_BASE_REG, UART_SIZE) < 0){
//         bprintf("ERROR: add device section UART\n");
//     }

//     /*if (pagetable_add_memory_section((paddr_t)shared_buff, elem_size * n_elems, 0, 1, 1) < 0) {
//         bprintf("ERROR: add memory section\n");
//     }*/

//     if(enable_mmu() < 0){
//         bprintf("ERROR: add device section GICC\n");
//     }
// }

void main(unsigned int REG, uint64_t sp)
{
    uart_rcar_configure();
    
    bprintf("Hello. I am the Guest 1\n");

    // bprintf("Guest: initializing MMU\n");
    // setup_mmu();

    bprintf("Guest: initializing GIC\n");
    arm_gic_init();

    /* Turning on the secondary core */
    // psci_cpu_on(1, (void*)__start);

    bprintf(WELCOME_STRING);

    /* Enable L1 data cache, setting bit 2 in System Control Register*/
    uint32_t valReg=0;
    asm volatile("mrs %0, sctlr_el1" : "=r" (valReg));
    asm volatile("msr sctlr_el1, %0" : : "r" (valReg|0x04));

    //srand(gen_timer_get_ptimestamp());

    /* Performance Monitors Control Register: reset counters */
    uint64_t val=0;
    asm volatile("mrs %0, pmcr_el0" : "=r" (val));
    asm volatile("msr pmcr_el0, %0" : : "r" (val|ARMV8_PMCR_P));
    asm volatile("isb");

    while(1){
        bprintf("Guest 1 is running\n");
        asm volatile("wfi");
    }

    /* Turning off the entire VM */
    psci_system_off();
}