/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#include "gicv3.h"
#include "serial.h"
#include "io_rw.h"
#include "cbfw_printf.h"
#include "helpers.h"
#include "reg.h"

static uint32_t _n_int = 0;
static boolean_t _gic_initialized = FALSE;

struct GICv3_rdist_if* gic_rdist =
    (struct GICv3_rdist_if*)GICR_BASE_ADDRESS;

/* This function returns the Interrupt of the signaled interrupt
 * and acknowledge the CPU Interface */
static uint32_t arm_gic_get_interrupt_id(void){
    uint64_t int_ID;
    arch_mrs(int_ID, ICC_IAR1_EL1);
    return (uint32_t)int_ID;
}

int arm_gic_generate_sgi(uint16_t intID, boolean_t targeted, uint8_t targetList){
    uint32_t sgi_reg = 0;
    int ret = GIC_OK;

    if(FALSE == _gic_initialized){
        ret |= GIC_NOT_INITIALIZED;
    }

    if(GIC_OK == ret){
        /* Compose the SGI Register Bit fields */
        if(FALSE == targeted){
            sgi_reg |= (uint64_t)(1ULL << SGI_IRM_SHIFT);
        } else {
            sgi_reg |= (uint64_t)targetList;
        }
        sgi_reg |= (uint64_t)intID << SGI_INT_ID_SHIFT;

        /* Write into the SGI Register. This will generate the interrupt */
        arch_msr(ICC_SGI1R_EL1, sgi_reg);
    }

    return ret;
}

void arm_gic_enable_int(uint32_t int_ID)
{
    uint64_t cpu_id;
    asm volatile("mrs %0, mpidr_el1": "=r"(cpu_id));
    cpu_id &= 0xF;

    if(int_ID <= MAX_PPI_ID){
        /* SGI or PPI */
        gic_rdist[cpu_id].sgis.GICR_ISENABLER[0] =(1ULL << (int_ID & 0x1f));
    } else if(int_ID <= _n_int){
        /* SPI */
        /* Dividing by 32 to get reg offset */
        const uint8_t offset = int_ID >> 5;

        /* Modulo 32 returns the bit within the reg */
        const uint8_t bit_pos = int_ID & 0x1F;

        /* GICD_ISENABLER + 4 * offset */
        const uint64_t address = GICD_BASE_ADDRESS + GICD_ISENABLER + (offset << 2);
        uint32_t reg32_value = io_read_32(address);
        reg32_value |= 1ULL << bit_pos;
        io_write_32(reg32_value, address);
        
        /* Set the routing property to this physical cpu id */
        io_write_64((uint64_t)(cpu_id), GICD_BASE_ADDRESS + GICD_IROUTER + (int_ID << 3));
    }
}

void arm_gic_disable_int(uint32_t int_ID)
{
    uint64_t cpu_id;
    asm volatile("mrs %0, mpidr_el1": "=r"(cpu_id));
    cpu_id &= 0xF;

    if(int_ID <= MAX_PPI_ID){
        /* SGI or PPI */
        gic_rdist[cpu_id].sgis.GICR_ICENABLER[0] =(1ULL << (int_ID & 0x1f));
    } else if(int_ID <= _n_int){
        /* SPI */
        /* Dividing by 32 to get reg offset */
        const uint8_t offset = int_ID >> 5;

        /* Modulo 32 returns the bit within the reg */
        const uint8_t bit_pos = int_ID & 0x1F;

        /* GICD_ISENABLER + 4 * offset */
        const uint64_t address = GICD_BASE_ADDRESS + GICD_ICENABLER + (offset << 2);
        uint32_t reg32_value = io_read_32(address);
        reg32_value |= 1ULL << bit_pos;
        io_write_32(reg32_value, address);
        
        /* Set the routing property to this physical cpu id */
        io_write_64((uint64_t)(1ULL << cpu_id), GICD_BASE_ADDRESS + GICD_IROUTER + (int_ID << 3));
    }
}

void arm_gic_dispatch_interrupt(int64_t (*handler)(uint64_t)){
    int64_t ret = 0;
    uint32_t intID;

    /* Retrieve the triggered Interrupt ID */
    intID = arm_gic_get_interrupt_id();

    if(GIC_SPURIOUS_INTERRUPT == intID){
        ret = -1;
    } else {
        if(NULL != handler){
            ret = handler(intID);
        }
    }

    /* Notify End Of intID handling */
    arch_msr(ICC_EOIR1_EL1, intID);

    /* Deactivate intID */
    arch_msr(ICC_DIR_EL1, intID);

    if(ret != 0){
        bprintf("CLARE-BasicFirmware - Error on handling interrupt %d\n", intID);
    }
}


static void arm_gic_dist_init(void){
    uint32_t temp_reg;
    uint32_t i;

    /* Disable the distributor */
    temp_reg = io_read_32(GICD_BASE_ADDRESS + GICD_CTLR);
    temp_reg &= ~(1 << GIC_INTR_GROUP1);
    io_write_32(temp_reg, GICD_BASE_ADDRESS + GICD_CTLR);

    /* Get number of interrupts */
    _n_int = io_read_32(GICD_BASE_ADDRESS + GICD_TYPER);
    _n_int &= IT_LINES_NO_MASK;
    _n_int = (_n_int + 1) << 5;
    
    /* Disable all SPI interrupts */
    for(i = MIN_SPI_ID; i < _n_int; i = i + 32){
        io_write_32(~0, GICD_BASE_ADDRESS + GICD_ICENABLER + (i >> 3));
    }

    /* Set each interrupt as edge-triggered */
    for(i = MIN_SPI_ID; i < _n_int; i = i + 16){
        io_write_32(~0, GICD_BASE_ADDRESS + GICD_ICFGR + (i >> 2));
    }
    
    /* Enable the distributor to forward interrupts */
    temp_reg = io_read_32(GICD_BASE_ADDRESS + GICD_CTLR);
    temp_reg |= 0x3;
    io_write_32(temp_reg, GICD_BASE_ADDRESS + GICD_CTLR);
}

static void arm_gic_cpu_init(void){
    /* GIC System Registers enable */
    arch_msr(ICC_SRE_EL1, ICC_SRE_EL1_SRE_ENABLE);

    /* Set Interrupt Priority Mask to the highest value */
    arch_msr(ICC_PMR_EL1, 0xff);

    /* Set the finest granularity of priority */
    arch_msr(ICC_BPR1_EL1, 0x0);

    /* Enable Group1 interrupts */
    arch_msr(ICC_IGRPEN1_EL1, 1);

    /* Enable Group0 interrupts */
    arch_msr(ICC_IGRPEN0_EL1, 1);

    /* Enable interrupts, disable bypass and set End Of Interrupt mode NS */
    arch_msr(ICC_CTLR_EL1, EOI_MODE_NS);
}

static void arm_gic_redist_init(void){
    uint64_t cpu_id;
    asm volatile("mrs %0, mpidr_el1": "=r"(cpu_id));
    cpu_id &= 0xF;

    /* Set PPI & SGI as Group 1 */
    gic_rdist[cpu_id].sgis.GICR_IGROUPR[0] = (uint32_t)~0;

    /* Disable PPIs & SGIs interrupts */
    gic_rdist[cpu_id].sgis.GICR_ICENABLER[0] = (uint32_t)~0;

    /* Set priority of each PPI & SGI */
    for(uint32_t int_id = 0; int_id < MIN_SPI_ID; int_id++){
        gic_rdist[cpu_id].sgis.GICR_IPRIORITYR[int_id] = 0; 
    }

    /* Enable SGIs interrupts */
    gic_rdist[cpu_id].sgis.GICR_ISENABLER[0] = 0xff;
}

void arm_gic_init(void){

    uint64_t cpu_id;
    asm volatile("mrs %0, mpidr_el1": "=r"(cpu_id));
    cpu_id &= 0xF;
    
    if(cpu_id == 0){
        /* Primary core code */

        /* Initialize GIC Distributor */
        arm_gic_dist_init();

        /* Ensure a complete barrier */
        asm volatile("isb\n"\
                     "dmb sy\n"\
                     "dsb sy\n");

        /* Mark the GIC Distributor as initialized */
        _gic_initialized = TRUE;
    } else {
        while(_gic_initialized == FALSE)asm volatile("nop");
    }

    /* Initialize GIC Redistributor */
    arm_gic_redist_init();

    /* Initialize GIC CPU Interface */
    arm_gic_cpu_init();
}
