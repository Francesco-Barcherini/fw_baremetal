/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#ifndef __GICV3_H__
#define __GICV3_H__

#include "guest.h"

struct GICv3_rdist_lpis_if
{
        volatile uint32_t GICR_CTLR;             // +0x0000 - RW - Redistributor Control Register
  const volatile uint32_t GICR_IIDR;             // +0x0004 - RO - Redistributor Implementer Identification Register
  const volatile uint32_t GICR_TYPER[2];         // +0x0008 - RO - Redistributor Type Register
        volatile uint32_t GICR_STATUSR;          // +0x0010 - RW - Redistributor Status register
        volatile uint32_t GICR_WAKER;            // +0x0014 - RW - Wake Request Registers
  const volatile uint32_t GICR_MPAMIDR;          // +0x0018 - RO - Reports maximum PARTID and PMG (GICv3.1)
        volatile uint32_t GICR_PARTID;           // +0x001C - RW - Set PARTID and PMG used for Redistributor memory accesses (GICv3.1)
  const volatile uint32_t padding1[8];           // +0x0020 - RESERVED
        volatile uint64_t GICR_SETLPIR;          // +0x0040 - WO - Set LPI pending (Note: IMP DEF if ITS present)
        volatile uint64_t GICR_CLRLPIR;          // +0x0048 - WO - Set LPI pending (Note: IMP DEF if ITS present)
  const volatile uint32_t padding2[6];           // +0x0058 - RESERVED
        volatile uint32_t GICR_SEIR;             // +0x0068 - WO - (Note: This was removed from the spec)
  const volatile uint32_t padding3;              // +0x006C - RESERVED
        volatile uint64_t GICR_PROPBASER;        // +0x0070 - RW - Sets location of the LPI configuration table
        volatile uint64_t GICR_PENDBASER;        // +0x0078 - RW - Sets location of the LPI pending table
  const volatile uint32_t padding4[8];           // +0x0080 - RESERVED
        volatile uint64_t GICR_INVLPIR;          // +0x00A0 - WO - Invalidates cached LPI config (Note: In GICv3.x: IMP DEF if ITS present)
  const volatile uint32_t padding5[2];           // +0x00A8 - RESERVED
        volatile uint64_t GICR_INVALLR;          // +0x00B0 - WO - Invalidates cached LPI config (Note: In GICv3.x: IMP DEF if ITS present)
  const volatile uint32_t padding6[2];           // +0x00B8 - RESERVED
        volatile uint64_t GICR_SYNCR;            // +0x00C0 - WO - Redistributor Sync
  const volatile uint32_t padding7[2];           // +0x00C8 - RESERVED
  const volatile uint32_t padding8[12];          // +0x00D0 - RESERVED
        volatile uint64_t GICR_MOVLPIR;          // +0x0100 - WO - IMP DEF
  const volatile uint32_t padding9[2];           // +0x0108 - RESERVED
        volatile uint64_t GICR_MOVALLR;          // +0x0110 - WO - IMP DEF
  const volatile uint32_t padding10[2];          // +0x0118 - RESERVED
};

struct GICv3_rdist_sgis_if
{
  const volatile uint32_t padding1[32];          // +0x0000 - RESERVED
        volatile uint32_t GICR_IGROUPR[3];       // +0x0080 - RW - Interrupt Group Registers (Security Registers in GICv1)
  const volatile uint32_t padding2[29];          // +0x008C - RESERVED
        volatile uint32_t GICR_ISENABLER[3];     // +0x0100 - RW - Interrupt Set-Enable Registers
  const volatile uint32_t padding3[29];          // +0x010C - RESERVED
        volatile uint32_t GICR_ICENABLER[3];     // +0x0180 - RW - Interrupt Clear-Enable Registers
  const volatile uint32_t padding4[29];          // +0x018C - RESERVED
        volatile uint32_t GICR_ISPENDR[3];       // +0x0200 - RW - Interrupt Set-Pending Registers
  const volatile uint32_t padding5[29];          // +0x020C - RESERVED
        volatile uint32_t GICR_ICPENDR[3];       // +0x0280 - RW - Interrupt Clear-Pending Registers
  const volatile uint32_t padding6[29];          // +0x028C - RESERVED
        volatile uint32_t GICR_ISACTIVER[3];     // +0x0300 - RW - Interrupt Set-Active Register
  const volatile uint32_t padding7[29];          // +0x030C - RESERVED
        volatile uint32_t GICR_ICACTIVER[3];     // +0x0380 - RW - Interrupt Clear-Active Register
  const volatile uint32_t padding8[29];          // +0x018C - RESERVED
        volatile uint8_t  GICR_IPRIORITYR[96];   // +0x0400 - RW - Interrupt Priority Registers
  const volatile uint32_t padding9[488];         // +0x0460 - RESERVED
        volatile uint32_t GICR_ICFGR[6];         // +0x0C00 - RW - Interrupt Configuration Registers
  const volatile uint32_t padding10[58];	       // +0x0C18 - RESERVED
        volatile uint32_t GICR_IGRPMODR[3];      // +0x0D00 - RW - Interrupt Group Modifier Register
  const volatile uint32_t padding11[61];	       // +0x0D0C - RESERVED
        volatile uint32_t GICR_NSACR;            // +0x0E00 - RW - Non-secure Access Control Register

};

struct GICv3_rdist_if
{
  struct GICv3_rdist_lpis_if   lpis  __attribute__((aligned (0x10000)));
  struct GICv3_rdist_sgis_if   sgis  __attribute__((aligned (0x10000)));
};

/* Interrupt categories */
#define MIN_SGI_ID                        (0)
#define MIN_SEC_SGI_ID	                  (8)
#define MAX_SGI_ID                        (15)
#define MIN_PPI_ID                        (16)
#define MAX_PPI_ID                        (31)
#define MIN_SPI_ID                        (32)
#define MAX_SPI_ID                        (1019)
#define MIN_EPPI_ID                       (1056)
#define MAX_EPPI_ID                       (1119)
#define MIN_ESPI_ID                       (4096)
#define MAX_ESPI_ID                       (5119)
#define MIN_LPI_ID                        (8192)

/*******************************************************************************
 * GIC Distributor interface register offsets that are common to GICv3 & GICv2
 ******************************************************************************/
#define GICD_CTLR			0x0
#define GICD_TYPER			0x4
#define GICD_IIDR			0x8
#define GICD_IGROUPR		0x80
#define GICD_ISENABLER		0x100
#define GICD_ICENABLER		0x180
#define GICD_ISPENDR		0x200
#define GICD_ICPENDR		0x280
#define GICD_ISACTIVER		0x300
#define GICD_ICACTIVER		0x380
#define GICD_IPRIORITYR		0x400
#define GICD_ITARGETSR		0x800
#define GICD_ICFGR			0xc00
#define GICD_NSACR			0xe00
#define GICD_SGIR			0xf00
#define GICD_CPENDSGIR		0xf10
#define GICD_SPENDSGIR		0xf20

#define GICD_IROUTER                                  (0x6000)

#define GIC_INTR_GROUP1					1

/* GICD_TYPER bit definitions */
#define IT_LINES_NO_MASK	0x1f

/* Interrupt and CPU ID mask for IAR register */
#define INT_AND_CPU_ID_MASK                           (0xffffff)

/*******************************************************************************
 * Definitions for CPU system register interface to GICv3
 ******************************************************************************/
#define ICC_IGRPEN1_EL1             S3_0_C12_C12_7
#define ICC_SGI1R                   S3_0_C12_C11_5
#define ICC_SRE_EL1                 S3_0_C12_C12_5
#define ICC_SRE_EL2                 S3_4_C12_C9_5
#define ICC_CTLR_EL1                S3_0_C12_C12_4
#define ICC_PMR_EL1                 S3_0_C4_C6_0
#define ICC_BPR1_EL1                S3_0_C12_C12_3 
#define ICC_RPR_EL1                 S3_0_C12_C11_3
#define ICC_IGRPEN0_EL1             S3_0_c12_c12_6
#define ICC_HPPIR0_EL1              S3_0_c12_c8_2
#define ICC_HPPIR1_EL1              S3_0_c12_c12_2
#define ICC_IAR0_EL1                S3_0_c12_c8_0
#define ICC_IAR1_EL1                S3_0_c12_c12_0
#define ICC_EOIR0_EL1               S3_0_c12_c8_1
#define ICC_EOIR1_EL1               S3_0_c12_c12_1
#define ICC_SGI1R_EL1               S3_0_C12_C11_5
#define ICC_DIR_EL1                 S3_0_C12_C11_1

#define SGIR_TGTLSTFLT_SHIFT	24
#define SGIR_TGTLSTFLT_MASK	0x3
#define SGIR_TGTLST_SHIFT	16
#define SGIR_TGTLST_MASK	0xff
#define SGIR_INTID_MASK		0xf

/* ICC_SGI1R_EL1 */
#define SGI_INT_ID_SHIFT            (24)
#define SGI_INT_ID_MASK             (0b1111)
#define SGI_IRM_SHIFT               (40)
#define SGI_IRM_MASK                (0b1ULL)
#define SGI_TARGET_LIST_MASK        (0xFF)

#define GICV3_SGIR_COMPOSE(targetFilter, targetList, intID) \
	((((targetFilter) & SGIR_TGTLSTFLT_MASK) << SGIR_TGTLSTFLT_SHIFT) | \
	 (((targetList) & SGI_TARGET_LIST_MASK)) | \
	 ((intID) & SGIR_INTID_MASK))

/* Constant to indicate a spurious interrupt in all GIC versions */
#define GIC_SPURIOUS_INTERRUPT		1023

/* ICC_SRE_EL1 */
#define ICC_SRE_EL1_SRE_ENABLE                        (1)

/* ICC_CTLR_EL1 bit definitions */
#define EOI_MODE_NS			(1 << 1)

/*------------------------------------------------------------------*/
/*-------------------------[ E R R O R S ]--------------------------*/
/*------------------------------------------------------------------*/
#define GIC_OK								0
#define GIC_NOT_INITIALIZED					1
#define GIC_WRONG_PARAMETERS				2

void arm_gic_enable_int(uint32_t int_ID);

void arm_gic_disable_int(uint32_t int_ID);

/* This function generates a Software Generated Interrupt
 * @param intID is the interrupt ID to be raised (legal value [0-15]).
 * @param targeted determines how the distributor must process 
 *        the requested SGI:
 *        TRUE:  Forward the interrupt to the CPU interfaces specified 
 *               in the CPUTargetList field
 *        FALSE: Forward the interrupt to all CPU interfaces except that 
 *               of the processor that requested the interrupt
 * @param targetList when TargetList Filter = 0b00 , defines the CPU 
 *        interfaces to which the Distributor must forward the interrupt
 */
int arm_gic_generate_sgi(uint16_t intID, boolean_t targeted, uint8_t targetList);

/* This function check if the interrupt is manageable and then
 * calls the corresponding handler */ 
void arm_gic_dispatch_interrupt(int64_t (*handler)(uint64_t));

/* This function initializes the GIC:
 * Distributor with all interrupts (PPI & SPI) disabled
 * with the highest NS interrupt priority
 * SPI to be triggered as active low
 * Enabling forwarding of Group 1 interrupts
 *
 * CPU Interface with the priority mask at the highest level
 * Enabling forwarding of Group 1 interrupts*/
void arm_gic_init(void);

#endif /* __GICV3_H__ */
