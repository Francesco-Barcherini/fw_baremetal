/* 
 * This file is part of CLARE-Middleware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */


#ifndef TTBL_H
#define TTBL_H

#include <inttypes.h>

typedef uint64_t vaddr_t;
typedef uint64_t paddr_t;
typedef uint64_t ttbl_entry;
typedef ttbl_entry  pt_t;

#ifndef NULL
#define NULL	0
#endif /* NULL */

/* 
 * Structure to manage ttbl.
 */
typedef struct pagetable_ctrl {
	ttbl_entry *base;
	ttbl_entry *next_ttbl;
} pt_ctrl_t;


#define TTBL_PAGESIZE				0x00001000ULL
#define TTBL_ENTRY_CNT				512		/** 512 * 64 bit = 4K **/

#define TTBL_VALIDATE_ENTRY_MASK	0x0000000000000001ULL
#define TTBL_IS_VALID_ENTRY_MASK	TTBL_VALIDATE_ENTRY_MASK /*(addr & MASK != 0? VALID : NOT) */

#define TTBL_TABLE_ENTRY_MASK		0x0000000000000002ULL

#define TTBL_TABLE_NEXTTBL_MASK		0x000000fffffff000ULL /* (addr & MASK = isolates [39:12]) */

#define TTBL_ENTRY_IS_VALID(entry)		(entry & TTBL_IS_VALID_ENTRY_MASK)

/* LEVEL 0, 1, 2 (BLOCK, TABLE) */
#define TTBL_BLOCK_ENTRY_MASK		0x0000000000000000ULL
#define TTBL_TABLE_ENTRY_MASK		0x0000000000000002ULL

/* LEVEL 3 (PAGE) */
#define TTBL_PAGE_LOWER_ATTR_MASK	0x0000000000000fffULL
#define TTBL_PAGE_LOWER_ATTR_SHIFT	0
#define TTBL_PAGE_OA_MASK			0x0000fffffffff000ULL
#define TTBL_PAGE_OA_SHIFT			12
#define TTBL_PAGE_UPPER_ATTR_MASK	0xfff8000000000000ULL
#define TTBL_PAGE_UPPER_ATTR_SHIFT	51

/* BLOCK */
#define TTBL_BLOCK_LOWER_ATTR_MASK	TTBL_PAGE_LOWER_ATTR_MASK /* (addr & MASK = isolates [11:2]) */
#define TTBL_BLOCK_LOWER_ATTR_SHIFT TTBL_PAGE_LOWER_ATTR_SHIFT
#define TTBL_BLOCK_UPPER_ATTR_MASK	0xfff0000000000000ULL /* (addr & MASK = isolates [63:52]) */
#define TTBL_BLOCK_UPPER_ATTR_SHIFT	52
#define TTBL_L1BLOCK_OA_MASK		0x0000ffffc0000000ULL /* (addr & MASK = isolates [47:30]) */
#define TTBL_L1BLOCK_OA_SHIFT		30
#define TTBL_L2BLOCK_OA_MASK		0x0000ffffffe00000ULL /* (addr & MASK = isolates [47:21]) */
#define TTBL_L2BLOCK_OA_SHIFT		21

/* BLOCK/PAGE ATTRIBUTES: COMMON */
#define TTBL_UPPER_PBHA_MASK		0x7800000000000000ULL	/* IGNORED */
#define TTBL_UPPER_PBHA_SHIFT		59
#define TTBL_UPPER_XN_MASK			0x0040000000000000ULL	/* 1 = Execute Never */
#define TTBL_UPPER_XN_SHIFT			54
#define TTBL_UPPER_CONT_MASK		0x0010000000000000ULL
#define TTBL_UPPER_CONT_SHIFT		52
#define TTBL_UPPER_DBM_MASK			0x0008000000000000ULL	/* Dirty bit */
#define TTBL_UPPER_DBM_SHIFT		51
#define TTBL_LOWER_AF_MASK			0x0000000000000400ULL	/* Access flag */
#define TTBL_LOWER_AF_SHIFT			10
#define TTBL_LOWER_SH_MASK			0x0000000000000300ULL
#define TTBL_LOWER_SH_SHIFT			8
#define TTBL_LOWER_AP_MASK			0x00000000000000c0ULL
#define TTBL_LOWER_AP_SHIFT			6


/* BLOCK/PAGE ATTRIBUTES: STAGE 1 */
#define TTBL_ST1_UPPER_PXN_MASK			0x0020000000000000ULL	/* RES0 */
#define TTBL_ST1_UPPER_PXN_SHIFT		53
#define TTBL_ST1_LOWER_NG_MASK			0x0000000000000800ULL	/* NG set to 1 in stage 2
															   RES0 in stage 1 if only one VA supported */
#define TTBL_ST1_LOWER_NG_SHIFT			11
#define TTBL_ST1_LOWER_NS_MASK			0x0000000000000020ULL 	/* NS == 0(1) => access (Non-)Secure
																				 phys addr space */
#define TTBL_ST1_LOWER_NS_SHIFT			5
#define TTBL_ST1_LOWER_ATTRIDX_MASK		0x000000000000001cULL	/* MAIR values, see reg.h */
#define TTBL_ST1_LOWER_ATTRIDX_SHIFT	2

/* AP VALUES */
#define TTBL_ST1_AP_SRW					0
#define TTBL_ST1_AP_SRW_URW				1
#define TTBL_ST1_AP_SR	 				2
#define TTBL_ST1_AP_SR_UR 				3

/* SH VALUES */
#define TTBL_SH_NONSHAREABLE		0
#define TTBL_SH_OUTERSHAREABLE		0x2
#define TTBL_SH_INNERSHAREABLE		0x3

#define MAIR(val, idx)						((val) << ((idx) * 8))
#define MAIR_ATTRIDX_DEVICE_nGnRnE			0UL
#define MAIR_ATTRIDX_DEVICE_nGnRE			1UL
#define MAIR_ATTRIDX_DEVICE_nGRE			2UL
#define MAIR_ATTRIDX_DEVICE_GRE				3UL
#define MAIR_ATTRIDX_NORMAL_WT				4UL
#define MAIR_ATTRIDX_NORMAL_WB				5UL
#define MAIR_ATTRIDX_NORMAL_NC				6UL

#define TCR_T0SZ_MASK					0x0000003f
#define TCR_T0SZ_VAL(in_bits)			((64 - (in_bits)) & TCR_T0SZ_MASK)
#define TCR_PS_SHIFT					16
#define TCR_PS_40BITS					(2 << TCR_PS_SHIFT)
#define TCR_TG0_SHIFT					14
#define TCR_SH0_SHIFT					12
#define TCR_ORGN0_SHIFT					10
#define TCR_IRGN0_SHIFT					8

#define SCTLR_MMU_ENABLE						1
#define SCTLR_NO_D_CACHEABILITY					(1U << 2)
#define SCTLR_NO_I_CACHEABILITY					(1 << 12)
/* Clean and invalidate by virtual address to point of coherency */
static inline void cache_entry_clean_invalidate(void *va) {
	__asm__ volatile("dc civac, %0\t\n"
		     "dsb sy\t\n"
		     "isb\t\n"
		     : : "r" ((uint64_t)va));
}

/**
 * This function add a memory section to the pagetable
 *
 * @param start_addr The base address of the section
 * @param size The size
 * @return 0 If succeds, <0 otherwise 
 */
int8_t pagetable_add_memory_section(paddr_t start_addr,
									uint64_t size,
									uint8_t cachable,
									uint8_t writable,
									uint8_t executable);

/**
 * This function add a device address space to the pagetable
 *
 * @param start_addr The base address of the section
 * @param size The size
 * @return 0 If succeds, <0 otherwise 
 */
int8_t pagetable_add_device_section(paddr_t start_addr, uint64_t size);

/**
 * This function enables the mmu. The pagetable must be previously created
 *
 * @return 0 if succeds, <0 otherwise
 */
int8_t enable_mmu(void);

/**
 * This function creates a simple pagetable taking the entire address
 * space on RAM as single section and flat-mapped 
 *
 * @param start_addr The base address of the RAM section
 * @param size The size
 * @return 0 If succeds, <0 otherwise 
 */
int8_t create_pagetable_flat_single_section(paddr_t start_addr, uint64_t size);

#endif /* TTBL_H */