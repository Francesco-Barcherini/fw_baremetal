/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <inttypes.h>

#define BIT(n) (1 << (n))

/* Registers */
#define SCSMR           0x00    /* Serial Mode Register */
#define SCBRR           0x04    /* Bit Rate Register */
#define SCSCR           0x08    /* Serial Control Register */
#define SCFTDR          0x0c    /* Transmit FIFO Data Register */
#define SCFSR           0x10    /* Serial Status Register */
#define SCFRDR          0x14    /* Receive FIFO Data Register */
#define SCFCR           0x18    /* FIFO Control Register */
#define SCFDR           0x1c    /* FIFO Data Count Register */
#define SCSPTR          0x20    /* Serial Port Register */
#define SCLSR           0x24    /* Line Status Register */
#define DL              0x30    /* Frequency Division Register */
#define CKS             0x34    /* Clock Select Register */
#define HSSRR           0x40    /* Sampling Rate Register */

/* SCSMR (Serial Mode Register) */
#define SCSMR_C_A       BIT(7)  /* Communication Mode */
#define SCSMR_CHR       BIT(6)  /* 7-bit Character Length */
#define SCSMR_PE        BIT(5)  /* Parity Enable */
#define SCSMR_O_E       BIT(4)  /* Odd Parity */
#define SCSMR_STOP      BIT(3)  /* Stop Bit Length */
#define SCSMR_CKS1      BIT(1)  /* Clock Select 1 */
#define SCSMR_CKS0      BIT(0)  /* Clock Select 0 */

/* SCSCR (Serial Control Register) */
#define SCSCR_TEIE      BIT(11) /* Transmit End Interrupt Enable */
#define SCSCR_TIE       BIT(7)  /* Transmit Interrupt Enable */
#define SCSCR_RIE       BIT(6)  /* Receive Interrupt Enable */
#define SCSCR_TE        BIT(5)  /* Transmit Enable */
#define SCSCR_RE        BIT(4)  /* Receive Enable */
#define SCSCR_REIE      BIT(3)  /* Receive Error Interrupt Enable */
#define SCSCR_TOIE      BIT(2)  /* Timeout Interrupt Enable */
#define SCSCR_CKE1      BIT(1)  /* Clock Enable 1 */
#define SCSCR_CKE0      BIT(0)  /* Clock Enable 0 */

/* SCFCR (FIFO Control Register) */
#define SCFCR_RTRG1     BIT(7)  /* Receive FIFO Data Count Trigger 1 */
#define SCFCR_RTRG0     BIT(6)  /* Receive FIFO Data Count Trigger 0 */
#define SCFCR_TTRG1     BIT(5)  /* Transmit FIFO Data Count Trigger 1 */
#define SCFCR_TTRG0     BIT(4)  /* Transmit FIFO Data Count Trigger 0 */
#define SCFCR_MCE       BIT(3)  /* Modem Control Enable */
#define SCFCR_TFRST     BIT(2)  /* Transmit FIFO Data Register Reset */
#define SCFCR_RFRST     BIT(1)  /* Receive FIFO Data Register Reset */
#define SCFCR_LOOP      BIT(0)  /* Loopback Test */

/* SCFSR (Serial Status Register) */
#define SCFSR_PER3      BIT(15) /* Parity Error Count 3 */
#define SCFSR_PER2      BIT(14) /* Parity Error Count 2 */
#define SCFSR_PER1      BIT(13) /* Parity Error Count 1 */
#define SCFSR_PER0      BIT(12) /* Parity Error Count 0 */
#define SCFSR_FER3      BIT(11) /* Framing Error Count 3 */
#define SCFSR_FER2      BIT(10) /* Framing Error Count 2 */
#define SCFSR_FER_1     BIT(9)  /* Framing Error Count 1 */
#define SCFSR_FER0      BIT(8)  /* Framing Error Count 0 */
#define SCFSR_ER        BIT(7)  /* Receive Error */
#define SCFSR_TEND      BIT(6)  /* Transmission ended */
#define SCFSR_TDFE      BIT(5)  /* Transmit FIFO Data Empty */
#define SCFSR_BRK       BIT(4)  /* Break Detect */
#define SCFSR_FER       BIT(3)  /* Framing Error */
#define SCFSR_PER       BIT(2)  /* Parity Error */
#define SCFSR_RDF       BIT(1)  /* Receive FIFO Data Full */
#define SCFSR_DR        BIT(0)  /* Receive Data Ready */

/* SCLSR (Line Status Register) on (H)SCIF */
#define SCLSR_TO        BIT(2)  /* Timeout */
#define SCLSR_ORER      BIT(0)  /* Overrun Error */

/* HSSRR (Sampling Rate Register) */
#define HSSRR_SRE            BIT(15)      /* Sampling Rate Register Enable */
#define HSSRR_SRCYC_DEF_VAL  0x7          /* Sampling rate default value */

int uart_rcar_configure();

void _itoa(char* buffer, int base, uint64_t value);

int puts_no_lock(const char *str);

int puts(const char *str);

int put(const char str);

#endif
