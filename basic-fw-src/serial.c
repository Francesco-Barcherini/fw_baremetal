/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#include "reg.h"
#include "serial.h"
#include <inttypes.h>

#define IS_HSCIF 	1
#define BAUDRATE 	921600
#define CLK_RATE 	266660000
#define SR 			(HSSRR_SRCYC_DEF_VAL + 1)

static uint8_t uart_rcar_read_8(uint32_t offs)
{
	return *((uint8_t *) (UART_BASE_REG + offs));
}

static void uart_rcar_write_8(uint32_t offs, uint8_t value)
{
	*((uint8_t *) (UART_BASE_REG + offs)) = value;
}

static uint16_t uart_rcar_read_16(uint32_t offs)
{
	return *((uint16_t *) (UART_BASE_REG + offs));
}

static void uart_rcar_write_16(uint32_t offs, uint16_t value)
{
	*((uint16_t *) (UART_BASE_REG + offs)) = value;
}

static void uart_rcar_set_baudrate(uint32_t baud_rate)
{
	uint8_t reg_val;

	if (IS_HSCIF) {
		//reg_val = CLK_RATE / (2 * (HSSRR_SRCYC_DEF_VAL + 1) * baud_rate) - 1;
		reg_val = (CLK_RATE / (2 * SR * BAUDRATE) - 1);
	} else {
		reg_val = ((CLK_RATE + 16 * baud_rate) / (32 * baud_rate) - 1);
	}
	uart_rcar_write_8(SCBRR, reg_val);
}

static int uart_rcar_poll_in(unsigned char *p_char)
{
	uint16_t reg_val;
	int ret = 0;

	//k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Receive FIFO empty */
	if (!((uart_rcar_read_16(SCFSR)) & SCFSR_RDF)) {
		ret = -1;
		goto unlock;
	}

	*p_char = uart_rcar_read_8(SCFRDR);

	reg_val = uart_rcar_read_16(SCFSR);
	reg_val &= ~SCFSR_RDF;
	uart_rcar_write_16(SCFSR, reg_val);

unlock:
	//k_spin_unlock(&data->lock, key);

	return ret;
}

static void uart_rcar_poll_out( unsigned char out_char)
{
	uint16_t reg_val;
	//k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Wait for empty space in transmit FIFO */
	while (!(uart_rcar_read_16(SCFSR) & SCFSR_TDFE)) {
	}

	uart_rcar_write_8(SCFTDR, out_char);

	/* Wait for end of transmition */
	while (!(uart_rcar_read_16(SCFSR) & SCFSR_TEND)) {
	}

	reg_val &= ~(SCFSR_TDFE | SCFSR_TEND);
	uart_rcar_write_16(SCFSR, reg_val);

	//k_spin_unlock(&data->lock, key);
}

int uart_rcar_configure()
{
	uint16_t reg_val;
	//k_spinlock_key_t key;

	/* Disable Transmit and Receive */
	reg_val = uart_rcar_read_16(SCSCR);
	reg_val &= ~(SCSCR_TE | SCSCR_RE);
	uart_rcar_write_16(SCSCR, reg_val);

	/* Emptying Transmit and Receive FIFO */
	reg_val = uart_rcar_read_16(SCFCR);
	reg_val |= (SCFCR_TFRST | SCFCR_RFRST);
	uart_rcar_write_16(SCFCR, reg_val);

	/* Resetting Errors Registers */
	reg_val = uart_rcar_read_16(SCFSR);
	reg_val &= ~(SCFSR_ER | SCFSR_DR | SCFSR_BRK | SCFSR_RDF);
	uart_rcar_write_16(SCFSR, reg_val);

	reg_val = uart_rcar_read_16(SCLSR);
	reg_val &= ~(SCLSR_TO | SCLSR_ORER);
	uart_rcar_write_16(SCLSR, reg_val);

	/* Select internal clock */
	reg_val = uart_rcar_read_16(SCSCR);
	reg_val &= ~(SCSCR_CKE1 | SCSCR_CKE0);
	uart_rcar_write_16(SCSCR, reg_val);

	/* Serial Configuration (8N1) & Clock divider selection */
	reg_val = uart_rcar_read_16(SCSMR);
	reg_val &= ~(/*SCSMR_C_A |*/ SCSMR_CHR | SCSMR_PE | SCSMR_O_E | SCSMR_STOP |
		     SCSMR_CKS1 | SCSMR_CKS0);
	uart_rcar_write_16(SCSMR, reg_val);

	if (IS_HSCIF) {
		/* TODO: calculate the optimal sampling and bit rates based on error rate */
		uart_rcar_write_16(HSSRR, HSSRR_SRE | HSSRR_SRCYC_DEF_VAL);
	}

	/* Set baudrate */
	uart_rcar_set_baudrate(BAUDRATE);

	/* FIFOs data count trigger configuration */
	reg_val = uart_rcar_read_16(SCFCR);
	reg_val &= ~(SCFCR_RTRG1 | SCFCR_RTRG0 | SCFCR_TTRG1 | SCFCR_TTRG0 |
		     SCFCR_MCE | SCFCR_TFRST | SCFCR_RFRST);
	uart_rcar_write_16(SCFCR, reg_val);

	/* Enable Transmit & Receive + disable Interrupts */
	reg_val = uart_rcar_read_16(SCSCR);
	reg_val |= (SCSCR_TE | SCSCR_RE);
	reg_val &= ~(SCSCR_TIE | SCSCR_RIE | SCSCR_TEIE | SCSCR_REIE |
		     SCSCR_TOIE);
	uart_rcar_write_16(SCSCR, reg_val);

	return 0;
}

// inline function to swap two numbers
static inline void swap(char *x, char *y) {
  char t = *x; *x = *y; *y = t;
}

// function to reverse buffer[i..j]
char* reverse(char *buffer, int i, int j)
{
  while (i < j)
    swap(&buffer[i++], &buffer[j--]);

  return buffer;
}

// Iterative function to implement itoa() function in C
void _itoa(char* buffer, int base, uint64_t value)
{
  if(base == 'd'){
    base = 10;
  } else if(base == 'x'){
    base = 16;
  } else {
    return;
  }

  // consider absolute value of number
  // uint64_t n = (uint64_t)abs(value);
  uint64_t n  = value;

  int i = 0;
  while (n)
  {
    uint64_t r = n % base;

    if (r >= 10) 
      buffer[i++] = 65 + (r - 10);
    else
      buffer[i++] = 48 + r;

    n = n / base;
  }

  // if number is 0
  if (i == 0)
    buffer[i++] = '0';

  // If base is 10 and value is negative, the resulting string 
  // is preceded with a minus sign (-)
  // With any other base, value is always considered unsigned
  if (value < 0 && base == 10)
    buffer[i++] = '-';

  buffer[i] = '\0'; // null terminate string

  // reverse the string and return it
  buffer = reverse(buffer, 0, i - 1);
}

int put(const char str)
{
  if(0xA == str){
    uart_rcar_poll_out(0xD);
  }
  uart_rcar_poll_out(str);
  return 0;
}

int puts_no_lock(const char *str)
{

  while (*str)
    put(*str++);

  return 0;
}

int puts(const char *str)
{

  while (*str)
    put(*str++);

  return 0;
}

