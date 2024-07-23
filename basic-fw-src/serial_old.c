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
#include <inttypes.h>

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
    *((uint32_t *) UART_BASE_REG) = 0xD;
  }
  *((unsigned int *) UART_BASE_REG) = str;
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

