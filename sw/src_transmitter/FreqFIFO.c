// FreqFIFO.c
// Runs on any  microcontroller
// Provide functions that initialize a FIFO, put data in, get data out,
// and return the current size.  The file includes a FIFO
// using index implementation
// July 15, 2026

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2026

 Copyright 2026 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include <stdint.h>
#include "FreqFIFO.h"
// ****ECE445L write this ****

// Single producer (TIMG8), single consumer (SysTick)
static volatile freqDataType buffer[FREQFIFOSIZE];
static volatile uint32_t putIndex, getIndex;

void FreqFifo_Init(void) {
   putIndex = 0;
   getIndex = 0;
}

uint32_t FreqFifo_Size(void) {
   return (putIndex - getIndex) & (FREQFIFOSIZE - 1);
}

int FreqFifo_PutBatch(const freqDataType *data, uint32_t count) {
   uint32_t next = putIndex;
   if(count > (FREQFIFOSIZE - 1 - FreqFifo_Size())) return 0;

   for(uint32_t i = 0; i < count; i++) {
      buffer[next] = data[i];
      next = (next + 1) & (FREQFIFOSIZE - 1);
   }
   putIndex = next;
   return 1;
}

int FreqFifo_Put(freqDataType data) {
   return FreqFifo_PutBatch(&data, 1);
}

int FreqFifo_Get(freqDataType *data) {
   uint32_t next = getIndex;
   if(next == putIndex) return 0;

   *data = buffer[next];
   getIndex = (next + 1) & (FREQFIFOSIZE - 1);
   return 1;
}
