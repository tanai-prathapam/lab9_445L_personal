// File **********Transmitter.c***********
// Lab 9
// Programs to implement transmitter functionality   
// ECE445L Fall 2026
//    Jonathan W. Valvano 7/15/26
// Possible Hardware/software assigned to transmitter
// 2-bit input, positive logic switches, positive logic software
//   bit1 PA28 SW1 switch 
//   bit0 PA27 SW2 switch 
// Joystick
//   PA24 VRx ADC0 channel 3
//   PA25 VRy ADC0 channel 2
//   PA26 SW positive logic internal pulldown
// MCP4921
// pin signal MSPM0   mode
//  1   Vdd   +3.3V
//  2   /CS   PA8    SPI0_CS0
//  3   SCLK  PB18   SPII_SCK
//  4   SDI   PB17   SPI_PICO
//  5   /LDAC ground
//  6   REF          1.255 V refernce
//  7   AGND         ground
//  8   OUT          DAC output
//   Timer0A periodic interrupt to generate input
//   FreqFifo linkage from Encoder to SendData
//   Timer1A periodic interrupt create a sequence of frequencies
//   SPI1 MCP4921 DAC output
//   SysTick ISR output to DAC

#include <stdint.h>
#include <ti/devices/msp/msp.h>
#include "Joystick.h"
#include "../inc/Timer.h"
#include "FreqFIFO.h"
#include "../inc/MCP4921.h"
#include "../inc/UART.h"
#include "../inc/LaunchPad.h"
#include "TLV5616.h"
#include "Transmitter.h"

// ****ECE445L write this ****

#define SYMBOL_SAMPLES 500 // 4096
#define FRAME_SYMBOLS 11
#define GAP_SAMPLES 8192
// static const uint16_t wave[64] = {
//   2048, 2148, 2248, 2345, 2440, 2531, 2617, 2698, 2772, 2840, 2899,
//   2951, 2994, 3028, 3052, 3067, 3072, 3067, 3052, 3028, 2994, 2951,
//   2899, 2840, 2772, 2698, 2617, 2531, 2440, 2345, 2248, 2148, 2048,
//   1948, 1848, 1751, 1656, 1565, 1479, 1398, 1324, 1256, 1197, 1145,
//   1102, 1068, 1044, 1029, 1024, 1029, 1044, 1068, 1102, 1145, 1197,
//   1256, 1324, 1398, 1479, 1565, 1656, 1751, 1848, 1948};

// 12-bit 64-element sine wave
// full amplitude wave
const uint16_t wave[64] = {
  2048,2244,2438,2629,2813,2991,3159,3317,3462,3594,3711,
  3812,3896,3962,4010,4038,4048,4038,4010,3962,3896,3812,
  3711,3594,3462,3317,3159,2991,2813,2629,2438,2244,2048,
  1852,1658,1467,1283,1105,937,779,634,502,385,284,
  200,134,86,58,48,58,86,134,200,284,385,
  502,634,779,937,1105,1283,1467,1658,1852
}; 

static volatile uint8_t pendingCommand;
static volatile uint32_t commandPending, busy;

// owned by systick
static uint32_t samplesLeft, symbolsLeft, gapLeft, phase, step;

int Transmitter_IsBusy(void) {
  return busy != 0;
}

int Transmitter_Send(uint8_t command) {
  uint32_t saved = __get_PRIMASK();
  __disable_irq();
  if(busy) {
    __set_PRIMASK(saved);
    return 0;
  }

  busy = 1;
  pendingCommand = command;
  commandPending = 1;
  __set_PRIMASK(saved);
  return 1;
}

void TIMG12_IRQHandler(void){ // priority 3
  if((TIMG12->CPU_INT.IIDX) == 1){ // this will acknowledge
    GPIOB->DOUTTGL31_0 = RED;   // toggle PB26
    // use for input from joystick
    // probably remove from here and move to wherever it needs to be called from
  }
}

void TIMG8_IRQHandler(void){// runs every 10ms, priority 1
  if((TIMG8->CPU_INT.IIDX) == 1){ // this will acknowledge
    if(!commandPending) return;
    GPIOB->DOUTTGL31_0 = GREEN; // toggle PB27

    uint8_t data = pendingCommand;
    uint32_t frame[FRAME_SYMBOLS], parity = 0;
    frame[0] = 2; // Start = logic 0

    for(uint32_t i = 0; i < 8; i++) {
      uint32_t bit = (data >> i) & 1;
      frame[i + 1] = bit ? 1 : 2;
      parity ^= bit;
    }
    frame[9] = parity ? 1 : 2; // Even parity
    frame[10] = 1; // Stop = logic 1
    if(FreqFifo_PutBatch(frame, FRAME_SYMBOLS)) commandPending = 0;

    GPIOB->DOUTTGL31_0 = GREEN; // toggle PB27
  }
}
  
// Highest priority Interrupt service routine
// Executed every 12.5ns * (period)
// ----------------    Sine Wave Generation     --------------------------------
void SysTick_Handler(void){ // priority 0
  if(gapLeft) {
    TLV5616_OutNonBlocking(2048);
    if(--gapLeft == 0) busy = 0;
    return;
  }
  if(samplesLeft == 0) {
    if(!FreqFifo_Get(&step)) {
      TLV5616_OutNonBlocking(2048);
      return;
    }

    if(symbolsLeft == 0) {
      symbolsLeft = FRAME_SYMBOLS;
    }

    samplesLeft = SYMBOL_SAMPLES; // play for 4096 samples
  }

  uint32_t sample = wave[phase];
  TLV5616_OutNonBlocking(sample);
  phase = (phase + step) & 63;
  if((--samplesLeft == 0) && (--symbolsLeft == 0)) {
    gapLeft = GAP_SAMPLES;
    // both frequencies have whole cycle per symbol, so phase is now zero
  }
}
//---------------------   Transmitter_Init    ---------------------
// initialize switch interface
// Input: none
// Output: none 
void Transmitter_Init(void){ 
  SysTick->CTRL = 0;
  busy = 0;
  commandPending = 0;
  samplesLeft = 0;
  symbolsLeft = 0;
  gapLeft = 0;
  phase = 0;
  step = 0;
  FreqFifo_Init();
  TimerG8_IntArm(10000, 40, 1); // 40 MHz
  SysTick->LOAD = 1600 - 1;
  SysTick->VAL = 0;
  SCB->SHP[1] &= ~0xC0000000; // highest priority (0)
  SysTick->CTRL = 7;
}


