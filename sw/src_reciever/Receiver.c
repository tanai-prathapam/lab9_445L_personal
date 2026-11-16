// Receiver.c
// Lab 9
// Programs to implement receiver functionality   
// ECE445L Fall 2026
//    Jonathan W. Valvano 7/15/26

// ----------------------------------------------------------------------------
// Hardware/software assigned to receiver
//   Timer G8 PB19 ADC1 channel 6 samples sound
//   DecodeFIFO linkage from  ADC to Decoder
//   main-loop runs decoder
// PB19 ADC1 channel 6 J3.23 microphone

//   PA2-PA7, SSI0, ST7735R

#include "Lab9Rcv.h"
#include <ti/devices/msp/msp.h>
#include "../inc/DFT.h"
#include "../inc/Timer.h"
#include "../inc/Median.h"
#include "../inc/ADC.h"
#include "DecodeFIFO.h"
// ****ECE445L write this ****



// -----------     Receiver_Init    -----------------------------
// Sample ADC1 channel 6
// Timer G8: enabled for real time sampling, priority 0
void Receiver_Init(uint32_t period,uint32_t prescale){
  ADC1_Init(6, ADCVREF_INT);
  TimerG8_IntArm(period, prescale, 0);
  TimerG6_IntArm(8192, 80, 1);
  TIMG6->COUNTERREGS.CTRCTL &= ~(0x01); // disable TIMG6 
  NoiseReject_Init();
}

void TIMG8_IRQHandler(void){// runs at the sampling rate, priority 0
  if((TIMG8->CPU_INT.IIDX) == 1){ // this will acknowledge
    #if DEBUG_MODE_RC == 1
    adc_output = NoiseReject(ADC1_In());
    #else
    // DecodeFifo_Put(NoiseReject(ADC1_In()));
    DecodeFifo_Put(ADC1_In());
    #endif
  }
}

void TIMG6_IRQHandler(void){// runs at baud rate, priority 1
  if((TIMG6->CPU_INT.IIDX) == 1){ // this will acknowledge
    //TODO
  }
}
// start, 8 data, even parity

uint8_t Receiver_Decode(void){ // called from main loop
  
  return 0;
}


