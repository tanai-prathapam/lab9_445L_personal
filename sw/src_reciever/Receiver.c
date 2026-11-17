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
#include "Receiver.h"
#include <stdio.h>
// ****ECE445L write this ****

// Circular buffer parameters
static volatile uint16_t circular_buffer[16];
static volatile uint8_t circular_buffer_head; // points to oldest sample

// Decoding parameters
uint8_t decode_index; // current bit being written to

// 16-bit vector encoding information about the frame being received.
// Bit 15: set to 1 when vector is being decoded. 
// Bit 14: set to 1 if validation of received message fails. 
//   Accompanied by bit 15 being set high.
// Bit 13: toggled on every 1 received in the frame. Should always be
//   0 once decoding is complete due to even parity. 
// Bits 9-0: the literal frame read, sans start bit. 
//   Bits 9-2 form the 8-bit validated vector pushed onto the FIFO. 
//   Bit 1 is the parity bit. Functionality is superceded by bit 2
//     outside of decoding scope. 
//   Bit 0 is an end bit, should always be high. 
volatile uint16_t frame; //shared between main and decoding interrupt and volatile makes it so the main loop reads the updated value

//Helpers
decodeDataType parse_frame (decodeDataType frame_main) {
  decodeDataType command_bits = frame_main >> 2;
  decodeDataType transmitted_command = 0x0;
  uint8_t mask = 0b100000; 
  int max_i = 6; 
  for (int i = 0; i < max_i; i++) {
    uint8_t temp = (command_bits & mask) >> (max_i - (i + 1));
    transmitted_command |= temp;
    mask = mask >> 1;
  }
  return transmitted_command;
}

static void Receiver_StopBitTimer(void){
  TIMG6->CPU_INT.IMASK = 0;
  TIMG6->COUNTERREGS.CTRCTL &= ~1;

  // Clear the timer zero event and its pending NVIC interrupt.
  TIMG6->CPU_INT.ICLR = 1;
  NVIC->ICPR[0] = (1 << 17);
}

// -----------     Receiver_Init    -----------------------------
// Sample ADC1 channel 6
// Timer G8: enabled for real time sampling, priority 0
void Receiver_Init(uint32_t period,uint32_t prescale){
  // Call with global interrupts disabled, as your main already does.
  frame = 0;
  decode_index = FRAME_SIZE;
  circular_buffer_head = 0;

  for(uint32_t i = 0; i < 16U; i++){
    circular_buffer[i] = 0;
  }

  DecodeFifo_Init();
  NoiseReject_Init();
  ADC1_Init(6, ADCVREF_INT);

  // 80 MHz / 80 / 10000 = 100 Hz.
  TimerG6_IntArm(10000, 80, 1);
  Receiver_StopBitTimer();

  // With period=3200, prescale=1: 12.5 kHz sampling.
  TimerG8_IntArm(period, prescale, 0);
}

void TIMG8_IRQHandler(void){ // runs at the sampling rate, priority 0
  if((TIMG8->CPU_INT.IIDX) == 1){ // this will acknowledge
    circular_buffer[circular_buffer_head] = NoiseReject(ADC1_In());
    circular_buffer_head = (circular_buffer_head + 1) & 0x0F;
  }
}

void TIMG6_IRQHandler(void){// runs at baud rate, priority 1
  if((TIMG6->CPU_INT.IIDX) == 1){ // this will acknowledge
    if(((frame & 0x8000) == 0) ||
      (decode_index >= FRAME_SIZE)){
      Receiver_StopBitTimer();
      return;
    }
    if(decode_index == FRAME_SIZE){
      return;
    }
    switch(Receiver_DecodeBit()){
      case ERROR:
        frame |= 0x4000; // do not finish decoding to reject entire
        // corrupted frame
      break;
      case ONE:
        // frame |= 1<<(FRAME_SIZE - (decode_index + 1));
        // if(decode_index < FRAME_SIZE - 1) {
        //   frame ^= 0x2000;
        // }
        if(decode_index < 8) {
          // first data and payload
          frame |= (uint16_t)(1 << (decode_index + 2));
          frame ^= 0x2000;
        } else if (decode_index == 8) {
          // parity received 
          frame |= 0x0002;
          frame ^= 0x2000;
        } else {
          // stop bit excluded from pariy
          frame |= 0x0001;
        }
      break;
      // Don't need to do anything for zero since frame is
      // initialized as all zeros
    }
    if((++decode_index) != FRAME_SIZE){ // decoding still WIP
      return;
    }
    // TIMG6->COUNTERREGS.LOAD = 0; // why isn't this disabling the timer?
    Receiver_StopBitTimer();
    // Begin frame interpretation
    #if DEBUG_MODE_RC != 4 && DEBUG_MODE_RC != 5 // don't validate in debug mode 4
    if(((frame & 0x6000) != 0) || ((frame & 0x01) == 0)){ // dropped bit, failed parity, or invalid stop bit
      frame = 0;
      return;
    }
    #endif
    // Push validated bit vector
    //decodeDataType transmitted_command = parse_frame((decodeDataType) (frame & 0xFF));
    decodeDataType transmitted_command = (decodeDataType)((frame >> 2) & 0xFF);
    DecodeFifo_Put(transmitted_command);
    frame = 0; // clear all bits, including bit 0
  }
}

void Receiver_DFT(int32_t *Mag_1, int32_t *Mag_2){
  uint16_t samples[16];

  uint32_t saved = __get_PRIMASK();
  __disable_irq();

  uint8_t head = circular_buffer_head;

  for(uint32_t i = 0; i < 16; i++){
    samples[i] = circular_buffer[(head + i) & 0x0F];
  }

  __set_PRIMASK(saved);
  static int32_t prevM1, prevM2;
  DFT_Init();
  for(uint8_t i = 0; i < 16; i++){
    // No critical section. Interrupts are guaranteed to overwrite
    // the oldest sample in the circular buffer, and the buffer is
    // read oldest first, so subsequent reads are unaffected by 
    // writes. 
    //DFT(i, circular_buffer[(circular_buffer_head + i) & 0x0F]);
    DFT(i, samples[i]);
  }
  // Low-pass IIR filter
  // prevM1 = 3 * (prevM1 >> 2) + (Mag1() >> 2);
  // prevM2 = 3 * (prevM2 >> 2) + (Mag2() >> 2);
  *Mag_1 = Mag1();
  *Mag_2 = Mag2();
}

rc_bit_t Receiver_DecodeBit(){
  int32_t Mag_1, Mag_2;
  Receiver_DFT(&Mag_1, &Mag_2);
  if(Mag_2 > ZERO_THRESHOLD){
    return ZERO;
  }
  if(Mag_1 > ONE_THRESHOLD){
    return ONE;
  }
  return ERROR;
}

// Starts decoding. Result will be pushed to stack if it's valid
void Receiver_Decode(void){ // called from main loop
  // decode_index = 0;
  // frame = 0x8000;
  // TIMG6->COUNTERREGS.LOAD = BAUD_PERIOD;
  uint32_t saved = __get_PRIMASK();
  __disable_irq();

  if((frame & 0x8000) != 0){
    __set_PRIMASK(saved);
    return;
  }

  Receiver_StopBitTimer();

  decode_index = 0;
  frame = 0x8000;

  // Subsequent decisions occur every 10 ms.
  TIMG6->COUNTERREGS.LOAD = BAUD_PERIOD - 1;

  TIMG6->CPU_INT.ICLR = 1;
  NVIC->ICPR[0] = (1 << 17);

  TIMG6->CPU_INT.IMASK = 1;
  TIMG6->COUNTERREGS.CTRCTL |= 1;

  __set_PRIMASK(saved);
}

// returns nonzero value if frame is actively being decoded. 
uint16_t Receiver_isDecoding(void){
  return frame & 0x8000;
}

// 8 up
// 4 down
// 2 left 
// 1 right

// lsb -> msb 
// 1000 00 00
// 010000 00
// 001000 00
// 000100 00

// 10 0000 
// 01 0000 
// 00 1000 
// 00 0100 
