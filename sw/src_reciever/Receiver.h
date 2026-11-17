// Receiver.h
// Programs to implement receiver functionality   
// ECE445L Fall 2026
//    Jonathan W. Valvano 7/16/26

// ----------------------------------------------------------------------------
// Hardware/software assigned to reciever
//   Timer G8 ADC1 samples sound
//   DecodeFIFO linkage from  ADC to Decoder
//   main-loop runs decoder

#ifndef __RECEIVER_H__
#define __RECEIVER_H__
#include <stdint.h>
// write this
// ***solution***
// Solution to Lab5 (not to be shown to students)

// Size of a frame, not including the start bit
#define FRAME_SIZE 10

// Threshold to detect 0
#define ZERO_THRESHOLD 3000
// Threshold to detect 1
#define ONE_THRESHOLD 2048

typedef enum RC_Bit{
  // 781.25Hz
  ONE,
  // 1562.5Hz - checked second, as 781.25Hz signals 
  //   can have 1562.5Hz harmonics
  ZERO,
  // Neither signal exceeds detection threshold
  ERROR
} rc_bit_t;


//helper
uint8_t parse_frame (decodeDataType frame_main);

// Sample time is 25ns*period*prescale
// Timer G8 is power Domain PD0
void Receiver_Init(uint32_t period,uint32_t prescale);

// Runs a 16-sample DTFS on the 16 most recent samples in the circular buffer. 
// Mag1 and Mag2 store the first and second frequency amplitudes, respectively.
void Receiver_DFT(int32_t *Mag_1, int32_t *Mag_2);

// Performs 16-sample DTFS on 16 most recent ADC samples and returns code 
// corresponding to detected bit. ZERO > ONE > ERROR for priority. 
rc_bit_t Receiver_DecodeBit();

// Call this function to decode an incoming FM signal. Expected to be called 
// upon detecting the input low. Validated 8-bit vector will be pushed to FIFO.
void Receiver_Decode(void);

uint16_t Receiver_isDecoding(void);

#endif
