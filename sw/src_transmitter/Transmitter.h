// File **********Transmitter.h***********
// Possible prototype for Lab9 
// Programs to implement transmitter functionality   
// EE445L Spring 2021
//    Jonathan W. Valvano 7/15/26

#ifndef __TRANSMITTER_H__
#define __TRANSMITTER_H__

#include <stdint.h>

#define TX_FORWARD  0x01
#define TX_BACKWARD 0x02
#define TX_LEFT     0x04
#define TX_RIGHT    0x08

void Transmitter_Init(void);

// Non-blocking: 1 = accepted and 0 = busy, command not saved
int Transmitter_Send(uint8_t command);
int Transmitter_IsBusy(void);

#endif
