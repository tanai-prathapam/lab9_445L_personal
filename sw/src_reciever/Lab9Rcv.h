#ifndef LAB9RCV
#define LAB9RCV 1
// 1: Test ADC input
// 2: Test frequency filtering behavior
// 3: Stream time-domain signal to UART to test ADC/mic integrity
// 0: Run Lab 9
#define DEBUG_MODE_RC 2

#include <stdint.h>

#if DEBUG_MODE_RC == 1
extern uint16_t adc_output;
#endif
//movement commands
#define CMD_STOP      0x00U
#define CMD_FORWARD   0x01U
#define CMD_BACKWARD  0x02U
#define CMD_LEFT      0x04U
#define CMD_RIGHT     0x08U
// 0 = original CLI control, 1 = controller commands
#define MOVEMENT_MODE 1

//adjustable speed starting values
#define ROBOT_SPEED   200U //200 = 20 rpm
#define MOVE_TICKS    50U //50 updates means each movement lasts 0.5 secs

#endif