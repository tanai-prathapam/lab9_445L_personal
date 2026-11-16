#ifndef LAB9RCV
#define LAB9RCV 1
// 1: Test ADC input
// 0: Run Lab 9
#define DEBUG_MODE_RC 1

#include <stdint.h>

#if DEBUG_MODE_RC == 1
extern uint16_t adc_output;
#endif
#endif