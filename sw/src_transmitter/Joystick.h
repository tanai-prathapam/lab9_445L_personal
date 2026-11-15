// File **********Joystick.h***********
// Lab 9
// Programs to interface with Switch buttons and Joystick   
// EE445L Fall 2026
//    Jonathan W. Valvano 7/15/26

// Possible hardware interface
// 2-bit input, positive logic switches, positive logic software
//   bit1 PA28 SW1 switch 
//   bit0 PA27 SW2 switch 
// Joystick
//   PA24 VRx ADC0 channel 3
//   PA25 VRy ADC0 channel 2
//   PA26 SW positive logic internal pulldown

    // write this
    // ***solution***

#ifndef JOYSTICK_H
#define JOYSTICK_H

extern uint32_t x, y;
extern uint32_t central_pos_x;
extern uint32_t central_pos_y;
extern uint32_t max_pos;

void Joystick_Init();
void Joystick_direction(uint32_t *x, uint32_t *y);

#endif
