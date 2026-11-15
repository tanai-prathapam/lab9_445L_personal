// File **********Joystick.c***********
// Lab9
// Programs to interface with Switch buttons and Joystick   
// ECE445L Fall 2026
//    Jonathan W. Valvano 7/15/26
// define your hardware interface
// 2-bit input, positive logic switches, positive logic software
//   bit1 PA28 SW1 switch 
//   bit0 PA27 SW2 switch 
// Joystick
//   PA24 VRx ADC0 channel 3
//   PA25 VRy ADC0 channel 2
//   PA26 SW positive logic internal pulldown

#include <stdint.h>
#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/ADC.h"
#include "../Joystick.h"
#include "../Transmitter.h"
// ****ECE445L write this ****

void Joystick_Init(){
    ADC_InitDual(ADC0, 3, 2, 0);
}

void Joystick_direction(uint32_t *x, uint32_t *y) {
    //Sample ADC Channels
    ADC_InDual(ADC0, x, y);
    
    //Build Directional Command
    uint32_t upper_tolerance_x = central_pos_x + max_pos * 0.1; //10% tolerance
    uint32_t lower_tolerance_x = central_pos_x - max_pos * 0.1; 
    uint32_t upper_tolerance_y = central_pos_y + max_pos * 0.1; //10% tolerance
    uint32_t lower_tolerance_y = central_pos_y - max_pos * 0.1; 
    uint8_t hex_command = 0;
    if (*x < lower_tolerance_x) hex_command = 0x08; // joystick right
    if (*x > upper_tolerance_x) hex_command = 0x04; //joystick left
    if (*y < lower_tolerance_y) hex_command = 0x02; // joystick back
    if (*y > upper_tolerance_y) hex_command = 0x01; //joystick forward

    //bits: 7_6_5_4_3_2_1_0
    // reserved_reserved_reserved_reserved_right_left_backward_forward
    Transmitter_Send(hex_command);
}




 
