/* Lab9main.c
 * Jonathan Valvano
 * v2.0.3
 * July 16, 2026
 *
 */

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/SSD1306.h"
#include "../inc/ADC.h"
#include "../inc/Timer.h"
#include "../inc/Tachometer.h"
#include "../inc/MotorControl.h"
#include "../inc/CLI.h"
#include "Receiver.h"

#include "PIController.h"

// Watch these variables in the debugger.
// Units are 0.1 RPM.
volatile uint32_t LeftSpeed;
volatile uint32_t RightSpeed;

uint32_t Duty,Period,Change;
uint32_t Left,Center,Right;

PIController L_PI, R_PI;

// Receiver microphone
//   PB19 analog input, ADC1 channel 6
// Motor
//   PB4  Motor_PWML=PB.4/TIMA1_C0;
//   PB1  Motor_PWMR=PB.1/TIMA1_C1;
//   PB0  Motor_DIR_L  1 means forward, 0 means backward
//   PB16 Motor_DIR_R, ***change from 2.0.0***
// tachometer
//   PB8  ELA  TA0_C0
//   PB7  ERB  not connected
//   PB6  ELB  not connected
//   PB12 ERA  TA0_C1
// Negative Logic Bumper switches
//   PA27 Left, Bump 0, 
//   PB15 Center Left, Bump 1, 
//   PA28 Center Right, Bump 2
//   PA31 Right, Bump 3
// SSD1306 I2C OLED
//   PB2  SCL I2C clock
//   PB3  SDA I2C data
// IR analog distance sensors
//   PA26 Right ADC0_1
//   PB24 Center ADC0_5 (remove J9)
//   PA24 Left ADC0_3
// TF Luna TOF distance sensor
//   TM4C MSPM0
//   PB1  PA8 TxD microcontroller sensor RxD pin 2
//   PB0  PA9 RxD microcontroller sensor TxD pin 3
// LaunchPad pins
//   PA0  red LED1
//   PB22 BLUE LED2
//   PB26 RED LED2
//   PB27 GREEN LED2
//   PA18 S1 positive logic switch
//   PB21 S2 negative logic switch
// Timers
//   TIMA0 for tachometer input capture
//   TIMA1 for PWM to motors
//   TIMG0 100 Hz periodic interrupt 
//         to check for stopped motors
//         to run motor controller
//   TIMG8 to sample ADC every 80us, 12.5kHz
// Remove +5V jumper on RSLK2 from debugger to target
// Remove J9: Disconnects thermistor from PB24
// Remove J16 J17 J18: Disconnects light sensor from the microcontroller PA22 PA27 PA26
// Insert J5: Connects PB22 to blue LED2
// Insert J6: Connects PB26 to red LED2
// Insert J7: Connects PB27 to green LED2

uint32_t Bump;
int main(void){ // use to run Lab 9 receiver
// ****ECE445L write this ****

 
  __disable_irq();
  LaunchPad_Init();
  Clock_Init_HFXT_40_80MHz(0);  // 0.005% accurate running off external crystal oscillator
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_OutBuffer();

  MC_Init(); // Initializes motors, tachometers, and bumpers
  TimerG0_IntArm(10000,40,2);
  uint32_t starter_RPM_to_send = 0; // divided by 10. e.g: 100 gives 10 RPM
  #if FF_MODE
    MC_PIController_changeparamsFF(&L_PI, 5, 5, starter_RPM_to_send, L_MOTOR_FFCONFIG);
    MC_PIController_changeparamsFF(&R_PI, 5, 5, starter_RPM_to_send, R_MOTOR_FFCONFIG);
  #else
    MC_PIController_changeparams(&L_PI, 9863, 772, starter_RPM_to_send);
    MC_PIController_changeparams(&R_PI, 9863, 772, starter_RPM_to_send);
  #endif
  __enable_irq();
  CLI_Init(&L_PI, &R_PI);
  while(1){
    CLI_Process();
    uint32_t rightRPM = Tachometer_GetRightrpm();
    uint32_t leftRPM = Tachometer_GetLeftrpm();
    SSD1306_SetCursor(0,3);
    SSD1306_OutString("RightRPM="); 
    SSD1306_OutUDec(rightRPM/10); SSD1306_OutChar('.'); SSD1306_OutUDec(rightRPM%10);

    SSD1306_SetCursor(0,4);
    SSD1306_OutString("LeftRPM="); 
    SSD1306_OutUDec(leftRPM/10); SSD1306_OutChar('.'); SSD1306_OutUDec(leftRPM%10);
  }
}
// runs in real time at 100 Hz
void TIMG0_IRQHandler(void){
  if((TIMG0->CPU_INT.IIDX) == 1){ // this will acknowledge
  // ****ECE445L write this ****


  }
}
