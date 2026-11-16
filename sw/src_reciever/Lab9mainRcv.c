/* Lab9main.c
 * Jonathan Valvano
 * v2.0.3
 * July 16, 2026
 *
 */

#include <stdint.h>
#include "Lab9Rcv.h"
#include "DecodeFIFO.h"
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
#include "../inc/DFT.h"
#include "../inc/UART.h"

#include "PIController.h"

// For debugger testing:
// 1. Set Command to a value listed above.
// 2. Set CommandReady to 1.
//
// Later, the decoder can write a valid received byte here.
// Zero is STOP, so CommandReady distinguishes STOP from no message.
volatile uint32_t Command = CMD_STOP;
volatile uint32_t CommandReady = 0;

// Robot state
volatile uint32_t ActiveCommand = CMD_STOP;
volatile uint32_t MoveTicksLeft = 0;

static void Robot_Stop(void);
static void Robot_Start(uint32_t command);
static void Robot_CommandTick(void);

// Units are 0.1 RPM. will appear in debugger
volatile uint32_t LeftSpeed;
volatile uint32_t RightSpeed;
volatile uint32_t Bump;

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
//   TIMG6 to run at baud rate of FM receiver, period 8.192ms
//   TIMG8 to sample ADC every 80us, 12.5kHz
// Remove +5V jumper on RSLK2 from debugger to target
// Remove J9: Disconnects thermistor from PB24
// Remove J16 J17 J18: Disconnects light sensor from the microcontroller PA22 PA27 PA26
// Insert J5: Connects PB22 to blue LED2
// Insert J6: Connects PB26 to red LED2
// Insert J7: Connects PB27 to green LED2

volatile uint32_t Bump;
#if DEBUG_MODE_RC == 1
uint16_t adc_output;
int main(void){
  __disable_irq();
  LaunchPad_Init();
  Clock_Init_HFXT_40_80MHz(0);  // 0.005% accurate running off external crystal oscillator
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_OutBuffer();
  Receiver_Init(SAMPLE_PERIOD, 1);

  
  uint16_t max = 0;
  __enable_irq();
  while(1){
    adc_output = NoiseReject(ADC1_In());
    SSD1306_SetCursor(0, 2);
    SSD1306_OutString("ADC input: ");
    SSD1306_OutUDec(adc_output);
    if(adc_output > max){
      max = adc_output;
    }
    SSD1306_SetCursor(0, 3);
    SSD1306_OutString("max input: ");
    SSD1306_OutUDec(max);
  }
}
#elif DEBUG_MODE_RC == 2
int main(void){
  __disable_irq();
  LaunchPad_Init();
  Clock_Init_HFXT_40_80MHz(0);  // 0.005% accurate running off external crystal oscillator
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_OutBuffer();
  Receiver_Init(SAMPLE_PERIOD, 1);
  int32_t f1m, f2m;
  
  __enable_irq();
  while(1){
    Receiver_DFT(&f1m, &f2m);
    SSD1306_SetCursor(0, 2);
    SSD1306_OutString("f1 mag: ");
    SSD1306_OutUDec(f1m);
    SSD1306_SetCursor(0, 3);
    SSD1306_OutString("f2 mag: ");
    SSD1306_OutUDec(f2m);
    SSD1306_SetCursor(0, 4);
    if(f1m > ONE_THRESHOLD){
      SSD1306_OutString("f1: 1 ");
    } else{
      SSD1306_OutString("f1: 0 ");
      if(f2m > ZERO_THRESHOLD){
        SSD1306_OutString("f2: 1");
      } else{
        SSD1306_OutString("f2: 0");
      }
    }
    
  }
}
#elif DEBUG_MODE_RC == 3
int main(void){
  __disable_irq();
  LaunchPad_Init();
  Clock_Init_HFXT_40_80MHz(0);  // 0.005% accurate running off external crystal oscillator
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_OutBuffer();
  Receiver_Init(SAMPLE_PERIOD, 1);
  UART_Init();
  
  __enable_irq();
  while(1){
    decodeDataType sample;
    if(!DecodeFifo_Get(&sample)){
      continue;
    }
    UART_OutSDec(sample);
    UART_OutString("\n");
  }
}
#elif DEBUG_MODE_RC == 4 || DEBUG_MODE_RC == 5
int main(void){
  __disable_irq();
  LaunchPad_Init();
  Clock_Init_HFXT_40_80MHz(0);  // 0.005% accurate running off external crystal oscillator
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_OutBuffer();
  Receiver_Init(SAMPLE_PERIOD, 1);
  int32_t one_m, zero_m;
  
  __enable_irq();
  while(1){
    if(Receiver_DecodeBit() != ZERO){
      SSD1306_SetCursor(0, 2);
    SSD1306_OutString("no frame          ");
      continue;
    }
    SSD1306_SetCursor(0, 2);
    SSD1306_OutString("start bit detected");
    decodeDataType frame = 0;
    while(DecodeFifo_Get(&frame) != DECODEFIFOSUCCESS){
      Receiver_Decode();
      // WFI();
    }
    SSD1306_SetCursor(0, 3);
    SSD1306_OutUHex32(frame);
  }
}
#else
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

  // initialize the CLI before enabling interrupts in CLI mode.
  #if MOVEMENT_MODE == 0
  CLI_Init(&L_PI, &R_PI);
  #endif

  __enable_irq();

  while(1){
  #if MOVEMENT_MODE == 0
    CLI_Process();
  #endif

    uint32_t rightRPM = Tachometer_GetRightrpm();
    uint32_t leftRPM = Tachometer_GetLeftrpm();
    SSD1306_SetCursor(0,3);
    SSD1306_OutString("RightRPM="); 
    SSD1306_OutUDec(rightRPM/10); SSD1306_OutChar('.'); SSD1306_OutUDec(rightRPM%10);

    SSD1306_SetCursor(0,4);
    SSD1306_OutString("LeftRPM="); 
    SSD1306_OutUDec(leftRPM/10); SSD1306_OutChar('.'); SSD1306_OutUDec(leftRPM%10);

    #if MOVEMENT_MODE == 1
      SSD1306_SetCursor(0,0);
      SSD1306_OutString("Cmd=");
      SSD1306_OutUDec(ActiveCommand);
      SSD1306_OutString("   ");

      SSD1306_SetCursor(0,1);
      SSD1306_OutString("Bump=");
      SSD1306_OutUDec(Bump);
      SSD1306_OutString("   ");
    #endif

  }
}
#endif
// runs in real time at 100 Hz
void TIMG0_IRQHandler(void){
  if((TIMG0->CPU_INT.IIDX) == 1){ // this will acknowledge
  // ****ECE445L write this ****
  Tachometer_CheckForStopped();
  LeftSpeed = Tachometer_GetLeftrpm();
  RightSpeed = Tachometer_GetRightrpm();
  Bump = Bump_In();

  #if MOVEMENT_MODE == 0
    // Run the existing controller for CLI operation.
    MC_PIControlLoop(&L_PI, &R_PI);
  #else
    Robot_CommandTick();
  #endif

  }
}


static void Robot_Stop(void){
  Motor_Forward(0, 0);

  L_PI.speed_ref = 0;
  R_PI.speed_ref = 0;

  L_PI.prev_error = 0;
  R_PI.prev_error = 0;

  L_PI.prev_out = 0;
  R_PI.prev_out = 0;

  ActiveCommand = CMD_STOP;
  MoveTicksLeft = 0;
}

static void Robot_Start(uint32_t command){
  // clear previous corrections when starting a new movement.
  L_PI.prev_error = 0;
  R_PI.prev_error = 0;

  L_PI.prev_out = 0;
  R_PI.prev_out = 0;

  L_PI.speed_ref = ROBOT_SPEED;
  R_PI.speed_ref = ROBOT_SPEED;

  // casic curve turns stop the inside wheel.
  if(command == CMD_LEFT){
    L_PI.speed_ref = 0;
  }

  if(command == CMD_RIGHT){
    R_PI.speed_ref = 0;
  }

  ActiveCommand = command;
  MoveTicksLeft = MOVE_TICKS;
}

static void Robot_CommandTick(void){
  // stop and cancel the current command on any bumper press.
  // releasing the bumper does not restart the old command.
  if(Bump){
    CommandReady = 0;
    Robot_Stop();
    return;
  }

  if(CommandReady){
    uint32_t command = Command;
    CommandReady = 0;

    if(command == CMD_STOP){
      Robot_Stop();

    }else if(ActiveCommand == CMD_STOP &&
             LeftSpeed == 0 &&
             RightSpeed == 0){

      // start movement only when the robot is stopped.
      if(command == CMD_FORWARD ||
         command == CMD_BACKWARD ||
         command == CMD_LEFT ||
         command == CMD_RIGHT){
        Robot_Start(command);
      }
    }
  }

  if(ActiveCommand != CMD_STOP){
    if(MoveTicksLeft == 0){
      Robot_Stop();

    }else{
      uint32_t leftDuty =
          PIController_update(&L_PI, LeftSpeed);

      uint32_t rightDuty =
          PIController_update(&R_PI, RightSpeed);

      if(ActiveCommand == CMD_BACKWARD){
        Motor_Backward(leftDuty, rightDuty);
      }else{
        Motor_Forward(leftDuty, rightDuty);
      }

      MoveTicksLeft--;
    }
  }
}


  

