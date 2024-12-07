

#include "Arduino.h"
#include <Wire.h>
#define ARDUINOFFBJOYSTICK_LIB_VERSION             (F("0.1.0"))

#define MPDEFAULTADDRESS 0x70 // I2C Multiplexor default address
#define ENCODER_MAX_VALUE 2048
#define ENCODER_MIN_VALUE 0

/* Wireing is the same as SMC3. Please refer to https://www.xsimulator.net/community/threads/smc3-arduino-3dof-motor-driver-and-windows-utilities.4957/
  ENA ----> IN1/RPWM
  ENB --+-> EN1/R_EN
        +-> EN2/L_EN (ie connect ENB to both R_EN and L_EN)
  PWM ----> IN2/L_PWM
https://github.com/luisllamasbinaburo/Arduino-BTS7960#usage
  BTS7960 motor1(L_EN, R_EN, L_PWM, R_PWM);
      ----> BTS7960 motor1(ENB, PWM, ENA);
*/
#define ENApin1  2          // ENA output pin for Motor H-Bridge 1 (ie PortD bit position) 
#define ENBpin1 3          // ENB output pin for Motor H-Bridge 1 (ie PortD bit position)
#define ENApin2 4          // ENA output pin for Motor H-Bridge 2 (ie PortD bit position)
#define ENBpin2 5          // ENB output pin for Motor H-Bridge 2 (ie PortD bit position)
#define ENApin3 6          // ENA output pin for Motor H-Bridge 3 (ie PortD bit position)
#define ENBpin3 7          // ENB output pin for Motor H-Bridge 3 (ie PortD bit position)
#define PWMpin1 9          // PWM output pin for Motor 1   
#define PWMpin2 10         // PWM output pin for Motor 2    
#define PWMpin3 11         // PWM output pin for Motor 3 


typedef signed long s32;
s32 ROTATION_MAX; //milos
s32 ROTATION_MID; //milos
