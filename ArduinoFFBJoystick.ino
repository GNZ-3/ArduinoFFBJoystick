// Prcuedure
// Download Code as ZIP from https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
// スケッチー＞ライブラリをインクルードー＞ ZIP形式のライブラリをインクルード
// DigitalWriteFast.hをコピー
#include "ArduinoFFBJoystick.h" // This
#include <Joystick.h>           // https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
#include "DigitalWriteFast.h"   //
#include "BTS7960.h"            // Motor Driver: https://github.com/luisllamasbinaburo/Arduino-BTS7960
#include <AS5600.h>             // Magnet encorder: https://github.com/RobTillaart/AS5600
#include <TCA9548.h>            // Multiplexor :https://github.com/RobTillaart/TCA9548

#define TOTALAXIS 2

typedef enum axis{
    xaxis=0,
    yaxis=1,
    Zaxis=2,
};

// Create magnetic encorder
AS5600 as5600[TOTALAXIS];

// create motors
/* Wireing is the same as SMC3. Please refer to https://www.xsimulator.net/community/threads/smc3-arduino-3dof-motor-driver-and-windows-utilities.4957/
  ENA ----> IN1/RPWM
  ENB --+-> EN1/R_EN
        +-> EN2/L_EN (ie connect ENB to both R_EN and L_EN)
  PWM ----> IN2/L_PWM
https://github.com/luisllamasbinaburo/Arduino-BTS7960#usage
  BTS7960 motor1(L_EN, R_EN, L_PWM, R_PWM);
      ----> BTS7960 motor1(ENB, PWM, ENA);
*/
const int ENApin1 =2;          // ENA output pin for Motor H-Bridge 1 (ie PortD bit position) 
const int ENBpin1 =3;          // ENB output pin for Motor H-Bridge 1 (ie PortD bit position)
const int ENApin2 =4;          // ENA output pin for Motor H-Bridge 2 (ie PortD bit position)
const int ENBpin2 =5;          // ENB output pin for Motor H-Bridge 2 (ie PortD bit position)
const int ENApin3 =6;          // ENA output pin for Motor H-Bridge 3 (ie PortD bit position)
const int ENBpin3 =7;          // ENB output pin for Motor H-Bridge 3 (ie PortD bit position)
const int PWMpin1 =9;          // PWM output pin for Motor 1   
const int PWMpin2 =10;         // PWM output pin for Motor 2    
const int PWMpin3 =11;         // PWM output pin for Motor 3 
BTS7960 motor[] = {
  BTS7960(ENBpin1, PWMpin1, ENApin1),
  BTS7960(ENBpin2, PWMpin2, ENApin2),
  BTS7960(ENBpin3, PWMpin3, ENApin3)
};

// Create multiplexor
TCA9548 MP(0x70);

// Create FFB jyoystick
Gains gains[TOTALAXIS];
EffectParams effectparams[TOTALAXIS];
s32 forces[TOTALAXIS] = {0};

/*
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  0, 0,                  // No Button count, no Hat Switch count
  true, true, true,     // X and Y, but no Z Axis
  false, false, false,   //  Rx, Ry, no Rz
  false, false,          // No rudder or throttle
  false, false, false);    // No accelerator, brake, or steering
*/

unsigned long prev, next, interval;

void setup() {
// Serial initialize and show lib version.
delay(5000);
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("ARDUINOFFBJOYSTICK_LIB_VERSION: ");
  Serial.println(ARDUINOFFBJOYSTICK_LIB_VERSION);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  Serial.print("TCA9548_LIB_VERSION: ");
  Serial.println(TCA9548_LIB_VERSION);

  // Joystick parmeters
  ROTATION_MAX=360.0;
  ROTATION_MID = ROTATION_MAX >> 1; // half
  gains[0].totalGain = 50;  //x axis gain
  gains[0].springGain = 80;
  gains[1].totalGain = 50;  //y axis gain
  gains[1].springGain = 70;
  gains[2].totalGain = 50;  //z axis ghain
  gains[2].springGain = 70;
/*
  Joystick.setGains(gains);
*/

  // I2C Multiplexor initialize
  Serial.print("Starting Multiplexor initialization.");
  Wire.begin();
  Wire.setClock(400000);
  if (MP.begin() == false){
    Serial.println(" FAILED!!");
  }else{
    Serial.println(" Success.");
  }

  // AS5600 is connected through Multiplexor and initialize.
  Serial.print("Starting AS5600 magnet encorder initialization.");
  for(int i=0; i<TOTALAXIS;i++){
    MP.enableChannel(i);
    if( as5600[i].isConnected() ){
      as5600[i].begin();
      as5600[i].setFastFilter(0); // milos, added to configure fast filter threshold (0-OFF is slow filter enable)
      as5600[i].setSlowFilter(0); // milos, added to configure slow filter or readout precision: 0-best(slowest), 3-worst(fastest)
      as5600[i].setDirection(AS5600_CLOCK_WISE); // For just in case
      as5600[i].resetCumulativePosition(ROTATION_MID); // milos, initialize at 0deg at startup
      Serial.print(" Success ch:" + String(i));
    }else{
      Serial.print("FAILED ch:" + String(i));
    }
    MP.disableChannel(i);
    delay(100);  //Enough time for switching other channel
  }
  Serial.println("");

// BTS7960 motor driver initialize
  Serial.print("Starting BTS7960 motor initialization.");
  for(int i=0; i<TOTALAXIS;i++){
    motor[i].Enable();
    motor[i].Stop();
    Serial.print(" Success :" + String(i));
  }
  Serial.println("");
  prev=millis();
  interval = 100;
//set Timer3
    cli();
    TCCR3A = 0; //set TCCR1A 0
    TCCR3B = 0; //set TCCR1B 0
    TCNT3  = 0; //counter init
    OCR3A = 399;
    TCCR3B |= (1 << WGM32); //open CTC mode
    TCCR3B |= (1 << CS31); //set CS11 1(8-fold Prescaler)
    TIMSK3 |= (1 << OCIE3A);
    sei();
}
//ISR
ISR(TIMER3_COMPA_vect){
/* Please care to remove this
  Joystick.getUSBPID();
*/
}
void loop() {
  unsigned long curr = millis();
  if ((curr - prev) >= interval) { 
    prev = curr;
    //Read magnetic encorder and set Joystick posiion
    s32 position[TOTALAXIS];
    for(int i=0; i<TOTALAXIS;i++){
      MP.enableChannel(i);
      position[i] = as5600[i].getCumulativePosition() - ROTATION_MID;
      Serial.print("[AS5600 ch:" + String(i) + "=" + String(position[i]) + "] ");
      MP.disableChannel(i);
    }
    //set Axis Spring Effect Param
    effectparams[0].springMaxPosition = ROTATION_MAX;         //x axis
    effectparams[0].springPosition = position[0];
    effectparams[1].springMaxPosition = ROTATION_MAX;         //y axis
    effectparams[1].springPosition = position[1];
    effectparams[2].springMaxPosition = ROTATION_MAX;         //z axis
    effectparams[2].springPosition = position[2];
/*
    Joystick.setXAxis( position[0] );
    Joystick.setYAxis( position[1] );
    Joystick.setZAxis( position[2] );
    Joystick.setEffectParams(effectparams);
    Joystick.getForce(forces);
    */
  // Apply force to motor
    for(int i=0; i<TOTALAXIS;i++){
      (forces[i] > 0)? motor[i].TurnLeft( forces[i] ) : motor[i].TurnRight( -forces[i] );
      Serial.print("[Force:" + String(i) + "=" + String(forces[i]) + "] ");
    }
    Serial.println("");
  }
}
