// Prcuedure
// Download Code as ZIP from https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
// スケッチー＞ライブラリをインクルードー＞ ZIP形式のライブラリをインクルード
// DigitalWriteFast.hをコピペ

// x,y,z 軸のFFB。FFBを利用しない場合はEffect無し、センサーエラー無効

#include "ArduinoFFBJoystick.h" // This
#include <Joystick.h>           // https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
#include "DigitalWriteFast.h"   //Not sure. Original FFB
#include "BTS7960.h"            // https://github.com/luisllamasbinaburo/Arduino-BTS7960
#include <AS5600.h>             // https://github.com/RobTillaart/AS5600
#include <TCA9548.h>            // https://github.com/RobTillaart/TCA9548
#include <Wire.h> 

// bts7960 3 wireing is the same as SMC3:https://www.xsimulator.net/community/threads/smc3-arduino-3dof-motor-driver-and-windows-utilities.4957/

typedef enum axis{
    xaxis=0,
    yaxis=1,
    Zaxis=2,
};
#define TOTALAXIS 2


// Create magnetic encorder
AS5600 as5600[TOTALAXIS];

/* PIN for bts7960
https://www.xsimulator.net/community/threads/smc3-arduino-3dof-motor-driver-and-windows-utilities.4957/
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

// create motors
BTS7960 motor[] = {
  BTS7960(ENBpin1, PWMpin1, ENApin1),
  BTS7960(ENBpin2, PWMpin2, ENApin2),
  BTS7960(ENBpin3, PWMpin3, ENApin3)
};

// Create multiplexor
TCA9548 MP(0x70);

// Create FFB related
Gains gains[TOTALAXIS];
EffectParams effectparams[TOTALAXIS];
s32 forces[TOTALAXIS] = {0};

//Create joystick
/*
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  0, 0,                  // No Button count, no Hat Switch count
  true, true, true,     // X and Y, but no Z Axis
  false, false, false,   //  Rx, Ry, no Rz
  false, false,          // No rudder or throttle
  false, false, false);    // No accelerator, brake, or steering
*/
void setup() {
  ROTATION_MAX=360.0;
  ROTATION_MID = ROTATION_MAX >> 1; // half

  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  Serial.print("TCA9548_LIB_VERSION: ");
  Serial.println(TCA9548_LIB_VERSION);


  // I2C initialize
  Serial.println("Starting Multiplexor initialization.");
  Wire.begin();
  Wire.setClock(400000);
  if (MP.begin() == false)
  {
    Serial.println("FAILED to initialize Multiplexor");
  }else{
    Serial.println("Success to initialize Multiplexor");
  }

  Serial.println("Starting AS5600 magnet encorder initialization.");
  for(int i=0; i<TOTALAXIS;i++){
    MP.enableChannel(i);
    if( as5600[i].isConnected() ){
      as5600[i].begin();
      as5600[i].setFastFilter(0); // milos, added to configure fast filter threshold (0-OFF is slow filter enable)
      as5600[i].setSlowFilter(0); // milos, added to configure slow filter or readout precision: 0-best(slowest), 3-worst(fastest)
      as5600[i].setDirection(AS5600_CLOCK_WISE); // For just in case
      as5600[i].resetCumulativePosition(ROTATION_MID); // milos, initialize at 0deg at startup
      Serial.print("Success to initilize AS5600 on ch:" + String(i));
    }else{
      Serial.print("FAIL to initilize AS5600 on ch: " + String(i));
    }
    MP.disableChannel(i);
    delay(100);
  }

  Serial.println("Starting BTS7960 motor initialization.");
  for(int i=0; i<TOTALAXIS;i++){
    motor[i].Enable();
    motor[i].Stop();
    Serial.print("Success to initilize motor:" + String(i));
  }

//set Axis gains
  gains[0].totalGain = 50;  //x axis
  gains[0].springGain = 80;
  gains[1].totalGain = 50;  //y axis
  gains[1].springGain = 70;
  gains[2].totalGain = 50;  //z axis
  gains[2].springGain = 70;

//gnz  Joystick.setGains(gains);

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
//gnz  Joystick.getUSBPID();
}
void loop() {
  //Set Joystick posiion from magnetic encorder
  s32 position[TOTALAXIS];
  for(int i=0; i<TOTALAXIS;i++){
    position[i] = as5600[i].getCumulativePosition() - ROTATION_MID;

  }
  /*
  s32 position[0] = as5600[0].getCumulativePosition() - ROTATION_MID;
  s32 position[1] = as5600[1].getCumulativePosition() - ROTATION_MID;
  s32 position[2] = as5600[2].getCumulativePosition() - ROTATION_MID;
  Joystick.setXAxis( position[0] );
  Joystick.setYAxis( position[1] );
  Joystick.setZAxis( position[2] );
*/
  //set Axis Spring Effect Param
  effectparams[0].springMaxPosition = ROTATION_MAX;         //x axis
  effectparams[0].springPosition = position[0];
  effectparams[1].springMaxPosition = ROTATION_MAX;         //y axis
  effectparams[1].springPosition = position[1];
  effectparams[2].springMaxPosition = ROTATION_MAX;         //z axis
  effectparams[2].springPosition = position[2];
//  Joystick.setEffectParams(effectparams);

//  Joystick.getForce(forces);
  for(int i=0; i<TOTALAXIS;i++){
    (forces[i] > 0)? motor[i].TurnLeft( forces[i] ) : motor[i].TurnRight( -forces[i] );
  }

}
