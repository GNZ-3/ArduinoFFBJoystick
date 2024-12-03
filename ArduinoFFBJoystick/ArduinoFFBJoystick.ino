// Prcuedure
// Download Code as ZIP from https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
// スケッチー＞ライブラリをインクルードー＞ ZIP形式のライブラリをインクルード
// DigitalWriteFast.hをコピペ

// x,y,z 軸のFFB。FFBを利用しない場合はEffect無し、センサーエラー無効

#include "ArduinoFFBJoystick.h" // This
#include <Joystick.h>           // https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
#include <Wire.h>               // For I2C connection with TCA9548A
#include "DigitalWriteFast.h"   //Not sure. Original FFB
#include "BTS7960.h"            // https://github.com/luisllamasbinaburo/Arduino-BTS7960
#include <TCA9548.h>            // https://github.com/RobTillaart/TCA9548/tree/master
#include "AS5600.h"             // https://github.com/RobTillaart/AS5600/tree/master
// bts7960 3 wireing is the same as SMC3:https://www.xsimulator.net/community/threads/smc3-arduino-3dof-motor-driver-and-windows-utilities.4957/

PCA9546  I2CMux(0x70); 
// I2C adddress
#define xaxis 0x0
#define yaxis 0x1
#define zaxis 0x2
// Create magnetic encorder
AMS_5600 as5600;
//AS5600L as5600y;
//AS5600L as5600z;

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

BTS7960 motorx(ENBpin1, PWMpin1, ENApin1);
//BTS7960 motory(ENBpin2, PWMpin2, ENApin2);
//BTS7960 motorz(ENBpin3, PWMpin3, ENApin3);


// Create FFB related
Gains gains[2];
EffectParams effectparams[2];
s32 forces[2] = {0};
//s32 turn;

//bool isOutOfRange = false;

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
  delay(5000);
  Serial.begin(115200);
  Serial.println("Starting initialization");
  ROTATION_MAX=360.0;
  ROTATION_MID = ROTATION_MAX >> 1; // half

  // I2C initialize
  Wire.begin();
  Wire.setClock(400000);


  // Multiplexor initialize
  Serial.println("Multiplexor TCA9546A initialization.");
  Serial.print("TCA9548_LIB_VERSION: ");
  Serial.println(TCA9548_LIB_VERSION);
  if(I2CMux.begin() == false){
    Serial.println("FAILED !! multiplexor initilization failed.");
  }else{
    Serial.println("Sucessfully multiplexor initialized.");
  };
//  I2CMux.selectChannel(0);

  // Magnet encorder initalize
  Serial.println("AS5600 magnet encorder initialization.");
//  as5600x.setFastFilter(0); // milos, added to configure fast filter threshold (0-OFF is slow filter enable)
//  as5600x.setSlowFilter(0); // milos, added to configure slow filter or readout precision: 0-best(slowest), 3-worst(fastest)
//  as5600x.setDirection(AS5600_CLOCK_WISE); // For just in case
//  as5600x.resetCumulativePosition(ROTATION_MID); // milos, initialize at 0deg at startup
  if( ams5600.detectMagnet()==1) ){
    Serial.print("AS5600 on magnet detected.");
  }else{
    Serial.print("FAILED!! AS5600 has no magnet.");
  };
  //I2CMux.disableChannel(0);
/*
  Serial.println("Starting AS5600 magnet encorder initialization for y axis");
  as5600y.begin();
  as5600y.setFastFilter(0); // milos, added to configure fast filter threshold (0-OFF is slow filter enable)
  as5600y.setSlowFilter(0); // milos, added to configure slow filter or readout precision: 0-best(slowest), 3-worst(fastest)
  as5600y.setDirection(AS5600_CLOCK_WISE); // For just in case
  as5600y.resetCumulativePosition(ROTATION_MID); // milos, initialize at 0deg at startup

  Serial.println("Starting AS5600 magnet encorder initialization for z axis");
  as5600z.begin();
  as5600z.setFastFilter(0); // milos, added to configure fast filter threshold (0-OFF is slow filter enable)
  as5600z.setSlowFilter(0); // milos, added to configure slow filter or readout precision: 0-best(slowest), 3-worst(fastest)
  as5600z.setDirection(AS5600_CLOCK_WISE); // For just in case
  as5600z.resetCumulativePosition(ROTATION_MID); // milos, initialize at 0deg at startup
*/
  Serial.println("Starting BTS7960 motor initialization for x axis");
  motorx.Enable();
  motorx.Stop();
  /*
  Serial.println("Starting BTS7960 motor initialization for y axis");
  motory.Enable();
  motory.Stop();
  Serial.println("Starting BTS7960 motor initialization for z axis");
  motorz.Enable();
  motorz.Stop();
*/
//set Axis gains
  gains[0].totalGain = 50;  //x axis
  gains[0].springGain = 80;
  gains[1].totalGain = 50;  //y axis
  gains[1].springGain = 70;
  gains[2].totalGain = 50;  //z axis
  gains[2].springGain = 70;
//  Joystick.setGains(gains);
//set Timer3
/*
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
  Joystick.getUSBPID();

*/
}
void loop() {
  delay(1000);
  Serial.println("Loop");

    
  //Set Joystick posiion from magnetic encorder
  Serial.println("chanel opem");
  I2CMux.selectChannel(0);
  delay(1000);

 //s32 positionx = as5600x.getCumulativePosition() - ROTATION_MID;
  //float test =as5600x.getCumulativePosition();
  uint16_t test2 = as5600x.rawAngle();
  Serial.print( as5600x.readAngle() );
Serial.print("Read completed  ");
  //I2CMux.disableChannel(0);
Serial.print(test2);

//Serial.println("  close channel");
/*
  s32 positiony = as5600y.getCumulativePosition() - ROTATION_MID;
  s32 positionz = as5600z.getCumulativePosition() - ROTATION_MID;

  Joystick.setXAxis( positionx );
  Joystick.setYAxis( positiony );
  Joystick.setZAxis( positionz );

  //set Axis Spring Effect Param
  effectparams[0].springMaxPosition = ROTATION_MAX;         //x axis
  effectparams[0].springPosition = positionx;
  effectparams[1].springMaxPosition = ROTATION_MAX;         //y axis
  effectparams[1].springPosition = positionx;
  effectparams[2].springMaxPosition = ROTATION_MAX;         //z axis
  effectparams[2].springPosition = positionx;

//  Joystick.setEffectParams(effectparams);

//  Joystick.getForce(forces);

//  (forces[0] > 0)? motorx.TurnLeft( forces[0] ) : motorx.TurnRight( -forces[0] )
  if(forces[0] > 0){
    motorx.TurnLeft( abs(forces[0]) );
  }else{
    motorx.TurnRight( abs(forces[0]) );
  }

  (forces[1] > 0)? motory.TurnLeft( forces[0] ) : motory.TurnRight( -forces[0] )
  (forces[2] > 0)? motorz.TurnLeft( forces[0] ) : motorz.TurnRight( -forces[0] )
*/

}
