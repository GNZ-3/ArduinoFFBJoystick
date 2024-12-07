// If you already installed Joystick library, uninstall it
// Download Code as ZIP from https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
// Sketch->library-Install from zip to install downloaded
// DigitalWriteFast.hをコピー

#include "ArduinoFFBJoystick.h" // This
#include "DigitalWriteFast.h"   //
#include <Joystick.h>           // https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
#include "BTS7960.h"            // Motor Driver:    https://github.com/luisllamasbinaburo/Arduino-BTS7960
#include <AS5600.h>             // Magnet encorder: https://github.com/RobTillaart/AS5600
#include <TCA9548.h>            // I2C Multiplexor: https://github.com/RobTillaart/TCA9548

#define TOTALAXIS 2 //Role, elevetor,yaw
#define LOOPINTERVAL 100  //100ms


// Create multiplexor
TCA9548 MP(MPDEFAULTADDRESS);

// Create magnetic encorder
AS5600 as5600[TOTALAXIS];

// create motors Each pin is defined in ArduinoFFBJoystick.h
BTS7960 motor[] = {
  BTS7960(ENBpin1, PWMpin1, ENApin1),
  BTS7960(ENBpin2, PWMpin2, ENApin2),
  BTS7960(ENBpin3, PWMpin3, ENApin3)
};

// Create FFB jyoystick
int32_t forces[TOTALAXIS] = {0};
Gains gains[TOTALAXIS];
EffectParams effectparams[TOTALAXIS];

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  8, 0,                  // No Button count, no Hat Switch count
  true, true, true,     // X and Y, but no Z Axis
  false, false, false,   //  Rx, Ry, no Rz
  false, false,          // No rudder or throttle
  false, false, false);    // No accelerator, brake, or steering

// Other parms
unsigned long prev, next;
int Axis_Center[TOTALAXIS]={0};

void setup() {
// Serial initialize and show lib version.
  delay(5000);
  s32 SetupError = 0x0;
  ROTATION_MAX = ENCODER_MAX_VALUE-1;
  ROTATION_MID = ENCODER_MAX_VALUE >> 1 ; // half
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("ARDUINOFFBJOYSTICK_LIB_VERSION: ");
  Serial.println(ARDUINOFFBJOYSTICK_LIB_VERSION);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  Serial.print("TCA9548_LIB_VERSION: ");
  Serial.println(TCA9548_LIB_VERSION);
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
  for(int i=0; i<=TOTALAXIS;i++){
    MP.enableChannel(i);
    if( as5600[i].isConnected() ){
      as5600[i].begin();
      as5600[i].setFastFilter(0); // milos, added to configure fast filter threshold (0-OFF is slow filter enable)
      as5600[i].setSlowFilter(0); // milos, added to configure slow filter or readout precision: 0-best(slowest), 3-worst(fastest)
      as5600[i].setDirection(AS5600_CLOCK_WISE); // For just in case
      as5600[i].resetCumulativePosition(Axis_Center[i]); // milos, initialize at 0deg at startup
      Serial.print(" Success ch:");
      Serial.print(i);
    }else{
      Serial.print("FAILED ch:" );
      Serial.print(i);
    }
    MP.disableChannel(i);
    delay(100);  //Enough time for switching other channel
  }
  Serial.println("");

// BTS7960 motor driver initialize
  Serial.print("Starting BTS7960 motor initialization.");
  for(int i=0; i<=TOTALAXIS;i++){
    motor[i].Enable();
    motor[i].Stop();
    Serial.print(" Success :");
    Serial.print(i);
  }
  Serial.println("");

// Joystick initialize
  Serial.print("Starting Joystick initialization.");

  gains[0].totalGain = 100;  //x axis gain
  gains[0].springGain = 100;
  gains[1].totalGain = 100;  //y axis gain
  gains[1].springGain = 70;
  gains[2].totalGain = 100;  //z axis ghain
  gains[2].springGain = 70;
  Joystick.setGains(gains);
  Joystick.setXAxisRange(0, 1023);
  Joystick.setYAxisRange(0, 1023);
  Joystick.setYAxisRange(0, 1023);
  Joystick.begin(true);
  Serial.println("\tSuccess");

  prev = millis();

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
  Joystick.getUSBPID();
}

void loop() {
  unsigned long curr = millis();
  if ((curr - prev) >= LOOPINTERVAL) { 
    prev = curr;
    //Read magnetic encorder and set Joystick posiion
    s32 position[TOTALAXIS];
    for(int i=0; i<=TOTALAXIS;i++){
      MP.enableChannel(i);
      position[i] = as5600[i].getCumulativePosition();
      //position[i] = as5600[i].getCumulativePosition() - ROTATION_MID;
      //position[i] = as5600[i].readAngle();

      Serial.print("Encrdr ");
      Serial.print(i);
      Serial.print(":");
      Serial.print(position[i]);
      Serial.print("\t");
      MP.disableChannel(i);
      delay(100);
    }
    //set Axis Spring Effect Param
    effectparams[0].springMaxPosition = 1023;         //x axis
    effectparams[0].springPosition = position[0];
    effectparams[1].springMaxPosition = 1023;         //y axis
    effectparams[1].springPosition = position[1];
    effectparams[2].springMaxPosition = 1023;         //z axis
    //effectparams[2].springPosition = position[2];
    Joystick.setEffectParams(effectparams);


    Joystick.setXAxis( position[0] );
    Joystick.setYAxis( position[1] );
    Joystick.setZAxis( position[2] );


    Joystick.getForce(forces);

     
      // Apply force to motor
    for(int i=0; i<=TOTALAXIS;i++){
   //   (forces[i] > 0)? motor[i].TurnLeft( forces[i] ) : motor[i].TurnRight( -forces[i] );
      //Serial.print("[Force:" + String(i) + "=" + String(forces[i]) + "] ");
      Serial.print("Force ");
      Serial.print(i);
      Serial.print(":");
      Serial.print(forces[i]);
      Serial.print("\t");
    }
    Serial.println("");
  }
}
