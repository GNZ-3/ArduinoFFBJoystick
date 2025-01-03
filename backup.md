// If you already installed Joystick library, uninstall it
// Download Code as ZIP from https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
// Sketch->library-Install from zip to install downloaded
// DigitalWriteFast.hをコピー

#include "ArduinoFFBJoystick.h" // This
//#include "DigitalWriteFast.h"   //Not used
//#include "Joystick.h"           // Did not work. https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
#include "Joystick.h"           // https://github.com/MHeironimus/ArduinoJoystickLibrary
#include "BTS7960.h"            // Motor Driver:    https://github.com/luisllamasbinaburo/Arduino-BTS7960
#include <AS5600.h>             // Magnet encorder: https://github.com/RobTillaart/AS5600
#include <TCA9548.h>            // I2C Multiplexor: https://github.com/RobTillaart/TCA9548

#define TOTALAXIS 3 //Role and yaw. Elevetor will be added later
#define LOOPINTERVAL 5000000  //100ms
#define BTS7960_ENA_1   7
#define BTS7960_LPWM_1  9
#define BTS7960_RPWM_1  10

#define BTS7960_ENA_2   8
#define BTS7960_LPWM_2  6
#define BTS7960_RPWM_2  11

#define BTS7960_ENA_3   12
#define BTS7960_LPWM_3  13
#define BTS7960_RPWM_3  3

// Create multiplexor
TCA9548 MP(MPDEFAULTADDRESS);

// Create magnetic encorder
AS5600 as5600[] = {
  AS5600(),
  AS5600(),
  AS5600()
};

// create motors Each pin is defined in ArduinoFFBJoystick.h
BTS7960 motor[] = {
  BTS7960(BTS7960_ENA_1, BTS7960_LPWM_1, BTS7960_RPWM_1),
  BTS7960(BTS7960_ENA_2, BTS7960_LPWM_2, BTS7960_RPWM_2),
  BTS7960(BTS7960_ENA_3, BTS7960_LPWM_3, BTS7960_RPWM_3),
};

// Create FFB jyoystick
int32_t forces[] = {0,0,0};
Gains gains[2];
EffectParams effectparams[2];
int position[] ={0,0,0};
int AxisMinLimit[] = {-32767,-32767,-32767};  //Windows joystick default range
int AxisMaxLimit[] = {32768,32768,32768};
int Axis_Offset[] = {0,0,0};
//s32 AxisCenter[TOTALAXIS-1] = {(ENCODER_MAX_VALUE+ENCODER_MIN_VALUE)/2};

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  8, 0,                  // No Button count, no Hat Switch count
  true, true, true,     // X and Y, but no Z Axis
  false, false, false,   //  Rx, Ry, no Rz
  false, false,          // No rudder or throttle
  false, false, false);    // No accelerator, brake, or steering

// Other parms
unsigned long prev, next;

void setup() {
// Serial initialize and show lib version.
  delay(5000);    // Sometimes FW did not work if loop starts. We have 5 sec for flash FW
  int SetupError = 0;
  //ROTATION_MAX = ENCODER_MAX_VALUE-1;
  //ROTATION_MID = ENCODER_MAX_VALUE >> 1 ; // half
  Serial.begin(115200);
  
  Serial.println(__FILE__);
  Serial.print("ARDUINOFFBJOYSTICK_LIB_VERSION: ");
  Serial.println(ARDUINOFFBJOYSTICK_LIB_VERSION);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  Serial.print("TCA9548_LIB_VERSION: ");
  Serial.println(TCA9548_LIB_VERSION);


  // I2C Multiplexor initialize
  Serial.println("\nStarting Multiplexor initialization.");
  Wire.begin();
  Wire.setClock(400000);

  if (MP.begin() == false){
    Serial.print("FAILED!!");
  }else{
    Serial.print("Success.");
    MP.disableAllChannels();
  }

  // AS5600 is connected through Multiplexor and initialize.
  Serial.println("\nStarting AS5600 magnet encorder initialization.");
  for(int i=0; i<TOTALAXIS;i++){
    Serial.print("\tch:");
    Serial.print(i);
    Serial.print(": ");
    MP.selectChannel(i);
    if( MP.isConnected(AS5600_DEFAULT_ADDRESS,i) ){
      as5600[i].begin();
      as5600[i].setFastFilter(0); // milos, added to configure fast filter threshold (0-OFF is slow filter enable)
      as5600[i].setSlowFilter(0); // milos, added to configure slow filter or readout precision: 0-best(slowest), 3-worst(fastest)
      as5600[i].setDirection(AS5600_CLOCK_WISE); // For just in case
      as5600[i].resetCumulativePosition(0); // ResetPosition let sensor unreadble
      Serial.print("Success");
      // Due to an resetCumulativePosition problem, let center as current position.
      Axis_Offset[i] = as5600[i].getCumulativePosition();
    }else{
      Serial.print("FAILED" );
      //Axis_Offset[i] = Axis_Offset[i];
    }
    MP.disableAllChannels();
    delay(100);  //Enough time for switching other channel
  }

  Serial.println("\nCheck AS5600 again to make sure it works.");
  for(int i=0; i<TOTALAXIS;i++){
    Serial.print("\tch:");
    Serial.print(i);
    Serial.print(": ");
    MP.selectChannel(i);
    if( MP.isConnected(AS5600_DEFAULT_ADDRESS,i) ){
      Serial.print(as5600[i].getCumulativePosition());
      Serial.print(" / ");
      Serial.print(Axis_Offset[i]);
    }else{
      Serial.print("FAILED" );
    }
    //MP.disableChannel(i);
    delay(100);  //Enough time for switching other channel
  }

// BTS7960 motor driver initialize
  Serial.println("\nStarting BTS7960 motor initialization.");
  for(int i=0; i<TOTALAXIS;i++){
    Serial.print("\t:ch:");
    Serial.print(i);
    Serial.print(" ");
    motor[i].Enable();
    motor[i].Stop();
    Serial.print("Success");
  }


// Joystick initialize
  Serial.println("\nStarting Joystick initialization.");
  gains[0].totalGain = 100;  //x axis gain
  gains[0].springGain = 100;
  gains[1].totalGain = 100;  //y axis gain
  gains[1].springGain = 70;
  gains[2].totalGain = 100;  //z axis ghain
  gains[2].springGain = 70;
  
  //Joystick.setGains(gains);

  Joystick.setXAxisRange(AxisMinLimit[0], AxisMaxLimit[0]);
  Joystick.setYAxisRange(AxisMinLimit[1], AxisMaxLimit[1]);
  Joystick.setZAxisRange(AxisMinLimit[2], AxisMaxLimit[2]);

  Joystick.begin();
  
  Serial.print("\t");
  Serial.print("Success");

  prev = millis();
  delay(5000);  

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
  Serial.println("\nInitialize completed.");

}

//ISR
ISR(TIMER3_COMPA_vect){
  //DynamicHID().RecvfromUsb();
  Joystick.getUSBPID();
}

void loop() {
  unsigned long curr = millis();
  if (true) {
//  if ((curr - prev) > LOOPINTERVAL) {
    //prev = curr;
    Serial.print("[ ");
    Serial.print(curr);
    Serial.print(" ]");
    //Read magnetic encorder and set Joystick posiion
    for(int i=0; i<TOTALAXIS;i++){
      MP.enableChannel(i);
      MP.selectChannel(i);
        Serial.print("\tEnc");
        Serial.print(i);
        Serial.print(":");
        if( MP.isConnected(AS5600_DEFAULT_ADDRESS,i) ){
          position[i] = as5600[i].getCumulativePosition();
          Serial.print(position[i]);
          //position[i] = position[i] - Axis_Offset[i];
        }else{
          //position[i] = Axis_Center[i];
          Serial.print("skip");
        }
        Serial.print("\tOff");
        Serial.print(i);
        Serial.print(":");
        Serial.print(Axis_Offset[i]);
      MP.disableChannel(i);
    }

    //set Axis Spring Effect Param
    effectparams[0].springMaxPosition = AxisMaxLimit[0];         //x axis
    effectparams[0].springPosition = position[0];
    effectparams[1].springMaxPosition = AxisMaxLimit[1];         //y axis
    effectparams[1].springPosition = position[1];
    effectparams[2].springMaxPosition = AxisMaxLimit[2];         //z axis
    effectparams[2].springPosition = position[2];

if ((curr - prev) > LOOPINTERVAL) {
    //Joystick.setEffectParams(effectparams);
}
    Joystick.setXAxis( position[0]*128 );
    Joystick.setYAxis( position[1]*128 );
    Joystick.setZAxis( position[2]*128 );

    Joystick.getForce(forces);

  //gnz
    forces[1]=position[0];
      
    // Apply force to motor
    for(int i=0; i<TOTALAXIS;i++){
//GNZ bug. System hang if analogwrite to motor 3
     // motor[i].Enable();
      (forces[i] > 0)? motor[i].TurnLeft( forces[i] ) : motor[i].TurnRight( -forces[i] );
      if(i !=2){
//        (forces[i] > 0)? motor[i].TurnLeft( forces[i] ) : motor[i].TurnRight( -forces[i] );
        //Serial.print("[Force:" + String(i) + "=" + String(forces[i]) + "] ");
      }
//gnz end
      Serial.print("\tForce");
      Serial.print(i);
      Serial.print(":");
      Serial.print(forces[i]);
    }
    Serial.println("");
    
  }
 // delay(500);
}
