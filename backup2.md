// Update board ID:%userprofile%\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6
// 起動時に各軸をキャリブレーション、GUIの追加
// X軸詳細化(タイミングプーリ購入）
// カルマンフィルタの追加
// センタスプリングの追加（ツール起動だけでOK?）
// パラメータチューニング（スプリング強度、フリクション、センター）用の切替スイッチと調整用ロータリーエンコーダの追加
#include <digitalWriteFast.h>
#include "Joystick.h"           // FFB Joystick: https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
#include "BTS7960.h"            // Motor Driver: https://github.com/luisllamasbinaburo/Arduino-BTS7960
#include "SSD1306Ascii.h"       // LCD Driver:   https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiAvrI2c.h"

#define BTS7960_ENA_1   7         //X motor
#define BTS7960_LPWM_1  6
#define BTS7960_RPWM_1  11        //Pin 5 (and 3) did not work. Try another pin 11
#define BTS7960_ENA_2   8         //Y motor
#define BTS7960_LPWM_2  9
#define BTS7960_RPWM_2  10

#define I2C_ADDRESS 0x3C

#define ENCODER_MIN     INT16_MIN //-32767
#define ENCODER_MAX     INT16_MAX //32768
#define NUMBER_OF_AXIS  3         // Which means 3 axis

Gains gains[NUMBER_OF_AXIS];                    // For FFB
EffectParams effectparams[NUMBER_OF_AXIS];      // For FFB
int32_t forces[NUMBER_OF_AXIS]     = {0,0,0};   // Force value for servo motor
int32_t value[NUMBER_OF_AXIS]      = {0,0,0};   // Encorder value
int32_t valuemin[NUMBER_OF_AXIS]   = {460, 170, 60};  //Joystick HW limit
int32_t valuemax[NUMBER_OF_AXIS]   = {860, 570, 260}; //Joystick HW limit
int32_t valuecen[NUMBER_OF_AXIS]   = { (valuemax[0]+valuemin[0])/2,(valuemax[1]+valuemin[1])/2,(valuemax[2]+valuemin[2])/2};  //Jostick center
bool outofrange[NUMBER_OF_AXIS]    = {false,false,false}; //Joystick reached HW limit
int32_t forcemax[NUMBER_OF_AXIS]   = {20,20,20};  //FFB torque
int32_t forcelimit[NUMBER_OF_AXIS] = {50,50,50};  //Reverse torque if joystick is out of HW limit 
char analogpin[NUMBER_OF_AXIS]     = {A0,A1,A2};  // Encorder PIN

//Create LCD
SSD1306AsciiAvrI2c oled;

// create servo motor
BTS7960 motor[] = {
  BTS7960(BTS7960_ENA_1, BTS7960_LPWM_1, BTS7960_RPWM_1),
  BTS7960(BTS7960_ENA_2, BTS7960_LPWM_2, BTS7960_RPWM_2),
};
//Create joystick
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  0, 0,                   // No Button, No Hat Switch
  true, true, true,       // X, Y and Z Axis
  false, false, false,    //  No Rx, Ry, Rz
  false, false,           // No rudder or throttle
  false, false, false);   // No accelerator, brake, or steering

void setup() {
  Serial.begin(115200);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);

  pinMode(analogpin[0], INPUT_PULLUP);  //Set pin mode for Encorder X
  pinMode(analogpin[1], INPUT_PULLUP);  //Set pin mode for Encorder y
  pinMode(analogpin[2], INPUT_PULLUP);  //Set pin mode for Encorder z

  Joystick.setXAxisRange(ENCODER_MIN, ENCODER_MAX);
  Joystick.setYAxisRange(ENCODER_MIN, ENCODER_MAX);
  Joystick.setZAxisRange(ENCODER_MIN, ENCODER_MAX);
  Joystick.setGains(gains);
  Joystick.begin(true);

// These may not needed. But just in case.
  motor[0].Enable();
  motor[1].Enable();
// Needed for FFB to get game FFB data through USB
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
// Needed for FFB to get game FFB data through USB
ISR(TIMER3_COMPA_vect){
  Joystick.getUSBPID();
}

void loop() {
  for(int i=0; i<NUMBER_OF_AXIS;i++){
    int proc = 0;
    value[i] = analogRead(analogpin[i]);
    Serial.print("\tEnc" + String(i) + ": " + String(value[i]) );
    /*
    proc = constrain(value[i], valuemin[i], valuemax[i]);
    outofrange[i] = (value[i] != proc);
    */
    if( value[i] > valuemax[i] ){
      proc = ENCODER_MAX;
      outofrange[i] = true; 
    }else if( value[i] < valuemin[i] ){
      proc = ENCODER_MIN;
      outofrange[i] = true; 
    }else{
      proc = map( value[i],valuemin[i],valuemax[i],ENCODER_MIN,ENCODER_MAX );
      outofrange[i] = false;
    }    
    Serial.print( " (" + String(proc) + ")" ); 
    effectparams[i].springMaxPosition = ENCODER_MAX; 
    effectparams[i].springPosition = proc;
    switch (i) {
      case 0:
        Joystick.setXAxis(proc);
      case 1:
        Joystick.setYAxis(proc);
      case 2:
        Joystick.setZAxis(proc);
    }
  }
  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);

//Apply force to each motor.
  for(int i=0; i<NUMBER_OF_AXIS;i++){
    int myforce;
    if( outofrange[i] ){
      myforce = forcelimit[i];
      (value[i] > 0)? motor[i].TurnLeft( myforce ) : motor[i].TurnRight( myforce );
      Serial.print("\t[Limit:" + String(i) + "=" + String(value[i]) + "\tApply=" + String(myforce) + "] ");
    } else {
      myforce = map( abs(forces[i]),0,255,0,forcemax[i] );
      (forces[i] > 0)? motor[i].TurnLeft( myforce ) : motor[i].TurnRight( myforce );
      Serial.print("\t[Force:" + String(i) + "=" + String(forces[i]) + "\tApply=" + String(myforce) + "] ");
    }
  }
  Serial.println("");
}

