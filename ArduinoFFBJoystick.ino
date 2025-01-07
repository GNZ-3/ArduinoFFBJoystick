// Update board ID:%userprofile%\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6
// 起動時に各軸をキャリブレーション、GUIの追加
// X軸詳細化(タイミングプーリ購入）
// カルマンフィルタの追加
// センタスプリングの追加（ツール起動だけでOK?）
// パラメータチューニング（スプリング強度、フリクション、センター）用の切替スイッチと調整用ロータリーエンコーダの追加
#include <digitalWriteFast.h>
#include "Joystick.h"           //FFB Joystick: https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
#include "BTS7960.h"            // Motor Driver:    https://github.com/luisllamasbinaburo/Arduino-BTS7960
//https://github.com/greiman/SSD1306Ascii
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;

#define BTS7960_ENA_1   7       //X motor
#define BTS7960_LPWM_1  6       //CCW
#define BTS7960_RPWM_1  11      //Pin 5 (and 3) did not work. Try another pin 11
#define BTS7960_ENA_2   8       //Y motor
#define BTS7960_LPWM_2  9
#define BTS7960_RPWM_2  10
#define ENCODER_MAX_VALUE INT16_MAX //32768
#define ENCODER_MIN_VALUE INT16_MIN //-32767
#define MAX_FORCE 50

int32_t forces[2]={0};          // Force value for servo motor
Gains gains[2];                 // For FFB
EffectParams effectparams[2];   // For FFB
int32_t valuex = 0;             // Read from encorder X
int32_t valuey = 0;             // Read from encorder y
int32_t valuez = 0;             // Read from encorder z
int32_t valueminx = 340;        // Min HW limit for Encorder x
int32_t valuemaxx = 740;        // Max HW limit for encorder X
int32_t valuecenx = (valuemaxx+valueminx)/2; 
int32_t valueminy = 170;        // Min HW limit for Encorder y
int32_t valuemaxy = 570;        // Max HW limit for encorder y
int32_t valueceny = (valuemaxy+valueminy)/2;
int32_t valueminz = 60;        // Min HW limit for Encorder z
int32_t valuemaxz = 260;        // Max HW limit for encorder z
int32_t valuecenz = (valuemaxz+valueminz)/2;

// create motor
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
  pinMode(A0, INPUT_PULLUP);  //Set pin mode for Encorder X
  pinMode(A1, INPUT_PULLUP);  //Set pin mode for Encorder y
  pinMode(A2, INPUT_PULLUP);  //Set pin mode for Encorder z

  Joystick.setXAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  Joystick.setYAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  Joystick.setZAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  Joystick.setGains(gains);
  Joystick.begin(true);

// These may not needed. But just in case.
  motor[0].Enable();
  motor[1].Enable();
// PWM 
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

ISR(TIMER3_COMPA_vect){
  Joystick.getUSBPID();
}

void loop() {
  valuex = analogRead(A0);
  Serial.print("EncX:");
  Serial.print(valuex);
  if(valuex > valuemaxx) valuex = valuemaxx; 
  if(valuex < valueminx) valuex = valueminx; 
  int procx = map(valuex,valueminx,valuemaxx,ENCODER_MIN_VALUE,ENCODER_MAX_VALUE);
  Joystick.setXAxis(procx);
  valuey = analogRead(A1);
  Serial.print("\tEncY:");
  Serial.print(valuey);
  if(valuey > valuemaxy) valuey = valuemaxy; 
  if(valuey < valueminy) valuey = valueminy; 
  int procy = map(valuey,valueminy,valuemaxy,ENCODER_MIN_VALUE,ENCODER_MAX_VALUE);
  Joystick.setYAxis(procy);
  valuez = analogRead(A2);
  Serial.print("\tEncZ:");
  Serial.print(valuez);
  if(valuez > valuemaxz) valuez = valuemaxz; 
  if(valuez < valueminz) valuez = valueminz; 
  int procz = map(valuez,valueminz,valuemaxz,ENCODER_MIN_VALUE,ENCODER_MAX_VALUE);
  Joystick.setZAxis(procz);

  effectparams[0].springMaxPosition = valuemaxx-valueminx;
  effectparams[0].springPosition = valuex-valuecenx;
  effectparams[1].springMaxPosition = valuemaxy-valueminy;
  effectparams[1].springPosition = valuey-valueceny;

  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);

//Apply force to each motor.
  for(int i=0; i<2;i++){
    int myforce = abs(forces[i]);
    if(myforce > MAX_FORCE) myforce = MAX_FORCE;
    (forces[i] > 0)? motor[i].TurnLeft( myforce ) : motor[i].TurnRight( myforce );
    Serial.print("\t[Force:" + String(i) + "=" + String(forces[i]) + "] ");
  }
  Serial.println("");
}

