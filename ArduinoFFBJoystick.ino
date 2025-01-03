// Update board ID:%userprofile%\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6
#include "Joystick.h"           //FFB Joystick: https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
#include <digitalWriteFast.h>   //Needed for Joystick
#include "BTS7960.h"            // Motor Driver:    https://github.com/luisllamasbinaburo/Arduino-BTS7960
#define BTS7960_ENA_1   8
#define BTS7960_LPWM_1  9
#define BTS7960_RPWM_1  10
#define BTS7960_ENA_2   7
#define BTS7960_LPWM_2  6
#define BTS7960_RPWM_2  5
#define ENCODER_MAX_VALUE INT16_MAX
#define ENCODER_MIN_VALUE INT16_MIN
#define MAX_PWM 50

int32_t forces[2]={0};
Gains gains[2];
EffectParams effectparams[2];
int32_t g_force = 0;
int32_t valuex = 0;
int32_t valuey = 0;
int32_t valuez = 0;
int32_t valueminx = 760;
int32_t valuemaxx = 960;
int32_t valuecenx = (valuemaxx+valueminx)/2;
int32_t valueminy = 200;
int32_t valuemaxy = 400;
int32_t valueceny = (valuemaxy+valueminy)/2;
int32_t valueminz = 30;
int32_t valuemaxz = 280;
int32_t valuecenz = (valuemaxz+valueminz)/2;


// create motors Each pin is defined in ArduinoFFBJoystick.h
BTS7960 motor[] = {
  BTS7960(BTS7960_ENA_1, BTS7960_LPWM_1, BTS7960_RPWM_1),
  BTS7960(BTS7960_ENA_2, BTS7960_LPWM_2, BTS7960_RPWM_2),
};

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  0, 0,                  // Button Count, Hat Switch Count
  true, true, true,     // X and Y, but no Z Axis
  false, false, false,   //  Rx, no Ry, Rz
  false, false,          // No rudder or throttle
  false, false, false);    // No accelerator, brake, or steering

void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);

  //Joystick.setXAxisRange(valueminx-valuecenx, valuemaxx-valuecenx);
  Joystick.setXAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  Joystick.setYAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  Joystick.setZAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  Joystick.setGains(gains);
  Joystick.begin(true);

  motor[0].Enable();
  motor[1].Enable();
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

unsigned int interval = 0;
void loop() {
  valuex = analogRead(A0);
  if(valuex > valuemaxx) valuex = valuemaxx; 
  if(valuex < valueminx) valuex = valueminx; 
  int procx = map(valuex,valueminx,valuemaxx,ENCODER_MIN_VALUE,ENCODER_MAX_VALUE);
  Joystick.setXAxis(procx);
  valuey = analogRead(A1);
  if(valuey > valuemaxy) valuey = valuemaxy; 
  if(valuey < valueminy) valuey = valueminy; 
  int procy = map(valuey,valueminy,valuemaxy,ENCODER_MIN_VALUE,ENCODER_MAX_VALUE);
  Joystick.setYAxis(procy);
  valuez = analogRead(A2);
  if(valuez > valuemaxz) valuez = valuemaxz; 
  if(valuez < valueminz) valuez = valueminz; 
  int procz = map(valuez,valueminz,valuemaxz,ENCODER_MIN_VALUE,ENCODER_MAX_VALUE);
  Joystick.setZAxis(procz);
  Serial.print("EncX:");
  Serial.print(valuex);
  Serial.print("\tEncY:");
  Serial.print(valuey);
  Serial.print("\tEncZ:");
  Serial.print(valuez);

  effectparams[0].springMaxPosition = valuemaxx;
  effectparams[0].springPosition = valuex;
  effectparams[1].springMaxPosition = valuemaxy;
  effectparams[1].springPosition = valuey;
  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);

  for(int i=0; i<2;i++){
    (forces[i] > 0)? motor[i].TurnLeft( forces[i] ) : motor[i].TurnRight( -forces[i] );
    Serial.print("\t[Force:" + String(i) + "=" + String(forces[i]) + "] ");
  }
  Serial.println("");
}

