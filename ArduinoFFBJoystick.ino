// Update board ID:%userprofile%\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6
// ゲーム内でZ軸とX軸を入れ替えてZ軸のFFBデータ有無を確認->J2M試製雷電の離陸でラダー軸になったこをと確認してFFBがあることを確認
// Z軸のFFBが有る場合、レポートとFFB受信データの問題と思われる
// 無い場合はエルロン軸と旋回計のボールから算出する（不要）

//update test message
#include <digitalWriteFast.h>
#include <EEPROM.h>                     // For save an axis data
#include <Wire.h>                       // For i2c communication
#include "Joystick.h"                   // FFB Joystick: https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
//#include "SSD1306Ascii.h"             // LCD Driver:   https://github.com/greiman/SSD1306Ascii
//#include "SSD1306AsciiAvrI2c.h"       // Same as above
//#include "SSD1306Ascii.h"             // LCD Driver:   https://github.com/greiman/SSD1306Ascii
//#include "SSD1306AsciiAvrI2c.h"       // LCD Driver:   https://github.com/greiman/SSD1306Ascii
#include <Button.h>                     // Button class https://github.com/madleech/Button
#define PIN_BTN1        2               //Joystick bnutton 1
#define PIN_BTN2        3               //Joystick bnutton 2
#define PIN_LED         13
#define I2C_ADD_SSD1306 0x3C            // I2C address for SSD1306 LCD
#define I2C_ADD_SLAVE   0x8             // I2C address for Slave mega2560
#define I2C_ADD_MASTER  0x9             // I2C address for master leonald
#define JOYSTICK_AXIS_MIN     INT16_MIN // -32767
#define JOYSTICK_AXIS_MAX     INT16_MAX // 32768
//#define ENCORDER_MAX          1023      // 10 bit (0-1023) each encoder has own min/max value
//#define ENCORDER_HALF  ENCORDER_MAX/2

#define FORCE_MIN       -250            //
#define FORCE_MAX       250             //
#define TOTALAXIS       3               // 3=x,y and z. set 2 if you do not have Z axis
#define LEFT            0               // Motor direction on BTS7960 board
#define RIGHT           1               // Motor direction on BTS7960 board

struct Data{                            // I2C packet structure between master and slave.
  byte cmd;                             // Not using
  byte axis;                            // 0=X, 1=Y, 2=Z
  byte dir;                             // LEFT=0 or RIGHT=1 
  uint8_t force;                        // abs value of force. So the range is 0-FORCE_MAX (not FORCE_MIN-FORCE_MAX)
};



Gains   gains[TOTALAXIS];                         // For FFB
EffectParams effectparams[TOTALAXIS];             // For FFB
int32_t forces[TOTALAXIS]   = {0,0,0};            // For FFB
int16_t value[TOTALAXIS]    = {0,0,0};            // Encorder value
int16_t valueproc[TOTALAXIS]= {0,0,0};            // Joystick axis value clipped by valuemin/valuemax
int16_t encoffset[TOTALAXIS]= {0,0,0};            // Encorder offset from center (ENCORDER_MAX/2).
int16_t encMax[TOTALAXIS]   = {1023,1020,1023};   // Actual max value of your encorder
int16_t encMin[TOTALAXIS]   = {19,17,19};         // Actual min value of your encorder
int16_t moverange[TOTALAXIS] = {400,200,200};     // how much encorder value from center.
uint8_t motorclp[TOTALAXIS] = {96,127,20};        // Max motor force while in range
uint8_t motormax[TOTALAXIS] = {127,255,50};       // Max mortor force that can apply to motor 
char    analogpin[TOTALAXIS]= {A0,A1,A2};         // Encorder PIN
char    axisname[TOTALAXIS] = { 'X', 'Y', 'Z' };  // Axis name to display in LCD
Data    sendData;                                 // Data packet between master and slave.
bool    FFBisWorking;

//Create LCD instance
//  SSD1306AsciiAvrI2c oled;
// create servo motor
//Create button
Button button1(PIN_BTN1);
Button button2(PIN_BTN2);
//Create joystick
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  0, 0,                   // No Button, No Hat Switch
  true, true, true,       // X, Y and Z Axis
  false, false, false,    //  No Rx, Ry, Rz
  false, false,           // No rudder or throttle
  false, false, false);   // No accelerator, brake, or steering

void setup() {
  Serial.begin(115200);
  //while (!Serial); // Leonardo: wait for serial monitor
  pinMode(analogpin[0], INPUT_PULLUP);  //Set pin mode for Encorder X
  pinMode(analogpin[1], INPUT_PULLUP);  //Set pin mode for Encorder y
  pinMode(analogpin[2], INPUT_PULLUP);  //Set pin mode for Encorder z
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED,LOW);   // Turn off onboard LED
  button1.begin();
	button2.begin();
  //LoadCalibrationdata();
  unsigned long starttime = millis();
  int count = 0;
  Serial.print("Wait user input. Timeout is 20sec.");
  while( count <5 ){
    if( button2.pressed() ){
      if( !validcalibrationdata() ){
        Serial.println("Skip calib. Centering joystick.");
        
        break;
      }else{
        Serial.println("Can not skip calib.");
        break;
      }
    }
    if( button1.pressed() ){
      Serial.println("Go calib.");
      calibration();
      break;
    }
    unsigned long nowtime = millis(); //+ 2000000; //20 sec
    if( ((nowtime - starttime) / 1000) > count ){
      count++;
      Serial.print(".");
    }
    delay(10);
  }
  if( !validcalibrationdata() ){
    calibration();
  }
  SaveCalibrationdata();
  Joystick.setXAxisRange(JOYSTICK_AXIS_MIN, JOYSTICK_AXIS_MAX);
  Joystick.setYAxisRange(JOYSTICK_AXIS_MIN, JOYSTICK_AXIS_MAX);
  Joystick.setZAxisRange(JOYSTICK_AXIS_MIN, JOYSTICK_AXIS_MAX);
  Joystick.setGains(gains);
  Joystick.begin(true);
  Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
  FFBisWorking = true;
  Wire.begin();

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
ISR(TIMER3_COMPA_vect) {
  Joystick.getUSBPID();
}

void loop() {
  for(uint8_t i=0; i<TOTALAXIS;i++){
    valueproc[i] = value[i] = myAnalogRead(i);
    Serial.print("\tEnc" + (String)axisname[i] + "\t" );
    Serial.print( value[i] );
    if(value[i] > moverange[i]){
      valueproc[i] = moverange[i];
    }else if(value[i] < -moverange[i]){
      valueproc[i] = -moverange[i];
    }
    int16_t proc = map( valueproc[i], -moverange[i], moverange[i],JOYSTICK_AXIS_MIN,JOYSTICK_AXIS_MAX ); //valueproc is int16_t
    Serial.print( "\t" ); 
    Serial.print( proc ); 
    effectparams[i].springMaxPosition = JOYSTICK_AXIS_MAX; 
    effectparams[i].springPosition = proc ;
    switch (i) {
      case 0:
        Joystick.setXAxis(proc);                // void Joystick_::setXAxis(int16_t value)
        break;  //Removing this cause some unexpectedvaiable change. Someimtes Leonald won't boot any more.
      case 1:
        Joystick.setYAxis(proc);
        break;
      case 2:
        Joystick.setZAxis(proc);
        break;
    }
  }
  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);

//Send force data to Slave Mega2560.
  if (Wire.getWireTimeoutFlag()){
    Wire.clearWireTimeoutFlag();
    Serial.print("Wire timeout detected. Disable FFB");
    digitalWrite(PIN_LED,HIGH);   // Turn on onboard LED
    FFBisWorking = false;
  }
  if(FFBisWorking){
    for(uint8_t i=0; i<TOTALAXIS;i++){
      int32_t motorforce;
      sendData.cmd = "m";
      sendData.axis = i;
      if(value[i] != valueproc[i]){
        // if sensor read value is different than process value, joystick posotion is exceeded HW limit. Need strong reverse force. 
        int force = 0.5*pow( value[i] - valueproc[i],2); //Generate force from clip & exceeded.
        if(forces[i] != 0) force += motorclp[i];
        sendData.force =(uint8_t) constrain( force,0,motormax[i] );
        sendData.dir = (valueproc[i] > 0 )? RIGHT:LEFT; 
        Serial.print("\tLIMIT:" +  (String)axisname[i] + "\t" );
        Serial.print( sendData.force );
      }else{
        // Joystick position is in range. Apply regular FFB force
        sendData.force = (uint8_t) map( abs(forces[i]) ,0,FORCE_MAX,0,motorclp[i] ); //FFB force range is -255 to 255 which converted to clip value 
        sendData.dir = (forces[i] > 0)? LEFT:RIGHT;
        Serial.print("\tforce" +  (String)axisname[i] + "\t" );
        Serial.print( sendData.force );
      }
      sendEvent( &sendData );
    }
  }else{
    Serial.print( "\tFFB is disabled." );
  }
  Serial.println("");
}

void sendEvent( Data *sendData){
  Wire.beginTransmission(I2C_ADD_SLAVE);
  Wire.write( (byte *)sendData, sizeof( Data));
  int state = Wire.endTransmission();
  Serial.print("\tSend:" +  (String)axisname[sendData->axis] + "\t" );
  Serial.print( state );
}

void calibration() {
  for (int i = 0; i < TOTALAXIS; i++) {
    encoffset[i] =0;
    encoffset[i] =  analogRead(i) ;   // 0-1023 -> 511
    Serial.print("\tOffset");
    Serial.print(i);
    Serial.print("\t");
    Serial.print(encoffset[i]);
  }
}

int myAnalogRead( int i) {
  int readval =constrain( analogRead( analogpin[i] ),encMin[i],encMax[i] );
  int x = readval - encoffset[i];
  if( x > (encMax[i]-encMin[i])/2 ){
    x = x - encMax[i] - 1 + encMin[i];
  }else if(x < -(encMax[i]-encMin[i])/2){
    x = x + encMax[i] + 1 - encMin[i];
  }
  return x;
}

bool validcalibrationdata(void) {
  for (int i = 0; i < TOTALAXIS; i++) {
    if (encoffset[i] == 0 ){
      Serial.print("Invalid Data at index:");
      Serial.println(encoffset[i]);
      return false;
    }
  }
  Serial.print("Data is OK.");
  return true;
}

// Load calibration date from EEPROM
void LoadCalibrationdata(){
    EEPROM.get(0,encoffset);
}
// Update calibration date to EEPROM if changed
void SaveCalibrationdata(){
    EEPROM.put(0,encoffset);
}