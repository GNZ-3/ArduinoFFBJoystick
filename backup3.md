// Update board ID:%userprofile%\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6
// 起動時に各軸をキャリブレーション、GUIの追加
// パラメータチューニング（スプリング強度、フリクション、センター）用の切替スイッチと調整用ロータリーエンコーダの追加
//C:\Users\LocalAdmin\Documents\Arduino\_Nomoreused\FFBPOC\build\SparkFun.avr.promicro\FFBPOC.ino.hex
#include <EEPROM.h>             // For save an axis data
#include <digitalWriteFast.h>   // For otary encorder switch: https://forum.arduino.cc/t/digitalwritefast-digitalreadfast-pinmodefast-etc/47037
#include "Joystick.h"           // FFB Joystick: https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
#include "BTS7960.h"            // Motor Driver: https://github.com/luisllamasbinaburo/Arduino-BTS7960
//#include "SSD1306Ascii.h"     // LCD Driver:   https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiAvrI2c.h" // LCD Driver:   https://github.com/greiman/SSD1306Ascii
#include <Button.h>             // Button class https://github.com/madleech/Button
// PIN definition
#define BTS7960_ENA_1   7     // Enable for X motor
#define BTS7960_LPWM_1  6     // Left PWM for X motor
#define BTS7960_RPWM_1 11     // Right PWM for X motor.Pin 5 and 3 did not work.
#define BTS7960_ENA_2   8     // Enable for Y motor
#define BTS7960_LPWM_2  9     // Left PWM for Y motor
#define BTS7960_RPWM_2 10     // Right PWM for Y motor
//#define CAP1188_RESET   16  // CAP1188 touch sensor reset pin
#define BUTTON1        14            //Joystick bnutton 1
#define BUTTON2        16     //Joystick bnutton 2
// #define I2C_ADD_CAP1188 0x29   // I2C address for CAP1199 touch sensor
#define I2C_ADD_SSD1306 0x3C  // I2C address for SSD1306 LCD
// Other define
#define ENCODER_MIN INT16_MIN // -32767
#define ENCODER_MAX INT16_MAX // 32768
#define NUMBER_OF_AXIS 3      // x, y and z axis

  enum Status {
    Center,
    TopLeft,
    BottomRight,
    ApplyForce,
    End,
    Aboart
  };


// Declare global valiable
Gains gains[NUMBER_OF_AXIS];                            // FFB gain
EffectParams effectparams[NUMBER_OF_AXIS];              // FFB effect parmeter
int32_t valuecen[NUMBER_OF_AXIS]   = { 0, 0, 0 };       //Jostick center value  492,350,168
int32_t forces[NUMBER_OF_AXIS]     = { 0, 0, 0 };       // Force value for servo motor
int32_t value[NUMBER_OF_AXIS]      = { 0, 0, 0 };       // Encorder value
int32_t valuemin[NUMBER_OF_AXIS]   = { 0, 0, 0 };       // {valuecen[0]-180, valuecen[1]-200, valuecen[2]-100};  //Joystick HW limit
int32_t valuemax[NUMBER_OF_AXIS]   = { 0, 0, 0 };       // {valuecen[0]+180, valuecen[1]+200, valuecen[2]+100}; //Joystick HW limit
int16_t outofrange[NUMBER_OF_AXIS] = { 0, 0, 0 };       //Joystick reached HW limit
int16_t forcemax[NUMBER_OF_AXIS]   = { 100, 100, 100 }; // FFB max torque
char    analogpin[NUMBER_OF_AXIS]  = { A0, A1, A2 };    // x, y  and Z encorder analog input PIN
char    axisname[NUMBER_OF_AXIS]   = { 'x', 'y', 'z' }; // Axis name to display in LCD

//Create LCD instance
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
//Create button
Button button1(BUTTON1);
Button button2(BUTTON2);

void setup() {
  pinMode(analogpin[0], INPUT_PULLUP);  //Set pin mode for X
  pinMode(analogpin[1], INPUT_PULLUP);  //Set pin mode for y
  pinMode(analogpin[2], INPUT_PULLUP);  //Set pin mode for z

  delay(3000);
  Serial.begin(115200);
  Serial.println("Serial init.");
  oled.begin(&Adafruit128x64, I2C_ADD_SSD1306);
  oled.clear();
  oled.setFont(Adafruit5x7);
  oled.set1X();
  oled.println("LCD init.");
  Serial.println("LCD init.");

  button1.begin();
	button2.begin();

  LoadCalibrationdata();
  Serial.println("Load data.");
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    Serial.print(axisname[i]);
    Serial.print(":");
    Serial.print(valuemin[i]);
    Serial.print("/");
    Serial.print(valuecen[i]);
    Serial.print("/");
    Serial.print(valuemax[i]);
    Serial.println("");
  }

  //LCDAnalogData();
  int timeout = millis() + 600000; //60 sec
  oled.setCursor(0,0);
  oled.println("\nCalib ?");
  while( true ){
    if( button2.pressed() ){
      Serial.println("Skip calib.");
      oled.println("\nSkip calib.");
      oled.clearToEOL();
      break;
    }
    if( button1.pressed() ){
      Serial.println("Go calib.");
      oled.println("\nGo calib.");
      oled.clearToEOL();
      calibration();
    }
    if(millis() > timeout){
      if(validcalibrationdata()){
        Serial.println("Timeout.");
        oled.println("\nTimeout.");
        oled.clearToEOL();
        break;
      }else{
        Serial.println("Repeat. Data is invalid.");
        oled.println("\nRepeat.");
        timeout = millis() + 600000;
      }
    }
    LoadAnalogData();
    LCDAnalogData(); // show axis data in LCD
    delay(10);
  }
  SaveCalibrationdata(); 
  delay(3000);


// Enable motors.
  motor[0].Enable();
  motor[1].Enable();

  Joystick.setXAxisRange(ENCODER_MIN, ENCODER_MAX);
  Joystick.setYAxisRange(ENCODER_MIN, ENCODER_MAX);
  Joystick.setZAxisRange(ENCODER_MIN, ENCODER_MAX);
  Joystick.setGains(gains);
  Joystick.begin(true);

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

void LoadAnalogData(){
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    value[i] = analogRead(analogpin[i]);
  }
}


bool validcalibrationdata(void) {
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    if (valuecen[i] == 0 || valuemin[i] == 0 || valuemax[i] == 0){
      Serial.print("Invalid Data cen/min/max is 0. Index:");
      Serial.println(i);
      return false;
    }
    if (valuecen[i] <= valuemin[i] || valuecen[i] >= valuemax[i]){
      Serial.print("Invalid Data Is not min<cen<max. Index:");
      Serial.println(i);
      return false;
    }
  }
  Serial.print("Data is OK.");
  return true;
}

// Load calibration date from EEPROM
void LoadCalibrationdata(){
  EEPROM.get(0,valuemin);
  EEPROM.get(sizeof(valuemin),valuecen);
  EEPROM.get(sizeof(valuemin)+sizeof(valuecen) ,valuemax);
}
// Update calibration date to EEPROM if changed
void SaveCalibrationdata(){
  EEPROM.put(0,valuemin);
  EEPROM.put(sizeof(valuemin),valuecen);
  EEPROM.put(sizeof(valuemin)+sizeof(valuecen) ,valuemax);
}

// Call from setup() to set joystick center and limit
// We are here because button 1 press then released.
// Press button1 to set center. button2 to clear.
// Press button1 to set topleft.  button2 to clear.
// Press button1 to set bottomright. button2 to clear.
// limit and spring center start
// Press button1 to set and end. button2 to clear.

void LCDAnalogData(){
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    oled.print(axisname[i]);
    oled.print(":");
    oled.print(value[i]);
    oled.print("/");
    oled.print(valuecen[i]);
    oled.clearToEOL();
  }
}
void FillArray(int32_t src[],int32_t dst[], int n){
    for(int i = 0; i < n; i++){ dst[i] = src[i]; }
}

void calibration() {
// Disable while in calibration
  motor[0].Disable();
  motor[1].Disable();
  Serial.println("Calib start.");
  oled.clear();
  oled.println("CALIBRATION");
  int state=Status::Center;
  while( state < Status::End ){
    oled.setCursor(0,1);
    switch(state){
      case Status::Center:
        oled.println("Center");
        oled.clearToEOL();
        if(button1.pressed()){
          state=Status::TopLeft;
          FillArray( value, valuecen,NUMBER_OF_AXIS);
          Serial.println("Calib center done.");
          delay(1000);
        }
        break;
      case Status::TopLeft:
        oled.println("Top Left");
        oled.clearToEOL();
        if(button1.pressed()){
          state=Status::BottomRight;
          FillArray( value, valuemin,NUMBER_OF_AXIS);
          Serial.println("Calibration topleft done.");
          delay(1000);
        }
        if(button2.pressed()){
          state=Status::Center;
          Serial.println("Back to calib center.");
          delay(1000);
        }
        break;
      case Status::BottomRight:
        oled.println("Bot Right");
        oled.clearToEOL();
        if(button1.pressed()){
          state=Status::ApplyForce;
          // Enable motors..
          motor[0].Enable();
          motor[1].Enable();
          FillArray( value, valuemax,NUMBER_OF_AXIS);
          Serial.println("Calibration bottomright done.");
          delay(1000);
        }
        if(button2.pressed()){
          state=Status::TopLeft;
          Serial.println("Go back to calib topleft.");
          delay(1000);
        }
        break;
      case Status::ApplyForce:
        oled.println("Force is ON. ");
        oled.clearToEOL();
        if(button1.pressed()){
          state=Status::End;
          Serial.println("Calib. Done.");
          delay(1000);
        }
        if(button2.pressed()){
          state=Status::BottomRight;
          motor[0].Disable();
          motor[1].Disable();
          Serial.println("Calibration go back to bottomright.");
          delay(1000);
        }
        /*

        //Apply spring force
        for(int i=0; i<NUMBER_OF_AXIS;i++){
          int myforce = 0;
          outofrange[i] = 0;
          if( value[i] > valuemax[i] ){
            outofrange[i] = value[i] - valuemax[i];
            myforce = forcemax[i] + pow(outofrange[i],2); 
          }
          if( value[i] < valuemin[i]  ){
            outofrange[i] = valuemin[i] - value[i];
            myforce = forcemax[i] + pow(outofrange[i],2); 
          }
          if(outofrange[i] == 0 ){
            myforce = map( abs(valuecen[i]-value[i]),0,255,0,forcemax[i] );
          }
          if(myforce > 255) myforce=255;
          (value[i] - valuecen[i] > 0)? motor[i].TurnRight( myforce ) : motor[i].TurnLeft( myforce );
        }
        */
        break;
    }
    LoadAnalogData();
    LCDAnalogData();
    delay(50);
  }
  Serial.println("Calibration completed.");
}

void loop() {
 //while(true){}
  LoadAnalogData();
  for(int i=0; i<NUMBER_OF_AXIS;i++){
    int proc = 0;
    Serial.print("\tEnc" );
    Serial.print(i);
    Serial.print(": " );
    Serial.print(value[i] );
    outofrange[i] = 0;
    if( value[i] > valuemax[i] ){
      proc = ENCODER_MAX;
      outofrange[i] = value[i] - valuemax[i]; 
    }
    if( value[i] < valuemin[i]  ){
      proc = ENCODER_MIN;
      outofrange[i] = valuemin[i] - value[i];
    }
    if(outofrange[i] == 0 ){
      proc = map( value[i],valuemin[i],valuemax[i],ENCODER_MIN,ENCODER_MAX );
    }
    Serial.print( "\t");
    Serial.print( proc );
    switch (i) {
      case 0:
        effectparams[i].springMaxPosition = ENCODER_MAX; 
        effectparams[i].springPosition = proc;
        Joystick.setXAxis(proc);
        break;  //Removing this cause some unexpectedvaiable change. Someimtes Leonald won't boot any more.
      case 1:
        effectparams[i].springMaxPosition = ENCODER_MAX; 
        effectparams[i].springPosition = proc;
        Joystick.setYAxis(proc);
        break;
      case 2:
        Joystick.setZAxis(proc);
        break;
    }
  }
  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);

//Apply force to each motor.
  for(int i=0; i<NUMBER_OF_AXIS-1;i++){
    int32_t myforce;
    if( outofrange[i] > 0 ){
      //myforce = (outofrange[i]>255)?255:outofrange[i];
      myforce = forcemax[i] + pow(outofrange[i],2); 
      if(myforce > 255) myforce=255;
      (value[i] - valuecen[i] > 0)? motor[i].TurnRight( myforce ) : motor[i].TurnLeft( myforce );
      Serial.print("\t[Limit:");
      Serial.print(i);
      Serial.print("=");
      Serial.print(value[i]);
      Serial.print("\tApply=");
      Serial.print(myforce);
      Serial.print("]");
      //Serial.print("\t[Limit:" + String(i) + "=" + String(value[i]) + "\tApply=" + String(myforce) + "] ");
    } else {
      myforce = map( abs(forces[i]),0,255,0,forcemax[i] );
      (forces[i] > 0)? motor[i].TurnLeft( myforce ) : motor[i].TurnRight( myforce );
      Serial.print("\t[Force:");
      Serial.print(i);
      Serial.print("=");
      Serial.print(forces[i]);
      Serial.print("\tApply=");
      Serial.print(myforce);
      Serial.print("] ");
//      Serial.print("\t[Force:" + String(i) + "=" + String(forces[i]) + "\tApply=" + String(myforce) + "] ");
    }
  }
  Serial.println("");
}