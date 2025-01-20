// Update board ID:%userprofile%\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6
// 起動時に各軸をキャリブレーション、GUIの追加
// パラメータチューニング（スプリング強度、フリクション、センター）用の切替スイッチと調整用ロータリーエンコーダの追加
#include <EEPROM.h>            // For save an axis data
#include <digitalWriteFast.h>  // For otary encorder switch: https://forum.arduino.cc/t/digitalwritefast-digitalreadfast-pinmodefast-etc/47037
#include "Joystick.h"          // FFB Joystick: https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
#include "BTS7960.h"           // Motor Driver: https://github.com/luisllamasbinaburo/Arduino-BTS7960
//#include "SSD1306Ascii.h"       // LCD Driver:   https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiAvrI2c.h"  // LCD Driver:   https://github.com/greiman/SSD1306Ascii
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

// Declare global valiable
Gains gains[NUMBER_OF_AXIS];                            // FFB gain
EffectParams effectparams[NUMBER_OF_AXIS];              // FFB effect parmeter
int32_t valuecen[NUMBER_OF_AXIS]   = { 0, 0, 0 };       //Jostick center value  492,350,168
int32_t forces[NUMBER_OF_AXIS]     = { 0, 0, 0 };       // Force value for servo motor
int32_t value[NUMBER_OF_AXIS]      = { 0, 0, 0 };       // Encorder value
int32_t valuemin[NUMBER_OF_AXIS]   = { 0, 0, 0 };       // {valuecen[0]-180, valuecen[1]-200, valuecen[2]-100};  //Joystick HW limit
int32_t valuemax[NUMBER_OF_AXIS]   = { 0, 0, 0 };       // {valuecen[0]+180, valuecen[1]+200, valuecen[2]+100}; //Joystick HW limit
int16_t outofrange[NUMBER_OF_AXIS] = {0,0,0};           //Joystick reached HW limit
int32_t forcemax[NUMBER_OF_AXIS]   = { 100, 100, 100 }; // FFB max torque
char    analogpin[NUMBER_OF_AXIS]  = { A0, A1, A2 };    // x, y  and Z encorder analog input PIN
char    axisname[NUMBER_OF_AXIS]   = { 'x', 'y', 'z' }; // Axis name to display in LCD
bool    button1                    = LOW;               // Button 1 is not press.
bool    button2                    = LOW;               // Button 2 is not press.
long    button1lastchange;
long    button2lastchange;
long    cooldelay                  = 50;                // 50ms button chattaring period
//Create LCD instance
SSD1306AsciiAvrI2c oled;

// create servo motor instance
BTS7960 motor[] = {
  BTS7960(BTS7960_ENA_1, BTS7960_LPWM_1, BTS7960_RPWM_1),
  BTS7960(BTS7960_ENA_2, BTS7960_LPWM_2, BTS7960_RPWM_2)
};

//Create joystick instance
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK,
                   2, 0,                  // 2 Button, No Hat Switch
                   true, true, true,      // X, Y and Z Axis
                   false, false, false,   //  No Rx, Ry, Rz
                   false, false,          // No rudder or throttle
                   false, false, false);  // No accelerator, brake, or steering

void setup() {
  delay(1000);
  Serial.begin(115200);
  oled.begin(&Adafruit128x64, I2C_ADD_SSD1306);
  oled.clear();
  oled.setFont(Adafruit5x7);
  oled.set1X();
  oled.println("LCD init.");
  Serial.println("LCD Initialized.");

  pinMode(analogpin[0], INPUT_PULLUP);  //Set pin mode for X
  pinMode(analogpin[1], INPUT_PULLUP);  //Set pin mode for y
  pinMode(analogpin[2], INPUT_PULLUP);  //Set pin mode for z
  pinMode(BUTTON1, INPUT_PULLUP);       //Set pinmode for button 1
  pinMode(BUTTON2, INPUT_PULLUP);       //Set pinmode for button 2

  motor[0].Enable();  // just in case.
  motor[1].Enable();  // just in case.

  Serial.println("Calibration process started.");
  LoadCalibrationdata();
  oled.println("\nCalibration ?");
  int timeout = millis() + 100000; //10 sec
  clearbuttonstate();
  while( timeout < millis() ){
    updatebuttonstate();
    if( button1==LOW || button2==LOW ){
      clearCalibrationData();
      break;
    }
  }
  while( ! validcalibrationdata() ){
    calibration();
  }
  oled.println("Calib completed.");
  SaveCalibrationdata();

// Set joystick parmeter and bigin
  Joystick.setXAxisRange(ENCODER_MIN, ENCODER_MAX);
  Joystick.setYAxisRange(ENCODER_MIN, ENCODER_MAX);
  Joystick.setZAxisRange(ENCODER_MIN, ENCODER_MAX);
  Joystick.setGains(gains);
  Joystick.begin(true);

  oled.println("Init complete.");
  Serial.println("Init complete.");
  delay(3000);
  oled.clear();

  // Get game FFB data from USB
  cli();
  TCCR3A = 0;  //set TCCR1A 0
  TCCR3B = 0;  //set TCCR1B 0
  TCNT3 = 0;   //counter init
  OCR3A = 399;
  TCCR3B |= (1 << WGM32);  //open CTC mode
  TCCR3B |= (1 << CS31);   //set CS11 1(8-fold Prescaler)
  TIMSK3 |= (1 << OCIE3A);
  sei();
}
// Needed for FFB to get game FFB data through USB
ISR(TIMER3_COMPA_vect) {
  Joystick.getUSBPID();
}

void updatebuttonstate(void){
  if(millis() > button1lastchange + cooldelay ){
    int buttonread = digitalRead(BUTTON1);
    if( buttonread != button1){
      button1 = buttonread;
      button1lastchange = millis();
    }
  }
  if(millis() > button2lastchange + cooldelay ){
    int buttonread = digitalRead(BUTTON2);
    if( buttonread != button2){
      button2 = buttonread;
      button2lastchange = millis();
    }
  }
}

void clearbuttonstate(void){
  while( digitalRead(BUTTON1)==LOW || digitalRead(BUTTON2)==LOW ){
    delay(cooldelay);
  }
  button1=HIGH;
  button2=HIGH;
}

bool validcalibrationdata(void) {
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    if (valuecen[i] == 0){
      Serial.println("Validation failed. Center value is 0.");
      return false;
    }
    if (valuemin[i] == 0){
      Serial.println("Validation failed. Min value is 0.");
      return false;
    }
    if (valuemax[i] == 0){
      Serial.println("Validation failed. Max value is 0.");
      return false;
    }
    if (valuecen[i] <= valuemin[i]){
      Serial.println("Validation failed. Center below than Min.");
      return false;
    }
    if (valuecen[i] >= valuemax[i]){
      Serial.println("Validation failed. Center grator than Max.");
      return false;
    }
  }
  return true;
}
void clearCalibrationData(void) {
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    valuecen[i] = 0;
    valuemin[i] = 0;
    valuemax[i] = 0;
  }
}

// Load calibration date from EEPROM
void LoadCalibrationdata(){
}
// Update calibration date to EEPROM if changed
void SaveCalibrationdata(){
}

//Call from setup() to set joystick parms
void calibration() {
  Serial.println("calibration process start.");
//  int readval[NUMBER_OF_AXIS];
  oled.clear();
  oled.println("Center joystick. ");
  Serial.println("Calibration center started.");
  clearbuttonstate();
  while( button1==HIGH && button2==HIGH ){
    oled.setCol(0);
    oled.clearToEOL();
    for (int i = 0; i < NUMBER_OF_AXIS; i++) {
      valuecen[i] = analogRead(analogpin[i]);
      oled.print(valuecen[i]);
      oled.print(" / ");
    }
    updatebuttonstate();
/*    if( button1 || button2 ){
      for (int i = 0; i < NUMBER_OF_AXIS; i++) {
        valuecen[i] = readval[i];
      }
      break;
    }*/
  }
  Serial.println("Calibration center completed.");
  oled.clear();
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    valuemin[i] = valuecen[i]-150;
    valuemax[i] = valuecen[i]+150;
    oled.print(axisname[i]);
    oled.print(": ");
    oled.print(valuemin[i]);
    oled.print(" / ");
    oled.print(valuecen[i]);
    oled.print(" / ");
    oled.print(valuemax[i]);
    oled.print(" / ");
  }

}

void loop() {
  oled.home();
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    int proc = 0;
    value[i] = analogRead(analogpin[i]);
    Serial.print("\tEnc" + String(i) + ":");
    //Serial.print(value[i]);
    oled.print(axisname[i]);
    oled.print(":");
    oled.println(value[i]);
    if (value[i] > valuemax[i]) {
      proc = ENCODER_MAX;
      outofrange[i] = true;
    } else if (value[i] < valuemin[i]) {
      proc = ENCODER_MIN;
      outofrange[i] = true;
    } else {
      proc = map(value[i], valuemin[i], valuemax[i], ENCODER_MIN, ENCODER_MAX);
      outofrange[i] = false;
    }
    Serial.print("\t" + String(proc) + "");
    effectparams[i].springMaxPosition = ENCODER_MAX;
    effectparams[i].springPosition = proc;
    switch (i) {
      case 0:
        Joystick.setXAxis(proc);
        break;
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

  //Apply force to each motor.
  oled.home();
  for(int i=0; i<NUMBER_OF_AXIS;i++){
    int16_t myforce;
    if( outofrange[i] ){
      myforce = (outofrange[i]>255)?255:outofrange[i];
      (value[i] - valuecen[i] > 0)? motor[i].TurnRight( myforce ) : motor[i].TurnLeft( myforce );
      Serial.print("\t[Limit:" + String(i) + "=" + String(value[i]) + "\tApply=" + String(myforce) + "] ");
    } else {
      myforce = map( abs(forces[i]),0,255,0,forcemax[i] );
      (forces[i] > 0)? motor[i].TurnLeft( myforce ) : motor[i].TurnRight( myforce );
      Serial.print("\t[Force:" + String(i) + "=" + String(forces[i]) + "\tApply=" + String(myforce) + "] ");
    }
    oled.setCol(65);
    oled.print(axisname[i]);
    oled.print("F:");
    oled.println(myforce);
  }
  Serial.println("");
}
