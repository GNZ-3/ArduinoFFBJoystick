// Update board ID:%userprofile%\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6
// 起動時に各軸をキャリブレーション、GUIの追加
// パラメータチューニング（スプリング強度、フリクション、センター）用の切替スイッチと調整用ロータリーエンコーダの追加
#include <EEPROM.h>            // For save an axis data
#include <digitalWriteFast.h>  // For otary encorder switch: https://forum.arduino.cc/t/digitalwritefast-digitalreadfast-pinmodefast-etc/47037
#include "Joystick.h"          // FFB Joystick: https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.git
#include "BTS7960.h"           // Motor Driver: https://github.com/luisllamasbinaburo/Arduino-BTS7960
//#include "SSD1306Ascii.h"       // LCD Driver:   https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiAvrI2c.h"  // LCD Driver:   https://github.com/greiman/SSD1306Ascii
// #include <Adafruit_CAP1188.h>   // CAP1188 key: https://github.com/adafruit/Adafruit_CAP1188_Library/tree/master
// PIN definition
#define BTS7960_ENA_1 7    // Enable for X motor
#define BTS7960_LPWM_1 6   // Left PWM for X motor
#define BTS7960_RPWM_1 11  // Right PWM for X motor.Pin 5 and 3 did not work.
#define BTS7960_ENA_2 8    // Enable for Y motor
#define BTS7960_LPWM_2 9   // Left PWM for Y motor
#define BTS7960_RPWM_2 10  // Right PWM for Y motor
//#define CAP1188_RESET   16      // CAP1188 touch sensor reset pin
#define BUTTON1 14  //Joystick bnutton 1
#define BUTTON2 16  //Joystick bnutton 2
// #define I2C_ADD_CAP1188 0x29    // I2C address for CAP1199 touch sensor
#define I2C_ADD_SSD1306 0x3C  // I2C address for SSD1306 LCD
// Other define
#define ENCODER_MIN INT16_MIN  // -32767
#define ENCODER_MAX INT16_MAX  // 32768
#define NUMBER_OF_AXIS 3       // x, y and z axis

// Declare global valiable
Gains gains[NUMBER_OF_AXIS];                                // FFB gain
EffectParams effectparams[NUMBER_OF_AXIS];                  // FFB effect parmeter
int32_t valuecen[NUMBER_OF_AXIS] = { 0, 0, 0 };             //Jostick center value  492,350,168
int32_t forces[NUMBER_OF_AXIS] = { 0, 0, 0 };               // Force value for servo motor
int32_t value[NUMBER_OF_AXIS] = { 0, 0, 0 };                // Encorder value
int32_t valuemin[NUMBER_OF_AXIS] = { 0, 0, 0 };             // {valuecen[0]-180, valuecen[1]-200, valuecen[2]-100};  //Joystick HW limit
int32_t valuemax[NUMBER_OF_AXIS] = { 0, 0, 0 };             // {valuecen[0]+180, valuecen[1]+200, valuecen[2]+100}; //Joystick HW limit
bool outofrange[NUMBER_OF_AXIS] = { false, false, false };  //true if joystick axis reached HW limit
int32_t forcemax[NUMBER_OF_AXIS] = { 100, 100, 100 };       // FFB max torque
int32_t forcelimit[NUMBER_OF_AXIS] = { 127, 127, 127 };     // Reverse torque if joystick axis is out of HW limit
char analogpin[NUMBER_OF_AXIS] = { A0, A1, A2 };            // x, y  and Z encorder analog input PIN
char axisname[NUMBER_OF_AXIS] = { 'x', 'y', 'z' };          // Axis name to display in LCD
bool button1;
bool button2;

//Create LCD instance
SSD1306AsciiAvrI2c oled;

// Create touch key instance
// Adafruit_CAP1188 cap = Adafruit_CAP1188(CAP1188_RESET);

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
  delay(500);
  Serial.begin(115200);
  oled.begin(&Adafruit128x64, I2C_ADD_SSD1306);
  oled.clear();
  oled.setFont(Adafruit5x7);
  oled.set1X();
  oled.println("LCD init.");
  Serial.println("LCD Initialized.");

  /*  Serial.print("Touch initializing.");
  oled.print("Touch init.");
  for(int i=0;i<5;i++){
    oled.print(i);
    if (!cap.begin(I2C_ADD_CAP1188)){
      oled.print(".");
      Serial.print(".");
      delay(500);
      continue;
    }
    oled.println("FAILED!!");
    Serial.println("FAILED!!");
    delay(5000);
  }
*/

  pinMode(analogpin[0], INPUT_PULLUP);  //Set pin mode for X
  pinMode(analogpin[1], INPUT_PULLUP);  //Set pin mode for y
  pinMode(analogpin[2], INPUT_PULLUP);  //Set pin mode for z
  pinMode(BUTTON1, INPUT_PULLUP);       //Set pinmode for button 1
  pinMode(BUTTON2, INPUT_PULLUP);       //Set pinmode for button 2

  motor[0].Enable();  // just in case.
  motor[1].Enable();  // just in case.

  Serial.println("Calibration process started.");
  oled.println("\nCalibration ?");
  int timeout = milles() + 100000; //10 sec
  clearbuttonstate();
  while( timeout < milles() )
    updatebuttonstate();
    if( button1 || button2 ){
      clearbuttonstate();
      calibration();
      oled.println("Done Calib.");
      break;
    }
  }else{
    oled.println("Skip Calib.");
  }
  delay(3000);

  Joystick.setXAxisRange(ENCODER_MIN, ENCODER_MAX);
  Joystick.setYAxisRange(ENCODER_MIN, ENCODER_MAX);
  Joystick.setZAxisRange(ENCODER_MIN, ENCODER_MAX);
  Joystick.setGains(gains);
  Joystick.begin(true);
  oled.println("Initilize complete.");
  Serial.println("Init complete.");
  delay(1000);
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

void updatebuttonstate(void)
{
  if( digitalRead(BUTTON1) == LOW){
    button1=true;
    break;
  }
  if( digitalRead(BUTTON2) == LOW){
    button2=true;
    break;
  }
}
void clearbuttonstate(void)
{
    button1=false;
    button2=false;
}

void BtnRelease(int timeout)
  long starttime = millis();
  while( millis() < starttime + timeout && digitalRead(BUTTON1) != HIGH && digitalRead(BUTTON2) != HIGH){
    delay(10);
  }
}

bool validrange(void) {
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    if (valuecen[i] == 0) return false;
    if (valuemin[i] == 0) return false;
    if (valuemax[i] == 0) return false;
    if (valuecen[i] < valuemin[i]) return false;
    if (valuecen[i] > valuemax[i]) return false;
  }
  return true;
}
//Call from setup() to set joystick parms
void calibration() {
 
  Serial.println("LCD cleanred.");
  bool istouched;
  int readval[NUMBER_OF_AXIS];
  oled.println("Center joystick. ");
  Serial.println("Calibration center started.");
    for (int i = 0; i < NUMBER_OF_AXIS; i++) {
      readval[i] = analogRead(analogpin[i]);
      oled.print(readval[i]);
      oled.print(" / ");
    }
//if(getBtnPress() == BUTTON1) return;

  istouched = false;
  while( digitalRead(BUTTON1) == HIGH && digitalRead(BUTTON2) == HIGH) {
     oled.clear();
      if (cap.touched() && 4 != 0) istouched = true;
    //
  }
    Serial.println("Calibration center completed.");
    delay(10000);

  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    valuecen[i] = readval[i];
    Serial.print("\t center");
    Serial.print(i);
    Serial.print("=");
    Serial.print(valuecen[i]);
  }
  delay(1000);

  oled.println("TopLeft joystick. ");
  istouched = false;
  while (!istouched) {
    oled.setRow(0);
    for (int i = 0; i < NUMBER_OF_AXIS; i++) {
      readval[i] = analogRead(analogpin[i]);
      oled.print(readval[i]);
      oled.print(" / ");
    }
//    if (cap.touched() && 4 != 0) istouched = true;
    delay(100);
  }
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    valuemin[i] = readval[i];
  }
  delay(1000);

  oled.println("Bottomright joystick. ");
  istouched = false;
  while (!istouched) {
    oled.setRow(0);
    for (int i = 0; i < NUMBER_OF_AXIS; i++) {
      readval[i] = analogRead(analogpin[i]);
      oled.print(readval[i]);
      oled.print(" / ");
    }
//    if (cap.touched() && 4 != 0) istouched = true;
    delay(100);
  }
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    valuemax[i] = readval[i];
  }
  delay(1000);
}


void loop() {
  oled.home();
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    int proc = 0;
    value[i] = analogRead(analogpin[i]);
    Serial.print("\tEnc" + String(i) + ":");
    Serial.print(value[i]);
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
      case 1:
        Joystick.setYAxis(proc);
      case 2:
        Joystick.setZAxis(proc);
    }
  }
  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);

  //Apply force to each motor.
  oled.home();
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    int myforce;
    if (outofrange[i]) {
      myforce = forcelimit[i];
      (value[i] - valuecen[i] > 0) ? motor[i].TurnRight(myforce) : motor[i].TurnLeft(myforce);
      Serial.print("\t[Limit" + String(i) + "=" + String(value[i]) + "\tApply=" + String(myforce) + "] ");
    } else {
      myforce = map(abs(forces[i]), 0, 255, 0, forcemax[i]);
      (forces[i] > 0) ? motor[i].TurnLeft(myforce) : motor[i].TurnRight(myforce);
      Serial.print("\t[Force" + String(i) + "=" + String(forces[i]) + "\tApply=" + String(myforce) + "] ");
    }
    oled.setCol(65);
    oled.print(axisname[i]);
    oled.print("F:");
    oled.println(myforce);
  }
  Serial.println("");
}
