#include "AVR_PWM.h"    //https://github.com/khoih-prog/AVR_PWM
#include <Wire.h>       //https://forum.arduino.cc/t/use-i2c-for-communication-between-arduinos/653958/2

#define I2C_ADD_SLAVE     0x8       // I2C address for Slave mega2560
#define I2C_ADD_MASTER    0x9       // I2C address for master leonald
#define PWMFREQUENCY      10000.0f  //20000.0f
#define TOTALAXIS         3         // Which means x,y and z axis
#define FORCE_MIN         -250     //
#define FORCE_MAX         250     //
#define LEFT              0
#define RIGHT             1
#define PIN_LPWM0         2
#define PIN_RPWM0         3
#define PIN_EN0           8
#define PIN_LPWM1         6
#define PIN_RPWM1         7  // 4 not work
#define PIN_EN1           9
#define PIN_LPWM2         11
#define PIN_RPWM2         12
#define PIN_EN2           10

#define PIN_LED           13


//#define HOLDDELAY         100       // Microsec(not milisec)
  // Pins tested OK in Mega
  //#define pinToUse      12            // Timer1B on Mega
  //#define pinToUse      11            // Timer1A on Mega
  //#define pinToUse       9            // Timer2B on Mega
  //#define pinToUse       2            // Timer3B on Mega
  //#define pinToUse       3            // Timer3C on Mega
  //#define pinToUse       5            // Timer3A on Mega
  //#define pinToUse       6            // Timer4A on Mega
  //#define pinToUse       7            // Timer4B on Mega
  //#define pinToUse       8            // Timer4C on Mega
  //#define pinToUse      46            // Timer5A on Mega
  //#define pinToUse      45            // Timer5B on Mega
  //#define pinToUse      44            // Timer5C on Mega
struct Data{
  byte cmd;
  byte axis;
  byte dir;
  uint8_t force;
};

// Declare
uint8_t LPWM_Pins[] =    { PIN_LPWM0, PIN_LPWM1, PIN_LPWM2 };  // PWM pins for left roatation
uint8_t RPWM_Pins[] =    { PIN_RPWM0, PIN_RPWM1, PIN_RPWM2 };  // PWM pins for right roatation
uint8_t EN_Pins[]   =    { PIN_EN0, PIN_EN1, PIN_EN2 };  // PWM pins for right roatation

//float freq[] =            { 0, 0, 0 };  // PWM freq
//AVR_PWM* PWM_Instance;

volatile Data rxData;                   // Data from master
volatile bool newRxData;                // True if new data
Data newData[TOTALAXIS];           // Data currently processing
Data oldData[TOTALAXIS];           // Previous data
bool inProc[TOTALAXIS];            // True if changing PWM by new data
long inProctime[TOTALAXIS];        // FOr caliculating cooling time

// creates pwm instance
AVR_PWM* LPWM_Instance[TOTALAXIS];
AVR_PWM* RPWM_Instance[TOTALAXIS];

void setup(){
  Serial.begin(115200);
  Serial.print(F("\nInitializing Slave."));
/*
  byte testpin =2;
  LPWM_Instance[0] = new AVR_PWM(8, PWMFREQUENCY, 50);
  LPWM_Instance[0]->setPWM();
  RPWM_Instance[0] = new AVR_PWM(11, PWMFREQUENCY, 50);
  RPWM_Instance[0]->setPWM();
  while(true){
    LPWM_Instance[0]->setPWM_DCPercentageInt_manual( 8, 32768 );
    LPWM_Instance[0]->setPWM_DCPercentageInt_manual( 6, 32768 );
    delay(1000);
    LPWM_Instance[0]->setPWM_DCPercentageInt_manual( 8, 12000 );
    LPWM_Instance[0]->setPWM_DCPercentageInt_manual( 6, 12000 );
    delay(1000);
  }
*/
  for (uint8_t i = 0; i < TOTALAXIS; i++){
    pinMode(EN_Pins[i], OUTPUT); //Enable pin must low while init PWM pin
    pinMode(LPWM_Pins[i], OUTPUT); 
    pinMode(RPWM_Pins[i], OUTPUT); 
    digitalWrite(EN_Pins[i],LOW);
    digitalWrite(LPWM_Pins[i],LOW);
    digitalWrite(RPWM_Pins[i],LOW);
    //digitalWrite(REN_Pins[i],LOW);
    LPWM_Instance[i] = new AVR_PWM(LPWM_Pins[i], PWMFREQUENCY, 0);
    LPWM_Instance[i]->setPWM();
    Serial.print("\tLeft:");
    Serial.print(LPWM_Pins[i]);
    delay(100);
    RPWM_Instance[i] = new AVR_PWM(RPWM_Pins[i], PWMFREQUENCY, 0);
    RPWM_Instance[i]->setPWM();
    Serial.print("\tRight:");
    Serial.print(RPWM_Pins[i]);
    delay(100);
    digitalWrite(EN_Pins[i],HIGH);
    Serial.print("\tEnable:");
    Serial.print(EN_Pins[i]);
  }
  Serial.print("\nSlave Initialized: ");
  Wire.begin(I2C_ADD_SLAVE);
  Serial.print(I2C_ADD_SLAVE );
  newRxData = false;
  Wire.onReceive(receiveEvent);
  Serial.print("\nStarted data reciving.\nInitialization end.");
}

void loop(){
  if( newRxData == true ) {
    if( rxData.axis < TOTALAXIS) {
      if(inProc[rxData.axis] == false ){
        newData[rxData.axis].cmd = rxData.cmd;
        newData[rxData.axis].dir = rxData.dir;
        newData[rxData.axis].force = rxData.force;
        newData[rxData.axis].axis = rxData.axis;
      }else{
        Serial.print( "Warning.Overflow." );
      }
    }else{
      Serial.print( "Warning.Index Out of range." );
    }
    newRxData = false;
  }
  for (uint8_t i = 0; i < TOTALAXIS; i++){
    inProc[i] = false;
    uint16_t new_level = 0;
    if( oldData[i].dir != newData[i].dir ){
      Serial.print( "Dir_Chg" );
      Serial.print( i );
      Serial.print( "\tF:" );
      Serial.print( newData[i].force );
      // Dir was Changed.
      inProc[i] = true;
      if( oldData[i].dir == LEFT ){c:\Users\LocalAdmin\Documents\Arduino\libraries\DynamicHID\PIDReportHandler.h c:\Users\LocalAdmin\Documents\Arduino\libraries\DynamicHID\PIDReportType.h c:\Users\LocalAdmin\Documents\Arduino\libraries\DynamicHID\DynamicHID.cpp c:\Users\LocalAdmin\Documents\Arduino\libraries\DynamicHID\DynamicHID.h c:\Users\LocalAdmin\Documents\Arduino\libraries\DynamicHID\PIDReportHandler.cpp
        LPWM_Instance[i]->setPWM_manual( LPWM_Pins[i], 0 ); //Stop PWM
        LPWM_Instance[i]->setPWM();
        delayMicroseconds(100);
        new_level = newData[i].force * RPWM_Instance[i]->getPWMPeriod() / 256.0f ;
        RPWM_Instance[i]->setPWM_manual( RPWM_Pins[i], new_level ); //force range 0-250 expand to PWM range 0-65535
        RPWM_Instance[i]->setPWM();
        delayMicroseconds(100);
        Serial.print( "\tPWM:" );
        Serial.print( new_level );
      }else{
        RPWM_Instance[i]->setPWM_manual( RPWM_Pins[i], 0 );
        RPWM_Instance[i]->setPWM();
        delayMicroseconds(100);
        new_level = newData[i].force * LPWM_Instance[i]->getPWMPeriod() / 256.0f ;
        LPWM_Instance[i]->setPWM_manual( LPWM_Pins[i], new_level ); //force range 0-250 expand to PWM range 0-65535
        LPWM_Instance[i]->setPWM();
        delayMicroseconds(100);
        Serial.print( "\tPWM:" );
        Serial.print( new_level );
      }
    }else if( oldData[i].force != newData[i].force ){
      Serial.print( "Force_chg" );
      Serial.print( i );
      Serial.print( "\tF:" );
      Serial.print( newData[i].force );
      // Only force was Changed.
      inProc[i] = true;
      //uint16_t dutycyclePercentInt;
      //dutycyclePercentInt = constrain(newData[i].force, 0, FORCE_MAX);  // Just in case.It will clip under 250. 

      if( newData[i].dir == LEFT){
        new_level = newData[i].force * LPWM_Instance[i]->getPWMPeriod() / 256.0f ;
        LPWM_Instance[i]->setPWM_manual( LPWM_Pins[i],new_level  ); //force range 0-250 expand to PWM range 0-65535
      }else{
        new_level = newData[i].force * RPWM_Instance[i]->getPWMPeriod() / 256.0f ;
        RPWM_Instance[i]->setPWM_manual( RPWM_Pins[i], new_level );
      }
        Serial.print( "\tPWM:" );
        Serial.print( new_level );
    }
    if(inProc[i] == true ){
      oldData[i]= newData[i];
      inProc[i] = false;
      Serial.println("");
    }
    /*
      Serial.print( "Changed" );
      Serial.print( i );
      Serial.print( "\tforce:" );
      Serial.print( oldData[i].force );
      Serial.print( "\tdir:" );
      Serial.println( oldData[i].dir );
    */
  }
}

void receiveEvent(int numBytesReceived) {
  if (newRxData == false) {
    // copy data to currentData
    Wire.readBytes( (byte*) &rxData, numBytesReceived);
    newRxData = true;
  } else {
    // dump the data
    while(Wire.available() > 0) {
      byte c = Wire.read();
    }
  }
}
