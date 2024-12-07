

#include "Arduino.h"
#include <Wire.h>
#define ARDUINOFFBJOYSTICK_LIB_VERSION             (F("0.1.0"))
//#include "sensors/MagneticSensorI2C.h"
//#include "SelectableI2CTCA9548A.h"


/* #include <SimpleFOC.h>
#ifndef MAGNETICSENSORSELECTABLEI2C_LIB_H
#define MAGNETICSENSORSELECTABLEI2C_LIB_H
class SelectableI2CTCA9548A {
  public:
    static constexpr uint8_t TCA9548A_I2C_ADDR_BASE = 0x70;
    static constexpr uint8_t TCA9548A_I2C_ADDR = TCA9548A_I2C_ADDR_BASE + 0;

    SelectableI2CTCA9548A(uint8_t _tca_addr = TCA9548A_I2C_ADDR)
      : tca_addr(_tca_addr) {
    }

    void init(TwoWire* _wire = &Wire) {
      wire = _wire;
    }

    void setTcaChannel(uint8_t _ch);

  protected:
    uint8_t tca_addr;
    TwoWire* wire;
};

class MagneticSensorSelectableI2C: public SelectableI2CTCA9548A, public MagneticSensorI2C {
  public:
    MagneticSensorSelectableI2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _msb_bits_used, uint8_t _ch = 0, uint8_t _tca_addr = TCA9548A_I2C_ADDR);

    MagneticSensorSelectableI2C(MagneticSensorI2CConfig_s config, uint8_t _ch = 0, uint8_t _tca_addr = TCA9548A_I2C_ADDR);

    void init(TwoWire* _wire = &Wire);

    float getSensorAngle() override;

  protected:
    uint8_t ch;
};

#endif
*/

typedef signed long s32;
s32 ROTATION_MAX; //milos
s32 ROTATION_MID; //milos

#define ENCODER_MAX_VALUE 2048
#define ENCODER_MIN_VALUE -2047
/* Wireing is the same as SMC3. Please refer to https://www.xsimulator.net/community/threads/smc3-arduino-3dof-motor-driver-and-windows-utilities.4957/
  ENA ----> IN1/RPWM
  ENB --+-> EN1/R_EN
        +-> EN2/L_EN (ie connect ENB to both R_EN and L_EN)
  PWM ----> IN2/L_PWM
https://github.com/luisllamasbinaburo/Arduino-BTS7960#usage
  BTS7960 motor1(L_EN, R_EN, L_PWM, R_PWM);
      ----> BTS7960 motor1(ENB, PWM, ENA);
*/
#define ENApin1  2          // ENA output pin for Motor H-Bridge 1 (ie PortD bit position) 
#define ENBpin1 3          // ENB output pin for Motor H-Bridge 1 (ie PortD bit position)
#define ENApin2 4          // ENA output pin for Motor H-Bridge 2 (ie PortD bit position)
#define ENBpin2 5          // ENB output pin for Motor H-Bridge 2 (ie PortD bit position)
#define ENApin3 6          // ENA output pin for Motor H-Bridge 3 (ie PortD bit position)
#define ENBpin3 7          // ENB output pin for Motor H-Bridge 3 (ie PortD bit position)
#define PWMpin1 9          // PWM output pin for Motor 1   
#define PWMpin2 10         // PWM output pin for Motor 2    
#define PWMpin3 11         // PWM output pin for Motor 3 

#define MPDEFAULTADDRESS 0x70 // I2C Multiplexor default address
