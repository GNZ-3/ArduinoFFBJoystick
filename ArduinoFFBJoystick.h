

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


