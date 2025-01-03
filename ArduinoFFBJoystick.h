#include "Arduino.h"
#include <Wire.h>
#define ARDUINOFFBJOYSTICK_LIB_VERSION             (F("0.1.0"))

#define MPDEFAULTADDRESS 0x70 // I2C Multiplexor default address
#define ENCODER_MAX_VALUE 4096
#define ENCODER_MIN_VALUE 0

typedef signed long s32;
s32 ROTATION_MAX; //milos
s32 ROTATION_MID; //milos
#define JOYSTICK_DEFAULT_REPORT_ID         0x01
#define JOYSTICK_DEFAULT_BUTTON_COUNT        32
#define JOYSTICK_DEFAULT_AXIS_MINIMUM         0
#define JOYSTICK_DEFAULT_AXIS_MAXIMUM      1023
#define JOYSTICK_DEFAULT_SIMULATOR_MINIMUM    0
#define JOYSTICK_DEFAULT_SIMULATOR_MAXIMUM 1023
#define JOYSTICK_DEFAULT_HATSWITCH_COUNT      2
#define JOYSTICK_HATSWITCH_COUNT_MAXIMUM      2
#define JOYSTICK_HATSWITCH_RELEASE           -1
#define JOYSTICK_TYPE_JOYSTICK             0x04
#define JOYSTICK_TYPE_GAMEPAD              0x05
#define JOYSTICK_TYPE_MULTI_AXIS           0x08

#define DIRECTION_ENABLE                   0x04
#define X_AXIS_ENABLE                      0x01
#define Y_AXIS_ENABLE                      0x02
#define FORCE_FEEDBACK_MAXGAIN              100
#define DEG_TO_RAD              ((float)((float)3.14159265359 / 180.0))

struct Gains{
    uint8_t totalGain         = FORCE_FEEDBACK_MAXGAIN;
	uint8_t constantGain      = FORCE_FEEDBACK_MAXGAIN;
	uint8_t rampGain          = FORCE_FEEDBACK_MAXGAIN;
	uint8_t squareGain        = FORCE_FEEDBACK_MAXGAIN;
	uint8_t sineGain          = FORCE_FEEDBACK_MAXGAIN;
	uint8_t triangleGain      = FORCE_FEEDBACK_MAXGAIN;
	uint8_t sawtoothdownGain  = FORCE_FEEDBACK_MAXGAIN;
	uint8_t sawtoothupGain    = FORCE_FEEDBACK_MAXGAIN;
	uint8_t springGain        = FORCE_FEEDBACK_MAXGAIN;
	uint8_t damperGain        = FORCE_FEEDBACK_MAXGAIN;
	uint8_t inertiaGain       = FORCE_FEEDBACK_MAXGAIN;
	uint8_t frictionGain      = FORCE_FEEDBACK_MAXGAIN;
	uint8_t customGain        = FORCE_FEEDBACK_MAXGAIN;
};

struct EffectParams{
    int32_t springMaxPosition = 0;
    int32_t springPosition = 0;

    int32_t damperMaxVelocity = 0;
    int32_t damperVelocity = 0;

    int32_t inertiaMaxAcceleration = 0;
    int32_t inertiaAcceleration = 0;

    int32_t frictionMaxPositionChange = 0;
    int32_t frictionPositionChange = 0;
};
