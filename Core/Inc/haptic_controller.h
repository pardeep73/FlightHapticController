/*
 * Assignment haptic controller, independent of the STM32 HAL.
 * Keeping the policy separate from I2C/PWM allows its transitions to be tested
 * with the same C implementation on a host computer and on the microcontroller.
 */
#ifndef HAPTIC_CONTROLLER_H
#define HAPTIC_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/* Each signed axis uses one region plus the direction in which it was entered. */
typedef enum
{
  HAPTIC_FREE = 0,
  HAPTIC_COMPLIANT,
  HAPTIC_CLAMPED
} HapticZone;

/* Autopilot is a latched mode; returning to level alone does not release it. */
typedef enum
{
  HAPTIC_MANUAL = 0,
  HAPTIC_AUTOPILOT
} HapticMode;

/*
 * All measured angles are degrees. Positive roll/pitch mean anticlockwise as
 * viewed in Wokwi; their servo commands are 90 minus the signed angle. Tilt is
 * sqrt(pitch^2 + roll^2), not a signed angle, and its servo command is 2*tilt.
 */
typedef struct
{
  float roll_deg;
  float pitch_deg;
  float tilt_deg;
} HapticAngles;

/* Direction is -1, 0 or +1, so hysteresis never persists across a sign reversal. */
typedef struct
{
  HapticZone zone;
  int8_t direction;
} HapticAxis;

/* Public state also provides convenient watch expressions in the debugger. */
typedef struct
{
  HapticAxis roll;
  HapticAxis pitch;
  HapticZone tilt_zone;
  HapticMode mode;
  HapticAngles angles;
  bool sensor_valid;
  float roll_servo_deg;
  float pitch_servo_deg;
  float tilt_servo_deg;
} HapticController;

/* Start in manual mode, with free regions and neutral servo positions. */
void Haptic_Init(HapticController *controller);

/*
 * Calculate assignment equations from acceleration components in any common
 * unit (g or raw ADC counts). Reject a zero/nonfinite vector and leave *out
 * unchanged on failure. A nonzero static gravity vector is required.
 */
bool Haptic_CalculateAngles(float ax, float ay, float az, HapticAngles *out);

/*
 * Advance all FSMs once per sample. button_pressed is one debounced rising-edge
 * event, not a raw GPIO level. A sensor failure latches autopilot, keeps the last
 * valid angles/region history, and neutralises all outputs until valid readings
 * resume. The operator must subsequently press the button at safe angles to
 * return to manual. Valid feedback continues in both operating modes because
 * the assignment does not model autopilot flight control or aircraft dynamics.
 */
void Haptic_Update(HapticController *controller, const HapticAngles *angles,
                   bool sensor_valid, bool button_pressed);

#endif /* HAPTIC_CONTROLLER_H */
