/* Haptic regions, 5-degree release hysteresis and latched flight-mode policy. */
#include "haptic_controller.h"

#include <math.h>
#include <stddef.h>

#define RAD_TO_DEG (57.29577951308232f)
#define AXIS_COMPLIANT_ENTER (30.0f)
#define AXIS_CLAMP_ENTER (50.0f)
#define TILT_COMPLIANT_ENTER (20.0f)
#define TILT_CLAMP_ENTER (70.0f)
#define RELEASE_DEADBAND (5.0f)

/*
 * Shared magnitude FSM. Outward transitions use the stated limits and inward
 * transitions require a 5-degree retreat. Exact upper limits enter clamping;
 * this resolves the brief's unspecified equality at +/-50 and at tilt 20.
 * Direct jumps across multiple regions settle in the correct state immediately.
 */
static HapticZone UpdateZone(HapticZone previous, float magnitude,
                              float compliant_enter, float clamp_enter)
{
  const float free_release = compliant_enter - RELEASE_DEADBAND;
  const float clamp_release = clamp_enter - RELEASE_DEADBAND;

  /* A large outward jump must clamp even when the previous state was free. */
  if (magnitude >= clamp_enter)
  {
    return HAPTIC_CLAMPED;
  }

  switch (previous)
  {
    case HAPTIC_CLAMPED:
      /* A sampled jump back to level may cross both release limits at once. */
      if (magnitude <= free_release)
      {
        return HAPTIC_FREE;
      }
      if (magnitude <= clamp_release)
      {
        return HAPTIC_COMPLIANT;
      }
      return HAPTIC_CLAMPED;

    case HAPTIC_COMPLIANT:
      return magnitude <= free_release ? HAPTIC_FREE : HAPTIC_COMPLIANT;

    case HAPTIC_FREE:
    default:
      return magnitude >= compliant_enter ? HAPTIC_COMPLIANT : HAPTIC_FREE;
  }
}

/*
 * Reuse the same FSM for positive/negative roll and pitch. If a sampled command
 * reverses sign, it has crossed the free region; evaluate the new side from
 * free rather than carrying the opposite side's clamped state through zero.
 */
static void UpdateAxis(HapticAxis *axis, float angle)
{
  const int8_t direction = angle > 0.0f ? 1 : (angle < 0.0f ? -1 : 0);

  if (direction != axis->direction)
  {
    axis->zone = HAPTIC_FREE;
  }
  axis->zone = UpdateZone(axis->zone, fabsf(angle),
                          AXIS_COMPLIANT_ENTER, AXIS_CLAMP_ENTER);
  axis->direction = direction;
}

/*
 * Wokwi draws increasing servo angles clockwise (positive SVG rotation).
 * The assignment defines clockwise as negative, so subtract the signed command
 * from the 90-degree neutral position. Positive roll/pitch then move anticlockwise.
 */
static float AxisServoAngle(const HapticAxis *axis, float angle)
{
  if (axis->zone == HAPTIC_FREE)
  {
    return 90.0f;
  }
  if (axis->zone == HAPTIC_CLAMPED)
  {
    /* Hold the hard boundary throughout its inward hysteresis band. */
    return 90.0f - (float)axis->direction * AXIS_CLAMP_ENTER;
  }
  return 90.0f - angle;
}

void Haptic_Init(HapticController *controller)
{
  if (controller == NULL)
  {
    return;
  }

  /* Zero-valued enums deliberately represent free regions and manual mode. */
  *controller = (HapticController){0};
  controller->roll_servo_deg = 90.0f;
  controller->pitch_servo_deg = 90.0f;
  controller->tilt_servo_deg = 0.0f;
}

bool Haptic_CalculateAngles(float ax, float ay, float az, HapticAngles *out)
{
  float scale;
  HapticAngles calculated;

  if (out == NULL || !isfinite(ax) || !isfinite(ay) || !isfinite(az))
  {
    return false;
  }

  /*
   * The equations are independent of common scale. Normalising by the largest
   * component avoids overflow/underflow in squaring without changing angles.
   */
  scale = fmaxf(fabsf(ax), fmaxf(fabsf(ay), fabsf(az)));
  if (scale == 0.0f)
  {
    return false;
  }
  ax /= scale;
  ay /= scale;
  az /= scale;

  calculated.roll_deg = atan2f(ay, az) * RAD_TO_DEG;
  calculated.pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  calculated.tilt_deg = sqrtf(calculated.pitch_deg * calculated.pitch_deg
                              + calculated.roll_deg * calculated.roll_deg);
  *out = calculated;
  return true;
}

void Haptic_Update(HapticController *controller, const HapticAngles *angles,
                   bool sensor_valid, bool button_pressed)
{
  if (controller == NULL)
  {
    return;
  }

  /* Invalid data must never flow into comparisons or PWM conversion. */
  controller->sensor_valid = sensor_valid && angles != NULL
      && isfinite(angles->roll_deg) && isfinite(angles->pitch_deg)
      && isfinite(angles->tilt_deg) && angles->tilt_deg >= 0.0f;
  if (!controller->sensor_valid)
  {
    controller->mode = HAPTIC_AUTOPILOT;
    controller->roll_servo_deg = 90.0f;
    controller->pitch_servo_deg = 90.0f;
    controller->tilt_servo_deg = 0.0f;
    return;
  }

  controller->angles = *angles;
  UpdateAxis(&controller->roll, angles->roll_deg);
  UpdateAxis(&controller->pitch, angles->pitch_deg);
  controller->tilt_zone = UpdateZone(controller->tilt_zone, angles->tilt_deg,
                                     TILT_COMPLIANT_ENTER, TILT_CLAMP_ENTER);

  /* Automatic entry has priority over a button event in the same sample. */
  if (controller->roll.zone == HAPTIC_CLAMPED
      && controller->pitch.zone == HAPTIC_CLAMPED)
  {
    controller->mode = HAPTIC_AUTOPILOT;
  }
  else if (button_pressed)
  {
    if (controller->mode == HAPTIC_MANUAL)
    {
      controller->mode = HAPTIC_AUTOPILOT;
    }
    else if (controller->roll.zone == HAPTIC_FREE
             && controller->pitch.zone == HAPTIC_FREE
             && fabsf(angles->roll_deg) < AXIS_COMPLIANT_ENTER
             && fabsf(angles->pitch_deg) < AXIS_COMPLIANT_ENTER)
    {
      /* A rejected press is consumed; a later safe sample needs a new press. */
      controller->mode = HAPTIC_MANUAL;
    }
  }

  controller->roll_servo_deg = AxisServoAngle(&controller->roll, angles->roll_deg);
  controller->pitch_servo_deg = AxisServoAngle(&controller->pitch, angles->pitch_deg);

  /* Tilt has no sign and uses twice the detected angle to fill the servo range. */
  switch (controller->tilt_zone)
  {
    case HAPTIC_CLAMPED:
      controller->tilt_servo_deg = 2.0f * TILT_CLAMP_ENTER;
      break;
    case HAPTIC_COMPLIANT:
      controller->tilt_servo_deg = 2.0f * angles->tilt_deg;
      break;
    case HAPTIC_FREE:
    default:
      controller->tilt_servo_deg = 0.0f;
      break;
  }
}
