/* Host regression tests for the assignment's angle equations and FSM policy. */
#include "main.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned checks;
static unsigned cases;

/* Fail with the original expression and source line rather than losing context. */
#define CHECK(condition) do { \
  ++checks; \
  if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
    exit(EXIT_FAILURE); \
  } \
} while (0)
#define NEAR(actual, expected) CHECK(fabsf((actual) - (expected)) < 0.002f)

/* Feed a consistent synthetic angle sample; accelerometer conversion is separate. */
static void sample(HapticController *c, float roll, float pitch, bool press)
{
  const HapticAngles a = {roll, pitch, sqrtf(roll * roll + pitch * pitch)};
  Haptic_Update(c, &a, true, press);
}

/* Construct a 1g static gravity vector for specified roll and pitch in degrees. */
static HapticAngles gravity_angles(float roll, float pitch)
{
  const float rad = 0.017453292519943295f;
  const float ax = -sinf(pitch * rad);
  const float ay = sinf(roll * rad) * cosf(pitch * rad);
  const float az = cosf(roll * rad) * cosf(pitch * rad);
  HapticAngles result = {0};
  NEAR(sqrtf(ax * ax + ay * ay + az * az), 1.0f);
  CHECK(Haptic_CalculateAngles(ax, ay, az, &result));
  NEAR(result.roll_deg, roll);
  NEAR(result.pitch_deg, pitch);
  NEAR(result.tilt_deg, sqrtf(roll * roll + pitch * pitch));
  return result;
}

static void test_initial_state(void)
{
  HapticController c;
  Haptic_Init(&c);
  CHECK(c.mode == HAPTIC_MANUAL);
  CHECK(c.roll.zone == HAPTIC_FREE && c.pitch.zone == HAPTIC_FREE);
  CHECK(c.tilt_zone == HAPTIC_FREE);
  NEAR(c.roll_servo_deg, 90.0f);
  NEAR(c.pitch_servo_deg, 90.0f);
  NEAR(c.tilt_servo_deg, 0.0f);
  ++cases;
}

static void test_angles(void)
{
  HapticAngles a;
  (void)gravity_angles(0.0f, 0.0f);
  (void)gravity_angles(40.0f, 0.0f);
  (void)gravity_angles(-40.0f, 0.0f);
  (void)gravity_angles(0.0f, 40.0f);
  (void)gravity_angles(0.0f, -40.0f);
  (void)gravity_angles(20.0f, 20.0f);
  (void)gravity_angles(-60.0f, 60.0f);
  /* Negative az must select the correct roll quadrant. */
  (void)gravity_angles(135.0f, 10.0f);
  CHECK(Haptic_CalculateAngles(-8192.0f, 0.0f, 8192.0f, &a));
  NEAR(a.pitch_deg, 45.0f);
  NEAR(a.roll_deg, 0.0f);
  /* Shared-scale normalisation must keep finite extreme inputs usable. */
  CHECK(Haptic_CalculateAngles(-FLT_MAX, 0.0f, FLT_MAX, &a));
  NEAR(a.pitch_deg, 45.0f);
  CHECK(Haptic_CalculateAngles(-FLT_MIN, 0.0f, FLT_MIN, &a));
  NEAR(a.pitch_deg, 45.0f);
  ++cases;
}

static void test_invalid_angles(void)
{
  HapticAngles a = {11.0f, 12.0f, 13.0f};
  CHECK(!Haptic_CalculateAngles(0.0f, 0.0f, 0.0f, &a));
  CHECK(!Haptic_CalculateAngles(NAN, 1.0f, 0.0f, &a));
  CHECK(!Haptic_CalculateAngles(0.0f, INFINITY, 1.0f, &a));
  CHECK(!Haptic_CalculateAngles(0.0f, 1.0f, -INFINITY, &a));
  CHECK(!Haptic_CalculateAngles(0.0f, 0.0f, 1.0f, NULL));
  NEAR(a.roll_deg, 11.0f);
  NEAR(a.pitch_deg, 12.0f);
  NEAR(a.tilt_deg, 13.0f);
  ++cases;
}

/* Repeat every axis transition with both signs and for both roll and pitch. */
static void test_axis_boundaries(bool pitch_axis, float sign)
{
  HapticController c;
  HapticAxis *axis;
  float *servo;
  const float magnitudes[] = {29.99f, 30.0f, 29.0f, 25.01f, 25.0f,
                              49.99f, 50.0f, 49.0f, 45.01f, 45.0f,
                              30.0f, 25.01f, 25.0f};
  const HapticZone expected[] = {HAPTIC_FREE, HAPTIC_COMPLIANT,
      HAPTIC_COMPLIANT, HAPTIC_COMPLIANT, HAPTIC_FREE,
      HAPTIC_COMPLIANT, HAPTIC_CLAMPED, HAPTIC_CLAMPED,
      HAPTIC_CLAMPED, HAPTIC_COMPLIANT, HAPTIC_COMPLIANT,
      HAPTIC_COMPLIANT, HAPTIC_FREE};
  Haptic_Init(&c);
  axis = pitch_axis ? &c.pitch : &c.roll;
  servo = pitch_axis ? &c.pitch_servo_deg : &c.roll_servo_deg;
  for (unsigned i = 0; i < sizeof magnitudes / sizeof magnitudes[0]; ++i)
  {
    const float angle = sign * magnitudes[i];
    sample(&c, pitch_axis ? 0.0f : angle, pitch_axis ? angle : 0.0f, false);
    CHECK(axis->zone == expected[i]);
    CHECK(axis->direction == (sign > 0.0f ? 1 : -1));
    if (expected[i] == HAPTIC_FREE) { NEAR(*servo, 90.0f); }
    else if (expected[i] == HAPTIC_CLAMPED) { NEAR(*servo, 90.0f - sign * 50.0f); }
    else { NEAR(*servo, 90.0f - angle); }
  }
  ++cases;
}

static void test_axis_jumps(void)
{
  HapticController c;
  Haptic_Init(&c);
  sample(&c, 100.0f, 0.0f, false);
  CHECK(c.roll.zone == HAPTIC_CLAMPED);
  NEAR(c.roll_servo_deg, 40.0f);
  sample(&c, -48.0f, 0.0f, false);
  CHECK(c.roll.zone == HAPTIC_COMPLIANT);
  NEAR(c.roll_servo_deg, 138.0f);
  sample(&c, -60.0f, 0.0f, false);
  CHECK(c.roll.zone == HAPTIC_CLAMPED);
  NEAR(c.roll_servo_deg, 140.0f);
  sample(&c, -10.0f, 0.0f, false);
  CHECK(c.roll.zone == HAPTIC_FREE);
  NEAR(c.roll_servo_deg, 90.0f);
  sample(&c, -60.0f, 0.0f, false);
  sample(&c, 28.0f, 0.0f, false);
  CHECK(c.roll.zone == HAPTIC_FREE);
  NEAR(c.roll_servo_deg, 90.0f);
  ++cases;
}

static void test_tilt_boundaries(void)
{
  HapticController c;
  const float magnitudes[] = {19.99f, 20.0f, 19.0f, 15.01f, 15.0f,
                              69.99f, 70.0f, 69.0f, 65.01f, 65.0f,
                              15.01f, 15.0f, 100.0f, 0.0f};
  const HapticZone expected[] = {HAPTIC_FREE, HAPTIC_COMPLIANT,
      HAPTIC_COMPLIANT, HAPTIC_COMPLIANT, HAPTIC_FREE,
      HAPTIC_COMPLIANT, HAPTIC_CLAMPED, HAPTIC_CLAMPED,
      HAPTIC_CLAMPED, HAPTIC_COMPLIANT, HAPTIC_COMPLIANT,
      HAPTIC_FREE, HAPTIC_CLAMPED, HAPTIC_FREE};
  Haptic_Init(&c);
  for (unsigned i = 0; i < sizeof magnitudes / sizeof magnitudes[0]; ++i)
  {
    sample(&c, magnitudes[i], 0.0f, false);
    CHECK(c.tilt_zone == expected[i]);
    if (expected[i] == HAPTIC_FREE) { NEAR(c.tilt_servo_deg, 0.0f); }
    else if (expected[i] == HAPTIC_CLAMPED) { NEAR(c.tilt_servo_deg, 140.0f); }
    else { NEAR(c.tilt_servo_deg, 2.0f * magnitudes[i]); }
  }
  ++cases;
}

static void test_gravity_to_outputs(void)
{
  HapticController c;
  HapticAngles a;
  Haptic_Init(&c);
  a = gravity_angles(40.0f, 0.0f);
  Haptic_Update(&c, &a, true, false);
  CHECK(c.roll.zone == HAPTIC_COMPLIANT);
  CHECK(c.pitch.zone == HAPTIC_FREE);
  NEAR(c.roll_servo_deg, 50.0f);
  NEAR(c.pitch_servo_deg, 90.0f);
  Haptic_Init(&c);
  a = gravity_angles(0.0f, -40.0f);
  Haptic_Update(&c, &a, true, false);
  CHECK(c.roll.zone == HAPTIC_FREE);
  CHECK(c.pitch.zone == HAPTIC_COMPLIANT);
  NEAR(c.pitch_servo_deg, 130.0f);
  Haptic_Init(&c);
  a = gravity_angles(20.0f, 20.0f);
  Haptic_Update(&c, &a, true, false);
  CHECK(c.roll.zone == HAPTIC_FREE && c.pitch.zone == HAPTIC_FREE);
  CHECK(c.tilt_zone == HAPTIC_COMPLIANT);
  NEAR(c.roll_servo_deg, 90.0f);
  NEAR(c.pitch_servo_deg, 90.0f);
  NEAR(c.tilt_servo_deg, 56.568542f);
  ++cases;
}

static void test_autopilot(void)
{
  HapticController c;
  Haptic_Init(&c);
  sample(&c, 60.0f, 0.0f, false);
  CHECK(c.mode == HAPTIC_MANUAL);
  sample(&c, 60.0f, -60.0f, true);
  CHECK(c.mode == HAPTIC_AUTOPILOT);
  NEAR(c.roll_servo_deg, 40.0f);
  NEAR(c.pitch_servo_deg, 140.0f);
  sample(&c, 0.0f, 0.0f, false);
  CHECK(c.mode == HAPTIC_AUTOPILOT);
  sample(&c, 0.0f, 0.0f, true);
  CHECK(c.mode == HAPTIC_MANUAL);
  /* A manual press requests autopilot even at level. */
  sample(&c, 0.0f, 0.0f, true);
  CHECK(c.mode == HAPTIC_AUTOPILOT);
  ++cases;
}

static void test_safe_manual_release(void)
{
  HapticController c;
  Haptic_Init(&c);
  sample(&c, 60.0f, 60.0f, false);
  sample(&c, 30.0f, 0.0f, true);
  CHECK(c.mode == HAPTIC_AUTOPILOT);
  /* Actual angles inside +/-30 do not erase an active release deadband. */
  sample(&c, 29.0f, 0.0f, true);
  CHECK(c.roll.zone == HAPTIC_COMPLIANT);
  CHECK(c.mode == HAPTIC_AUTOPILOT);
  /* The earlier unsafe event must not be queued for later. */
  sample(&c, 0.0f, 0.0f, false);
  CHECK(c.mode == HAPTIC_AUTOPILOT);
  sample(&c, 0.0f, 0.0f, true);
  CHECK(c.mode == HAPTIC_MANUAL);
  /* With free histories, a fresh event at 29 degrees is safe. */
  sample(&c, 0.0f, 0.0f, true);
  sample(&c, 29.0f, -29.0f, true);
  CHECK(c.mode == HAPTIC_MANUAL);
  ++cases;
}

static void test_sensor_failure(void)
{
  HapticController c;
  HapticAngles invalid = {NAN, 0.0f, 0.0f};
  Haptic_Init(&c);
  sample(&c, 40.0f, 0.0f, false);
  Haptic_Update(&c, NULL, false, true);
  CHECK(!c.sensor_valid);
  CHECK(c.mode == HAPTIC_AUTOPILOT);
  CHECK(c.roll.zone == HAPTIC_COMPLIANT);
  NEAR(c.angles.roll_deg, 40.0f);
  NEAR(c.roll_servo_deg, 90.0f);
  NEAR(c.pitch_servo_deg, 90.0f);
  NEAR(c.tilt_servo_deg, 0.0f);
  sample(&c, 40.0f, 0.0f, false);
  CHECK(c.sensor_valid);
  CHECK(c.mode == HAPTIC_AUTOPILOT);
  NEAR(c.roll_servo_deg, 50.0f);
  Haptic_Update(&c, &invalid, true, false);
  CHECK(!c.sensor_valid);
  NEAR(c.roll_servo_deg, 90.0f);
  invalid = (HapticAngles){0.0f, 0.0f, -1.0f};
  Haptic_Update(&c, &invalid, true, true);
  CHECK(!c.sensor_valid && c.mode == HAPTIC_AUTOPILOT);
  sample(&c, 0.0f, 0.0f, false);
  CHECK(c.mode == HAPTIC_AUTOPILOT);
  sample(&c, 0.0f, 0.0f, true);
  CHECK(c.mode == HAPTIC_MANUAL);
  ++cases;
}

int main(void)
{
  test_initial_state();
  test_angles();
  test_invalid_angles();
  test_axis_boundaries(false, 1.0f);
  test_axis_boundaries(false, -1.0f);
  test_axis_boundaries(true, 1.0f);
  test_axis_boundaries(true, -1.0f);
  test_axis_jumps();
  test_tilt_boundaries();
  test_gravity_to_outputs();
  test_autopilot();
  test_safe_manual_release();
  test_sensor_failure();
  printf("PASS: %u cases, %u checks (angle conversion, signed FSMs, tilt, modes, faults)\n",
         cases, checks);
  return EXIT_SUCCESS;
}
