/* Exercise the real bit-level driver against an edge-driven I2C slave model. */
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned cases;
static unsigned checks;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
  fprintf(stderr, "FAIL software I2C line %d: %s\n", __LINE__, #condition); \
  exit(EXIT_FAILURE); } } while (0)

typedef enum { RECEIVE_ADDRESS, RECEIVE_REGISTER, RECEIVE_DATA,
               SLAVE_ACK, SEND_DATA, MASTER_ACK, IGNORE } SlavePhase;

typedef struct
{
  bool master_scl, master_sda, scl;
  bool slave_low, active;
  bool clock_stuck, data_stuck;
  bool nack_address, nack_read_address, nack_register, frozen_tick;
  unsigned nack_data_index;
  unsigned release_after_pulses, recovery_pulses;
  uint32_t base_ms;
  uint64_t us, stretch_until;
  uint32_t stretch_us;
  bool stretch_used;
  SlavePhase phase, after_ack;
  unsigned bits;
  uint8_t value, pointer, read_value;
  bool ack_value, ack_set, ack_clock_seen, master_ack_ready;
  unsigned starts, repeated_starts, stops, addresses, registers;
  unsigned written_bytes, read_bytes, master_acks, master_nacks;
  uint8_t address_bytes[8];
  uint8_t memory[256];
} Slave;

static bool line_sda(const Slave *s)
{
  return s->master_sda && !s->slave_low && !s->data_stuck;
}

static void start_condition(Slave *s)
{
  ++s->starts;
  if (s->active) ++s->repeated_starts;
  s->active = true;
  s->phase = RECEIVE_ADDRESS;
  s->bits = 0;
  s->value = 0;
  s->slave_low = false;
}

static void stop_condition(Slave *s)
{
  ++s->stops;
  s->active = false;
  s->slave_low = false;
}

/* Decode bytes received on actual rising SCL edges, including address/direction. */
static void received_byte(Slave *s)
{
  const uint8_t value = s->value;
  bool acknowledge = true;
  SlavePhase next = IGNORE;
  if (s->phase == RECEIVE_ADDRESS)
  {
    CHECK(s->addresses < sizeof s->address_bytes);
    s->address_bytes[s->addresses++] = value;
    acknowledge = (value >> 1) == 0x68U && !s->nack_address
        && !(s->nack_read_address && (value & 1U) != 0);
    next = (value & 1U) != 0 ? SEND_DATA : RECEIVE_REGISTER;
    s->read_value = s->memory[s->pointer];
  }
  else if (s->phase == RECEIVE_REGISTER)
  {
    ++s->registers;
    acknowledge = !s->nack_register;
    if (acknowledge) s->pointer = value;
    next = RECEIVE_DATA;
  }
  else
  {
    ++s->written_bytes;
    acknowledge = s->written_bytes != s->nack_data_index;
    if (acknowledge) s->memory[s->pointer++] = value;
    next = RECEIVE_DATA;
  }
  s->ack_value = acknowledge;
  s->after_ack = acknowledge ? next : IGNORE;
  s->phase = SLAVE_ACK;
  s->ack_set = false;
  s->ack_clock_seen = false;
}

static void rising_edge(Slave *s)
{
  if (s->data_stuck && s->release_after_pulses != 0)
  {
    if (++s->recovery_pulses >= s->release_after_pulses) s->data_stuck = false;
  }
  if (!s->active) return;
  switch (s->phase)
  {
    case RECEIVE_ADDRESS:
    case RECEIVE_REGISTER:
    case RECEIVE_DATA:
      s->value = (uint8_t)((s->value << 1) | (line_sda(s) ? 1U : 0U));
      if (++s->bits == 8) received_byte(s);
      break;
    case SLAVE_ACK:
      if (s->ack_set) s->ack_clock_seen = true;
      break;
    case SEND_DATA:
      if (++s->bits == 8)
      {
        ++s->read_bytes;
        s->phase = MASTER_ACK;
        s->master_ack_ready = false;
      }
      break;
    case MASTER_ACK:
      if (s->master_ack_ready)
      {
        if (!line_sda(s))
        {
          ++s->master_acks;
          ++s->pointer;
          s->read_value = s->memory[s->pointer];
          s->bits = 0;
          s->phase = SEND_DATA;
        }
        else
        {
          ++s->master_nacks;
          s->phase = IGNORE;
        }
      }
      break;
    case IGNORE:
      break;
  }
}

static void falling_edge(Slave *s)
{
  if (!s->active) return;
  if (s->phase == SLAVE_ACK)
  {
    if (!s->ack_set)
    {
      s->slave_low = s->ack_value;
      s->ack_set = true;
    }
    else if (s->ack_clock_seen)
    {
      s->slave_low = false;
      s->phase = s->after_ack;
      s->bits = 0;
      s->value = 0;
    }
  }
  if (s->phase == SEND_DATA)
  {
    s->slave_low = ((s->read_value >> (7U - s->bits)) & 1U) == 0;
  }
  else if (s->phase == MASTER_ACK && !s->master_ack_ready)
  {
    s->slave_low = false;
    s->master_ack_ready = true;
  }
}

static void refresh_clock(Slave *s)
{
  const bool next = s->master_scl && !s->clock_stuck && s->us >= s->stretch_until;
  if (next != s->scl)
  {
    s->scl = next;
    if (next) rising_edge(s); else falling_edge(s);
  }
}

static void set_scl(void *context, bool released)
{
  Slave *s = context;
  if (released && !s->master_scl && s->stretch_us != 0 && !s->stretch_used)
  {
    s->stretch_used = true;
    s->stretch_until = s->us + s->stretch_us;
  }
  s->master_scl = released;
  refresh_clock(s);
}

static void set_sda(void *context, bool released)
{
  Slave *s = context;
  const bool previous = line_sda(s);
  s->master_sda = released;
  const bool next = line_sda(s);
  if (s->scl && next != previous)
  {
    if (next) stop_condition(s); else start_condition(s);
  }
}

static bool read_scl(void *context)
{
  Slave *s = context;
  refresh_clock(s);
  return s->scl;
}

static bool read_sda(void *context) { return line_sda(context); }

static void delay_us(void *context, uint32_t us)
{
  Slave *s = context;
  CHECK(us > 0 && us <= 1000U);
  s->us += us;
  CHECK(s->us < 1000000U); /* Also fails a runaway clock-stretch loop. */
  refresh_clock(s);
}

static uint32_t get_tick(void *context)
{
  const Slave *s = context;
  return s->base_ms + (s->frozen_tick ? 0U : (uint32_t)(s->us / 1000U));
}

static SoftI2CBus setup(Slave *s)
{
  memset(s, 0, sizeof *s);
  s->master_scl = s->master_sda = s->scl = true;
  s->memory[0x75] = 0x68;
  return (SoftI2CBus){.context = s, .set_scl = set_scl, .set_sda = set_sda,
      .read_scl = read_scl, .read_sda = read_sda,
      .delay_us = delay_us, .get_tick = get_tick};
}

static void check_released(const Slave *s)
{
  CHECK(s->master_scl && s->master_sda);
}

static void test_identity_read(void)
{
  ++cases;
  Slave s;
  SoftI2CBus bus = setup(&s);
  uint8_t value = 0;
  CHECK(SoftI2C_ReadRegister(&bus, 0x68, 0x75, &value, 1) == SOFT_I2C_OK);
  CHECK(value == 0x68);
  CHECK(s.starts == 2 && s.repeated_starts == 1 && s.stops == 1);
  CHECK(s.addresses == 2 && s.address_bytes[0] == 0xD0 && s.address_bytes[1] == 0xD1);
  CHECK(s.registers == 1 && s.read_bytes == 1 && s.master_acks == 0 && s.master_nacks == 1);
  check_released(&s);
}

static void test_acceleration_burst(void)
{
  ++cases;
  Slave s;
  SoftI2CBus bus = setup(&s);
  const uint8_t expected[] = {0x80, 0x00, 0x12, 0x34, 0xFF, 0xFE};
  memcpy(&s.memory[0x3B], expected, sizeof expected);
  uint8_t result[6] = {0};
  CHECK(SoftI2C_ReadRegister(&bus, 0x68, 0x3B, result, sizeof result) == SOFT_I2C_OK);
  CHECK(memcmp(expected, result, sizeof expected) == 0);
  CHECK(s.read_bytes == 6 && s.master_acks == 5 && s.master_nacks == 1);
  CHECK(s.starts == 2 && s.stops == 1);
  CHECK(s.us < 2000U); /* Leaves ample space in a 20 ms control period. */
  check_released(&s);
}

static void test_register_write(void)
{
  ++cases;
  Slave s;
  SoftI2CBus bus = setup(&s);
  const uint8_t values[] = {0x00, 0x11, 0xA5};
  CHECK(SoftI2C_WriteRegister(&bus, 0x68, 0x6B, values, sizeof values) == SOFT_I2C_OK);
  CHECK(memcmp(&s.memory[0x6B], values, sizeof values) == 0);
  CHECK(s.addresses == 1 && s.address_bytes[0] == 0xD0);
  CHECK(s.registers == 1 && s.written_bytes == 3 && s.starts == 1 && s.stops == 1);
  check_released(&s);
}

static void test_nacks(void)
{
  ++cases;
  Slave s;
  SoftI2CBus bus = setup(&s);
  uint8_t value = 0xAA;
  s.nack_address = true;
  CHECK(SoftI2C_ReadRegister(&bus, 0x68, 0x75, &value, 1) == SOFT_I2C_NACK);
  CHECK(value == 0xAA && s.registers == 0 && s.stops == 1);
  check_released(&s);
  bus = setup(&s);
  s.nack_read_address = true;
  CHECK(SoftI2C_ReadRegister(&bus, 0x68, 0x75, &value, 1) == SOFT_I2C_NACK);
  CHECK(value == 0xAA && s.addresses == 2 && s.read_bytes == 0 && s.stops == 1);
  check_released(&s);
  bus = setup(&s);
  s.nack_register = true;
  CHECK(SoftI2C_ReadRegister(&bus, 0x68, 0x75, &value, 1) == SOFT_I2C_NACK);
  CHECK(s.registers == 1 && s.read_bytes == 0 && s.stops == 1);
  check_released(&s);
  bus = setup(&s);
  s.nack_data_index = 1;
  CHECK(SoftI2C_WriteRegister(&bus, 0x68, 0x6B, &value, 1) == SOFT_I2C_NACK);
  CHECK(s.memory[0x6B] == 0 && s.written_bytes == 1 && s.stops == 1);
  check_released(&s);
}

static void test_clock_stretch(void)
{
  ++cases;
  Slave s;
  SoftI2CBus bus = setup(&s);
  s.stretch_us = 500;
  uint8_t value = 0;
  CHECK(SoftI2C_ReadRegister(&bus, 0x68, 0x75, &value, 1) == SOFT_I2C_OK);
  CHECK(value == 0x68 && s.stretch_used && s.us >= 500U);
  check_released(&s);
  bus = setup(&s);
  s.stretch_us = 3000;
  CHECK(SoftI2C_ReadRegister(&bus, 0x68, 0x75, &value, 1) == SOFT_I2C_TIMEOUT);
  CHECK(s.us >= 2000U && s.us < 10000U);
  check_released(&s);
  /* The secondary poll limit must stop a transaction even if SysTick freezes. */
  bus = setup(&s);
  s.stretch_us = 3000;
  s.frozen_tick = true;
  CHECK(SoftI2C_ReadRegister(&bus, 0x68, 0x75, &value, 1) == SOFT_I2C_TIMEOUT);
  CHECK(s.us >= 2000U && s.us < 10000U);
  check_released(&s);
}

static void test_bus_recovery(void)
{
  ++cases;
  Slave s;
  SoftI2CBus bus = setup(&s);
  CHECK(SoftI2C_Recover(&bus) == SOFT_I2C_OK);
  check_released(&s);
  bus = setup(&s);
  s.data_stuck = true;
  s.release_after_pulses = 9;
  CHECK(SoftI2C_Recover(&bus) == SOFT_I2C_OK);
  CHECK(s.recovery_pulses == 9 && !s.data_stuck);
  check_released(&s);
  bus = setup(&s);
  s.data_stuck = true;
  CHECK(SoftI2C_Recover(&bus) == SOFT_I2C_BUS_STUCK);
  check_released(&s);
  bus = setup(&s);
  s.data_stuck = true;
  uint8_t value = 0xAA;
  CHECK(SoftI2C_ReadRegister(&bus, 0x68, 0x75, &value, 1) == SOFT_I2C_BUS_STUCK);
  CHECK(value == 0xAA && s.addresses == 0);
  check_released(&s);
  bus = setup(&s);
  s.clock_stuck = true;
  s.scl = false;
  CHECK(SoftI2C_Recover(&bus) == SOFT_I2C_TIMEOUT);
  CHECK(s.us >= 2000U && s.us < 10000U);
  check_released(&s);
}

static void test_time_wrap_and_invalid_arguments(void)
{
  ++cases;
  Slave s;
  SoftI2CBus bus = setup(&s);
  s.base_ms = UINT32_MAX - 1U;
  s.stretch_us = 3000;
  uint8_t value;
  CHECK(SoftI2C_ReadRegister(&bus, 0x68, 0x75, &value, 1) == SOFT_I2C_TIMEOUT);
  CHECK(s.us >= 2000U && s.us < 10000U);
  bus = setup(&s);
  CHECK(SoftI2C_Recover(NULL) == SOFT_I2C_INVALID_ARGUMENT);
  CHECK(SoftI2C_ReadRegister(&bus, 0x80, 0, &value, 1) == SOFT_I2C_INVALID_ARGUMENT);
  CHECK(SoftI2C_ReadRegister(&bus, 0x68, 0, NULL, 1) == SOFT_I2C_INVALID_ARGUMENT);
  CHECK(SoftI2C_WriteRegister(&bus, 0x68, 0, &value, 0) == SOFT_I2C_INVALID_ARGUMENT);
  bus.read_sda = NULL;
  CHECK(SoftI2C_Recover(&bus) == SOFT_I2C_INVALID_ARGUMENT);
  CHECK(s.starts == 0 && s.us == 0);
}

int main(void)
{
  test_identity_read();
  test_acceleration_burst();
  test_register_write();
  test_nacks();
  test_clock_stretch();
  test_bus_recovery();
  test_time_wrap_and_invalid_arguments();
  printf("PASS: %u software I2C cases, %u checks (edge-driven slave, ACK/NACK, burst, recovery, timeout)\n", cases, checks);
  return 0;
}
