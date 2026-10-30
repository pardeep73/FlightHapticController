#!/usr/bin/env sh
# Compile each production main.c section against independent host regressions.
# Test flags select sections; all application implementation remains in main.c.
set -eu
repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_build_dir=$(mktemp -d "${TMPDIR:-/tmp}/flight-haptic-tests.XXXXXX")
trap 'rm -rf "$test_build_dir"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic \
  -DHAPTIC_TEST_CONTROLLER -I"$repo_dir/Core/Inc" \
  "$repo_dir/tests/test_haptic_controller.c" "$repo_dir/Core/Src/main.c" \
  -lm -o "$test_build_dir/test_haptic_controller"
"$test_build_dir/test_haptic_controller"
# Adapter calls real controller code while HAL/software-bus doubles supply IO.
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic \
  -DHAPTIC_TEST_APP -I"$repo_dir/Core/Inc" \
  -include "$repo_dir/tests/mock/main.h" \
  "$repo_dir/tests/test_haptic_app.c" "$repo_dir/Core/Src/main.c" \
  -lm -o "$test_build_dir/test_haptic_app"
"$test_build_dir/test_haptic_app"
# USART register doubles expose actual serial output and transmitter stalls.
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic \
  -DHAPTIC_TEST_DIAGNOSTICS -I"$repo_dir/Core/Inc" \
  -include "$repo_dir/tests/mock/diagnostics_hal.h" \
  "$repo_dir/tests/test_haptic_diagnostics.c" "$repo_dir/Core/Src/main.c" \
  -o "$test_build_dir/test_haptic_diagnostics"
"$test_build_dir/test_haptic_diagnostics"
# Exercise bit-level bus transactions against an edge-driven slave model.
"$repo_dir/tests/run_soft_i2c.sh"
# Run the real GPIO platform adapter with controlled SysTick and line inputs.
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic \
  -DHAPTIC_TEST_GPIO_I2C -I"$repo_dir/Core/Inc" \
  -include "$repo_dir/tests/mock/gpio_hal.h" \
  "$repo_dir/tests/test_haptic_gpio_i2c.c" "$repo_dir/Core/Src/main.c" \
  -o "$test_build_dir/test_haptic_gpio_i2c"
"$test_build_dir/test_haptic_gpio_i2c"
