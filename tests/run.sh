#!/usr/bin/env sh
# Build host regression tests in a temporary directory, without ARM hardware.
set -eu
repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_build_dir=$(mktemp -d "${TMPDIR:-/tmp}/flight-haptic-tests.XXXXXX")
trap 'rm -rf "$test_build_dir"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic \
  -I"$repo_dir/Core/Inc" \
  "$repo_dir/tests/test_haptic_controller.c" \
  "$repo_dir/Core/Src/haptic_controller.c" \
  -lm -o "$test_build_dir/test_haptic_controller"
"$test_build_dir/test_haptic_controller"
# Preinclude only the mock HAL contract; compile the real adapter unchanged.
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic \
  -I"$repo_dir/Core/Inc" -include "$repo_dir/tests/mock/main.h" \
  "$repo_dir/tests/test_haptic_app.c" \
  "$repo_dir/Core/Src/haptic_app.c" \
  "$repo_dir/Core/Src/haptic_controller.c" \
  -lm -o "$test_build_dir/test_haptic_app"
"$test_build_dir/test_haptic_app"
# Exercise serial output against USART/GPIO register doubles, including stalls.
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic \
  -I"$repo_dir/Core/Inc" -include "$repo_dir/tests/mock/diagnostics_hal.h" \
  "$repo_dir/tests/test_haptic_diagnostics.c" \
  "$repo_dir/Core/Src/haptic_diagnostics.c" \
  -o "$test_build_dir/test_haptic_diagnostics"
"$test_build_dir/test_haptic_diagnostics"
