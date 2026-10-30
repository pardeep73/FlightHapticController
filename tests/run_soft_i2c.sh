#!/usr/bin/env sh
# Exercise the production main.c bus protocol with an edge-driven slave model.
set -eu
repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_build_dir=$(mktemp -d "${TMPDIR:-/tmp}/flight-soft-i2c-tests.XXXXXX")
trap 'rm -rf "$test_build_dir"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic \
  -DHAPTIC_TEST_SOFT_I2C -I"$repo_dir/Core/Inc" \
  "$repo_dir/tests/test_soft_i2c.c" "$repo_dir/Core/Src/main.c" \
  -o "$test_build_dir/test_soft_i2c"
"$test_build_dir/test_soft_i2c"
