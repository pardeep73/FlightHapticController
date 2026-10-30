# Stage 2 — development environment and clean build

Status: clean build passed; user-supplied Wokwi output confirms firmware execution.
Sensor initialization fails and remains an open defect for the peripheral/sensor
verification stages. Stage 2 execution success does not establish sensor success.

This stage checks the build environment and simulator configuration. The repository
already contained a full controller before this fresh staged workflow. This stage
builds that existing application; it does not provide a newly written minimal
application or verify the controller's features. No application C/header files or
vendor sources were changed during Stage 2.

The assignment permits AI editing only and prohibits AI-generated code. The
existing README records AI assistance in the existing application. These checks
and packaging do not establish that the application is permitted for submission.

## Files prepared in this stage

- `.vscode/tasks.json`: the default build task explicitly runs `make -j4 all`.
- `.vscode/extensions.json`: recommends the Wokwi extension; it does not install it.
- `docs/stage-2/build.log`: actual output of the first clean build.
- `docs/stage-2/checks.json`: recorded build/configuration checks and firmware hashes.
- `docs/stage-2/wokwi-observation.txt`: first two complete user-supplied terminal
  records, with their provenance and excerpt limits.
- `downloads/assignment-stage-2.zip`: project sources, configuration, these records
  and freshly compiled `build/FlightHapticController.elf`, `.hex` and `.bin`.

The ZIP excludes Git history, old downloads, old Windows `Debug` outputs and the
cloud compiler installation. It is a development package, not the final student-ID
submission ZIP; no results video or completed report is included.

## Observed build results

The initial build used a fresh temporary source directory with no existing `build`
directory or object files. The command was `make -j4 all`.

| Item | Observed result |
| --- | --- |
| Compiler | ARM GNU GCC 14.2.1, Debian package 15:14.2.rel1-1 |
| Build tool | GNU Make 4.4.1 |
| Compiler/linker exit code | 0 |
| Flash allocated | 28,924 / 32,768 bytes |
| RAM statically allocated/reserved by linker | 3,032 / 12,288 bytes |
| Compiler errors | 0 |
| Compiler warnings | 3 unused-parameter warnings in ST HAL sources |
| Output files | ELF, HEX and BIN generated |

The warnings concern `Bank` in `stm32c0xx_hal_flash_ex.c` and `Regulator` in two
functions in `stm32c0xx_hal_pwr.c`. They remain visible in the log; vendor code
was not changed to hide them. Static RAM usage does not measure runtime stack
growth or prove runtime memory safety.

The ELF is an ARM/Thumb executable with the expected Cortex-M0+ compatible
instruction-set attributes. Its vector table and entry address were checked.
These are file/configuration checks, not observed CPU execution.
The existing tilt wire uses the diagram label `PB0.1`. Configuration parsing does
not establish that this label maps correctly in the simulator; verify the board
pin model and actual output during the peripheral stage before declaring that
connection correct. No wiring was changed on the basis of that assumption.

## User-reported Wokwi observation, 30 October 2026

The user supplied repeated firmware diagnostic records from Wokwi. This proves
the application reached its reporting/recovery loop. The running image's hash and
execution through the VS Code build task were not independently checked.

| Observation | Supported conclusion |
| --- | --- |
| Repeated firmware-generated terminal records | Firmware execution confirmed by user-supplied evidence. |
| First record has `who=0x68`, `split=4`, `serr=00000000` | Identity read succeeded through the separate pointer-write/read path, and initialization advanced beyond identity checking. |
| First record reports `sensor wake write failed`, register `0x6B` | Initialization next failed while writing the power-management register. |
| `bus=GPIO`, `hwerr=0x00000020`, failed recovery | The hardware wake write timed out and GPIO fallback initialization subsequently failed. |
| Later records report `I2C recovery failed` | Recovery remains unsuccessful; retained identity/split fields do not prove a new read on every retry. |

The hardware trace is deliberately first-failure-only: `EndHardwareTrace` does
not replace an already captured snapshot. Therefore the displayed `HW flag=4`,
`ISR=00008043` and `CR2=020120D0` describe the earlier hardware read timeout, not
the later wake-write failure. The final displayed HAL/error values have also
been replaced by the GPIO recovery result. They do not identify the original
write's exact HAL return or awaited flag.

The exact cause of the wake-write timeout is unconfirmed. The next useful
diagnostic is a separate snapshot for the write to `0x6B`, captured before
fallback, including the original HAL return/error, awaited flag, ISR and CR2.
The low GPIO input readings alone do not establish a physical wiring fault.
No sensor code was changed based on an unmeasured hypothesis.

## Open and run the included image in VS Code

1. Download `assignment-stage-2.zip` from GitHub and extract it completely into a
   fresh directory. Do not run files from inside the ZIP.
2. In VS Code, select **File → Open Folder** and choose the inner
   `FlightHapticController-stage-2` folder. This folder must directly contain
   `wokwi.toml`, `diagram.json`, `Makefile` and `build`.
3. Open Extensions, find and install **Wokwi Simulator** (extension ID
   `wokwi.wokwi-vscode`). Follow any account/license prompts shown by the extension.
4. The freshly compiled ELF is included, so you can observe it without installing
   a compiler first. To compile source yourself, use the prerequisites below.
5. Open the Command Palette with **Ctrl+Shift+P** and run **Wokwi: Start Simulator**.
6. Observe whether the circuit opens, simulation time advances and the Wokwi
   Terminal receives diagnostic messages. Let it run long enough to collect the
   first complete status record.
7. Save the exact terminal text and a screenshot showing the running simulator.
   Include any extension error verbatim if startup fails.

## Expected observations and how to interpret them

- Seeing the circuit alone proves the diagram loaded; it does not prove that the
  firmware executed. Advancing simulation time together with firmware diagnostics
  provides stronger execution evidence.
- The existing firmware's status record begins with `mode=` and includes either
  `IMU=OK` or `IMU=FAULT`. These are formats found in the source, not fabricated
  successful output from a simulator run.
- `IMU=FAULT` is a real sensor/communication failure to investigate. It can still
  demonstrate that the firmware reached its reporting loop, but it does not pass
  sensor verification or later controller stages.
- LED colour alone is insufficient to establish success. Preserve the diagnostic
  text instead of interpreting a red or green light as a complete test result.
- If there is no output, report that observation and any extension error. Do not
  assume success or change controller logic before locating the failing layer.

## Build the source yourself

Prerequisites: GNU Make and an ARM GNU C toolchain with newlib and the nano/nosys
spec files. The supplied linker script uses syntax documented for GCC 11 or later;
the build recorded here used GCC 14.2.1.

In this prepared cloud workspace, run from the project root:

```sh
make -j4 all
```

On another computer with `arm-none-eabi-gcc` and the other toolchain executables
on PATH, run in a compatible terminal:

```sh
make TOOLCHAIN_PATH= CROSS=arm-none-eabi- -j4 all
```

For a fresh rebuild, `make clean` removes this project's `build` directory; run the
appropriate build command again afterwards. The default VS Code build task uses
the prepared cloud location if present, otherwise ARM executables on PATH.

On Windows, install GNU Make as well as the ARM toolchain if you want to rebuild.
The existing `scripts/setup-toolchain.sh` is for the documented Debian cloud
environment, not a Windows installer. The old checked-in `Debug` files are not
used by the portable Makefile.

## Remaining verification

The cloud executor has no Wokwi CLI or VS Code executable, and no configured
`WOKWI_CLI_TOKEN`. Only the presence of the token was checked; no credentials were
read or disclosed. The agent cannot run Wokwi here; the execution result above
comes from the user's supplied terminal text.

The Stage 1 verification CSV remains a frozen planning baseline. Stage 2 results
are recorded separately in `checks.json`. The clean build and user-supplied
execution evidence now support the Stage 2 execution gate; the historical Stage 1
CSV has not been rewritten to imply later functionality was verified.

Stage 2's build/execution gate is complete, with an open sensor defect.
Peripheral behaviour, acceleration validity, angle maths, FSM behaviour and
assignment compliance require their later planned checks.
Student identity labels remain placeholders and must be filled before submission.
