# Stage 4 — capture the failed sensor write

Status: diagnostic setup prepared; the live write-state capture and sensor fix
are not yet verified. Stage 3's exact servo calibration and movement/direction
checks remain open.

## Goal and current evidence

The uploaded Stage 3 recording proves the MPU6050 acknowledges the device
address and register 0x6B, but no complete value byte follows. The separate
identity read returns 0x68. Both sensor-bus wires later remain high even though
software GPIO recovery reports low readings.

The vendor memory-write function waits for transfer-complete-reload (TCR) after
the register-address byte, then requests the next transmit byte (TXIS). The
recording does not distinguish which wait failed. We must capture that state
before choosing a driver change.

The existing firmware already records the current wait flag and register values
in memory. Its terminal preserves only the first failure. A debugger can pause
the program after the wake write returns and inspect the current values before
GPIO fallback replaces the displayed error. No new C code is needed for this
diagnostic, and the supplied ELF/HEX/BIN are unchanged.

The assignment's AI editing-only restriction and the existing application's
disclosed AI assistance remain applicable. These diagnostic tooling files do not
establish permission to submit AI-generated work.

## What is included

- `wokwi.toml`: enables the local debugger on port 3333 and names an optional
  waveform recording `stage-4-capture.vcd`.
- `.vscode/tasks.json`: adds **Capture Stage 4 wake write**, which runs GDB using
  the command file below. The usual build task is retained.
- `docs/stage-4/capture-write.gdb`: debugger commands that capture existing
  variables and leave the CPU paused. The script calls no HAL functions and
  contains no target-memory or register assignment commands.
- `docs/stage-4/checks.json`: separates completed inspections from pending live
  debugging and communication tests.
- `docs/stage-4/offline-gdb-validation.txt`: actual offline GDB validation output;
  no live sensor register values are claimed by this transcript.
- `downloads/assignment-stage-4.zip`: complete project/support files and the
  unchanged compiled firmware for this diagnostic.

## Prerequisite: an ARM-capable GDB executable

GDB is a debugger: it can stop the simulated CPU and inspect its state.
On Windows, use `arm-none-eabi-gdb.exe` from an ARM GNU/STM32 toolchain. It must
run on the same computer/VS Code host as Wokwi. The compiler alone is not proof
that this debugger is installed.

If it is on PATH, the task can use the name `arm-none-eabi-gdb`. Otherwise enter
its full executable path when prompted. A compatible `gdb-multiarch` is another
option on Linux. A general x86-only debugger is not sufficient.
Enter an executable path without surrounding quote characters in the task prompt.

Official ARM GNU toolchain downloads:
https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads

No Microsoft C++ debug extension is needed for this command-line task; the Wokwi
extension and an ARM-capable GDB are needed. Tool availability on the user's PC
has not been established by the cloud checks.
The user confirmed an ARM/STM32 toolchain is installed; the exact GDB executable
location and version on that computer still need the task's normal launch check.

## Checks completed before packaging

GDB 16.3 loaded the existing ARM ELF, accepted the conditional breakpoint and
the capture command definitions, and type-checked all 18 printed field
expressions. The offline command-file validation exited 0 with no stderr.
It used a temporary copy with the remote connection and `continue` disabled;
no target was connected and no register values were captured. The live script's
actions still require a user-side Wokwi run.

## Run one capture

1. Download and fully extract `assignment-stage-4.zip` into a fresh directory.
2. Open the inner `FlightHapticController-stage-4` folder in VS Code. Use the
   included firmware so the debugger and simulator load the same ELF.
3. Stop any previous Wokwi session. From the Command Palette run exactly
   **Wokwi: Start Simulator and Wait for Debugger**. Paused startup is expected.
4. Run **Tasks: Run Task**, then **Capture Stage 4 wake write**.
5. Enter the GDB executable name or full path when prompted. The task connects,
   sets the conditional breakpoint, runs the program and pauses after the wake
   write returns.
6. Look for a record beginning **STAGE4 op=WRITE reg=0x6b** in the task terminal
   and for `stage-4-write.txt` in the project root. Send that file or its complete
   text for diagnosis.
7. After preserving the record, stop Wokwi and enter `quit` at the GDB prompt.

The program pauses during sensor initialization, so the ordinary Wokwi terminal
may not yet show its repeating IMU report. This is expected for this capture.
Starting a normal run and attaching after GPIO fallback is too late: hardware
write retries may not occur once software transport is selected. Start a new
paused session for every attempt. Each capture replaces `stage-4-write.txt`;
preserve earlier captures separately if needed.

If the task cannot find GDB, cannot connect, or produces no STAGE4 record, preserve
the exact error. Do not mark the diagnostic as passed or change sensor code.

## Equivalent terminal command

Run this from the project root after starting Wokwi in wait-for-debugger mode,
using the GDB executable available on your machine:

```sh
arm-none-eabi-gdb --quiet --nx --command=docs/stage-4/capture-write.gdb build/FlightHapticController.elf
```

## Captured fields and interpretation limits

| Field | Meaning |
| --- | --- |
| STAGE4 HAL | Actual return from the just-completed register write |
| STAGE4 err | Current error in the I2C handle, before GPIO fallback |
| TRACE flag/ISR/CR2 | Last traced hardware flag and register values for this operation |
| LIVE registers | Peripheral register values at the breakpoint |
| XFER | Remaining byte count and HAL transfer state |
| CONTEXT | Identity/read-path context and whether GPIO transport was already selected |
| Backtrace | Call stack confirming the capture location |

At this breakpoint `haptic_app_status.i2c_error` can still contain an older
result, so the script reads `sensor_i2c->ErrorCode` instead. It does not call
functions inside the simulated target to collect data.
Breakpoints and pausing can affect execution timing. Preserve the actual result
if the write behaves differently while debugging; do not force an expected error.

For a pure timeout, a traced TCR flag (0x80) would support the reload-wait
hypothesis. TXIS (0x02) would instead point to the next transmit-data wait.
Other outcomes must be interpreted from the actual record. These are possible
interpretations, not predicted successful results or a confirmed fix.

## Completion gates

- Capture the actual wake-write result and wait state before fallback.
- Establish the failure phase from that evidence.
- Make only a permitted, justified implementation change if one is needed.
- Demonstrate successful wake/configuration writes and real acceleration reads.
- Verify repeated sensor reads respond to simulator inputs at the planned rate.

Stage 4 is not complete merely because this debugger setup is prepared. Sensor
success must be demonstrated by actual measurements. Angle/FSM checks are later
stages, and the blocked servo checks must also be revisited.

## Official references and inspected source

- Wokwi debugger workflow:
  https://raw.githubusercontent.com/wokwi/wokwi-docs/main/docs/vscode/debugging.md
- Debug-server port configuration:
  https://raw.githubusercontent.com/wokwi/wokwi-docs/main/docs/vscode/project-config.md
- Existing vendor source:
  `Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_i2c.c`, functions
  `HAL_I2C_Mem_Write` and `I2C_RequestMemoryWrite`.
- Existing application source:
  `Core/Src/main.c`, functions `Sensor_Write` and `EndHardwareTrace`.
