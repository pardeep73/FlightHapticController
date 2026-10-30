# Stage 3 — peripheral observations

Status: observation setup prepared; runtime peripheral checks have not been run.

Stage 2 established a clean build and user-reported Wokwi execution. It also
established that sensor initialization fails while writing register 0x6B.
No valid acceleration or full controller behaviour has been demonstrated.

This stage adds a Wokwi logic analyzer to the existing circuit. A logic analyzer
records digital high/low signals over time. Its VCD output is a timestamped signal
record, which can establish pulse timing and button/LED signal changes.

No application C/header code or vendor code is changed. The existing compiled
firmware is retained with the Stage 2 hashes. This is a measurement setup, not
new controller implementation. The assignment's AI editing-only rule and the
existing application's disclosed AI assistance remain relevant; this pack does
not establish permission to submit AI-generated work.

## What is provided

- The root `diagram.json` has the same original components/connections plus an
  eight-channel logic analyzer and its recording connections.
- `wokwi.toml` saves the recording as `stage-3-capture.vcd` when simulation stops.
- `peripheral-results.csv` contains expected observations and empty result fields.
- `checks.json` distinguishes completed static inspections from pending runtime
  measurements and the known sensor fault.
- The project ZIP contains the existing ELF/HEX/BIN and source/support files.

## Recording channels

| Analyzer input | Name | Connection | Purpose |
| --- | --- | --- | --- |
| D0 | ROLL_PWM | PB4 | Roll pulse timing |
| D1 | PITCH_PWM | PB5 | Pitch pulse timing |
| D2 | TILT_PWM | PB0.1 | Tilt pulse timing |
| D3 | AUTO_LED | PA2 | Autopilot indicator output |
| D4 | MANUAL_LED | PB11 | Manual indicator output |
| D5 | BUTTON | PA12 | Released/pressed input signal |
| D6 | SCL | PB8 | Additional sensor-bus observation |
| D7 | SDA | PB9 | Additional sensor-bus observation |

Analyzer GND connects to the same ground as the board. These are observation
connections: they do not command the servos or bypass sensor validation.

Wokwi's official board definition explicitly maps `PB0.1` to MCU pin `PB0`.
This resolves the previous uncertainty about the label; actual output timing
and servo calibration still need measurements.

## Perform this first recording

1. Download and fully extract `assignment-stage-3.zip` into a fresh folder.
2. In VS Code, open the inner `FlightHapticController-stage-3` folder containing
   `wokwi.toml`, `diagram.json` and `build`.
3. Run **Wokwi: Start Simulator** from the Command Palette. The compiled image is
   included. Follow any extension account/license prompts if necessary.
4. Let simulation time advance to about 3 seconds so startup is included.
5. Press and hold the blue AUTOPILOT / MANUAL button for about 1 second, release
   it, then let the simulation continue for about 1 more second.
6. Capture a screenshot showing all three servo horns and both indicator LEDs.
   Preserve the first complete terminal status record and any extension error.
7. Use the square **Stop** control, rather than Pause. In VS Code the analyzer
   recording should be saved as `stage-3-capture.vcd` in the project root.
8. Supply that VCD file and the screenshot for analysis. If the file is missing,
   report that result and the extension message; do not assume a recording exists.

The VCD file is ignored by Git to avoid accidental large commits. Preserve your
copy separately as evidence; ignoring it does not mean it is unwanted.

## Predicted observations, not measured results

The current timer configuration requests a 48 MHz timer clock, prescaler 47 and
period register 19999. This predicts:

`48,000,000 / (47 + 1) / (19999 + 1) = 50 Hz`, or 20 ms per period.

The existing fault behaviour commands pitch/roll to 90° and tilt to 0°. Its
existing conversion function predicts the following steady-state pulse widths:

| Signal | Predicted period | Predicted high duration | Software angle command |
| --- | --- | --- | --- |
| Roll | 20,000 microseconds | 1,472 microseconds | 90° |
| Pitch | 20,000 microseconds | 1,472 microseconds | 90° |
| Tilt | 20,000 microseconds | 544 microseconds | 0° |

These numbers are derived from the source, not a captured waveform. Measure
several complete periods after startup, excluding partial initial pulses.
Wokwi's servo reference does not document its pulse-to-angle calibration, so
the screenshot must separately confirm the actual displayed positions. Correct
timing alone does not prove the displayed angle is correct.

The existing application initially selects manual before the sensor check and
then selects autopilot on sensor failure. A startup capture is expected to show
the manual output active first and the autopilot output active afterwards; the
exact transition time has not been measured. This describes the current code,
not an assignment-mandated startup policy.

After the reported fault, the red autopilot LED is expected to stay on and the
green manual LED to stay off. Pressing the button is not expected to release
autopilot while `IMU=FAULT`. Lack of a mode change therefore does not establish
that the button wiring is faulty. The BUTTON waveform should separately show
low when released, high while held, and low again after release. Short contact
bounce may be visible; software debounce is a later verification task.

## What remains blocked or separate

- These observations can check neutral pulse output, timing, LED signals and the
  button's electrical signal. They do not validate software input sampling or
  button debounce merely by showing a wire changing level.
- Servo travel and clockwise/anticlockwise behaviour cannot be proved from fixed
  neutral commands. They remain blocked with this faulted firmware until a
  permitted independent peripheral test or functioning sensor-driven test is
  available. Do not mark Stage 3 fully passed before these checks are supported.
- SCL/SDA channels provide additional evidence. Their appearance alone does not
  establish valid acceleration data or the precise cause of the wake-write timeout.
- The wake write still needs a current-operation hardware snapshot before GPIO
  fallback. Adding an analyzer does not itself fix or diagnose that timeout.

## Official references consulted

- Board definition, including PB0.1 alias:
  https://raw.githubusercontent.com/wokwi/wokwi-boards/main/boards/st-nucleo-c031c6/board.json
- Logic analyzer pins, channel names and recording:
  https://raw.githubusercontent.com/wokwi/wokwi-docs/main/docs/parts/wokwi-logic-analyzer.md
- VS Code VCD output configuration:
  https://raw.githubusercontent.com/wokwi/wokwi-docs/main/docs/vscode/project-config.md
- Servo pins and angle range; no pulse calibration table is supplied:
  https://raw.githubusercontent.com/wokwi/wokwi-docs/main/docs/parts/wokwi-servo.md

Source inspection and documentation lookup are not simulator measurements. All
unobserved results remain NOT_RUN or BLOCKED in the records.
