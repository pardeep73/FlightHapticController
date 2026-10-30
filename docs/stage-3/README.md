# Stage 3 — peripheral observations

Status: steady PWM, LED output states and raw button signals verified from the
user-supplied capture. Screenshots support the expected neutral orientations;
exact angle calibration is unverified, and movement/direction checks remain
blocked by the sensor fault.

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
- `peripheral-results.csv` contains expected observations and measured outcomes.
- `checks.json` distinguishes completed static inspections from pending runtime
  measurements and the known sensor fault.
- `evidence/capture.vcd` preserves the uploaded waveform unchanged, and
  `measurements.json` records the measurement method, ranges and provenance.
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
This resolves the previous uncertainty about the label. Steady output timing is
now measured below; exact angle calibration and movement checks remain open.

## Observed results — 31 October 2026

The supplied VCD has a 1 ns timescale and a final timestamp of 29.7442084 seconds.
Its header date converts to 31 October 2026, 00:02:17 Asia/Calcutta. The agent
analyzed the user's recording rather than running Wokwi independently.

| Check | Actual observation | Status |
| --- | --- | --- |
| Roll PWM | Median period 20,000 us; median high 1,472 us; 1,486 complete high pulses | PASS, steady timing |
| Pitch PWM | Median period 20,000 us; median high 1,472 us; 1,486 complete high pulses | PASS, steady timing |
| Tilt PWM | Median steady period 20,000 us; median high 544 us; 1,487 complete steady high pulses | PASS, steady timing |
| Autopilot output | Low initially; high from 117.409125 ms onward | PASS, output signal |
| Manual output | High at 2.765275 ms; low from 117.417375 ms onward | PASS, output signal |
| Button signal | Five press/release pairs; stable holds approximately 148–264 ms | PASS, raw input signal |
| Servo visual positions | Screenshots are consistent with neutral orientations, but exact angles are not established | PARTIAL |
| Independent movement and direction | Not demonstrated by fixed neutral outputs | BLOCKED |
| Sensor bus | Startup transactions recorded; wake-write payload missing | RECORDED, sensor still failing |

Steady periods range from 19,999.975 to 20,000.025 us. Roll/pitch high durations
range from 1,471.975 to 1,472.025 us; steady tilt high durations range from
543.975 to 544.025 us. The median period corresponds to 50 Hz.

Two startup details are preserved rather than hidden in the steady results:

- The first tilt high pulse is 1,444 us, followed by the steady 544 us pulses.
  This is an observed startup transient; its exact cause has not been verified.
- Both LED output signals are high for 8.25 us during the manual-to-autopilot
  transition. Subsequent steady autopilot-high/manual-low behaviour is confirmed.

Ten contact-bounce episodes of approximately 2.04–2.64 ms separate the five
button press/release pairs. The shorter holds are sufficient for the raw signal
check; another recording solely to hold the button for one second is unnecessary.
These measurements do not establish software input sampling or debounce correctness.

Measurements use actual state changes, ignore redundant same-value assignments,
and count only high intervals with both a rising and falling edge. Steady PWM
statistics use complete pulses/rising edges starting at or after 20 ms; the first
tilt pulse is reported separately and unclosed final highs are excluded.

## Sensor-bus evidence from this recording

The startup clock/data trace decodes into these complete byte sequences:

1. Write-address byte D0 ACK, register 75 ACK, then a stall.
2. Write-address byte D0 ACK, register 75 ACK, then STOP.
3. Read-address byte D1 ACK, identity byte 68, then the master's normal ending
   NACK and STOP.
4. Write-address byte D0 ACK, wake register 6B ACK, but no complete value byte.

D0/D1 are the wire-format write/read address bytes for the sensor's 7-bit address
0x68. After the wake-register ACK, SCL remains low until 115.338425 ms. Both bus
signals are then high through the final timestamp, over 29.6 seconds later.
Consequently the console's later GPIO `lines=0/0` is not proof that the external
bus remains held low. This observation does not identify why GPIO readback differs.

The failed write is narrowed to the phase after register-address acknowledgement
and before a complete payload byte. A RELOAD/TCR-phase limitation is a hypothesis,
not a confirmed root cause; the following data-ready wait is another phase to
distinguish. A write-specific hardware snapshot before fallback is still needed.

The CSV uses PASS for the stated measured scope, PARTIAL for incomplete visual
calibration, BLOCKED for unavailable movement tests, and RECORDED for diagnostic
bus evidence that does not establish successful sensor initialization.

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

## Source expectations used for comparison

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

These expected numbers are derived from the source; measured results are above. Measure
several complete periods after startup, excluding partial initial pulses.
Wokwi's servo reference does not document its pulse-to-angle calibration, so
the screenshot must separately confirm the actual displayed positions. Correct
timing alone does not prove the displayed angle is correct.

The existing application initially selects manual before the sensor check and
then selects autopilot on sensor failure. A startup capture is expected to show
the manual output active first and the autopilot output active afterwards. The
actual transition times from the supplied capture are recorded above. This
describes the current code, not an assignment-mandated startup policy.

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

Source inspection and documentation lookup are not simulator measurements. The
actual observations above come from the user-supplied VCD and screenshots;
unverified scope remains explicitly partial or blocked in the records.
