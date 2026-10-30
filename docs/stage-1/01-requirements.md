# Stage 1 — requirements and progress

Source: ENGE_X200_MCP_Assignment_2.pdf, all five pages, supplied by the student.
Prepared: 30 October 2026. This pack starts a fresh requirements baseline; it does
not certify any existing code or previous results.

## Purpose and academic restriction

These are AI-generated planning aids, not a completed report or implementation.
Page 4 permits AI for editing such as spelling and grammar only and prohibits
AI-generated code. Permission to use AI for planning or design is not established
by the PDF. Keep this provenance visible and follow the course's rules.

## How to use these four files

1. Read this checklist alongside the assignment.
2. Read `02-assumptions.md`: its proposals are not assignment requirements.
3. Open `03-verification.csv` in a spreadsheet or text editor. All checks start
   `NOT_RUN`; record evidence before changing a result.
4. Use `04-report-outline.md` as a set of writing prompts after doing the work.

The requirement IDs below are referenced by the verification log. `Required`
means the brief expressly calls for it. `Should` preserves the PDF's wording.
`External` means information is referenced but was not supplied.
Every implementation and submission requirement is currently unverified.

## Keeping the verification log honest

- Define the specific inputs, preconditions, independent expected result and build
  identifier before running a test. Planning rows are not completed test cases.
- Duplicate a scenario row for each input, axis or direction needed; use unique
  IDs such as `V-008-a`. One passing sample does not complete an entire scenario.
- Record acceleration in g, angles in degrees, and servo positions in degrees.
  Label the order of any grouped values, for example `pitch=...; roll=...; tilt=...`.
- For runtime cases, record actual outputs and a log, screenshot or video reference.
  For inspection/admin cases, identify the reviewed artifact or authoritative record.
- Status values: `NOT_RUN`, `PASS`, `FAIL`, `BLOCKED`, or `N/A`. Use `PASS` only
  after observing the result; explain `BLOCKED` or `N/A` in the notes. N/A cannot
  waive a mandatory requirement; it can cover a genuinely conditional check.
- Leave fields blank when no measurement exists. Do not replace absent data with
  zero, an expected value or a fabricated result.
- V-043 is an engineering robustness check, not a separately stated assignment
  requirement. Engineering checks must not be presented as additional course rules.

## Requirements checklist

| ID | Type | Requirement | PDF page | Planned evidence |
| --- | --- | --- | --- | --- |
| REQ-01 | Required | Target Nucleo C031C6 / STM32C031C6 and demonstrate in Wokwi, with a project executable through VS Code. | 1–3 | Project configuration and observed simulator execution. |
| REQ-02 | Required | Use the pin assignments in the hardware table below. | 3 | Wiring and configuration inspection, then peripheral checks. |
| REQ-03 | Required | Drive all three servo channels with 50 Hz PWM. | 3 | Configuration review and a timing observation where available. |
| REQ-04 | Required | Obtain the acceleration components from an MPU6050 accelerometer. | 2 | Successful sensor reads responding to changed simulator inputs. |
| REQ-05 | Required | Calculate pitch, roll and combined tilt using the supplied equations. | 2 | Comparison with independently calculated reference angles. |
| REQ-06 | Required | Have a subroutine that calculates those angles. | 2 | Source inspection and calculation checks. |
| REQ-07 | Required | Enforce the specified pitch and roll regions independently. | 1 | Positive/negative input tests through each region. |
| REQ-08 | Required | Enforce the specified combined-tilt regions. | 1–2 | Tests through each tilt region. |
| REQ-09 | Required | In free motion, pitch/roll servos stay at 90° and tilt stays at 0°. | 2 | Observed positions with free-region inputs. |
| REQ-10 | Required | Pitch/roll servos track the commanded axis angle one-to-one in the compliant region. | 2 | Expected versus observed servo movement. |
| REQ-11 | Required | Clamp the outputs at the stated impenetrable limits. | 1–2 | Inputs beyond each limit and observed limited outputs. |
| REQ-12 | Required | Treat clockwise movement relative to Figure 1 as negative. | 2–3 | Direction checks against the actual simulator orientation. |
| REQ-13 | Required | In the compliant region, tilt servo commands use twice the detected tilt angle; free motion remains at 0° and the clamp limit is 140°. | 2 | Observed scaling, neutral and clamp positions. |
| REQ-14 | Required | Apply a 5° return deadband from clamped to compliant. | 2 | Increasing and decreasing input sequences. |
| REQ-15 | Required | Apply a 5° return deadband from compliant to free motion. | 2 | Increasing and decreasing input sequences. |
| REQ-16 | Required | Implement a finite state machine matching the required behaviour. | 3 | Student-created diagrams, source and transition evidence. |
| REQ-17 | Required | Automatically activate autopilot when pitch and roll are simultaneously in their impenetrable regions. | 2 | Both-axis and single-axis comparison tests. |
| REQ-18 | Required | Leave autopilot only with a button press and both pitch and roll in their free-motion range. | 2 | Permitted and blocked transition tests. |
| REQ-19 | Required | A button press in manual mode returns the system to autopilot. | 2 | Observed manual-to-autopilot transition. |
| REQ-20 | Required | Separate LEDs indicate autopilot and manual modes. | 2–3 | Mode and indicator observations. |
| REQ-21 | Should | Poll the MPU6050 through I²C at the same rate as servo updates. | 3 | Measured or otherwise justified update timing. |
| REQ-22 | Required | Fully comment all self-written code. | 4 | Source review. |
| REQ-23 | Required | Acknowledge reused lecture/practical code in both report and source comments. | 4 | Attribution review against actual reused sources. |
| REQ-24 | Required | Add the student's name and ID visibly through Wokwi diagram.json. | 4 | File inspection and simulator screenshot/video. |
| REQ-25 | Required | Supply every file needed to execute the program through VS Code and Wokwi. | 1, 4 | Successful run from a fresh extraction of the submitted project. |
| REQ-26 | Required | Submit a 1,500–2,000-word report. | 1, 4 | Final PDF and recorded word-count method. |
| REQ-27 | Required | Explain code logic, custom subroutines, FSM implementation and relevant register configurations. | 4 | Report review against the final implementation. |
| REQ-28 | Required | Make FSM diagrams a primary report feature, showing state actions, transition triggers, links to code and supporting results. | 4 | Diagram/report/code consistency review. |
| REQ-29 | Required | Document acceleration triples demonstrating compliant and impenetrable entry for individual axes and combined tilt. | 4 | Input/result tables tied to actual observations. |
| REQ-30 | Required | Documented acceleration cases have magnitude approximately 1g; reasonable Wokwi precision error is allowed. | 4 | Calculated magnitude and disclosed deviation for every documented case. |
| REQ-31 | Required | At minimum show compliant feedback for "roll only", "pitch only", and tilt only while pitch and roll remain free. See A-13 for the individual-axis wording ambiguity. | 4 | Three distinct documented demonstrations, with the interpretation disclosed. |
| REQ-32 | Required | Include a screen recording of the project working in Wokwi, matching report results and FSM transitions. | 4 | Final-build video reviewed against report and diagrams. |
| REQ-33 | Required | Create all diagrams using software; no hand-drawn FSMs. | 4 | Diagram source/export review. |
| REQ-34 | Conditional | If code appears in the report, format it as text, not screenshots; put major code blocks in appendices. | 4 | PDF inspection, or justified N/A if no code is included. |
| REQ-35 | Required | Upload student_id.zip containing the runnable code/project files and results video, using the actual student ID. | 1 | Final ZIP contents and filename review. |
| REQ-36 | Required | Upload the report separately as student_id.pdf, using the actual student ID. | 1 | Separate final PDF and filename review. |
| REQ-37 | Required | Submit through myLearning by Tuesday 3 November 2026, 11:59 PM. Timezone is not stated. | 1 | Authoritative deadline check and submission receipt. |
| REQ-38 | Required rule | AI use is limited to editing such as spelling and grammar; generating code with AI is prohibited. | 4 | Process/provenance review against course policy. |
| REQ-39 | External | The detailed marking rubric is on myLearning and was not included in the supplied PDF. | 4 | Rubric comparison if it becomes available; otherwise record the limitation. |

## Fixed hardware mapping

| Signal | Pin | Configuration stated in the assignment |
| --- | --- | --- |
| Roll servo | PB4 | PWM, 50 Hz |
| Pitch servo | PB5 | PWM, 50 Hz |
| Tilt servo | PB0 | PWM, 50 Hz |
| Autopilot LED | PA2 | Output |
| Manual LED | PB11 | Output |
| MPU6050 SCL | PB8 | I²C clock |
| MPU6050 SDA | PB9 | I²C data |
| Autopilot button | PA12 | Input with pull-down |

## Equations and limits copied from the brief

All feedback thresholds are in degrees. Angle calculations must use consistent
units; the implementation and its unit handling have not been created here.

- Roll: φ = atan2(ay, az).
- Pitch: θ = atan2(-ax, sqrt(ay² + az²)).
- Combined tilt: α = sqrt(θ² + φ²).

| Quantity | Free | Compliant | Impenetrable |
| --- | --- | --- | --- |
| Pitch or roll γ | -30° < γ < 30° | 30° ≤ γ < 50°, or -50° < γ ≤ -30° | γ > 50°, or γ < -50°; clamp at the respective ±50° limit |
| Combined tilt α | 0° ≤ α < 20° | 20° < α < 70° | α ≥ 70° |

The written inequalities omit exactly ±50° for pitch/roll and exactly 20° for
tilt. See the explicitly provisional entries in `02-assumptions.md`.
The 5° deadband applies to both kinds of return transition; the brief's explicit
example is pitch returning from clamped to compliant at ±45°.

## Recommendations and exclusions

- Multiple FSMs and reusing one axis FSM are suggested, not compulsory architecture.
- Lecture 8 I²C material and Lecture 9 mathematics examples are recommended.
- Building one axis first is recommended.
- A flowchart and a full code listing in the report are not required.
- No particular source-file count, video duration, numerical 1g tolerance or
  mandatory LED colours are specified in the PDF.
- Slider movement in the video need not be mechanically realistic. This does not
  remove the 1g requirement for documented report test cases.

## Stage 1 review and expectations

- [x] All five assignment pages reviewed.
- [x] Requirements distinguished from recommendations and assumptions.
- [x] A verification record and report outline prepared.
- [ ] Student has reviewed the checklist against their copy of the brief.
- [ ] Student name and ID supplied for the eventual simulator and submission.
- [ ] Additional rubric/lecturer information checked, if available.
- [ ] Provisional choices reviewed before the stages that depend on them.

Expected result at this stage: four readable planning files, with all verification
records `NOT_RUN` and no claimed implementation results. No simulator output is
required to validate Stage 1. Building and executing firmware belongs to Stage 2.

Stage 2 acceptance target: a clean project build and observed execution on the
specified Wokwi board. Existing project files, if any, do not establish that this
fresh baseline has passed that target.
