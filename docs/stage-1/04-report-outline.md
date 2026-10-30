# Stage 1 — report planning prompts

This AI-generated outline contains prompts, not assessed report prose or results.
The assignment permits AI editing only; permission to use AI-generated planning
materials is not established. Write the report from your own work and follow the
course's rules. Do not submit this outline as the final report.

Final deliverable: a separate student_id.pdf of 1,500–2,000 words.
Suggested body budget: approximately 1,800 words. These allocations are optional;
word-count exclusions were not specified, so check the actual counting method.

## Identification

- Student name: TO FILL.
- Student ID: TO FILL.
- Final project/build identifier: TO FILL AFTER IMPLEMENTATION.
- Report word count and counting method: TO FILL.
- Additional course instructions/rubric available: TO RECORD.

## 1. Purpose and scope — about 100 words

- What does the assignment require the simulator to demonstrate?
- What do the servos represent?
- What assumptions have you made, and where are they disclosed?

Related requirements: REQ-01, REQ-07, REQ-08.

## 2. Hardware and relevant configuration — about 250 words

- Which board, pins and peripherals does your actual project use?
- Which register configurations are important, and why?
- How did you verify PWM timing, input configuration and sensor communication?
- Which course examples or other permitted sources did you actually reuse?

Related requirements: REQ-02, REQ-03, REQ-04, REQ-21, REQ-23, REQ-27.

## 3. Measurement and angle-calculation subroutine — about 150 words

- What acceleration data enters your subroutine?
- How does your implementation relate to the three supplied equations?
- How do you handle units and check signs?
- What independent checks support the calculation's correctness?

Related requirements: REQ-05, REQ-06, REQ-27.

## 4. FSM design and connection to code — about 600 words

- Insert your own software-created FSM diagrams after designing the system.
- Show each state's actions and each transition's trigger.
- Explain the return deadband and your documented boundary interpretations.
- Explain how the diagrams correspond to the final code.
- Explain operating-mode changes and their input conditions.

No completed diagram, transition table or implementation is supplied here.

Related requirements: REQ-07–REQ-20, REQ-27, REQ-28, REQ-33.

## 5. Verification and results — about 500 words

- Reference the actual final build used for the observations.
- Document acceleration triples, vector magnitudes and expected versus observed results.
- Cover compliant and clamped entry for individual axes and combined tilt.
- Include the required roll-only, pitch-only and tilt-only compliant cases.
- Disclose the A-13 interpretation of "roll only" and "pitch only"; combined tilt
  may also respond. Do not claim that all other outputs remain free in those cases.
- Include return-transition and operating-mode evidence.
- Relate evidence to the diagrams and the matching video.
- Explain failures and corrections honestly; mark anything still unverified.

Result table fields to fill after testing:

| Test ID | Acceleration triple and magnitude | Independently expected result | Observed result | Evidence reference |
| --- | --- | --- | --- | --- |
| TO FILL | TO FILL | TO FILL | TO FILL | TO FILL |

Related requirements: REQ-14, REQ-15, REQ-17–REQ-20, REQ-29–REQ-32.

## 6. Limitations and conclusion — about 200 words

- What is verified by your evidence, and what remains uncertain?
- How did simulator precision affect the documented cases?
- Which assumptions remain interpretations rather than confirmed requirements?
- What conclusions are supported by the actual results?

## References and optional appendices

- Acknowledge reused course code here and in source comments.
- Cite the official documentation actually used; do not invent references.
- A full code listing is not required in the report.
- If included, major code blocks belong in appendices as formatted text, not screenshots.
- A flowchart is not required. All diagrams must be software-created.

Related requirements: REQ-22, REQ-23, REQ-33, REQ-34.

## Final report and video review

- [ ] Report is 1,500–2,000 words using a disclosed counting method.
- [ ] Diagrams show actions and triggers and match the actual code.
- [ ] Results come from observed tests of the identified build.
- [ ] Documented acceleration magnitudes satisfy the approximately 1g requirement.
- [ ] Required isolated compliant cases are present.
- [ ] Recording shows actual Wokwi operation and agrees with the report.
- [ ] Name and ID are visible in the Wokwi diagram.
- [ ] Acknowledgements match the actual sources used.
- [ ] No hand-drawn diagrams or code screenshots appear.
- [ ] Final PDF is separate from the ZIP containing runnable files and video.

All boxes intentionally remain unchecked until the work is completed and reviewed.
