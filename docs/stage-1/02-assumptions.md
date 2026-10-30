# Stage 1 — assumptions and unresolved information

These entries are planning proposals, not quotations from the assignment or
verified implementation decisions. The user has asked us to proceed using
reasonable assumptions; this does not change the course's academic rules.
No state machine or controller implementation is supplied in this pack.

Status meanings:

- `PROVISIONAL`: a working interpretation to disclose and revisit if authoritative
  guidance differs. It is not a claim about the marker's intended interpretation.
- `DEFERRED`: decide and record before the named dependent stage.
- `EXTERNAL`: information is unavailable; do not invent it.

| ID | Gap in the supplied material | Proposed handling | Status | Review before |
| --- | --- | --- | --- | --- |
| A-01 | Exactly ±50° is excluded by the written pitch/roll inequalities, although the later deadband example refers to ±50° as the clamp-entry limit. | Provisionally assign these exact endpoints to the clamped region. Record this interpretation in the student's design notes. | PROVISIONAL | Stage 6, single-axis design |
| A-02 | Exactly 20° is excluded by the written tilt inequalities. | Provisionally assign exactly 20° to the compliant region so the regions have no gap. | PROVISIONAL | Stage 7, combined tilt |
| A-03 | The equality comparisons on return thresholds are incompletely specified. | Provisionally allow a return when the 5° margin has been reached, including equality. Record the actual choice and verify immediately around it. | PROVISIONAL | Stage 6, single-axis design |
| A-04 | The startup mode is not specified. | Propose autopilot at startup as a conservative mode-interlock choice. This is not proof that the brief requires it or a statement about real aircraft safety. | PROVISIONAL | Stage 8, operating modes |
| A-05 | Servo actions specifically during autopilot are not defined. | Record the student's interpretation before mode integration. Do not invent an aircraft dynamics model or stabilization requirement. Required haptic behaviour must remain demonstrable. | DEFERRED | Stage 8, operating modes |
| A-06 | Autopilot exit refers to angles in the free-motion range without resolving how that relates to states retained by the deadband. | Explicitly document whether the exit check uses numerical bounds, retained axis states, or both, and verify the chosen interpretation. | DEFERRED | Stage 8, operating modes |
| A-07 | No numerical tolerance around 1g is supplied. | Record the actual vector magnitude and deviation for every documented case. Choose a justified tolerance after observing simulator resolution; do not invent a grading allowance. | DEFERRED | Stage 9, documented tests |
| A-08 | Deadline timezone is absent. | myLearning is authoritative. Do not convert 11:59 PM using the student's local timezone by assumption. Aim to finish the package before the deadline day. | EXTERNAL | Final submission |
| A-09 | Video file type and duration are not specified. | Propose a commonly playable MP4 if the recording tool supports it; length should be sufficient to show required evidence. Neither is a PDF mandate. | PROVISIONAL | Stage 10, recording |
| A-10 | Word-count inclusions/exclusions are not specified. | Keep the report in range using a disclosed counting method; do not assume tables, captions or appendices are exempt. The outline's word budget is a planning aid only. | DEFERRED | Stage 10, final report |
| A-11 | Detailed rubric and additional myLearning instructions were not supplied. | Use the five-page PDF as the present baseline. Record the limitation and revise the checklist if further material becomes available. | EXTERNAL | Design review and final submission |
| A-12 | AI planning/design permission is not established; the PDF allows editing only. | Keep the AI provenance of this pack visible. Do not represent these planning files as student-authored assessed work or treat the user's request as course permission. | EXTERNAL | Any use beyond the stated editing allowance |
| A-13 | "Roll only" and "pitch only" are ambiguous: combined tilt is at least the magnitude of either individual angle, so a compliant individual-axis angle also exceeds tilt's entry threshold. | Provisionally interpret these as only one individual axis being compliant while the other individual axis is free; tilt may also respond. Tilt-only still requires both individual axes to remain free, as expressly stated. Disclose the interpretation in the results. | PROVISIONAL | Stage 7 design and Stage 9 documented tests |

## Updating this log

For any changed interpretation, record the date, the old and new choice, the reason,
the authoritative source if available, and which diagrams/tests need review.
Do not silently rewrite an assumption after seeing a failing result.

| Date | Assumption ID | Change and reason | Source or evidence | Affected work |
| --- | --- | --- | --- | --- |
| To fill | To fill | To fill | To fill | To fill |
