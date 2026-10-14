# Agent instructions: scaffolding_assembly_core

This library is the shared core: one process format, one scene model and one plan check used by several
consumers. A change that looks local here can break a consumer that is not in front of you.

## Changes to the core need careful review

Applies to any change of behaviour or format: `process/` (types, read, write, validate, plan_check, relations,
scenes), `scene.py`, `robot.py`, `urdf.py`, `kinematics.py`, the mirrors, and `doc/process_format.md`.
Pure refactors with identical behaviour and test-only changes are exempt.

1. **Name the consumers first.** At least:
   - the Rhino design workflow (`bar_joint_rhino_design_workflow`: `scripts/core/process_source.py`,
     `process_builder.py`, `process_planning.py`, `plan_steps.py`, the viewers and `rs_*` commands);
   - the headless planner (`scripts/plan_process.py` there, and `husky_assembly_tamp`);
   - the execution monitor (`husky-assembly-teleop`), once it reads processes;
   - process folders already exported (`writer.schema`, §9 of the format doc).
2. **Get a review from the other side.** At least one subagent reviews the change from a consumer's point of
   view, not the author's: give it the diff and the consumer's code, not your conclusion, and ask what breaks
   there (a new enum value, a field that may now be empty, a rule that now passes or fails, ids it looks up).
3. **Run the tests of both sides:** this repo's `tests/`, and the consumer tests that touch the core (in the Rhino
   repo at least `tests/test_process_builder.py`, `test_process_planning.py`, `test_plan_steps.py`,
   `test_process_source.py`). Report failures that were there before the change separately.
4. **Decide the schema version.** A change to the file format raises `SCHEMA` in `process/version.py` when old
   files can no longer be read, or new files would be misread by the old library; say which in the change.
5. **Ask before** changing the format or the plan-check rules when the user did not ask for exactly that.

## Always update the docs with the change

- `doc/process_format.md`: every field, enum value, id code and plan-check rule the change touches.
- Markdown in the consumers that describes the changed behaviour (in the Rhino repo `docs/`, e.g.
  `schema2_integration.md`, `rhino_toolbar_entrypoints.md`, `color_preview_reference.md`).
- Published reports (claude.ai artifacts and docs) that describe the format or the core, e.g. "Process format,
  schema 2.1": update them when your tools can reach them; otherwise list them as not updated. Leave pages marked
  superseded as they are.
- In the final message, list what was updated and what was not.
