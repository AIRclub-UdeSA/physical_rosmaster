## Summary

<!-- What changed, and why. -->

## Runtime behavior

<!-- Does this change a launch arg, topic, or driver default? If yes, say so
explicitly — this affects a shared physical robot. -->

- [ ] No runtime behavior change
- [ ] Runtime behavior changes (described above)

## Safety-relevant?

<!-- Does this touch motor commands, safety limits, watchdogs, or timeout
behavior? If yes, call it out clearly so reviewers check it carefully. -->

- [ ] No
- [ ] Yes (described above)

## Verification

<!-- How did you verify this? Be precise about validation status — if
something is floor-tested but not ground-truth-verified, say so rather than
overstating it. -->

- [ ] Workstation build + `colcon test` pass
- [ ] Robot-side validation completed (required for motion/driver/odometry
      changes — see `docs/robot_side_verification_todo.md` and
      `docs/odometry_validation.md`)
- [ ] N/A — no robot-side validation needed

## Test plan

<!-- What a reviewer should check, or what you ran. -->

## Related issues

<!-- Fixes #, Closes #, Related to # -->
