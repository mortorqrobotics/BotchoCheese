# Team 1515 Mortorq - 2026 Rebuilt Robot Code

This README is the current quick-reference for drivers, operators, pit crew, and programmers.

## Robot Summary

- Drivetrain is CTRE Phoenix 6 swerve with field-centric default driving.
- Autonomous modes are loaded from `src/main/deploy/pathplanner/autos/`.
- Teleop no longer resets pose from the selected auto when enabled.
- Pivot is manual only. There is no working auto-home routine in the current code.
- Limelight pose fusion is present in code but currently disabled.

## Current Controller Bindings

### Driver Controller (`port 0`)

- Left stick `Y`: drive forward and backward.
- Left stick `X`: strafe left and right.
- Right stick `X`: rotate robot.
- Left bumper (hold): swerve brake.
- D-pad up/down/left/right (hold): robot-centric crawl at fixed speed.
- Start (press): reseed field-centric heading only.

### Operator Controller (`port 1`)

- D-pad up (hold): pivot up.
- D-pad down (hold): pivot down.
- Left trigger (hold): intake / anti-jam sequence.
- B (hold): reverse intake, indexer, and feeder.
- Right trigger (toggle): big shot.
- Right bumper (toggle): regular shot.
- X (toggle): SmartDashboard-programmed shot using `Shots/X Back RPS` and `Shots/X Front RPS`.
- Y (toggle): lob shot.
- A (toggle): line-drive shot.

## Operator Notes

- Left trigger is hold-to-run. Releasing it stops the intake sequence.
- `B` is hold-to-run. Releasing it stops the reverse/un-jam sequence.
- Right trigger, right bumper, `X`, `Y`, and `A` are toggled shots. Press once to start, press again to stop.
- Pivot control is manual. Do not expect it to home itself or move to saved positions.

## Autonomous

- Dashboard chooser key: `Auto Mode`.
- On autonomous enable, the robot seeds pose from the selected auto's starting pose when one is available.
- If no auto is selected, the robot runs a no-op command in autonomous.
- Deploy now deletes old files from the roboRIO deploy directory, which helps prevent stale autos and paths from lingering between events.

## Dashboard Items

- `Auto Mode`: autonomous chooser.
- `Shots/X Back RPS`: custom back shooter speed for the `X` shot.
- `Shots/X Front RPS`: custom front shooter speed for the `X` shot.
- `Swerve/* Raw Abs (rot)`: raw absolute encoder values for each swerve module.

## Vision Notes

- `LimelightHomography.update(...)` is currently disabled in `Robot`.
- The codebase still contains some vision alignment commands, but they are not currently bound to the driver controller.

## Important Files

- `src/main/java/frc/BotchoCheese/RobotContainer.java`: controller bindings and command wiring.
- `src/main/java/frc/BotchoCheese/Robot.java`: robot mode lifecycle behavior.
- `src/main/java/frc/BotchoCheese/Subsystems/Pivot.java`: manual pivot behavior.
- `src/main/deploy/pathplanner/`: autonomous and path assets.

## Team Workflow Notes

- Keep this README in sync with `RobotContainer.java`.
- If bindings change, update this document in the same PR.
- Treat `controllerbounds.txt` as potentially stale unless it is updated alongside the code.
