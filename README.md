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
- Left trigger (hold): slow rotate left (in place).
- Right trigger (hold): slow rotate right (in place).
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

### Auto readiness indicators

- `Auto/SelectedValid`: `true` when a real auto is selected, `false` when the chooser is still on `Select Auto`.
- `Auto/SelectedName`: the currently selected auto name.
- `Auto/Status`: human-readable autonomous status and failure reason.
- `Auto/StartPoseSeeded`: `true` once the selected auto's start pose has been seeded successfully.

## Dashboard Items

- `Auto Mode`: autonomous chooser.
- `Auto/SelectedValid`: quick validity check for the selected auto.
- `Auto/SelectedName`: selected auto name.
- `Auto/Status`: autonomous readiness / failure status.
- `Auto/StartPoseSeeded`: whether autonomous start pose seeding succeeded.
- `Shots/X Back RPS`: custom back shooter speed for the `X` shot.
- `Shots/X Front RPS`: custom front shooter speed for the `X` shot.
- `Swerve/* Raw Abs (rot)`: raw absolute encoder values for each swerve module.
- `SwerveCal/* OffsetToPaste (rot)`: copy these directly into `TunerConstants` encoder offsets (Option 1 calibration flow).
- `SwerveCal/PasteLine *`: per-module ready-to-paste lines for `TunerConstants`.
- `SwerveCal/PasteBlock`: four-line ready-to-paste block for all module offsets.
- `SwerveCal/Instruction`: quick reminder of the calibration workflow.

## Shooter Tuning Knobs

- Shared shoot conveyor duty cycles and shooter spin-up timeout now live in `RobotContainer`:
- `SHOOT_INTAKE_DUTY`
- `SHOOT_INDEXER_DUTY`
- `SHOOT_FEEDER_DUTY`
- `SHOOTER_SPINUP_TIMEOUT_SECONDS`
- These values are used by both operator shoot buttons and the registered PathPlanner `Shoot` named command.

## CAN Motor Map And Config

| CAN ID | Device | Subsystem / Module | Bus | Motor Type / Arrangement | Stator Limit (A) | Supply Limit (A) | Neutral Mode | Key Config Notes |
|---|---|---|---|---|---:|---:|---|---|
| 0 | Front Left Drive | Swerve Front Left | CANivore (`1515Canivore`) | `TalonFX_Integrated` drive | Not explicitly set in code | Not explicitly set in code | Coast | Left side drive invert flag `false` |
| 1 | Front Left Steer | Swerve Front Left | CANivore (`1515Canivore`) | `TalonFX_Integrated` steer | 45 | Not explicitly set in code | Coast | Steer invert `false`, feedback source `FusedCANcoder` |
| 2 | Front Right Drive | Swerve Front Right | CANivore (`1515Canivore`) | `TalonFX_Integrated` drive | Not explicitly set in code | Not explicitly set in code | Coast | Right side drive invert flag `true` |
| 3 | Front Right Steer | Swerve Front Right | CANivore (`1515Canivore`) | `TalonFX_Integrated` steer | 45 | Not explicitly set in code | Coast | Steer invert `false`, feedback source `FusedCANcoder` |
| 4 | Back Left Drive | Swerve Back Left | CANivore (`1515Canivore`) | `TalonFX_Integrated` drive | Not explicitly set in code | Not explicitly set in code | Coast | Left side drive invert flag `false` |
| 5 | Back Left Steer | Swerve Back Left | CANivore (`1515Canivore`) | `TalonFX_Integrated` steer | 45 | Not explicitly set in code | Coast | Steer invert `false`, feedback source `FusedCANcoder` |
| 6 | Back Right Drive | Swerve Back Right | CANivore (`1515Canivore`) | `TalonFX_Integrated` drive | Not explicitly set in code | Not explicitly set in code | Coast | Right side drive invert flag `true` |
| 7 | Back Right Steer | Swerve Back Right | CANivore (`1515Canivore`) | `TalonFX_Integrated` steer | 45 | Not explicitly set in code | Coast | Steer invert `false`, feedback source `FusedCANcoder` |
| 18 | Indexer Motor | Indexer | roboRIO CAN | `TalonFXS` + `Minion_JST` | 40 | 30 | Brake | Duty-cycle output command |
| 20 | Left Pivot Motor (Leader) | Pivot | roboRIO CAN | `TalonFX` | 60 | 40 | Brake by config; switched to Coast in `disabledInit()` | Manual voltage control; follower pair |
| 21 | Right Pivot Motor (Follower) | Pivot | roboRIO CAN | `TalonFX` follower (Opposed) | 60 | 40 | Brake by config; switched to Coast in `disabledInit()` | Follows ID 20 with `MotorAlignmentValue.Opposed` |
| 22 | Intake Motor | Intake | roboRIO CAN | `TalonFXS` + `Minion_JST` | 40 | 30 | Brake | Inverted `Clockwise_Positive`; duty-cycle output |
| 23 | Feeder Motor | Feeder | roboRIO CAN | `TalonFX` | 60 | 40 | Brake | Duty-cycle output |
| 24 | Back Left Shooter | Shooter | roboRIO CAN | `TalonFX` | 40 | 30 | Brake | Inverted `Clockwise_Positive`; velocity closed-loop |
| 25 | Back Right Shooter (Follower) | Shooter | roboRIO CAN | `TalonFX` follower (Aligned) | 40 | 30 | Brake | Follows ID 24 with `MotorAlignmentValue.Aligned` |
| 26 | Front Shooter | Shooter | roboRIO CAN | `TalonFX` | 40 | 30 | Brake | Inverted `Clockwise_Positive`; velocity closed-loop |

### Swerve Azimuth Sensors (Not Motors)

| CAN ID | Device | Module | Bus | Notes |
|---|---|---|---|---|
| 10 | CANcoder | Front Left | CANivore (`1515Canivore`) | Encoder invert flag `false`; code-side offset in `TunerConstants` |
| 11 | CANcoder | Front Right | CANivore (`1515Canivore`) | Encoder invert flag `false`; code-side offset in `TunerConstants` |
| 12 | CANcoder | Back Left | CANivore (`1515Canivore`) | Encoder invert flag `false`; code-side offset in `TunerConstants` |
| 13 | CANcoder | Back Right | CANivore (`1515Canivore`) | Encoder invert flag `false`; code-side offset in `TunerConstants` |

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
