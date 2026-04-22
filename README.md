# Team 1515 Mortorq - 2026 Rebuilt Robot Code

This README is for drivers, programmers, and pit crew to understand how the robot behaves right now.

## Robot Behavior Summary

- Drivetrain is CTRE Phoenix 6 swerve with field-centric default driving.
- Driver has teleop shot alignment buttons (`X`, `Y`, `B`) that pathfind to fixed shooting poses.
- Driver has a homing/reset button (`Start`) to reset pose estimate at the hub-front reference spot.
- Autonomous options are loaded from `src/main/deploy/pathplanner/autos/*.auto`.
- Alliance flipping is automatic for both auto and teleop shot pathfinding.

## Current Controller Bindings

### Driver Controller (`port 0`)

- Left stick `Y/X`: field-centric translation.
- Right stick `X`: field-centric rotation.
- Left bumper (hold): swerve brake.
- D-pad up/down/left/right (hold): robot-centric crawl at fixed speed.
- Start (press): reset pose to hub-home reference and reseed field-centric.
- X (press): pathfind to left shot setpoint.
- Y (press): pathfind to middle shot setpoint.
- B (press): pathfind to right shot setpoint.
- X/Y/B (release): cancel active driver pathfind command.
- Left trigger (hold): `StrafeToTag`.
- Right trigger (hold): `RotateToTag`.

### Operator Controller (`port 1`)

- D-pad up (hold): pivot up.
- D-pad down (hold): pivot down.
- D-pad left (hold): pivot to bottom and home.
- D-pad right (hold): pivot up, then oscillate down/up sequence.
- Left trigger (toggle): intake + reverse index/feed + shooter anti-jam behavior.
- B (hold): reverse intake/indexer/feeder.
- Right trigger (toggle): big shot sequence.
- Right bumper (toggle): regular shot sequence.
- Y (toggle): lob shot sequence.
- A (toggle): line-drive shot sequence.

## How Teleop Shot Alignment Works

1. Driver physically places robot at the front of the hub reference spot.
2. Driver presses `Start` to reset pose estimate to `HUB_HOME_POSE_BLUE` (flipped automatically on Red).
3. Driver presses `X`, `Y`, or `B` to pathfind to left/middle/right shot poses.
4. Releasing `X`, `Y`, or `B` cancels the active pathfind command.

This method is used because a full `resetPose(...)` is more reliable than only reseeding heading when the robot has been moved by hand.

## PathPlanner and Setpoint Source of Truth

### Runtime files

- Autos: `src/main/deploy/pathplanner/autos/`
- Paths: `src/main/deploy/pathplanner/paths/`
- Navgrid: `src/main/deploy/pathplanner/navgrid.json`

### Code source of truth for teleop shot/homing setpoints

- `src/main/java/frc/BotchoCheese/Constants/PathPlannerSetpoints.java`

The robot does not parse path JSON at runtime for these teleop shot targets anymore.
If you tune shot/homing poses in PathPlanner, copy updated values into `PathPlannerSetpoints.java`.

Current headings for shot setpoints are intentionally tied to `goalEndState.rotation` from:

- `Left side.path`
- `Middle.path`
- `Right side.path`

## Autonomous Selection

- Auto chooser key on dashboard: `Auto Mode`.
- All `.auto` files are listed; no Red-name filtering is applied.
- In teleop init, robot seeds pose from selected auto start pose when available.

## Vision Notes

- `StrafeToTag` and `RotateToTag` are available from driver triggers.
- `LimelightHomography.update(...)` is currently disabled in `Robot` (`kUseLimelight = false`).

## Important Files

- `src/main/java/frc/BotchoCheese/RobotContainer.java` (all bindings and command wiring)
- `src/main/java/frc/BotchoCheese/Robot.java` (mode lifecycle behavior)
- `src/main/java/frc/BotchoCheese/Constants/PathPlannerSetpoints.java` (teleop shot/home setpoints)
- `src/main/deploy/pathplanner/` (auto/path assets)

## Team Workflow Notes

- Keep this README and `PathPlannerSetpoints.java` in sync with current driver behavior.
- If controls change in `RobotContainer`, update this document in the same PR.
- Old `controllerbounds.txt` may be stale; use `RobotContainer` and this README as the current reference.
