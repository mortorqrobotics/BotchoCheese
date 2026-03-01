# Team 1515 Mortorq - 2026 REBUILT Robot Code

Java/WPILib robot code for Team 1515's 2026 robot, built on the command-based framework with CTRE Phoenix 6 swerve and subsystem control.

## Stack

- Java 17
- WPILib 2026
- GradleRIO `2026.2.1`
- CTRE Phoenix 6
- Limelight vision integration

## Quick Start

### Prerequisites

- WPILib 2026 installed (includes JDK 17 + Gradle tooling)
- FRC vendor dependencies installed (from `vendordeps/`)
- VS Code with WPILib extension (recommended)

### Build

```powershell
.\gradlew.bat build
```

### Run Tests

```powershell
.\gradlew.bat test
```

### Simulate

```powershell
.\gradlew.bat simulateJava
```

### Deploy to RoboRIO

```powershell
.\gradlew.bat deploy
```

Team number and deploy target are resolved from WPILib project preferences.

## Project Layout

```text
src/main/java/frc/BotchoCheese/
  Commands/        # Driver-assist and autonomous command logic
  Constants/       # Robot IDs, PID gains, and field constants
  Subsystems/      # Climber, intake, shooter, feeder, indexer, drivetrain
  Utils/           # Limelight helper integration
  RobotContainer   # Command bindings and subsystem wiring
  Robot            # Main robot lifecycle entry
```

## Vision and Shooter Notes

- `StrafeToTag` uses Limelight `tx/ty` with robot-centric PID control for tag alignment.
- `ShooterInterpolated` adds distance-based interpolation for shooter speed and hood position.
- Several `RobotMap` constants are placeholders and marked with `TODO` (for example motor IDs and gains); tune and validate before competition use.

## Key Files

- `src/main/java/frc/BotchoCheese/RobotContainer.java`
- `src/main/java/frc/BotchoCheese/Constants/RobotMap.java`
- `src/main/java/frc/BotchoCheese/Commands/StrafeToTag.java`
- `src/main/java/frc/BotchoCheese/Subsystems/ShooterInterpolated.java`

## Contributing

See `CONTRIBUTING.md` for workflow expectations and contribution guidelines.
