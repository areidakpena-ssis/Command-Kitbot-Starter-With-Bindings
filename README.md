# Command-Kitbot-Starter-With-Bindings

## Overview
This repository contains the reference implementation of the 2026 FRC KitBot code, built
using the WPILib Command-Based framework. The robot features a differential drivebase, a
flywheel shooter, an intake, and a loader mechanism, with all subsystems fully implemented
and wired to a controller. This is the "answer key" version — see
`Command-Kitbot-Starter` (no bindings) for the student starting point used in the intro
lesson.

## Hardware Specifications

| Component | Hardware / Motors | Notes |
| :--- | :--- | :--- |
| **Drivebase** | 4x CIM Motors with REV SparkMax controllers | Configured as a Differential Drive. Left side inverted, right side not. |
| **Drive Encoder** | 1x CTRE CANcoder | Mounted on the right side in a WCP throughbore encoder housing. Only one physically installed — see Known Issues. |
| **Flywheel** | 1x CTRE Kraken X60 | Utilizes the integrated TalonFX controller. |
| **Intake** | 1x CTRE Kraken X60 | Utilizes the integrated TalonFX controller. Configured `Clockwise_Positive`. |
| **Loader** | 1x REV NEO with SparkMax | Configured as a brushless motor, inverted. |

## Subsystems
* **`DriveSubsystem`**: Controls the differential drivebase using a split-stick arcade
  drive setup. Currently tracks distance using a single right-side CANcoder —
  `getAverageDistanceMeters()` returns just the right-side reading until a second encoder
  is installed.
* **`Flywheel`**: Manages the shooter wheel using closed-loop velocity control (software
  PID + feedforward via `VoltageOut`). Target speed is adjustable at runtime in 1000 RPM
  steps.
* **`IntakeClass`**: Handles ingestion of game pieces. Open-loop (duty cycle) by design —
  see the class javadoc for why closed-loop wasn't used here.
* **`Loader`**: Feeds game pieces between the intake and the flywheel. Open-loop, with
  three distinct speeds depending on direction of travel (see Intake & Loader Constants
  below) — faster stages later in the direction of travel, to avoid jams.

## Controller Bindings
The robot is controlled via an Xbox Controller plugged into Port 0.

| Input | Action |
| :--- | :--- |
| **Left Joystick (Y-Axis)** | Arcade Drive: Forward / Backward |
| **Right Joystick (X-Axis)** | Arcade Drive: Rotation (Turning) |
| **A Button (Hold)** | Run Drivebase Feedforward Test at 1.0 m/s |
| **B Button (Hold)** | Run Drivebase Feedforward Test at 2.0 m/s |
| **X Button (Hold)** | Run Drivebase Feedforward Test at -1.0 m/s |
| **Y Button (Hold)** | Run Drivebase Feedforward Test at -2.0 m/s |
| **Right Trigger (Hold)** | Fires game piece (runs Flywheel and Loader-to-Flywheel in parallel) |
| **Left Trigger (Toggle, debounced 0.05s)** | Toggles Intake + Loader-with-Intake together |
| **Right Bumper (Press, debounced 0.05s)** | Increases Flywheel target speed by 1000 RPM |
| **Left Bumper (Press, debounced 0.05s)** | Decreases Flywheel target speed by 1000 RPM |

## Key Constants & Configuration

### CAN IDs
| Subsystem | Component | CAN ID |
| :--- | :--- | :--- |
| **Drivebase** | Left Leader | 11 |
| | Left Follower | 8 |
| | Right Leader | 10 |
| | Right Follower | 7 |
| | Right CANcoder | 4 |
| **Flywheel** | Flywheel Motor | 9 |
| **Intake** | Intake Motor | 12 |
| **Loader** | Loader Motor | 19 |

### Physical Constants
| Property | Value |
| :--- | :--- |
| **Wheel Diameter** | 0.1524 Meters (approx. 6 inches) |
| **Drive Gear Ratio** | 8.45 |
| **Track Width** | 0.55 Meters |
**Note:** encoder (CTRE CANcoder) is mounted directly on the output axle, so gear ratio
should not be used for conversions.

### Drivebase Control Constants (SysId Tuned)
* **Feedforward:** `kDriveBase_kS` = 1.008, `kDriveBase_kV` = 2.575, `kDriveBase_kA` = 0.675
* **Feedback (PID):** `kDriveBase_kP` = 3.21, `kDriveBase_kI` = 0.0, `kDriveBase_kD` = 0.0

### Flywheel Control Constants (SysId Tuned)
* **Feedforward:** `kFlywheel_kS` = 0.06, `kFlywheel_kV` = 0.10, `kFlywheel_kA` = 0.025
* **Feedback (PID):** `kFlywheel_kP` = 0.13, `kFlywheel_kI` = 0.0, `kFlywheel_kD` = 0.0

### Intake & Loader Constants (bench-tested, open-loop)
* **Intake:** `kIntakeDefaultOutput` = 0.3 (used for both forward and reverse). Original
  design note: intake surface speed roughly 2x robot ground speed is a reasonable
  starting point if retuning.
* **Loader:** `kLoaderToIntakeOutput` = 0.35, `kLoaderToFlywheelOutput` = 0.4,
  `kLoaderFromIntakeOutput` = 0.7.
  > **Note:** the comment block above these constants in `Constants.java` still describes
  > an older ordering principle (`kLoaderToIntakeOutput` < `kLoaderFromIntakeOutput` <
  > `kLoaderToFlywheelOutput`, each downstream stage faster than the last) that the
  > *current* bench-tested values no longer follow — `kLoaderFromIntakeOutput` (0.7) is
  > now the fastest of the three, not the middle one. Worth deciding whether the values
  > or the comment (or both) need another pass before this is treated as final.
* Reserved-for-later constants: `kIntakeDefaultTargetRPM`, `kLoaderIntakeTargetRPM`,
  `kLoaderToFlywheelTargetRPM` — not used yet, kept for a possible future closed-loop
  conversion of Intake/Loader (same pattern as Flywheel).

## Development Status & Next Steps

### Completed
* Core command-based skeleton implemented for all four subsystems, all fully working —
  no stubs remaining.
* Flywheel and Drivebase feedforward/feedback constants calculated via WPILib SysId and
  implemented.
* Epilogue telemetry logging added for actual speeds; `testFeedforwardCommand` open-loop
  velocity test commands wired to face buttons (`A`/`B`/`X`/`Y`).
* Real `IntakeClass` written (open-loop, both directions) and `Loader` reworked from a
  single hardcoded speed into three direction-specific, bench-tested speeds.
* Flywheel's fire sequence now explicitly requires both `Flywheel` and `Loader` via
  `Commands.parallel(...)` in `RobotContainer`, so the scheduler can't run a conflicting
  command on either subsystem mid-fire.
* Intake and Loader inversion bugs fixed (`InvertedValue.Clockwise_Positive` on Intake,
  `.inverted(true)` on Loader) — both previously spun the wrong physical direction by
  default.
* Autonomous stop bug fixed — `Autos.driveDistance()` now actually calls
  `driveSubsystem.stopMotors()` in its `finallyDo`, rather than building an unused
  `Command` object and discarding it.

### To Do (real remaining engineering work)
* **Hardware:** install a second drivebase encoder on the left side to improve tracking
  reliability (`getAverageDistanceMeters()` is a placeholder using only the right side
  until then).
* **Autonomous Tuning:** calibrate the turning scalar (`kAutoTurnDistanceScalar`,
  currently `0.125`) once the second encoder is installed.
* **Kinematics:** incorporate WPILib `DifferentialDriveKinematics` for more advanced
  trajectory following in autonomous modes.
* **Loader constants:** resolve the stale ordering comment noted above, either by
  retuning the values or rewriting the comment to match reality.
* **Loader reverse-direction strain:** `runWithIntakeCommand(true)` currently flips the
  loader's duty cycle sign immediately with no pause — a coast-before-reverse step
  (`Commands.sequence` + `waitSeconds`) was discussed but not implemented; worth adding
  if motor strain becomes a real concern.

### Left as discussion points (intentional, not bugs)
A few known simplifications were deliberately left in rather than fixed, as material for
class discussion or student exercises:
* `Loader.runLoaderCommand()`'s `finallyDo(() -> m_loaderMotor.stopMotor())` could become
  a method reference (`m_loaderMotor::stopMotor`) — left as a lambda.
* `Flywheel.stopShooterCommand()` was renamed for naming consistency, but its body still
  uses `setVoltage(0.0)` rather than `stopMotor()` (inconsistent with how
  `runShooterCommand()`'s own cleanup stops the motor), and is still a lambda rather than
  a method reference.
* `Flywheel.increaseShooterSpeedCommand()` / `decreaseShooterSpeedCommand()` remain two
  separate, near-duplicate methods rather than one shared parameterized helper.
* `DriveSubsystem.stopDriveCommand()` / `resetEncoderCommand()` still use verbose lambda
  form (`runOnce( () -> this.stopMotors() )`) rather than method references.
* `ExampleSubsystem.java` / `ExampleCommand.java` are kept in the repo as standalone,
  unused WPILib template files — not referenced from `RobotContainer` (the old unused
  `m_exampleSubsystem` field there was removed), but still called from
  `Autos.exampleAuto()`, which itself isn't wired to anything.
* The left-trigger binding's `.debounce(0.05)` uses the default `DebounceType.kBoth`
  (delays both press and release); duration was shortened from an earlier `0.5` that
  made toggling feel unresponsive, but switching to `kRising` (debounce only the press)
  wasn't applied.