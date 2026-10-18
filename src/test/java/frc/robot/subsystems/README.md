# Testing the intake extension

The extension is the pair of intake screws. At power-up the encoder reads 0 no matter where the mechanism actually sits, so extend and retract do nothing until calibration has driven into both hard stops.

`IntakeSubsystem` reads the TalonFX screws and the limit switch and applies whatever `ExtensionCalibrator` decides. These utilities feed that same calibrator scripted readings, because Phoenix simulation can set position and velocity but cannot set stator current. A current spike while the screw is barely moving is what counts as a hard stop.

Robot simulation does not run this sweep. In sim, `IntakeSubsystem` calls `assumeCalibrated` and pretends the travel is `kSimTravelRotations`.

## Run the suite

From the repository root:

```bash
./gradlew test --tests frc.robot.subsystems.ExtensionCalibrationTest
```

One case:

```bash
./gradlew test --tests frc.robot.subsystems.ExtensionCalibrationTest.limitOnTheFirstStopZerosThereAndParksInsideBothEnds
```

A failure message includes `harness.trace()`. Each line is one 20 ms sample:

```text
0.520 SEEK_SECOND cmd=-3.00 pos=4.00 vel=0.00 amps=45.0 limit=true
```

`cmd` is the voltage applied on that sample. The phase name is the phase after the sample. The trace keeps the last 30 lines.

## Pieces

| Class | Where | Role |
| --- | --- | --- |
| `ExtensionCalibrator` | `src/main/java/frc/robot/subsystems/` | The sweep. One `step` takes a position, a velocity, a current, the limit switch, and a timestamp. |
| `ExtensionCalibrator.Config` | same file | Stall current, stall speed, inrush, stall time, timeout, minimum travel, backoff, and the inward voltage. `Config.intake()` copies `Constants.Intake`. |
| `ExtensionCalibrationHarness` | this directory | Holds a reading for a length of time, or runs a whole sweep between two stops. |
| `ExtensionCalibrationTest` | this directory | The cases below. Add a new `@Test` here. |

`Config` argument order:

```text
inwardVoltage, stallCurrentAmps, stallVelocityRps, inrushSeconds,
stallSeconds, timeoutSeconds, minTravelRotations, backoffRotations
```

The tests build that config with the same numbers as `Constants.Intake` (`-3 V`, `35 A`, `0.5 rps`, `0.3 s`, `0.15 s`, `15 s`, `10 rot`, `1 rot`). Change the constants and update `config()` in the test, or construct the harness with `Config.intake()` when the case should follow the robot.

Each screw Talon also has a firmware current cap: `40 A` stator and `30 A` supply (`kStatorCurrentLimitAmps`, `kSupplyCurrentLimitAmps`). Stator current is the torque. The stator cap has to stay above the `35 A` stall threshold. The Talon clamps on the cap, and a clamp that never crosses `35 A` would never look like a hard stop.

## Script a sweep

`HardStop` is an encoder reading in rotations and whether the limit switch is pressed at that end. Pressed means that end is fully in.

`sweep` forces a new calibration, drives into the first stop, then reverses into the second. The first move uses the inward voltage.

```java
@Test
void seventyRotationsBetweenStops() {
  ExtensionCalibrationHarness harness = ExtensionCalibrationHarness.create(
      new ExtensionCalibrator.Config(-3.0, 35.0, 0.5, 0.3, 0.15, 15.0, 10.0, 1.0));

  harness.sweep(new HardStop(10.0, true), new HardStop(80.0, false));

  assertTrue(harness.calibrator().isCalibrated(), harness.trace());
  assertEquals(1.0, harness.calibrator().retractPositionRotations(), 1e-9);
  assertEquals(69.0, harness.calibrator().extendPositionRotations(), 1e-9);
  assertEquals(70.0, harness.zeroLeftRotations(), 1e-9);
  assertEquals(-70.0, harness.zeroRightRotations(), 1e-9);
  assertNull(harness.warning());
}
```

Travel is the fully-out reading minus the fully-in reading. Here that is `80 - 10 = 70`. Backoff is `1` rotation with the same sign as the travel, so the screws park inside each stop:

- retract target = `1`
- extend target = `70 - 1 = 69`
- left encoder is shifted so the fully-in stop becomes 0: `80 - 10 = 70`
- right encoder is the opposite of that shift: `-70`

Put the pressed switch on the second stop when the inward voltage reaches the outer end first. The calibrator then treats the second reading as fully in, and the shift on the finish sample is 0 because the screw is already sitting on the in stop.

If both stops report the same switch state, the first stop is treated as fully in and `harness.warning()` is `ExtensionCalibrator.SWITCH_WARNING`. That warning is recorded even when the travel is then rejected.

A negative travel (the outward end has a smaller encoder reading) gets a negative backoff. From `0` pressed to `-40` open, retract is `-1` and extend is `-39`.

## What makes a hard stop

A sample is a stall only when the current is above the stall current and the speed is below the stall speed. Equal to either threshold does not count. The stall has to hold for `stallSeconds`.

The first `inrushSeconds` after each direction change ignore current, including a real stall. Motion during that window does not count either.

The second seek ignores a stall until the screw has gone faster than the stall speed after the inrush window. That keeps the stop it just left from counting as the other end.

Each direction has its own timeout, measured from when that direction started. On timeout, `harness.error()` is `ExtensionCalibrator.TIMEOUT_ERROR`, the phase is `FAILED`, and the encoder is not zeroed.

Travel shorter than `minTravelRotations` fails with `ExtensionCalibrator.TRAVEL_ERROR` and does not zero the encoder.

## Write a partial sweep

`sweep` is the whole trip. Use the smaller calls when a case cares about one window:

- `start(true)` begins a sweep at t = 0. `start(false)` leaves a finished sweep alone.
- `hold(seconds, position, velocityRps, currentAmps, limitPressed)` keeps feeding that reading. It keeps going after the phase changes.
- `holdUntilPhaseChange(...)` stops when the phase changes, so a stall does not eat the next direction's inrush window.
- `seek(stop)` coasts through the inrush window, coasts long enough for the second stop to be allowed, then stalls on that stop.

`seek` uses a coast speed of `stallVelocity + 2` rps and a coast current of `stallCurrent - 1` A, then a stall current of `stallCurrent + 10` A at 0 rps. The limit switch is open while coasting and matches the `HardStop` only while stalled.

```java
harness.start(true);
harness.seek(new HardStop(4.0, true));
assertEquals(Phase.SEEK_SECOND, harness.calibrator().phase(), harness.trace());

// Still against the far stop, but the screw never left the first one.
harness.hold(0.6, 74.0, 0.0, 80.0, false);
assertEquals(Phase.SEEK_SECOND, harness.calibrator().phase(), harness.trace());
```

After the first stop, the voltage on that sample is still the inward voltage (`-3`). The reversed voltage (`+3`) is applied on the next sample. `lastVoltage()` reads the voltage the calibrator just commanded.

`calibrator().onCommandEnd(true)` is a cancelled command: an in-progress sweep returns to `IDLE`, and a finished one stays `DONE`. `onCommandEnd(false)` leaves a `FAILED` sweep failed.

## Cases already covered

| Test | What it checks |
| --- | --- |
| `limitOnTheFirstStopZerosThereAndParksInsideBothEnds` | Switch pressed on the first stop. Encoder zeroed there, targets parked 1 rotation inside each end. |
| `limitOnTheSecondStopTreatsThatEndAsFullyIn` | Switch pressed only on the second stop. That end becomes 0. |
| `matchingSwitchReadingsFallBackToTheInwardDirection` | Switch never changes. First stop is fully in, and the warning is set. |
| `negativeTravelBacksOffTowardTheScrews` | Outward encoder reading is smaller. Backoff follows that sign. |
| `stopsCloserThanTheMinimumTravelFail` | Travel under 10 rotations fails and does not zero. |
| `ambiguousSwitchStillWarnsWhenTheTravelIsTooShort` | Same switch reading still warns when the travel is then rejected. |
| `firstSeekTimesOutWithoutAStall` | No stall before the first direction times out. |
| `secondSeekGetsItsOwnTimeout` | The first stop does not use up the second direction's timeout. |
| `stallDuringInrushDoesNotCount` | A spike inside the first 0.3 s is ignored, and a later sweep still finishes. |
| `secondStallWaitsUntilTheScrewHasMoved` | The second stop does not count until the screw has moved. |
| `currentAtTheStatorCapStillCountsAsAHardStop` | 40 A, the firmware torque cap, still trips the 35 A detector. |
| `currentAndVelocityAtTheThresholdAreNotAStall` | 35 A or 0.5 rps exactly is not a stop. |
| `reversingVoltageIsAppliedOnTheStepAfterTheFirstStop` | Polarity flips on the sample after the first stop. |
| `finishedSweepIsLeftAloneUntilForced` | `start(false)` keeps the old targets. A later `sweep` replaces them. |
| `cancellingClearsAnInProgressSweepAndKeepsAFinishedOne` | Cancel mid-sweep versus cancel after success. |
| `aFailedSweepStaysFailedWhenTheCommandEnds` | A normal end leaves `FAILED` in place. |
| `assumedCalibrationSkipsTheSweep` | `assumeCalibrated` is the simulation shortcut. No encoder write. |

## On the robot

The running robot prints the same sweep on SmartDashboard:

| Key | Meaning |
| --- | --- |
| `Intake/CalibrationPhase` | `IDLE`, `SEEK_FIRST`, `SEEK_SECOND`, `DONE`, or `FAILED` |
| `Intake/Calibrated` | Extend and retract are allowed |
| `Intake/ScrewCurrentAmps` | Higher of the two stator currents |
| `Intake/ScrewStatorLimitAmps` | Torque cap applied to each extension screw |
| `Intake/ScrewPositionRotations` | Left screw, after the in-stop has been shifted to 0 |
| `Intake/ExtendPositionRotations` | Left-screw target just inside the outer stop |
| `Intake/RetractPositionRotations` | Left-screw target just inside the fully-in stop |

`Intake/Recalibrate` runs the sweep again. Teleop schedules it once on enable, and it does nothing if a sweep already succeeded. Auto runs it before the selected auto and skips the auto if calibration fails.
