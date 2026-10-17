// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.ExtensionCalibrationHarness.HardStop;
import frc.robot.subsystems.ExtensionCalibrator.Config;
import frc.robot.subsystems.ExtensionCalibrator.Phase;

/**
 * Runs the intake extension sweep against scripted hard stops. The numbers match
 * {@code Constants.Intake} so a change to those thresholds shows up here.
 */
class ExtensionCalibrationTest {
  private static final double kEpsilon = 1e-9;

  private static Config config() {
    return new Config(-3.0, 35.0, 0.5, 0.3, 0.15, 15.0, 10.0, 1.0);
  }

  private static ExtensionCalibrationHarness harness() {
    return ExtensionCalibrationHarness.create(config());
  }

  @Test
  void limitOnTheFirstStopZerosThereAndParksInsideBothEnds() {
    ExtensionCalibrationHarness harness = harness();
    harness.sweep(new HardStop(4.0, true), new HardStop(74.0, false));

    assertTrue(harness.calibrator().isCalibrated(), harness.trace());
    assertFalse(harness.calibrator().isFailed(), harness.trace());
    assertEquals(Phase.DONE, harness.calibrator().phase());
    assertEquals(1.0, harness.calibrator().retractPositionRotations(), kEpsilon);
    assertEquals(69.0, harness.calibrator().extendPositionRotations(), kEpsilon);
    assertEquals(70.0, harness.zeroLeftRotations(), kEpsilon);
    assertEquals(-70.0, harness.zeroRightRotations(), kEpsilon);
    assertNull(harness.warning());
  }

  @Test
  void limitOnTheSecondStopTreatsThatEndAsFullyIn() {
    ExtensionCalibrationHarness harness = harness();
    harness.sweep(new HardStop(74.0, false), new HardStop(4.0, true));

    assertTrue(harness.calibrator().isCalibrated(), harness.trace());
    assertEquals(1.0, harness.calibrator().retractPositionRotations(), kEpsilon);
    assertEquals(69.0, harness.calibrator().extendPositionRotations(), kEpsilon);
    assertEquals(0.0, harness.zeroLeftRotations(), kEpsilon);
    assertEquals(0.0, harness.zeroRightRotations(), kEpsilon);
    assertNull(harness.warning());
  }

  @Test
  void matchingSwitchReadingsFallBackToTheInwardDirection() {
    ExtensionCalibrationHarness harness = harness();
    harness.sweep(new HardStop(4.0, false), new HardStop(74.0, false));

    assertTrue(harness.calibrator().isCalibrated(), harness.trace());
    assertEquals(ExtensionCalibrator.SWITCH_WARNING, harness.warning());
    assertEquals(1.0, harness.calibrator().retractPositionRotations(), kEpsilon);
    assertEquals(69.0, harness.calibrator().extendPositionRotations(), kEpsilon);
    assertEquals(70.0, harness.zeroLeftRotations(), kEpsilon);
  }

  @Test
  void negativeTravelBacksOffTowardTheScrews() {
    ExtensionCalibrationHarness harness = harness();
    harness.sweep(new HardStop(0.0, true), new HardStop(-40.0, false));

    assertTrue(harness.calibrator().isCalibrated(), harness.trace());
    assertEquals(-1.0, harness.calibrator().retractPositionRotations(), kEpsilon);
    assertEquals(-39.0, harness.calibrator().extendPositionRotations(), kEpsilon);
    assertEquals(-40.0, harness.zeroLeftRotations(), kEpsilon);
    assertEquals(40.0, harness.zeroRightRotations(), kEpsilon);
  }

  @Test
  void stopsCloserThanTheMinimumTravelFail() {
    ExtensionCalibrationHarness harness = harness();
    harness.sweep(new HardStop(4.0, true), new HardStop(8.0, false));

    assertFalse(harness.calibrator().isCalibrated(), harness.trace());
    assertTrue(harness.calibrator().isFailed());
    assertEquals(Phase.FAILED, harness.calibrator().phase());
    assertEquals(ExtensionCalibrator.TRAVEL_ERROR, harness.error());
    assertFalse(harness.zeroed());
    assertNull(harness.warning());
  }

  @Test
  void ambiguousSwitchStillWarnsWhenTheTravelIsTooShort() {
    ExtensionCalibrationHarness harness = harness();
    harness.sweep(new HardStop(4.0, false), new HardStop(8.0, false));

    assertEquals(ExtensionCalibrator.TRAVEL_ERROR, harness.error());
    assertEquals(ExtensionCalibrator.SWITCH_WARNING, harness.warning());
    assertFalse(harness.zeroed());
  }

  @Test
  void firstSeekTimesOutWithoutAStall() {
    Config config = new Config(-3.0, 35.0, 0.5, 0.3, 0.15, 0.5, 10.0, 1.0);
    ExtensionCalibrationHarness harness = ExtensionCalibrationHarness.create(config);
    harness.start(true);
    harness.hold(0.6, 10.0, 3.0, 5.0, false);

    assertEquals(Phase.FAILED, harness.calibrator().phase(), harness.trace());
    assertEquals(ExtensionCalibrator.TIMEOUT_ERROR, harness.error());
    assertFalse(harness.zeroed());
  }

  @Test
  void secondSeekGetsItsOwnTimeout() {
    Config config = new Config(-3.0, 35.0, 0.5, 0.3, 0.15, 0.8, 10.0, 1.0);
    ExtensionCalibrationHarness harness = ExtensionCalibrationHarness.create(config);
    harness.start(true);
    harness.seek(new HardStop(4.0, true));
    assertEquals(Phase.SEEK_SECOND, harness.calibrator().phase(), harness.trace());

    harness.hold(0.9, 40.0, 3.0, 5.0, false);
    assertEquals(Phase.FAILED, harness.calibrator().phase(), harness.trace());
    assertEquals(ExtensionCalibrator.TIMEOUT_ERROR, harness.error());
  }

  @Test
  void stallDuringInrushDoesNotCount() {
    ExtensionCalibrationHarness harness = harness();
    harness.start(true);
    harness.hold(0.28, 4.0, 0.0, 80.0, true);

    assertEquals(Phase.SEEK_FIRST, harness.calibrator().phase(), harness.trace());
    assertFalse(harness.calibrator().isFailed());

    harness.seek(new HardStop(4.0, true));
    harness.seek(new HardStop(74.0, false));
    assertTrue(harness.calibrator().isCalibrated(), harness.trace());
  }

  @Test
  void secondStallWaitsUntilTheScrewHasMoved() {
    ExtensionCalibrationHarness harness = harness();
    harness.start(true);
    harness.seek(new HardStop(4.0, true));
    assertEquals(Phase.SEEK_SECOND, harness.calibrator().phase(), harness.trace());

    // Past the inrush window, hard against the other stop, but velocity never left the first one.
    harness.hold(0.6, 74.0, 0.0, 80.0, false);
    assertEquals(Phase.SEEK_SECOND, harness.calibrator().phase(), harness.trace());

    harness.hold(0.1, 30.0, 3.0, 5.0, false);
    harness.holdUntilPhaseChange(0.3, 74.0, 0.0, 80.0, false);
    assertTrue(harness.calibrator().isCalibrated(), harness.trace());
    assertEquals(69.0, harness.calibrator().extendPositionRotations(), kEpsilon);
  }

  @Test
  void currentAndVelocityAtTheThresholdAreNotAStall() {
    ExtensionCalibrationHarness harness = harness();
    harness.start(true);
    harness.hold(0.4, 4.0, 0.5, 35.0, true);
    harness.hold(0.4, 4.0, 0.0, 35.0, true);

    assertEquals(Phase.SEEK_FIRST, harness.calibrator().phase(), harness.trace());
    assertFalse(harness.calibrator().isFailed());
  }

  @Test
  void reversingVoltageIsAppliedOnTheStepAfterTheFirstStop() {
    ExtensionCalibrationHarness harness = harness();
    harness.start(true);
    harness.seek(new HardStop(4.0, true));

    assertEquals(Phase.SEEK_SECOND, harness.calibrator().phase(), harness.trace());
    assertEquals(-3.0, harness.lastVoltage(), kEpsilon);

    harness.hold(0.02, 10.0, 0.0, 0.0, false);
    assertEquals(3.0, harness.lastVoltage(), kEpsilon);
    assertEquals(Phase.SEEK_SECOND, harness.calibrator().phase());
  }

  @Test
  void finishedSweepIsLeftAloneUntilForced() {
    ExtensionCalibrationHarness harness = harness();
    harness.sweep(new HardStop(4.0, true), new HardStop(74.0, false));
    double extend = harness.calibrator().extendPositionRotations();

    harness.start(false);
    assertTrue(harness.calibrator().isCalibrated());
    assertEquals(Phase.DONE, harness.calibrator().phase());
    assertEquals(extend, harness.calibrator().extendPositionRotations(), kEpsilon);

    harness.sweep(new HardStop(0.0, true), new HardStop(-40.0, false));
    assertEquals(-39.0, harness.calibrator().extendPositionRotations(), kEpsilon);
    assertEquals(Phase.DONE, harness.calibrator().phase());
  }

  @Test
  void cancellingClearsAnInProgressSweepAndKeepsAFinishedOne() {
    ExtensionCalibrationHarness harness = harness();
    harness.start(true);
    harness.hold(0.1, 4.0, 3.0, 5.0, false);
    harness.calibrator().onCommandEnd(true);

    assertEquals(Phase.IDLE, harness.calibrator().phase());
    assertFalse(harness.calibrator().isFailed());
    assertFalse(harness.calibrator().isCalibrated());

    harness.sweep(new HardStop(4.0, true), new HardStop(74.0, false));
    harness.calibrator().onCommandEnd(true);
    assertEquals(Phase.DONE, harness.calibrator().phase());
    assertTrue(harness.calibrator().isCalibrated());
  }

  @Test
  void aFailedSweepStaysFailedWhenTheCommandEnds() {
    Config config = new Config(-3.0, 35.0, 0.5, 0.3, 0.15, 0.5, 10.0, 1.0);
    ExtensionCalibrationHarness harness = ExtensionCalibrationHarness.create(config);
    harness.start(true);
    harness.hold(0.6, 10.0, 3.0, 5.0, false);
    harness.calibrator().onCommandEnd(false);

    assertEquals(Phase.FAILED, harness.calibrator().phase());
    assertTrue(harness.calibrator().isFailed());
  }

  @Test
  void assumedCalibrationSkipsTheSweep() {
    ExtensionCalibrator calibrator = new ExtensionCalibrator(config());
    calibrator.assumeCalibrated(1.0, 69.0);
    ExtensionCalibrationHarness harness = new ExtensionCalibrationHarness(calibrator);

    harness.start(false);
    assertTrue(calibrator.isCalibrated());
    assertEquals(69.0, calibrator.extendPositionRotations(), kEpsilon);
    assertEquals(Phase.DONE, calibrator.phase());
    assertFalse(harness.zeroed());
  }
}
