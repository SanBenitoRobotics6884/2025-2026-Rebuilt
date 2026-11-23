// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Constants.Intake.kIntakeRollerMinExtension;
import static frc.robot.Constants.Constants.Intake.kTravelRotations;
import static frc.robot.subsystems.IntakeSubsystem.intakeRollerAllowed;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** The intake roller may only spin once the intake is far enough out that its wheels clear. */
class IntakeRollerGateTest {
  static final double kThreshold = kIntakeRollerMinExtension * kTravelRotations;

  @Test
  void blockedWhenRetractedOrPartlyOut() {
    assertFalse(intakeRollerAllowed(true, false, 0.0, 0.0));
    assertFalse(intakeRollerAllowed(true, false, kThreshold - 0.1, kThreshold - 0.1));
  }

  @Test
  void allowedFromTheMinExtensionPointOut() {
    assertTrue(intakeRollerAllowed(true, false, kThreshold, kThreshold));
    assertTrue(intakeRollerAllowed(true, false, kTravelRotations, kTravelRotations));
  }

  @Test
  void usesTheScrewThatIsFurtherIn() {
    assertFalse(intakeRollerAllowed(true, false, kThreshold + 1.0, kThreshold - 0.1));
    assertFalse(intakeRollerAllowed(true, false, kThreshold - 0.1, kThreshold + 1.0));
  }

  @Test
  void extendedSensorAlwaysAllows() {
    assertTrue(intakeRollerAllowed(true, true, 0.0, 0.0));
    assertTrue(intakeRollerAllowed(false, true, 0.0, 0.0));
  }

  @Test
  void notHomedBlocksEvenIfTheEncodersLookFarOut() {
    // Powered up away from S1: the encoders read from wherever it started, so don't trust them.
    assertFalse(intakeRollerAllowed(false, false, kTravelRotations, kTravelRotations));
  }
}
