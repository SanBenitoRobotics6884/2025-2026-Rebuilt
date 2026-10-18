// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.Constants.Score.*;
import static frc.robot.Constants.Constants.Vision.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Checks the shot math a novice is most likely to retune: distance, RPM, and when feeding may start. */
class HubMathTest {
  @Test
  void rpmFollowsAStraightLineBetweenTheTwoTunedShots() {
    assertEquals(kCloseRpm, HubMath.rpmForDistance(kCloseDistanceMeters), 1e-6);
    assertEquals(kFarRpm, HubMath.rpmForDistance(kFarDistanceMeters), 1e-6);
    assertEquals(
        (kCloseRpm + kFarRpm) / 2.0,
        HubMath.rpmForDistance((kCloseDistanceMeters + kFarDistanceMeters) / 2.0),
        1e-6);
  }

  @Test
  void rpmKeepsTheSameSlopeOutsideTheTwoTunedShots() {
    double slope = (kFarRpm - kCloseRpm) / (kFarDistanceMeters - kCloseDistanceMeters);
    assertEquals(kCloseRpm - slope, HubMath.rpmForDistance(kCloseDistanceMeters - 1.0), 1e-6);
    assertEquals(kFarRpm + slope, HubMath.rpmForDistance(kFarDistanceMeters + 1.0), 1e-6);
    assertEquals(kMaxShooterRpm, HubMath.rpmForDistance(100.0), 1e-6);
    assertEquals(0.0, HubMath.rpmForDistance(Double.NaN), 1e-6);
    assertEquals(0.0, HubMath.rpmForDistance(0.0), 1e-6);
  }

  @Test
  void feedWaitsUntilTheWheelsReachTheTargetRpm() {
    assertFalse(HubMath.shooterHasReachedRpm(2199.0, 2200.0));
    assertTrue(HubMath.shooterHasReachedRpm(2200.0, 2200.0));
    assertTrue(HubMath.shooterHasReachedRpm(2300.0, 2200.0));
    assertFalse(HubMath.shooterHasReachedRpm(2200.0, 0.0));
  }

  @Test
  void tagOnTheRightTurnsTheRobotRight() {
    // Positive yaw is a tag to the right. Positive swerve rotation is a left turn,
    // so the aim speed has to come back negative.
    assertTrue(HubMath.turnRadiansPerSecond(10.0, 5.0) < 0.0);
    assertTrue(HubMath.turnRadiansPerSecond(-10.0, 5.0) > 0.0);
    assertEquals(0.0, HubMath.turnRadiansPerSecond(kAimOffsetDegrees, 5.0), 1e-9);
    assertEquals(-5.0, HubMath.turnRadiansPerSecond(180.0, 5.0), 1e-9);
  }

  @Test
  void horizontalDistanceUsesTheCameraMountAndIgnoresHeight() {
    double alongTheFloor = HubMath.horizontalDistanceMeters(new Translation3d(2.0, 0.0, 0.0));
    double withHeight = HubMath.horizontalDistanceMeters(new Translation3d(2.0, 0.0, 1.0));

    assertEquals(Math.hypot(CAM_XPOSE + 2.0, CAM_YPOSE), alongTheFloor, 1e-9);
    assertEquals(alongTheFloor, withHeight, 1e-9);
    assertTrue(Double.isNaN(HubMath.horizontalDistanceMeters(new Translation3d())));
  }

  @Test
  void pitchFallbackIsTheHeightTriangle() {
    double expected =
        (HUB_APRIL_TAG_HEIGHT - CAM_HEIGHT) / Math.tan(Units.degreesToRadians(20.0));
    assertEquals(expected, HubMath.distanceFromPitch(20.0), 1e-9);
    assertTrue(Double.isNaN(HubMath.distanceFromPitch(0.0)));
  }
}
