// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.Constants.Score.*;
import static frc.robot.Constants.Constants.Vision.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

/** Checks the shot math: where the hub is, RPM for a distance, aiming, and when feeding may start. */
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
  void feedWaitsUntilTheWheelsAreWithinToleranceOfTheTargetRpm() {
    double ready = 2200.0 - kShooterReadyToleranceRpm;
    assertFalse(HubMath.shooterHasReachedRpm(ready - 1.0, 2200.0));
    assertTrue(HubMath.shooterHasReachedRpm(ready, 2200.0));
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
  void hubCenterIsTheMiddleOfEachHub() {
    // From the 2026 field map: each hub's faces are about 1.2 m apart, and the blue and red hubs
    // mirror each other across the middle of the field.
    assertEquals(FIELD_LAYOUT.getFieldLength(),
        HubMath.BLUE_HUB_CENTER.getX() + HubMath.RED_HUB_CENTER.getX(), 0.01);
    assertEquals(HubMath.BLUE_HUB_CENTER.getY(), HubMath.RED_HUB_CENTER.getY(), 0.01);
    // The hub's middle sits halfway between its two outermost faces.
    double nearest = Double.POSITIVE_INFINITY;
    double farthest = 0;
    for (int id : BLUE_HUB_TAG_IDS) {
      double x = tagPose(id).getX();
      nearest = Math.min(nearest, x);
      farthest = Math.max(farthest, x);
    }
    double faceToFace = farthest - nearest;
    assertTrue(faceToFace > 0.5 && faceToFace < 2.0, "hub width " + faceToFace);
    assertEquals((nearest + farthest) / 2.0, HubMath.BLUE_HUB_CENTER.getX(), 1e-9);
  }

  @Test
  void hubFromAnyTagPointsAtTheSameMiddle() {
    // Robot somewhere in front of the blue hub, turned a little. For every blue hub tag, build the
    // camera's view of that tag from the field geometry, and check the hub's middle comes out
    // where it really is, relative to the robot.
    Pose2d robot = new Pose2d(1.6, 3.4, Rotation2d.fromDegrees(20));
    Pose3d camera = new Pose3d(robot).transformBy(ROBOT_TO_CAMERA);
    Translation2d expected =
        HubMath.BLUE_HUB_CENTER.minus(robot.getTranslation()).rotateBy(robot.getRotation().unaryMinus());
    for (int id : BLUE_HUB_TAG_IDS) {
      Pose3d tag = FIELD_LAYOUT.getTagPose(id).orElseThrow();
      Transform3d cameraToTag = new Transform3d(camera, tag);
      Translation2d hub = HubMath.hubFromRobot(cameraToTag, tag.toPose2d(), HubMath.BLUE_HUB_CENTER);
      assertEquals(expected.getX(), hub.getX(), 1e-6, "tag " + id);
      assertEquals(expected.getY(), hub.getY(), 1e-6, "tag " + id);
    }
  }

  @Test
  void hubStraightAheadIsZeroYawAndRightIsPositive() {
    assertEquals(0.0, HubMath.yawDegrees(new Translation2d(3.0, 0.0)), 1e-9);
    assertTrue(HubMath.yawDegrees(new Translation2d(3.0, -0.5)) > 0.0); // -Y is to the right
    assertTrue(HubMath.yawDegrees(new Translation2d(3.0, 0.5)) < 0.0);
  }

  @Test
  void noThreeDimensionalSolveGivesNoHub() {
    Pose2d tag = tagPose(BLUE_HUB_TAG_IDS[0]);
    assertNull(HubMath.hubFromRobot(new Transform3d(), tag, HubMath.BLUE_HUB_CENTER));
  }

  @Test
  void shotModeUsesTheTagThenARecentPositionThenTheFixedSpeed() {
    double fresh = kPoseFallbackMaxAgeSeconds - 1.0;
    double stale = kPoseFallbackMaxAgeSeconds + 1.0;
    // In our zone.
    assertEquals(HubMath.ShotMode.AUTO_AIM, HubMath.shotMode(true, true, stale, Double.NaN, true));
    assertEquals(HubMath.ShotMode.POSE, HubMath.shotMode(false, false, fresh, 3.0, true));
    assertEquals(HubMath.ShotMode.FIXED, HubMath.shotMode(false, false, stale, 3.0, true));
    assertEquals(HubMath.ShotMode.FIXED,
        HubMath.shotMode(false, false, Double.POSITIVE_INFINITY, 3.0, true));
    assertEquals(HubMath.ShotMode.FIXED,
        HubMath.shotMode(false, false, fresh, Double.NaN, true)); // no alliance
  }

  @Test
  void outsideOurZoneIsAlwaysFixed() {
    double fresh = kPoseFallbackMaxAgeSeconds - 1.0;
    // Hub tag in view but the robot is outside our zone: no auto-aim (G407).
    assertEquals(HubMath.ShotMode.FIXED, HubMath.shotMode(true, false, fresh, 3.0, true));
    // No tag, trusted position, but outside our zone.
    assertEquals(HubMath.ShotMode.FIXED, HubMath.shotMode(false, false, fresh, 3.0, false));
  }

  @Test
  void allianceZoneIsMeasuredFromOurOwnWall() {
    double length = FIELD_LAYOUT.getFieldLength();
    double y = 4.0;
    assertTrue(HubMath.inAllianceZone(new Translation2d(kAllianceZoneDepthMeters - 0.01, y), false));
    assertFalse(HubMath.inAllianceZone(new Translation2d(kAllianceZoneDepthMeters + 0.01, y), false));
    assertTrue(HubMath.inAllianceZone(
        new Translation2d(length - kAllianceZoneDepthMeters + 0.01, y), true));
    assertFalse(HubMath.inAllianceZone(
        new Translation2d(length - kAllianceZoneDepthMeters - 0.01, y), true));
    // Our own zone only: blue's zone is not red's.
    assertFalse(HubMath.inAllianceZone(new Translation2d(1.0, y), true));
    // The hub's alliance-wall face sits on the zone's edge (manual 5.4).
    assertEquals(kAllianceZoneDepthMeters, tagPose(26).getX(), 0.05);
  }

  @Test
  void robotFieldPoseFromAnyHubTag() {
    Pose2d robot = new Pose2d(2.1, 5.2, Rotation2d.fromDegrees(-25));
    Pose3d camera = new Pose3d(robot).transformBy(ROBOT_TO_CAMERA);
    for (int id : BLUE_HUB_TAG_IDS) {
      Pose3d tag = FIELD_LAYOUT.getTagPose(id).orElseThrow();
      Pose2d found = HubMath.robotFieldPose(new Transform3d(camera, tag), tag.toPose2d());
      assertEquals(robot.getX(), found.getX(), 1e-6, "tag " + id);
      assertEquals(robot.getY(), found.getY(), 1e-6, "tag " + id);
      assertEquals(robot.getRotation().getDegrees(), found.getRotation().getDegrees(), 1e-6);
    }
  }

  static Pose2d tagPose(int id) {
    return FIELD_LAYOUT.getTagPose(id).orElseThrow().toPose2d();
  }
}
