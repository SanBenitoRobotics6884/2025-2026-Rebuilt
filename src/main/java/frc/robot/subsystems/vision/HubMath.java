// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.Constants.Score.*;
import static frc.robot.Constants.Constants.Vision.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The arithmetic for shooting at the hub. No motors and no camera live here, so the steps can be
 * read on their own.
 *
 * <ol>
 *   <li>Find the middle of the hub relative to the robot, from any hub AprilTag the camera sees.
 *   <li>Turn that distance into a shooter RPM. The same straight line is used at every distance,
 *       so a measured distance is not replaced with a closer or farther one.
 *   <li>Turn until the middle of the hub is straight ahead.
 *   <li>Feed a ball only once the wheels are close to that RPM.
 * </ol>
 */
public final class HubMath {
  private HubMath() {}

  /** Middle of the blue hub on the floor, in field coordinates. */
  public static final Translation2d BLUE_HUB_CENTER = hubCenter(BLUE_HUB_TAG_IDS);

  /** Middle of the red hub on the floor, in field coordinates. */
  public static final Translation2d RED_HUB_CENTER = hubCenter(RED_HUB_TAG_IDS);

  /** Middle of our alliance's hub. */
  public static Translation2d hubCenter(boolean red) {
    return red ? RED_HUB_CENTER : BLUE_HUB_CENTER;
  }

  /** How the right-trigger shot picks its speed and who aims. */
  public enum ShotMode {
    /** In our zone with a hub tag in view: the robot aims itself, the distance sets the RPM. */
    AUTO_AIM,
    /** In our zone (by the trusted tracked position), no hub tag: the driver aims. */
    POSE,
    /** Outside our zone, or no way to tell: fixed speed, the driver aims. Also the pass shot. */
    FIXED
  }

  /**
   * Picks the shot mode. Scoring is only legal from our ALLIANCE ZONE (rule G407), so both
   * distance-based modes also require knowing the robot is in it.
   *
   * @param hubInView a hub tag was seen within kMaxSightingAgeSeconds
   * @param sightingInZone the robot's position worked out from that hub tag is in our zone
   * @param secondsSincePoseFix time since a vision pose last agreed with the tracked position
   * @param poseDistanceMeters distance from the tracked position to our hub's middle, or NaN when
   *     unknown (no alliance)
   * @param poseInZone the tracked position is in our zone
   */
  public static ShotMode shotMode(
      boolean hubInView,
      boolean sightingInZone,
      double secondsSincePoseFix,
      double poseDistanceMeters,
      boolean poseInZone) {
    if (hubInView) {
      // The hub tag places the robot more surely than the tracked position, so it decides.
      return sightingInZone ? ShotMode.AUTO_AIM : ShotMode.FIXED;
    }
    if (secondsSincePoseFix <= kPoseFallbackMaxAgeSeconds
        && Double.isFinite(poseDistanceMeters)
        && poseDistanceMeters > 0
        && poseInZone) {
      return ShotMode.POSE;
    }
    return ShotMode.FIXED;
  }

  /**
   * True when the robot's center is inside our ALLIANCE ZONE: within kAllianceZoneDepthMeters of
   * our own alliance wall (blue at x = 0, red at the far end).
   */
  public static boolean inAllianceZone(Translation2d robotCenter, boolean red) {
    return red
        ? robotCenter.getX() >= FIELD_LAYOUT.getFieldLength() - kAllianceZoneDepthMeters
        : robotCenter.getX() <= kAllianceZoneDepthMeters;
  }

  /**
   * Where the robot is on the field, worked out from one tag's 3D solve and where the field map
   * puts that tag. Independent of the tracked position, so it is right even before vision has
   * corrected odometry.
   */
  public static Pose2d robotFieldPose(Transform3d cameraToTag, Pose2d tagFieldPose) {
    Transform3d robotToTag = ROBOT_TO_CAMERA.plus(cameraToTag);
    Transform2d robotToTag2d =
        new Transform2d(
            robotToTag.getTranslation().toTranslation2d(),
            new Rotation2d(robotToTag.getRotation().getZ()));
    return tagFieldPose.transformBy(robotToTag2d.inverse());
  }

  /**
   * Middle of the hub that carries these tags: halfway between its opposite faces, from where the
   * field map puts each tag.
   */
  public static Translation2d hubCenter(int[] tagIds) {
    double minX = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    for (int id : tagIds) {
      Translation2d tag = FIELD_LAYOUT.getTagPose(id).orElseThrow().toPose2d().getTranslation();
      minX = Math.min(minX, tag.getX());
      maxX = Math.max(maxX, tag.getX());
      minY = Math.min(minY, tag.getY());
      maxY = Math.max(maxY, tag.getY());
    }
    return new Translation2d((minX + maxX) / 2.0, (minY + maxY) / 2.0);
  }

  /**
   * Where the middle of the hub is, relative to the robot center, from one tag's 3D solve.
   *
   * <p>The camera reports where the tag is and which way it faces. {@code ROBOT_TO_CAMERA} turns
   * that into robot coordinates. The field map says where the hub's middle is relative to that
   * tag, so adding that offset (turned the way the tag faces) gives the hub's middle. This works
   * from any face of the hub, and it already accounts for the camera being off-center.
   *
   * @param cameraToTag the tag's position and facing as the camera solved it
   * @param tagFieldPose where the field map puts that tag
   * @param hubCenterField the hub's middle, in field coordinates
   * @return the hub's middle in robot coordinates (+X forward, +Y left, meters), or null when the
   *     camera did not solve a 3D position (pipeline not in 3D mode)
   */
  public static Translation2d hubFromRobot(
      Transform3d cameraToTag, Pose2d tagFieldPose, Translation2d hubCenterField) {
    if (cameraToTag.getTranslation().getNorm() < 1e-3) {
      return null;
    }
    Transform3d robotToTag = ROBOT_TO_CAMERA.plus(cameraToTag);
    Translation2d tagFromRobot = robotToTag.getTranslation().toTranslation2d();
    Rotation2d tagFacingInRobot = new Rotation2d(robotToTag.getRotation().getZ());
    Translation2d hubFromTag =
        hubCenterField
            .minus(tagFieldPose.getTranslation())
            .rotateBy(tagFieldPose.getRotation().unaryMinus());
    return tagFromRobot.plus(hubFromTag.rotateBy(tagFacingInRobot));
  }

  /**
   * Degrees the hub is off straight ahead, positive to the right (the same sign PhotonVision uses
   * for a tag's yaw).
   */
  public static double yawDegrees(Translation2d hubFromRobot) {
    return -Math.toDegrees(Math.atan2(hubFromRobot.getY(), hubFromRobot.getX()));
  }

  /**
   * Shooter speed, in rotations per minute, for a floor distance.
   *
   * <p>Two shots are tuned by hand: a close one and a far one. The RPM changes by the same amount
   * for every extra meter, including distances closer than the close shot and farther than the
   * far shot. The measured distance is used as-is. An unknown or non-positive distance returns 0,
   * which means "do not shoot." The result is capped at {@link
   * frc.robot.Constants.Constants.Score#kMaxShooterRpm} so a bad camera reading cannot ask the
   * motor for an endless speed.
   */
  public static double rpmForDistance(double distanceMeters) {
    if (!Double.isFinite(distanceMeters) || distanceMeters <= 0) {
      return 0;
    }
    double span = kFarDistanceMeters - kCloseDistanceMeters;
    if (Math.abs(span) < 1e-6) {
      return kCloseRpm;
    }

    // Start at the close shot's RPM. Add the same RPM for every meter past that shot.
    // Closer than the close shot, metersPastClose is negative and the RPM comes down.
    double metersPastClose = distanceMeters - kCloseDistanceMeters;
    double rpmPerMeter = (kFarRpm - kCloseRpm) / span;
    double rpm = kCloseRpm + metersPastClose * rpmPerMeter;
    return MathUtil.clamp(rpm, 0.0, kMaxShooterRpm);
  }

  /**
   * True when the wheels have climbed to within kShooterReadyToleranceRpm of {@code targetRpm}.
   *
   * <p>This is the only reason to wait before feeding. The shooter keeps accelerating for a while
   * after we ask for a new speed, and a ball that enters during that climb comes out short.
   * {@code measuredRpm} is the speed the motor reports right now, not the speed we asked for.
   */
  public static boolean shooterHasReachedRpm(double measuredRpm, double targetRpm) {
    return targetRpm > 0 && measuredRpm >= targetRpm - kShooterReadyToleranceRpm;
  }

  /**
   * How far off center the hub is, after kAimOffsetDegrees. Positive means the hub is to the
   * right of where we want it.
   */
  public static double aimErrorDegrees(double yawDegrees) {
    return yawDegrees - kAimOffsetDegrees;
  }

  /**
   * Turn speed, in radians per second, that aims the robot at the middle of the hub.
   *
   * <p>PhotonVision's yaw is positive when the tag is to the right of the camera. The swerve
   * drive treats a positive rotation as counter-clockwise, which is a left turn. The minus sign
   * makes a tag on the right produce a right turn. The result is capped at the drivetrain's
   * maximum so a tag at the edge of the picture cannot demand a faster spin than the robot allows.
   */
  public static double turnRadiansPerSecond(double yawDegrees, double maxTurnRadiansPerSecond) {
    double errorRadians = Units.degreesToRadians(aimErrorDegrees(yawDegrees));
    double turn = -kTurnP * errorRadians;
    return MathUtil.clamp(turn, -maxTurnRadiansPerSecond, maxTurnRadiansPerSecond);
  }
}
