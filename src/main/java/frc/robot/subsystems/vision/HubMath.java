// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.Constants.Score.*;
import static frc.robot.Constants.Constants.Vision.*;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The arithmetic for shooting at one hub AprilTag. No motors and no camera live here, so the
 * steps can be read on their own.
 *
 * <ol>
 *   <li>Measure the distance from the robot to the tag, along the floor.
 *   <li>Turn that distance into a shooter RPM. The same straight line is used at every distance,
 *       so a measured distance is not replaced with a closer or farther one.
 *   <li>Turn toward the tag until it sits in the middle of the camera.
 *   <li>Feed a ball only once the wheels have actually reached that RPM. They are heavy and take
 *       time to speed up. Feeding earlier throws the ball short.
 * </ol>
 */
public final class HubMath {
  private HubMath() {}

  /**
   * Floor distance from the center of the robot to this tag, in meters.
   *
   * <p>The camera's 3D solve is used first. That offset is in camera coordinates (forward, left,
   * up), so it is rotated into the robot's coordinates and the height is ignored. What is left is
   * the distance along the carpet, which is what the shot arc depends on.
   *
   * <p>If the camera did not solve a 3D position, this falls back to the old triangle: the tag is
   * a known height above the camera, and the camera reports the angle up to the tag. Not-a-number
   * means we still know which way to turn, but we do not know how hard to shoot.
   */
  public static double distanceMeters(PhotonTrackedTarget target) {
    double fromCamera = horizontalDistanceMeters(target.getBestCameraToTarget().getTranslation());
    if (Double.isFinite(fromCamera)) {
      return fromCamera;
    }
    return distanceFromPitch(target.getPitch());
  }

  /**
   * Floor distance for a camera-to-tag offset. The offset is in the camera's coordinates.
   * Returns not-a-number when the camera did not actually solve a position.
   */
  public static double horizontalDistanceMeters(Translation3d cameraToTag) {
    if (cameraToTag.getNorm() < 1e-3) {
      return Double.NaN;
    }

    // ROBOT_TO_CAMERA is where the Limelight is bolted. Rotating the offset puts it in the robot's
    // coordinates, and adding the bolt position moves the measurement from "in front of the lens"
    // to "from the center of the robot."
    Translation3d fromRobot =
        ROBOT_TO_CAMERA
            .getTranslation()
            .plus(cameraToTag.rotateBy(ROBOT_TO_CAMERA.getRotation()));
    return Math.hypot(fromRobot.getX(), fromRobot.getY());
  }

  /**
   * Backup distance, in meters, from the angle of the tag in the picture.
   *
   * <p>{@code targetPitchDegrees} is positive when the tag is above the crosshair. The camera
   * height and the tag height are constants, so the floor distance is the adjacent side of a
   * right triangle: opposite (height) divided by tan(angle).
   */
  public static double distanceFromPitch(double targetPitchDegrees) {
    double angleRadians = HUB_CAM_PITCH + Units.degreesToRadians(targetPitchDegrees);
    // A tag straight ahead, level with the camera, makes this triangle unusable.
    if (Math.abs(angleRadians) < Units.degreesToRadians(1.0)) {
      return Double.NaN;
    }
    return (HUB_APRIL_TAG_HEIGHT - CAM_HEIGHT) / Math.tan(angleRadians);
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
   * True when the wheels have climbed all the way to {@code targetRpm}.
   *
   * <p>This is the only reason to wait before feeding. The shooter keeps accelerating for a while
   * after we ask for a new speed, and a ball that enters during that climb comes out short.
   * {@code measuredRpm} is the speed the motor reports right now, not the speed we asked for.
   */
  public static boolean shooterHasReachedRpm(double measuredRpm, double targetRpm) {
    return targetRpm > 0 && measuredRpm >= targetRpm;
  }

  /**
   * How far off center the tag is, after the shooter-to-camera offset. Positive means the tag is
   * to the right of where we want it.
   */
  public static double aimErrorDegrees(double yawDegrees) {
    return yawDegrees - kAimOffsetDegrees;
  }

  /**
   * Turn speed, in radians per second, that aims the robot at the tag.
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
