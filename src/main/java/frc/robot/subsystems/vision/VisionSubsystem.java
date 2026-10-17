// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.Constants.Vision.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Turns Limelight pictures into a field position and gently corrects swerve odometry with it.
 *
 * <p>What is happening, in order:
 *
 * <ol>
 *   <li>PhotonVision is running on the Limelight, not on the RoboRIO. It finds AprilTags in the
 *       camera image and sends the results over NetworkTables. This class only reads those results.
 *   <li>A tag tells us where the <b>camera</b> is. {@code ROBOT_TO_CAMERA} shifts that into where
 *       the <b>robot</b> is, because the Limelight is not in the center of the robot.
 *   <li>The field layout says where every tag is bolted to the field. Comparing the tags we see
 *       with that map produces a field pose (x, y, and heading in meters and degrees).
 *   <li>We throw out poses that are probably wrong: unknown tags, off the field, too old, a single
 *       far-away tag, or a single tag that can be solved in two different places.
 *   <li>A good pose is handed to the swerve pose estimator. Wheel odometry still tracks motion
 *       between camera frames. Vision only nudges that estimate back toward the tags. The Pigeon
 *       keeps ownership of heading, because a camera heading is much noisier than the gyro.
 * </ol>
 *
 * <p>This class is a {@link SubsystemBase}, so {@link #periodic()} runs automatically about 50
 * times a second. Nothing has to press a button to start it.
 *
 * <p>Watch it on SmartDashboard under {@code Vision/}. {@code Vision/Status} is the short version
 * of what the last frame did. {@code Vision Field} is the raw camera pose, drawn separately from
 * the fused swerve pose.
 */
public class VisionSubsystem extends SubsystemBase {
  /**
   * One camera pose that passed the filters and was sent to the drivetrain.
   *
   * <p>Drive code that needs the camera's opinion (aiming at the hub, for example) should read
   * {@link #getLatestMeasurement()}. Check {@code timestampSeconds} first. A pose from half a
   * second ago is where the robot <b>was</b>, not where it is.
   *
   * @param pose field pose of the robot. The origin is the blue alliance wall, no matter which
   *     alliance we are on
   * @param timestampSeconds when the photo was taken, on the RoboRIO clock. This is earlier than
   *     "now" by the camera's latency
   * @param standardDeviations how many meters (and radians) this pose might be wrong by. A bigger
   *     number means "trust the wheels more than this picture"
   * @param tagCount how many recognized AprilTags were in the picture
   * @param multiTag true when two or more tags were solved together. That is more accurate than
   *     one tag
   * @param ambiguity how unsure a single-tag solve is, from 0 (sure) to 1 (could be two places).
   *     Negative means the camera could not score it
   */
  public record VisionMeasurement(
      Pose2d pose,
      double timestampSeconds,
      Matrix<N3, N1> standardDeviations,
      int tagCount,
      boolean multiTag,
      double ambiguity) {}

  private final CommandSwerveDrivetrain drivetrain;

  // NetworkTables client. The name must match the camera nickname in the PhotonVision dashboard,
  // or this object is connected to nothing and isConnected() stays false.
  private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);

  // Knows the field map and where the Limelight is bolted on. Given a camera result, it returns
  // the robot's field pose.
  private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(FIELD_LAYOUT, ROBOT_TO_CAMERA);

  // A second field widget so we can see the camera pose and the swerve pose at the same time.
  // If they disagree by a lot, the camera mount numbers or the field layout are wrong.
  private final Field2d field = new Field2d();

  // Fake camera used only while simulating. Null on a real robot.
  private final VisionSystemSim visionSim;

  // Empty until the first picture survives the filters. Stays at the last good pose after that,
  // even if later pictures are rejected.
  private Optional<VisionMeasurement> latestMeasurement = Optional.empty();
  private String rejectReason = "rejected";

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // Driver mode shows the raw image and stops detecting tags. Force the AprilTag pipeline so a
    // dashboard left in driver mode cannot silently disable vision for a match.
    camera.setDriverMode(false);
    camera.setPipelineIndex(APRILTAG_PIPELINE_INDEX);
    SmartDashboard.putData("Vision Field", field);

    if (RobotBase.isSimulation()) {
      // There is no Limelight in simulation. This builds a pretend one that can "see" the same
      // AprilTags, from the robot pose the swerve simulator reports.
      visionSim = new VisionSystemSim("limelight");
      visionSim.addAprilTags(FIELD_LAYOUT);
      var cameraProperties = new SimCameraProperties();
      cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
      cameraProperties.setCalibError(0.35, 0.10);
      cameraProperties.setFPS(30);
      cameraProperties.setAvgLatencyMs(35);
      cameraProperties.setLatencyStdDevMs(10);
      var cameraSim = new PhotonCameraSim(camera, cameraProperties);
      visionSim.addCamera(cameraSim, ROBOT_TO_CAMERA);
      cameraSim.enableDrawWireframe(true);
    } else {
      visionSim = null;
    }
  }

  /**
   * Latest pose that was fused into the drivetrain. Empty until the first accepted frame.
   * Use {@code timestampSeconds} to ignore a measurement that is too old for aiming.
   */
  public Optional<VisionMeasurement> getLatestMeasurement() {
    return latestMeasurement;
  }

  /**
   * Runs every robot cycle. Reads whatever pictures the Limelight has finished since last time,
   * estimates a pose for each one, and fuses the ones that look trustworthy.
   */
  @Override
  public void periodic() {
    // PhotonVision publishes a connected flag. False means the Limelight is off, unplugged, or
    // the camera nickname in Constants does not match the dashboard.
    boolean connected = camera.isConnected();
    SmartDashboard.putBoolean("Vision/Connected", connected);
    if (!connected) {
      publishStatus("disconnected", 0, false, 0);
      return;
    }

    // The Limelight can finish several frames between robot cycles. Read all of them, oldest
    // first, so the pose estimator can rewind the robot to where it was when each photo was taken.
    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
    if (unreadResults.isEmpty()) {
      // Not a failure. The camera is just slower than the 50 Hz robot loop.
      SmartDashboard.putString("Vision/Status", "no new frames");
      return;
    }

    String status = "no targets";
    int tagCount = 0;
    boolean multiTag = false;
    double ambiguity = 0;
    for (var result : unreadResults) {
      if (!result.hasTargets()) {
        // Picture arrived, but no AprilTag was in view. Wheels keep tracking on their own.
        status = "no targets";
        continue;
      }

      // Best case: the Limelight saw 2+ tags and already solved them together, on the coprocessor.
      // That uses the real geometry of the tags, so the pose is much harder to flip.
      Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(result);
      multiTag = estimate.isPresent();
      if (estimate.isEmpty()) {
        // Only one tag, or multi-tag was not enabled on the Limelight. Fall back to the single
        // tag whose solve is least likely to have two possible positions.
        estimate = poseEstimator.estimateLowestAmbiguityPose(result);
        multiTag = false;
      }
      if (estimate.isEmpty()) {
        status = "no pose";
        continue;
      }

      var targets = result.getTargets();
      tagCount = countKnownTags(targets);
      ambiguity = lowestAmbiguity(targets);
      var accepted = accept(estimate.get(), targets, tagCount, multiTag, ambiguity);
      if (accepted.isEmpty()) {
        status = rejectReason;
        continue;
      }

      VisionMeasurement measurement = accepted.get();
      latestMeasurement = Optional.of(measurement);

      // Hand the pose to CTRE's pose estimator. The timestamp lets it apply the correction at the
      // moment the photo was taken, not at the moment we finished reading it. The standard
      // deviations tell it how hard to pull odometry toward the camera.
      drivetrain.addVisionMeasurement(
          measurement.pose(), measurement.timestampSeconds(), measurement.standardDeviations());
      field.setRobotPose(measurement.pose());
      status = "accepted";
      tagCount = measurement.tagCount();
      multiTag = measurement.multiTag();
      ambiguity = measurement.ambiguity();
    }

    publishStatus(status, tagCount, multiTag, ambiguity);
  }

  /**
   * Simulation only. Moves the fake Limelight to wherever the simulated swerve drive says the
   * robot is, so the next {@link #periodic()} has tags to see.
   */
  public void simulationPeriodic(Pose2d robotPose) {
    if (visionSim != null) {
      visionSim.update(robotPose);
    }
  }

  /**
   * Decides whether this pose is safe to fuse. Returning empty does not stop the robot. It only
   * means "ignore this picture and keep using wheel odometry."
   */
  private Optional<VisionMeasurement> accept(
      EstimatedRobotPose estimate,
      List<PhotonTrackedTarget> targets,
      int tagCount,
      boolean multiTag,
      double ambiguity) {
    // A tag id that is not on the 2026 field map cannot be turned into a field position.
    // This also catches a layout mismatch, like welded tags in code and AndyMark tags on the camera.
    if (tagCount == 0) {
      rejectReason = "rejected: unknown tag";
      return Optional.empty();
    }

    Pose2d pose = estimate.estimatedPose.toPose2d();

    // A solve that puts the robot in the stands is a bad solve. One meter of margin covers being
    // right against the wall without accepting a pose from the next field over.
    if (!onField(pose)) {
      rejectReason = "rejected: off field";
      return Optional.empty();
    }

    // latency is how long ago the photo was taken. The drivetrain only remembers a short history
    // of poses, so a late result has nowhere to be inserted. A negative latency means the
    // timestamp is in the future, which is also nonsense.
    double latency = Timer.getFPGATimestamp() - estimate.timestampSeconds;
    if (latency < -0.05 || latency > MAX_MEASUREMENT_LATENCY_SECONDS) {
      rejectReason = "rejected: stale";
      return Optional.empty();
    }

    // One tag seen at an angle often has two legal 3D solutions, a real one and a mirrored one.
    // Ambiguity near 0 means those two solutions disagree, so we can tell them apart. Ambiguity
    // near 1 means they look the same and the camera may have picked the wrong one. Multi-tag
    // solves do not have this problem, so they skip this check.
    if (!multiTag && (ambiguity < 0 || ambiguity > MAX_SINGLE_TAG_AMBIGUITY)) {
      rejectReason = "rejected: ambiguity";
      return Optional.empty();
    }

    Matrix<N3, N1> standardDeviations = standardDeviations(estimate, targets, tagCount);
    if (standardDeviations == null) {
      rejectReason = "rejected: too far";
      return Optional.empty();
    }

    return Optional.of(
        new VisionMeasurement(
            pose, estimate.timestampSeconds, standardDeviations, tagCount, multiTag, ambiguity));
  }

  /**
   * How hard the drivetrain should trust this picture.
   *
   * <p>The three numbers are "expected error" in x meters, y meters, and heading radians. The pose
   * estimator treats them like a weight. Small numbers pull odometry toward the camera. Large
   * numbers leave odometry almost alone.
   *
   * <p>Two or more tags start more trusted than one tag. Trust gets worse as the tags get farther
   * away, because a one-pixel error in the image is a big position error at long range. A single
   * tag past {@link frc.robot.Constants.Constants.Vision#MAX_SINGLE_TAG_DISTANCE_METERS} is dropped
   * instead of fused. Heading stays loose either way, so vision cannot fight the Pigeon.
   *
   * @return null when the picture should not be fused at all
   */
  private Matrix<N3, N1> standardDeviations(
      EstimatedRobotPose estimate, List<PhotonTrackedTarget> targets, int tagCount) {
    if (tagCount == 0) {
      return null;
    }

    // Average distance from the robot to each tag it can actually name. Unknown ids are skipped
    // so a stray detection does not dilute the distance.
    double averageDistance = 0;
    for (var target : targets) {
      var tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) {
        continue;
      }
      averageDistance +=
          tagPose
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(estimate.estimatedPose.toPose2d().getTranslation());
    }
    averageDistance /= tagCount;

    if (tagCount == 1 && averageDistance > MAX_SINGLE_TAG_DISTANCE_METERS) {
      return null;
    }

    var standardDeviations = tagCount > 1 ? MULTI_TAG_STD_DEVS : SINGLE_TAG_STD_DEVS;
    // Grows with distance squared. A tag twice as far is trusted a lot less, not just twice less.
    return standardDeviations.times(1 + (averageDistance * averageDistance / 30));
  }

  /** How many detected tags exist in the loaded field layout. */
  private int countKnownTags(List<PhotonTrackedTarget> targets) {
    int count = 0;
    for (var target : targets) {
      if (poseEstimator.getFieldTags().getTagPose(target.getFiducialId()).isPresent()) {
        count++;
      }
    }
    return count;
  }

  /**
   * Smallest ambiguity among targets that actually have a score. PhotonVision uses a negative
   * number when it could not compute one, and those are ignored here. If every target is negative,
   * this returns -1 and a single-tag pose is rejected.
   */
  private static double lowestAmbiguity(List<PhotonTrackedTarget> targets) {
    double lowest = Double.POSITIVE_INFINITY;
    for (var target : targets) {
      double ambiguity = target.getPoseAmbiguity();
      if (ambiguity >= 0) {
        lowest = Math.min(lowest, ambiguity);
      }
    }
    return Double.isFinite(lowest) ? lowest : -1;
  }

  /** True when the pose is on the field carpet, plus a one meter margin. */
  private static boolean onField(Pose2d pose) {
    double margin = 1.0;
    return pose.getX() >= -margin
        && pose.getX() <= FIELD_LAYOUT.getFieldLength() + margin
        && pose.getY() >= -margin
        && pose.getY() <= FIELD_LAYOUT.getFieldWidth() + margin;
  }

  /** Mirrors the decision from this cycle onto SmartDashboard. */
  private void publishStatus(String status, int tagCount, boolean multiTag, double ambiguity) {
    SmartDashboard.putString("Vision/Status", status);
    SmartDashboard.putNumber("Vision/TagCount", tagCount);
    SmartDashboard.putBoolean("Vision/MultiTag", multiTag);
    SmartDashboard.putNumber("Vision/Ambiguity", ambiguity);
    latestMeasurement.ifPresent(
        measurement -> {
          SmartDashboard.putNumber("Vision/PoseX", measurement.pose().getX());
          SmartDashboard.putNumber("Vision/PoseY", measurement.pose().getY());
          SmartDashboard.putNumber(
              "Vision/PoseDegrees", measurement.pose().getRotation().getDegrees());
          // How old the pose on the dashboard is. Climbing latency usually means the Limelight
          // is overloaded or the NetworkTables link is slow.
          SmartDashboard.putNumber(
              "Vision/LatencySeconds", Timer.getFPGATimestamp() - measurement.timestampSeconds());
        });
  }
}
