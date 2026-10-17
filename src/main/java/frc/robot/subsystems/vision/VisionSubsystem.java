// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.Constants.Vision.*;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {
  /** PhotonVision processing state. */
  public enum VisionState {
    /** Waiting for a vision command. */
    IDLE,
    /** Looking for a specific tag. */
    SEARCHING,
    /** Fusing a global pose into drivetrain odometry. */
    GLOBAL_POSE,
    /** Reading distance to the hub. */
    DISTANCE_READ,
    /** Aligning to a target. */
    ALIGNMENT
  }

  private final CommandSwerveDrivetrain drivetrain;
  private VisionState currentState = VisionState.IDLE;
  private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);
  private final PhotonCameraSim simCamera = new PhotonCameraSim(camera);
  private Pose2d robotPose;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    choosePipeline(0);
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
    if (unreadResults.isEmpty()) {
      return;
    }

    var result = unreadResults.get(unreadResults.size() - 1);
    var multiTagResult = result.getMultiTagResult();
    boolean hasTargets = result.hasTargets();
    double resultTimestamp = result.getTimestampSeconds();

    switch (currentState) {
      case IDLE:
        break;
      case SEARCHING:
        if (hasTargets) {
          if (multiTagResult.isPresent()) {
            currentState = VisionState.GLOBAL_POSE;
          }
        } else {
          currentState = VisionState.ALIGNMENT;
        }
        break;
      case GLOBAL_POSE:
        if (multiTagResult.isPresent()) {
          var bestPnpPose = multiTagResult.get().estimatedPose.best;
          robotPose = new Pose2d(
              bestPnpPose.getMeasureX(),
              bestPnpPose.getMeasureY(),
              bestPnpPose.getRotation().toRotation2d());
          swerveMeasurement(drivetrain, robotPose, resultTimestamp);
        } else {
          currentState = VisionState.SEARCHING;
        }
        break;
      case DISTANCE_READ:
        break;
      case ALIGNMENT:
        if (!hasTargets) {
          currentState = VisionState.SEARCHING;
        }
        break;
      default:
        break;
    }
  }

  public void setState(VisionState newState) {
    this.currentState = newState;
  }

  public void fullSnapshot() {
    camera.takeInputSnapshot();
    camera.takeOutputSnapshot();
  }

  /**
   * Pushes a verified vision target pose into the drivetrain's Kalman filter odometry.
   *
   * @param drivetrain drivetrain subsystem reference
   * @param pose calculated global pose of the robot
   * @param estimatedPose vision estimate whose timestamp is applied
   */
  public void swerveMeasurement(
      CommandSwerveDrivetrain drivetrain, Pose2d pose, EstimatedRobotPose estimatedPose) {
    drivetrain.addVisionMeasurement(pose, estimatedPose.timestampSeconds);
  }

  /** Returns vision processing to idle. */
  public void returnToIdle() {
    currentState = VisionState.IDLE;
  }

  public void returnToSearch() {
    currentState = VisionState.SEARCHING;
  }

  public void choosePipeline(int index) {
    camera.setPipelineIndex(index);
    switch (index) {
      case 0:
        camera.setFPSLimit(45);
        break;
      case 1:
        camera.setFPSLimit(60);
        break;
      default:
        camera.setPipelineIndex(0);
        break;
    }
  }

  /**
   * Pushes a verified vision target pose into the drivetrain's Kalman filter odometry.
   *
   * @param drivetrain drivetrain subsystem reference
   * @param pose calculated global pose of the robot
   * @param timestampSeconds camera frame capture timestamp
   */
  public void swerveMeasurement(
      CommandSwerveDrivetrain drivetrain, Pose2d pose, double timestampSeconds) {
    drivetrain.addVisionMeasurement(pose, timestampSeconds);
  }

  /** Idle when {@code isIdle} is true, otherwise searching. */
  public Command visionIndex(boolean isIdle) {
    if (isIdle) {
      return runOnce(this::returnToIdle);
    } else {
      return runOnce(this::returnToSearch);
    }
  }
}
