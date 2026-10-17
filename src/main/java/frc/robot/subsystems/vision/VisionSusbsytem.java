// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.Constants.Constants.Vision.*;

public class VisionSusbsytem extends SubsystemBase {
  
  // FSM PhotonChoice
  public enum VisionState {
    IDLE, // Awaitng for vision command
    SEARCHING, // Looking for specific tag
    GLOBAL_POSE, // Using Global Pose we can chart our robots pose on the field and get the relative distance from the hub using coordinates from above.
    DISTANCE_READ,
    ALIGNMENT
  }

  private CommandSwerveDrivetrain m_drivetrain;
  private VisionState currentState = VisionState.IDLE;
  private PhotonCamera m_randomAssCamera = new PhotonCamera(CAMERA_NAME);
  private PhotonCameraSim m_randomAssSimCamera = new PhotonCameraSim(m_randomAssCamera);
  private Pose2d m_robotPose;

  /** Creates a new VisionSusbsytem. */
  public VisionSusbsytem(CommandSwerveDrivetrain m_drivetrain) {
    this.m_drivetrain = m_drivetrain; // fuck yeah, inject that dependency.
    // Setting specific pipeline index, might need to change depending on goals lowk.
    choosePipeline(0);
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> unreadResults = m_randomAssCamera.getAllUnreadResults();
    if (!unreadResults.isEmpty()) {
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
          } else {currentState = VisionState.ALIGNMENT;}

          break;
        case GLOBAL_POSE:
          if (multiTagResult.isPresent()) {
            var bestPNPPose = multiTagResult.get().estimatedPose.best;
              m_robotPose = new Pose2d(
                bestPNPPose.getMeasureX(),
                bestPNPPose.getMeasureY(),
                bestPNPPose.getRotation().toRotation2d()
              );
             swerveMeasurement(m_drivetrain, m_robotPose, resultTimestamp);  
          } else {currentState = VisionState.SEARCHING;}

          break;
        case DISTANCE_READ:
          
          break;
        case ALIGNMENT:
          if (!hasTargets) {
            currentState =  VisionState.SEARCHING;
          }



          break;
      }
    }
  }

  public void setState(VisionState newState) {
    this.currentState = newState;
  }

  public void fullSnapshot() {
    m_randomAssCamera.takeInputSnapshot();
    m_randomAssCamera.takeOutputSnapshot();
  }
  
  // Needs to called repeatedly to always update with vision
  // However, this makes the idea of a vision "drift" more probable.
  public void swerveMeasurement(CommandSwerveDrivetrain m_drivetrain, Pose2d m_pose2d, EstimatedRobotPose m_estPose) {
    m_drivetrain.addVisionMeasurement(m_pose2d, m_estPose.timestampSeconds); 
  }

  // Returns the VisionState to IDLE, your freaking welcome.
  public void returnToIdle() {
    currentState = VisionState.IDLE;
  }

  public void returnToSearch() {
    currentState = VisionState.SEARCHING;
  }

  public void choosePipeline(int index) {
    m_randomAssCamera.setPipelineIndex(index);
    switch (index) {
      case 0: // VISION PROCESSING PIPELINE
        m_randomAssCamera.setFPSLimit(45);
        break;
      case 1: // DRIVER PROCESSING PIPELINE
        m_randomAssCamera.setFPSLimit(60);
        break;
      default: // ANY INDEX NOT REGISTERED LINKS DIRECTLY BACK TO case:0
        m_randomAssCamera.setPipelineIndex(0);
        break;
    }
  }

  /**
   * Pushes a verified vision target pose into the drivetrain's Kalman Filter odometry.
   * @param m_drivetrain Your drivetrain subsystem reference.
   * @param m_pose2d The calculated global Pose2d of the robot.
   * @param timestampSeconds The camera frame's exact capture timestamp.
   */
  public void swerveMeasurement(CommandSwerveDrivetrain m_drivetrain, Pose2d m_pose2d, double timestampSeconds) {
    // Fuses the calculation directly into the drivetrain's Kalman Filter
    m_drivetrain.addVisionMeasurement(m_pose2d, timestampSeconds); 
  }


  // TRUE -> IDLE, FALSE -> SEARCH
  public Command VisionIndex(boolean isIdle) {
    if (isIdle) {
       return runOnce(() -> returnToIdle());
    } else {
      return runOnce(() -> returnToSearch());
    }
  }
}
