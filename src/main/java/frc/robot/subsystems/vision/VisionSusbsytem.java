// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;

import com.fasterxml.jackson.databind.BeanDescription;

import edu.wpi.first.math.geometry.Pose2d;
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

  private VisionState currentState = VisionState.IDLE;
  private PhotonCamera m_randomAssCamera = new PhotonCamera(CAMERA_NAME);
  private PhotonCameraSim m_randomAssSimCamera = new PhotonCameraSim(m_randomAssCamera);

  /** Creates a new VisionSusbsytem. */
  public VisionSusbsytem() {
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

      switch (currentState) {
        case IDLE:     
          break;
        case SEARCHING:
          if (hasTargets) {
            if (multiTagResult.isPresent()) {
              currentState = VisionState.GLOBAL_POSE;
            }
          } else {currentState = VisionState.ALIGNMENT;}
        case GLOBAL_POSE:
          var bestPNPPose = multiTagResult.get().estimatedPose.best;
          Pose2d robotPose = new Pose2d(
            bestPNPPose.getMeasureX(),
            bestPNPPose.getMeasureY(),
            bestPNPPose.getRotation().toRotation2d()
          );

        case DISTANCE_READ:

        case ALIGNMENT:
          if (!hasTargets) {
            currentState =  VisionState.SEARCHING;
          }
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
}
