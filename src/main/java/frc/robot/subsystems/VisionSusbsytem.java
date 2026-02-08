// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSusbsytem extends SubsystemBase {
  PhotonCamera m_randomAssCamera = new PhotonCamera("HD_USB_CAMERA");
  //PhotonTrackedTarget bestTarget = unreadResults.get(0).getBestTarget();

  boolean AprilTagSight = false;

  double targetYaw;
  double targetPitch;
  double targetSkew;
  double targetArea;

  /** Creates a new VisionSusbsytem. */
  public VisionSusbsytem() {

  }

  @Override
  public void periodic() {
    
    List<PhotonPipelineResult> unreadResults = m_randomAssCamera.getAllUnreadResults();
    //Checks to see if sigma awesome unread results is not empty.
    if (!unreadResults.isEmpty()) {
      //Always sees the latest entry in the pipeline.
      var result = unreadResults.get(unreadResults.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          targetYaw = target.getYaw();
          targetPitch = target.getPitch();
          targetSkew = target.getSkew();
          targetArea = target.getArea();

        }
      }
    }

    SmartDashboard.putBoolean("April Tag", AprilTagSight);
    SmartDashboard.putNumber("Yaw:", targetYaw);
    SmartDashboard.putNumber("Pitch", targetPitch);
    SmartDashboard.putNumber("Skew", targetSkew);
    SmartDashboard.putNumber("Yaw:", targetArea);

  }
}
