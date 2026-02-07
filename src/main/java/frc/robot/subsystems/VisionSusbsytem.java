// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSusbsytem extends SubsystemBase {
  PhotonCamera m_randomAssCamera = new PhotonCamera("HD_USB_CAMERA");
  List<PhotonPipelineResult> unreadResults = m_randomAssCamera.getAllUnreadResults();
  PhotonTrackedTarget bestTarget = unreadResults.get(0).getBestTarget();
  

  /** Creates a new VisionSusbsytem. */
  public VisionSusbsytem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
