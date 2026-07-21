// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.Constants.Vision.HUB_APRIL_TAG_HEIGHT;
import static frc.robot.Constants.Constants.Vision.HUB_APRIL_TAG_PITCH;
import static frc.robot.Constants.Constants.Vision.CAM_HEIGHT;
import static frc.robot.Constants.Constants.Vision.CAM_PITCH;
import static frc.robot.Constants.Constants.Vision.CAM_ROLL;
import static frc.robot.Constants.Constants.Vision.CAM_XPOSE;
import static frc.robot.Constants.Constants.Vision.CAM_YAW;
import static frc.robot.Constants.Constants.Vision.CAM_YPOSE;
import static frc.robot.Constants.Constants.Vision.HUB_CAM_PITCH;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class VisionSusbsytem extends SubsystemBase {
private final PhotonCamera m_camera = new PhotonCamera("HD_USB_CAMERA");
  //PhotonTrackedTarget bestTarget = unreadResults.get(0).getBestTarget();
private AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
private final Transform3d robotTocam = new Transform3d(
    new Translation3d(
    CAM_XPOSE, 
    CAM_YPOSE,     
    CAM_HEIGHT), 
    new Rotation3d(
    CAM_ROLL,
    CAM_PITCH,
    CAM_YAW));

private final PhotonPoseEstimator m_photonPoseEstimator = new PhotonPoseEstimator(layout, robotTocam);
private final Optional<Alliance> alliance = DriverStation.getAlliance();

private double targetYaw;
private double targetPitch;
private double targetSkew;
private double targetArea;
private double targetDistance;

private CommandSwerveDrivetrain m_drivetrain;

  public VisionSusbsytem(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
  var results = m_camera.getAllUnreadResults(); // Obtains all the April Tag results into a list.
  if (!results.isEmpty()) {
    for (var result : results) {
      var bestTarget = result.getBestTarget(); // bestTarget
      // needs Multitag to be enabled on the photon dashboard first, make sure this is true pretty pls
      Optional<EstimatedRobotPose> pose = m_photonPoseEstimator.estimateCoprocMultiTagPose(result);
      if(pose.isPresent()) {
        Pose2d robotPose = new Pose2d(
        pose.get().estimatedPose.getX(),
        pose.get().estimatedPose.getY(),
        pose.get().estimatedPose.getRotation().toRotation2d()
        );
        // Corrects odometry pose estimate using vision measurement.
        m_drivetrain.addVisionMeasurement(
          robotPose, 
          pose.get().timestampSeconds
        );  
      }

      targetYaw = bestTarget.getYaw();           // Horizontal angle to target
      targetPitch = bestTarget.getPitch();       // Vertical angle to target
      targetSkew = bestTarget.getSkew();         // Rotation angle of target
      targetArea = bestTarget.getArea();         // Size of target in view (0-100)
    }
  }

  /* Smart Dashboard Stuff */
    SmartDashboard.putNumber("Yaw:", targetYaw);
    SmartDashboard.putNumber("Pitch", targetPitch);
    SmartDashboard.putNumber("Skew", targetSkew);
    SmartDashboard.putNumber("Area:", targetArea);
}

  // Obtaining distance to april tags on hubs.
  public double getHubTargetDistance() {
      return targetDistance = PhotonUtils.calculateDistanceToTargetMeters(
        CAM_HEIGHT, 
        HUB_APRIL_TAG_HEIGHT, 
        HUB_CAM_PITCH, 
        HUB_APRIL_TAG_PITCH);
        // Not done yet, but instead, just grab all the data from the current id obtained, it's easier if possible.
        // If necessary, just grab the data from specific targets if necessary.
  }

  /** Camera snapshots both input and output. */
  public void camSnapshot() {
    m_camera.takeInputSnapshot();
    m_camera.takeOutputSnapshot();
  }

  /**
   * Returns specific bestTarget data depending on the integer in the paramter.
   * Cases: 1 ->  targetYaw, 2 -> targetPitch, 3 -> targetSkew, 4 -> targetArea, 5 -> targetDistance
   * @return
   * Data from bestTarget
   */
  public double bestTargetReturn(int targetData) {
    switch (targetData) {
      case 1:
        return targetYaw;
      case 2:
        return targetPitch;
      case 3:
        return targetSkew;
      case 4:
        return targetArea;
      case 5:
        return targetDistance;
      default:
        return targetDistance;
    }
    
  }
  //*Align the bot to the hub*/
  public void robotAlign() {
    if (alliance.get().equals(Alliance.Red)) {
    
    } else if (alliance.get().equals(Alliance.Blue)) {

    } else {

    }
  }
}

