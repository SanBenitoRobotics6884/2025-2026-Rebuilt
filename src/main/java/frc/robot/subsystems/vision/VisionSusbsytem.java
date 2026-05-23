// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import static frc.robot.Constants.Constants.Vision.*;

public class VisionSusbsytem extends SubsystemBase {
  PhotonCamera m_randomAssCamera = new PhotonCamera("HD_USB_CAMERA");
  //PhotonCamera m_driverCamera = new PhotonCamera(getName());
  //PhotonTrackedTarget bestTarget = unreadResults.get(0).getBestTarget();
  AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  Transform3d robotTocam = new Transform3d(new Translation3d(
    Units.inchesToMeters(11.25), //These are rough estimates,
    Units.inchesToMeters(5),     //we'll adjust them as we go if they prove to be inaccurate. 
    Units.inchesToMeters(8.5)), new Rotation3d(0,0,0));
  PhotonPoseEstimator m_poseEstimator = new PhotonPoseEstimator(layout, robotTocam);
  Optional<Alliance> alliance = DriverStation.getAlliance();
  boolean AprilTagSight = false;

  double targetYaw;
  double targetPitch;
  double targetSkew;
  double targetArea;
  double targetDistance;
  

  private CommandSwerveDrivetrain m_drivetrain;

  public VisionSusbsytem(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
  var results = m_randomAssCamera.getAllUnreadResults();

  SmartDashboard.putNumber("Camera Results Count", results.size());
  for(var result : results){
    Optional<EstimatedRobotPose> pose = m_poseEstimator.update(result);

     SmartDashboard.putBoolean("Has Valid Pose", pose.isPresent());

    if(pose.isPresent()){
      Pose2d robotPose = new Pose2d(
        pose.get().estimatedPose.getX(),
        pose.get().estimatedPose.getY(),
        pose.get().estimatedPose.getRotation().toRotation2d()
    );

    SmartDashboard.putNumber("Vision Pose X", robotPose.getX());
    SmartDashboard.putNumber("Vision Pose Y", robotPose.getY());
    SmartDashboard.putNumber("Vision Pose Angle", robotPose.getRotation().getDegrees());

    // Corrects odometry pose estimate using vision measurement.
    m_drivetrain.addVisionMeasurement(
                robotPose, 
                pose.get().timestampSeconds
    );
  }

  // If target is found in the pipeline, then the resulting code should get the data of the BEST target
  if(result.hasTargets()){
    AprilTagSight = true;
    
      var bestTarget = result.getBestTarget();
      targetYaw = bestTarget.getYaw();           // Horizontal angle to target
      targetPitch = bestTarget.getPitch();       // Vertical angle to target
      targetSkew = bestTarget.getSkew();         // Rotation angle of target
      targetArea = bestTarget.getArea();         // Size of target in view (0-100)
      
      SmartDashboard.putNumber("Distance to Target (m)", targetDistance);
    } else {
      AprilTagSight = false;
    }
  }

    SmartDashboard.putBoolean("April Tag", AprilTagSight);
    SmartDashboard.putNumber("Yaw:", targetYaw);
    SmartDashboard.putNumber("Pitch", targetPitch);
    SmartDashboard.putNumber("Skew", targetSkew);
    SmartDashboard.putNumber("Area:", targetArea);
  }

  // Obtaining distance to april tags on hubs.
  public double getHubTargetDistance() {
    targetDistance = PhotonUtils.calculateDistanceToTargetMeters(
                  CAM_HEIGHT,
                  APRIL_TAG_HUB_HIEGHT,
                  CAM_PITCH_RADIANS,
                  TARGET_PITCH_RADIANS
                );
    return targetDistance;
    }

  public void camSnapshot() {
    m_randomAssCamera.takeInputSnapshot();
    m_randomAssCamera.takeOutputSnapshot();
  }
}

