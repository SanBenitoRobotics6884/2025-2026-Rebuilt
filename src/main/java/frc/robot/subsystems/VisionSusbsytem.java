// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class VisionSusbsytem extends SubsystemBase {
  PhotonCamera m_randomAssCamera = new PhotonCamera("HD_USB_CAMERA");
  //PhotonTrackedTarget bestTarget = unreadResults.get(0).getBestTarget();
  AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  Transform3d robotTocam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
 PhotonPoseEstimator m_poseEstimator = new PhotonPoseEstimator(layout, robotTocam);

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
        
        m_drivetrain.addVisionMeasurement(robotPose, pose.get().timestampSeconds);

    m_drivetrain.addVisionMeasurement(
                robotPose, 
                pose.get().timestampSeconds
    );
  }

  if(result.hasTargets()){
      var bestTarget = result.getBestTarget();
      
      AprilTagSight = true;
      targetYaw = bestTarget.getYaw();           // Horizontal angle to target
      targetPitch = bestTarget.getPitch();       // Vertical angle to target
      targetSkew = bestTarget.getSkew();         // Rotation angle of target
      targetArea = bestTarget.getArea();         // Size of target in view (0-100)
      
  
      double distance = Math.sqrt(
        Math.pow(bestTarget.getBestCameraToTarget().getX(), 2) +
        Math.pow(bestTarget.getBestCameraToTarget().getY(), 2) +
        Math.pow(bestTarget.getBestCameraToTarget().getZ(), 2)
      );
      
      SmartDashboard.putNumber("Distance to Target (m)", distance);
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

  public double getTargetDistance() {
    return targetDistance;
  }
}

