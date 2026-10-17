// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {
  /**
   * Numbers that describe the Limelight and how much we trust it.
   *
   * <p>PhotonVision does the image processing on the Limelight. This class is only the robot-side
   * half: which camera to listen to, where that camera is bolted on, which field map to use, and
   * how skeptical to be of a pose before it is allowed to move swerve odometry.
   *
   * <p>Measure the camera from the center of the robot, on the floor. +X is forward, +Y is left,
   * +Z is up. If these inches are wrong, every vision pose is wrong by that same amount.
   */
  public static final class Vision {
    /**
     * Name of the camera in the PhotonVision dashboard (http://photonvision.local:5800). A fresh
     * Limelight 3 image calls the built-in camera {@code Camera_Module_v1}. If someone renames it
     * in the dashboard, this string has to be renamed to the exact same thing or the robot will
     * report {@code Vision/Connected = false} forever.
     */
    public static final String CAMERA_NAME = "Camera_Module_v1";

    /**
     * Which PhotonVision pipeline to run. Pipeline 0 has to be an AprilTag pipeline with 3D
     * tracking turned on. Pipelines are configured on the Limelight, not in this file.
     */
    public static final int APRILTAG_PIPELINE_INDEX = 0;

    /** Camera position relative to the robot origin. */
    public static final double CAM_XPOSE = Units.inchesToMeters(11.25);
    public static final double CAM_YPOSE = Units.inchesToMeters(5);
    public static final double CAM_HEIGHT = Units.inchesToMeters(8.5);

    /**
     * Which way the lens points, in radians. Zero means the lens faces straight forward, level
     * with the floor. Rotation order is roll (twist), then pitch (tilt up/down), then yaw (turn
     * left/right).
     */
    public static final double CAM_YAW = Units.degreesToRadians(0);
    public static final double CAM_PITCH = Units.degreesToRadians(0);
    public static final double CAM_ROLL = Units.degreesToRadians(0);

    /**
     * The inches and angles above, packed into the one object PhotonVision wants. This is what
     * shifts a "where is the camera" answer into a "where is the robot" answer.
     */
    public static final Transform3d ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(CAM_XPOSE, CAM_YPOSE, CAM_HEIGHT),
            new Rotation3d(CAM_ROLL, CAM_PITCH, CAM_YAW));

    /**
     * Map of every official 2026 AprilTag. The Limelight must be given the same map in its
     * settings, or multi-tag poses will not agree with this one.
     *
     * <p>Most events use the welded field, which is what is loaded here. AndyMark fields are a
     * different map ({@link AprilTagFields#k2026RebuiltAndymark}). Using the wrong one shifts the
     * robot by a few inches in a way that is painful to debug.
     */
    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /**
     * Starting "how wrong could this be?" for a vision pose, as [x meters, y meters, heading
     * radians].
     *
     * <p>These are not accuracy goals. They are weights. 0.5 m means the pose estimator treats the
     * camera as rough and only nudges odometry. 4 m means it barely trusts the camera at all.
     * Heading is large on purpose (8 rad is more than a full turn of uncertainty) so a noisy
     * camera angle cannot overwrite the Pigeon.
     *
     * <p>One tag starts at 4 m. Two or more tags start at 0.5 m. {@code VisionSubsystem} then makes
     * these worse as the tags get farther away. Tune them by comparing {@code Vision Field} to the
     * swerve pose while someone pushes the robot around.
     */
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

    /**
     * Single-tag poses with ambiguity above this are thrown away. 0 is a clean solve. 0.2 is
     * already suspicious. 1 means the tag looks the same in two different 3D positions, and the
     * camera may have picked the wrong one.
     */
    public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.2;

    /**
     * A lone tag farther than this is ignored. At long range, one pixel of camera error is a large
     * position error, and there is no second tag to cross-check it.
     */
    public static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0;

    /**
     * Pictures older than this are ignored. The drivetrain only keeps a short history of where it
     * was, and a correction has to land inside that history.
     */
    public static final double MAX_MEASUREMENT_LATENCY_SECONDS = 0.5;

    /** Hub measurements. */
    public static final double HUB_APRIL_TAG_HEIGHT = Units.inchesToMeters(44.25);
    public static final double HUB_CAM_PITCH = Units.degreesToRadians(0);

    /** Hub fiducial ids. */
    public static final int RED_HUB_FIDUCIAL_L = 9;
    public static final int RED_HUB_FIDUCIAL_R = 10;
    public static final int BLUE_HUB_FIDUCIAL_L = 25;
    public static final int BLUE_HUB_FIDUCIAL_R = 26;
  }

  public static final class Intake {
    public static final int kLeftScrewId = 20;
    public static final int kRightScrewId = 30;
    public static final int kIntakeRollerId = 10;
    public static final int kStorageRollerId = 40;

    public static final int kLimitSwitchDio = 0;

    public static final double kP = 0.4;

    /** Duty cycle for the intake roller. Negative pulls a game piece in. */
    public static final double kIntakeSpeed = -0.4;

    /** Duty cycle for the storage roller. */
    public static final double kStorageSpeed = 0.4;

    /**
     * Starting screw setpoints, in rotations, before the limit switch homes them.
     * If zeroed with the intake in, these are about 5 and -73.
     * If zeroed with the intake out, these are about -5 and 73.
     */
    public static final double kInitialExtendRotations = 5.0;
    public static final double kInitialRetractRotations = -73.0;

    /** How far to back off the limit switch when homing, in rotations. */
    public static final double kLimitSwitchBackoffRotations = 1.0;

    /** Rotations from the homed extend position back to the retract position. */
    public static final double kHomingRetractOffsetRotations = 60.0;
  }

  public static final class Outtake {
    public static final int kShooterLeaderId = 34;
    public static final int kShooterFollowerId = 62;
    public static final int kIndexerId = 50;

    /** Closed-loop shooter and indexer speed, in rotations per second (3000 rpm). */
    public static final double kShooterVelocityRps = 3000.0 / 60.0;

    public static final double kP = 0.7;

    /** Multiplier on {@link #kShooterVelocityRps} for the slow shooter. */
    public static final double kShooterSlowScale = 0.5;

    /** Multiplier on {@link #kShooterVelocityRps} for the indexer during a slow shot. */
    public static final double kIndexerSlowScale = 0.25;
  }

  public static final class OperatorConstants {
    public static final double kSlowDriveScale = 0.5;

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    /** WPILib POV angles: up = 0, right = 90, down = 180, left = 270. */
    public static final int kPovUp = 0;
    public static final int kPovRight = 90;
    public static final int kPovDown = 180;
    public static final int kPovLeft = 270;

    /** Linear units are meters. Angular units are radians. */
    public static final double kMaxVelocity = 0;
    public static final double kMaxAcceleration = 0;
    public static final double kMaxAngularVelocity = 0;
    public static final double kMaxAngularAcceleration = 0;
  }

  private Constants() {}
}
