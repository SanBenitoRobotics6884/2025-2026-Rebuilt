// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class Vision {
    public static final String CAMERA_NAME = "HD_USB_CAMERA";

    /** Camera position relative to the robot origin. */
    public static final double CAM_XPOSE = Units.inchesToMeters(11.25);
    public static final double CAM_YPOSE = Units.inchesToMeters(5);
    public static final double CAM_HEIGHT = Units.inchesToMeters(8.5);

    /** Camera orientation as Euler angles. */
    public static final double CAM_YAW = Units.degreesToRadians(0);
    public static final double CAM_PITCH = Units.degreesToRadians(0);
    public static final double CAM_ROLL = Units.degreesToRadians(0);

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
    public static final int LIMIT_SWITCH_STORAGE_DIO = 0;

    public static final double PID_P_VALUE = 0.4;

    public static final double TAKE_SPEED = -0.250;
    public static final double STORAGEROLLER_SPEED = 0.15;

    /*
     * If zeroed intake in -> 5, -73
     * If zeroed intake out -> -5, 73
     * These are updated at runtime when the limit switch trips.
     */
    public static double IN_TAKE_TARGET_ROTATIONS = 5.0;
    public static double OUT_TAKE_TARGET_ROTATIONS = -73.0;
    public static final double LIMIT_SWITCH_ROTATIONS = 1.0;
  }

  public static final class Outtake {
    public static final double OUTTAKE_SPEED = 0.98;
    public static final double INDEX_SPEED = 0.5;

    public static final double OUTTAKE_SPEED_SLOW = 0.5;
    public static final double INDEX_SPEED_SLOW = 0.25;

    public static final double PID_P_VALUE = 0.7;
  }

  public static final class Container {
    public static final double SLOW_SWERVE_SPEED = 0.5;

    public static final int CONTROLER_PORT = 0;

    /*
     * These do not match WPILib POV angles (up = 0, right = 90, down = 180, left = 270).
     * The bindings use these values as-is.
     */
    public static final int D_PAD_LEFT = 0;
    public static final int D_PAD_DOWN = 90;
    public static final int D_PAD_RIGHT = 180;
    public static final int D_PAD_UP = 270;

    /** Linear units are meters. Angular units are radians. */
    public static final double MAX_VELOCITY = 0;
    public static final double MAX_ACCELERATION = 0;
    public static final double MAX_ANGULAR_VELOCITY = 0;
    public static final double MAX_ANGULAR_ACCELERATION = 0;
  }

  private Constants() {}
}
