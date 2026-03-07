package frc.robot.Constants;

import com.pathplanner.lib.config.RobotConfig;

public class Constants {
  
  public class Intake {
    public static final int LIMIT_SWITCH_STORAGE_DIO = 0;

    public static final double PID_P_VALUE = 0.25;
  

    public static final double DUTYCYCLE_OUTPUT = 0.5;

   
    public static final double TAKE_SPEED = -0.375;
    public static final double STORAGEROLLER_SPEED = 0.25;

    public static final double IN_TAKE_TARGET_ROTATIONS = -55.0;
    public static final double OUT_TAKE_TARGET_ROTATIONS = 0.0;
  }

  public class Outtake {

    public static final double DUTYCYCLE_OUTPUT = 0.5;

    public static final double OUTTAKE_SPEED = 0.98;
    public static final double INDEX_SPEED = 1.0;

    public static final double OUTTAKE_SPEED_SLOW = 0.5;
    public static final double INDEX_SPEED_SLOW = 0.5;

    public static final double PID_P_VALUE = 0.25;
    // public static final double PID_I_VALUE = 0.25;
    // public static final double PID_D_VALUE = 0.25;
  }

  public class Container {
    public static final double SLOW_SWERVE_SPEED = 0.5;

    public static final int CONTROLER_PORT = 0;

    // From POV angles, it is translated to D-Pad buttons:
    public static final int D_PAD_LEFT = 0;
    public static final int D_PAD_DOWN = 90;
    public static final int D_PAD_RIGHT = 180;
    public static final int D_PAD_UP = 270;
  }
}