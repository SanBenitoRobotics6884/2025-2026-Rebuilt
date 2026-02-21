package frc.robot.Constants;

import com.pathplanner.lib.config.RobotConfig;

public class Constants {
  public class Climb {
    public static final int CLIMB_MOTOR_1_ID = 33;
    public static final int CLIMB_MOTOR_2_ID = 34;

    public static final double DUTYCYCLE_OUTPUT = 0.5;

    public static final double UP_CLIMB_SPEED = 1.0;
    public static final double DOWN_CLIMB_SPEED = -1.0;
  }
  
  public class Intake {
    public static final int LIMIT_SWITCH_UNDEPLOY_DIO = 0;

    public static final double PID_P_VALUE = 1;

    public static final double DUTYCYCLE_OUTPUT = 0.5;

   
    public static final double TAKE_SPEED = -0.375;
    public static final double STORAGEROLLER_SPEED = 0.25;

    public static final double IN_TAKE_TARGET_ROTATIONS = 50.0;
    public static final double OUT_TAKE_TARGET_ROTATIONS = 0.0;
  }

  public class Outtake {

    public static final double DUTYCYCLE_OUTPUT = 0.5;

    public static final double OUTTAKE_SPEED = 1.0;
  }
}