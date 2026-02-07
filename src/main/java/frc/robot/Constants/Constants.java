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
    public static final int IN_OUT_TAKE_MOTOR1_ID = 1;
    public static final int IN_OUT_TAKE_MOTOR2_ID = 60;
    public static final int TAKE_MOTOR1_ID = 10;
    public static final int TAKE_MOTOR2_ID = 14;

    public static final double DUTYCYCLE_OUTPUT = 0.5;

    public static final double IN_TAKE_SPEED = 1.0;
    public static final double OUT_TAKE_SPEED = -1.0;
    public static final double TAKE_SPEED = 1.0;
  }

  public class Outtake {
    public static final int OUT_MOTOR1_ID = 31;
    public static final int OUT_MOTOR2_ID = 32;

    public static final double DUTYCYCLE_OUTPUT = 0.5;

    public static final double OUTTAKE_SPEED = 1.0;
  }
}