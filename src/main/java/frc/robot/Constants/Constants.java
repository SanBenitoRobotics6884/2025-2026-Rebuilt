package frc.robot.Constants;

import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class Constants {
  
  public class Intake {
    public static final int LIMIT_SWITCH_STORAGE_DIO = 0;

    public static final double PID_P_VALUE = 0.4;
  
  
    public static final double TAKE_SPEED = -0.250;
    public static final double STORAGEROLLER_SPEED = 0.15;

    /* Lowkey, just switch the rotation direciton (negative to pos)
     if robot was zeroed while the intake was out.
     */

    public static double IN_TAKE_TARGET_ROTATIONS = 5.0;
    public static double OUT_TAKE_TARGET_ROTATIONS = -73.0;
    public static final double LIMIT_SWITCH_ROTATIONS = 1.0;
  }

  public class Outtake {
    public static final double OUTTAKE_SPEED = 0.98;
    public static final double INDEX_SPEED = 0.5;

    public static final double OUTTAKE_SPEED_SLOW = 0.5;
    public static final double INDEX_SPEED_SLOW = 0.25;

    public static final double PID_P_VALUE = 0.7;
    // public static final double PID_I_VALUE = 0.25;
    // public static final double PID_D_VALUE = 0.25;
  }

  public class Container {
    public static final double SLOW_SWERVE_SPEED = 0.5;

    public static final int CONTROLER_PORT = 0;

    // These are incorrect, please fix them after competiton. POV degrees start with the top being 0 and the bottom being 180.
    // Therefore, D_PAD_DOWN is not 90, it would be 180. Fix these when possible and relay it to the drive team.
    // From POV angles, it is translated to D-Pad buttons:
    public static final int D_PAD_LEFT = 0;
    public static final int D_PAD_DOWN = 90;
    public static final int D_PAD_RIGHT = 180;
    public static final int D_PAD_UP = 270;
  }

  public class Vision {
    public static final double CAM_HEIGHT = Units.inchesToMeters(8.5);
    public static final double APRIL_TAG_HUB_HIEGHT = Units.inchesToMeters(44.25);

    public static final double CAM_PITCH_RADIANS = Units.degreesToRadians(0);
    public static final double TARGET_PITCH_RADIANS = Units.degreesToRadians(0);
    // FIDUCIAL IDS
    public static final int RED_HUB_FIDUCIAL_L = 9;
    public static final int RED_HUB_FIDUCIAL_R = 10;
    
    public static final int BLUE_HUB_FIDUCIAL_L = 25;
    public static final int BLUE_HUB_FIDUCIAL_R = 26;
  }
}