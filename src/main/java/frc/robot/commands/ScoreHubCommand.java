// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.vision.HubMath;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.HubSighting;

/**
 * Holds the right trigger's scoring sequence.
 *
 * <p>While this command runs, the driver still drives forward and sideways with the left stick.
 * The right stick does not turn the robot, because this command turns it toward our hub AprilTag.
 * The sequence each cycle is:
 *
 * <ol>
 *   <li>Read the latest hub tag from the Limelight.
 *   <li>Turn to put that tag in the center of the camera.
 *   <li>Measure the distance and spin the shooter up to the RPM that distance needs. The distance
 *       is used as measured. Spinning starts immediately so the wheels can gain speed.
 *   <li>Feed a ball only after the measured wheel speed reaches that RPM. The wheels are heavy
 *       and keep accelerating after the command is sent. Feeding during that climb throws the
 *       ball short.
 * </ol>
 *
 * <p>Releasing the trigger ends the command, which stops the shooter and the feeder.
 */
public class ScoreHubCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final OuttakeSubsystem outtake;
  private final VisionSubsystem vision;
  private final CommandXboxController driver;
  private final double maxSpeedMetersPerSecond;
  private final double maxTurnRadiansPerSecond;
  private final double slowScale;

  // Same field-centric drive the driver uses, except rotation is our aim speed and not the stick.
  // Rotational deadband stays at 0. That deadband is for stick noise, and it would also eat the
  // small corrections that finish the aim.
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withRotationalDeadband(0.0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Creates a new ScoreHubCommand. */
  public ScoreHubCommand(
      CommandSwerveDrivetrain drivetrain,
      OuttakeSubsystem outtake,
      VisionSubsystem vision,
      CommandXboxController driver,
      double maxSpeedMetersPerSecond,
      double maxTurnRadiansPerSecond,
      double slowScale) {
    this.drivetrain = drivetrain;
    this.outtake = outtake;
    this.vision = vision;
    this.driver = driver;
    this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
    this.maxTurnRadiansPerSecond = maxTurnRadiansPerSecond;
    this.slowScale = slowScale;

    drive.withDeadband(maxSpeedMetersPerSecond * 0.1);

    // Slow mode also wants the drivetrain. Cancel that incoming command instead of letting it
    // cancel this shot. This command reads the bumper itself and slows down.
    withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    addRequirements(drivetrain, outtake);
  }

  @Override
  public void execute() {
    // Left stick: forward is -Y and left is -X, matching the normal drive command.
    // Left bumper still means slow mode while we are aiming.
    double speed = driveSpeed();
    double forward = -driver.getLeftY() * speed;
    double strafe = -driver.getLeftX() * speed;

    Optional<HubSighting> sighting = vision.getHubSighting();
    double turn;
    double targetRpm = 0;
    boolean atTargetRpm = false;
    boolean feeding = false;

    if (sighting.isEmpty()) {
      // No tag lately. Give the right stick back so the driver can turn to go look for it,
      // and do not spin the shooter or feed a ball at a target we cannot see.
      turn = MathUtil.applyDeadband(-driver.getRightX(), 0.1) * maxTurnRadiansPerSecond;
      outtake.stopShooter();
      outtake.stopIndexer();
    } else {
      HubSighting hub = sighting.get();

      // Step 1. Turn toward the tag. This replaces the right stick until the tag is lost.
      turn = HubMath.turnRadiansPerSecond(hub.yawDegrees(), maxTurnRadiansPerSecond);

      // Step 2. The measured distance picks the RPM. Start the wheels now. They will not be at
      // that speed yet.
      targetRpm = HubMath.rpmForDistance(hub.distanceMeters());
      if (targetRpm > 0) {
        outtake.setShooterRpm(targetRpm);
      } else {
        outtake.stopShooter();
      }

      // Step 3. Feed only once the wheels have reached the RPM we just asked for. While they are
      // still gaining speed, the feeder stays off.
      atTargetRpm = HubMath.shooterHasReachedRpm(outtake.getShooterRpm(), targetRpm);
      feeding = atTargetRpm;
      if (feeding) {
        outtake.index();
      } else {
        outtake.stopIndexer();
      }
    }

    drivetrain.setControl(
        drive.withVelocityX(forward).withVelocityY(strafe).withRotationalRate(turn));

    SmartDashboard.putNumber("Score/TargetRpm", targetRpm);
    SmartDashboard.putNumber("Score/ShooterRpm", outtake.getShooterRpm());
    SmartDashboard.putBoolean("Score/AtTargetRpm", atTargetRpm);
    SmartDashboard.putBoolean("Score/Feeding", feeding);
  }

  @Override
  public void end(boolean interrupted) {
    outtake.stopShooter();
    outtake.stopIndexer();
    SmartDashboard.putBoolean("Score/Feeding", false);
  }

  /** Runs until the trigger is released. {@code whileTrue} stops it for us. */
  @Override
  public boolean isFinished() {
    return false;
  }

  private double driveSpeed() {
    if (driver.leftBumper().getAsBoolean()) {
      return maxSpeedMetersPerSecond * slowScale;
    }
    return maxSpeedMetersPerSecond;
  }
}
