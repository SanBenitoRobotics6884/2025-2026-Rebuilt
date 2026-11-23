// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Constants.Outtake.kShooterVelocityRps;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.vision.HubMath;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.HubSighting;

/**
 * Holds the right trigger's scoring sequence.
 *
 * <p>While this command runs, the driver still drives forward and sideways with the left stick.
 * While our hub AprilTag is in view, the right stick does not turn the robot, because this
 * command turns it toward the tag.
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
 * <p><b>Fallback:</b> when the Limelight has not seen our hub tag lately, the right stick turns the
 * robot again and the shooter spins at the fixed kShooterVelocityRps instead. A ball is still fed
 * only once the wheels reach that speed. The moment the tag is seen again, aiming takes over.
 *
 * <p>The extension stows as soon as the trigger is pulled, so the screws are moving in before a
 * ball is fed. Releasing the trigger stops the shooter and the feeder, and puts the extension
 * back out if it was deployed when the shot started.
 */
public class ScoreHubCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final OuttakeSubsystem outtake;
  private final IntakeSubsystem intake;
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
      IntakeSubsystem intake,
      VisionSubsystem vision,
      CommandXboxController driver,
      double maxSpeedMetersPerSecond,
      double maxTurnRadiansPerSecond,
      double slowScale) {
    this.drivetrain = drivetrain;
    this.outtake = outtake;
    this.intake = intake;
    this.vision = vision;
    this.driver = driver;
    this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
    this.maxTurnRadiansPerSecond = maxTurnRadiansPerSecond;
    this.slowScale = slowScale;

    drive.withDeadband(maxSpeedMetersPerSecond * 0.1);

    addRequirements(drivetrain, outtake, intake);
  }

  /**
   * Slow mode also wants the drivetrain. Cancel that incoming command instead of letting it cancel
   * this shot. This command reads the bumper itself and slows down.
   *
   * <p>Overridden here rather than calling {@code withInterruptBehavior()} in the constructor:
   * that wraps this command in a new one and marks this one as used, so scheduling it from the
   * trigger would throw and crash the robot code.
   */
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }

  @Override
  public void initialize() {
    // Start stowing immediately, during the spin-up, so the screws are moving in before a ball
    // is fed. The first call remembers whether we were deployed.
    intake.retractForShot();
  }

  @Override
  public void execute() {
    intake.retractForShot();
    // Left stick: forward is -Y and left is -X, matching the normal drive command.
    // Left bumper still means slow mode while we are aiming.
    double speed = driveSpeed();
    double forward = -driver.getLeftY() * speed;
    double strafe = -driver.getLeftX() * speed;

    Optional<HubSighting> sighting = vision.getHubSighting();
    boolean autoAim = sighting.isPresent();
    double turn;
    double targetRpm;

    if (autoAim) {
      HubSighting hub = sighting.get();
      // Turn toward the tag. This replaces the right stick until the tag is lost.
      turn = HubMath.turnRadiansPerSecond(hub.yawDegrees(), maxTurnRadiansPerSecond);
      // The measured distance picks the RPM.
      targetRpm = HubMath.rpmForDistance(hub.distanceMeters());
    } else {
      // No tag lately (vision down, tag blocked, or out of view). Give the right stick back so the
      // driver can aim by hand, and fall back to the fixed shooter speed.
      turn = MathUtil.applyDeadband(-driver.getRightX(), 0.1) * maxTurnRadiansPerSecond;
      targetRpm = kShooterVelocityRps * 60.0;
    }

    // Start the wheels now; they will not be at that speed yet. Feed only once they have reached
    // it. The wheels are heavy, and feeding during the climb throws the ball short.
    boolean atTargetRpm = false;
    if (targetRpm > 0) {
      outtake.setShooterRpm(targetRpm);
      atTargetRpm = HubMath.shooterHasReachedRpm(outtake.getShooterRpm(), targetRpm);
    } else {
      outtake.stopShooter();
    }
    boolean feeding = atTargetRpm;
    if (feeding) {
      outtake.index();
    } else {
      outtake.stopIndexer();
    }

    drivetrain.setControl(
        drive.withVelocityX(forward).withVelocityY(strafe).withRotationalRate(turn));

    SmartDashboard.putString("Score/Mode", autoAim ? "AUTO_AIM" : "FIXED");
    SmartDashboard.putNumber("Score/TargetRpm", targetRpm);
    SmartDashboard.putNumber("Score/ShooterRpm", outtake.getShooterRpm());
    SmartDashboard.putBoolean("Score/AtTargetRpm", atTargetRpm);
    SmartDashboard.putBoolean("Score/Feeding", feeding);
  }

  @Override
  public void end(boolean interrupted) {
    outtake.stopShooter();
    outtake.stopIndexer();
    intake.restoreAfterShot();
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
