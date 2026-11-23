// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Constants.Outtake.kShooterVelocityRps;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.vision.HubMath;
import frc.robot.subsystems.vision.HubMath.ShotMode;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.HubSighting;

/**
 * Holds the right trigger's scoring sequence.
 *
 * <p>While this command runs, the driver still drives forward and sideways with the left stick.
 * While any of our hub's AprilTags is in view, the right stick does not turn the robot, because
 * this command turns it toward the middle of the hub.
 * The sequence each cycle is:
 *
 * <ol>
 *   <li>Read where the middle of our hub is, from whichever of its tags the Limelight sees.
 *   <li>Turn until the middle of the hub is straight ahead.
 *   <li>Measure the distance to the middle of the hub and spin the shooter up to the RPM that
 *       distance needs. The distance
 *       is used as measured. Spinning starts immediately so the wheels can gain speed.
 *   <li>Feed a ball only after the measured wheel speed reaches that RPM. The wheels are heavy
 *       and keep accelerating after the command is sent. Feeding during that climb throws the
 *       ball short.
 * </ol>
 *
 * <p><b>Alliance zone:</b> scoring is only legal from our ALLIANCE ZONE (rule G407). Outside it,
 * or when the robot cannot tell, the shot is always FIXED (driver aims, kShooterVelocityRps),
 * which is also the pass shot. Inside it:
 *
 * <p><b>Fallbacks:</b> when no hub tag has been seen within kMaxSightingAgeSeconds, the right
 * stick turns the robot again. If a vision pose agreed with the robot's tracked field position
 * within kPoseFallbackMaxAgeSeconds (POSE mode), the distance from that position to the hub's middle
 * still sets the RPM. Otherwise (FIXED mode) the shooter spins at kShooterVelocityRps. A ball is
 * fed only once the wheels reach the RPM in every mode. The moment a hub tag is seen again,
 * aiming takes over.
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
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean red = alliance.isPresent() && alliance.get() == Alliance.Red;
    // Scoring is only legal from our ALLIANCE ZONE (G407). Without an alliance we cannot tell.
    boolean sightingInZone =
        alliance.isPresent()
            && sighting.isPresent()
            && HubMath.inAllianceZone(sighting.get().robotFieldPosition(), red);
    boolean poseInZone =
        alliance.isPresent()
            && HubMath.inAllianceZone(drivetrain.getPose().getTranslation(), red);
    double poseDistance = poseDistanceToHubMeters();
    ShotMode mode =
        HubMath.shotMode(
            sighting.isPresent(),
            sightingInZone,
            vision.secondsSinceTrustedPose(),
            poseDistance,
            poseInZone);
    double turn;
    double targetRpm;

    switch (mode) {
      case AUTO_AIM -> {
        HubSighting hub = sighting.get();
        // Turn toward the middle of the hub. This replaces the right stick until no tag is seen.
        turn = HubMath.turnRadiansPerSecond(hub.yawDegrees(), maxTurnRadiansPerSecond);
        // The measured distance picks the RPM.
        targetRpm = HubMath.rpmForDistance(hub.distanceMeters());
      }
      case POSE -> {
        // No hub tag in view, but the robot's tracked position is recent: the driver aims, and
        // the distance from that position picks the RPM.
        turn = stickTurn();
        targetRpm = HubMath.rpmForDistance(poseDistance);
      }
      default -> {
        // Outside our zone (a pass), or no way to tell where we are: the driver aims and the
        // shooter runs at the fixed speed.
        turn = stickTurn();
        targetRpm = kShooterVelocityRps * 60.0;
      }
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

    SmartDashboard.putString("Score/Mode", mode.name());
    SmartDashboard.putBoolean("Score/InAllianceZone", sightingInZone || poseInZone);
    SmartDashboard.putNumber("Score/PoseDistanceMeters", poseDistance);
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

  /** Right stick turning, for when the driver aims. */
  private double stickTurn() {
    return MathUtil.applyDeadband(-driver.getRightX(), 0.1) * maxTurnRadiansPerSecond;
  }

  /**
   * Floor distance from the robot's tracked field position to the middle of our hub, or NaN when
   * the alliance is not known yet.
   */
  private double poseDistanceToHubMeters() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return Double.NaN;
    }
    Translation2d hub = HubMath.hubCenter(alliance.get() == Alliance.Red);
    return hub.getDistance(drivetrain.getPose().getTranslation());
  }

  private double driveSpeed() {
    if (driver.leftBumper().getAsBoolean()) {
      return maxSpeedMetersPerSecond * slowScale;
    }
    return maxSpeedMetersPerSecond;
  }
}
