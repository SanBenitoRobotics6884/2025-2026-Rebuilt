// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.Constants.Outtake.kShooterVelocityRps;
import static frc.robot.Constants.Constants.Score.kPoseFallbackMaxAgeSeconds;
import static frc.robot.Constants.Constants.Vision.FIELD_LAYOUT;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeSet;
import java.util.function.BooleanSupplier;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.ScoreHubCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExtensionStateMachine.State;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.vision.HubMath;
import frc.robot.subsystems.vision.VisionSubsystem;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/**
 * Holds the right trigger (ScoreHubCommand) in simulated teleop:
 *
 * <ol>
 *   <li>Facing away from our hub (other tags still in view): expect the POSE fallback, the RPM
 *       for the tracked distance to the hub's middle, the right stick turning the robot, and a
 *       feed once the wheels reach speed.
 *   <li>Mid-field with no tags in view for longer than kPoseFallbackMaxAgeSeconds: expect the
 *       FIXED fallback at kShooterVelocityRps.
 *   <li>Facing our hub a little off-center: expect AUTO_AIM, the robot turning onto its middle,
 *       the RPM picked from distance, and a feed once the wheels reach speed.
 *   <li>In the opposing alliance's zone (both alliances), facing their hub or passing back: expect
 *       FIXED even with their hub's tags in view, the right stick turning the robot, and a feed.
 *   <li>Intake fully extended when the trigger is pulled: expect it to stow to S1 during the
 *       shot and go back out to S2 after release.
 * </ol>
 *
 * <p>Run with {@code ./gradlew teleopSim}. Real time (the CTRE sim uses the wall clock), so it is
 * not part of {@code ./gradlew build}.
 */
@Tag("teleopSim")
class ScoreHubTeleopSimTest {
  static final double kLoopSeconds = 0.01;
  static final double kHoldSeconds = 4.0;
  static final double kTagDistanceMeters = 2.5;
  /** The blue hub's tag that faces the blue alliance wall, on the hub's center line. */
  static final int kAllianceFaceTag = 26;
  static final double kStartAimErrorDegrees = 10.0;
  /** Xbox right stick X is axis 4. */
  static final int kRightXAxis = 4;
  /** In the no-tag case, the right stick is held only this long, so the robot never turns onto a tag. */
  static final double kStickPulseSeconds = 0.3;
  /** Aim is "on" within this many degrees: the camera sits off the robot's center line. */
  static final double kAimToleranceDegrees = 4.0;
  /** Give up waiting for one intake move after this long. */
  static final double kIntakeMoveLimitSeconds = 8.0;

  CommandSwerveDrivetrain m_drivetrain;
  OuttakeSubsystem m_outtake;
  IntakeSubsystem m_intake;
  VisionSubsystem m_vision;
  long m_loops;
  double m_nextLoopTime;
  final List<String> m_report = new ArrayList<>();
  final List<String> m_failures = new ArrayList<>();

  @Test
  void rightTriggerFallsBackWithoutTheTagAndAutoAimsWithIt() throws InterruptedException {
    assertTrue(HAL.initialize(500, 0), "HAL did not start");
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    // An Xbox controller on port 0, so the right stick can be moved.
    DriverStationSim.setJoystickAxisCount(0, 6);
    DriverStationSim.setJoystickButtonCount(0, 10);
    DriverStationSim.setJoystickPOVCount(0, 1);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    m_drivetrain = TunerConstants.createDrivetrain();
    m_outtake = new OuttakeSubsystem();
    m_intake = new IntakeSubsystem();
    m_vision = new VisionSubsystem(m_drivetrain);
    // Hold still when nothing else drives (coasting would drift: the sim has no friction).
    SwerveRequest.RobotCentric hold =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
    m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(() -> hold));
    double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double maxTurn = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    CommandXboxController driver = new CommandXboxController(0);
    m_nextLoopTime = Timer.getFPGATimestamp();
    runFor(0.5);

    Pose2d face = FIELD_LAYOUT.getTagPose(kAllianceFaceTag).orElseThrow().toPose2d();
    // Aim is judged against the middle of the hub, which is what the robot aims at.
    Pose2d tag = new Pose2d(HubMath.BLUE_HUB_CENTER, Rotation2d.kZero);
    // kTagDistanceMeters straight out from that face, turned to look at it.
    Pose2d facingTag =
        face.transformBy(
            new Transform2d(new Translation2d(kTagDistanceMeters, 0), Rotation2d.k180deg));

    // 1. Facing away from the hub: fallback, with the driver turning on the right stick.
    Pose2d facingAway = new Pose2d(facingTag.getTranslation(), face.getRotation());
    // Facing the blue wall, the camera still sees wall tags, so the robot's position is fresh:
    // POSE mode, RPM from the distance to the hub's middle.
    double awayRpm =
        HubMath.rpmForDistance(HubMath.BLUE_HUB_CENTER.getDistance(facingAway.getTranslation()));
    runCase("No hub tag (facing away)", facingAway, 0.5, "POSE", awayRpm, true,
        new ScoreHubCommand(m_drivetrain, m_outtake, m_intake, m_vision, driver, maxSpeed, maxTurn, 0.5),
        tag, 1.0);

    // 1b. No tag in view for longer than kPoseFallbackMaxAgeSeconds: mid-field, facing a side
    // wall, nothing to see. The tracked position is too old to trust, so FIXED mode.
    Pose2d blind = new Pose2d(FIELD_LAYOUT.getFieldLength() / 2.0, 6.0, Rotation2d.kCCW_90deg);
    runCase("No tags at all (position too old)", blind, 0.5, "FIXED", kShooterVelocityRps * 60.0,
        true,
        new ScoreHubCommand(m_drivetrain, m_outtake, m_intake, m_vision, driver, maxSpeed, maxTurn, 0.5),
        tag, kPoseFallbackMaxAgeSeconds + 1.0);

    // 2. Facing the hub, a little off-center: auto-aim turns onto its middle.
    Pose2d offCenter =
        facingTag.transformBy(
            new Transform2d(Translation2d.kZero, Rotation2d.fromDegrees(kStartAimErrorDegrees)));
    double expectedRpm =
        HubMath.rpmForDistance(HubMath.BLUE_HUB_CENTER.getDistance(offCenter.getTranslation()));
    runCase("Hub tag in view", offCenter, 0.0, "AUTO_AIM", expectedRpm, false,
        new ScoreHubCommand(m_drivetrain, m_outtake, m_intake, m_vision, driver, maxSpeed, maxTurn, 0.5),
        tag, 1.0);

    // 4. In the OPPOSING alliance's zone: scoring is not allowed there, so no auto-aim even with
    // their hub's tags in view. Fixed speed, driver aims (passing back toward our side).
    Translation2d redHub = HubMath.RED_HUB_CENTER;
    Pose2d redHubPose = new Pose2d(redHub, Rotation2d.kZero);
    Pose2d inRedZoneFacingTheirHub =
        new Pose2d(redHub.plus(new Translation2d(3.1, 0)), Rotation2d.k180deg);
    runCase("Blue robot in red's zone, facing red's hub", inRedZoneFacingTheirHub, 0.5, "FIXED",
        kShooterVelocityRps * 60.0, true,
        new ScoreHubCommand(m_drivetrain, m_outtake, m_intake, m_vision, driver, maxSpeed, maxTurn, 0.5),
        redHubPose, 1.0);
    Pose2d inRedZonePassing = new Pose2d(
        FIELD_LAYOUT.getFieldLength() - 2.5, 1.5, Rotation2d.fromDegrees(160));
    runCase("Blue robot in red's zone, passing toward our side", inRedZonePassing, 0.5, "FIXED",
        kShooterVelocityRps * 60.0, true,
        new ScoreHubCommand(m_drivetrain, m_outtake, m_intake, m_vision, driver, maxSpeed, maxTurn, 0.5),
        redHubPose, 1.0);

    // 4b. The same for a red robot in blue's zone, facing blue's hub.
    setAlliance(true);
    Pose2d inBlueZoneFacingTheirHub = new Pose2d(
        HubMath.BLUE_HUB_CENTER.minus(new Translation2d(3.1, 0)), Rotation2d.kZero);
    runCase("Red robot in blue's zone, facing blue's hub", inBlueZoneFacingTheirHub, 0.5, "FIXED",
        kShooterVelocityRps * 60.0, true,
        new ScoreHubCommand(m_drivetrain, m_outtake, m_intake, m_vision, driver, maxSpeed, maxTurn, 0.5),
        tag, 1.0);
    setAlliance(false);

    // 3. Intake fully out when the trigger is pulled: it stows for the shot, then goes back out.
    runReExtendCase(facingAway,
        new ScoreHubCommand(m_drivetrain, m_outtake, m_intake, m_vision, driver, maxSpeed, maxTurn, 0.5));

    System.out.println();
    System.out.println("Right trigger in simulated teleop:");
    m_report.forEach(line -> System.out.println("  " + line));
    System.out.println();
    if (!m_failures.isEmpty()) {
      fail("Right trigger problems:\n  " + String.join("\n  ", m_failures));
    }
  }

  void runCase(String name, Pose2d start, double rightStickX, String expectedMode,
      double expectedTargetRpm, boolean expectStickTurn, ScoreHubCommand shot, Pose2d tag,
      double settleSeconds) throws InterruptedException {
    CommandScheduler.getInstance().cancelAll();
    m_outtake.stopShooter();
    m_outtake.stopIndexer();
    // Place the robot twice: camera frames arrive late and, right after a teleport, would be
    // applied against the previous spot. A real robot never teleports.
    m_drivetrain.resetPose(start);
    setRightStick(0.0);
    runFor(0.4);
    m_drivetrain.resetPose(start);
    runFor(settleSeconds); // let the camera sim settle at the new pose

    double startAimError = aimErrorDegrees(tag);
    SmartDashboard.putString("Score/Mode", ""); // don't read the previous case's mode
    setRightStick(rightStickX);
    CommandScheduler.getInstance().schedule(shot);
    double start0 = Timer.getFPGATimestamp();

    String mode = "";
    boolean modeSteady = true;
    List<String> modeChanges = new ArrayList<>();
    double targetRpm = 0;
    double maxShooterRpm = 0;
    double maxTurnRate = 0;
    double firstFeedSeconds = Double.NaN;
    double maxIndexerRps = 0;
    TreeSet<Integer> tagsSeen = new TreeSet<>();
    int maxTagsInView = 0;
    while (Timer.getFPGATimestamp() - start0 < kHoldSeconds) {
      if (rightStickX != 0 && Timer.getFPGATimestamp() - start0 > kStickPulseSeconds) {
        setRightStick(0.0);
      }
      tick();
      var sighting = m_vision.getHubSighting();
      sighting.ifPresent(hub -> tagsSeen.add(hub.tagId()));
      maxTagsInView =
          Math.max(maxTagsInView, (int) SmartDashboard.getNumber("Vision/TagCount", 0));
      String now = SmartDashboard.getString("Score/Mode", "");
      if (!now.isEmpty()) {
        if (!mode.isEmpty() && !now.equals(mode)) {
          modeSteady = false;
          modeChanges.add(String.format("%s at %.2f s", now, Timer.getFPGATimestamp() - start0));
        }
        mode = now;
      }
      targetRpm = SmartDashboard.getNumber("Score/TargetRpm", 0);
      maxShooterRpm = Math.max(maxShooterRpm, m_outtake.getShooterRpm());
      maxTurnRate =
          Math.max(maxTurnRate, Math.abs(m_drivetrain.getState().Speeds.omegaRadiansPerSecond));
      if (SmartDashboard.getBoolean("Score/Feeding", false) && Double.isNaN(firstFeedSeconds)) {
        firstFeedSeconds = Timer.getFPGATimestamp() - start0;
      }
      maxIndexerRps = Math.max(maxIndexerRps, Math.abs(m_outtake.getIndexerRps()));
    }
    double endAimError = aimErrorDegrees(tag);
    CommandScheduler.getInstance().cancel(shot);
    setRightStick(0.0);
    runFor(0.5);
    // The intake started retracted, so releasing must not send it out.
    boolean stayedIn =
        m_intake.isRetracted() && m_intake.getExtensionState() != State.EXTENDING;

    m_report.add(name + ":");
    m_report.add(String.format("  camera saw up to %d tags; %s", maxTagsInView,
        tagsSeen.isEmpty() ? "none of them on our hub" : "our hub tags " + tagsSeen));
    m_report.add(String.format("  mode %s%s, target %.0f rpm, shooter reached %.0f rpm",
        mode, modeSteady ? "" : " (changed: " + String.join(", ", modeChanges) + ")",
        targetRpm, maxShooterRpm));
    m_report.add(Double.isNaN(firstFeedSeconds)
        ? "  never fed"
        : String.format("  fed after %.2f s, indexer up to %.1f rps", firstFeedSeconds, maxIndexerRps));
    m_report.add(String.format("  turn rate up to %.2f rad/s, aim error %.1f deg -> %.1f deg%s",
        maxTurnRate, startAimError, endAimError,
        tagsSeen.isEmpty() ? "" : " (hub tags used: " + tagsSeen + ")"));

    m_report.add(stayedIn
        ? "  intake started in and stayed in after release"
        : "  intake moved after release (state " + m_intake.getExtensionState() + ")");
    check(name, stayedIn, "intake started retracted but did not stay in after release");
    check(name, mode.equals(expectedMode), "mode was " + mode + ", expected " + expectedMode);
    check(name, modeSteady, "mode changed during the hold");
    check(name, Math.abs(targetRpm - expectedTargetRpm) < 150,
        String.format("target %.0f rpm, expected about %.0f", targetRpm, expectedTargetRpm));
    check(name, !Double.isNaN(firstFeedSeconds),
        String.format("never fed (shooter reached %.0f of %.0f rpm)", maxShooterRpm, targetRpm));
    if (expectStickTurn) {
      check(name, maxTurnRate > 0.5, "right stick did not turn the robot");
    } else {
      check(name, Math.abs(endAimError) < kAimToleranceDegrees,
          String.format("did not aim at the tag (still %.1f deg off)", endAimError));
    }
  }

  /**
   * Extends the intake to S2, holds the right trigger until the intake has stowed to S1, releases,
   * and checks that it goes back out to S2.
   */
  void runReExtendCase(Pose2d start, ScoreHubCommand shot) throws InterruptedException {
    String name = "Intake out when the shot starts";
    CommandScheduler.getInstance().cancelAll();
    m_drivetrain.resetPose(start);
    setRightStick(0.0);
    runFor(0.5);

    m_intake.extend();
    double extendSeconds = runUntil(m_intake::isExtended, kIntakeMoveLimitSeconds);
    check(name, m_intake.isExtended(), "could not extend the intake to S2 before the shot");

    CommandScheduler.getInstance().schedule(shot);
    double stowSeconds = runUntil(m_intake::isRetracted, kIntakeMoveLimitSeconds);
    boolean stowed = m_intake.isRetracted();
    runFor(0.3); // keep holding a moment after it reaches S1

    CommandScheduler.getInstance().cancel(shot); // release the trigger
    double backOutSeconds = runUntil(m_intake::isExtended, kIntakeMoveLimitSeconds);
    boolean backOut = m_intake.isExtended();

    m_report.add(name + ":");
    m_report.add(String.format("  extended to S2 in %.2f s before the shot", extendSeconds));
    m_report.add(stowed
        ? String.format("  trigger held: stowed to S1 in %.2f s", stowSeconds)
        : "  trigger held: never reached S1");
    m_report.add(backOut
        ? String.format("  trigger released: back out to S2 in %.2f s", backOutSeconds)
        : "  trigger released: did not go back out (state " + m_intake.getExtensionState() + ")");

    check(name, stowed, "intake did not stow to S1 while the trigger was held");
    check(name, backOut, "intake did not go back out to S2 after release");
  }

  /** Runs until {@code done} is true or {@code limitSeconds} pass. Returns the seconds it took. */
  double runUntil(BooleanSupplier done, double limitSeconds)
      throws InterruptedException {
    double start = Timer.getFPGATimestamp();
    while (!done.getAsBoolean() && Timer.getFPGATimestamp() - start < limitSeconds) {
      tick();
    }
    return Timer.getFPGATimestamp() - start;
  }

  void check(String name, boolean ok, String problem) {
    if (!ok) {
      m_failures.add(name + ": " + problem);
    }
  }

  /** Degrees the robot's heading is off from pointing straight at {@code tag} (the hub's middle). */
  double aimErrorDegrees(Pose2d tag) {
    Pose2d robot = m_drivetrain.getPose();
    Rotation2d toTag = tag.getTranslation().minus(robot.getTranslation()).getAngle();
    return toTag.minus(robot.getRotation()).getDegrees();
  }

  void setAlliance(boolean red) {
    DriverStationSim.setAllianceStationId(red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  void setRightStick(double value) {
    DriverStationSim.setJoystickAxis(0, kRightXAxis, value);
    DriverStationSim.notifyNewData();
  }

  void runFor(double seconds) throws InterruptedException {
    for (int i = 0; i < seconds / kLoopSeconds; i++) {
      tick();
    }
  }

  /** 10 ms steps on the real clock: intake sync every step, the 20 ms robot loop every other. */
  void tick() throws InterruptedException {
    m_nextLoopTime += kLoopSeconds;
    double wait = m_nextLoopTime - Timer.getFPGATimestamp();
    if (wait > 0) {
      Thread.sleep((long) (wait * 1000), (int) ((wait * 1e9) % 1_000_000));
    } else if (wait < -0.1) {
      m_nextLoopTime = Timer.getFPGATimestamp();
    }
    m_intake.syncLoop();
    if (m_loops++ % 2 == 0) {
      DriverStation.refreshData();
      m_vision.simulationPeriodic(m_drivetrain.getPose());
      CommandScheduler.getInstance().run();
    }
  }
}
