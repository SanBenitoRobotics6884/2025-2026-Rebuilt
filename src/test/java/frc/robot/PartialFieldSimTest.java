// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.Constants.Score.kCloseDistanceMeters;
import static frc.robot.Constants.Constants.Vision.BLUE_HUB_TAG_IDS;
import static frc.robot.Constants.Constants.Vision.FORWARD_BLEND_SECONDS;
import static frc.robot.Constants.Constants.Vision.RED_HUB_TAG_IDS;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.ScoreHubCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.vision.HubMath;
import frc.robot.subsystems.vision.VisionSubsystem;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/**
 * Practice on a partial field: only the blue hub and its AprilTags, no auto routine to set the
 * robot's position, and the robot powered up somewhere else than it thinks.
 *
 * <p>The test keeps the robot's TRUE position itself, moved only by how the wheels actually
 * turned, and shows that to the simulated camera. The robot's own estimate starts wrong (where
 * every boot starts) and can only be fixed by seeing the hub. Then the robot drives in to the close
 * shot distance, where the level camera cannot see the hub, and holds the right trigger.
 *
 * <p>Fails if the shot runs in POSE mode with an RPM that does not match the true distance: that
 * would be a shot aimed with a wrong position. FIXED is reported but allowed, since it is the safe
 * fallback when the position cannot be trusted.
 *
 * <p>Run with {@code ./gradlew teleopSim} (real time, not part of {@code ./gradlew build}).
 */
@Tag("teleopSim")
class PartialFieldSimTest {
  static final double kLoopSeconds = 0.01;
  /** The robot starts this far from the hub's middle, in the hub's alliance zone. */
  static final double kStartMeters = 3.6;
  /** ...this many degrees off straight out from the hub's alliance-wall face. */
  static final double kStartAngleDegrees = 30.0;
  static final double kDriveSpeedMetersPerSecond = 1.0;
  static final double kShotSeconds = 2.0;
  /** In POSE mode the RPM must be within this of the true distance's RPM. */
  static final double kRpmToleranceRpm = 100.0;
  /** The driver's "forward" may not move more than this in one 20 ms loop (no jumps). */
  static final double kMaxForwardStepDegrees = 5.0;
  /** ...and must be this close to standard once FORWARD_BLEND_SECONDS have passed. */
  static final double kForwardSettledDegrees = 3.0;

  CommandSwerveDrivetrain m_drivetrain;
  OuttakeSubsystem m_outtake;
  IntakeSubsystem m_intake;
  VisionSubsystem m_vision;
  CommandXboxController m_driver;
  double m_maxSpeed;
  double m_maxTurn;
  long m_loops;
  double m_nextLoopTime;
  /** Where the robot really is. Moved only by its measured wheel motion, never by vision. */
  Pose2d m_truth = Pose2d.kZero;
  /** Which real-world direction the driver's joystick "forward" pointed last loop, or null. */
  Rotation2d m_lastDriverForward;
  double m_maxForwardStepDegrees;
  double m_enabledResetSeconds = Double.NaN;
  double m_resetsSoFar;
  final List<String> m_report = new ArrayList<>();
  final List<String> m_failures = new ArrayList<>();

  @Test
  void practiceWithOnlyTheHub() throws InterruptedException {
    assertTrue(HAL.initialize(500, 0), "HAL did not start");
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setJoystickAxisCount(0, 6);
    DriverStationSim.setJoystickButtonCount(0, 10);
    DriverStationSim.setJoystickPOVCount(0, 1);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    m_drivetrain = TunerConstants.createDrivetrain();
    m_outtake = new OuttakeSubsystem();
    m_intake = new IntakeSubsystem();
    m_vision = new VisionSubsystem(m_drivetrain);
    SwerveRequest.RobotCentric hold =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
    m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(() -> hold));
    m_maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    m_maxTurn = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    m_driver = new CommandXboxController(0);
    m_nextLoopTime = Timer.getFPGATimestamp();

    // The practice hub carries one alliance's tags; the Driver Station picks the alliance.
    runScenario("Blue A: sees the hub while disabled, then drives in", false, false, 2.0);
    runScenario("Blue B: enabled right away, then drives in", false, false, 0.0);
    runScenario("Red A: sees the hub while disabled, then drives in", true, true, 2.0);
    runScenario("Red B: enabled right away, then drives in", true, true, 0.0);
    // A mix-up: a hub with blue tags, but the Driver Station set to a red station.
    runScenario("Mix-up: blue-tag hub, Driver Station on red", false, true, 2.0);

    System.out.println();
    System.out.println("Partial field (only one hub's tags), robot booted at the wrong spot:");
    m_report.forEach(line -> System.out.println("  " + line));
    System.out.println();
    if (!m_failures.isEmpty()) {
      fail("Partial-field problems:\n  " + String.join("\n  ", m_failures));
    }
  }

  /**
   * @param redHub the practice hub carries the red hub's tags (else blue's)
   * @param redAlliance the Driver Station is set to a red station (else blue)
   */
  void runScenario(String name, boolean redHub, boolean redAlliance, double disabledLookSeconds)
      throws InterruptedException {
    m_vision.simUseOnlyTags(redHub ? RED_HUB_TAG_IDS : BLUE_HUB_TAG_IDS);
    DriverStationSim.setAllianceStationId(
        redAlliance ? AllianceStationID.Red1 : AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().cancelAll();
    setEnabled(false);
    // The hub that is physically there (the one the robot really drives up to).
    Translation2d hub = HubMath.hubCenter(redHub);
    // Truly: in that hub's alliance zone, kStartAngleDegrees off straight out from its
    // alliance-wall face, facing the hub. The robot will believe it faces 0 deg, so its heading
    // starts wrong on both alliances.
    Rotation2d out = (redHub ? Rotation2d.kZero : Rotation2d.k180deg)
        .plus(Rotation2d.fromDegrees(redHub ? kStartAngleDegrees : -kStartAngleDegrees));
    Translation2d startSpot = hub.plus(new Translation2d(kStartMeters, 0).rotateBy(out));
    m_truth = new Pose2d(startSpot, out.plus(Rotation2d.k180deg));
    // What the robot believes at power-up: the origin, facing 0 deg.
    m_drivetrain.resetPose(Pose2d.kZero);
    m_drivetrain.getPigeon2().setYaw(0);
    runFor(0.3);
    m_drivetrain.resetPose(Pose2d.kZero);
    m_vision.forgetTrustedPose(); // as after a power-up
    m_report.add(name + ":");
    m_report.add("  boot:          " + compare());

    // Sit disabled, looking at the hub (pits / queue), if this scenario does.
    runFor(disabledLookSeconds);
    if (disabledLookSeconds > 0) {
      m_report.add("  after looking: " + compare());
    }

    // Enable and drive straight in until truly at the close shot distance. The camera can see the
    // hub at the start of the drive but not at the end.
    setEnabled(true);
    m_lastDriverForward = null;
    m_maxForwardStepDegrees = 0;
    m_enabledResetSeconds = Double.NaN;
    m_resetsSoFar = SmartDashboard.getNumber("Vision/PoseResets", 0);
    SwerveRequest.RobotCentric forward = new SwerveRequest.RobotCentric()
        .withVelocityX(kDriveSpeedMetersPerSecond)
        .withDriveRequestType(DriveRequestType.Velocity);
    Command drive = m_drivetrain.applyRequest(() -> forward);
    CommandScheduler.getInstance().schedule(drive);
    double start = Timer.getFPGATimestamp();
    while (hub.getDistance(m_truth.getTranslation()) > kCloseDistanceMeters
        && Timer.getFPGATimestamp() - start < 5.0) {
      tick();
    }
    CommandScheduler.getInstance().cancel(drive);
    runFor(0.3); // stop
    m_report.add("  at the shot:   " + compare());

    // Hold the right trigger.
    SmartDashboard.putString("Score/Mode", "");
    ScoreHubCommand shot = new ScoreHubCommand(
        m_drivetrain, m_outtake, m_intake, m_vision, m_driver, m_maxSpeed, m_maxTurn, 0.5);
    CommandScheduler.getInstance().schedule(shot);
    runFor(kShotSeconds);
    String mode = SmartDashboard.getString("Score/Mode", "?");
    double rpm = SmartDashboard.getNumber("Score/TargetRpm", 0);
    double believed = SmartDashboard.getNumber("Score/PoseDistanceMeters", Double.NaN);
    CommandScheduler.getInstance().cancel(shot);
    runFor(0.1);

    // The driver's "forward": it must never jump, and must be back to standard
    // FORWARD_BLEND_SECONDS after a reset made while driving.
    if (!Double.isNaN(m_enabledResetSeconds)) {
      double wait = FORWARD_BLEND_SECONDS + 0.5 - (Timer.getFPGATimestamp() - m_enabledResetSeconds);
      runFor(Math.max(0, wait));
    }
    Rotation2d standard = redAlliance ? Rotation2d.k180deg : Rotation2d.kZero;
    double forwardOff = Math.abs(driverForward().minus(standard).getDegrees());
    m_report.add(String.format("  driver forward: %s; never moved more than %.1f deg per 20 ms;"
        + " now %.1f deg from standard",
        Double.isNaN(m_enabledResetSeconds)
            ? "no reset while driving"
            : "reset while driving, eased back at " + (180.0 / FORWARD_BLEND_SECONDS) + " deg/s",
        m_maxForwardStepDegrees, forwardOff));
    if (m_maxForwardStepDegrees > kMaxForwardStepDegrees) {
      m_failures.add(String.format("%s: the driver's forward jumped %.1f deg in one loop", name,
          m_maxForwardStepDegrees));
    }
    if (!redHub == !redAlliance && forwardOff > kForwardSettledDegrees) {
      m_failures.add(String.format("%s: the driver's forward is still %.1f deg from standard",
          name, forwardOff));
    }

    double trueMeters = hub.getDistance(m_truth.getTranslation());
    double rightRpm = HubMath.rpmForDistance(trueMeters);
    m_report.add(String.format("  shot: mode %s, %.0f RPM; true distance %.2f m needs %.0f RPM"
        + " (robot believed %.2f m)", mode, rpm, trueMeters, rightRpm, believed));
    if ("POSE".equals(mode) && Math.abs(rpm - rightRpm) > kRpmToleranceRpm) {
      m_failures.add(String.format("%s: POSE mode shot at %.0f RPM, but the true distance needs"
          + " %.0f (it believed %.2f m, truly %.2f m)", name, rpm, rightRpm, believed, trueMeters));
    }
  }

  /**
   * The real-world direction (true field frame) the driver's joystick "forward" points: the robot's
   * true heading, plus how far the drivetrain's "forward" is from where it thinks it faces.
   */
  Rotation2d driverForward() {
    return m_truth.getRotation().plus(
        m_drivetrain.getOperatorForwardDirection().minus(m_drivetrain.getPose().getRotation()));
  }

  /** "estimate vs truth" for the report. */
  String compare() {
    Pose2d est = m_drivetrain.getPose();
    return String.format("believes (%.2f, %.2f, %.0f deg), truly (%.2f, %.2f, %.0f deg): off by"
            + " %.2f m and %.0f deg",
        est.getX(), est.getY(), est.getRotation().getDegrees(), m_truth.getX(), m_truth.getY(),
        m_truth.getRotation().getDegrees(),
        est.getTranslation().getDistance(m_truth.getTranslation()),
        Math.abs(est.getRotation().minus(m_truth.getRotation()).getDegrees()));
  }

  void setEnabled(boolean enabled) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  void runFor(double seconds) throws InterruptedException {
    for (int i = 0; i < seconds / kLoopSeconds; i++) {
      tick();
    }
  }

  /**
   * 10 ms steps on the real clock. Every 20 ms: move the truth by the wheels' measured motion, show
   * the truth to the camera, and run the robot loop.
   */
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
      ChassisSpeeds speeds = m_drivetrain.getState().Speeds; // robot-relative, from the wheels
      double dt = 2 * kLoopSeconds;
      m_truth = m_truth.exp(new edu.wpi.first.math.geometry.Twist2d(
          speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt,
          speeds.omegaRadiansPerSecond * dt));
      DriverStation.refreshData();
      m_vision.simulationPeriodic(m_truth);
      CommandScheduler.getInstance().run();
      if (DriverStation.isEnabled()) {
        double resets = SmartDashboard.getNumber("Vision/PoseResets", 0);
        if (resets > m_resetsSoFar && Double.isNaN(m_enabledResetSeconds)) {
          m_enabledResetSeconds = Timer.getFPGATimestamp();
        }
        m_resetsSoFar = resets;
        Rotation2d forward = driverForward();
        if (m_lastDriverForward != null) {
          m_maxForwardStepDegrees = Math.max(m_maxForwardStepDegrees,
              Math.abs(forward.minus(m_lastDriverForward).getDegrees()));
        }
        m_lastDriverForward = forward;
      }
    }
  }
}
