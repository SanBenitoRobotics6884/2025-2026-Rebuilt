// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.Constants.Score.kCloseDistanceMeters;
import static frc.robot.Constants.Constants.Score.kFarDistanceMeters;
import static frc.robot.Constants.Constants.Vision.CAM_HEIGHT;
import static frc.robot.Constants.Constants.Vision.CAM_PITCH;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.TreeSet;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.ScoreHubCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.vision.HubMath;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.HubSighting;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/**
 * Checks the hub auto-aim against the simulated Limelight from many places around the hub.
 *
 * <p>Each spot is a distance from the middle of the hub, out through one of its faces. The robot
 * is placed there, given a second for the camera to see, then the right trigger is held. It
 * reports which hub tags were used, the measured distance to the hub's middle against the truth,
 * the RPM picked, and how far the robot ends up from pointing at the hub's middle.
 *
 * <p>Fails when a spot outside our alliance zone is not FIXED (rule G407), when a hub tag is in
 * view in our zone but the distance or aim is wrong, or when at the two tuned
 * shot distances (kCloseDistanceMeters, kFarDistanceMeters) neither the tag nor the POSE fallback
 * gives the right RPM.
 *
 * <p>Run with {@code ./gradlew teleopSim} (real time, not part of {@code ./gradlew build}).
 */
@Tag("teleopSim")
class AutoAimSimTest {
  static final double kLoopSeconds = 0.01;
  static final double kHoldSeconds = 3.0;
  /** Measured distance to the hub's middle must be within this of the truth, in meters. */
  static final double kDistanceToleranceMeters = 0.15;
  /** Aim must end within this many degrees of the hub's middle. */
  static final double kAimToleranceDegrees = 3.0;
  /** Without a hub tag, the POSE fallback's RPM must be within this of the true distance's RPM. */
  static final double kRpmToleranceRpm = 100.0;

  /** Which face of a hub the robot stands in front of. */
  enum Face {
    ALLIANCE_WALL,
    CENTER_FIELD,
    LEFT_SIDE,
    RIGHT_SIDE
  }

  /**
   * @param meters distance from the hub's middle to the robot center
   * @param sidewaysMeters slide along the face, positive to the robot's left
   * @param headingOffsetDegrees start turned this far from pointing at the hub (positive = left)
   * @param mustSee fail if this spot does not get the right RPM, from the tag or the POSE fallback
   */
  record Spot(String name, boolean red, Face face, double meters, double sidewaysMeters,
      double headingOffsetDegrees, boolean mustSee) {}

  CommandSwerveDrivetrain m_drivetrain;
  OuttakeSubsystem m_outtake;
  IntakeSubsystem m_intake;
  VisionSubsystem m_vision;
  CommandXboxController m_driver;
  double m_maxSpeed;
  double m_maxTurn;
  long m_loops;
  double m_nextLoopTime;
  final List<String> m_rows = new ArrayList<>();
  final List<String> m_failures = new ArrayList<>();

  @Test
  void autoAimFromAroundTheHub() throws InterruptedException {
    assertTrue(HAL.initialize(500, 0), "HAL did not start");
    DriverStation.silenceJoystickConnectionWarning(true);
    setAlliance(false);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
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
    m_maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    m_maxTurn = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    m_driver = new CommandXboxController(0);
    m_nextLoopTime = Timer.getFPGATimestamp();
    runFor(0.5);

    List<Spot> spots = List.of(
        new Spot("Blue wall face, close tune pt", false, Face.ALLIANCE_WALL, kCloseDistanceMeters,
            0, 0, true),
        new Spot("Blue wall face, 2.6 m", false, Face.ALLIANCE_WALL, 2.6, 0, 0, false),
        new Spot("Blue wall face, 3.1 m", false, Face.ALLIANCE_WALL, 3.1, 0, 0, false),
        new Spot("Blue wall face, 3.6 m", false, Face.ALLIANCE_WALL, 3.6, 0, 0, false),
        new Spot("Blue wall face, far tune pt", false, Face.ALLIANCE_WALL, kFarDistanceMeters,
            0, 0, true),
        new Spot("Blue wall face, turned 20 deg", false, Face.ALLIANCE_WALL, 3.1, 0, 20, false),
        new Spot("Blue wall face, 1 m to side", false, Face.ALLIANCE_WALL, 3.1, 1.0, 0, false),
        new Spot("Blue center-field face", false, Face.CENTER_FIELD, 3.1, 0, 0, false),
        new Spot("Blue left side face", false, Face.LEFT_SIDE, 3.1, 0, 0, false),
        new Spot("Blue right side face", false, Face.RIGHT_SIDE, 3.1, 0, 0, false),
        new Spot("Red wall face, 3.1 m", true, Face.ALLIANCE_WALL, 3.1, 0, 0, false),
        new Spot("Red left side face", true, Face.LEFT_SIDE, 3.1, 0, 0, false));
    for (Spot spot : spots) {
      runSpot(spot);
    }

    System.out.println();
    System.out.printf(
        "Auto-aim at the hub's middle (simulated Limelight 3, camera pitch %.0f deg (negative ="
            + " up), %.1f in up):%n",
        Math.toDegrees(CAM_PITCH), CAM_HEIGHT / 0.0254);
    System.out.printf("  %-30s %-17s %-7s %-7s %-6s %-9s %-8s %s%n", "Spot", "Hub tags used",
        "True m", "Meas m", "RPM", "Mode", "Aim deg", "Hub seen");
    m_rows.forEach(row -> System.out.println("  " + row));
    System.out.println();
    if (!m_failures.isEmpty()) {
      fail("Auto-aim problems:\n  " + String.join("\n  ", m_failures));
    }
  }

  void runSpot(Spot spot) throws InterruptedException {
    setAlliance(spot.red());
    CommandScheduler.getInstance().cancelAll();
    m_outtake.stopShooter();
    m_outtake.stopIndexer();

    Translation2d hub = spot.red() ? HubMath.RED_HUB_CENTER : HubMath.BLUE_HUB_CENTER;
    Rotation2d out = faceDirection(spot.red(), spot.face());
    Translation2d spotPosition = hub
        .plus(new Translation2d(spot.meters(), 0).rotateBy(out))
        .plus(new Translation2d(0, -spot.sidewaysMeters()).rotateBy(out));
    Rotation2d towardHub = hub.minus(spotPosition).getAngle();
    Pose2d start = new Pose2d(spotPosition,
        towardHub.plus(Rotation2d.fromDegrees(spot.headingOffsetDegrees())));

    // Place the robot until it stays put. Camera frames arrive about 35 ms late and are applied
    // against where the robot was then; right after a teleport that is the previous spot, which
    // drags the pose. A real robot never teleports, so this is only a test-setup step.
    for (int attempt = 0; attempt < 5 && !atSpot(start); attempt++) {
      m_drivetrain.resetPose(start);
      runFor(0.4);
    }
    // Settle, keeping the first hub sighting. Single tags far away are sometimes rejected as too
    // ambiguous for a frame or two, so one instant is not a fair sample.
    Optional<HubSighting> first = Optional.empty();
    for (int i = 0; i < 0.6 / kLoopSeconds; i++) {
      tick();
      if (first.isEmpty()) {
        first = m_vision.getHubSighting();
      }
    }
    if (!atSpot(start)) {
      Pose2d p = m_drivetrain.getPose();
      m_failures.add(String.format("%s: test could not place the robot (at %.2f, %.2f)",
          spot.name(), p.getX(), p.getY()));
    }

    String visionBefore = SmartDashboard.getString("Vision/HubStatus", "?");
    TreeSet<Integer> tagsUsed = new TreeSet<>();
    first.ifPresent(h -> tagsUsed.add(h.tagId()));
    int maxTagsAtOnce = first.map(HubSighting::tagCount).orElse(0);

    SmartDashboard.putString("Score/Mode", "");
    ScoreHubCommand shot = new ScoreHubCommand(
        m_drivetrain, m_outtake, m_intake, m_vision, m_driver, m_maxSpeed, m_maxTurn, 0.5);
    CommandScheduler.getInstance().schedule(shot);
    double startTime = Timer.getFPGATimestamp();
    int ticks = 0;
    int ticksSeen = 0;
    while (Timer.getFPGATimestamp() - startTime < kHoldSeconds) {
      tick();
      ticks++;
      Optional<HubSighting> now = m_vision.getHubSighting();
      if (now.isPresent()) {
        ticksSeen++;
        tagsUsed.add(now.get().tagId());
        maxTagsAtOnce = Math.max(maxTagsAtOnce, now.get().tagCount());
      }
    }
    String mode = SmartDashboard.getString("Score/Mode", "?");
    double rpm = SmartDashboard.getNumber("Score/TargetRpm", 0);
    CommandScheduler.getInstance().cancel(shot);
    runFor(0.1);

    double trueMeters = hub.getDistance(start.getTranslation());
    double measured = first.map(HubSighting::distanceMeters).orElse(Double.NaN);
    double aim = bearingErrorDegrees(m_drivetrain.getPose(), hub);
    String tags = tagsUsed.isEmpty()
        ? "none"
        : tagsUsed.toString().replace(" ", "") + (maxTagsAtOnce > 1 ? " x" + maxTagsAtOnce : "");

    m_rows.add(String.format("%-30s %-17s %-7.2f %-7s %-6.0f %-9s %-8.1f %3.0f%%", spot.name(),
        tags, trueMeters, fmt(measured), rpm, mode, aim, 100.0 * ticksSeen / Math.max(1, ticks)));

    // Scoring is only legal from our alliance zone (G407): anywhere else the shot must be FIXED,
    // even with a hub tag in view.
    if (!HubMath.inAllianceZone(start.getTranslation(), spot.red())) {
      m_rows.set(m_rows.size() - 1, m_rows.get(m_rows.size() - 1) + "  (outside our zone)");
      if (!"FIXED".equals(mode)) {
        m_failures.add(spot.name() + ": outside our alliance zone but mode was " + mode);
      }
      return;
    }
    if (first.isEmpty()) {
      // No hub tag: the position fallback should still pick the RPM for the true distance.
      double expectedRpm = HubMath.rpmForDistance(trueMeters);
      boolean poseRpmOk = "POSE".equals(mode) && Math.abs(rpm - expectedRpm) <= kRpmToleranceRpm;
      if (spot.mustSee() && !poseRpmOk) {
        m_failures.add(String.format("%s: no hub tag in view and mode %s gave %.0f RPM; the"
            + " distance needs %.0f", spot.name(), mode, rpm, expectedRpm));
      }
      return;
    }
    if (Math.abs(measured - trueMeters) > kDistanceToleranceMeters) {
      m_failures.add(String.format("%s: measured %.2f m to the hub's middle, true %.2f m",
          spot.name(), measured, trueMeters));
    }
    if (Math.abs(aim) > kAimToleranceDegrees) {
      m_failures.add(String.format("%s: ended %.1f deg off the hub's middle", spot.name(), aim));
    }
    if (!"AUTO_AIM".equals(mode)) {
      m_failures.add(spot.name() + ": saw a hub tag but ended in mode " + mode);
    }
  }

  boolean atSpot(Pose2d spot) {
    Pose2d now = m_drivetrain.getPose();
    return now.getTranslation().getDistance(spot.getTranslation()) < 0.05
        && Math.abs(now.getRotation().minus(spot.getRotation()).getDegrees()) < 2.0;
  }

  /** The direction pointing out of the given face of the given hub, in field coordinates. */
  static Rotation2d faceDirection(boolean red, Face face) {
    // Blue's alliance wall is at x = 0, so its hub's wall face points toward -x; red mirrors it.
    Rotation2d towardOwnWall = red ? Rotation2d.kZero : Rotation2d.k180deg;
    return switch (face) {
      case ALLIANCE_WALL -> towardOwnWall;
      case CENTER_FIELD -> towardOwnWall.plus(Rotation2d.k180deg);
      case LEFT_SIDE -> Rotation2d.kCCW_90deg;
      case RIGHT_SIDE -> Rotation2d.kCW_90deg;
    };
  }

  /** Degrees between where the robot points and the direction to {@code target}. */
  static double bearingErrorDegrees(Pose2d robot, Translation2d target) {
    Rotation2d toTarget = target.minus(robot.getTranslation()).getAngle();
    return toTarget.minus(robot.getRotation()).getDegrees();
  }

  static String fmt(double value) {
    return Double.isNaN(value) ? "-" : String.format("%.2f", value);
  }

  void setAlliance(boolean red) {
    DriverStationSim.setAllianceStationId(red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
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
