// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Constants.OperatorConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.ScoreHubCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double slowSpeed = kSlowDriveScale;
  private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  /* Bindings for swerve drive. */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(maxSpeed * 0.1)
      .withRotationalDeadband(maxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(maxSpeed);

  private final CommandXboxController m_driverController =
      new CommandXboxController(kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(kOperatorControllerPort);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final SendableChooser<Command> autoChooser;
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final OuttakeSubsystem m_outtake = new OuttakeSubsystem();
  // Just constructing this starts vision. Subsystems are polled every robot cycle, so periodic()
  // keeps reading the Limelight without a button, a default command, or an auto step.
  private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

  /** Physical constraints for pathfinding. Linear units are m/s and m/s^2. Angular units are rad/s and rad/s^2. */
  public final PathConstraints pathConstraints = new PathConstraints(
      kMaxVelocity, kMaxAcceleration, kMaxAngularVelocity, kMaxAngularAcceleration);

  public RobotContainer() {
    // Auto names stay as they are so existing PathPlanner files keep resolving.
    // One-shot commands set the motors and end. Held commands are for the driver buttons.
    NamedCommands.registerCommand("runIntakeCommand", m_intake.runOnce(m_intake::intake));
    NamedCommands.registerCommand("runIntakeBackCommand", m_intake.reverseIntakeCommand());
    NamedCommands.registerCommand("stopIntakeCommand", m_intake.stopRollersCommand());
    NamedCommands.registerCommand("deployIntakeCommand", m_intake.runOnce(m_intake::extend));
    NamedCommands.registerCommand("stopStorageCommand", m_intake.stopScrewsCommand());
    NamedCommands.registerCommand("undepolyIntakeCommand", m_intake.runOnce(m_intake::retract));
    NamedCommands.registerCommand("runRollersCommand", m_intake.runOnce(m_intake::runStorage));
    NamedCommands.registerCommand("stopRollersCommand", m_intake.stopStorageCommand());
    NamedCommands.registerCommand("runIndexCommand", m_outtake.startIndexerCommand());
    NamedCommands.registerCommand("stopIndexCommand", m_outtake.stopIndexerCommand());
    // Stow the extension when the shooter starts, and put it back on stop if that shot began
    // deployed. The wait already in the autos is the time the screws have to get in before a ball
    // is fed.
    NamedCommands.registerCommand(
        "runOuttakecommand",
        m_intake.runOnce(m_intake::retractForShot).andThen(m_outtake.startShooterCommand()));
    NamedCommands.registerCommand(
        "stopOuttakeCommand",
        m_outtake.stopShooterCommand().andThen(m_intake.runOnce(m_intake::restoreAfterShot)));

    drivetrain.configurePathPlanner();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    int controllerMode = 0;
    boolean isDriverControllerConnected = DriverStation.isJoystickConnected(kDriverControllerPort);
    boolean isOpControllerConnected = DriverStation.isJoystickConnected(kOperatorControllerPort);
    if (isDriverControllerConnected && isOpControllerConnected) {
      controllerMode = 1;
    } else {
      controllerMode = 0;
    }
    switch (controllerMode) {
      case 1:
        bindMechanismControls(m_operatorController);
        break;
      default:
        bindMechanismControls(m_driverController);
        break;
    }

    // Slow mode.
    m_driverController.leftBumper().whileTrue(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-m_driverController.getLeftY() * maxSpeed * slowSpeed)
                .withVelocityY(-m_driverController.getLeftX() * maxSpeed * slowSpeed)
                .withRotationalRate(-m_driverController.getRightX() * maxAngularRate * slowSpeed)
        )
    );

    // X is forward and Y is left, per WPILib convention.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-m_driverController.getLeftY() * maxSpeed)
                .withVelocityY(-m_driverController.getLeftX() * maxSpeed)
                .withRotationalRate(-m_driverController.getRightX() * maxAngularRate)
        )
    );

    // Idle while disabled so the configured neutral mode is applied to the drive motors.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    // SysId routines. Each routine should be run once in a single log.
    m_driverController.back().and(m_driverController.y())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController.back().and(m_driverController.x())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController.start().and(m_driverController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driverController.start().and(m_driverController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    m_driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void bindMechanismControls(CommandXboxController controller) {
    // kPovDown is angle 180, which this robot previously called D_PAD_RIGHT.
    // kPovRight is angle 90, which this robot previously called D_PAD_DOWN.
    controller.pov(kPovDown).whileTrue(m_intake.extendCommand())
        .onFalse(m_intake.stopScrewsCommand());
    controller.pov(kPovRight).whileTrue(m_intake.retractCommand())
        .onFalse(m_intake.stopScrewsCommand());
    // D-pad up sweeps both hard stops and writes the measured extension. Releasing the button
    // does not cancel the sweep. Teleop only, so the button cannot take the intake during auto.
    controller.pov(kPovUp)
        .and(RobotModeTriggers.teleop())
        .onTrue(m_intake.recalibrateCommand());
    controller.leftTrigger().whileTrue(m_intake.intakeCommand())
        .onFalse(m_intake.stopRollersCommand());
    controller.b().whileTrue(m_intake.runStorageCommand())
        .onFalse(m_intake.stopRollersCommand());
    controller.a().whileTrue(m_intake.reverseStorageCommand())
        .whileFalse(m_intake.stopRollersCommand());
    controller.a().whileTrue(m_intake.reverseIntakeCommand())
        .onFalse(m_intake.stopRollersCommand());

    // Turn toward our hub AprilTag, spin the shooter for the measured distance, and feed a ball
    // only once the wheels reach that RPM. The extension stows for the shot and, if it started
    // deployed, goes back out when the trigger is released. The driver's left stick still drives.
    controller.rightTrigger().whileTrue(
        new ScoreHubCommand(
            drivetrain,
            m_outtake,
            m_intake,
            vision,
            m_driverController,
            maxSpeed,
            maxAngularRate,
            slowSpeed));
    controller.b().whileTrue(m_outtake.indexCommand())
        .onFalse(m_outtake.stopIndexerCommand());
    controller.x().whileTrue(m_outtake.reverseIndexCommand())
        .whileFalse(m_outtake.stopIndexerCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Writes the live screw position so the next power cycle can seed the encoder. */
  public void saveIntakeExtension() {
    m_intake.saveMeasuredExtension();
  }

  /**
   * Simulation only. The real Limelight is not here, so each cycle we tell the fake camera where
   * the simulated robot is. It then pretends to see whatever AprilTags would be in front of that
   * pose. WPILib calls this after the main robot loop.
   */
  public void simulationPeriodic() {
    vision.simulationPeriodic(drivetrain.getPose());
  }
}
