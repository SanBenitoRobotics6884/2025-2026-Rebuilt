// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Constants.Container.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double slowSpeed = SLOW_SWERVE_SPEED;
  private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  /* Bindings for swerve drive. */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(maxSpeed * 0.1)
      .withRotationalDeadband(maxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(maxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController joystickOp = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final SendableChooser<Command> autoChooser;
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final OuttakeSubsystem outtake = new OuttakeSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

  /** Physical constraints for pathfinding. Linear units are m/s and m/s^2. Angular units are rad/s and rad/s^2. */
  public final PathConstraints pathConstraints = new PathConstraints(
      MAX_VELOCITY, MAX_ACCELERATION, MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);

  public RobotContainer() {
    NamedCommands.registerCommand("runIntakeCommand", intake.runIntakeCommand());
    NamedCommands.registerCommand("runIntakeBackCommand", intake.runIntakeBackteleCommand());
    NamedCommands.registerCommand("stopIntakeCommand", intake.stopIntakeCommand());
    NamedCommands.registerCommand("deployIntakeCommand", intake.extendIntakeCommand());
    NamedCommands.registerCommand("stopStorageCommand", intake.stopStorageCommand());
    NamedCommands.registerCommand("undepolyIntakeCommand", intake.retractIntakeCommand());
    NamedCommands.registerCommand("runRollersCommand", intake.runStorgeRollersCommand());
    NamedCommands.registerCommand("stopRollersCommand", intake.stopRollerCommand());
    NamedCommands.registerCommand("runIndexCommand", outtake.runIndexCommand());
    NamedCommands.registerCommand("stopIndexCommand", outtake.stopIndexCommand());
    NamedCommands.registerCommand("runOuttakecommand", outtake.runOuttakecommand());
    NamedCommands.registerCommand("stopOuttakeCommand", outtake.stopOuttakeCommand());

    drivetrain.configurePathPlanner();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    int controllerMode = 0;
    boolean isDriverControllerConnected = DriverStation.isJoystickConnected(0);
    boolean isOpControllerConnected = DriverStation.isJoystickConnected(1);
    if (isDriverControllerConnected && isOpControllerConnected) {
      controllerMode = 1;
    } else {
      controllerMode = 0;
    }
    switch (controllerMode) {
      case 1:
        bindMechanismControls(joystickOp);
        break;
      default:
        bindMechanismControls(joystick);
        break;
    }

    // Slow mode.
    joystick.leftBumper().whileTrue(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * maxSpeed * slowSpeed)
                .withVelocityY(-joystick.getLeftX() * maxSpeed * slowSpeed)
                .withRotationalRate(-joystick.getRightX() * maxAngularRate * slowSpeed)
        )
    );

    // X is forward and Y is left, per WPILib convention.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * maxSpeed)
                .withVelocityY(-joystick.getLeftX() * maxSpeed)
                .withRotationalRate(-joystick.getRightX() * maxAngularRate)
        )
    );

    // Idle while disabled so the configured neutral mode is applied to the drive motors.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    // SysId routines. Each routine should be run once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void bindMechanismControls(CommandXboxController controller) {
    controller.pov(D_PAD_RIGHT).whileTrue(Commands.sequence(intake.extendIntakeTeleCommand()))
        .onFalse(Commands.sequence(intake.stopStorageCommand()));
    controller.pov(D_PAD_DOWN).whileTrue(Commands.sequence(intake.retractIntakeTeleCommand()))
        .onFalse(Commands.sequence(intake.stopStorageCommand()));
    controller.leftTrigger().whileTrue(Commands.sequence(intake.runIntaketeleCommand()))
        .onFalse(Commands.sequence(intake.stopIntakeCommand()));
    controller.b().whileTrue(Commands.sequence(intake.runStorgeRollersteleCommand()))
        .onFalse(Commands.sequence(intake.stopIntakeCommand()));
    controller.a().whileTrue(Commands.sequence(intake.runStorgeRollersBackteleCommand()))
        .whileFalse(Commands.sequence(intake.stopIntakeCommand()));
    controller.a().whileTrue(Commands.sequence(intake.runIntakeBackteleCommand()))
        .onFalse(Commands.sequence(intake.stopIntakeCommand()));

    controller.rightTrigger().whileTrue(Commands.sequence(outtake.runOuttaketelecommand()))
        .onFalse(Commands.sequence(outtake.stopOuttakeCommand()));
    controller.b().whileTrue(Commands.sequence(outtake.runIndexteleCommand()))
        .onFalse(Commands.sequence(outtake.stopIndexCommand()));
    controller.x().whileTrue(Commands.sequence(outtake.runReverseIndexteleCommand()))
        .whileFalse(Commands.sequence(outtake.stopIndexCommand()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
