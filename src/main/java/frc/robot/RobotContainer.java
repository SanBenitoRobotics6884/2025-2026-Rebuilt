// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Constants.Container.*;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
import frc.robot.subsystems.vision.VisionSusbsytem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double slowSpeed = SLOW_SWERVE_SPEED; // Reduce speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
   
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystickOp = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    private final SendableChooser<Command> autoChooser;
    public IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    public OuttakeSubsystem m_OuttakeSubsystem = new OuttakeSubsystem();
    public VisionSusbsytem m_visionSubsystem = new VisionSusbsytem(drivetrain);

    // Physical constraints on the robot for the algorithim to be more accurate.
    public PathConstraints m_pathConstraints = new PathConstraints(
            MAX_VELOCITY, MAX_ACCELERATION, MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);
    
    public RobotContainer() {
        
    NamedCommands.registerCommand("runIntakeCommand", m_IntakeSubsystem.runIntakeCommand());
    NamedCommands.registerCommand("runIntakeBackCommand", m_IntakeSubsystem.runIntakeBackteleCommand());
    NamedCommands.registerCommand("stopIntakeCommand", m_IntakeSubsystem.stopIntakeCommand());
    NamedCommands.registerCommand("deployIntakeCommand", m_IntakeSubsystem.extendIntakeCommand());
    NamedCommands.registerCommand("stopStorageCommand", m_IntakeSubsystem.stopStorageCommand());
    NamedCommands.registerCommand("undepolyIntakeCommand", m_IntakeSubsystem.retractIntakeCommand());
    NamedCommands.registerCommand("runRollersCommand", m_IntakeSubsystem.runStorgeRollersCommand());
    NamedCommands.registerCommand("stopRollersCommand", m_IntakeSubsystem.stopRollerCommand());
    NamedCommands.registerCommand("runIndexCommand", m_OuttakeSubsystem.runIndexCommand());
    NamedCommands.registerCommand("stopIndexCommand", m_OuttakeSubsystem.stopIndexCommand());
    NamedCommands.registerCommand("runOuttakecommand", m_OuttakeSubsystem.runOuttakecommand());
    NamedCommands.registerCommand("stopOuttakeCommand", m_OuttakeSubsystem.stopOuttakeCommand());

    drivetrain.DriveSubsystem();
    
    if (AutoBuilder.isConfigured()) {
        System.out.print("AutoBuilder is configured");
        /*
        try {
        PathPlannerPath m_align = PathPlannerPath.fromPathFile(null);
        Command pathfindHubAlign = AutoBuilder.pathfindThenFollowPath(m_align, m_pathConstraints);
        } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
        }*/
    }
     autoChooser = AutoBuilder.buildAutoChooser();
     SmartDashboard.putData("Auto Chooser", autoChooser);
    // In meters per second.
        configureBindings();

    }

    private void configureBindings() {
        int controllerMode = 0;
        boolean isDriverControllerConnected = DriverStation.isJoystickConnected(0);
        boolean isOpControllerConnected = DriverStation.isJoystickConnected(1);
        if (isDriverControllerConnected && isOpControllerConnected) {
            controllerMode = 1; // 1st mode: both controllers operates
        } else {
            controllerMode = 0; // Default mode: one controller operates
        }
        switch (controllerMode) {
            case 1:
                //Intake controlls
                    joystickOp.pov(D_PAD_RIGHT).whileTrue(Commands.sequence(m_IntakeSubsystem.extendIntakeTeleCommand()))
                                               .onFalse(Commands.sequence(m_IntakeSubsystem.stopStorageCommand()));
                    joystickOp.pov(D_PAD_DOWN).whileTrue(Commands.sequence(m_IntakeSubsystem.retractIntakeTeleCommand()))
                                              .onFalse(Commands.sequence(m_IntakeSubsystem.stopStorageCommand()));
                    joystickOp.leftTrigger().whileTrue(Commands.sequence(m_IntakeSubsystem.runIntaketeleCommand()))
                                            .onFalse(Commands.sequence(m_IntakeSubsystem.stopIntakeCommand()));
                    joystickOp.b().whileTrue(Commands.sequence(m_IntakeSubsystem.runStorgeRollersteleCommand()))
                                  .onFalse(Commands.sequence(m_IntakeSubsystem.stopIntakeCommand()));
                    joystickOp.a().whileTrue(Commands.sequence(m_IntakeSubsystem.runStorgeRollersBackteleCommand()))
                                  .whileFalse(Commands.sequence(m_IntakeSubsystem.stopIntakeCommand()));
                    joystickOp.a().whileTrue(Commands.sequence(m_IntakeSubsystem.runIntakeBackteleCommand()))
                                  .onFalse(Commands.sequence(m_IntakeSubsystem.stopIntakeCommand()));
                    
                //OutTake controlls
                    joystickOp.rightTrigger().whileTrue(Commands.sequence(m_OuttakeSubsystem.runOuttaketelecommand()))
                                             .onFalse(Commands.sequence(m_OuttakeSubsystem.stopOuttakeCommand()));
                    joystickOp.b().whileTrue(Commands.sequence(m_OuttakeSubsystem.runIndexteleCommand()))
                                  .onFalse(Commands.sequence(m_OuttakeSubsystem.stopIndexCommand()));
                    joystickOp.x().whileTrue(Commands.sequence(m_OuttakeSubsystem.runReverseIndexteleCommand()))
                                  .whileFalse(Commands.sequence(m_OuttakeSubsystem.stopIndexCommand()));
                break;
            default:
                //Intake controlls
                    joystick.pov(D_PAD_RIGHT).whileTrue(Commands.sequence(m_IntakeSubsystem.extendIntakeTeleCommand()))
                                             .onFalse(Commands.sequence(m_IntakeSubsystem.stopStorageCommand()));
                    joystick.pov(D_PAD_DOWN).whileTrue(Commands.sequence(m_IntakeSubsystem.retractIntakeTeleCommand()))
                                            .onFalse(Commands.sequence(m_IntakeSubsystem.stopStorageCommand()));
                    joystick.leftTrigger().whileTrue(Commands.sequence(m_IntakeSubsystem.runIntaketeleCommand()))
                                          .onFalse(Commands.sequence(m_IntakeSubsystem.stopIntakeCommand()));
                    joystick.b().whileTrue(Commands.sequence(m_IntakeSubsystem.runStorgeRollersteleCommand()))
                                .onFalse(Commands.sequence(m_IntakeSubsystem.stopIntakeCommand()));
                    joystick.a().whileTrue(Commands.sequence(m_IntakeSubsystem.runStorgeRollersBackteleCommand()))
                                .whileFalse(Commands.sequence(m_IntakeSubsystem.stopIntakeCommand()));
                    joystick.a().whileTrue(Commands.sequence(m_IntakeSubsystem.runIntakeBackteleCommand()))
                                .onFalse(Commands.sequence(m_IntakeSubsystem.stopIntakeCommand()));
                    
                //OutTake controlls
                    joystick.rightTrigger().whileTrue(Commands.sequence(m_OuttakeSubsystem.runOuttaketelecommand()))
                                           .onFalse(Commands.sequence(m_OuttakeSubsystem.stopOuttakeCommand()));
                    joystick.b().whileTrue(Commands.sequence(m_OuttakeSubsystem.runIndexteleCommand()))
                                .onFalse(Commands.sequence(m_OuttakeSubsystem.stopIndexCommand()));
                    joystick.x().whileTrue(Commands.sequence(m_OuttakeSubsystem.runReverseIndexteleCommand()))
                                .whileFalse(Commands.sequence(m_OuttakeSubsystem.stopIndexCommand()));
        }
    
        
    //Limit Switch
        // new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), m_IntakeSubsystem::isLimitPressed) 
        // //.whileTrue(m_IntakeSubsystem.extendIntakeCommand());   
        // .onTrue(m_IntakeSubsystem.littleExtendIntakeCommand());
            

    //Slow Mode
        joystick.leftBumper().whileTrue(
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * slowSpeed) // Conley should play pressure with Jerry.
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * slowSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * slowSpeed) // JERRY IS A FAT GAY CHUD!
            )
        );
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
       
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize); 
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
       return autoChooser.getSelected();
    }
}
