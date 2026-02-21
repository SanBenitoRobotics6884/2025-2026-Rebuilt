// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.ObjectInputStream.GetField;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.Constants.Intake.*;

public class RobotContainer {
    private DigitalInput i_limitswitchundeploy = new DigitalInput(LIMIT_SWITCH_UNDEPLOY_ID);

   
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double slowSpeed; // Reduce speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick m_Joystick = new Joystick(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser;
     public IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
     public OuttakeSubsystem m_OuttakeSubsystem = new OuttakeSubsystem();
     public ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();



    public RobotContainer() {
        drivetrain.DriveSubsystem();
        if (AutoBuilder.isConfigured()) {
            System.out.print("it is configured");
        }
        //config

     // For convenience a programmer could change this when going to competition.
     autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("middle start to left start to midfield to right collecting then shooting then climb", autoChooser);
    SmartDashboard.putData("middle start to left start to midfield to right collecting then shooting", autoChooser);
    SmartDashboard.putData("Middle start to right start to midfield to left collecting then shooting then climb", autoChooser);
    SmartDashboard.putData("Middle start to right start to midfield to left collecting then shooting", autoChooser);
    SmartDashboard.putData("Middle start shoot then climb", autoChooser);
    SmartDashboard.putData("Left start to midfield to right collecting then shooting then climb", autoChooser);
    SmartDashboard.putData("Left start to midfield to right collecting then shooting", autoChooser);
    SmartDashboard.putData("Left to middle of alliance field shoot then climb", autoChooser);
    SmartDashboard.putData("Right start to midfield to left collecting then shooting then climb", autoChooser);
    SmartDashboard.putData("Right start to midfield to left collecting then shooting", autoChooser);
    SmartDashboard.putData("Right start to midle shoot then climb", autoChooser);



        configureBindings();
    
    NamedCommands.registerCommand("RunIntakeCommand", m_IntakeSubsystem.runIntakeCommand());
    NamedCommands.registerCommand("StopIntakeCommand", m_IntakeSubsystem.stopTakeCommand());
    NamedCommands.registerCommand("DeployIntakeCommand", m_IntakeSubsystem.deployIntakeCommand());
    NamedCommands.registerCommand("UndepolyIntakeCommand", m_IntakeSubsystem.undeployIntakeCommand());
    NamedCommands.registerCommand("StopInOutTakeCommand", m_IntakeSubsystem.stopInOutTakeCommand());
    NamedCommands.registerCommand("RunOuttakeCommand", m_OuttakeSubsystem.runOuttakecommand());
    NamedCommands.registerCommand("StopIntakeCommand", m_OuttakeSubsystem.stopOuttakeCommand());
    }

    private void configureBindings() {
      // i_limitSwitchdeploy.whileTrue(Commands.sequence(m_IntakeSubsystem.stopInOutTakeCommand()));
       

        joystick.pov(90).whileTrue(Commands.sequence(m_IntakeSubsystem.deployIntakeCommand()))
                               .onFalse(Commands.sequence(m_IntakeSubsystem.stopInOutTakeCommand()));
        joystick.pov(180).whileTrue(Commands.sequence(m_IntakeSubsystem.undeployIntakeCommand()))
                              .onFalse(Commands.sequence(m_IntakeSubsystem.stopInOutTakeCommand()));
        joystick.leftTrigger().whileTrue(Commands.sequence(m_IntakeSubsystem.runIntakeCommand()))
                              .onFalse(Commands.sequence(m_IntakeSubsystem.stopTakeCommand()));
        joystick.b().whileTrue(Commands.sequence(m_IntakeSubsystem.runStorgeRollersCommand()))
                    .onFalse(Commands.sequence(m_IntakeSubsystem.stopTakeCommand()));

        joystick.rightTrigger().whileTrue(Commands.sequence(m_OuttakeSubsystem.runOuttakecommand()))
                               .onFalse(Commands.sequence(m_OuttakeSubsystem.stopOuttakeCommand()));

        joystick.pov(0).whileTrue(Commands.sequence(m_ClimbSubsystem.climbUpCommand()))
                    .onFalse(Commands.sequence(m_ClimbSubsystem.stopClimbCommand()));
        joystick.pov(270).whileTrue(Commands.sequence(m_ClimbSubsystem.climbDownCommand()))
                    .onFalse(Commands.sequence(m_ClimbSubsystem.stopClimbCommand()));
        
         if (i_limitswitchundeploy.get()) {
            m_IntakeSubsystem.stopInOutTakeCommand();
        } // silly thingy here :applause:

        if (m_Joystick.getRawButton(5)) {
            slowSpeed = 0.5;
        } else {
            slowSpeed = 1;
        }
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * slowSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * slowSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * slowSpeed) // Drive counterclockwise with negative X (left)
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
