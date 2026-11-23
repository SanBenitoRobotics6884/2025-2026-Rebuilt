// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Constants;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  /** Log and replay timestamp and joystick data. */
  private final HootAutoReplay timeAndJoystickReplay = new HootAutoReplay()
      .withTimestampReplay()
      .withJoystickReplay();

  public Robot() {
    robotContainer = new RobotContainer();
    // Check the two intake lead screws every 10 ms, separately from the 20 ms main loop, so a jam
    // on one screw stops both quickly even if the command scheduler runs long.
    addPeriodic(robotContainer::intakeSyncLoop, Constants.Intake.kSyncPeriodSeconds);
  }

  @Override
  public void robotPeriodic() {
    timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    robotContainer.stopIntakeExtension();
    robotContainer.stopMechanisms();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(autonomousCommand);
    }
    // Covers going straight from auto to teleop with no disabled period in between, which
    // practice mode can do. An intake extend or retract in progress is left to finish.
    robotContainer.stopMechanisms();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    // Only runs in simulation, never on the real robot. Updates the fake Limelight.
    robotContainer.simulationPeriodic();
  }
}
