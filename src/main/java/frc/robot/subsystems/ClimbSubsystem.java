// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Constants.Climb.*;

public class ClimbSubsystem extends SubsystemBase {
  TalonFX m_climbmotor1 = new TalonFX(CLIMB_MOTOR_1_ID);
    TalonFX m_climbmotor2 = new TalonFX(CLIMB_MOTOR_2_ID);
    DutyCycleOut speed = new DutyCycleOut(DUTYCYCLE_OUTPUT);
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbUp() {
    m_climbmotor1.setControl(speed);
    m_climbmotor2.setControl(speed);

    m_climbmotor1.set(UP_CLIMB_SPEED);
    m_climbmotor2.set(UP_CLIMB_SPEED);
  }
  public void climbDown() {
    m_climbmotor1.setControl(speed);
    m_climbmotor2.setControl(speed);

    m_climbmotor1.set(DOWN_CLIMB_SPEED);
    m_climbmotor2.set(DOWN_CLIMB_SPEED);
  }

  public void stopMotors() {
    m_climbmotor1.set(0);
    m_climbmotor2.set(0);
  }

  public Command climbUpCommand() {
    return run(this::climbUp);
  }
   public Command climbDownCommand() {
    return run(this::climbDown);
  }

  public Command stopClimbCommand() {
    return runOnce(this::stopMotors);
  }
}
