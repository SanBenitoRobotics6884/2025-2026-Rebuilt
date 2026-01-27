// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  TalonFX m_climbmotor1 = new TalonFX(0);
    TalonFX m_climbmotor2 = new TalonFX(89);
    DutyCycleOut speed = new DutyCycleOut(0);
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

    m_climbmotor1.set(1);
    m_climbmotor2.set(1);
  }
  public void climbDown() {
    m_climbmotor1.setControl(speed);
    m_climbmotor2.setControl(speed);

    m_climbmotor1.set(-1);
    m_climbmotor2.set(-1);
  }
  public Command climbUpCommand() {
    return runOnce(this::climbUp);
  }
   public Command climbDownCommand() {
    return runOnce(this::climbDown);
  }
}
