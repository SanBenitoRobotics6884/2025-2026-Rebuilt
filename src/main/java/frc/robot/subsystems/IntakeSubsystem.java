// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX m_intakeMotor1;
  TalonFX m_intakeMotor2;
  TalonFX m_intakeMoter3;
  TalonFX m_intakeMoter4;
  DutyCycleOut speed;
  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_intakeMotor1 = new TalonFX(0);
    m_intakeMotor2 = new TalonFX(1);
    m_intakeMoter3 = new TalonFX(2);
    m_intakeMoter4 = new TalonFX(3);
    speed = new DutyCycleOut(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void deoplyInTake(){
    m_intakeMotor1.setControl(speed);
    m_intakeMotor2.setControl(speed);
    m_intakeMotor1.set(1);
    m_intakeMotor2.set(1);
  }
  public void runIntake() { 
    m_intakeMoter3.setControl(speed);
    m_intakeMoter4.setControl(speed);
    m_intakeMoter3.set(1);
    m_intakeMoter4.set(1);
  }
  public void undeoplyInTake(){
    m_intakeMotor1.setControl(speed);
    m_intakeMotor2.setControl(speed);
    m_intakeMotor1.set(-1);
    m_intakeMotor2.set(-1);
  }

  public Command deployIntakeCommand() {
    return runOnce(this::deoplyInTake);
  }

  public Command runIntakeCommand() {
    return runOnce(this::runIntake);
  }

  public Command undeployIntakeCommand() {
    return runOnce(this::undeoplyInTake);
  }
}
