// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX m_intakeMotor1;
  TalonFX m_intakeMotor2;
  TalonFX m_intakeMoter3;
  TalonFX m_intakeMoter4;

  PositionVoltage p_PositionRequest = new PositionVoltage(0).withSlot(0);

  DutyCycleOut speed;
  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_intakeMotor1 = new TalonFX(IN_OUT_TAKE_MOTOR1_ID);
    m_intakeMotor2 = new TalonFX(IN_OUT_TAKE_MOTOR2_ID);
    m_intakeMoter3 = new TalonFX(TAKE_MOTOR1_ID);
    m_intakeMoter4 = new TalonFX(TAKE_MOTOR2_ID);

     var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE; // Tune this value (output per rotation of error)
    // Add kI, kD, kS, kV if needed for better control
    m_intakeMotor1.getConfigurator().apply(slot0Configs);
    m_intakeMotor2.getConfigurator().apply(slot0Configs);

    speed = new DutyCycleOut(DUTYCYCLE_OUTPUT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void deoplyInTake(){
    m_intakeMotor1.setControl(speed);
    m_intakeMotor2.setControl(speed);

    m_intakeMotor1.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
    m_intakeMotor2.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
  }
  public void runIntake() { 
    m_intakeMoter3.setControl(speed);
    m_intakeMoter4.setControl(speed);

    m_intakeMoter3.set(TAKE_SPEED);
    m_intakeMoter4.set(TAKE_SPEED);
  }
  public void undeoplyInTake(){
    m_intakeMotor1.setControl(speed);
    m_intakeMotor2.setControl(speed);

    m_intakeMotor1.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
    m_intakeMotor2.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
  }

  public void stopInOutTake() {
    m_intakeMotor1.set(0);
    m_intakeMotor2.set(0);
  }

  public void stopTake() {
    m_intakeMoter3.set(0);
    m_intakeMoter4.set(0);
  }

  public Command deployIntakeCommand() {
    return run(this::deoplyInTake);
  }

  public Command runIntakeCommand() {
    return run(this::runIntake);
  }

  public Command undeployIntakeCommand() {
    return run(this::undeoplyInTake);
  }

  public Command stopInOutTakeCommand() {
    return runOnce(this::stopInOutTake);
  }

  public Command stopTakeCommand() {
    return runOnce(this::stopTake);
  }
}
