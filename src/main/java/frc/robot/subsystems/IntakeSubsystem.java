// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX m_leftLinearScrew;
  TalonFX m_rightLinearScrew;
  TalonFX m_intakeRoller;
  TalonFX m_storageRoller;

  PositionVoltage p_PositionRequest = new PositionVoltage(0).withSlot(0);

  DutyCycleOut speed;

 // DigitalInput i_limitSwitch = new DigitalInput(0); // DIO 0
  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_leftLinearScrew = new TalonFX(20);
    m_rightLinearScrew = new TalonFX(30);
    m_intakeRoller = new TalonFX(10);
    m_storageRoller = new TalonFX(40);

     var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE; // Tune this value (output per rotation of error)
    // Add kI, kD, kS, kV if needed for better control
    m_leftLinearScrew.getConfigurator().apply(slot0Configs);
    m_rightLinearScrew.getConfigurator().apply(slot0Configs);

    speed = new DutyCycleOut(DUTYCYCLE_OUTPUT);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void deoplyInTake(){
    m_leftLinearScrew.setControl(speed);
    m_rightLinearScrew.setControl(speed);

    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
    m_rightLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
  }
  public void runIntake() { 
    m_intakeRoller.setControl(speed);
    m_storageRoller.setControl(speed);

    m_intakeRoller.set(TAKE_SPEED);
    m_storageRoller.set(TAKE_SPEED);
  }
  public void undeoplyInTake(){
    m_leftLinearScrew.setControl(speed);
    m_rightLinearScrew.setControl(speed);

    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
    m_rightLinearScrew.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
  }
  public void runStorageRoller(){
    m_storageRoller.setControl(speed);

    m_storageRoller.set(TAKE_SPEED);
  }

  public void stopStorage() {
    m_leftLinearScrew.set(0);
    m_rightLinearScrew.set(0);
  }

  public void stopTake() {
    m_intakeRoller.set(0);
    m_storageRoller.set(0);
  }

 // public boolean isLimitPressed() {
   // return !i_limitSwitch.get();
 // }

  public Command deployIntakeCommand() {
    return run(this::deoplyInTake);
  }

  public Command runIntakeCommand() {
    return run(this::runIntake);
  }
public Command runStorgeRollersCommand(){
  return run(this::runStorageRoller);
}
  public Command undeployIntakeCommand() {
    return run(this::undeoplyInTake);
  }

  public Command stopStorageCommand() {
    return runOnce(this::stopStorage);
  }

  public Command stopTakeCommand() {
    return runOnce(this::stopTake);
  }
}
