// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI.SimDeviceInfo;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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

  private DigitalInput i_limitSwitch = new DigitalInput(15);
  private final Timer simTimer = new Timer();

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

    if (RobotBase.isSimulation()) {
            simTimer.start();
    }

   // i_limitSwitch = new DigitalInput(0); // DIO 0
  }

  public boolean isLimitPressed() {
        if (RobotBase.isSimulation()){
            return simTimer.hasElapsed(5.0);
        }
        return !i_limitSwitch.get(); 
        // If using NC wiring, invert it
    }

    public void resetTimers() {
        simTimer.start();
        simTimer.reset();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void deployIntake(){
    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
    m_rightLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
  }
  public void runIntake() { 

    m_intakeRoller.set(TAKE_SPEED);
    m_storageRoller.set(STORAGEROLLER_SPEED);
  }
  public void undeployIntake(){
    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
    m_rightLinearScrew.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
  }
  public void runStorageRoller(){
    m_storageRoller.set(STORAGEROLLER_SPEED);
  }
  public void runStorageRollerBack(){
    m_storageRoller.set(-STORAGEROLLER_SPEED);
  }
  public void stopStorage() {
    m_leftLinearScrew.set(0);
    m_rightLinearScrew.set(0);
  }

  public void stopTake() {
    m_intakeRoller.set(0);
    m_storageRoller.set(0);
  }

//  public boolean isLimitPressed() {
//    return !i_limitSwitch.get();
//  }

  public Command deployIntakeCommand() {
    return run(this::deployIntake);
  }

  public Command runIntakeCommand() {
    return run(this::runIntake);
  }
public Command runStorgeRollersCommand(){
  return run(this::runStorageRoller);
}
public Command runStorgeRollersBackCommand(){
  return run(this::runStorageRollerBack);
}
  public Command undeployIntakeCommand() {
    return run(this::undeployIntake);
  }

  public Command stopStorageCommand() {
    return runOnce(this::stopStorage);
  }

  public Command stopTakeCommand() {
    return runOnce(this::stopTake);
  }
}
