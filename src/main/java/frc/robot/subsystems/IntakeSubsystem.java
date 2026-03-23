// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX m_leftLinearScrew;
  TalonFX m_rightLinearScrew;
  TalonFX m_intakeRoller;
  TalonFX m_storageRoller;

  PositionVoltage p_PositionRequest = new PositionVoltage(0).withSlot(0);
  // PositionVoltage p_LimitVoltge = new PositionVoltage(0).withSlot(0);
  private DigitalInput i_limitSwitch = new DigitalInput(0);
  private final Timer simTimer = new Timer();

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_leftLinearScrew = new TalonFX(20);
    m_rightLinearScrew = new TalonFX(30);
    m_intakeRoller = new TalonFX(10);
    m_storageRoller = new TalonFX(40);

  //RPM
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE; // Tune this value (output per rotation of error)
    
    // Add kI, kD, kS, kV if needed for better control
    m_leftLinearScrew.getConfigurator().apply(slot0Configs);
    m_rightLinearScrew.getConfigurator().apply(slot0Configs);
    m_rightLinearScrew.setControl(new Follower(m_leftLinearScrew.getDeviceID(), MotorAlignmentValue.Aligned));
    final VelocityVoltage m_request = new VelocityVoltage(0);
    double targetRPM = 3000;
    double targetRPS = targetRPM / 60.0;

    m_storageRoller.setControl(m_request.withVelocity(targetRPS));

    if (RobotBase.isSimulation()) {
            simTimer.start();
    }
  }

//Limit Switch
  public boolean isLimitPressed() {
        // if (RobotBase.isSimulation()){
        //     return simTimer.hasElapsed(5.0);
        // }
        boolean x = i_limitSwitch.get(); // for debugging purposes
        return !i_limitSwitch.get(); 
        // If using NC wiring, invert it
    }

  //Timers
    public void resetTimers() {
        simTimer.start();
        simTimer.reset();
    }
    

  @Override
  public void periodic() {
    double lposition = m_leftLinearScrew.getPosition(true).getValueAsDouble();
    double rposition = m_rightLinearScrew.getPosition(true).getValueAsDouble();
    // System.out.println(lposition);
    // System.out.println(rposition);
    if(isLimitPressed()){
      littleExtenedIntake();
    }
    // This method will be called once per scheduler run
  }

  //MOTOR USES

//Intake Extension
  public void extendIntake() {
    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
    m_rightLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
  }
  public void retractIntake() {
      m_leftLinearScrew.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
      m_rightLinearScrew.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
  }
  public void littleExtenedIntake() {
    double position = m_leftLinearScrew.getPosition(true).getValueAsDouble();
    double targetPosition = position - LIMIT_SWITCH_ROTATIONS;
    System.out.println(position);
    System.out.println(targetPosition);
    System.out.println(IN_TAKE_TARGET_ROTATIONS);
    System.out.println(OUT_TAKE_TARGET_ROTATIONS);
    // IN_TAKE_TARGET_ROTATIONS = targetPosition;
    // OUT_TAKE_TARGET_ROTATIONS = targetPosition - 60;
    OUT_TAKE_TARGET_ROTATIONS = targetPosition - 60;
    IN_TAKE_TARGET_ROTATIONS = targetPosition;
    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(targetPosition));
    m_rightLinearScrew.setControl(p_PositionRequest.withPosition(targetPosition));
  }

//Normal Runs
  public void runIntake() { 
    m_intakeRoller.set(TAKE_SPEED);
  }
  public void runIntakeBack(){
    m_intakeRoller.set(-TAKE_SPEED);
  }
  public void runStorageRoller(){
    m_storageRoller.set(STORAGEROLLER_SPEED);
  }
  public void runStorageRollerBack(){
    m_storageRoller.set(-STORAGEROLLER_SPEED);
  }

//Stops
  public void stopStorage() {
    if(!isLimitPressed()){
        m_leftLinearScrew.set(0);
        m_rightLinearScrew.set(0);
    }
  }
  public void stopIntake() {
    m_intakeRoller.set(0);
    m_storageRoller.set(0);
  }

  //COMMANDS

//Extension Commands
  public Command extendIntakeCommand() {
    return run(this::extendIntake);
  }
  public Command retractIntakeCommand() {
    return run(this::retractIntake);
  }
  public Command littleExtendIntakeCommand() {
    return run(this::littleExtenedIntake);
  }

//Normal Run Commands
  public Command runIntakeCommand() {
    return run(this::runIntake);
  }
  public Command runIntakeBackCommand(){
    return run(this::runIntakeBack);
  }
  public Command runStorgeRollersCommand(){
    return run(this::runStorageRoller);
  }
  public Command runStorgeRollersBackCommand(){
    return run(this::runStorageRollerBack);
  }

//Stop Commands
  public Command stopStorageCommand() {
    return runOnce(this::stopStorage);
  }
  public Command stopIntakeCommand() {
    return runOnce(this::stopIntake);
  }
}
