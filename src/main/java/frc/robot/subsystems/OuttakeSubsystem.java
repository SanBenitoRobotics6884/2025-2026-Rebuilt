// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Constants.Outtake.*;

public class OuttakeSubsystem extends SubsystemBase {
  TalonFX m_outtakeMotor1;
  TalonFX m_outtakeMotor2;
  DutyCycleOut speed;

  final VelocityVoltage m_request = new VelocityVoltage(0);
  double targetRPM = 4000;
  double targetRPS = targetRPM / 60.0;
  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    m_outtakeMotor1 = new TalonFX(60);
    m_outtakeMotor2 = new TalonFX(50);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE; // Tune this value (output per rotation of error)
    // slot0Configs.kI = PID_I_VALUE;
    // slot0Configs.kD = PID_D_VALUE;
    // Add kI, kD, kS, kV if needed for better control
    m_outtakeMotor1.getConfigurator().apply(slot0Configs);
    m_outtakeMotor2.getConfigurator().apply(slot0Configs);
 
    // speed = new DutyCycleOut(DUTYCYCLE_OUTPUT); //origianly no subtraction to speed

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runOuttake() {
    //m_outtakeMotor1.setControl(speed);
    m_outtakeMotor1.setControl(m_request.withVelocity(-targetRPS));
   
  }
  public void runindex(){
    m_outtakeMotor2.setControl(m_request.withVelocity(targetRPS));
  }

  public void runOuttakeSlow() {
    m_outtakeMotor1.setControl(m_request.withVelocity(-targetRPS * OUTTAKE_SPEED_SLOW));
    m_outtakeMotor2.setControl(m_request.withVelocity(targetRPS * INDEX_SPEED_SLOW));
  }
  public void runIndexback() {
    m_outtakeMotor2.setControl(m_request.withVelocity(-targetRPS));
  }

  public void stopOuttake() {
    m_outtakeMotor1.set(0);
  }
  public void stopindex(){
     m_outtakeMotor2.set(0);
  }

  public Command runOuttakecommand() {
    return run(this::runOuttake);
  }
  public Command runIndexCommand(){
    return run(this::runindex);
  }
  public Command runOuttakeSlowCommand() {
    return run(this::runOuttakeSlow);
  }
  public Command runIndexBackCommand() {
    return run(this::runIndexback);
  }

  public Command stopOuttakeCommand() {
    return runOnce(this::stopOuttake);
  }
  public Command stopIndexCommand() {
    return runOnce(this::stopindex);
  }
}
