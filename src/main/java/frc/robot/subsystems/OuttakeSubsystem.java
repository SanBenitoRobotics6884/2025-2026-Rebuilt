// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Constants.Outtake.*;

public class OuttakeSubsystem extends SubsystemBase {
  TalonFX m_outtakeMotor1;
  TalonFX m_outtakeMotor2;
  DutyCycleOut speed;
  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    m_outtakeMotor1 = new TalonFX(60);
    m_outtakeMotor2 = new TalonFX(50);
    speed = new DutyCycleOut(DUTYCYCLE_OUTPUT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runOuttake() {
    m_outtakeMotor1.setControl(speed);
    m_outtakeMotor2.setControl(speed);
    m_outtakeMotor1.set(-OUTTAKE_SPEED);
    m_outtakeMotor2.set(OUTTAKE_SPEED);
  }

  public void stopOuttake() {
    m_outtakeMotor1.set(0);
    m_outtakeMotor2.set(0);
  }

  public Command runOuttakecommand() {
    return run(this::runOuttake);
  }

  public Command stopOuttakeCommand() {
    return runOnce(this::stopOuttake);
  }
}
