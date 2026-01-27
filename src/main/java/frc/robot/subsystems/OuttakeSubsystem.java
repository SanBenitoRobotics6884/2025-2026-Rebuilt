// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
  TalonFX m_outtakeMotor1;
  TalonFX m_outtakeMotor2;
  DutyCycleOut speed;
  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    m_outtakeMotor1 = new TalonFX(0);
    m_outtakeMotor2 = new TalonFX(1);
    speed = new DutyCycleOut(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runOuttake() {
    m_outtakeMotor1.setControl(speed);
    m_outtakeMotor2.setControl(speed);
    m_outtakeMotor1.set(-1);
    m_outtakeMotor2.set(1);
  }

  public Command runOuttakecommand() {
    return runOnce(this::runOuttake);
  }
}
