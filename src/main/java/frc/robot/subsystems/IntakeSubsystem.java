// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Constants.Intake.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_leftScrew = new TalonFX(kLeftScrewId);
  private final TalonFX m_rightScrew = new TalonFX(kRightScrewId);
  private final TalonFX m_intakeRoller = new TalonFX(kIntakeRollerId);
  private final TalonFX m_storageRoller = new TalonFX(kStorageRollerId);

  private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);
  private final DigitalInput m_limitSwitch = new DigitalInput(kLimitSwitchDio);
  private final Timer m_simTimer = new Timer();

  /** Updated by {@link #home()} when the limit switch trips. */
  private double m_extendRotations = kInitialExtendRotations;
  private double m_retractRotations = kInitialRetractRotations;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    var gains = new Slot0Configs();
    gains.kP = kP;

    m_leftScrew.getConfigurator().apply(gains);
    m_rightScrew.getConfigurator().apply(gains);
    m_rightScrew.setControl(
        new Follower(m_leftScrew.getDeviceID(), MotorAlignmentValue.Aligned));

    if (RobotBase.isSimulation()) {
      m_simTimer.start();
    }
  }

  public boolean isLimitPressed() {
    // Invert when the switch is wired normally closed.
    return !m_limitSwitch.get();
  }

  public void resetTimers() {
    m_simTimer.start();
    m_simTimer.reset();
  }

  @Override
  public void periodic() {
    if (isLimitPressed()) {
      home();
    }
  }

  /** Move the screws to the deployed intake position. */
  public void extend() {
    m_leftScrew.setControl(m_positionRequest.withPosition(-m_extendRotations));
    m_rightScrew.setControl(m_positionRequest.withPosition(m_extendRotations));
  }

  /** Move the screws to the stowed position. */
  public void retract() {
    m_leftScrew.setControl(m_positionRequest.withPosition(-m_retractRotations));
    m_rightScrew.setControl(m_positionRequest.withPosition(m_retractRotations));
  }

  /** Back off the limit switch and record the extend and retract setpoints from that position. */
  public void home() {
    double position = m_leftScrew.getPosition(true).getValueAsDouble();
    double targetPosition = position - kLimitSwitchBackoffRotations;
    m_retractRotations = targetPosition - kHomingRetractOffsetRotations;
    m_extendRotations = targetPosition;
    m_leftScrew.setControl(m_positionRequest.withPosition(targetPosition));
    m_rightScrew.setControl(m_positionRequest.withPosition(targetPosition));
  }

  /** Drive both screws to the same side of the extend setpoint. */
  public void resetScrews() {
    m_leftScrew.setControl(m_positionRequest.withPosition(-m_extendRotations));
    m_rightScrew.setControl(m_positionRequest.withPosition(-m_extendRotations));
  }

  public void intake() {
    m_intakeRoller.set(kIntakeSpeed);
  }

  public void reverseIntake() {
    m_intakeRoller.set(-kIntakeSpeed);
  }

  public void runStorage() {
    m_storageRoller.set(kStorageSpeed);
  }

  public void reverseStorage() {
    m_storageRoller.set(-kStorageSpeed);
  }

  /** Stop the linear screws, unless homing currently has the limit switch pressed. */
  public void stopScrews() {
    if (!isLimitPressed()) {
      m_leftScrew.set(0);
      m_rightScrew.set(0);
    }
  }

  /** Stop the intake roller and the storage roller. */
  public void stopRollers() {
    m_intakeRoller.set(0);
    m_storageRoller.set(0);
  }

  /** Stop the storage roller. */
  public void stopStorage() {
    m_storageRoller.set(0);
  }

  public Command extendCommand() {
    return run(this::extend);
  }

  public Command retractCommand() {
    return run(this::retract);
  }

  public Command homeCommand() {
    return run(this::home);
  }

  public Command resetScrewsCommand() {
    return run(this::resetScrews);
  }

  public Command intakeCommand() {
    return run(this::intake);
  }

  public Command reverseIntakeCommand() {
    return run(this::reverseIntake);
  }

  public Command runStorageCommand() {
    return run(this::runStorage);
  }

  public Command reverseStorageCommand() {
    return run(this::reverseStorage);
  }

  public Command stopScrewsCommand() {
    return runOnce(this::stopScrews);
  }

  public Command stopRollersCommand() {
    return runOnce(this::stopRollers);
  }

  public Command stopStorageCommand() {
    return runOnce(this::stopStorage);
  }
}
