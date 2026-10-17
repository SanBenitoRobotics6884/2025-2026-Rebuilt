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
  private final TalonFX leftLinearScrew = new TalonFX(20);
  private final TalonFX rightLinearScrew = new TalonFX(30);
  private final TalonFX intakeRoller = new TalonFX(10);
  private final TalonFX storageRoller = new TalonFX(40);

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_STORAGE_DIO);
  private final Timer simTimer = new Timer();

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE;

    leftLinearScrew.getConfigurator().apply(slot0Configs);
    rightLinearScrew.getConfigurator().apply(slot0Configs);
    rightLinearScrew.setControl(
        new Follower(leftLinearScrew.getDeviceID(), MotorAlignmentValue.Aligned));

    if (RobotBase.isSimulation()) {
      simTimer.start();
    }
  }

  public boolean isLimitPressed() {
    // Invert when the switch is wired normally closed.
    return !limitSwitch.get();
  }

  public void resetTimers() {
    simTimer.start();
    simTimer.reset();
  }

  @Override
  public void periodic() {
    if (isLimitPressed()) {
      littleExtenedIntake();
    }
  }

  public void extendIntake() {
    leftLinearScrew.setControl(positionRequest.withPosition(-IN_TAKE_TARGET_ROTATIONS));
    rightLinearScrew.setControl(positionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
  }

  public void retractIntake() {
    leftLinearScrew.setControl(positionRequest.withPosition(-OUT_TAKE_TARGET_ROTATIONS));
    rightLinearScrew.setControl(positionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
  }

  public void littleExtenedIntake() {
    double position = leftLinearScrew.getPosition(true).getValueAsDouble();
    double targetPosition = position - LIMIT_SWITCH_ROTATIONS;
    OUT_TAKE_TARGET_ROTATIONS = targetPosition - 60;
    IN_TAKE_TARGET_ROTATIONS = targetPosition;
    leftLinearScrew.setControl(positionRequest.withPosition(targetPosition));
    rightLinearScrew.setControl(positionRequest.withPosition(targetPosition));
  }

  public void extendReset() {
    leftLinearScrew.setControl(positionRequest.withPosition(-IN_TAKE_TARGET_ROTATIONS));
    rightLinearScrew.setControl(positionRequest.withPosition(-IN_TAKE_TARGET_ROTATIONS));
  }

  public void runIntake() {
    intakeRoller.set(-0.4);
  }

  public void runIntakeBack() {
    intakeRoller.set(0.4);
  }

  public void runStorageRoller() {
    storageRoller.set(0.4);
  }

  public void runStorageRollerBack() {
    storageRoller.set(0.4);
  }

  public void stopStorage() {
    if (!isLimitPressed()) {
      leftLinearScrew.set(0);
      rightLinearScrew.set(0);
    }
  }

  public void stopIntake() {
    intakeRoller.set(0);
    storageRoller.set(0);
  }

  public void stopRollers() {
    storageRoller.set(0);
  }

  public Command extendIntakeCommand() {
    return runOnce(this::retractIntake);
  }

  public Command retractIntakeCommand() {
    return runOnce(this::retractIntake);
  }

  public Command littleExtendIntakeCommand() {
    return runOnce(this::littleExtenedIntake);
  }

  public Command extendresetCommand() {
    return run(this::extendReset);
  }

  public Command extendIntakeTeleCommand() {
    return run(this::extendIntake);
  }

  public Command retractIntakeTeleCommand() {
    return run(this::retractIntake);
  }

  public Command littleExtendIntakeTeleCommand() {
    return run(this::littleExtenedIntake);
  }

  public Command extendresetTeleCommand() {
    return run(this::extendReset);
  }

  public Command runIntakeCommand() {
    return runOnce(this::runIntake);
  }

  public Command runIntakeBackCommand() {
    return runOnce(this::runIntakeBack);
  }

  public Command runStorgeRollersCommand() {
    return runOnce(this::runStorageRoller);
  }

  public Command runStorgeRollersBackCommand() {
    return runOnce(this::runStorageRollerBack);
  }

  public Command runIntaketeleCommand() {
    return run(this::runIntake);
  }

  public Command runIntakeBackteleCommand() {
    return run(this::runIntakeBack);
  }

  public Command runStorgeRollersteleCommand() {
    return run(this::runStorageRoller);
  }

  public Command runStorgeRollersBackteleCommand() {
    return run(this::runStorageRollerBack);
  }

  public Command stopStorageCommand() {
    return runOnce(this::stopStorage);
  }

  public Command stopIntakeCommand() {
    return runOnce(this::stopIntake);
  }

  public Command stopRollerCommand() {
    return runOnce(this::stopRollers);
  }
}
