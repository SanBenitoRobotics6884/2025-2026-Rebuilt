// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Constants.Outtake.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
  private final TalonFX alyssaMotor = new TalonFX(34);
  private final TalonFX nathanMotor = new TalonFX(62);
  private final TalonFX indexMotor = new TalonFX(50);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(1);
  private final double targetRps = 3000.0 / 60.0;

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    var slot0Configs = new Slot0Configs();
    var slot1Configs = new Slot0Configs();
    var slot2Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE;
    slot1Configs.kP = PID_P_VALUE;
    slot2Configs.kP = PID_P_VALUE;

    alyssaMotor.getConfigurator().apply(slot1Configs);
    nathanMotor.getConfigurator().apply(slot2Configs);
    indexMotor.getConfigurator().apply(slot0Configs);
  }

  public void runOuttake() {
    alyssaMotor.setControl(velocityRequest.withVelocity(targetRps));
    nathanMotor.setControl(new StrictFollower(34));
  }

  public void runIndex() {
    indexMotor.setControl(velocityRequest.withVelocity(targetRps));
  }

  public void reverseIndex() {
    indexMotor.setControl(velocityRequest.withVelocity(-targetRps));
  }

  public void runOuttakeSlow() {
    alyssaMotor.setControl(velocityRequest.withVelocity(targetRps * OUTTAKE_SPEED_SLOW));
    nathanMotor.setControl(new StrictFollower(34));
    indexMotor.setControl(velocityRequest.withVelocity(targetRps * INDEX_SPEED_SLOW));
  }

  public void stopOuttake() {
    alyssaMotor.set(0);
    nathanMotor.set(0);
  }

  public void stopIndex() {
    indexMotor.set(0);
  }

  public Command runOuttakecommand() {
    return runOnce(this::runOuttake);
  }

  public Command runOuttakeSlowCommand() {
    return runOnce(this::runOuttakeSlow);
  }

  public Command runIndexCommand() {
    return runOnce(this::runIndex);
  }

  public Command runReverseIndexCommand() {
    return runOnce(this::reverseIndex);
  }

  public Command runOuttaketelecommand() {
    return run(this::runOuttake);
  }

  public Command runOuttakeSlowteleCommand() {
    return run(this::runOuttakeSlow);
  }

  public Command runIndexteleCommand() {
    return run(this::runIndex);
  }

  public Command runReverseIndexteleCommand() {
    return run(this::reverseIndex);
  }

  public Command stopOuttakeCommand() {
    return runOnce(this::stopOuttake);
  }

  public Command stopIndexCommand() {
    return runOnce(this::stopIndex);
  }
}
