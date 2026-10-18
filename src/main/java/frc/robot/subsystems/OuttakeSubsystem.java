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
  private final TalonFX m_shooterLeader = new TalonFX(kShooterLeaderId);
  private final TalonFX m_shooterFollower = new TalonFX(kShooterFollowerId);
  private final TalonFX m_indexer = new TalonFX(kIndexerId);

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(1);
  // The feeder has its own request. Sharing one object with the shooter would let the feeder
  // speed overwrite the shooter speed if both are set in the same cycle.
  private final VelocityVoltage m_indexerRequest = new VelocityVoltage(1);

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    var shooterGains = new Slot0Configs();
    shooterGains.kP = kP;

    m_shooterLeader.getConfigurator().apply(shooterGains);
    m_shooterFollower.getConfigurator().apply(shooterGains);
    m_indexer.getConfigurator().apply(shooterGains);

    // Ask the leader for a fresh speed often enough that "are we at RPM yet?" is not a stale number.
    m_shooterLeader.getVelocity().setUpdateFrequency(50);
  }

  /**
   * Spin the shooter at an exact speed.
   *
   * <p>{@code rpm} is rotations per minute, which is the unit people tune shots in. The motor
   * controller wants rotations per second, so this divides by 60. The follower copies the leader
   * so both wheels match.
   */
  public void setShooterRpm(double rpm) {
    double rotationsPerSecond = rpm / 60.0;
    m_shooterLeader.setControl(m_velocityRequest.withVelocity(rotationsPerSecond));
    m_shooterFollower.setControl(new StrictFollower(kShooterLeaderId));
  }

  /** Measured shooter speed, in rotations per minute. */
  public double getShooterRpm() {
    return m_shooterLeader.getVelocity().getValueAsDouble() * 60.0;
  }

  /** Spin the shooter leader. The follower copies it. */
  public void shoot() {
    m_shooterLeader.setControl(m_velocityRequest.withVelocity(kShooterVelocityRps));
    m_shooterFollower.setControl(new StrictFollower(kShooterLeaderId));
  }

  /** Spin the feeder forward. This is what pushes a ball into the shooter. */
  public void index() {
    m_indexer.setControl(m_indexerRequest.withVelocity(kShooterVelocityRps));
  }

  /** Spin the feeder backward. */
  public void reverseIndex() {
    m_indexer.setControl(m_indexerRequest.withVelocity(-kShooterVelocityRps));
  }

  /** Spin the shooter at half speed and the feeder at a quarter speed. */
  public void shootSlow() {
    m_shooterLeader.setControl(
        m_velocityRequest.withVelocity(kShooterVelocityRps * kShooterSlowScale));
    m_shooterFollower.setControl(new StrictFollower(kShooterLeaderId));
    m_indexer.setControl(m_indexerRequest.withVelocity(kShooterVelocityRps * kIndexerSlowScale));
  }

  public void stopShooter() {
    m_shooterLeader.set(0);
    m_shooterFollower.set(0);
  }

  public void stopIndexer() {
    m_indexer.set(0);
  }

  /** Reapplies shooter velocity until the command is interrupted. */
  public Command shootCommand() {
    return run(this::shoot);
  }

  /** Reapplies the slow shot until the command is interrupted. */
  public Command shootSlowCommand() {
    return run(this::shootSlow);
  }

  /** Reapplies indexer velocity until the command is interrupted. */
  public Command indexCommand() {
    return run(this::index);
  }

  /** Reapplies reverse indexer velocity until the command is interrupted. */
  public Command reverseIndexCommand() {
    return run(this::reverseIndex);
  }

  /**
   * Sets shooter velocity once and ends, so an auto step can continue while the wheels stay spun
   * up.
   */
  public Command startShooterCommand() {
    return runOnce(this::shoot);
  }

  /**
   * Sets indexer velocity once and ends, so an auto step can continue while the feeder stays spun
   * up.
   */
  public Command startIndexerCommand() {
    return runOnce(this::index);
  }

  public Command stopShooterCommand() {
    return runOnce(this::stopShooter);
  }

  public Command stopIndexerCommand() {
    return runOnce(this::stopIndexer);
  }
}
