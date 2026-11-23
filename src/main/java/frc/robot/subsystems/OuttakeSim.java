// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation only: the two shooter wheels and the indexer, each a Kraken X60 spinning a wheel.
 *
 * <p>Each motor model is driven by the voltage its simulated Talon outputs, and its speed goes
 * back into the Talon, so {@link OuttakeSubsystem#getShooterRpm()} rises and falls the way the
 * real wheels would. There is no friction and no ball load, so the real wheels will be a little
 * slower.
 */
final class OuttakeSim {
  /** Rotating inertia of one shooter wheel as each motor sees it, in kg·m². A guess. */
  private static final double kShooterInertiaKgM2 = 0.002;

  /** Rotating inertia of the indexer, in kg·m². A guess. */
  private static final double kIndexerInertiaKgM2 = 0.0005;

  private final Wheel m_leader;
  private final Wheel m_follower;
  private final Wheel m_indexer;

  OuttakeSim(TalonFX leader, TalonFX follower, TalonFX indexer) {
    m_leader = new Wheel(leader, kShooterInertiaKgM2);
    m_follower = new Wheel(follower, kShooterInertiaKgM2);
    m_indexer = new Wheel(indexer, kIndexerInertiaKgM2);
  }

  void update(double dtSeconds) {
    double battery = RobotController.getBatteryVoltage();
    m_leader.update(dtSeconds, battery);
    m_follower.update(dtSeconds, battery);
    m_indexer.update(dtSeconds, battery);
  }

  /** One motor and the wheel it spins. */
  private static final class Wheel {
    private final TalonFXSimState m_talon;
    private final DCMotorSim m_motor;

    Wheel(TalonFX talon, double inertiaKgM2) {
      m_talon = talon.getSimState();
      m_talon.setMotorType(TalonFXSimState.MotorType.KrakenX60);
      DCMotor gearbox = DCMotor.getKrakenX60(1);
      m_motor =
          new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, inertiaKgM2, 1.0), gearbox);
    }

    void update(double dtSeconds, double battery) {
      m_talon.setSupplyVoltage(battery);
      m_motor.setInputVoltage(m_talon.getMotorVoltage());
      m_motor.update(dtSeconds);
      m_talon.setRawRotorPosition(m_motor.getAngularPositionRotations());
      m_talon.setRotorVelocity(m_motor.getAngularVelocityRPM() / 60.0);
    }
  }
}
