// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Constants.Intake.*;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import com.ctre.phoenix6.sim.CANdiSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Simulation only: two Kraken X60 lead screws and the CANdi limit sensors.
 *
 * <p>Each screw is a WPILib motor model driven by the voltage its simulated Talon outputs. Its
 * position goes back into the Talon, so {@link IntakeSubsystem} sees the screws really turn. The
 * intake's position is where it was at power-up plus how far the screws have turned. S1 closes at
 * 0 (retracted) and S2 at {@code kTravelRotations} (extended). Positions are motor rotations,
 * positive = out, same as the robot code.
 *
 * <p>Sim-only knobs:
 *
 * <ul>
 *   <li>{@code INTAKE_SIM_START_ROT} environment variable: where the intake is at power-up, in
 *       rotations. Default 0 (retracted, so it boots homed). Try 40 to practice the walk-in.
 *   <li>{@code IntakeSim/JamRight} on the dashboard: stops the right screw to watch a fault.
 * </ul>
 */
final class IntakeSim {
  /**
   * Rotating inertia each motor sees, in kg·m². A guess: rotor plus lead screw plus a share of the
   * intake. Bigger only makes the screws slower to get up to speed.
   */
  private static final double kInertiaKgM2 = 0.0005;

  /** The hard stops sit this far past each sensor, in rotations. */
  private static final double kHardStopPastSensorRot = 2.0;

  private final Screw m_left;
  private final Screw m_right;
  private final CANdiSimState m_candi;

  IntakeSim(
      TalonFX left,
      TalonFX right,
      CANdi candi,
      InvertedValue leftInverted,
      InvertedValue rightInverted,
      double startRotations) {
    m_left = new Screw(left, leftInverted, startRotations);
    m_right = new Screw(right, rightInverted, startRotations);
    m_candi = candi.getSimState();
    SmartDashboard.putBoolean("IntakeSim/JamRight", false);
    pushSensors();
  }

  /** Power-up position from {@code INTAKE_SIM_START_ROT}, or 0 (retracted). */
  static double startRotationsFromEnvironment() {
    String value = System.getenv("INTAKE_SIM_START_ROT");
    if (value == null || value.isBlank()) {
      return 0.0;
    }
    try {
      return Double.parseDouble(value.trim());
    } catch (NumberFormatException e) {
      return 0.0;
    }
  }

  /** Advance the physics by {@code dtSeconds}. */
  void update(double dtSeconds) {
    double battery = RobotController.getBatteryVoltage();
    m_left.update(dtSeconds, battery, false);
    m_right.update(dtSeconds, battery, SmartDashboard.getBoolean("IntakeSim/JamRight", false));
    pushSensors();
  }

  /** Teleport the intake to {@code rotations} without touching the encoders. For tests. */
  void setPosition(double rotations) {
    m_left.setPhysical(rotations);
    m_right.setPhysical(rotations);
    pushSensors();
  }

  /** Where the intake really is, in rotations from fully retracted. */
  double position() {
    return (m_left.physical() + m_right.physical()) / 2.0;
  }

  private void pushSensors() {
    double position = position();
    // CANdi default config: a sensor is "closed" when its input is not floating.
    m_candi.setS1State(position <= 0.0 ? S1StateValue.Low : S1StateValue.Floating);
    m_candi.setS2State(position >= kTravelRotations ? S2StateValue.Low : S2StateValue.Floating);
  }

  /** One screw: motor model, its Talon's sim state, and where it physically is. */
  private static final class Screw {
    private final TalonFXSimState m_talon;
    private final DCMotorSim m_motor;
    /** Physical position = motor model position + this. The motor model starts at 0. */
    private double m_offset;

    Screw(TalonFX talon, InvertedValue inverted, double startRotations) {
      m_talon = talon.getSimState();
      m_talon.setMotorType(TalonFXSimState.MotorType.KrakenX60);
      // Match the Talon's inversion, so "positive = out" in the robot code is positive here too.
      m_talon.Orientation =
          inverted == InvertedValue.Clockwise_Positive
              ? ChassisReference.Clockwise_Positive
              : ChassisReference.CounterClockwise_Positive;
      DCMotor gearbox = DCMotor.getKrakenX60(1);
      m_motor =
          new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, kInertiaKgM2, 1.0), gearbox);
      m_offset = startRotations;
    }

    void update(double dtSeconds, double battery, boolean jammed) {
      m_talon.setSupplyVoltage(battery);
      if (jammed) {
        m_motor.setState(m_motor.getAngularPositionRad(), 0.0);
      } else {
        m_motor.setInputVoltage(m_talon.getMotorVoltage());
        m_motor.update(dtSeconds);
      }

      double min = -kHardStopPastSensorRot;
      double max = kTravelRotations + kHardStopPastSensorRot;
      double physical = physical();
      if (physical < min || physical > max) {
        double clamped = Math.max(min, Math.min(max, physical));
        m_motor.setState((clamped - m_offset) * 2.0 * Math.PI, 0.0);
      }

      m_talon.setRawRotorPosition(m_motor.getAngularPositionRotations());
      m_talon.setRotorVelocity(m_motor.getAngularVelocityRPM() / 60.0);
    }

    double physical() {
      return m_motor.getAngularPositionRotations() + m_offset;
    }

    void setPhysical(double rotations) {
      // Shift the offset, not the motor, so the Talon's encoder does not jump.
      m_offset = rotations - m_motor.getAngularPositionRotations();
    }
  }
}
