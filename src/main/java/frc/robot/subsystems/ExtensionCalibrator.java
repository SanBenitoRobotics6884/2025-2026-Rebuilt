// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Constants.Intake.kCalibrateTimeoutSeconds;
import static frc.robot.Constants.Constants.Intake.kHardStopBackoffRotations;
import static frc.robot.Constants.Constants.Intake.kInrushSeconds;
import static frc.robot.Constants.Constants.Intake.kInwardVoltage;
import static frc.robot.Constants.Constants.Intake.kMinTravelRotations;
import static frc.robot.Constants.Constants.Intake.kStallCurrentAmps;
import static frc.robot.Constants.Constants.Intake.kStallSeconds;
import static frc.robot.Constants.Constants.Intake.kStallVelocityRps;

/**
 * Hard-stop sweep for the intake screws. The encoder reads 0 at power-up no matter where the
 * extension actually is, so extend and retract stay put until both ends have been found.
 *
 * <p>Each {@link #step(Sample)} takes one reading. The clock is whatever the caller passes in, so
 * the robot can use FPGA time and a test can step in milliseconds. Voltage, the limit switch, and
 * the stall current are inputs. This class does not talk to a motor.
 */
public final class ExtensionCalibrator {
  public enum Phase {
    IDLE,
    SEEK_FIRST,
    SEEK_SECOND,
    DONE,
    FAILED
  }

  /** Thresholds for one sweep. {@link #intake()} is what the robot uses. */
  public static final class Config {
    public final double inwardVoltage;
    public final double stallCurrentAmps;
    public final double stallVelocityRps;
    public final double inrushSeconds;
    public final double stallSeconds;
    public final double timeoutSeconds;
    public final double minTravelRotations;
    public final double backoffRotations;

    public Config(
        double inwardVoltage,
        double stallCurrentAmps,
        double stallVelocityRps,
        double inrushSeconds,
        double stallSeconds,
        double timeoutSeconds,
        double minTravelRotations,
        double backoffRotations) {
      this.inwardVoltage = inwardVoltage;
      this.stallCurrentAmps = stallCurrentAmps;
      this.stallVelocityRps = stallVelocityRps;
      this.inrushSeconds = inrushSeconds;
      this.stallSeconds = stallSeconds;
      this.timeoutSeconds = timeoutSeconds;
      this.minTravelRotations = minTravelRotations;
      this.backoffRotations = backoffRotations;
    }

    public static Config intake() {
      return new Config(
          kInwardVoltage,
          kStallCurrentAmps,
          kStallVelocityRps,
          kInrushSeconds,
          kStallSeconds,
          kCalibrateTimeoutSeconds,
          kMinTravelRotations,
          kHardStopBackoffRotations);
    }
  }

  /** One cycle of screw feedback. Velocity is rotations per second and may be signed. */
  public static final class Sample {
    public final double positionRotations;
    public final double velocityRps;
    public final double currentAmps;
    public final boolean limitPressed;
    public final double timestampSeconds;

    public Sample(
        double positionRotations,
        double velocityRps,
        double currentAmps,
        boolean limitPressed,
        double timestampSeconds) {
      this.positionRotations = positionRotations;
      this.velocityRps = velocityRps;
      this.currentAmps = currentAmps;
      this.limitPressed = limitPressed;
      this.timestampSeconds = timestampSeconds;
    }
  }

  /** What the screws should do with the sample that was just taken. */
  public static final class Action {
    public final boolean applyVoltage;
    public final double voltage;
    public final boolean zeroEncoder;
    public final double encoderLeftRotations;
    public final double encoderRightRotations;
    public final boolean holdRetract;
    public final boolean stopScrews;
    public final String error;
    public final String warning;

    private Action(
        boolean applyVoltage,
        double voltage,
        boolean zeroEncoder,
        double encoderLeftRotations,
        double encoderRightRotations,
        boolean holdRetract,
        boolean stopScrews,
        String error,
        String warning) {
      this.applyVoltage = applyVoltage;
      this.voltage = voltage;
      this.zeroEncoder = zeroEncoder;
      this.encoderLeftRotations = encoderLeftRotations;
      this.encoderRightRotations = encoderRightRotations;
      this.holdRetract = holdRetract;
      this.stopScrews = stopScrews;
      this.error = error;
      this.warning = warning;
    }

    static Action none() {
      return new Action(false, 0.0, false, 0.0, 0.0, false, false, null, null);
    }

    static Action seeking(double voltage) {
      return new Action(true, voltage, false, 0.0, 0.0, false, false, null, null);
    }

    static Action failed(String error, boolean applyVoltage, double voltage, String warning) {
      return new Action(applyVoltage, voltage, false, 0.0, 0.0, false, true, error, warning);
    }

    static Action finished(
        double voltage, double leftRotations, double rightRotations, String warning) {
      return new Action(
          true, voltage, true, leftRotations, rightRotations, true, false, null, warning);
    }
  }

  public static final String SWITCH_WARNING =
      "Intake limit switch did not identify the fully-in stop. Using kInwardVoltage direction.";
  public static final String TIMEOUT_ERROR = "Intake calibration timed out before a hard stop.";
  public static final String TRAVEL_ERROR =
      "Intake hard stops were too close together to calibrate.";

  private final Config m_config;

  private Phase m_phase = Phase.IDLE;
  private boolean m_calibrated;
  private boolean m_failed;
  private double m_appliedVoltage;
  private double m_firstPositionRotations;
  private boolean m_firstHitSwitch;
  /** Second seek must leave the first stop before another spike can count. */
  private boolean m_requireMotion;
  private boolean m_seenMotion;
  private double m_phaseStartSeconds;
  private boolean m_stallTiming;
  private double m_stallStartSeconds;
  private double m_extendPositionRotations;
  private double m_retractPositionRotations;

  public ExtensionCalibrator(Config config) {
    m_config = config;
  }

  public Config config() {
    return m_config;
  }

  public Phase phase() {
    return m_phase;
  }

  public boolean isCalibrated() {
    return m_calibrated;
  }

  public boolean isFailed() {
    return m_failed;
  }

  public double extendPositionRotations() {
    return m_extendPositionRotations;
  }

  public double retractPositionRotations() {
    return m_retractPositionRotations;
  }

  /**
   * Marks the sweep finished without moving. Simulation has no stator current, so the robot uses
   * this instead of grinding until the timeout.
   */
  public void assumeCalibrated(double retractPositionRotations, double extendPositionRotations) {
    m_retractPositionRotations = retractPositionRotations;
    m_extendPositionRotations = extendPositionRotations;
    m_calibrated = true;
    m_failed = false;
    m_phase = Phase.DONE;
  }

  /**
   * Starts a sweep at {@code nowSeconds}. Returns false when a finished sweep is left alone
   * ({@code force} is false).
   */
  public boolean start(boolean force, double nowSeconds) {
    if (m_calibrated && !force) {
      return false;
    }
    m_calibrated = false;
    m_failed = false;
    m_phase = Phase.SEEK_FIRST;
    m_appliedVoltage = m_config.inwardVoltage;
    m_firstHitSwitch = false;
    m_requireMotion = false;
    m_seenMotion = false;
    m_phaseStartSeconds = nowSeconds;
    m_stallTiming = false;
    return true;
  }

  /** Drops an in-progress sweep when its command is cancelled. A finished sweep is left alone. */
  public void onCommandEnd(boolean interrupted) {
    if (m_calibrated) {
      return;
    }
    if (interrupted) {
      m_phase = Phase.IDLE;
      m_failed = false;
    }
  }

  public Action step(Sample sample) {
    if (m_calibrated || m_failed || m_phase == Phase.IDLE || m_phase == Phase.DONE) {
      return Action.none();
    }
    double phaseElapsed = sample.timestampSeconds - m_phaseStartSeconds;
    if (phaseElapsed >= m_config.timeoutSeconds) {
      return fail(TIMEOUT_ERROR, false, 0.0, null);
    }

    // The voltage captured here is the one for this cycle. Reversing happens on the next step.
    double voltage = m_appliedVoltage;
    if (phaseElapsed < m_config.inrushSeconds) {
      m_stallTiming = false;
      return Action.seeking(voltage);
    }
    double velocityRps = Math.abs(sample.velocityRps);
    if (velocityRps > m_config.stallVelocityRps) {
      m_seenMotion = true;
    }
    if (m_requireMotion && !m_seenMotion) {
      return Action.seeking(voltage);
    }
    boolean stalled =
        velocityRps < m_config.stallVelocityRps && sample.currentAmps > m_config.stallCurrentAmps;
    if (!stalled) {
      m_stallTiming = false;
      return Action.seeking(voltage);
    }
    if (!m_stallTiming) {
      m_stallTiming = true;
      m_stallStartSeconds = sample.timestampSeconds;
    }
    if (sample.timestampSeconds - m_stallStartSeconds < m_config.stallSeconds) {
      return Action.seeking(voltage);
    }
    return capture(sample, voltage);
  }

  private Action capture(Sample sample, double voltage) {
    if (m_phase == Phase.SEEK_FIRST) {
      m_firstPositionRotations = sample.positionRotations;
      m_firstHitSwitch = sample.limitPressed;
      m_phase = Phase.SEEK_SECOND;
      m_appliedVoltage = -m_config.inwardVoltage;
      m_requireMotion = true;
      m_seenMotion = false;
      m_phaseStartSeconds = sample.timestampSeconds;
      m_stallTiming = false;
      return Action.seeking(voltage);
    }
    return finish(sample, voltage);
  }

  private Action finish(Sample sample, double voltage) {
    boolean firstIsIn;
    String warning = null;
    if (m_firstHitSwitch == sample.limitPressed) {
      firstIsIn = true;
      warning = SWITCH_WARNING;
    } else {
      firstIsIn = m_firstHitSwitch;
    }

    double inPosition = firstIsIn ? m_firstPositionRotations : sample.positionRotations;
    double outPosition = firstIsIn ? sample.positionRotations : m_firstPositionRotations;
    double travel = outPosition - inPosition;
    if (Math.abs(travel) < m_config.minTravelRotations) {
      return fail(TRAVEL_ERROR, true, voltage, warning);
    }

    double shifted = sample.positionRotations - inPosition;
    double backoff = Math.copySign(m_config.backoffRotations, travel);
    m_retractPositionRotations = backoff;
    m_extendPositionRotations = travel - backoff;
    m_calibrated = true;
    m_failed = false;
    m_phase = Phase.DONE;
    return Action.finished(voltage, shifted, -shifted, warning);
  }

  private Action fail(String error, boolean applyVoltage, double voltage, String warning) {
    m_failed = true;
    m_calibrated = false;
    m_phase = Phase.FAILED;
    return Action.failed(error, applyVoltage, voltage, warning);
  }
}
