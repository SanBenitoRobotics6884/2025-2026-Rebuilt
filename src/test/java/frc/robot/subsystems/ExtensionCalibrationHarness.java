// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Feeds scripted screw readings into an {@link ExtensionCalibrator}.
 *
 * <p>The TalonFX sim state can set position and velocity, but it cannot set stator current, and a
 * current spike is what makes a hard stop. This harness is that missing plant: hold a coast, then
 * hold a stall, on a 20 ms clock. Time is kept in whole milliseconds so the stall and inrush
 * windows land on the same instants as the second literals in {@link ExtensionCalibrator.Config}.
 */
public final class ExtensionCalibrationHarness {
  public static final int kDtMs = 20;

  /** Encoder reading and limit-switch state at one end of the screws. */
  public static final class HardStop {
    public final double positionRotations;
    public final boolean limitPressed;

    public HardStop(double positionRotations, boolean limitPressed) {
      this.positionRotations = positionRotations;
      this.limitPressed = limitPressed;
    }
  }

  private final ExtensionCalibrator m_calibrator;
  private final List<String> m_trace = new ArrayList<>();
  private int m_elapsedMs;
  private boolean m_zeroed;
  private double m_zeroLeftRotations;
  private double m_zeroRightRotations;
  private double m_lastVoltage;
  private String m_error;
  private String m_warning;

  public ExtensionCalibrationHarness(ExtensionCalibrator calibrator) {
    m_calibrator = calibrator;
  }

  public static ExtensionCalibrationHarness create(ExtensionCalibrator.Config config) {
    return new ExtensionCalibrationHarness(new ExtensionCalibrator(config));
  }

  public ExtensionCalibrator calibrator() {
    return m_calibrator;
  }

  public void start(boolean force) {
    m_elapsedMs = 0;
    m_zeroed = false;
    m_error = null;
    m_warning = null;
    m_trace.clear();
    m_calibrator.start(force, seconds());
  }

  public boolean zeroed() {
    return m_zeroed;
  }

  public double zeroLeftRotations() {
    return m_zeroLeftRotations;
  }

  public double zeroRightRotations() {
    return m_zeroRightRotations;
  }

  public double lastVoltage() {
    return m_lastVoltage;
  }

  public String error() {
    return m_error;
  }

  public String warning() {
    return m_warning;
  }

  public String trace() {
    return String.join("\n", m_trace);
  }

  /** Holds one reading for {@code seconds}. Samples continue after the phase changes. */
  public void hold(
      double seconds,
      double positionRotations,
      double velocityRps,
      double currentAmps,
      boolean limitPressed) {
    advance(seconds, false, positionRotations, velocityRps, currentAmps, limitPressed);
  }

  /**
   * Holds one reading until the phase changes or {@code seconds} runs out. A stall segment uses
   * this so samples after the spike do not eat the next seek's inrush window.
   */
  public void holdUntilPhaseChange(
      double seconds,
      double positionRotations,
      double velocityRps,
      double currentAmps,
      boolean limitPressed) {
    advance(seconds, true, positionRotations, velocityRps, currentAmps, limitPressed);
  }

  /**
   * One direction of the sweep. Coasts through the inrush window, coasts again so the second stop
   * is allowed to count, then stalls on {@code stop} long enough to trip.
   */
  public void seek(HardStop stop) {
    ExtensionCalibrator.Config config = m_calibrator.config();
    double cruiseRps = config.stallVelocityRps + 2.0;
    double movingAmps = Math.max(0.0, config.stallCurrentAmps - 1.0);
    double stallAmps = config.stallCurrentAmps + 10.0;
    double dtSeconds = kDtMs / 1000.0;
    hold(
        Math.max(0.0, config.inrushSeconds - dtSeconds),
        stop.positionRotations,
        cruiseRps,
        movingAmps,
        false);
    hold(dtSeconds * 3.0, stop.positionRotations, cruiseRps, movingAmps, false);
    holdUntilPhaseChange(
        config.stallSeconds + dtSeconds * 3.0,
        stop.positionRotations,
        0.0,
        stallAmps,
        stop.limitPressed);
  }

  /** Starts a forced sweep and drives into {@code first}, then back into {@code second}. */
  public void sweep(HardStop first, HardStop second) {
    start(true);
    seek(first);
    seek(second);
  }

  private void advance(
      double seconds,
      boolean untilPhaseChange,
      double positionRotations,
      double velocityRps,
      double currentAmps,
      boolean limitPressed) {
    ExtensionCalibrator.Phase phase = m_calibrator.phase();
    int durationMs = (int) Math.round(seconds * 1000.0);
    int endMs = m_elapsedMs + durationMs;
    while (m_elapsedMs + kDtMs <= endMs) {
      if (untilPhaseChange && m_calibrator.phase() != phase) {
        break;
      }
      m_elapsedMs += kDtMs;
      ExtensionCalibrator.Action action =
          m_calibrator.step(
              new ExtensionCalibrator.Sample(
                  positionRotations, velocityRps, currentAmps, limitPressed, seconds()));
      if (action.applyVoltage) {
        m_lastVoltage = action.voltage;
      }
      if (action.zeroEncoder) {
        m_zeroed = true;
        m_zeroLeftRotations = action.encoderLeftRotations;
        m_zeroRightRotations = action.encoderRightRotations;
      }
      if (action.error != null) {
        m_error = action.error;
      }
      if (action.warning != null) {
        m_warning = action.warning;
      }
      remember(positionRotations, velocityRps, currentAmps, limitPressed);
    }
  }

  private void remember(
      double positionRotations, double velocityRps, double currentAmps, boolean limitPressed) {
    m_trace.add(
        String.format(
            Locale.US,
            "%.3f %s cmd=%.2f pos=%.2f vel=%.2f amps=%.1f limit=%s",
            seconds(),
            m_calibrator.phase(),
            m_lastVoltage,
            positionRotations,
            velocityRps,
            currentAmps,
            limitPressed));
    if (m_trace.size() > 30) {
      m_trace.remove(0);
    }
  }

  private double seconds() {
    return m_elapsedMs / 1000.0;
  }
}
