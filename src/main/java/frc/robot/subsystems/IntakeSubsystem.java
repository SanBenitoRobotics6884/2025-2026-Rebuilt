// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Constants.Intake.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ExtensionStateMachine.State;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_leftScrew = new TalonFX(kLeftScrewId);
  private final TalonFX m_rightScrew = new TalonFX(kRightScrewId);
  private final TalonFX m_intakeRoller = new TalonFX(kIntakeRollerId);
  private final TalonFX m_storageRoller = new TalonFX(kStorageRollerId);

  /**
   * CANdi with the two extension limit sensors. S1 is fully retracted, S2 is fully extended. If a
   * sensor reads backwards on the dashboard, set {@code S1CloseState} / {@code S2CloseState} in a
   * {@code CANdiConfiguration} and apply it in the constructor.
   */
  private final CANdi m_candi = new CANdi(kCandiId);
  private final StatusSignal<Boolean> m_retractedSignal = m_candi.getS1Closed();
  private final StatusSignal<Boolean> m_extendedSignal = m_candi.getS2Closed();
  private final StatusSignal<Angle> m_leftPosSignal = m_leftScrew.getPosition();
  private final StatusSignal<Angle> m_rightPosSignal = m_rightScrew.getPosition();

  private final PositionVoltage m_leftRequest = new PositionVoltage(0).withSlot(0);
  private final PositionVoltage m_rightRequest = new PositionVoltage(0).withSlot(0);
  private final NeutralOut m_neutral = new NeutralOut();

  private final ExtensionStateMachine m_extension =
      new ExtensionStateMachine(
          new ScrewSync.Config(
              kSyncSpeedRps,
              kFollowToleranceRot,
              kSyncFaultRot,
              kTrackingFaultRot,
              kFaultDebounceSeconds,
              kNoProgressSeconds,
              -kTravelMarginRot,
              kTravelRotations + kTravelMarginRot),
          kExtensionTimeoutSeconds,
          kUnhomedRetractStepRot);

  // Latest readings, refreshed by syncLoop().
  private double m_leftPos;
  private double m_rightPos;
  private boolean m_retracted;
  private boolean m_extended;
  private boolean m_sensorsOk;

  /**
   * Duty cycle the intake roller has been asked for (0 = off). It only reaches the motor while
   * {@link #intakeRollerAllowed} says the intake is far enough out. See syncLoop().
   */
  private double m_intakeRollerRequest;
  private boolean m_intakeRollerAllowed;

  /** Simulated screws and sensors. Null on the real robot. */
  private final IntakeSim m_sim;

  /** True from the first retract of a shot until that shot puts the screws back. */
  private boolean m_holdingShotRetraction;
  /** Latched on the first retract of a shot. True only when that shot started fully extended. */
  private boolean m_restoreExtensionAfterShot;

  /** Creates a new IntakeSubsystem. Every boot starts in {@link State#REST}. */
  public IntakeSubsystem() {
    InvertedValue rightInverted =
        kRightScrewAlignment == MotorAlignmentValue.Opposed ? flip(kLeftScrewInverted) : kLeftScrewInverted;
    configureScrew(m_leftScrew, kLeftScrewInverted);
    configureScrew(m_rightScrew, rightInverted);

    // Fresh readings for every sync check.
    BaseStatusSignal.setUpdateFrequencyForAll(
        1.0 / kSyncPeriodSeconds,
        m_leftPosSignal,
        m_rightPosSignal,
        m_retractedSignal,
        m_extendedSignal);

    // Both encoders start equal, so the screws are compared turn for turn from power-up. The first
    // retract that reaches S1 sets both to 0 again.
    zeroScrews();
    stopScrews();

    // In simulation, fake screws and sensors stand in for the real ones. Created before the S1
    // read below so the simulated sensor already shows where the intake starts.
    m_sim =
        RobotBase.isSimulation()
            ? new IntakeSim(
                m_leftScrew,
                m_rightScrew,
                m_candi,
                kLeftScrewInverted,
                rightInverted,
                IntakeSim.startRotationsFromEnvironment())
            : null;

    // 0 only means "retracted" if the intake was retracted at power-up. If S1 is not closed (or
    // can't be read), retract walks in a few rotations per press until S1 homes it.
    boolean atRetracted = m_retractedSignal.waitForUpdate(0.25).getValue();
    boolean homed = m_retractedSignal.getStatus().isOK() && atRetracted;
    m_extension.setHomed(homed);
    if (!homed) {
      DriverStation.reportWarning(
          "Intake not homed: it was not at the retracted sensor (S1) at power-up. Press retract"
              + " to walk it in " + kUnhomedRetractStepRot + " rotations at a time until S1.",
          false);
    }
  }

  /** S1: the intake is fully retracted. */
  public boolean isRetracted() {
    return m_retracted;
  }

  /** S2: the intake is fully extended. */
  public boolean isExtended() {
    return m_extended;
  }

  public State getExtensionState() {
    return m_extension.state();
  }

  /**
   * The lead-screw sync check. {@code Robot} runs this every {@link
   * frc.robot.Constants.Constants.Intake#kSyncPeriodSeconds}. It reads both screw
   * encoders and both limit sensors, checks that each screw really turned as far as it was told,
   * and then either moves both on together or stops both.
   */
  public void syncLoop() {
    if (m_sim != null) {
      // Simulation: move the fake screws once per sync check, the rate the real encoders report at.
      m_sim.update(kSyncPeriodSeconds);
    }
    readSensors();
    double now = Timer.getFPGATimestamp();

    State before = m_extension.state();
    State state;
    if (m_sensorsOk) {
      state = m_extension.update(m_leftPos, m_rightPos, m_retracted, m_extended, now);
    } else {
      // No trustworthy encoder or sensor data: do not move the screws on a guess.
      m_extension.stop();
      state = m_extension.state();
    }

    if (state == State.FAULT && before != State.FAULT) {
      DriverStation.reportError(
          "Intake screws stopped, " + m_extension.fault() + ": " + m_extension.faultDetail()
              + ". Press extend or retract to re-align them.",
          false);
    } else if (before != State.REST && state == State.REST) {
      if (m_extension.lastMoveTimedOut()) {
        DriverStation.reportError(
            "Intake move timed out after " + kExtensionTimeoutSeconds + " s without its limit sensor.",
            false);
      } else if (m_extension.lastMoveHitSoftLimit()) {
        DriverStation.reportWarning(
            "Intake reached the travel limit without its CANdi sensor triggering. Check the sensor"
                + " and kTravelRotations.",
            false);
      }
      if (before == State.RETRACTING && m_retracted) {
        zeroScrews();
        m_extension.setHomed(true);
      }
    }
    if (!m_extension.isHomed() && m_sensorsOk && m_retracted && !m_extension.isDriving()) {
      // Sitting on S1 (e.g. the power-up read failed): that is home, no move needed.
      zeroScrews();
      m_extension.setHomed(true);
    }
    if (m_extension.takeExtendRefused()) {
      DriverStation.reportWarning(
          "Intake not homed: extend ignored. Retract to the S1 sensor first.", false);
    }

    if (m_extension.isDriving()) {
      m_leftScrew.setControl(
          m_leftRequest
              .withPosition(m_extension.leftTarget())
              .withVelocity(m_extension.leftVelocity()));
      m_rightScrew.setControl(
          m_rightRequest
              .withPosition(m_extension.rightTarget())
              .withVelocity(m_extension.rightVelocity()));
    } else {
      stopScrews();
    }

    // Intake roller: only spin it once the intake is far enough out that the wheels clear.
    m_intakeRollerAllowed =
        m_sensorsOk
            && intakeRollerAllowed(m_extension.isHomed(), m_extended, m_leftPos, m_rightPos);
    m_intakeRoller.set(m_intakeRollerAllowed ? m_intakeRollerRequest : 0.0);
  }

  /**
   * Whether the intake is far enough out for the intake roller to spin without rubbing.
   *
   * <p>Fully extended (S2 closed) is always far enough. Otherwise the encoders decide, using the
   * screw that is further in, but only once the intake is homed. Before that, the encoder
   * position is a guess, so the roller waits.
   */
  static boolean intakeRollerAllowed(
      boolean homed, boolean extendedSensor, double leftPos, double rightPos) {
    if (extendedSensor) {
      return true;
    }
    if (!homed) {
      return false;
    }
    return Math.min(leftPos, rightPos) >= kIntakeRollerMinExtension * kTravelRotations;
  }

  /** Dashboard only. The control work happens in {@link #syncLoop()}. */
  @Override
  public void periodic() {
    SmartDashboard.putString("Intake/State", m_extension.state().name());
    SmartDashboard.putString("Intake/Fault", m_extension.fault().name());
    SmartDashboard.putBoolean("Intake/Homed", m_extension.isHomed());
    SmartDashboard.putBoolean("Intake/RollerAllowed", m_intakeRollerAllowed);
    SmartDashboard.putNumber(
        "Intake/ExtendedPercent", 100.0 * Math.min(m_leftPos, m_rightPos) / kTravelRotations);
    SmartDashboard.putNumber("Intake/LeftPos", m_leftPos);
    SmartDashboard.putNumber("Intake/RightPos", m_rightPos);
    SmartDashboard.putNumber("Intake/Apart", m_leftPos - m_rightPos);
    SmartDashboard.putNumber("Intake/LeftTarget", m_extension.leftTarget());
    SmartDashboard.putNumber("Intake/RightTarget", m_extension.rightTarget());
    SmartDashboard.putBoolean("Intake/RetractedLimit", m_retracted);
    SmartDashboard.putBoolean("Intake/ExtendedLimit", m_extended);
    SmartDashboard.putBoolean("Intake/SensorsOK", m_sensorsOk);
    SmartDashboard.putBoolean("Intake/RestoreAfterShot", m_restoreExtensionAfterShot);
  }

  /**
   * Simulation only: put the intake back to fully retracted and homed, with everything stopped,
   * so a test can start the next auto from a known state. Does nothing on the real robot.
   */
  public void simResetRetracted() {
    if (m_sim == null) {
      return;
    }
    m_extension.stop();
    m_extension.setHomed(true);
    stopScrews();
    stopRollers();
    clearShotMemory();
    m_sim.setPosition(0.0);
    zeroScrews();
  }

  /** Simulation only: where the simulated intake really is, in rotations. NaN on the robot. */
  public double simPosition() {
    return m_sim == null ? Double.NaN : m_sim.position();
  }

  /** Start extending. The sync loop stops it when S2 triggers. Safe to call from auto. */
  public void extend() {
    m_extension.requestExtend(m_leftPos, m_rightPos, m_extended, Timer.getFPGATimestamp());
  }

  /** Start retracting. The sync loop stops it when S1 triggers. Safe to call from auto. */
  public void retract() {
    m_extension.requestRetract(m_leftPos, m_rightPos, m_retracted, Timer.getFPGATimestamp());
  }

  /** Stop the extension where it is. A fault stays latched until the next extend or retract. */
  public void stopExtension() {
    m_extension.stop();
    stopScrews();
  }

  /**
   * Retract the extension for a shot. The first call remembers whether it was fully extended so
   * {@link #restoreAfterShot()} can send it back out. Further calls during the same shot do
   * nothing, so a retract that stopped early is not restarted every loop.
   */
  public void retractForShot() {
    if (m_holdingShotRetraction) {
      return;
    }
    m_restoreExtensionAfterShot = isExtended() || m_extension.state() == State.EXTENDING;
    m_holdingShotRetraction = true;
    retract();
  }

  /** Extend again when the shot that retracted the intake had started extended. */
  public void restoreAfterShot() {
    if (!m_holdingShotRetraction) {
      return;
    }
    m_holdingShotRetraction = false;
    boolean restore = m_restoreExtensionAfterShot;
    m_restoreExtensionAfterShot = false;
    if (restore) {
      extend();
    }
  }

  /**
   * Forget a shot that never finished, e.g. an auto that ended between starting and stopping the
   * shooter. Without this, the next shot would skip its retract and might extend on release
   * because of what the unfinished shot remembered. Does not move the screws.
   */
  public void clearShotMemory() {
    m_holdingShotRetraction = false;
    m_restoreExtensionAfterShot = false;
  }

  public void intake() {
    // syncLoop() starts it once the intake is far enough out.
    m_intakeRollerRequest = kIntakeSpeed;
  }

  public void reverseIntake() {
    m_intakeRollerRequest = -kIntakeSpeed;
  }

  public void runStorage() {
    m_storageRoller.set(kStorageSpeed);
  }

  public void reverseStorage() {
    m_storageRoller.set(-kStorageSpeed);
  }

  /** Stop the intake roller and the storage roller. */
  public void stopRollers() {
    m_intakeRollerRequest = 0;
    m_intakeRoller.set(0);
    m_storageRoller.set(0);
  }

  /** Stop the storage roller. */
  public void stopStorage() {
    m_storageRoller.set(0);
  }

  /** Start extending and end right away. The sync loop finishes the move. */
  public Command extendCommand() {
    return runOnce(this::extend);
  }

  /** Start retracting and end right away. The sync loop finishes the move. */
  public Command retractCommand() {
    return runOnce(this::retract);
  }

  /** Extend and wait until the move ends (or faults), for auto steps that must not continue early. */
  public Command extendAndWaitCommand() {
    return extendCommand().andThen(waitUntilStopped());
  }

  /** Retract and wait until the move ends (or faults), for auto steps that must not continue early. */
  public Command retractAndWaitCommand() {
    return retractCommand().andThen(waitUntilStopped());
  }

  public Command stopExtensionCommand() {
    return runOnce(this::stopExtension);
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

  public Command stopRollersCommand() {
    return runOnce(this::stopRollers);
  }

  public Command stopStorageCommand() {
    return runOnce(this::stopStorage);
  }

  /** Does not require the subsystem, so a shot or button can still change the move meanwhile. */
  private Command waitUntilStopped() {
    return Commands.waitUntil(() -> !m_extension.isDriving());
  }

  private void readSensors() {
    StatusCode status =
        BaseStatusSignal.refreshAll(
            m_leftPosSignal, m_rightPosSignal, m_retractedSignal, m_extendedSignal);
    m_sensorsOk = status.isOK();
    m_leftPos = m_leftPosSignal.getValueAsDouble();
    m_rightPos = m_rightPosSignal.getValueAsDouble();
    m_retracted = m_retractedSignal.getValue();
    m_extended = m_extendedSignal.getValue();
  }

  private void stopScrews() {
    // Each screw is stopped on its own; neither follows the other.
    m_leftScrew.setControl(m_neutral);
    m_rightScrew.setControl(m_neutral);
  }

  private void zeroScrews() {
    m_leftScrew.setPosition(0);
    m_rightScrew.setPosition(0);
    m_leftPos = 0;
    m_rightPos = 0;
  }

  /**
   * Brake when stopped, positive = outward, position gains, and torque/battery current caps. Each
   * screw gets the same settings so they behave the same.
   */
  private void configureScrew(TalonFX screw, InvertedValue inverted) {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = inverted;
    config.Slot0.kP = kScrewKp;
    config.Slot0.kV = kScrewKv;
    config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = kSupplyCurrentLimitAmps;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.0;
    StatusCode status = screw.getConfigurator().apply(config);
    if (!status.isOK()) {
      DriverStation.reportError(
          "Intake screw TalonFX " + screw.getDeviceID() + " was not configured: " + status, false);
    }
  }

  private static InvertedValue flip(InvertedValue value) {
    return value == InvertedValue.Clockwise_Positive
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
  }
}
