// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Constants.Intake.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
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

  private final VoltageOut m_voltageOut = new VoltageOut(0);
  private final ExtensionStateMachine m_extension =
      new ExtensionStateMachine(kExtensionTimeoutSeconds);

  /** True from the first retract of a shot until that shot puts the screws back. */
  private boolean m_holdingShotRetraction;
  /** Latched on the first retract of a shot. True only when that shot started fully extended. */
  private boolean m_restoreExtensionAfterShot;

  /** Creates a new IntakeSubsystem. Every boot starts in {@link State#REST}. */
  public IntakeSubsystem() {
    applyScrewCurrentLimit(m_leftScrew);
    applyScrewCurrentLimit(m_rightScrew);
    m_rightScrew.setControl(new Follower(m_leftScrew.getDeviceID(), kRightScrewAlignment));

    // Read the sensors faster than the 20 ms loop so a trigger is never a whole loop late.
    BaseStatusSignal.setUpdateFrequencyForAll(100, m_retractedSignal, m_extendedSignal);

    stopScrews();
  }

  /** S1: the intake is fully retracted. */
  public boolean isRetracted() {
    return m_retractedSignal.getValue();
  }

  /** S2: the intake is fully extended. */
  public boolean isExtended() {
    return m_extendedSignal.getValue();
  }

  public State getExtensionState() {
    return m_extension.state();
  }

  /** Runs every 20 ms robot loop: read the sensors, step the state machine, drive the screws. */
  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(m_retractedSignal, m_extendedSignal);
    boolean retracted = isRetracted();
    boolean extended = isExtended();

    State before = m_extension.state();
    State state = m_extension.update(retracted, extended, Timer.getFPGATimestamp());
    if (before != State.REST && state == State.REST && m_extension.lastMoveTimedOut()) {
      DriverStation.reportError(
          "Intake " + before.name().toLowerCase() + " timed out after "
              + kExtensionTimeoutSeconds + " s without its limit sensor. Check the CANdi wiring.",
          false);
    }

    switch (state) {
      case EXTENDING:
        m_leftScrew.setControl(m_voltageOut.withOutput(kExtendVoltage));
        break;
      case RETRACTING:
        m_leftScrew.setControl(m_voltageOut.withOutput(kRetractVoltage));
        break;
      case REST:
      default:
        stopScrews();
        break;
    }

    SmartDashboard.putString("Intake/State", state.name());
    SmartDashboard.putBoolean("Intake/RetractedLimit", retracted);
    SmartDashboard.putBoolean("Intake/ExtendedLimit", extended);
    SmartDashboard.putBoolean("Intake/CANdiConnected", m_retractedSignal.getStatus().isOK());
    SmartDashboard.putBoolean("Intake/RestoreAfterShot", m_restoreExtensionAfterShot);
  }

  /** Start extending. The periodic loop stops it when S2 triggers. Safe to call from auto. */
  public void extend() {
    m_extension.requestExtend(isExtended(), Timer.getFPGATimestamp());
  }

  /** Start retracting. The periodic loop stops it when S1 triggers. Safe to call from auto. */
  public void retract() {
    m_extension.requestRetract(isRetracted(), Timer.getFPGATimestamp());
  }

  /** Stop the extension where it is. */
  public void stopExtension() {
    m_extension.stop();
    stopScrews();
  }

  /**
   * Retract the extension for a shot. The first call remembers whether it was fully extended so
   * {@link #restoreAfterShot()} can send it back out. Further calls during the same shot do
   * nothing, so a retract that timed out is not restarted every loop.
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

  /** Stop the intake roller and the storage roller. */
  public void stopRollers() {
    m_intakeRoller.set(0);
    m_storageRoller.set(0);
  }

  /** Stop the storage roller. */
  public void stopStorage() {
    m_storageRoller.set(0);
  }

  /** Start extending and end right away. The periodic loop finishes the move. */
  public Command extendCommand() {
    return runOnce(this::extend);
  }

  /** Start retracting and end right away. The periodic loop finishes the move. */
  public Command retractCommand() {
    return runOnce(this::retract);
  }

  /** Extend and wait until the move ends, for auto steps that must not continue early. */
  public Command extendAndWaitCommand() {
    return extendCommand().andThen(waitUntilResting());
  }

  /** Retract and wait until the move ends, for auto steps that must not continue early. */
  public Command retractAndWaitCommand() {
    return retractCommand().andThen(waitUntilResting());
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

  private boolean isResting() {
    return m_extension.state() == State.REST;
  }

  /** Does not require the subsystem, so a shot or button can still change the move meanwhile. */
  private Command waitUntilResting() {
    return Commands.waitUntil(this::isResting);
  }

  private void stopScrews() {
    // The right screw is a follower, so only the leader is commanded.
    m_leftScrew.setControl(m_voltageOut.withOutput(0));
  }

  /**
   * Caps torque and battery current on one extension screw. The follower still produces its own
   * torque, so both screws get the same limit.
   */
  private void applyScrewCurrentLimit(TalonFX screw) {
    CurrentLimitsConfigs limits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kStatorCurrentLimitAmps)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(kSupplyCurrentLimitAmps)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLowerLimit(kSupplyCurrentLimitAmps)
            .withSupplyCurrentLowerTime(0.0);
    StatusCode status = screw.getConfigurator().apply(limits);
    if (!status.isOK()) {
      DriverStation.reportError(
          "Extension current limit was not applied to TalonFX " + screw.getDeviceID() + ": " + status,
          false);
    }
  }
}
