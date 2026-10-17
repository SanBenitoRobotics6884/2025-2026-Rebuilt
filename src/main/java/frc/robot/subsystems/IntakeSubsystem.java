// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Constants.Intake.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private enum CalibrationPhase {
    IDLE,
    SEEK_FIRST,
    SEEK_SECOND,
    DONE,
    FAILED
  }

  private final TalonFX m_leftScrew = new TalonFX(kLeftScrewId);
  private final TalonFX m_rightScrew = new TalonFX(kRightScrewId);
  private final TalonFX m_intakeRoller = new TalonFX(kIntakeRollerId);
  private final TalonFX m_storageRoller = new TalonFX(kStorageRollerId);

  private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);
  private final VoltageOut m_voltageOut = new VoltageOut(0);
  private final DigitalInput m_limitSwitch = new DigitalInput(kLimitSwitchDio);
  private final Timer m_simTimer = new Timer();
  private final Timer m_phaseTimer = new Timer();
  private final Timer m_stallTimer = new Timer();

  /**
   * Left-screw encoder targets in rotations. Zero is the fully-in hard stop, after calibration
   * shifts the encoder. The right screw is commanded to the opposite number.
   */
  private double m_extendPositionRotations;
  private double m_retractPositionRotations;

  private CalibrationPhase m_phase = CalibrationPhase.IDLE;
  private boolean m_calibrated;
  private boolean m_failed;
  private double m_appliedVoltage;
  private double m_firstPositionRotations;
  private boolean m_firstHitSwitch;
  /** Second seek must leave the first stop before another spike can count. */
  private boolean m_requireMotion;
  private boolean m_seenMotion;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    var gains = new Slot0Configs();
    gains.kP = kP;

    m_leftScrew.getConfigurator().apply(gains);
    m_rightScrew.getConfigurator().apply(gains);
    m_rightScrew.setControl(
        new Follower(m_leftScrew.getDeviceID(), MotorAlignmentValue.Aligned));

    // Broadcast current and velocity fast enough to see a hard-stop spike.
    m_leftScrew.getStatorCurrent().setUpdateFrequency(50);
    m_rightScrew.getStatorCurrent().setUpdateFrequency(50);
    m_leftScrew.getVelocity().setUpdateFrequency(50);
    m_leftScrew.getPosition().setUpdateFrequency(50);

    if (RobotBase.isSimulation()) {
      // Simulation has no stator current, so a hard-stop sweep would only time out.
      m_retractPositionRotations = kHardStopBackoffRotations;
      m_extendPositionRotations = kSimTravelRotations - kHardStopBackoffRotations;
      m_calibrated = true;
      m_phase = CalibrationPhase.DONE;
      m_simTimer.start();
    }

    SmartDashboard.putData("Intake/Recalibrate", recalibrateCommand());
  }

  public boolean isCalibrated() {
    return m_calibrated;
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
    SmartDashboard.putBoolean("Intake/Calibrated", m_calibrated);
    SmartDashboard.putString("Intake/CalibrationPhase", m_phase.name());
    SmartDashboard.putNumber("Intake/ScrewCurrentAmps", screwCurrentAmps());
    SmartDashboard.putNumber(
        "Intake/ScrewPositionRotations", m_leftScrew.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake/ExtendPositionRotations", m_extendPositionRotations);
    SmartDashboard.putNumber("Intake/RetractPositionRotations", m_retractPositionRotations);
  }

  /**
   * Drives into both hard stops once per boot. The encoder reads 0 at power-up no matter where the
   * screws actually are, so extend and retract stay put until this finishes.
   */
  public Command calibrateCommand() {
    return buildCalibrateCommand(false);
  }

  /** Runs the hard-stop sweep again, even if a previous pass already succeeded. */
  public Command recalibrateCommand() {
    return buildCalibrateCommand(true);
  }

  /** Move the screws to the deployed position, just inside the outer hard stop. */
  public void extend() {
    if (!m_calibrated) {
      return;
    }
    setLeftScrewPosition(m_extendPositionRotations);
  }

  /** Move the screws to the stowed position, just inside the fully-in hard stop. */
  public void retract() {
    if (!m_calibrated) {
      return;
    }
    setLeftScrewPosition(m_retractPositionRotations);
  }

  /** Drive both screws to the same side of the extend setpoint. */
  public void resetScrews() {
    if (!m_calibrated) {
      return;
    }
    double position = -m_extendPositionRotations;
    m_leftScrew.setControl(m_positionRequest.withPosition(position));
    m_rightScrew.setControl(m_positionRequest.withPosition(position));
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

  /** Stop the linear screws. */
  public void stopScrews() {
    m_leftScrew.set(0);
    m_rightScrew.set(0);
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

  private Command buildCalibrateCommand(boolean force) {
    return run(this::executeCalibration)
        .beforeStarting(() -> startCalibration(force))
        .until(() -> m_calibrated || m_failed)
        .finallyDo(this::finishCalibrationCommand)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  private void startCalibration(boolean force) {
    if (m_calibrated && !force) {
      return;
    }
    m_calibrated = false;
    m_failed = false;
    m_phase = CalibrationPhase.SEEK_FIRST;
    m_appliedVoltage = kInwardVoltage;
    m_firstHitSwitch = false;
    m_requireMotion = false;
    m_seenMotion = false;
    m_rightScrew.setControl(
        new Follower(m_leftScrew.getDeviceID(), MotorAlignmentValue.Aligned));
    m_phaseTimer.restart();
    m_stallTimer.stop();
    m_stallTimer.reset();
  }

  private void executeCalibration() {
    if (m_calibrated || m_failed || m_phase == CalibrationPhase.IDLE
        || m_phase == CalibrationPhase.DONE) {
      return;
    }
    if (m_phaseTimer.hasElapsed(kCalibrateTimeoutSeconds)) {
      failCalibration("Intake calibration timed out before a hard stop.");
      return;
    }

    m_leftScrew.setControl(m_voltageOut.withOutput(m_appliedVoltage));
    if (!m_phaseTimer.hasElapsed(kInrushSeconds)) {
      m_stallTimer.stop();
      m_stallTimer.reset();
      return;
    }
    double velocityRps = Math.abs(m_leftScrew.getVelocity().getValueAsDouble());
    if (velocityRps > kStallVelocityRps) {
      m_seenMotion = true;
    }
    if (m_requireMotion && !m_seenMotion) {
      return;
    }
    if (velocityRps >= kStallVelocityRps || screwCurrentAmps() <= kStallCurrentAmps) {
      m_stallTimer.stop();
      m_stallTimer.reset();
      return;
    }
    if (!m_stallTimer.isRunning()) {
      m_stallTimer.restart();
    }
    if (m_stallTimer.hasElapsed(kStallSeconds)) {
      captureHardStop();
    }
  }

  private void captureHardStop() {
    double position = m_leftScrew.getPosition().getValueAsDouble();
    boolean switchPressed = isLimitPressed();
    if (m_phase == CalibrationPhase.SEEK_FIRST) {
      m_firstPositionRotations = position;
      m_firstHitSwitch = switchPressed;
      m_phase = CalibrationPhase.SEEK_SECOND;
      m_appliedVoltage = -kInwardVoltage;
      m_requireMotion = true;
      m_seenMotion = false;
      m_phaseTimer.restart();
      m_stallTimer.stop();
      m_stallTimer.reset();
      return;
    }
    finishCalibration(position, switchPressed);
  }

  private void finishCalibration(double secondPositionRotations, boolean secondHitSwitch) {
    boolean firstIsIn;
    if (m_firstHitSwitch == secondHitSwitch) {
      firstIsIn = true;
      DriverStation.reportWarning(
          "Intake limit switch did not identify the fully-in stop. Using kInwardVoltage direction.",
          false);
    } else {
      firstIsIn = m_firstHitSwitch;
    }

    double inPosition = firstIsIn ? m_firstPositionRotations : secondPositionRotations;
    double outPosition = firstIsIn ? secondPositionRotations : m_firstPositionRotations;
    double travel = outPosition - inPosition;
    if (Math.abs(travel) < kMinTravelRotations) {
      failCalibration("Intake hard stops were too close together to calibrate.");
      return;
    }

    double shifted = m_leftScrew.getPosition().getValueAsDouble() - inPosition;
    m_leftScrew.setPosition(shifted);
    m_rightScrew.setPosition(-shifted);

    double backoff = Math.copySign(kHardStopBackoffRotations, travel);
    m_retractPositionRotations = backoff;
    m_extendPositionRotations = travel - backoff;
    m_calibrated = true;
    m_failed = false;
    m_phase = CalibrationPhase.DONE;
    // Leave the screws just off whichever stop we finished on, then stow them.
    retract();
  }

  private void failCalibration(String message) {
    m_failed = true;
    m_calibrated = false;
    m_phase = CalibrationPhase.FAILED;
    stopScrews();
    DriverStation.reportError(message, false);
  }

  private void finishCalibrationCommand(boolean interrupted) {
    if (m_calibrated) {
      return;
    }
    if (interrupted) {
      m_phase = CalibrationPhase.IDLE;
      m_failed = false;
    }
    stopScrews();
  }

  private double screwCurrentAmps() {
    return Math.max(
        m_leftScrew.getStatorCurrent().getValueAsDouble(),
        m_rightScrew.getStatorCurrent().getValueAsDouble());
  }

  private void setLeftScrewPosition(double leftRotations) {
    m_leftScrew.setControl(m_positionRequest.withPosition(leftRotations));
    m_rightScrew.setControl(m_positionRequest.withPosition(-leftRotations));
  }
}
