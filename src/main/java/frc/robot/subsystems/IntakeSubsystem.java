// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Constants.Intake.*;

import java.nio.file.Files;
import java.nio.file.Path;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_leftScrew = new TalonFX(kLeftScrewId);
  private final TalonFX m_rightScrew = new TalonFX(kRightScrewId);
  private final TalonFX m_intakeRoller = new TalonFX(kIntakeRollerId);
  private final TalonFX m_storageRoller = new TalonFX(kStorageRollerId);

  private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);
  private final VoltageOut m_voltageOut = new VoltageOut(0);
  private final DigitalInput m_limitSwitch = new DigitalInput(kLimitSwitchDio);
  private final Timer m_simTimer = new Timer();
  private final ExtensionCalibrator m_calibrator =
      new ExtensionCalibrator(ExtensionCalibrator.Config.intake());
  private final Path m_extensionMemoryPath =
      Filesystem.getOperatingDirectory().toPath().resolve(kExtensionMemoryFile);

  /** Skip rewriting the file when the screws were already told to hold this spot. */
  private static final double kPositionWriteDeadbandRotations = 0.05;

  /**
   * Encoder readback within this many rotations of the seeded value means {@code setPosition} has
   * been echoed, so a later measured save will not replace the file with the power-up zero.
   */
  private static final double kSeedEchoToleranceRotations = 0.05;

  private double m_savedPositionRotations = Double.NaN;
  /** Measured saves stay off until the motor echoes the seed, or a command writes a new target. */
  private boolean m_acceptMeasuredPosition;
  private boolean m_reportedWriteFailure;

  /** True from the first retract of a shot until that shot puts the screws back. */
  private boolean m_holdingShotRetraction;
  /** Latched on the first retract of a shot. True only when that shot started deployed. */
  private boolean m_restoreExtensionAfterShot;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    var gains = new Slot0Configs();
    gains.kP = kP;

    m_leftScrew.getConfigurator().apply(gains);
    m_rightScrew.getConfigurator().apply(gains);
    applyScrewCurrentLimit(m_leftScrew);
    applyScrewCurrentLimit(m_rightScrew);
    m_rightScrew.setControl(
        new Follower(m_leftScrew.getDeviceID(), MotorAlignmentValue.Aligned));

    // Broadcast current and velocity fast enough to see a hard-stop spike.
    m_leftScrew.getStatorCurrent().setUpdateFrequency(50);
    m_rightScrew.getStatorCurrent().setUpdateFrequency(50);
    m_leftScrew.getVelocity().setUpdateFrequency(50);
    m_leftScrew.getPosition().setUpdateFrequency(50);

    if (RobotBase.isSimulation()) {
      m_simTimer.start();
    }
    if (!restoreExtensionMemory() && RobotBase.isSimulation()) {
      // Simulation has no stator current, so a hard-stop sweep would only time out.
      m_calibrator.assumeCalibrated(
          kHardStopBackoffRotations, kSimTravelRotations - kHardStopBackoffRotations);
    }

    SmartDashboard.putData("Intake/Recalibrate", recalibrateCommand());
  }

  public boolean isCalibrated() {
    return m_calibrator.isCalibrated();
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
    SmartDashboard.putBoolean("Intake/Calibrated", m_calibrator.isCalibrated());
    SmartDashboard.putString("Intake/CalibrationPhase", m_calibrator.phase().name());
    SmartDashboard.putNumber("Intake/ScrewCurrentAmps", screwCurrentAmps());
    SmartDashboard.putNumber("Intake/ScrewStatorLimitAmps", kStatorCurrentLimitAmps);
    SmartDashboard.putNumber(
        "Intake/ScrewPositionRotations", m_leftScrew.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "Intake/ExtendPositionRotations", m_calibrator.extendPositionRotations());
    SmartDashboard.putNumber(
        "Intake/RetractPositionRotations", m_calibrator.retractPositionRotations());
    SmartDashboard.putNumber("Intake/SavedPositionRotations", m_savedPositionRotations);
    SmartDashboard.putBoolean("Intake/Extended", isExtended());
    SmartDashboard.putBoolean("Intake/RestoreAfterShot", m_restoreExtensionAfterShot);
    if (!m_acceptMeasuredPosition && m_calibrator.isCalibrated()) {
      var position = m_leftScrew.getPosition();
      if (position.getStatus() == StatusCode.OK
          && !Double.isNaN(m_savedPositionRotations)
          && Math.abs(position.getValueAsDouble() - m_savedPositionRotations)
              <= kSeedEchoToleranceRotations) {
        m_acceptMeasuredPosition = true;
      }
    }
  }

  /**
   * Writes the live encoder position when it is safe to trust. Disabled-init calls this so the
   * next boot matches where the screws stopped. A boot that has not yet heard the seeded position
   * echoed back leaves the file alone.
   */
  public void saveMeasuredExtension() {
    if (!m_calibrator.isCalibrated() || !m_acceptMeasuredPosition) {
      return;
    }
    var position = m_leftScrew.getPosition();
    position.refresh();
    if (position.getStatus() != StatusCode.OK) {
      return;
    }
    rememberPosition(position.getValueAsDouble());
  }

  /**
   * Sweeps both hard stops when the screws have no saved position. A restored file counts as
   * already calibrated, so this command ends immediately.
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
    if (!m_calibrator.isCalibrated()) {
      return;
    }
    double target = m_calibrator.extendPositionRotations();
    setLeftScrewPosition(target);
    rememberPosition(target);
  }

  /** Move the screws to the stowed position, just inside the fully-in hard stop. */
  public void retract() {
    if (!m_calibrator.isCalibrated()) {
      return;
    }
    double target = m_calibrator.retractPositionRotations();
    setLeftScrewPosition(target);
    rememberPosition(target);
  }

  /**
   * True when the screws were last told to hold the deployed setpoint. A shot reads this once,
   * before it stows them, so it can deploy again only if the shot started there.
   */
  public boolean isExtended() {
    if (!m_calibrator.isCalibrated() || Double.isNaN(m_savedPositionRotations)) {
      return false;
    }
    return Math.abs(m_savedPositionRotations - m_calibrator.extendPositionRotations())
        <= kExtendedToleranceRotations;
  }

  /**
   * Stow the screws for a shot. The first call remembers {@link #isExtended()} so a later
   * {@link #restoreAfterShot()} can deploy again. Further calls only keep holding the stow.
   */
  public void retractForShot() {
    if (!m_holdingShotRetraction) {
      m_restoreExtensionAfterShot = isExtended();
      m_holdingShotRetraction = true;
    }
    retract();
  }

  /** Deploy the screws again when the shot that stowed them had started extended. */
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

  /** Drive both screws to the same side of the extend setpoint. */
  public void resetScrews() {
    if (!m_calibrator.isCalibrated()) {
      return;
    }
    double position = -m_calibrator.extendPositionRotations();
    m_leftScrew.setControl(m_positionRequest.withPosition(position));
    m_rightScrew.setControl(m_positionRequest.withPosition(position));
    rememberPosition(position);
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

  /** Stop the linear screws and remember where they actually stopped. */
  public void stopScrews() {
    m_leftScrew.set(0);
    m_rightScrew.set(0);
    saveMeasuredExtension();
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
        .until(() -> m_calibrator.isCalibrated() || m_calibrator.isFailed())
        .finallyDo(this::finishCalibrationCommand)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  private void startCalibration(boolean force) {
    if (!m_calibrator.start(force, Timer.getFPGATimestamp())) {
      return;
    }
    // The sweep is about to move the screws, so the old file is no longer where they are.
    m_savedPositionRotations = Double.NaN;
    m_acceptMeasuredPosition = false;
    ExtensionMemory.delete(m_extensionMemoryPath);
    m_rightScrew.setControl(
        new Follower(m_leftScrew.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  private void executeCalibration() {
    ExtensionCalibrator.Action action =
        m_calibrator.step(
            new ExtensionCalibrator.Sample(
                m_leftScrew.getPosition().getValueAsDouble(),
                m_leftScrew.getVelocity().getValueAsDouble(),
                screwCurrentAmps(),
                isLimitPressed(),
                Timer.getFPGATimestamp()));
    if (action.warning != null) {
      DriverStation.reportWarning(action.warning, false);
    }
    if (action.error != null) {
      DriverStation.reportError(action.error, false);
    }
    if (action.applyVoltage) {
      m_leftScrew.setControl(m_voltageOut.withOutput(action.voltage));
    }
    if (action.zeroEncoder) {
      seedEncoder(action.encoderLeftRotations);
      // Record the position at the hard stop, then retract() below replaces it with the stow
      // target the screws are told to hold.
      rememberPosition(action.encoderLeftRotations);
    }
    if (action.holdRetract) {
      // Leave the screws just off whichever stop we finished on, then stow them.
      retract();
    }
    if (action.stopScrews) {
      stopScrews();
    }
  }

  private void finishCalibrationCommand(boolean interrupted) {
    m_calibrator.onCommandEnd(interrupted);
    if (m_calibrator.isCalibrated()) {
      return;
    }
    stopScrews();
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

  private double screwCurrentAmps() {
    return Math.max(
        m_leftScrew.getStatorCurrent().getValueAsDouble(),
        m_rightScrew.getStatorCurrent().getValueAsDouble());
  }

  private void setLeftScrewPosition(double leftRotations) {
    m_leftScrew.setControl(m_positionRequest.withPosition(leftRotations));
    m_rightScrew.setControl(m_positionRequest.withPosition(-leftRotations));
  }

  /**
   * Seeds both screw encoders in the calibrated frame. The right screw is commanded to the
   * opposite rotation everywhere else, so its sensor has to start there too.
   */
  private void seedEncoder(double leftRotations) {
    m_leftScrew.setPosition(leftRotations);
    m_rightScrew.setPosition(-leftRotations);
  }

  /** @return true when a saved sweep was applied to the calibrator and the encoders */
  private boolean restoreExtensionMemory() {
    if (!Files.isRegularFile(m_extensionMemoryPath)) {
      return false;
    }
    var saved = ExtensionMemory.read(m_extensionMemoryPath);
    if (saved.isEmpty()) {
      DriverStation.reportError(
          "Intake extension file could not be read: " + m_extensionMemoryPath, false);
      return false;
    }
    ExtensionMemory.Snapshot snapshot = saved.get();
    m_calibrator.assumeCalibrated(snapshot.retractRotations, snapshot.extendRotations);
    seedEncoder(snapshot.positionRotations);
    m_savedPositionRotations = snapshot.positionRotations;
    m_acceptMeasuredPosition = false;
    DriverStation.reportWarning(
        "Intake extension restored at " + snapshot.positionRotations + " rotations.", false);
    return true;
  }

  /** Writes the calibrated ends and this left-screw position. Repeated targets are skipped. */
  private void rememberPosition(double leftRotations) {
    if (!m_calibrator.isCalibrated() || !Double.isFinite(leftRotations)) {
      return;
    }
    if (!Double.isNaN(m_savedPositionRotations)
        && Math.abs(m_savedPositionRotations - leftRotations) < kPositionWriteDeadbandRotations) {
      return;
    }
    boolean wrote =
        ExtensionMemory.write(
            m_extensionMemoryPath,
            new ExtensionMemory.Snapshot(
                m_calibrator.retractPositionRotations(),
                m_calibrator.extendPositionRotations(),
                leftRotations));
    if (!wrote) {
      if (!m_reportedWriteFailure) {
        DriverStation.reportError(
            "Could not write intake extension file: " + m_extensionMemoryPath, false);
        m_reportedWriteFailure = true;
      }
      return;
    }
    m_reportedWriteFailure = false;
    m_savedPositionRotations = leftRotations;
    m_acceptMeasuredPosition = true;
  }
}
