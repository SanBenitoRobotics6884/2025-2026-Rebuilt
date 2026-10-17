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
      m_calibrator.assumeCalibrated(
          kHardStopBackoffRotations, kSimTravelRotations - kHardStopBackoffRotations);
      m_simTimer.start();
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
    SmartDashboard.putNumber(
        "Intake/ScrewPositionRotations", m_leftScrew.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "Intake/ExtendPositionRotations", m_calibrator.extendPositionRotations());
    SmartDashboard.putNumber(
        "Intake/RetractPositionRotations", m_calibrator.retractPositionRotations());
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
    if (!m_calibrator.isCalibrated()) {
      return;
    }
    setLeftScrewPosition(m_calibrator.extendPositionRotations());
  }

  /** Move the screws to the stowed position, just inside the fully-in hard stop. */
  public void retract() {
    if (!m_calibrator.isCalibrated()) {
      return;
    }
    setLeftScrewPosition(m_calibrator.retractPositionRotations());
  }

  /** Drive both screws to the same side of the extend setpoint. */
  public void resetScrews() {
    if (!m_calibrator.isCalibrated()) {
      return;
    }
    double position = -m_calibrator.extendPositionRotations();
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
        .until(() -> m_calibrator.isCalibrated() || m_calibrator.isFailed())
        .finallyDo(this::finishCalibrationCommand)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  private void startCalibration(boolean force) {
    if (!m_calibrator.start(force, Timer.getFPGATimestamp())) {
      return;
    }
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
      m_leftScrew.setPosition(action.encoderLeftRotations);
      m_rightScrew.setPosition(action.encoderRightRotations);
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
