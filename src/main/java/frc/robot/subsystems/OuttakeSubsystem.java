// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Constants.Outtake.*;

public class OuttakeSubsystem extends SubsystemBase {
  TalonFX m_outtakeMotor;
  TalonFX m_indexMotor;
  DutyCycleOut speed;

  final VelocityVoltage m_request = new VelocityVoltage(0);
  double targetRPM = 4000;
  double targetRPS = targetRPM / 60.0;

  InterpolatingDoubleTreeMap m_linearInterpolator = new InterpolatingDoubleTreeMap();

  /** Creates a new OuttakeSubsystem. */

  public OuttakeSubsystem() {
    m_outtakeMotor = new TalonFX(60);
    m_indexMotor = new TalonFX(50);
  
  //RPM
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE; // Tune this value (output per rotation of error)
    // slot0Configs.kI = PID_I_VALUE;
    // slot0Configs.kD = PID_D_VALUE;
    // Add kI, kD, kS, kV if needed for better control
    m_outtakeMotor.getConfigurator().apply(slot0Configs);
    m_indexMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //MOTOR USES

//Normal Run
  public void runOuttake() {
    m_outtakeMotor.setControl(m_request.withVelocity(-targetRPS));
  }
  public void runIndex(){
    m_indexMotor.setControl(m_request.withVelocity(targetRPS));
  }
    public void reverseIndex() {
    m_indexMotor.setControl(m_request.withVelocity(-targetRPS));
  }

//Slow Mode
  public void runOuttakeSlow() {
    m_outtakeMotor.setControl(m_request.withVelocity(-targetRPS * OUTTAKE_SPEED_SLOW));
    m_indexMotor.setControl(m_request.withVelocity(targetRPS * INDEX_SPEED_SLOW));
  }

//Stops
  public void stopOuttake() {
    m_outtakeMotor.set(0);
  }
  public void stopIndex(){
     m_indexMotor.set(0);
  }
  
  // Needs to be fed the distance to the hub from the bot.
    public void advShootBall(double aprilTagDist) {
    /*
     * The method will shoot the ball, but takes into account the distance of the select AprilTag and decide speed from there.
     * Plans to use Linear Interpolation for this with PhotonVision.
     */
    m_outtakeMotor.set(interpolateDistance(aprilTagDist));
  }

  public double interpolateDistance(double distance) {
    // Needs measurements.
    m_linearInterpolator.put(1.0, 1.0);  // key -> meters away from hub, value -> motor speed from (0 - 1)
    m_linearInterpolator.put(null, null);
    return m_linearInterpolator.get(distance);
  }

  //COMMANDS

//Run Commands
  public Command runOuttakecommand() {
    return run(this::runOuttake);
  }
    public Command runOuttakeSlowCommand() {
    return run(this::runOuttakeSlow);
  }
    public Command runIndexCommand() {
      return run(this::runIndex);
    }
    public Command runReverseIndexCommand() {
    return run(this::reverseIndex);
  }

//Stop Commands
  public Command stopOuttakeCommand() {
    return runOnce(this::stopOuttake);
  }
  public Command stopIndexCommand() {
    return runOnce(this::stopIndex);
  }
}
