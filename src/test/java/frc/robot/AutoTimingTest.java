// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ExtensionStateMachine.State;
import frc.robot.subsystems.IntakeSubsystem;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/**
 * Runs every PathPlanner auto in simulation and reports how long each one takes.
 *
 * <p>Run with {@code ./gradlew autoTiming}. Not part of {@code ./gradlew build}: the simulated
 * motors and swerve drive run on the real clock, so this takes about as long as the autos do
 * (roughly 20 s each).
 *
 * <p>An auto fails if it is still running after the 20 s auto period, or if it asks for
 * {@code extendIntakeAndWait} and the simulated intake never reaches its extended sensor (S2).
 * Each auto starts with the intake retracted and homed, as it would be in a match.
 */
@Tag("autoTiming")
class AutoTimingTest {
  static final double kAutoPeriodSeconds = 20.0;
  /** Stop waiting this long after the start, so a stuck auto can't hang the test. */
  static final double kGiveUpSeconds = 25.0;
  static final double kLoopSeconds = 0.01;

  record Result(String name, double seconds, boolean finished, double extendSeconds,
      boolean needsExtend, boolean reachedExtended) {
    boolean tooLong() {
      return !finished || seconds > kAutoPeriodSeconds;
    }

    boolean intakeProblem() {
      return needsExtend && !reachedExtended;
    }
  }

  RobotContainer m_container;
  long m_loops;
  double m_nextLoopTime;

  @Test
  void everyAutoFinishesInsideTheAutoPeriod() throws Exception {
    assertTrue(HAL.initialize(500, 0), "HAL did not start");
    // No controllers in this test; don't flood the output with "joystick not available".
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    m_container = new RobotContainer();
    m_nextLoopTime = Timer.getFPGATimestamp();

    List<String> names = new ArrayList<>(AutoBuilder.getAllAutoNames());
    names.sort(String::compareToIgnoreCase);
    assertTrue(!names.isEmpty(), "No autos found in the deploy folder");

    List<Result> results = new ArrayList<>();
    for (String name : names) {
      results.add(runAuto(name));
    }

    System.out.println();
    System.out.println("Auto timing (simulation, real time). Limit " + kAutoPeriodSeconds + " s.");
    System.out.printf("%-66s %9s %10s  %s%n", "Auto", "Total s", "Extend s", "Result");
    StringBuilder failures = new StringBuilder();
    for (Result r : results) {
      String verdict;
      if (r.tooLong()) {
        verdict = r.finished() ? "FAIL: over 20 s" : "FAIL: still running at " + kGiveUpSeconds + " s";
      } else if (r.intakeProblem()) {
        verdict = "FAIL: intake never reached S2";
      } else {
        verdict = "ok";
      }
      System.out.printf("%-66s %9.2f %10s  %s%n", r.name(), r.seconds(),
          r.needsExtend() ? String.format("%.2f", r.extendSeconds()) : "-", verdict);
      if (!verdict.equals("ok")) {
        failures.append("\n  ").append(r.name()).append(": ").append(verdict);
      }
    }
    System.out.println();

    if (failures.length() > 0) {
      fail("Autos with problems:" + failures);
    }
  }

  Result runAuto(String name) throws IOException, InterruptedException {
    // Disabled, everything stopped, intake retracted and homed.
    setEnabled(false);
    runFor(0.2);
    m_container.simResetForAuto();
    runFor(0.3);

    boolean needsExtend = autoFileText(name).contains("\"extendIntakeAndWait\"");
    Command auto = AutoBuilder.buildAuto(name);

    setEnabled(true);
    runFor(0.2); // let the simulated devices see the enable
    CommandScheduler.getInstance().schedule(auto);
    double start = Timer.getFPGATimestamp();

    IntakeSubsystem intake = m_container.intake();
    double extendSeconds = 0;
    boolean reachedExtended = false;
    while (auto.isScheduled() && Timer.getFPGATimestamp() - start < kGiveUpSeconds) {
      tick();
      if (intake.getExtensionState() == State.EXTENDING) {
        extendSeconds += kLoopSeconds;
      }
      reachedExtended |= intake.isExtended();
    }
    boolean finished = !auto.isScheduled();
    double seconds = Timer.getFPGATimestamp() - start;
    if (!finished) {
      CommandScheduler.getInstance().cancel(auto);
    }
    setEnabled(false);
    return new Result(name, seconds, finished, extendSeconds, needsExtend, reachedExtended);
  }

  void setEnabled(boolean enabled) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  void runFor(double seconds) throws InterruptedException {
    for (int i = 0; i < seconds / kLoopSeconds; i++) {
      tick();
    }
  }

  /**
   * One 10 ms step, kept in step with the real clock: the intake sync check every step, and the
   * normal 20 ms robot loop (driver station data, commands, subsystems) every other step. This is
   * what {@code Robot} does on the real robot.
   */
  void tick() throws InterruptedException {
    m_nextLoopTime += kLoopSeconds;
    double wait = m_nextLoopTime - Timer.getFPGATimestamp();
    if (wait > 0) {
      Thread.sleep((long) (wait * 1000), (int) ((wait * 1e9) % 1_000_000));
    } else if (wait < -0.1) {
      m_nextLoopTime = Timer.getFPGATimestamp(); // fell behind; don't try to catch up in a burst
    }
    m_container.intakeSyncLoop();
    if (m_loops++ % 2 == 0) {
      DriverStation.refreshData();
      CommandScheduler.getInstance().run();
    }
  }

  static String autoFileText(String name) throws IOException {
    Path file = Filesystem.getDeployDirectory().toPath()
        .resolve("pathplanner").resolve("autos").resolve(name + ".auto");
    return Files.readString(file);
  }
}
