// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.ExtensionStateMachine.State;

/**
 * Checks the rest / extending / retracting / realigning / fault rules with two fake screws. The
 * fake intake's S1 triggers at 0 rotations and S2 at {@link #kTravel}.
 */
class ExtensionStateMachineTest {
  static final double kDt = 0.01;
  static final double kTravel = 40.0;
  static final double kUnhomedStep = 7.0;

  final ExtensionStateMachine m_machine =
      new ExtensionStateMachine(ScrewSyncTest.config(), 15.0, kUnhomedStep);
  double m_left;
  double m_right;
  double m_leftSpeed = 20.0;
  double m_rightSpeed = 20.0;
  double m_now;

  boolean retracted() {
    return Math.min(m_left, m_right) <= 0.05;
  }

  boolean extended() {
    return Math.max(m_left, m_right) >= kTravel - 0.05;
  }

  State tick() {
    m_now += kDt;
    State state = m_machine.update(m_left, m_right, retracted(), extended(), m_now);
    if (m_machine.isDriving()) {
      m_left = ScrewSyncTest.Plant.approach(m_left, m_machine.leftTarget(), m_leftSpeed * kDt);
      m_right = ScrewSyncTest.Plant.approach(m_right, m_machine.rightTarget(), m_rightSpeed * kDt);
    }
    return state;
  }

  State runUntilStopped(double seconds) {
    State state = m_machine.state();
    for (int i = 0; i < seconds / kDt; i++) {
      state = tick();
      if (!m_machine.isDriving()) {
        break;
      }
    }
    return state;
  }

  void extend() {
    m_machine.requestExtend(m_left, m_right, extended(), m_now);
  }

  void retract() {
    m_machine.requestRetract(m_left, m_right, retracted(), m_now);
  }

  @Test
  void startsAtRest() {
    assertEquals(State.REST, m_machine.state());
    assertEquals(State.REST, tick());
  }

  @Test
  void extendRunsUntilExtendedSensorThenRetractUntilRetracted() {
    extend();
    assertEquals(State.EXTENDING, tick());
    assertEquals(State.REST, runUntilStopped(10));
    assertTrue(extended());
    assertFalse(m_machine.lastMoveHitSoftLimit());

    retract();
    assertEquals(State.RETRACTING, tick());
    assertEquals(State.REST, runUntilStopped(10));
    assertTrue(retracted());
  }

  @Test
  void requestAtTheMatchingLimitStaysAtRest() {
    retract(); // already at S1
    assertEquals(State.REST, tick());
  }

  @Test
  void jamStopsBothAndTheNextPressRealignsThenMoves() {
    extend();
    for (int i = 0; i < 100; i++) {
      tick();
    }
    m_rightSpeed = 0; // right jams
    assertEquals(State.FAULT, runUntilStopped(2));
    double leftAtFault = m_left;
    for (int i = 0; i < 50; i++) {
      tick();
    }
    assertEquals(leftAtFault, m_left, 1e-9); // stays stopped while faulted

    m_rightSpeed = 20; // jam cleared
    retract();
    assertEquals(State.REALIGNING, m_machine.state());
    // Realign brings left back to right, then the retract runs to S1.
    assertEquals(State.REST, runUntilStopped(10));
    assertTrue(retracted());
    assertEquals(m_left, m_right, 0.25);
  }

  @Test
  void stopKeepsAFaultLatched() {
    extend();
    for (int i = 0; i < 50; i++) {
      tick();
    }
    m_leftSpeed = 0;
    assertEquals(State.FAULT, runUntilStopped(2));
    m_machine.stop();
    assertEquals(State.FAULT, m_machine.state());
  }

  @Test
  void screwsStartingOutOfLineRealignFirst() {
    m_left = 3.0;
    m_right = 1.0;
    extend();
    assertEquals(State.REALIGNING, m_machine.state());
    assertEquals(State.REST, runUntilStopped(15));
    assertTrue(extended());
    assertEquals(m_left, m_right, 0.25);
  }

  @Test
  void oppositeRequestReversesTheMove() {
    extend();
    for (int i = 0; i < 100; i++) {
      tick();
    }
    retract();
    assertEquals(State.RETRACTING, tick());
    assertEquals(State.REST, runUntilStopped(10));
    assertTrue(retracted());
  }

  @Test
  void stopGoesToRest() {
    extend();
    tick();
    m_machine.stop();
    assertEquals(State.REST, tick());
  }

  /** Powered up away from S1: the encoders read {@code position}, but the robot does not trust it. */
  void powerUpNotHomedAt(double position) {
    m_left = position;
    m_right = position;
    m_machine.setHomed(false);
  }

  @Test
  void notHomedRetractStopsAfterOneStepWithoutASoftLimitWarning() {
    powerUpNotHomedAt(20.0);
    retract();
    assertEquals(State.RETRACTING, tick());
    assertEquals(State.REST, runUntilStopped(5));
    assertEquals(20.0 - kUnhomedStep, m_left, 0.3);
    assertEquals(20.0 - kUnhomedStep, m_right, 0.3);
    assertFalse(m_machine.lastMoveHitSoftLimit());
    assertFalse(m_machine.isHomed());
  }

  @Test
  void eachRetractPressWalksAnotherStepUntilS1() {
    powerUpNotHomedAt(20.0);
    retract();
    runUntilStopped(5);
    retract();
    runUntilStopped(5);
    assertEquals(20.0 - 2 * kUnhomedStep, m_left, 0.5);

    retract(); // 6 rotations left: S1 ends this step early
    assertEquals(State.REST, runUntilStopped(5));
    assertTrue(retracted());
    assertFalse(m_machine.lastMoveHitSoftLimit());
  }

  @Test
  void notHomedRefusesExtend() {
    powerUpNotHomedAt(20.0);
    extend();
    assertEquals(State.REST, m_machine.state());
    assertTrue(m_machine.takeExtendRefused());
    assertFalse(m_machine.takeExtendRefused()); // reported once
    assertEquals(State.REST, tick());
    assertEquals(20.0, m_left, 1e-9);
  }

  @Test
  void deadS1SensorNeverMovesMoreThanOneStepPerPress() {
    powerUpNotHomedAt(3.0); // S1 would be at 0, but pretend it never triggers
    m_machine.requestRetract(m_left, m_right, false, m_now);
    for (int i = 0; i < 500 && m_machine.isDriving(); i++) {
      m_now += kDt;
      m_machine.update(m_left, m_right, false, false, m_now);
      m_left = ScrewSyncTest.Plant.approach(m_left, m_machine.leftTarget(), m_leftSpeed * kDt);
      m_right = ScrewSyncTest.Plant.approach(m_right, m_machine.rightTarget(), m_rightSpeed * kDt);
    }
    assertEquals(State.REST, m_machine.state());
    assertEquals(3.0 - kUnhomedStep, m_left, 0.3);
  }

  @Test
  void homingRestoresNormalLimitsAndExtend() {
    powerUpNotHomedAt(5.0);
    retract();
    assertEquals(State.REST, runUntilStopped(5));
    assertTrue(retracted());
    m_machine.setHomed(true); // IntakeSubsystem does this after zeroing at S1

    extend();
    assertEquals(State.EXTENDING, tick());
    assertEquals(State.REST, runUntilStopped(10));
    assertTrue(extended());
  }

  @Test
  void notHomedJamStillFaults() {
    powerUpNotHomedAt(20.0);
    retract();
    for (int i = 0; i < 10; i++) {
      tick();
    }
    m_leftSpeed = 0;
    assertEquals(State.FAULT, runUntilStopped(2));
  }
}
