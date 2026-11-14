// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.ExtensionStateMachine.State;

/** Checks the rest / extending / retracting rules without a robot. */
class ExtensionStateMachineTest {
  private static final double kTimeout = 5.0;

  private final ExtensionStateMachine m_machine = new ExtensionStateMachine(kTimeout);

  @Test
  void startsAtRest() {
    assertEquals(State.REST, m_machine.state());
    assertEquals(State.REST, m_machine.update(false, false, 0.0));
  }

  @Test
  void extendRunsUntilExtendedSensor() {
    m_machine.requestExtend(false, 0.0);
    assertEquals(State.EXTENDING, m_machine.update(true, false, 0.02));
    assertEquals(State.EXTENDING, m_machine.update(false, false, 1.0));
    assertEquals(State.REST, m_machine.update(false, true, 1.02));
    assertFalse(m_machine.lastMoveTimedOut());
  }

  @Test
  void retractRunsUntilRetractedSensor() {
    m_machine.requestRetract(false, 0.0);
    assertEquals(State.RETRACTING, m_machine.update(false, true, 0.02));
    assertEquals(State.RETRACTING, m_machine.update(false, false, 1.0));
    assertEquals(State.REST, m_machine.update(true, false, 1.02));
  }

  @Test
  void requestAtTheMatchingLimitStaysAtRest() {
    m_machine.requestExtend(true, 0.0);
    assertEquals(State.REST, m_machine.update(false, true, 0.02));
    m_machine.requestRetract(true, 0.04);
    assertEquals(State.REST, m_machine.update(true, false, 0.06));
  }

  @Test
  void oppositeRequestSwitchesDirectionAndRestartsTimeout() {
    m_machine.requestExtend(false, 0.0);
    m_machine.update(false, false, 4.0);
    m_machine.requestRetract(false, 4.0);
    assertEquals(State.RETRACTING, m_machine.update(false, false, 8.0));
    assertEquals(State.REST, m_machine.update(true, false, 8.02));
  }

  @Test
  void repeatedRequestDoesNotRestartTimeout() {
    m_machine.requestExtend(false, 0.0);
    m_machine.requestExtend(false, 4.0);
    assertEquals(State.REST, m_machine.update(false, false, 5.1));
    assertTrue(m_machine.lastMoveTimedOut());
  }

  @Test
  void missingSensorTimesOutToRest() {
    m_machine.requestRetract(false, 0.0);
    assertEquals(State.RETRACTING, m_machine.update(false, false, kTimeout));
    assertEquals(State.REST, m_machine.update(false, false, kTimeout + 0.02));
    assertTrue(m_machine.lastMoveTimedOut());
  }

  @Test
  void stopGoesToRest() {
    m_machine.requestExtend(false, 0.0);
    m_machine.stop();
    assertEquals(State.REST, m_machine.update(false, false, 0.02));
  }
}
