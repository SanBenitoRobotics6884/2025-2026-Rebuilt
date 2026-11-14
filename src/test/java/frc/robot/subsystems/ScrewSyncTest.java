// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.ScrewSync.Fault;
import frc.robot.subsystems.ScrewSync.Mode;
import frc.robot.subsystems.ScrewSync.Result;

/**
 * Feeds the sync check scripted encoder readings. Each fake screw moves toward its target at up to
 * {@code maxSpeed} rotations per second, unless it is jammed.
 */
class ScrewSyncTest {
  static final double kDt = 0.01;

  static ScrewSync.Config config() {
    return new ScrewSync.Config(10.0, 0.25, 1.0, 1.5, 0.03, 0.5, -5.0, 75.0);
  }

  /** Two fake screws driven by a ScrewSync. */
  static final class Plant {
    final ScrewSync sync = new ScrewSync(config());
    double left;
    double right;
    double leftMaxSpeed = 20.0;
    double rightMaxSpeed = 20.0;
    double now;

    Result tick() {
      now += kDt;
      Result result = sync.step(left, right, now);
      if (sync.mode() != Mode.IDLE) {
        left = approach(left, sync.leftTarget(), leftMaxSpeed * kDt);
        right = approach(right, sync.rightTarget(), rightMaxSpeed * kDt);
      }
      return result;
    }

    Result runUntilNotRunning(double seconds) {
      Result result = Result.RUNNING;
      for (int i = 0; i < seconds / kDt && result == Result.RUNNING; i++) {
        result = tick();
      }
      return result;
    }

    /** Runs for a while, expecting no fault and no finish. */
    void runFor(double seconds) {
      for (int i = 0; i < seconds / kDt; i++) {
        assertEquals(Result.RUNNING, tick());
      }
    }

    static double approach(double pos, double target, double maxStep) {
      double delta = Math.max(-maxStep, Math.min(maxStep, target - pos));
      return pos + delta;
    }
  }

  @Test
  void bothScrewsTrackAndReachTravelLimit() {
    Plant p = new Plant();
    p.sync.startSynced(0, 0, 1, 0);
    assertEquals(Result.DONE, p.runUntilNotRunning(20));
    assertEquals(75.0, p.left, 0.3);
    assertEquals(75.0, p.right, 0.3);
  }

  @Test
  void staysWithinToleranceAndOneStepWhileOneIsSlow() {
    Plant p = new Plant();
    p.rightMaxSpeed = 6.0; // slower than the 10 rot/s target speed
    p.sync.startSynced(0, 0, 1, 0);
    double worst = 0;
    for (int i = 0; i < 300; i++) {
      assertEquals(Result.RUNNING, p.tick());
      worst = Math.max(worst, Math.abs(p.left - p.right));
    }
    // The target only moves once both are within 0.25 of it, then moves one 10 ms step (0.1).
    assertTrue(worst <= 0.25 + 0.1 + 1e-9, "screws drifted " + worst);
    assertTrue(p.left > 10, "should still make progress, at the slow screw's speed");
  }

  @Test
  void jammedScrewStopsBoth() {
    Plant p = new Plant();
    p.sync.startSynced(0, 0, 1, 0);
    p.runFor(1.0);
    double rightAtJam = p.right;
    p.rightMaxSpeed = 0; // right jams
    Result result = p.runUntilNotRunning(2.0);
    assertEquals(Result.FAULT, result);
    assertEquals(Mode.IDLE, p.sync.mode());
    // The free screw went no further than the follow tolerance past the jammed one.
    assertTrue(p.left - rightAtJam <= 0.25 + 0.2 + 1e-9, "left ran on to " + p.left);
  }

  @Test
  void screwsPushedApartFault() {
    Plant p = new Plant();
    p.sync.startSynced(0, 0, 1, 0);
    p.runFor(0.5);
    p.left += 2.0; // something shoves one side
    assertEquals(Result.FAULT, p.runUntilNotRunning(0.1));
    assertEquals(Fault.SCREWS_APART, p.sync.fault());
  }

  @Test
  void bothStuckIsNoProgress() {
    Plant p = new Plant();
    p.sync.startSynced(0, 0, 1, 0);
    p.runFor(0.5);
    p.leftMaxSpeed = 0;
    p.rightMaxSpeed = 0;
    assertEquals(Result.FAULT, p.runUntilNotRunning(2.0));
    assertTrue(
        p.sync.fault() == Fault.NO_PROGRESS
            || p.sync.fault() == Fault.LEFT_OFF_TARGET
            || p.sync.fault() == Fault.RIGHT_OFF_TARGET);
  }

  @Test
  void oneNoisyReadingDoesNotFault() {
    ScrewSync sync = new ScrewSync(config());
    sync.startSynced(0, 0, 1, 0);
    assertEquals(Result.RUNNING, sync.step(0, 0, 0.01));
    assertEquals(Result.RUNNING, sync.step(5.0, 0, 0.02)); // one bad frame
    assertEquals(Result.RUNNING, sync.step(0.1, 0.1, 0.03));
  }

  @Test
  void realignMovesOnlyTheScrewThatIsAhead() {
    Plant p = new Plant();
    p.left = 30.0;
    p.right = 28.0; // right jammed during an extend, left got ahead
    p.sync.startRealign(p.left, p.right, 1, 0);
    p.now = 0;
    assertEquals(Result.DONE, p.runUntilNotRunning(2.0));
    assertEquals(28.0, p.right, 1e-9); // held still the whole time
    assertEquals(28.0, p.left, 0.25);
  }

  @Test
  void realignWhileRetractingMovesTheInnerScrewOut() {
    Plant p = new Plant();
    p.left = 10.0;
    p.right = 12.0; // left got further in during a retract
    p.sync.startRealign(p.left, p.right, -1, 0);
    assertEquals(Result.DONE, p.runUntilNotRunning(2.0));
    assertEquals(12.0, p.right, 1e-9);
    assertEquals(12.0, p.left, 0.25);
  }

  @Test
  void realignFaultsIfTheMovingScrewIsStuck() {
    Plant p = new Plant();
    p.left = 30.0;
    p.right = 28.0;
    p.leftMaxSpeed = 0;
    p.sync.startRealign(p.left, p.right, 1, 0);
    assertEquals(Result.FAULT, p.runUntilNotRunning(2.0));
  }
}
