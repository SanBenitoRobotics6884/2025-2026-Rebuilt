// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/**
 * Decides what the intake extension is doing, and where each lead screw should be.
 *
 * <p>This class has no hardware in it. {@link IntakeSubsystem} reads both screw encoders and the
 * two CANdi limit sensors every 10 ms, hands them to {@link #update}, and drives the screws to
 * {@link #leftTarget()} / {@link #rightTarget()} while {@link #isDriving()} is true. The screws
 * are kept turning together by {@link ScrewSync}.
 *
 * <ul>
 *   <li>{@link State#REST}: screws stopped. This is where every reboot starts.
 *   <li>{@link State#EXTENDING}: both screws move out together until S2 (extended) triggers.
 *   <li>{@link State#RETRACTING}: both screws move in together until S1 (retracted) triggers.
 *   <li>{@link State#REALIGNING}: one screw moves alone to line up with the other, then the
 *       requested move starts.
 *   <li>{@link State#FAULT}: a screw jammed or the two got out of step. Both are stopped. The next
 *       extend or retract request re-aligns them first.
 * </ul>
 */
public final class ExtensionStateMachine {
  public enum State {
    REST,
    EXTENDING,
    RETRACTING,
    REALIGNING,
    FAULT
  }

  private final ScrewSync m_sync;
  private final double m_timeoutSeconds;

  private State m_state = State.REST;
  /** Direction to move once re-aligned: +1 extend, -1 retract. */
  private int m_pendingDirection;
  /** Direction of the last move, used to decide which screw is "ahead" after a fault. */
  private int m_lastDirection = 1;
  private double m_moveStartSeconds;
  private boolean m_timedOut;
  private boolean m_softLimitReached;

  /** @param timeoutSeconds last-resort cap on one move, in case every other check is fooled */
  public ExtensionStateMachine(ScrewSync.Config config, double timeoutSeconds) {
    m_sync = new ScrewSync(config);
    m_timeoutSeconds = timeoutSeconds;
  }

  public State state() {
    return m_state;
  }

  /** True when the screws should be position-controlled to the targets this loop. */
  public boolean isDriving() {
    return m_state == State.EXTENDING
        || m_state == State.RETRACTING
        || m_state == State.REALIGNING;
  }

  public double leftTarget() {
    return m_sync.leftTarget();
  }

  public double rightTarget() {
    return m_sync.rightTarget();
  }

  public double leftVelocity() {
    return m_sync.leftVelocity();
  }

  public double rightVelocity() {
    return m_sync.rightVelocity();
  }

  public ScrewSync.Fault fault() {
    return m_sync.fault();
  }

  public String faultDetail() {
    return m_sync.faultDetail();
  }

  /** True when the last move ended on the overall timeout instead of a sensor. */
  public boolean lastMoveTimedOut() {
    return m_timedOut;
  }

  /** True when the last move stopped at the travel limit without its sensor triggering. */
  public boolean lastMoveHitSoftLimit() {
    return m_softLimitReached;
  }

  /** Start moving out. Does nothing when S2 says it is already extended (unless faulted). */
  public void requestExtend(
      double leftPos, double rightPos, boolean extendedLimit, double nowSeconds) {
    request(1, leftPos, rightPos, extendedLimit, nowSeconds);
  }

  /** Start moving in. Does nothing when S1 says it is already retracted (unless faulted). */
  public void requestRetract(
      double leftPos, double rightPos, boolean retractedLimit, double nowSeconds) {
    request(-1, leftPos, rightPos, retractedLimit, nowSeconds);
  }

  /** Stop where the screws are. A fault stays a fault, so the next move still re-aligns first. */
  public void stop() {
    m_sync.stop();
    if (m_state != State.FAULT) {
      m_state = State.REST;
    }
  }

  /**
   * Run one check.
   *
   * @param leftPos left screw encoder, rotations, positive = out
   * @param rightPos right screw encoder, rotations, positive = out
   * @param retractedLimit S1 is triggered
   * @param extendedLimit S2 is triggered
   * @param nowSeconds current time
   * @return the state for this loop
   */
  public State update(
      double leftPos,
      double rightPos,
      boolean retractedLimit,
      boolean extendedLimit,
      double nowSeconds) {
    if (!isDriving()) {
      return m_state;
    }

    // The sensors end a move before anything else.
    if ((m_state == State.EXTENDING && extendedLimit)
        || (m_state == State.RETRACTING && retractedLimit)) {
      m_sync.stop();
      m_state = State.REST;
      return m_state;
    }

    ScrewSync.Result result = m_sync.step(leftPos, rightPos, nowSeconds);
    if (result == ScrewSync.Result.FAULT) {
      m_state = State.FAULT;
      return m_state;
    }

    if (nowSeconds - m_moveStartSeconds > m_timeoutSeconds) {
      m_sync.stop();
      m_timedOut = true;
      m_state = State.REST;
      return m_state;
    }

    if (result == ScrewSync.Result.DONE) {
      if (m_state == State.REALIGNING) {
        boolean alreadyThere = m_pendingDirection > 0 ? extendedLimit : retractedLimit;
        if (alreadyThere) {
          m_state = State.REST;
        } else {
          startSynced(m_pendingDirection, leftPos, rightPos, nowSeconds);
        }
      } else {
        m_softLimitReached = true;
        m_state = State.REST;
      }
    }
    return m_state;
  }

  private void request(
      int direction, double leftPos, double rightPos, boolean alreadyThere, double nowSeconds) {
    State target = direction > 0 ? State.EXTENDING : State.RETRACTING;
    if (m_state == target) {
      return;
    }
    m_timedOut = false;
    m_softLimitReached = false;

    boolean outOfLine = !m_sync.inLine(leftPos, rightPos);
    if (m_state == State.FAULT || outOfLine) {
      m_pendingDirection = direction;
      m_moveStartSeconds = nowSeconds;
      m_sync.startRealign(leftPos, rightPos, m_lastDirection, nowSeconds);
      m_state = State.REALIGNING;
      return;
    }
    if (m_state == State.REALIGNING) {
      // Finish lining up first, then go the newly requested way.
      m_pendingDirection = direction;
      return;
    }
    if (alreadyThere) {
      m_sync.stop();
      m_state = State.REST;
      return;
    }
    startSynced(direction, leftPos, rightPos, nowSeconds);
  }

  private void startSynced(int direction, double leftPos, double rightPos, double nowSeconds) {
    m_lastDirection = direction;
    m_moveStartSeconds = nowSeconds;
    m_sync.startSynced(leftPos, rightPos, direction, nowSeconds);
    m_state = direction > 0 ? State.EXTENDING : State.RETRACTING;
  }
}
