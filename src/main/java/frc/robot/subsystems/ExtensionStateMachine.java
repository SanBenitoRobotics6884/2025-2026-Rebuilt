// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/**
 * Decides whether the intake extension should be resting, moving out, or moving in.
 *
 * <p>This class has no hardware in it. {@link IntakeSubsystem} reads the two CANdi limit sensors,
 * hands them to {@link #update} every 20 ms robot loop, and drives the screws from the state that
 * comes back. Keeping the rules here means unit tests can check them without a robot.
 *
 * <ul>
 *   <li>{@link State#REST}: screws stopped. This is where every reboot starts.
 *   <li>{@link State#EXTENDING}: drive out until the extended sensor (S2) triggers.
 *   <li>{@link State#RETRACTING}: drive in until the retracted sensor (S1) triggers.
 * </ul>
 *
 * <p>A move that runs longer than the timeout also goes back to {@link State#REST}, so a broken
 * or unplugged sensor cannot leave the screws pushing against the end of travel.
 */
public final class ExtensionStateMachine {
  public enum State {
    REST,
    EXTENDING,
    RETRACTING
  }

  private final double m_timeoutSeconds;

  private State m_state = State.REST;
  private double m_moveStartSeconds;
  private boolean m_timedOut;

  /** @param timeoutSeconds longest a single move may run before it gives up and rests */
  public ExtensionStateMachine(double timeoutSeconds) {
    m_timeoutSeconds = timeoutSeconds;
  }

  public State state() {
    return m_state;
  }

  /** True when the last move ended because it ran out of time instead of reaching its sensor. */
  public boolean lastMoveTimedOut() {
    return m_timedOut;
  }

  /**
   * Start moving out. Ignored when the extended sensor is already triggered.
   *
   * @param extendedLimit S2 is triggered right now
   * @param nowSeconds current time, used for the timeout
   */
  public void requestExtend(boolean extendedLimit, double nowSeconds) {
    start(State.EXTENDING, extendedLimit, nowSeconds);
  }

  /**
   * Start moving in. Ignored when the retracted sensor is already triggered.
   *
   * @param retractedLimit S1 is triggered right now
   * @param nowSeconds current time, used for the timeout
   */
  public void requestRetract(boolean retractedLimit, double nowSeconds) {
    start(State.RETRACTING, retractedLimit, nowSeconds);
  }

  /** Stop wherever the screws are. */
  public void stop() {
    m_state = State.REST;
  }

  /**
   * Run one robot loop.
   *
   * @param retractedLimit S1 is triggered (fully retracted)
   * @param extendedLimit S2 is triggered (fully extended)
   * @param nowSeconds current time
   * @return the state to drive the screws with this loop
   */
  public State update(boolean retractedLimit, boolean extendedLimit, double nowSeconds) {
    switch (m_state) {
      case EXTENDING:
        if (extendedLimit) {
          m_state = State.REST;
        } else if (isPastTimeout(nowSeconds)) {
          m_timedOut = true;
          m_state = State.REST;
        }
        break;
      case RETRACTING:
        if (retractedLimit) {
          m_state = State.REST;
        } else if (isPastTimeout(nowSeconds)) {
          m_timedOut = true;
          m_state = State.REST;
        }
        break;
      case REST:
      default:
        break;
    }
    return m_state;
  }

  private void start(State target, boolean alreadyThere, double nowSeconds) {
    m_timedOut = false;
    if (alreadyThere) {
      m_state = State.REST;
      return;
    }
    // Switching direction mid-move restarts the timeout for the new direction.
    if (m_state != target) {
      m_moveStartSeconds = nowSeconds;
    }
    m_state = target;
  }

  private boolean isPastTimeout(double nowSeconds) {
    return nowSeconds - m_moveStartSeconds > m_timeoutSeconds;
  }
}
