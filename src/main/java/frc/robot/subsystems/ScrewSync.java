// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/**
 * Keeps the two intake lead screws turning together, turn for turn.
 *
 * <p>Both screws are position-controlled to targets that this class moves a small step at a time.
 * Before every step it checks each screw's own encoder. The target only moves on once BOTH screws
 * have actually reached it, so the faster screw waits for the slower one. If a screw falls too
 * far behind, the two drift too far apart, or nothing moves for too long, {@link #step} reports a
 * {@link Fault} and the caller stops both motors.
 *
 * <p>Positions are motor rotations, positive = outward, and both encoders read 0 when retracted.
 *
 * <p>Two modes:
 *
 * <ul>
 *   <li>{@link Mode#SYNCED}: both screws follow one shared target in one direction.
 *   <li>{@link Mode#REALIGN}: one screw is held still while the other moves alone to match it.
 *       Used after a fault, or before a move when the screws start out of line.
 * </ul>
 *
 * <p>No hardware here, so unit tests can feed it scripted encoder readings.
 */
public final class ScrewSync {
  public enum Mode {
    IDLE,
    SYNCED,
    REALIGN
  }

  public enum Result {
    /** Still moving. */
    RUNNING,
    /** Realign finished, or a synced move reached the soft travel limit. */
    DONE,
    /** Stop both screws now. See {@link #fault()}. */
    FAULT
  }

  public enum Fault {
    NONE,
    /** The two screws are further apart than the sync limit. */
    SCREWS_APART,
    /** The left screw is too far from where it was told to be. */
    LEFT_OFF_TARGET,
    /** The right screw is too far from where it was told to be. */
    RIGHT_OFF_TARGET,
    /** The target could not advance for too long: something is holding the screws. */
    NO_PROGRESS
  }

  /** Tuning numbers. All distances are motor rotations. */
  public static final class Config {
    final double speedRps;
    final double followTolerance;
    final double syncFault;
    final double trackingFault;
    final double debounceSeconds;
    final double noProgressSeconds;
    final double minTarget;
    final double maxTarget;

    /**
     * @param speedRps how fast the target moves while both screws keep up
     * @param followTolerance both screws must be this close to the target before it moves on
     * @param syncFault fault when the screws are further apart than this
     * @param trackingFault fault when a screw is further than this from its target
     * @param debounceSeconds a fault condition must last this long, so one noisy reading can't trip it
     * @param noProgressSeconds fault when the target has not moved for this long
     * @param minTarget lowest target ever commanded (a little past retracted)
     * @param maxTarget highest target ever commanded (a little past extended)
     */
    public Config(
        double speedRps,
        double followTolerance,
        double syncFault,
        double trackingFault,
        double debounceSeconds,
        double noProgressSeconds,
        double minTarget,
        double maxTarget) {
      this.speedRps = speedRps;
      this.followTolerance = followTolerance;
      this.syncFault = syncFault;
      this.trackingFault = trackingFault;
      this.debounceSeconds = debounceSeconds;
      this.noProgressSeconds = noProgressSeconds;
      this.minTarget = minTarget;
      this.maxTarget = maxTarget;
    }
  }

  /** Never step further than this many seconds' worth at once, e.g. after a loop overrun. */
  private static final double kMaxStepSeconds = 0.05;

  private final Config m_config;

  private Mode m_mode = Mode.IDLE;
  private Fault m_fault = Fault.NONE;
  private String m_faultDetail = "";

  private double m_leftTarget;
  private double m_rightTarget;
  private double m_leftVelocity;
  private double m_rightVelocity;

  /** +1 out, -1 in. In REALIGN, the direction the moving screw travels. */
  private int m_direction;
  /** REALIGN only: true when the left screw moves and the right is held. */
  private boolean m_realignMovesLeft;
  /** REALIGN only: where the moving screw is headed (the held screw's position). */
  private double m_realignGoal;

  private double m_lastTime = Double.NaN;
  private double m_lastAdvanceTime;
  private double m_badSince = Double.NaN;

  public ScrewSync(Config config) {
    m_config = config;
  }

  public Mode mode() {
    return m_mode;
  }

  public Fault fault() {
    return m_fault;
  }

  /** The numbers behind the last fault, for the Driver Station message. */
  public String faultDetail() {
    return m_faultDetail;
  }

  public double leftTarget() {
    return m_leftTarget;
  }

  public double rightTarget() {
    return m_rightTarget;
  }

  /** Feedforward for the left screw this step, in rotations per second. */
  public double leftVelocity() {
    return m_leftVelocity;
  }

  /** Feedforward for the right screw this step, in rotations per second. */
  public double rightVelocity() {
    return m_rightVelocity;
  }

  /** True when the screws are within the follow tolerance of each other. */
  public boolean inLine(double leftPos, double rightPos) {
    return Math.abs(leftPos - rightPos) <= m_config.followTolerance;
  }

  /**
   * Start (or reverse) a synced move. The shared target starts at the screw that is further
   * behind in the new direction, so neither screw is ever pushed ahead of the other.
   *
   * @param direction +1 extend, -1 retract
   */
  public void startSynced(double leftPos, double rightPos, int direction, double now) {
    m_direction = direction > 0 ? 1 : -1;
    double start = m_direction > 0 ? Math.min(leftPos, rightPos) : Math.max(leftPos, rightPos);
    m_leftTarget = start;
    m_rightTarget = start;
    begin(Mode.SYNCED, now);
  }

  /**
   * Start re-aligning. The screw that is ahead in {@code aheadDirection} is moved back alone to
   * the other screw's position. The other screw is held where it is, because it may be the one
   * that jammed.
   *
   * @param aheadDirection +1 if the failed move was extending, -1 if it was retracting
   */
  public void startRealign(double leftPos, double rightPos, int aheadDirection, double now) {
    int ahead = aheadDirection >= 0 ? 1 : -1;
    // "Ahead" when extending means further out; when retracting, further in.
    m_realignMovesLeft = ahead * (leftPos - rightPos) >= 0;
    double moving = m_realignMovesLeft ? leftPos : rightPos;
    m_realignGoal = m_realignMovesLeft ? rightPos : leftPos;
    m_direction = m_realignGoal >= moving ? 1 : -1;
    m_leftTarget = leftPos;
    m_rightTarget = rightPos;
    begin(Mode.REALIGN, now);
  }

  /** Stop following without a fault. */
  public void stop() {
    m_mode = Mode.IDLE;
    m_leftVelocity = 0;
    m_rightVelocity = 0;
  }

  /**
   * One check. Call every loop (10 ms) with fresh encoder readings.
   *
   * @return what the caller should do with the motors
   */
  public Result step(double leftPos, double rightPos, double now) {
    if (m_mode == Mode.IDLE) {
      return Result.DONE;
    }
    double dt = Double.isNaN(m_lastTime) ? 0 : Math.max(0, Math.min(now - m_lastTime, kMaxStepSeconds));
    m_lastTime = now;
    m_leftVelocity = 0;
    m_rightVelocity = 0;

    // 1. Verify: did each screw actually get to where it was told?
    double leftErr = leftPos - m_leftTarget;
    double rightErr = rightPos - m_rightTarget;
    double apart = leftPos - rightPos;

    Fault bad = Fault.NONE;
    if (m_mode == Mode.SYNCED && Math.abs(apart) > m_config.syncFault) {
      bad = Fault.SCREWS_APART;
    } else if (Math.abs(leftErr) > m_config.trackingFault) {
      bad = Fault.LEFT_OFF_TARGET;
    } else if (Math.abs(rightErr) > m_config.trackingFault) {
      bad = Fault.RIGHT_OFF_TARGET;
    } else if (now - m_lastAdvanceTime > m_config.noProgressSeconds) {
      bad = Fault.NO_PROGRESS;
    }

    if (bad != Fault.NONE) {
      if (Double.isNaN(m_badSince)) {
        m_badSince = now;
      }
      if (now - m_badSince >= m_config.debounceSeconds) {
        m_fault = bad;
        m_faultDetail =
            String.format(
                "left %.2f (target %.2f), right %.2f (target %.2f), apart %.2f rotations",
                leftPos, m_leftTarget, rightPos, m_rightTarget, apart);
        m_mode = Mode.IDLE;
        return Result.FAULT;
      }
    } else {
      m_badSince = Double.NaN;
    }

    boolean leftArrived = Math.abs(leftErr) <= m_config.followTolerance;
    boolean rightArrived = Math.abs(rightErr) <= m_config.followTolerance;

    if (m_mode == Mode.REALIGN) {
      return stepRealign(leftPos, rightPos, leftArrived, rightArrived, dt, now);
    }

    // 2. Advance only when both screws are on target. Otherwise hold so the slow one catches up.
    double limit = m_direction > 0 ? m_config.maxTarget : m_config.minTarget;
    boolean atLimit = m_direction > 0 ? m_leftTarget >= limit : m_leftTarget <= limit;
    if (atLimit) {
      return leftArrived && rightArrived ? Result.DONE : Result.RUNNING;
    }
    if (leftArrived && rightArrived && dt > 0) {
      double next = m_leftTarget + m_direction * m_config.speedRps * dt;
      next = m_direction > 0 ? Math.min(next, limit) : Math.max(next, limit);
      m_leftTarget = next;
      m_rightTarget = next;
      m_leftVelocity = m_direction * m_config.speedRps;
      m_rightVelocity = m_leftVelocity;
      m_lastAdvanceTime = now;
    }
    return Result.RUNNING;
  }

  private Result stepRealign(
      double leftPos, double rightPos, boolean leftArrived, boolean rightArrived, double dt, double now) {
    double movingPos = m_realignMovesLeft ? leftPos : rightPos;
    double movingTarget = m_realignMovesLeft ? m_leftTarget : m_rightTarget;
    boolean movingArrived = m_realignMovesLeft ? leftArrived : rightArrived;

    if (Math.abs(movingPos - m_realignGoal) <= m_config.followTolerance
        && Math.abs(movingTarget - m_realignGoal) < 1e-9) {
      m_mode = Mode.IDLE;
      return Result.DONE;
    }
    if (movingArrived && dt > 0) {
      double next = movingTarget + m_direction * m_config.speedRps * dt;
      next = m_direction > 0 ? Math.min(next, m_realignGoal) : Math.max(next, m_realignGoal);
      double velocity = m_direction * m_config.speedRps;
      if (m_realignMovesLeft) {
        m_leftTarget = next;
        m_leftVelocity = velocity;
      } else {
        m_rightTarget = next;
        m_rightVelocity = velocity;
      }
      m_lastAdvanceTime = now;
    }
    return Result.RUNNING;
  }

  private void begin(Mode mode, double now) {
    m_mode = mode;
    m_fault = Fault.NONE;
    m_faultDetail = "";
    m_lastTime = now;
    m_lastAdvanceTime = now;
    m_badSince = Double.NaN;
    m_leftVelocity = 0;
    m_rightVelocity = 0;
  }
}
