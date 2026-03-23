# TODO 2: Tune PID So the Mechanism Doesn't Slam Hard Stops

## Why This Matters

Right now, when you tell a motor to go to a position (like "go to rotation 50"), it gets there by going full speed and then suddenly stopping. That sudden stop slams the mechanism into its target — vibrating, overshooting, or banging into hard stops. Over time this breaks things.

The fix is tuning the **PID controller** — the math that controls how the motor gets to its target.

---

## What is PID? (The Short Version)

PID stands for **P**roportional, **I**ntegral, **D**erivative. These are three numbers that control how a motor moves to a target position.

Think of it like driving a car to a stop sign:

- **P (Proportional)** = "How hard do I press the gas based on how far away I am?"
  - Far from the stop sign → press hard. Close to the stop sign → press lighter.
  - **Problem:** With only P, you might overshoot — like rolling past the stop sign because you were going too fast when you got close.

- **D (Derivative)** = "Am I approaching too fast? Ease off!"
  - This is like your brain saying "I'm closing in fast, better start braking NOW"
  - D looks at how quickly the error is *changing*. If you're getting close fast, it pushes back to slow you down.
  - **This is what we need.** It's the "anti-slam" term.

- **I (Integral)** = "I've been slightly off for a while, let me push a little harder to fix it"
  - We are **NOT using this right now**. On position control with hard stops, I gain can cause the motor to build up force and slam. Leave it alone.

### What We Have Now

```java
slot0Configs.kP = 0.25;  // Proportional — already set
// kD is missing — no braking/damping at all!
```

With only P and no D, the motor charges toward the target and has no "brakes." Adding D gives it smooth deceleration.

---

## Step 1: Add PID_D_VALUE to Constants

Open `src/main/java/frc/robot/Constants/Constants.java`

Find the `Intake` class:

```java
public class Intake {
    public static final int LIMIT_SWITCH_STORAGE_DIO = 0;

    public static final double PID_P_VALUE = 0.25;
```

Add the D value right after P:

```java
public class Intake {
    public static final int LIMIT_SWITCH_STORAGE_DIO = 0;

    public static final double PID_P_VALUE = 0.25;
    public static final double PID_D_VALUE = 0.01;  // Start small, tune up
```

Now find the `Outtake` class:

```java
public class Outtake {

    public static final double DUTYCYCLE_OUTPUT = 0.5;

    public static final double OUTTAKE_SPEED = 0.98;
    public static final double INDEX_SPEED = 1.0;

    public static final double OUTTAKE_SPEED_SLOW = 0.5;
    public static final double INDEX_SPEED_SLOW = 0.5;

    public static final double PID_P_VALUE = 0.25;
    // public static final double PID_I_VALUE = 0.25;
    // public static final double PID_D_VALUE = 0.25;
```

Change it to:

```java
public class Outtake {

    public static final double DUTYCYCLE_OUTPUT = 0.5;

    public static final double OUTTAKE_SPEED = 0.98;
    public static final double INDEX_SPEED = 1.0;

    public static final double OUTTAKE_SPEED_SLOW = 0.5;
    public static final double INDEX_SPEED_SLOW = 0.5;

    public static final double PID_P_VALUE = 0.25;
    // public static final double PID_I_VALUE = 0.25;  // DO NOT uncomment — causes windup on hard stops
    public static final double PID_D_VALUE = 0.01;  // Start small, tune up
```

**Important:** Do NOT uncomment `PID_I_VALUE`. The old value of 0.25 is way too high and the I term can cause dangerous force buildup on mechanisms with hard stops.

---

## Step 2: Apply kD in IntakeSubsystem

Open `src/main/java/frc/robot/subsystems/IntakeSubsystem.java`

Find this block in the constructor:

```java
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE; // Tune this value (output per rotation of error)
    // Add kI, kD, kS, kV if needed for better control
    m_leftLinearScrew.getConfigurator().apply(slot0Configs);
    m_rightLinearScrew.getConfigurator().apply(slot0Configs);
```

Change it to:

```java
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE;
    slot0Configs.kD = PID_D_VALUE;
    m_leftLinearScrew.getConfigurator().apply(slot0Configs);
    m_rightLinearScrew.getConfigurator().apply(slot0Configs);
```

---

## Step 3: Apply kD in OuttakeSubsystem

Open `src/main/java/frc/robot/subsystems/OuttakeSubsystem.java`

Find this block in the constructor:

```java
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE; // Tune this value (output per rotation of error)
    // slot0Configs.kI = PID_I_VALUE;
    // slot0Configs.kD = PID_D_VALUE;
    // Add kI, kD, kS, kV if needed for better control
    m_outtakeMotor1.getConfigurator().apply(slot0Configs);
    m_outtakeMotor2.getConfigurator().apply(slot0Configs);
```

Change it to:

```java
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE;
    slot0Configs.kD = PID_D_VALUE;
    m_outtakeMotor1.getConfigurator().apply(slot0Configs);
    m_outtakeMotor2.getConfigurator().apply(slot0Configs);
```

---

## Step 4: Add Telemetry for Tuning

You need to see what the motor is doing to tune PID. If you already added telemetry in TODO 1, you'll add to it. If not, start fresh.

In `IntakeSubsystem.java`, make sure you have this import at the top:

```java
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
```

Then update `periodic()`:

```java
@Override
public void periodic() {
    SmartDashboard.putBoolean("Limit Switch Pressed", isLimitPressed());
    SmartDashboard.putNumber("Left Screw Position", m_leftLinearScrew.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Right Screw Position", m_rightLinearScrew.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Left Screw Velocity", m_leftLinearScrew.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Screw Velocity", m_rightLinearScrew.getVelocity().getValueAsDouble());
}
```

**Velocity is key here.** When the mechanism is approaching its target, velocity should gradually decrease (slow down), not stay high and then drop to zero suddenly.

---

## Step 5: Tune kD on the Real Robot

This is the part you do with the robot. **PID tuning is a process — you try values, watch what happens, and adjust.**

### The Tuning Process

1. Start with `PID_D_VALUE = 0.01` (already set in Step 1)
2. Deploy code to the robot
3. Open Shuffleboard and graph the **velocity** and **position** values
4. Command a deploy (D-Pad Right) and watch the graph

### What to Look For

**Too little D (mechanism slams):**
- Velocity stays high all the way to the target
- Position overshoots the target (goes past 50, then comes back)
- You hear/feel a hard impact at the end of travel
- **Fix:** Increase kD (try 0.02, then 0.03, etc.)

**Too much D (mechanism is sluggish or vibrates):**
- The mechanism slows down way too early and creeps to the target
- OR the mechanism oscillates (shakes back and forth) near the target
- The motor makes a buzzing/whining noise near the target position
- **Fix:** Decrease kD (go back to the last value that didn't vibrate)

**Just right (smooth stop):**
- Velocity is high at the start, gradually decreases as it approaches the target
- Position reaches the target without overshooting
- The mechanism decelerates smoothly, like a car braking gently for a stop sign
- No slamming, no vibrating, no oscillation

### Tuning Steps (do this as a team)

| Attempt | kD Value | What Happened? | Next Step |
|---------|----------|----------------|-----------|
| 1 | 0.01 | _write it down_ | _increase or decrease?_ |
| 2 | _next value_ | _write it down_ | _increase or decrease?_ |
| 3 | _next value_ | _write it down_ | _increase or decrease?_ |

Copy this table onto paper or a whiteboard. Fill it in as you test. **Write down every attempt.** This is real engineering data.

### Tips

- Only change **one value at a time**. Don't change kP and kD in the same attempt.
- Make small changes (0.01 increments). Big jumps make it hard to find the sweet spot.
- Test **both directions** — deploy (going to position 50) and undeploy (going back to position 0). They might behave differently.
- If you change the mechanism (different weight, different friction), you may need to re-tune.
- The Outtake motors use velocity control (RPM), not position control. kD helps there too — it reduces overshoot when spinning up. Tune it separately from the intake screws.

---

## A Note About kI (Integral) — Why We're Skipping It

You might be wondering why we're ignoring the I term. Here's why:

Integral gain adds up error over time. If a motor can't quite reach its target (maybe there's friction), I says "push harder and harder until you get there."

**The problem:** If there's a hard stop preventing the motor from reaching the target, I keeps adding up and tells the motor to push harder... and harder... and harder. This is called **integral windup** and it can generate dangerous amounts of force.

On mechanisms with physical stops (like linear screws), **do not use I gain** unless you also add anti-windup protection, which is an advanced topic. For now, P + D is what you need.

---

## How to Know You're Done

- [ ] `PID_D_VALUE` is added to both `Intake` and `Outtake` in `Constants.java`
- [ ] `slot0Configs.kD` is set in both `IntakeSubsystem.java` and `OuttakeSubsystem.java`
- [ ] Telemetry shows position AND velocity on the dashboard
- [ ] You tuned kD using the table above and found a value that stops smoothly
- [ ] Mechanism does not slam, overshoot, or vibrate when reaching target position
- [ ] Both deploy and undeploy directions were tested
- [ ] Your final kD values are written down somewhere the team can find them

---

## Files You Changed

| File | What You Changed |
|------|-----------------|
| `Constants.java` | Added `PID_D_VALUE` to both `Intake` and `Outtake` classes |
| `IntakeSubsystem.java` | Applied `kD` in constructor, added velocity telemetry to `periodic()` |
| `OuttakeSubsystem.java` | Applied `kD` in constructor |
