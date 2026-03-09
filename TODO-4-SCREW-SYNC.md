# TODO 4: Make Sure Left and Right Linear Screws Stay Synchronized

## Why This Matters

The intake mechanism is driven by **two** linear screws — one on the left, one on the right. They need to move at the same speed and reach the same position at the same time. If one gets ahead of the other, the mechanism **twists**. A twisted mechanism can:

- Bind up and stall a motor
- Bend or break mechanical parts
- Jam so badly you can't retract the intake

Right now, both screws get the same position target and the same PID values, so they *should* move together. But "should" isn't good enough — different friction, different loads, a slightly tighter screw, or a wiring difference can cause one side to lag behind.

---

## How It Works Right Now

Look at `deployIntake()`:

```java
public void deployIntake(){
    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
    m_rightLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
}
```

Both motors get told "go to position 50." Each motor runs its own PID loop independently. They don't talk to each other. If the left side has less friction, it gets there first. If the right motor is slightly weaker, it falls behind.

---

## Step 1: Add Telemetry to See the Problem (or Confirm It's Fine)

Before you change anything, you need to **measure**. You may have already added position telemetry in TODO 1 and 2. If so, you need to add one more value: the **difference** between the two screw positions.

Open `src/main/java/frc/robot/subsystems/IntakeSubsystem.java`

Make sure you have this import:

```java
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
```

Update `periodic()` to include a drift value:

```java
@Override
public void periodic() {
    double leftPos = m_leftLinearScrew.getPosition().getValueAsDouble();
    double rightPos = m_rightLinearScrew.getPosition().getValueAsDouble();
    double drift = Math.abs(leftPos - rightPos);

    SmartDashboard.putBoolean("Limit Switch Pressed", isLimitPressed());
    SmartDashboard.putNumber("Left Screw Position", leftPos);
    SmartDashboard.putNumber("Right Screw Position", rightPos);
    SmartDashboard.putNumber("Screw Drift", drift);
    SmartDashboard.putNumber("Left Screw Velocity", m_leftLinearScrew.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Screw Velocity", m_rightLinearScrew.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake Roller RPM", m_intakeRoller.getVelocity().getValueAsDouble() * 60.0);
}
```

**"Screw Drift"** shows how far apart the two screws are in rotations. Ideally this number is 0. Anything under 0.5 rotations is probably fine. Over 1.0 rotation means you have a problem.

---

## Step 2: Test and Observe

1. Deploy the code
2. Open Shuffleboard and watch "Left Screw Position", "Right Screw Position", and "Screw Drift"
3. Command a deploy (D-Pad Right) — watch both positions climb toward 50
4. Command an undeploy (D-Pad Down) — watch both positions go back toward 0
5. Repeat several times

### Record What You See

| Test | Left Final Pos | Right Final Pos | Drift | Notes |
|------|---------------|-----------------|-------|-------|
| Deploy 1 | _____  | _____ | _____ | _____ |
| Undeploy 1 | _____ | _____ | _____ | _____ |
| Deploy 2 | _____ | _____ | _____ | _____ |
| Undeploy 2 | _____ | _____ | _____ | _____ |
| Deploy 3 | _____ | _____ | _____ | _____ |
| Undeploy 3 | _____ | _____ | _____ | _____ |

### What You're Looking For

**Drift stays under 0.5 rotations** → You're fine! The independent PID loops are handling it. Keep the telemetry so you can monitor during matches, but no code changes needed. Skip to "How to Know You're Done."

**Drift is between 0.5 and 2.0 rotations** → Concerning but not emergency. Go to Step 3.

**Drift is over 2.0 rotations** → The mechanism is probably visibly twisting. Go to Step 3 now. Also check the mechanical side — is one screw physically harder to turn? Is something loose?

---

## Step 3: Fix It With Follower Mode (If Drift Is a Problem)

If your tests showed unacceptable drift, the best fix is to make one motor **follow** the other. Instead of both motors running their own independent PID, the right motor copies exactly what the left motor does.

### 3a: Add the Follower import

At the top of `IntakeSubsystem.java`, add:

```java
import com.ctre.phoenix6.controls.Follower;
```

### 3b: Set up the follower in the constructor

Find the section where you create the motors:

```java
    m_leftLinearScrew = new TalonFX(20);
    m_rightLinearScrew = new TalonFX(30);
```

Add the follower setup **after** the PID config is applied:

```java
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE;
    slot0Configs.kD = PID_D_VALUE;
    m_leftLinearScrew.getConfigurator().apply(slot0Configs);
    m_rightLinearScrew.getConfigurator().apply(slot0Configs);

    // Make right screw follow left screw exactly
    m_rightLinearScrew.setControl(new Follower(20, false));
    // 20 = CAN ID of the left linear screw (the "leader")
    // false = same direction (set to true if the right screw is mounted opposite and needs to spin the other way)
```

### 3c: Stop commanding the right screw directly

Now that the right screw follows the left automatically, you should only command the left screw. The right will copy it.

Change `deployIntake()`:

```java
public void deployIntake(){
    if (isLimitPressed()) {
        m_leftLinearScrew.set(0);
        m_rightLinearScrew.set(0);
        return;
    }
    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
    // m_rightLinearScrew follows automatically — no command needed
}
```

Change `undeployIntake()`:

```java
public void undeployIntake(){
    if (isLimitPressed()) {
        m_leftLinearScrew.set(0);
        m_rightLinearScrew.set(0);
        return;
    }
    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
    // m_rightLinearScrew follows automatically — no command needed
}
```

Change `stopStorage()`:

```java
public void stopStorage() {
    m_leftLinearScrew.set(0);
    // Right screw will also stop because it follows left
    // But we explicitly stop it too, just to be safe
    m_rightLinearScrew.set(0);
}
```

**Note on stopping:** When you call `.set(0)` on the leader, the follower should also stop. We explicitly stop the right screw too as a safety measure — belt and suspenders.

### 3d: Direction Check

The `false` in `new Follower(20, false)` means "spin the same direction as the leader."

**This depends on how your screws are mounted:**
- If both screws are mounted the same way (both facing the same direction) → use `false`
- If they're mirrored (one faces left, one faces right) → use `true` to invert the follower

**How to check:** Disconnect the mechanism from the screws so they spin freely. Command a deploy. Do both screws spin the same direction?
- Same direction → `false` is correct
- Opposite directions → change to `true`
- If you're not sure, start with `false`, watch carefully on first test, and be ready to disable

---

## Step 4: Re-Test After Changes

If you implemented follower mode, test again:

1. Deploy and watch "Screw Drift" — it should now be near zero at all times
2. The right screw position should exactly track the left screw position
3. Try multiple deploy/undeploy cycles
4. Gently apply resistance to one side (carefully!) — both should respond together

---

## Understanding Follower Mode vs. Independent Control

| | Independent (current code) | Follower (Step 3) |
|---|---|---|
| **How it works** | Each motor runs its own PID to reach the target | Right motor copies left motor's output exactly |
| **When they match** | If friction/load is equal on both sides | Always — by definition |
| **Risk if one side binds** | The free side keeps going, mechanism twists | Both sides respond together |
| **When to use** | Mechanisms where slight drift is acceptable | Mechanisms where two motors MUST stay in sync |

---

## How to Know You're Done

- [ ] Telemetry shows left position, right position, AND drift
- [ ] You tested at least 3 deploy/undeploy cycles and recorded the data
- [ ] Drift is consistently under 0.5 rotations (either naturally or with follower mode)
- [ ] If you implemented follower mode: right screw tracks left screw exactly
- [ ] If you implemented follower mode: direction is correct (both screws move the mechanism the right way)
- [ ] The mechanism deploys and retracts without visible twisting

---

## Files You Changed

| File | What You Changed |
|------|-----------------|
| `IntakeSubsystem.java` | Added drift telemetry to `periodic()` |
| `IntakeSubsystem.java` | (If needed) Added `Follower` import, set up follower in constructor, removed right screw commands from deploy/undeploy |

No changes to `Constants.java` or `RobotContainer.java` for this fix.
