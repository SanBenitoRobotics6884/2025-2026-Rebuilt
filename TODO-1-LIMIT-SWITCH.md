# TODO 1: Make the Linear Screws Stop When the Limit Switch is Hit

## Why This Matters

Right now, when the limit switch gets pressed, the robot tries to stop the linear screws — but they immediately start moving again. That means the mechanism keeps pushing into a hard stop, which can strip gears, bend parts, or burn out motors.

This is the **highest priority fix** because it can physically break the robot.

---

## Understanding the Bug

Let's trace what happens step by step:

1. You press D-Pad Right → `deployIntakeCommand()` starts running
2. `deployIntakeCommand()` uses `run()`, which means **it runs `deployIntake()` over and over, every 20ms, forever** (until you release the button)
3. Every 20ms, `deployIntake()` tells both screws: "go to position 50"
4. The screws move toward position 50...
5. The limit switch gets pressed!
6. The trigger in `RobotContainer` fires `stopStorageCommand()` → sets both screws to 0 power
7. **But wait** — `deployIntake()` runs again 20ms later and says "go to position 50" again
8. The screws start moving again, even though the limit switch is pressed
9. The stop only lasted for one cycle (20ms). The mechanism keeps slamming.

**The fix:** Before commanding the screws to move, check if the limit switch is already pressed. If it is, don't send the command.

---

## Step 1: Add the Safety Check to `deployIntake()`

Open `src/main/java/frc/robot/subsystems/IntakeSubsystem.java`

Find this method:

```java
public void deployIntake(){
    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
    m_rightLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
}
```

Change it to:

```java
public void deployIntake(){
    if (isLimitPressed()) {
        m_leftLinearScrew.set(0);
        m_rightLinearScrew.set(0);
        return;
    }
    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
    m_rightLinearScrew.setControl(p_PositionRequest.withPosition(IN_TAKE_TARGET_ROTATIONS));
}
```

**What this does:** Every time the robot tries to command the screws, it first asks "is the limit switch pressed?" If yes, it stops the screws and skips the rest of the method. The `return;` keyword exits the method early — nothing after it runs.

---

## Step 2: Decide About `undeployIntake()`

Talk with your team: **which direction hits the hard stop?**

- If the limit switch is at the **deployed** end → only guard `deployIntake()` (done in Step 1)
- If the limit switch is at the **undeployed/retracted** end → guard `undeployIntake()` instead (or also)
- If you're not sure → guard both for now, it's safer

To guard `undeployIntake()`, find:

```java
public void undeployIntake(){
    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
    m_rightLinearScrew.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
}
```

Change it to:

```java
public void undeployIntake(){
    if (isLimitPressed()) {
        m_leftLinearScrew.set(0);
        m_rightLinearScrew.set(0);
        return;
    }
    m_leftLinearScrew.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
    m_rightLinearScrew.setControl(p_PositionRequest.withPosition(OUT_TAKE_TARGET_ROTATIONS));
}
```

---

## Step 3: Add Telemetry So You Can See What's Happening

Still in `IntakeSubsystem.java`, add this import at the top of the file (with the other imports):

```java
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
```

Then find the `periodic()` method:

```java
@Override
public void periodic() {

    // This method will be called once per scheduler run
}
```

Change it to:

```java
@Override
public void periodic() {
    SmartDashboard.putBoolean("Limit Switch Pressed", isLimitPressed());
    SmartDashboard.putNumber("Left Screw Position", m_leftLinearScrew.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Right Screw Position", m_rightLinearScrew.getPosition().getValueAsDouble());
}
```

**What this does:** Every cycle, it sends the limit switch state and both screw positions to the dashboard. You'll be able to see these values in Shuffleboard or SmartDashboard while the robot runs.

---

## Step 4: Test in Simulation

You can test this without the real robot using the simulator.

### How the Sim Limit Switch Works

Look at the `isLimitPressed()` method in your code:

```java
public boolean isLimitPressed() {
    if (RobotBase.isSimulation()){
        return simTimer.hasElapsed(5.0);
    }
    return !i_limitSwitch.get();
}
```

In simulation mode, the limit switch automatically "triggers" 5 seconds after the timer starts. The timer starts when the subsystem is created, and it resets when `resetTimers()` is called. So you have a 5-second window to test before the switch activates.

### Running the Test

1. Open a terminal in the project folder
2. Run: `./gradlew simulateJava`
3. The simulator window will open — enable **Teleop** mode
4. Open Shuffleboard (or SmartDashboard) and look for:
   - `Limit Switch Pressed` — should show `false` at first
   - `Left Screw Position` and `Right Screw Position`
5. Press D-Pad Right (or simulate it) to deploy the intake
6. Watch the screw positions change
7. After 5 seconds, `Limit Switch Pressed` should flip to `true`
8. **Check:** Do the screw positions STOP changing? Do they stay still?

### What "Pass" Looks Like
- Before 5 seconds: screws move toward target (position numbers changing)
- At 5 seconds: `Limit Switch Pressed` → `true`
- After 5 seconds: screw positions **stop changing**, screw power reads 0
- If you keep holding D-Pad Right, screws should **still** not move

### What "Fail" Looks Like
- Limit switch shows `true` but screws keep moving
- Screw positions keep changing after 5 seconds
- This means the guard in `deployIntake()` isn't working — double-check your code

---

## Step 5: Test on the Real Robot

Once simulation passes:

1. Deploy the code to the robot
2. Open Shuffleboard and find the same values
3. **Safety first:** Have someone ready to disable the robot
4. Slowly trigger a deploy command
5. Manually press the limit switch with your hand while the screws are moving
6. The screws should stop immediately and stay stopped
7. Release the limit switch → the screws should start moving again (if you're still holding the deploy button)

---

## How to Know You're Done

- [ ] `deployIntake()` checks `isLimitPressed()` before commanding the screws
- [ ] You decided with your team whether `undeployIntake()` also needs the check
- [ ] Telemetry is showing limit switch state and screw positions on the dashboard
- [ ] Simulation test passes (screws stop at 5 seconds and stay stopped)
- [ ] Real robot test passes (screws stop when limit switch is physically pressed)

---

## Files You Changed

| File | What You Changed |
|------|-----------------|
| `IntakeSubsystem.java` | Added limit switch check in `deployIntake()`, added SmartDashboard telemetry in `periodic()`, added SmartDashboard import |
| `IntakeSubsystem.java` | (Optional) Added limit switch check in `undeployIntake()` |

No changes needed in `Constants.java` or `RobotContainer.java` for this fix.
