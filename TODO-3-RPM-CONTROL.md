# TODO 3: Add RPM (Velocity) Control to the Intake Roller

## Why This Matters

Right now, the intake roller motor is controlled like this:

```java
m_intakeRoller.set(TAKE_SPEED);  // TAKE_SPEED = -0.375
```

That `.set()` call sends a **percentage of power** — it says "run at 37.5% power." But power percentage is NOT the same as speed. Here's why that's a problem:

- At 37.5% power with no load → the roller spins fast
- At 37.5% power while grabbing a game piece → the roller slows way down because of resistance
- At 37.5% power with a low battery → the roller spins even slower

The motor doesn't know it's supposed to maintain a certain speed. It just applies power and hopes for the best.

### What RPM Control Does

RPM control (also called **velocity control**) tells the motor "spin at exactly this speed" instead of "apply this much power." The motor's built-in PID controller adjusts the power automatically to hold that speed — if it hits resistance, it pushes harder. If it's going too fast, it backs off.

Your code already does this for the **storage roller** and the **outtake motors** using `VelocityVoltage`. The intake roller is the one that got missed.

---

## Step 1: Add Intake RPM Constant

Open `src/main/java/frc/robot/Constants/Constants.java`

Find the `Intake` class and add a target RPM:

```java
public class Intake {
    public static final int LIMIT_SWITCH_STORAGE_DIO = 0;

    public static final double PID_P_VALUE = 0.25;
    public static final double PID_D_VALUE = 0.01;  // from TODO 2

    public static final double DUTYCYCLE_OUTPUT = 0.5;

    public static final double TAKE_SPEED = -0.375;
    public static final double STORAGEROLLER_SPEED = 0.25;

    public static final double IN_TAKE_TARGET_ROTATIONS = 50.0;
    public static final double OUT_TAKE_TARGET_ROTATIONS = 0.0;
```

Add this at the bottom of the Intake class (before the closing `}`):

```java
    public static final double INTAKE_ROLLER_RPM = 3000.0;  // Target RPM — adjust to what works for your game piece
```

**Note:** 3000 RPM is a starting point — it matches what the storage roller uses. You may need to tune this up or down depending on how aggressively you want to grab game pieces.

---

## Step 2: Add VelocityVoltage to the Intake Roller

Open `src/main/java/frc/robot/subsystems/IntakeSubsystem.java`

### 2a: Create a velocity request object

Find the area near the top where the control objects are created:

```java
  PositionVoltage p_PositionRequest = new PositionVoltage(0).withSlot(0);

  DutyCycleOut speed;
```

Add a velocity request right after:

```java
  PositionVoltage p_PositionRequest = new PositionVoltage(0).withSlot(0);
  VelocityVoltage m_intakeVelocity = new VelocityVoltage(0);

  DutyCycleOut speed;
```

### 2b: Apply PID config to the intake roller motor

In the constructor, the PID config is only applied to the linear screws right now. You need to also apply it to the intake roller.

Find this block:

```java
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE;
    slot0Configs.kD = PID_D_VALUE;
    m_leftLinearScrew.getConfigurator().apply(slot0Configs);
    m_rightLinearScrew.getConfigurator().apply(slot0Configs);
```

**Important:** The intake roller might need different PID values than the linear screws. A roller spinning freely is very different from a screw pushing a mechanism. For now, we'll use the same values to get it working, then tune separately if needed.

Add one line after the screw configs:

```java
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = PID_P_VALUE;
    slot0Configs.kD = PID_D_VALUE;
    m_leftLinearScrew.getConfigurator().apply(slot0Configs);
    m_rightLinearScrew.getConfigurator().apply(slot0Configs);
    m_intakeRoller.getConfigurator().apply(slot0Configs);
```

### 2c: Change `runIntake()` to use velocity control

Find:

```java
  public void runIntake() { 
   m_intakeRoller.set(TAKE_SPEED);
   m_storageRoller.set(STORAGEROLLER_SPEED);
  }
```

Change to:

```java
  public void runIntake() { 
   double targetRPS = INTAKE_ROLLER_RPM / 60.0;  // Convert RPM to rotations per second
   m_intakeRoller.setControl(m_intakeVelocity.withVelocity(-targetRPS));  // Negative = intake direction
   m_storageRoller.set(STORAGEROLLER_SPEED);
  }
```

**Why divide by 60?** The motor controller thinks in rotations per **second** (RPS), but humans think in rotations per **minute** (RPM). There are 60 seconds in a minute, so RPM ÷ 60 = RPS.

**Why negative?** The old `TAKE_SPEED` was negative (-0.375), which means the roller spins in the intake direction. We keep the negative sign to maintain the same direction.

### 2d: Change `runIntakeRollerBack()` to use velocity control

Find:

```java
  public void runIntakeRollerBack(){
    m_intakeRoller.set(-TAKE_SPEED);
  }
```

Change to:

```java
  public void runIntakeRollerBack(){
    double targetRPS = INTAKE_ROLLER_RPM / 60.0;
    m_intakeRoller.setControl(m_intakeVelocity.withVelocity(targetRPS));  // Positive = reverse/eject direction
  }
```

---

## Step 3: Add Telemetry for the Intake Roller

In `periodic()`, add the roller velocity so you can see if RPM control is working. If you already have telemetry from TODO 1 and 2, just add these lines:

```java
@Override
public void periodic() {
    SmartDashboard.putBoolean("Limit Switch Pressed", isLimitPressed());
    SmartDashboard.putNumber("Left Screw Position", m_leftLinearScrew.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Right Screw Position", m_rightLinearScrew.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Left Screw Velocity", m_leftLinearScrew.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Screw Velocity", m_rightLinearScrew.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake Roller RPM", m_intakeRoller.getVelocity().getValueAsDouble() * 60.0);  // Convert RPS back to RPM for readability
}
```

**Note:** We multiply by 60 when *reading* to convert back to RPM. Humans read RPM better than RPS.

---

## Step 4: Test

### What to Check on the Dashboard

1. Command the intake to run (Left Trigger)
2. Watch "Intake Roller RPM" on Shuffleboard
3. It should climb to near your target (3000 RPM) and hold steady

### What "Pass" Looks Like

- RPM quickly reaches ~3000 and stays within ~200 of that (2800-3200 is fine)
- When you lightly add resistance (carefully!), RPM dips briefly then recovers
- When you release the trigger, RPM drops to 0

### What "Fail" Looks Like

- RPM never reaches the target → kP might be too low, or the target is too high for the motor
- RPM oscillates wildly (bounces between numbers) → kP is too high or kD is too low
- RPM reads 0 or doesn't change → check that you're using `setControl()` not `.set()`, and that the VelocityVoltage import is correct

### Tuning the Target RPM

If 3000 RPM is too fast or too slow for your game piece:
1. Change `INTAKE_ROLLER_RPM` in Constants.java
2. Redeploy and test
3. You want fast enough to grab the piece reliably, but not so fast it launches it through the robot

---

## How to Know You're Done

- [ ] `INTAKE_ROLLER_RPM` constant added to `Constants.java`
- [ ] `VelocityVoltage` request object created in `IntakeSubsystem`
- [ ] PID config applied to `m_intakeRoller` in the constructor
- [ ] `runIntake()` uses `setControl()` with `VelocityVoltage` instead of `.set()`
- [ ] `runIntakeRollerBack()` also uses velocity control
- [ ] Dashboard shows intake roller RPM
- [ ] RPM holds steady at target during testing

---

## Files You Changed

| File | What You Changed |
|------|-----------------|
| `Constants.java` | Added `INTAKE_ROLLER_RPM` to the `Intake` class |
| `IntakeSubsystem.java` | Added `VelocityVoltage` object, applied PID to intake roller, changed `runIntake()` and `runIntakeRollerBack()` to velocity control, added RPM telemetry |
