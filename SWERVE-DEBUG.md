# Swerve Debug Guide — Deep Space Robotics (FRC 6884)

## Quick Reference: CAN ID Map

| Module | Drive Motor | Steer Motor | CANcoder | Encoder Offset (rot) |
|--------|-------------|-------------|----------|----------------------|
| **FL** | 1 | 2 | 0 | -0.47216796875 |
| **FR** | 3 | 4 | 3 | -0.16357421875 |
| **BL** | 5 | 6 | 2 | -0.193115234375 |
| **BR** | 7 | 8 | 1 | 0.2744140625 |

**CAN Bus:** `Galigma Jr`
**Pigeon 2 ID:** 0
**Drive inversion:** Left side = false, Right side = true
**All steer motors:** Not inverted
**All CANcoders:** Inverted = true

---

## Dashboard Telemetry (SwerveDebug)

The `SwerveDebug` class publishes per-module data under `Swerve/` in SmartDashboard / Shuffleboard / Glass. Available keys:

| Key | What it tells you |
|-----|-------------------|
| `Swerve/FL/Angle (deg)` | Current wheel heading |
| `Swerve/FL/Target Angle (deg)` | Where PID wants the wheel |
| `Swerve/FL/Angle Error (deg)` | Difference (normalized ±180°) |
| `Swerve/FL/Speed (m/s)` | Current wheel speed |
| `Swerve/FL/Target Speed (m/s)` | Commanded wheel speed |
| `Swerve/FL/Steer OK` | Boolean — false if angle error > 15° |
| `Swerve/Heading (deg)` | Robot heading from Pigeon |
| `Swerve/Odometry Hz` | Odometry loop rate |

Replace `FL` with `FR`, `BL`, `BR` for other modules.

### How to set up in Shuffleboard

1. Connect to the robot and open Shuffleboard
2. Go to **Sources > NetworkTables > SmartDashboard > Swerve**
3. Drag per-module values onto a tab
4. Recommended layout: 4-column grid (one per module) with Angle, Target Angle, Angle Error, Steer OK
5. Set `Steer OK` widget type to **Boolean Box** (green/red)

---

## Debugging Flowchart

### 🚨 Motor spinning on startup

```
1. Is the CAN ID correct in TunerConstants?
   → Open Tuner X, verify physical device matches code
   → Check EXPECTED CAN IDs on dashboard under Swerve/XX/CAN/

2. Is the CANcoder offset correct?
   → Physically point wheel straight forward
   → Read raw absolute position in Tuner X
   → The offset should make that position read ~0
   → Compare to value in TunerConstants

3. Is the encoder inversion correct?
   → Rotate module clockwise (from top)
   → CANcoder value should increase (positive direction)
   → If it decreases, flip kXXEncoderInverted

4. Is the steer motor inversion correct?
   → Command a small positive voltage to steer motor in Tuner X
   → Motor should turn the same direction CANcoder reads as positive
   → If opposite, flip kXXSteerMotorInverted

5. Is there a CAN wiring issue?
   → Check for flickering LEDs on the device
   → Look for CAN errors in Driver Station log
   → Verify bus termination (120Ω between CAN-H and CAN-L)
```

### 🔄 Module drifting / not holding angle

```
1. Check Swerve/XX/Angle Error on dashboard
   → Small oscillation (±1-2°): normal
   → Large steady error: PID gains too low or encoder issue
   → Growing/runaway error: inversion mismatch (see above)

2. Check steer PID gains (TunerConstants)
   → Current: kP=100, kI=0, kD=0.5, kS=0.1, kV=2.66
   → If oscillating: reduce kP or increase kD
   → If sluggish: increase kP slightly

3. Is the coupling ratio correct?
   → Current: 3.125 (drive turns per azimuth turn)
   → Wrong value causes the drive motor to fight the steer
```

### 🐌 Robot driving wrong direction / rotating unexpectedly

```
1. Check drive motor inversions
   → Left side should NOT be inverted (kInvertLeftSide = false)
   → Right side SHOULD be inverted (kInvertRightSide = true)
   → Verify in Tuner X: positive voltage = wheel spinning "forward"

2. Check Pigeon 2 orientation
   → Pigeon must be mounted with the correct axis facing forward
   → Reset heading: press Y button (seeds field-centric)

3. Check module positions in TunerConstants
   → FL: (+10.875, +10.875) inches
   → FR: (+10.875, -10.875)
   → BL: (-10.875, +10.875)
   → BR: (-10.875, -10.875)
   → If swapped, kinematics will be wrong
```

### 📡 CAN bus issues

```
1. Check Driver Station for CAN utilization (should be < 70%)
2. Look for "CAN frame not received" errors in console
3. Verify CAN bus name matches: "Galigma Jr"
4. Check physical connections:
   → All CAN-H (yellow) and CAN-L (green) daisy-chained
   → 120Ω termination resistor at each end of bus
   → No CAN ID conflicts (each device unique within type)
```

---

## Pre-Match Checklist

- [ ] All 8 motors + 4 CANcoders + Pigeon visible in Tuner X
- [ ] All `Steer OK` booleans are green on dashboard
- [ ] Angle errors < 5° with robot stationary
- [ ] Wheels physically point forward when code commands 0°
- [ ] Field-centric heading seeded (Y button)
- [ ] No CAN errors in Driver Station log
- [ ] Battery voltage > 12.5V

## Re-Calibrating CANcoder Offsets

If you ever need to redo offsets (e.g., after mechanical changes):

1. Physically point all 4 wheels perfectly forward (use a straight edge)
2. Open Tuner X → select each CANcoder
3. Read the **Absolute Position** value
4. That value (negated) becomes the new `kXXEncoderOffset` in `TunerConstants.java`
5. Or: use **Tuner X Swerve Project Generator** to regenerate the file

---

*Last updated: 2026-02-20*
*Robot: Deep Space Robotics FRC 6884*
