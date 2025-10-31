# Differential Drive Robot - Wiring Diagram

**Hardware**: Arduino Mega 2560, L298N Motor Driver, 2x DC Motors, Encoders, MPU6050
**Last Updated**: 2025-10-30

---

## Quick Reference Pin Map

### Arduino Mega 2560 Pin Allocation

```
MOTOR CONTROL PINS:
├─ Pin 5:  Motor A (Left) PWM Speed
├─ Pin 6:  Motor B (Right) PWM Speed
├─ Pin 8:  Motor A Direction 1
├─ Pin 9:  Motor A Direction 2
├─ Pin 10: Motor B Direction 1
├─ Pin 11: Motor B Direction 2
│
ENCODER PINS:
├─ Pin 2:  Left Encoder (Interrupt)
├─ Pin 3:  Right Encoder (Interrupt)
│
I2C PINS (IMU):
├─ Pin 20: SDA (I2C Data)
├─ Pin 21: SCL (I2C Clock)
│
SAFETY/STATUS:
├─ Pin 12: Emergency Stop Button
├─ Pin 13: Status LED
│
POWER:
├─ GND:    Ground (common)
├─ 5V:     Arduino logic power
└─ Vin:    Battery input (optional)
```

---

## Complete Wiring Guide

### 1. Motor Power Connection (L298N Driver)

**Component**: L298N Dual Motor Driver Module

```
L298N PIN LAYOUT (Top View):
┌─────────────────────────────┐
│ +5V    GND   GND   12V+ 12V+│
│  │      │     │     │    │  │
│  1      2     3     4    5  │
│                             │
│  IN1  IN2  IN3  IN4  ENA  ENB│
│  │    │    │    │    │    │  │
│ 15    14   13   12   11   10 │
│                             │
│ OUT1 OUT2 OUT3 OUT4          │
│  │    │    │    │            │
│  6    7    8    9            │
└─────────────────────────────┘
```

**Connections to Arduino**:

| L298N Pin | Arduino Mega Pin | Purpose | Color |
|-----------|-----------------|---------|-------|
| IN1 | 8 | Motor A direction | Purple |
| IN2 | 9 | Motor A direction | Gray |
| IN3 | 10 | Motor B direction | Blue |
| IN4 | 11 | Motor B direction | Green |
| ENA | 5 | Motor A speed (PWM) | Red |
| ENB | 6 | Motor B speed (PWM) | Orange |
| GND | GND | Ground | Black |
| +5V | 5V | Logic power | Red |

**Connections to Motors**:

| L298N Pin | Motor Connection | Purpose |
|-----------|------------------|---------|
| OUT1, OUT2 | Left Motor | Motor power (either direction) |
| OUT3, OUT4 | Right Motor | Motor power (either direction) |

**Connections to Battery**:

| L298N Pin | Battery Connection | Purpose | Note |
|-----------|-------------------|---------|------|
| +12V (12V+) | Battery + | Motor power supply | Choose voltage matching motors |
| GND (2x) | Battery - | Return path | Use both GND pins for current |

---

### 2. Motor Wiring Detail

**Left Motor (Motor A)**:
```
Motor A (Left):
┌────────────────┐
│   DC Motor     │
│  +-|  3-6V     │
│  +-|           │
└────────────────┘
    │         │
    │         └─── OUT2 (L298N)
    └──────────────── OUT1 (L298N)

Direction Control:
IN1=HIGH, IN2=LOW   → Forward
IN1=LOW,  IN2=HIGH  → Backward
IN1=LOW,  IN2=LOW   → Stop
IN1=HIGH, IN2=HIGH  → Stop
```

**Right Motor (Motor B)**:
```
Motor B (Right):
┌────────────────┐
│   DC Motor     │
│  +-|  3-6V     │
│  +-|           │
└────────────────┘
    │         │
    │         └─── OUT4 (L298N)
    └──────────────── OUT3 (L298N)

Direction Control:
IN3=HIGH, IN4=LOW   → Forward
IN3=LOW,  IN4=HIGH  → Backward
IN3=LOW,  IN4=LOW   → Stop
IN3=HIGH, IN4=HIGH  → Stop
```

**Motor Connection Wires**:
- Use **18AWG stranded wire** for motor connections (lower resistance)
- Solder or use crimp connectors (not breadboard jumpers)
- Add **1N4007 diode** across motor leads (cathode to +, anode to -)
  - Protects against back-EMF spikes
  - Prevents L298N damage during braking

---

### 3. Encoder Wiring

**Encoder Kit Connection**:

Each encoder has 3-5 wires:
- **VCC**: 5V power
- **GND**: Ground
- **A**: Quadrature signal A (interrupt pin)
- **B**: Quadrature signal B
- **(Optional) Index**: Zero reference pulse

**Left Encoder**:
```
Left Encoder:
┌─────────────┐
│  Quad Enc   │
│  A  B  5V   │
│  │  │  │    │
│  │  │  └─── 5V
│  │  └─────── GND
│  └────────── Pin 2 (Interrupt)
└─────────────┘

Additional:
Signal B (if available) → Pin 2 same signal (redundant)
Index signal → Optional, ignore for basic setup
```

**Right Encoder**:
```
Right Encoder:
┌─────────────┐
│  Quad Enc   │
│  A  B  5V   │
│  │  │  │    │
│  │  │  └─── 5V
│  │  └─────── GND
│  └────────── Pin 3 (Interrupt)
└─────────────┘
```

**Connection Table**:

| Encoder Signal | Left Encoder (Mega Pin) | Right Encoder (Mega Pin) | Notes |
|---|---|---|---|
| VCC | 5V | 5V | Power supply |
| GND | GND | GND | Ground (common) |
| A (Signal) | Pin 2 | Pin 3 | Interrupt pins (hardware) |
| B (Signal) | Optional | Optional | Not used in basic firmware |

**Pull-up Resistors** (if not built-in):
- Add 10k resistor from signal A to 5V (both encoders)
- Ensures clean digital transitions

---

### 4. IMU (MPU6050) I2C Connection

**MPU6050 Pinout**:
```
MPU6050 Module (Top View):
┌──────────────────┐
│ VCC  GND  SDA SCL│
│  │    │    │   │ │
│  1    2    3   4 │
│                  │
│  INT  AD0  (Reserved)
│  │    │          │
│  5    6          │
└──────────────────┘
```

**Connections to Arduino Mega**:

| MPU6050 Pin | Arduino Mega Pin | Purpose | Wire Color |
|---|---|---|---|
| VCC | 3.3V | Power supply | Red |
| GND | GND | Ground | Black |
| SDA | Pin 20 | I2C Data | Green |
| SCL | Pin 21 | I2C Clock | Yellow |
| AD0 | GND | I2C address select (optional) | - |
| INT | (Unused) | Interrupt (optional) | - |

**I2C Pullup Resistors** (REQUIRED):
```
Pullup Resistor Circuit:
         5V
          │
          ┌─────┐
          │10k Ω│  (Alternative: 4.7k)
          └──┬──┘
             │
        ┌────┴─────────┐
        │              │
      SDA             SCL
      (Pin 20)       (Pin 21)
        │              │
        │              │
      MPU6050        MPU6050
      (SDA pin)      (SCL pin)
        │              │
        └──────┬───────┘
               │
              GND
```

**Alternate Configuration** (if 3.3V rail unavailable):
```
Pullup to 5V with voltage divider:
         5V
          │
          ┌─────┐
          │10k Ω│
          └──┬──┘
             │
        ┌────┴─────────┐
        │              │
       SDA (with       SCL (with
       voltage divider) voltage divider)
```

---

### 5. Power System Wiring

**Battery Configuration for 6V System**:

```
Battery Configuration (4x AA holder):
┌─────────────────────────────┐
│  [+] [+] [-] [-]            │
│   │   │   │   │             │
│   ├───┤   ├───┤             │
│   │ │ │   │ │ │             │
│  1.5V 1.5V 1.5V 1.5V = 6V   │
│   │   │   │   │             │
│   ├───┴───┴───┤             │
│       │   │                 │
│       +   -                 │
│       │   │                 │
│       6V DC (Nominal)       │
└─────────────────────────────┘
        │   │
        │   └──→ GND (Common Return)
        │
        └─→ L298N +12V input
            Battery + lead
```

**Power Distribution**:
```
Battery (+6V)
    │
    ├──→ L298N +12V ──→ Motor Power
    │
    ├──→ [Fuse 3A]
    │
    ├──→ Arduino Vin (via 5V regulator on board)
    │
    └──→ GND (Return)

Battery (GND)
    │
    ├──→ L298N GND ──→ Motor Return
    │
    ├──→ Arduino GND
    │
    └──→ MPU6050 GND
```

**Alternative: Separate Battery for Logic**:
```
Motor Power:        Logic Power:
Battery 7.4V ────→ L298N        Battery 5V ────→ Arduino
    GND ──────→ GND (shared)    GND ──────→ GND (shared)

Note: MUST share common ground between batteries
```

---

### 6. Emergency Stop Circuit

**Emergency Stop Button (Safety)**:

```
Emergency Stop Circuit (Pin 12):
       +5V
        │
        ┌──────────────┐
        │              │
        ├─[10k Pull-up]
        │              │
      [Button]         │
        │              │
        └──────→ Pin 12 (Input, active LOW)
                │
               GND
```

**Operation**:
- Button open (normal): Pin 12 reads HIGH (5V) → Motors enabled
- Button pressed (emergency): Pin 12 reads LOW (0V) → Motors stop
- LED on Pin 13 goes out when emergency stop activated

---

### 7. Complete System Diagram

```
DIFFERENTIAL DRIVE ROBOT SYSTEM ARCHITECTURE:

┌────────────────────────────────────────────────────────────┐
│                    COMPUTER / ROS2                        │
│                    (USB Interface)                        │
└────────────────────┬───────────────────────────────────────┘
                     │
                     │ Serial (USB)
                     │ Baud: 115200
                     │
        ┌────────────▼─────────────────┐
        │     Arduino Mega 2560        │
        │                              │
        │  ┌──────────────────────┐    │
        │  │  PWM Motor Control   │    │
        │  │ Pins: 5,6,8,9,10,11 │    │
        │  └──────────┬───────────┘    │
        │             │                │
        │  ┌──────────▼───────────┐    │
        │  │ Encoder Interrupt    │    │
        │  │ Pins: 2,3            │    │
        │  └──────────┬───────────┘    │
        │             │                │
        │  ┌──────────▼───────────┐    │
        │  │  I2C Bus             │    │
        │  │ SDA: 20, SCL: 21     │    │
        │  └──────────┬───────────┘    │
        │             │                │
        └─────────┬───┼────────────────┘
                  │   │
         ┌────────▼─┐ │
         │ L298N    │ │
         │ Driver   │ │
         └────┬─────┘ │
              │       │
    ┌─────────▼──┐    │
    │             │    │
  Motor A      Motor B  MPU6050
  (Left)       (Right)  (IMU)
    │           │       │
    │           │    ┌──┴──┐
    │           │    │     │
  Encoder L  Encoder R  Accel
                       Gyro
```

---

## Assembly Checklist

### Pre-Assembly Verification

- [ ] Arduino Mega powers on (verify USB connection)
- [ ] All motors spin freely (no binding)
- [ ] Encoders are mechanically sound
- [ ] MPU6050 has no corrosion
- [ ] L298N module shows correct pins labeled
- [ ] Battery tests good with multimeter
- [ ] All wires properly gauged (18AWG for motors)

### During Assembly

- [ ] Motor connections soldered (not breadboard)
- [ ] Encoder wires strain-relieved at connector
- [ ] I2C pullup resistors soldered in place
- [ ] Flywheel diodes installed (correct polarity)
- [ ] Emergency stop button mounted securely
- [ ] Status LED visible from above
- [ ] No wire shorts touching chassis

### Post-Assembly Testing

- [ ] Motor A spins with PWM command (forward/backward)
- [ ] Motor B spins with PWM command (forward/backward)
- [ ] Encoder A increments when motor A spins
- [ ] Encoder B increments when motor B spins
- [ ] MPU6050 responds on I2C (use Wire scanner)
- [ ] Emergency stop button cuts power
- [ ] Battery voltage stable under load

---

## Common Wiring Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Motor power inverted | Motors won't run | Reverse + and - connections |
| I2C pullups missing | IMU not responding | Add 4.7k resistors to 3.3V |
| Encoder polarity backwards | Encoder counts wrong | Swap signal wires |
| Motor ground not common | Signals noisy | Connect all grounds together |
| Flywheel diode backwards | Arduino damaged | Reverse diode (band on +) |
| Wrong Arduino pins | Features don't work | Verify pin numbers in code |
| Missing pull-up resistor | Emergency stop always on | Add 10k resistor to 5V |

---

## Testing Wiring Integrity

### Motor Test
```cpp
// In Arduino IDE Serial Monitor (115200 baud)
// Send: M64064  (both motors forward)
// Expected: Both motors spin forward
// Send: M191191 (both motors backward)
// Expected: Both motors spin backward
```

### Encoder Test
```cpp
// Spin motor A by hand
// Expected: Left encoder count increases
// Spin motor B by hand
// Expected: Right encoder count increases
```

### IMU Test
```cpp
// Tilt robot different directions
// Expected: Accelerometer values change
// Spin robot in place
// Expected: Gyroscope Z value changes
```

### Power Supply Test
```
With multimeter:
- Battery: 6.0V nominal
- L298N +12V input: 5.8-6.2V
- Arduino 5V: 4.9-5.1V
- Motor at idle: <100 mA
- Motor spinning: 200-500 mA
```

---

## Troubleshooting Wiring Issues

### Issue: Motors don't move when commanded

**Check**:
1. Battery voltage with multimeter (should be >5.5V)
2. Motor connections solid (manually wiggle, should feel tight)
3. L298N module has +12V and GND connected
4. Direction pins (8, 9, 10, 11) connected
5. PWM pins (5, 6) connected
6. No fuse blown (test with multimeter)

### Issue: Encoders not counting

**Check**:
1. Encoder discs properly mounted on motor shafts
2. Interrupt pins (2, 3) connected correctly
3. Signal rises to 5V (use oscilloscope or logic analyzer)
4. Pullup resistors present (if external encoders)

### Issue: I2C not responding

**Check**:
1. SDA (Pin 20) and SCL (Pin 21) connected to MPU6050
2. I2C pullup resistors installed (must have)
3. Module not powered to 5V (use 3.3V!)
4. No other I2C devices conflicting (remove if present)

### Issue: Unstable motor control

**Check**:
1. Check for wire shorts under robot
2. Verify encoder wires not near power lines
3. Reduce motor PWM values (below 200)
4. Add capacitors across battery (100μF, 10μF)

---

## Wire Color Coding Standard

**Recommended Colors**:
```
Power Signals:
  Red   → +5V or +Battery
  Black → GND / Return
  Orange → +12V (Motor power)

Motor Signals:
  Purple → Motor A Direction
  Gray   → Motor A Direction
  Blue   → Motor B Direction
  Green  → Motor B Direction
  Brown  → Motor A Speed (PWM)
  Yellow → Motor B Speed (PWM)

I2C Signals:
  Green  → SDA (Data)
  Yellow → SCL (Clock)

Encoder Signals:
  White  → Encoder Signal A
  Tan    → Encoder Signal B
```

---

## Parts Required for Wiring

| Item | Quantity | Purpose |
|------|----------|---------|
| 18AWG Stranded Wire | 2m | Motor connections |
| 22AWG Solid Wire | 2m | Breadboard connections |
| Solder + Iron | 1 | Permanent motor connections |
| Breadboard | 1 | Arduino connections |
| Jumper wires | 20+ | Signal connections |
| 4.7k Resistors | 2 | I2C pullups |
| 10k Resistor | 1 | Emergency stop pullup |
| 1N4007 Diodes | 2 | Motor protection |
| Connectors (JST/XT60) | 4 | Battery connection |
| Multimeter | 1 | Testing connections |

---

## Dimension Reference

```
Arduino Mega 2560:
  101.5 × 53.3 mm

L298N Module:
  47 × 42 mm

MPU6050 Breakout:
  ~20 × 16 mm

DC Motor (typical):
  Ø 20 × 20 mm shaft

Breadboard (830 holes):
  ~270 × 67 mm
```

---

## Next Steps

1. **Verify all connections** match this diagram
2. **Flash [arduino_code.ino](arduino_code.ino)** to Arduino
3. **Test with [Testing Guide](testing_guide.md)**
4. **Follow [Motor Calibration](motor_calibration.md)** for speed matching
5. **Proceed to [ROS2 Integration](ros2_integration.md)**

---

**Need help?** Check [Testing Guide](testing_guide.md) for detailed troubleshooting procedures.
