# 6-DOF Robotic Arm - Bill of Materials

**Total Cost (Recommended)**: $120-180 USD

---

## Quick Summary

| Component | Qty | Cost | Total |
|-----------|-----|------|-------|
| Servo Motors (High Torque) | 2 | $15 | $30 |
| Servo Motors (Medium) | 3 | $12 | $36 |
| Servo Motor (Light) | 1 | $10 | $10 |
| Arduino Mega | 1 | $12 | $12 |
| PCA9685 Servo Driver | 1 | $5 | $5 |
| 6V Power Supply | 1 | $15 | $15 |
| Mechanical Structure | 1 | $40 | $40 |
| Servo Driver Board + Cables | 1 | $10 | $10 |
| **TOTAL (Recommended)** | | | **$158** |

---

## Servo Motor Selection

### Joint Requirements

| Joint | Purpose | Torque Needed | Recommended Servo | Speed | Cost |
|-------|---------|---------------|-------------------|-------|------|
| J0 (Base Rotation) | Rotate entire arm | Very High (50+ kg-cm) | MG996R or XL430 | 0.17 s/60° | $15 |
| J1 (Shoulder) | Lift arm up/down | Very High (40+ kg-cm) | MG996R or XL320 | 0.18 s/60° | $15 |
| J2 (Upper Arm) | Move forearm up/down | High (30+ kg-cm) | MG995 or XL320 | 0.20 s/60° | $12 |
| J3 (Elbow) | Bend forearm | High (30+ kg-cm) | MG995 or XL320 | 0.20 s/60° | $12 |
| J4 (Wrist Pitch) | Wrist up/down | Medium (20+ kg-cm) | MG995 or XL320 | 0.20 s/60° | $12 |
| J5 (Wrist Roll) | Rotate wrist | Low (15+ kg-cm) | SG90 or XL320 | 0.10 s/60° | $10 |

### Servo Type Comparison

| Type | Cost | Torque | Speed | Durability | Learning Curve |
|------|------|--------|-------|-----------|-----------------|
| Standard Analog (MG995, MG996R) | $10-15 | Good | Moderate | Fair | Easy |
| Digital/DYNAMIXEL (XL320, XL430) | $20-40 | Excellent | Good | Excellent | Harder |
| Budget Servo (SG90, MG90S) | $5-10 | Low | Fast | Poor | Very Easy |

**Recommendation for Learning**: Start with MG995/MG996R analog servos

### Detailed Servo Specifications

**MG996R (High Torque Analog)**
- Torque: 11 kg-cm @ 6V (13 kg-cm @ 7.4V)
- Speed: 0.17 s/60° @ 6V
- Voltage: 4.8-7.2V
- Weight: 55g
- Cost: $12-18
- Best for: Base, shoulder joints
- Where to buy: Amazon, Adafruit

**MG995 (Medium Torque Analog)**
- Torque: 10 kg-cm @ 6V
- Speed: 0.20 s/60°
- Voltage: 4.8-7.2V
- Weight: 55g
- Cost: $8-12
- Best for: Elbow, wrist pitch
- Where to buy: Amazon, eBay

**SG90 (Standard Micro Servo)**
- Torque: 1.6 kg-cm @ 5V
- Speed: 0.12 s/60° @ 5V
- Voltage: 4.8-6V
- Weight: 9g
- Cost: $3-5
- Best for: Wrist roll, light loads
- Where to buy: Any electronics shop

**XL320 (DYNAMIXEL Smart Servo)**
- Torque: 3.7 kg-cm @ 7.4V
- Speed: 0.26 s/60° @ 12V
- Voltage: 6.5-12V
- Weight: 17g
- Cost: $25-35
- Best for: Modular, daisy-chain control
- Where to buy: Robotis store, Adafruit

---

## Microcontroller Options

### Arduino Mega 2560 (Recommended for Beginners)
- **Cost**: $8-15
- **Pros**: Lots of I/O pins, familiar platform, easy to learn
- **Cons**: No built-in servo driver
- **Need**: External PCA9685 servo driver module
- **Where**: Amazon, Arduino store

### Arduino Due (More Powerful)
- **Cost**: $30-40
- **Pros**: 32-bit processor, faster
- **Cons**: Different programming, overkill for 6 servos
- **Where**: Arduino store

### Teensy 3.6 (Professional)
- **Cost**: $25-30
- **Pros**: Very fast, PWM capable
- **Cons**: Smaller community, steeper learning curve
- **Where**: pjrc.com

**Recommendation**: Arduino Mega 2560 for learning

---

## Servo Driver Module

**PCA9685 16-Channel PWM Servo Driver**
- Cost: $3-8 on Aliexpress, $10-15 from Adafruit
- Channels: 16 (can control up to 16 servos)
- Communication: I2C bus
- Operating voltage: 5V logic, separate servo power supply
- Features:
  - Selectable I2C address (daisy-chain multiple boards)
  - Accurate 12-bit resolution
  - Adjustable frequency (24-1600 Hz)
  - Compatible with all analog servo types

**Why PCA9685?**
- Arduino Mega has limited PWM pins (only 6 dedicated PWM)
- PCA9685 provides 16 independent PWM channels via I2C
- Single I2C bus controls all servos
- Separates servo power from Arduino power
- Only 2 Arduino pins needed (SDA, SCL)

**Where to Buy**:
- **Budget**: Aliexpress ($3-5, 2-4 week shipping)
- **Fast**: Amazon ($8-10, 2-day shipping)
- **Trusted**: Adafruit ($15, guaranteed quality)

---

## Power System

### Power Supply Options

**Option 1: 6V Regulated Supply (Recommended)**
- Voltage: 6V DC
- Current: 10A or higher (measure total servo draw)
- Type: Regulated desktop supply
- Cost: $15-25
- Best for: Testing and development
- Where: Electronics stores, Amazon

**Option 2: LiPo Battery Pack**
- Voltage: 2S LiPo = 7.4V nominal
- Capacity: 2200 mAh
- Cost: $20-30
- Best for: Mobile operation
- Need: Charger ($10-15)

**Option 3: Multiple NiMH Batteries**
- Type: 4-6 AA NiMH batteries in holder
- Voltage: 4.8-7.2V
- Cost: $10-15
- Best for: Budget option

**Power Calculation**:
```
Typical servo consumption:
  At rest: 0-5 mA per servo
  Moving (no load): 100-200 mA per servo
  Stalled: 500-1000 mA per servo

Total for 6 servos:
  Idle: ~30 mA
  Moving: ~900 mA
  Peak (stall): ~3-4 A

Recommendation: 10A power supply minimum
```

---

## Mechanical Structure

### Option 1: 3D-Printed (Best for Learning)
- **Cost**: $30-50 in filament
- **Time**: 15-20 hours print time
- **Materials**: PLA or PETG filament
- **Design Sources**:
  - Thingiverse.com
  - Printables.com
  - GitHub robotics projects
- **Advantages**: Customizable, easy to iterate
- **Disadvantages**: Requires printer, print time

**Popular Designs**:
- "Arduino 6-DOF Robotic Arm" by Dejan (Thingiverse)
- "PhantomX Reactor Arm" (open-source)
- "Simple 6-DOF arm" various makers

### Option 2: Aluminum Extrusion Kit
- **Cost**: $80-150
- **Accuracy**: High (CNC machined)
- **Assembly**: More complex (nuts, bolts, alignment)
- **Design**: Professional appearance
- **Examples**: UR5e-style kits

### Option 3: Commercial Kit
- **Cost**: $200-400
- **Included**: Everything pre-assembled
- **Support**: Full documentation
- **Examples**: Dobot M1, Arduino Braccio+

**Recommendation**: 3D-printed design for learning

---

## Detailed Component List

### Electronics Core ($35-50)

| Component | Qty | Cost | Notes |
|-----------|-----|------|-------|
| Arduino Mega 2560 | 1 | $12 | Microcontroller |
| PCA9685 Servo Driver | 1 | $5 | I2C PWM controller |
| USB Cable (Type B) | 1 | $2 | Programming |
| I2C Cables (4-wire) | 2 | $3 | Connect to PCA9685 |
| Breadboard | 1 | $5 | Prototyping |
| Jumper Wires | 20 | $3 | Connections |
| **Subtotal** | | **$30** | |

### Servo Motors ($94-130)

| Servo | Qty | Cost | Total | Purpose |
|-------|-----|------|-------|---------|
| MG996R (High Torque) | 2 | $15 | $30 | Base, Shoulder |
| MG995 (Medium Torque) | 3 | $12 | $36 | Elbow, Wrist, Arm |
| SG90 (Light Duty) | 1 | $8 | $8 | Wrist Roll |
| **Subtotal** | | | **$74** | |

### Power System ($25-40)

| Component | Qty | Cost | Notes |
|-----------|-----|------|-------|
| 6V Power Supply (10A) | 1 | $20 | Regulated supply |
| Power Distribution Board | 1 | $3 | Split power rails |
| Capacitor (100µF) | 1 | $1 | Power smoothing |
| Fuse & Holder | 1 | $2 | Safety |
| **Subtotal** | | **$26** | |

### Mechanical & Hardware ($30-50)

| Component | Qty | Cost | Notes |
|-----------|-----|------|-------|
| 3D-Printed Parts (filament) | 1 kg | $20 | PLA or PETG |
| Fasteners (screws, nuts, washers) | Assorted | $5 | Various sizes |
| Servo brackets/mounts | 6 | $3 | Servo attachment |
| Coupling pieces | 4 | $2 | Joint connections |
| **Subtotal** | | **$30** | |

---

## Optional Components

### Enhanced Functionality

| Component | Cost | Purpose |
|-----------|------|---------|
| Magnetic gripper | $20 | Pick up objects |
| 2-finger gripper | $15 | Precision gripping |
| Camera | $15 | Computer vision |
| Proximity sensor | $3 | Object detection |
| LED status light | $2 | Visual feedback |

### Improved Control

| Component | Cost | Purpose |
|-----------|------|---------|
| Servo feedback sensor | $10 | Joint angle feedback |
| Current sensor | $5 | Monitor servo load |
| Voltage regulator | $3 | Stable 5V for logic |
| Terminal blocks | $3 | Easier connections |

---

## Sourcing Guide

### Best Suppliers for 6-DOF Arm

**Aliexpress** ($80-100 total)
- Pros: Lowest prices
- Cons: 2-4 week shipping, no returns
- Good for: Entire BOM bulk order
- Popular: Search "servo motor combo lot"

**Amazon** ($120-150 total)
- Pros: Fast shipping, easy returns
- Cons: Higher prices
- Good for: Emergency parts, quick start
- Filter: "Prime eligible"

**Adafruit** ($150-180 total)
- Pros: Quality guaranteed, tutorials included
- Cons: Highest prices
- Good for: Learning, support, known compatibility
- Site: adafruit.com

**Local Electronics Store**
- Pros: See components, immediate purchase
- Cons: Limited selection, higher prices
- Good for: Testing before committing

**Maker Spaces**
- Pros: Free/cheap used components
- Cons: May not have all parts
- Good for: Budget-conscious, community

---

## Complete BOM Template

**For Your 6-DOF Arm Project**:

```
SERVO MOTORS:
□ MG996R × 2         @ $__ each = $__
□ MG995 × 3          @ $__ each = $__
□ SG90 × 1           @ $__ each = $__

CONTROL ELECTRONICS:
□ Arduino Mega       @ $__ = $__
□ PCA9685 Driver     @ $__ = $__
□ I2C Cables         @ $__ = $__
□ USB Cable          @ $__ = $__

POWER:
□ 6V 10A Supply      @ $__ = $__
□ Power Board        @ $__ = $__
□ Capacitors         @ $__ = $__
□ Fuse               @ $__ = $__

MECHANICAL:
□ 3D Filament 1kg    @ $__ = $__
□ Fasteners Kit      @ $__ = $__
□ Servo Mounts       @ $__ = $__

OPTIONAL:
□ Gripper            @ $__ = $__
□ Feedback Sensors   @ $__ = $__

                    TOTAL: $__
```

---

## Cost Optimization Strategies

### Budget Build (~$100)
- Use 3D-printed parts (filament cost: $20)
- Use SG90 servos where possible (but insufficient torque)
- Use USB-powered Arduino (simpler)
- Skip gripper initially

### Standard Build (~$150)
- Mix servo types (high torque + medium)
- Dedicated 6V supply
- 3D printed structure
- Basic gripper

### Premium Build (~$250+)
- All DYNAMIXEL smart servos
- Aluminum structure
- Advanced 2-3 finger gripper
- Feedback sensors
- Computer vision

---

## Where to Buy Checklist

```
□ Search part name on Aliexpress (cheapest)
□ Compare with Amazon Prime (fast)
□ Check Adafruit (trusted)
□ Visit local maker space (free/cheap)
□ Order from most cost-effective + reliable combo

Example:
  Servos: Aliexpress ($40, 3 weeks)
  Arduino: Amazon ($12, 2 days)
  Power: Local store ($15, today)
```

---

## Quality Assurance

### Upon Receipt

- [ ] Count all components against BOM
- [ ] Test servo motor rotation (no servo, just power)
- [ ] Verify Arduino USB connection
- [ ] Test I2C with scanner sketch
- [ ] Check 3D parts for defects
- [ ] Inspect fasteners for corrosion

### Before Assembly

- [ ] Servo horns move smoothly
- [ ] Arduino programs correctly
- [ ] PCA9685 responds to I2C commands
- [ ] No loose components or debris

---

## Version History

| Component | Notes |
|-----------|-------|
| Servo motors | Prices vary, substitute models with similar specs |
| Power supply | 6V, 10A minimum |
| Arduino | Can use Due or Teensy for advanced features |
| Driver board | PCA9685 is standard, other I2C PWM boards work |

---

**Ready to order?** Check current prices on your favorite supplier!
