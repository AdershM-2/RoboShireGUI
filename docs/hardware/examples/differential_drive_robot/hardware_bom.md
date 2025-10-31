# Differential Drive Robot - Bill of Materials (BOM)

**Last Updated**: 2025-10-30
**Total Cost (Recommended)**: $50-75 USD

---

## Quick Summary

| Category | Count | Unit Cost | Total |
|----------|-------|-----------|-------|
| Microcontroller (Arduino Mega) | 1 | $8-12 | $10 |
| Motor Driver (L298N) | 1 | $2-5 | $3 |
| DC Motors | 2 | $3-8 | $8 |
| Encoders | 2 | $3-5 | $6 |
| IMU (MPU6050) | 1 | $2-4 | $3 |
| Robot Chassis | 1 | $12-20 | $15 |
| Battery + Cables | 1 | $8-15 | $10 |
| Breadboard + Wires | 1 | $5-10 | $7 |
| **TOTAL (Recommended)** | - | - | **$62** |

---

## Detailed Component List

### 1. Microcontroller

**Primary Option: Arduino Mega 2560**
- **Part**: Arduino Mega 2560 Rev3
- **Quantity**: 1
- **Cost**: $8-12 USD
- **Where to Buy**:
  - Amazon: Search "Arduino Mega 2560"
  - Arduino.cc: Official store
  - AliExpress: Budget option (~$7)
- **Why**: 54 I/O pins, enough for motors + encoders + IMU
- **Alternative**: Arduino Uno ($5) - fewer pins but works for basic setup

**Specifications**:
- Processor: ATmega2560
- Clock Speed: 16 MHz
- RAM: 8 KB
- Flash: 256 KB
- I/O Pins: 54 digital, 16 analog
- I2C Pins: SDA=20, SCL=21

### 2. Motor Driver

**Primary: L298N Dual Motor Driver**
- **Part**: L298N H-Bridge Motor Driver
- **Quantity**: 1
- **Cost**: $2-5 USD
- **Where to Buy**:
  - Amazon: ~$4
  - AliExpress: ~$2 (with shipping)
  - SparkFun/Adafruit: ~$10 (premium)
- **Why**: Controls 2 DC motors independently, easy to use

**Specifications**:
- Max current: 3A (continuous), 5A (peak)
- Operating voltage: 5-35V (motor supply)
- Logic voltage: 5V (Arduino compatible)
- Outputs: 4 (2 for each motor)

**Alternative Options**:
- TB6612 Motor Driver ($3) - More efficient, smaller
- DRV8833 Dual Motor Driver ($5) - Lower current
- Victor SPX ($30+) - Professional/competition robots

**Pin Connections (Mega)**:
```
L298N Pin | Arduino Mega | Purpose
----------|--------------|--------
IN1       | Pin 8        | Left motor direction
IN2       | Pin 9        | Left motor direction
IN3       | Pin 10       | Right motor direction
IN4       | Pin 11       | Right motor direction
ENA       | Pin 5        | Left motor speed (PWM)
ENB       | Pin 6        | Right motor speed (PWM)
GND       | GND          | Ground
12V       | Battery+     | Motor power
```

### 3. DC Motors

**Primary: 3-6V DC Motor with Gearbox**
- **Quantity**: 2 (identical)
- **Cost**: $3-8 each ($6-16 total)
- **Where to Buy**:
  - Amazon: Search "DC motor 100 rpm gearbox"
  - Aliexpress: Cheapest option
  - Adafruit: Higher quality
- **Why**: Gearbox provides torque for robot weight + wheel

**Specifications (Recommended)**:
- Voltage: 3-6V (7.4V LiPo compatible)
- Speed: 50-150 RPM (slower = more torque)
- Torque: >1 kg-cm (depends on gearbox)
- Shaft diameter: 3-4mm
- Size: Small enough to fit chassis

**Motor Selection Guide**:

| RPM | Use Case | Pros | Cons |
|-----|----------|------|------|
| 30-50 | Heavy robot | Strong, slow | Low max speed |
| 75-150 | Balanced | Good compromise | Need good battery |
| 200+ | Light/fast | Quick response | Weak torque |

**Recommended**: 100 RPM geared motor (~$6 pair)

**Where to Buy**:
- **Adafruit**: "Micro Gearmotor" ($5-8 each)
- **Amazon**: "DC motor gearbox 100 rpm" ($3-5 each)
- **Local**: Hobby shops often have 12V geared motors
- **Alternative**: Use 24-gear motor + microcontroller speed control

**Electrical Specs**:
- Supply voltage: 3-6V DC
- Current draw: 100-300mA per motor (nominal)
- Stall current: 500-800mA (when stuck)
- Max continuous current: 1A per motor

### 4. Wheel Encoders

**Primary: Quadrature Encoder Kit**
- **Quantity**: 2 (one per motor)
- **Cost**: $3-6 each ($6-12 total)
- **Where to Buy**:
  - Adafruit: Encoder kit with mounting ($5 each)
  - Amazon: "Rotary encoder module" ($3 each)
  - Aliexpress: Bulk option ($2 each)

**Specifications**:
- Type: Quadrature (optical or mechanical)
- Pulsesper revolution: 20-30 typical
- Channels: 2 (A and B)
- Voltage: 3.3V or 5V compatible

**What They Do**:
- Measure wheel rotation
- Count distance traveled
- Detect speed in real-time
- Enable odometry calculations

**Pin Connections (Mega)**:
```
Encoder A | Encoder B | Arduino Pin | Purpose
----------|-----------|-------------|--------
Signal A  | Signal A  | Pin 2       | Left motor interrupt
Signal B  | Signal B  | Pin 3       | Left motor interrupt
Signal A  | Signal A  | Pin 21      | Right motor interrupt
Signal B  | Signal B  | Pin 20      | Right motor interrupt
GND       | GND       | GND         | Ground
VCC       | VCC       | 5V          | Power
```

**Alternative Encoders**:
- Hall effect sensors ($1) - Magnetic pulses
- Reflective optical ($2) - Paper disc with IR
- Absolute encoders ($10+) - High precision

### 5. IMU Sensor (Inertial Measurement Unit)

**Primary: MPU6050**
- **Quantity**: 1
- **Cost**: $2-4 USD
- **Where to Buy**:
  - Amazon: ~$3
  - Aliexpress: ~$2
  - Adafruit: ~$15 (premium)
- **Why**: Measures acceleration (3-axis) and rotation (3-axis), enables sensor fusion

**Specifications**:
- Gyroscope: ±250°/s to ±2000°/s
- Accelerometer: ±2g to ±16g
- Communication: I2C (400 kHz typical)
- Address: 0x68 (default) or 0x69 (if AD0 pin high)
- Voltage: 3.3V (with 5V tolerant I2C)

**What It Does**:
- Measures angular velocity (gyroscope)
- Measures linear acceleration (accelerometer)
- Combines for orientation tracking
- Reduces odometry drift over time

**I2C Connections (Mega)**:
```
MPU6050 Pin | Arduino Pin | Purpose
------------|-------------|--------
VCC         | 3.3V        | Power
GND         | GND         | Ground
SDA         | Pin 20      | I2C data
SCL         | Pin 21      | I2C clock
```

**I2C Pullup Resistors**:
- Required: 4.7k resistors from SDA and SCL to 3.3V
- Or use 10k resistors to 5V (voltage divider needed)
- Many breakout boards have built-in pullups

**Alternative IMUs**:
- BNO055 ($8-15) - Better, includes magnetometer
- ICM-20689 ($5) - Similar to MPU6050
- LSM6DSOX ($10+) - High-end option

### 6. Robot Chassis

**Primary: 2-Wheel Robot Chassis Kit**
- **Quantity**: 1
- **Cost**: $12-20 USD
- **Where to Buy**:
  - Amazon: "2WD robot chassis kit" ($15-20)
  - Adafruit: "Robot chassis kit" ($25)
  - Aliexpress: Budget option ($12)
  - Local maker spaces: Often have spare chassis

**What's Included**:
- Plastic chassis base
- 2 motor mounts
- 2 wheels (typically ~7cm diameter)
- 1 caster wheel
- Screws and hardware
- Breadboard mounting slots

**Specifications**:
- Base size: ~15cm x 10cm
- Weight capacity: 500g-1kg
- Wheel size: 6-8cm diameter
- Motor mounting: Universal servo mounts

**Alternative Chassis Options**:

| Option | Cost | Size | Quality |
|--------|------|------|---------|
| Budget kit | $12 | Small | Plastic, may wobble |
| Standard kit | $18 | Medium | Good for learning |
| Premium kit | $30+ | Varies | Metal, modular |
| DIY (cardboard) | $0 | Custom | Prototyping only |

**Where to Source**:
- **Amazon**: "2WD robot chassis" (quick delivery)
- **Aliexpress**: Cheapest but slow shipping
- **Adafruit**: Quality assurance, fast shipping
- **Local maker space**: Often free or cheap used chassis

### 7. Power System

**Battery Options**:

#### Option A: 4x AA Battery Holder (Recommended for Learning)
- **Voltage**: 6V nominal (4 x 1.5V)
- **Capacity**: 2000-3000 mAh
- **Cost**: $5-8
- **Runtime**: 1-2 hours
- **Pros**: Safe, standard batteries, easy to replace
- **Cons**: Heavier, lower power density

#### Option B: 7.4V LiPo Battery Pack
- **Voltage**: 7.4V (2S LiPo)
- **Capacity**: 1500-2000 mAh
- **Cost**: $8-15
- **Runtime**: 2-4 hours
- **Pros**: Lighter, more power, good performance
- **Cons**: Requires special charger, fire hazard if damaged

#### Option C: USB Power Bank (Development Only)
- **Voltage**: 5V (single USB output)
- **Capacity**: 10,000 mAh typical
- **Cost**: $10-20
- **Runtime**: 4-8 hours
- **Pros**: Convenient, safe, standard format
- **Cons**: Lower current (may brownout motors)

**Recommended**: 4xAA holder for beginners, LiPo for advanced

**Supporting Components**:
- **XT60 Connectors** ($2) - Easy motor power connection
- **JST Connectors** ($2) - For battery to Arduino
- **Power Distribution Board** ($3) - Splits battery to multiple devices
- **Voltage Regulator** ($2) - If using 12V battery for 5V logic

**Power Budget Calculation**:
```
Typical consumption:
- Arduino: 50 mA
- Motor Driver: 10 mA
- Single motor: 100-300 mA (depends on load)
- IMU: 5 mA

Total: ~400-600 mA at idle, 1-2A when moving

With 2000 mAh battery:
- Runtime = 2000 mAh / 1000 mA = 2 hours average
- Peak usage = 3A for 30 min
```

### 8. Wiring & Breadboarding

**Required**:
- **Breadboard**: Large 830-hole ($5-8)
- **Jumper Wire Kit**: 140pc assorted ($5-8)
- **USB Cable**: Type B for Arduino ($2)

**Recommended**:
- **Solid Core Wire**: 22AWG for breadboard ($3)
- **Stranded Wire**: 18AWG for motors ($3-5)
- **Wire Stripper**: Automatic ($5-10)
- **Multimeter**: Digital ($8-15)

**Safety**:
- **Resistor Pack**: 100, 220, 1k, 10k ohm ($3)
- **Diodes**: 1N4007 for flywheel protection ($2)
- **Fuse**: Mini blade 3A ($2)

**Connector Options**:
- **Alligator Clips**: Quick testing ($3)
- **XT60 Connectors**: Heavy duty battery ($2)
- **JST Connectors**: Motor connectors ($1)

---

## Complete Build Package List

### Minimal Build ($40-50)
Perfect for learning, but motors may struggle:
- Arduino Mega: $10
- L298N Driver: $3
- 2x DC Motors (60 RPM): $6
- Robot chassis: $15
- 4xAA battery holder: $5
- Breadboard + wires: $8
- Encoder simulator (software only): $0
- **Total**: $47

### Recommended Build ($60-75)
Full functionality, good performance:
- Arduino Mega: $10
- L298N Driver: $3
- 2x DC Motors (100 RPM geared): $10
- Encoders (2x): $8
- MPU6050 IMU: $3
- Robot chassis: $15
- 2S LiPo battery + charger: $15
- Breadboard + quality wires: $10
- **Total**: $74

### Premium Build ($100+)
Professional-grade components:
- Arduino Mega or STM32: $15
- TB6612 Motor Driver: $5
- 2x Quality 150 RPM motors: $20
- Precision quadrature encoders: $15
- BNO055 9-axis IMU: $15
- Custom 3D-printed chassis: $20
- 3S LiPo battery + charger: $30
- Professional wiring kit: $20
- **Total**: $140+

---

## Supplier Recommendations

### Best Overall Value
**Amazon**:
- Fast shipping (Prime eligible)
- Easy returns
- Slightly higher prices
- Good for rapid prototyping

### Budget Option
**Aliexpress**:
- Lowest prices (30-50% off)
- Slow shipping (2-4 weeks)
- Check seller reviews carefully
- Good for bulk orders

### Educational / Quality
**Adafruit**:
- Premium quality components
- Excellent documentation
- Educational resources
- Higher prices but worth it

### Specialized Electronics
**SparkFun**:
- Large parts selection
- Good for specific components
- Fast shipping
- Professional grade

### Local Options
**Electronics Hobby Shops**:
- See components in person
- Immediate availability
- Support local business
- Often higher prices

---

## Estimated Total Cost Breakdown

```
Component Costs:
  Electronics (MCU, Driver, IMU):     $20
  Motors & Gearbox:                   $10
  Encoders (optional):                $6
  Power System:                       $10
  Chassis & Wheels:                   $15
  Breadboard & Wires:                 $8
  Miscellaneous (connectors, etc):    $3
  ─────────────────────────────────────
  TOTAL:                              $72

Cost Reduction Strategies:
  - Skip encoders initially: -$6 = $66
  - Use Arduino Uno: -$3 = $69
  - Use AA battery holder: -$5 = $67
  - Build own chassis (cardboard): -$15 = $57

Cost Increase Options:
  - Upgrade to LiPo: +$10 = $82
  - Add second microcontroller: +$10 = $82
  - Precision encoders: +$8 = $80
```

---

## Purchasing Tips

1. **Check Compatibility**: Verify motor shaft size matches wheel hub
2. **Read Reviews**: Look for complaints about dead-on-arrival parts
3. **Shipping Time**: Budget 2-4 weeks for Aliexpress, 2 days for Amazon Prime
4. **Bundle Deals**: Look for "robot kits" that bundle several items
5. **Surplus Parts**: Check if your school/maker space has spare motors
6. **Used Parts**: eBay often has deals on used components
7. **Bulk Discounts**: If buying for a class, negotiate with suppliers

---

## Quality Assurance

### Upon Receipt, Check:
- [ ] All components listed in BOM received
- [ ] No visible damage or corrosion
- [ ] Motors spin freely
- [ ] Wheels not bent
- [ ] Connector pins not bent
- [ ] Breadboard contacts clean
- [ ] Cable insulation intact

### Before Assembly:
- [ ] Test each motor with 4.5V battery
- [ ] Test encoders with Arduino sketch
- [ ] Test IMU I2C communication
- [ ] Test L298N driver with known-good motor
- [ ] Measure component dimensions

---

## Alternative Components Chart

| Component | Recommended | Budget | Premium |
|-----------|-------------|--------|---------|
| Microcontroller | Mega $10 | Uno $5 | STM32 $15 |
| Motor Driver | L298N $3 | L293D $2 | TB6612 $5 |
| Motor (2x) | 100 RPM $10 | 60 RPM $6 | Servo $30 |
| Encoder (2x) | Quad $8 | Hall $4 | Absolute $20 |
| IMU | MPU6050 $3 | None $0 | BNO055 $15 |
| Chassis | Kit $15 | DIY $0 | Custom $40 |
| Power | AA holder $5 | USB $5 | LiPo $15 |

---

## Shipping & Delivery Times

| Supplier | Shipping Time | Cost | Notes |
|----------|---------------|------|-------|
| Amazon Prime | 1-2 days | Free | Expensive items |
| Amazon Standard | 5-7 days | Free | Standard items |
| Aliexpress | 14-30 days | Free | Bulk discounts |
| DHL Express | 3-5 days | $10+ | Speed option |
| Local Pickup | Same day | $0 | Check availability |

---

## Custom Build Configuration Tool

Use this template to calculate YOUR specific cost:

```
Your Robot Configuration:
  Microcontroller: __________ × $__ = $__
  Motor Driver: __________ × $__ = $__
  Motors: __________ × $__ = $__
  Encoders: __________ × $__ = $__
  IMU: __________ × $__ = $__
  Chassis: __________ × $__ = $__
  Battery: __________ × $__ = $__
  Wiring: __________ × $__ = $__
  ─────────────────────────────────────
  SUBTOTAL: $__
  Shipping: + $__
  Tax (est.): + $__
  ═════════════════════════════════════
  TOTAL: $__
```

---

## Money-Saving Tips

1. **Bulk Ordering**: Buy 2-3 robots worth, share cost with friend
2. **Seasonal Sales**: Order before holidays when prices drop
3. **Free Shipping**: Combine items to reach free shipping threshold
4. **Open Box Deals**: Amazon returns at discount
5. **Student Discounts**: Adafruit offers 10% student discount
6. **Educational Pricing**: Schools may get bulk discounts
7. **Kit Deals**: "Robot kits" often cheaper than buying separately
8. **Local Hackerspaces**: Borrow or buy used equipment cheap

---

## Common BOM Mistakes to Avoid

1. **Wrong motor voltage**: 12V motor with 5V Arduino = disaster
2. **Insufficient current**: USB power bank can't drive two motors
3. **I2C pullup missing**: IMU won't work without 4.7k resistors
4. **Wrong connector**: XT60 male/female mismatch
5. **Under-sized wire**: 28AWG for motor power = fire hazard
6. **Forgot encoders**: Odometry impossible without them
7. **Dead battery**: Always test battery before assembly
8. **No flywheel diode**: Motor spikes damage Arduino

---

## Warranty & Returns Policy

**Amazon**:
- 30-day return window
- Full refund, even if opened
- No questions asked for defective items

**Aliexpress**:
- Buyer Protection: 90 days
- File dispute if not received
- Return shipping usually on you
- Expect 30-day resolution

**Adafruit**:
- 1-year warranty on defects
- Friendly customer service
- Return shipping prepaid
- "Satisfaction guaranteed"

---

## BOM Tracking Checklist

Print this and check off as you receive items:

```
□ Arduino Mega 2560         Cost: $__  Received: __/__
□ L298N Motor Driver        Cost: $__  Received: __/__
□ Motor #1                  Cost: $__  Received: __/__
□ Motor #2                  Cost: $__  Received: __/__
□ Encoder #1                Cost: $__  Received: __/__
□ Encoder #2                Cost: $__  Received: __/__
□ MPU6050 IMU               Cost: $__  Received: __/__
□ Robot Chassis             Cost: $__  Received: __/__
□ Battery + Holder          Cost: $__  Received: __/__
□ Breadboard                Cost: $__  Received: __/__
□ Jumper Wires              Cost: $__  Received: __/__
□ USB Cable                 Cost: $__  Received: __/__

Total Spent: $__
Date Completed: __/__/__
```

---

**Ready to order?** Start with the supplier links and check current prices!
