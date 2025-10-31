# Line Following Robot - Bill of Materials

**Total Cost (Recommended)**: $35-50 USD

---

## Quick Summary

| Component | Qty | Cost | Total |
|-----------|-----|------|-------|
| Arduino Uno | 1 | $5-8 | $7 |
| IR Sensor Array (QTR-8A) | 1 | $18-25 | $20 |
| DC Motors (with gearbox) | 2 | $3-5 | $8 |
| Motor Driver (L298N) | 1 | $2-4 | $3 |
| Robot Chassis | 1 | $10-15 | $12 |
| Battery + Wiring | 1 | $5-8 | $7 |
| **TOTAL (Recommended)** | | | **$57** |

---

## Component Details

### Microcontroller

**Arduino Uno (Recommended)**
- Cost: $5-8 (Aliexpress), $12-25 (Arduino official)
- Specs:
  - 14 digital I/O pins
  - 6 analog input pins (A0-A5)
  - 16 MHz processor
  - 2 KB RAM
- Why: Simpler than Mega, enough pins for line follower
- Where: Amazon, Aliexpress, Arduino store

**Arduino Nano (Budget Alternative)**
- Cost: $2-4
- Specs: Same as Uno, smaller form factor
- Why: Lower cost, fits tight spaces
- Trade-off: Soldering required for I/O

**Arduino Mega (Overkill but Works)**
- Cost: $8-12
- Specs: 54 I/O pins, more memory
- Why: Overkill for line follower alone
- Better for: Adding extra sensors later

**Recommendation**: Arduino Uno for beginners

---

### IR Sensor Array

**Option 1: QTR-8A Analog Sensor Array (BEST)**
- Cost: $18-25
- Sensors: 8 IR sensors in one module
- Output: Analog voltage for each sensor
- Interface: 8 analog pins (A0-A7 on Arduino)
- Range: 3-20mm detection distance
- Response time: ~300µs
- Where: Pololu store, Adafruit, Aliexpress
- Pros:
  - Easy to use (plug and play)
  - Good accuracy
  - Built-in LED emitters
  - Calibration routines available
- Cons:
  - Slightly expensive
  - Fixed 8-sensor configuration

**Option 2: QTR-5A (5 Sensor Array)**
- Cost: $12-18
- Sensors: 5 IR sensors
- Output: Analog (A0-A4)
- Pros: Cheaper, still sufficient for most tracks
- Cons: Less lateral coverage

**Option 3: Individual IR Sensor Modules**
- Cost: $1-2 each
- Sensors: Single IR module per unit
- Output: Digital (threshold) or analog (modulated)
- Where: Amazon, Aliexpress, eBay
- Pros:
  - Very cheap
  - Flexible placement
  - Custom array size
- Cons:
  - Require individual wiring
  - Inconsistent calibration
  - More troubleshooting

**Option 4: Vision-Based (USB Camera)**
- Cost: $8-15
- Camera: USB webcam or Pi Camera
- Detection: OpenCV black line detection
- Pros: Robust to lighting, line color detection
- Cons: More CPU power needed, higher latency

**Recommendation**: QTR-8A for reliable performance

---

## Motor System

### DC Motors

**Option 1: 3-6V Geared DC Motor (RECOMMENDED)**
- Cost: $3-5 per motor
- Voltage: 3-6V DC
- Speed: 100-200 RPM (depends on gear ratio)
- Torque: 1-3 kg-cm
- Weight: ~20g per motor
- Where: Amazon, Aliexpress, hobby shops
- Best for: Line followers, good torque/speed balance
- Buy: Matched pair for consistent speeds

**Option 2: High-Speed DC Motor**
- Cost: $2-4
- Speed: 200+ RPM
- Torque: Lower (0.5-1 kg-cm)
- Best for: Lightweight robots, smooth floors
- Trade-off: More likely to slip on carpet

**Option 3: Servo Motor** (Not recommended for wheels)
- Cost: $5-10
- Servo motors are for steering, not continuous rotation
- Use only for gripper or steering rack

**Motor Specifications to Check**:
```
Important Parameters:
  - Operating voltage: Should match battery voltage
  - RPM: 100-150 is good for line following
  - Torque: At least 1 kg-cm to carry robot weight
  - Shaft diameter: Should fit wheel hub
```

**Wheel Selection**:
- Diameter: 6-8 cm (standard for small robots)
- Width: 2-3 cm
- Surface: Rubber (better traction than plastic)
- Hub: Should match motor shaft (3-4mm)
- Cost: Usually included with chassis kit

### Motor Driver

**L298N Dual Motor Driver (RECOMMENDED)**
- Cost: $2-4 (Aliexpress), $5-10 (Amazon)
- Channels: 2 (perfect for 2-motor robot)
- Max current: 2A per motor (sufficient)
- Logic voltage: 5V (Arduino compatible)
- Motor voltage: Up to 12V
- Where: Aliexpress, Amazon, electronics shops
- Connections:
  - IN1, IN2 = Motor A direction
  - IN3, IN4 = Motor B direction
  - ENA, ENB = Motor A, B speed (PWM)
  - +12V = Motor power supply
  - GND = Return path

**L293D (Older Alternative)**
- Cost: $1-2
- Pros: Very common, lots of examples
- Cons: Limited current (600mA), gets hot
- Only use for very lightweight robots

**TB6612 Motor Driver (Premium)**
- Cost: $3-5
- Pros: More efficient, cool operation
- Cons: Overkill for line follower
- Use if: Expanding to more motors later

**Recommendation**: L298N for reliability

---

## Power System

### Battery Options

**Option 1: 4xAA Battery Holder (RECOMMENDED)**
- Voltage: 6V nominal (with alkaline batteries)
- Capacity: 2000-3000 mAh (alkaline)
- Cost: $3-5
- Life: 5-10 hours typical
- Pros: Safe, replaceable, commonly available
- Cons: Heavier than LiPo
- Where: Any electronics store
- Batteries needed: 4x AA (alkaline or NiMH)

**Option 2: 2S LiPo Battery (5-15 minutes)**
- Voltage: 7.4V nominal
- Capacity: 500-1500 mAh
- Cost: $8-15 (including charger)
- Life: 30-60 minutes with 2000mAh
- Pros: Lightweight, high power density
- Cons: Requires charger, fire risk if damaged
- Use with: Voltage regulator to 6V

**Option 3: USB Power Bank (Development Only)**
- Cost: $10-20
- Pros: Convenient, safe
- Cons: Not intended for motors
- Use only for: Testing without motors

**Option 4: 3-4xAA NiMH Rechargeables**
- Voltage: 4.8V or 6.0V
- Capacity: 2000-2500 mAh
- Cost: Batteries $5-8, charger $10-15
- Pros: Reusable, good for environment
- Cons: Lower voltage when depleted

**Recommendation**: 4xAA alkaline holder (safest for learning)

### Power Connectors

**JST Connectors** (Battery to board)
- Cost: $1-2 per set
- Purpose: Easy battery connection
- Types: JST-PH (smaller), JST-XH (larger)
- Where: Any electronics store

**Alligator Clip Wires** (Temporary connections)
- Cost: $2-3
- Purpose: Quick testing without soldering
- Where: Any electronics store

---

## Robot Chassis

### Chassis Kit

**2WD Robot Chassis (All-in-One)**
- Cost: $12-20
- Includes:
  - Plastic base platform
  - 2 motor mounts
  - 2 wheels (6-8cm diameter)
  - 1 caster wheel
  - Battery holder
  - Breadboard mounting area
  - Fasteners
- Where: Amazon, Aliexpress ("2WD robot chassis")
- Popular options:
  - Smart car chassis kit (basic, $12)
  - TT Motor chassis (higher quality, $18)
  - Metal chassis (premium, $25+)

**Materials**:
- Acrylic (clear plastic) - lighter but breakable
- Steel - heavier, more durable
- Aluminum - expensive, professional

### Individual Components (DIY Build)

| Component | Cost | Purpose |
|-----------|------|---------|
| Plastic base | $3 | Robot platform |
| Motor mounts | $2 | Hold motors |
| Wheels (2) | $3 | Motor output |
| Caster wheel | $2 | Stabilization |
| Fasteners | $2 | Assembly |
| **Subtotal** | $12 | Same as kit |

**Recommendation**: Buy pre-made chassis kit (easier assembly)

---

## Wiring & Miscellaneous

### Required

| Component | Qty | Cost | Notes |
|-----------|-----|------|-------|
| Breadboard (830 holes) | 1 | $3-5 | Prototyping |
| Jumper wires | 20+ | $2-3 | Connections |
| Stranded wire (18AWG) | 2m | $2 | Motor wiring |
| USB cable (A to B) | 1 | $2 | Arduino programming |
| Resistors 10k | 4 | $1 | Pull-ups |
| Capacitor 100µF | 1 | $1 | Power smoothing |

### Recommended for Reliability

| Component | Cost | Purpose |
|-----------|------|---------|
| Ferrite beads | $1 | Noise filtering |
| Heatshrink tubing | $2 | Wire protection |
| Diodes 1N4007 | $1 | Motor flywheel protection |
| Fuse 3A | $1 | Safety |
| Soldering iron | $15 | Permanent connections |

---

## Complete BOM by Vendor

### Total Budget Estimate

**Minimal Build** ($28-35):
- Arduino Uno: $5
- QTR-5A (5 sensors): $12
- L298N driver: $2
- Chassis (basic): $10
- Battery + wires: $4
- **Total**: $33

**Standard Build** ($45-57, RECOMMENDED):
- Arduino Uno: $7
- QTR-8A (8 sensors): $20
- L298N driver: $3
- Chassis (quality): $15
- Battery + wires: $7
- Breadboard + jumpers: $5
- **Total**: $57

**Premium Build** ($60-80):
- Arduino Mega: $10
- QTR-8A: $20
- TB6612 driver: $4
- Premium chassis: $20
- LiPo battery + charger: $15
- Quality wiring kit: $10
- **Total**: $79

---

## Purchasing Checklist

```
ELECTRONICS:
□ Arduino Uno                    Cost: $__ Source: _______
□ QTR-8A Sensor Array            Cost: $__ Source: _______
□ L298N Motor Driver             Cost: $__ Source: _______
□ USB Cable Type B               Cost: $__ Source: _______

MECHANICS:
□ 2WD Robot Chassis Kit          Cost: $__ Source: _______
□ Motors (if not in kit)         Cost: $__ Source: _______
□ 4xAA Battery Holder            Cost: $__ Source: _______

WIRING:
□ Breadboard (830-hole)          Cost: $__ Source: _______
□ Jumper Wire Kit (20+)          Cost: $__ Source: _______
□ Stranded Wire (18AWG, 2m)      Cost: $__ Source: _______
□ Resistors 10k (4x)             Cost: $__ Source: _______

TOTAL ESTIMATED COST: $__
```

---

## Quality Assurance Upon Receipt

**Electronics**:
- [ ] Arduino programs via USB
- [ ] QTR sensor module lights up
- [ ] L298N driver powers on
- [ ] Motors spin freely (test with batteries)
- [ ] No visible corrosion or damage

**Mechanical**:
- [ ] Wheels roll freely
- [ ] Motors mount securely
- [ ] Chassis not cracked
- [ ] Caster wheel pivots smoothly

**Batteries**:
- [ ] Test with multimeter (should show >5V)
- [ ] Check expiration date
- [ ] No corrosion on terminals

---

## Storage & Safety

**Electronics**:
- Store in dry location
- Protect from static discharge
- Keep away from moisture

**Batteries**:
- Store in cool place (not hot car)
- Don't leave inserted in robot unused
- Alkaline: Keep original packaging
- NiMH: Store partially charged
- LiPo: Store in fireproof container

**Motors**:
- Protect from dust
- Don't let wires get pinched
- Prevent corrosion

---

## Alternative Suppliers

**Where to Buy**:
| Vendor | Shipping | Price | Selection |
|--------|----------|-------|-----------|
| Aliexpress | 2-4 weeks | Lowest | Good |
| Amazon | 1-2 days | Medium | Excellent |
| Adafruit | 2-3 days | Higher | Excellent |
| eBay | Varies | Variable | Good |
| Local Store | Today | Higher | Limited |

---

## Cost Breakdown Table

```
STANDARD BUILD COST ANALYSIS:

Fixed Costs (Buy Once):
  Arduino Uno:           $7  (reusable for many projects)
  Breadboard:            $3  (reusable)
  USB Cable:             $2  (reusable)
  Jumper Wires:          $3  (reusable)
  Tools (optional):      $20 (optional, one-time)
  ───────────────────────────
  Subtotal:             $35 (many reusable)

Project-Specific Costs:
  QTR-8A Sensor:        $20 (project specific)
  L298N Driver:          $3 (reusable)
  Chassis Kit:          $15 (for this project)
  Motors (in kit):       $0 (included)
  Battery/Holder:        $7 (reusable)
  Wiring:                $2 (consumable)
  ───────────────────────────
  Subtotal:             $47

TOTAL FIRST BUILD:      $82 (with tools)
SECOND BUILD:           $47 (reuse Arduino, etc.)

VALUE: Buy once, build twice!
```

---

## FAQ

**Q: Can I use a different Arduino?**
A: Yes - Uno, Nano, Mega all work. Uno is recommended.

**Q: Is QTR-8A really necessary?**
A: No. QTR-5A or even individual sensors work, but QTR-8A is easier.

**Q: Can I use 2x AA instead of 4x AA?**
A: Technically yes (3V), but motors may be too weak. Try 4 or 3 rechargeable NiMH (4.5V).

**Q: Where's the cheapest place to buy?**
A: Aliexpress typically (but 2-4 week shipping). Amazon if you need it fast.

**Q: Do I need a servo motor for the wheels?**
A: No! Standard DC motors are much better for continuous rotation.

**Q: Can I add a gripper?**
A: Yes, add SG90 servo and small gripper (~$15 more).

---

## Recommended Kits

**If You Want Everything Pre-Packaged**:
- Search "line following robot kit arduino" on Amazon
- Usually $40-60 for complete kit
- Includes: Chassis, motors, sensors, Arduino
- Trade-off: Less flexibility than buying separately

---

## Money-Saving Tips

1. **Buy in bulk**: Multiple robot projects = bulk discounts
2. **Aliexpress deals**: Watch for seasonal sales
3. **Used components**: Check eBay or local maker spaces
4. **Student discounts**: Adafruit offers 10% student discount
5. **Electronics recycling**: Salvage old components

---

## Version Notes

- **Arduino**: Can substitute with Arduino Micro, Nano
- **Sensors**: QTR-5A works if 8 is too expensive
- **Motors**: Any 3-6V geared motor works
- **Chassis**: Any 2WD kit compatible
- **Battery**: 4xAA, 3xAA (NiMH), or 2S LiPo all work

---

**Ready to order?** Check current prices and shipping times before committing!
