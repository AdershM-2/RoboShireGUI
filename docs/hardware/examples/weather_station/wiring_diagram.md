# Weather Station - Wiring Diagram

**Difficulty**: Beginner
**Time**: 15-20 minutes
**Tools**: None (no soldering required)

---

## Overview

This guide shows how to connect all weather station components to the ESP32 using a breadboard. Follow the color-coded wiring carefully.

### Components to Connect
1. ESP32 DevKit (main controller)
2. DHT22 Temperature/Humidity Sensor
3. BMP280 Pressure Sensor
4. Photoresistor (LDR) + 10kΩ Resistor

---

## Complete Wiring Diagram (ASCII Art)

```
                    ESP32 DevKit
              ┌─────────────────────┐
              │  ┌───────────────┐  │
     USB ────>│  │               │  │
              │  │   ESP32       │  │
              │  │   WROOM-32    │  │
              │  │               │  │
              │  └───────────────┘  │
              │                     │
    DHT22 ────┤ GPIO 4              │
              │                     │
    SDA   ────┤ GPIO 21 (I2C)       │──── BMP280 (I2C)
    SCL   ────┤ GPIO 22 (I2C)       │
              │                     │
    LDR   ────┤ GPIO 34 (ADC)       │
              │                     │
    3.3V  ────┤ 3V3                 │──── to sensors
    GND   ────┤ GND                 │──── to sensors
              │                     │
              └─────────────────────┘
```

---

## Breadboard Layout

```
                        Breadboard
    ┌────────────────────────────────────────────────┐
    │  +=======+   +=======+   +=======+             │
    │  | Power |   | Sensor|   | Sensor|             │
    │  | Rails |   |  #1   |   |  #2   |             │
    │  +=======+   +=======+   +=======+             │
    │    │ │          │           │                   │
    │  + Red       DHT22       BMP280                │
    │  - Blue                                         │
    │    │ │                                          │
    │  ┌─┴─┴───────────────────────────────┐         │
    │  │                                   │         │
    │  │         ESP32 DevKit              │         │
    │  │      (plugged into middle)        │         │
    │  │                                   │         │
    │  └───────────────────────────────────┘         │
    │                                                 │
    │    [10kΩ]  [LDR]  <-- Light sensor circuit     │
    │                                                 │
    └────────────────────────────────────────────────┘

Legend:
+ Red   = Positive power rail (3.3V)
- Blue  = Ground rail (GND)
```

---

## Step-by-Step Wiring

### Step 1: Prepare the Breadboard

1. **Place ESP32** in the middle of breadboard
   - Straddle the center gap
   - Make sure all pins have accessible holes on both sides
   - Note which row is GND and which is 3V3

2. **Identify power rails**
   - Red line = Positive (+)
   - Blue/Black line = Ground (-)

### Step 2: Connect Power Rails

**ESP32 to Breadboard Power**:
```
ESP32 Pin    →  Breadboard
---------       ----------
3V3          →  Red (+) rail
GND          →  Blue (-) rail
```

**Jumper wires**:
- Use RED wire for 3.3V
- Use BLACK wire for GND
- Keep wires short to avoid clutter

**Photo placeholder**: *Power rails connected to ESP32*

---

### Step 3: Connect DHT22 Sensor

**DHT22 Pinout** (looking at front with holes):
```
    ┌─────────────┐
    │             │
    │    DHT22    │
    │  ┌───────┐  │
    │  │ ##### │  │ <-- White grid (front)
    │  └───────┘  │
    └─────────────┘
     │   │   │   │
     1   2   3   4
    VCC DATA NC GND

Pin 1: VCC (Power) - Red wire
Pin 2: DATA (Signal) - Yellow wire
Pin 3: NC (Not Connected) - Leave empty
Pin 4: GND (Ground) - Black wire
```

**3-Pin Module** (if you have the module version):
```
    ┌─────────────┐
    │    DHT22    │
    │   MODULE    │
    └─────────────┘
     │   │   │
    VCC DATA GND
```

**Connections**:
```
DHT22 Pin     Wire Color    →  ESP32 Pin
---------     ----------        ---------
VCC (1)       Red           →  3V3
DATA (2)      Yellow        →  GPIO 4
GND (4)       Black         →  GND
```

**Pin Assignment Table**:
| DHT22 | ESP32 | Wire Color |
|-------|-------|------------|
| VCC   | 3V3   | Red        |
| DATA  | GPIO 4 | Yellow    |
| GND   | GND   | Black      |

**Notes**:
- If using 4-pin sensor, leave pin 3 (NC) unconnected
- Some DHT22 modules have built-in pull-up resistor, no external needed
- If bare sensor, add 10kΩ resistor between VCC and DATA

**Photo placeholder**: *DHT22 wired to ESP32*

---

### Step 4: Connect BMP280 Sensor

**BMP280 Module Pinout**:
```
    ┌─────────────┐
    │   BMP280    │
    │   MODULE    │
    └─────────────┘
     │ │ │ │ │ │
     1 2 3 4 5 6

Pin 1: VCC (3.3V)
Pin 2: GND (Ground)
Pin 3: SCL (I2C Clock)
Pin 4: SDA (I2C Data)
Pin 5: CSB (Chip Select - leave unconnected for I2C)
Pin 6: SDO (Address Select - connect to GND for 0x76)
```

**Connections**:
```
BMP280 Pin    Wire Color    →  ESP32 Pin
----------    ----------        ---------
VCC           Red           →  3V3
GND           Black         →  GND
SCL           Green         →  GPIO 22 (SCL)
SDA           Blue          →  GPIO 21 (SDA)
SDO           Black         →  GND (sets I2C address to 0x76)
```

**Pin Assignment Table**:
| BMP280 | ESP32 | Wire Color | Notes |
|--------|-------|------------|-------|
| VCC    | 3V3   | Red        | Power |
| GND    | GND   | Black      | Ground |
| SCL    | GPIO 22 | Green    | I2C Clock |
| SDA    | GPIO 21 | Blue     | I2C Data |
| SDO    | GND   | Black      | Address = 0x76 |
| CSB    | -     | -          | Leave unconnected |

**I2C Address Configuration**:
- **SDO connected to GND**: Address = 0x76 (recommended)
- **SDO connected to VCC**: Address = 0x77 (alternate)
- If multiple BMP280 on same bus, use different addresses

**Photo placeholder**: *BMP280 wired to ESP32*

---

### Step 5: Connect Light Sensor (LDR)

**LDR + Resistor Circuit** (Voltage Divider):
```
    3.3V (from ESP32)
      │
      │
    [10kΩ Resistor]
      │
      ├──────────> to ESP32 GPIO 34 (ADC)
      │
    [LDR Photoresistor]
      │
      │
    GND (from ESP32)
```

**Physical Layout on Breadboard**:
```
Row 1:  3.3V rail  ───┐
Row 2:  10kΩ resistor ┤ (one leg)
Row 3:  10kΩ resistor ┴ (other leg)
Row 4:  LDR ──────────┬ (one leg)
Row 5:  Wire to GPIO 34 (from Row 3-4)
Row 6:  LDR ──────────┤ (other leg)
Row 7:  GND rail ─────┘
```

**Connections**:
```
Component           →  Connection
---------              ----------
10kΩ Resistor (leg 1) → 3.3V rail (Red)
10kΩ Resistor (leg 2) → Middle point (Row 3)
LDR (leg 1)           → Middle point (Row 3)
LDR (leg 2)           → GND rail (Black)
Wire from Row 3       → ESP32 GPIO 34
```

**Pin Assignment**:
| Component | ESP32 Pin | Notes |
|-----------|-----------|-------|
| 10kΩ → LDR junction | GPIO 34 | ADC1 Channel 6 |

**Why this circuit works**:
- Voltage divider creates variable voltage based on light
- More light → LDR resistance drops → higher voltage at GPIO 34
- Less light → LDR resistance increases → lower voltage at GPIO 34
- ESP32 ADC reads voltage (0-3.3V) as value (0-4095)

**Photo placeholder**: *LDR voltage divider on breadboard*

---

## Complete Pin Assignment Table

| Sensor | Signal | ESP32 Pin | GPIO # | Protocol | Wire Color |
|--------|--------|-----------|--------|----------|------------|
| DHT22  | DATA   | GPIO 4    | 4      | 1-Wire   | Yellow     |
| BMP280 | SDA    | GPIO 21   | 21     | I2C      | Blue       |
| BMP280 | SCL    | GPIO 22   | 22     | I2C      | Green      |
| LDR    | Analog | GPIO 34   | 34     | ADC      | White      |
| Power  | 3.3V   | 3V3       | -      | Power    | Red        |
| Ground | GND    | GND       | -      | Ground   | Black      |

---

## Pin Diagram with Photo Reference

```
ESP32-DEVKIT-V1 Pinout (30 pins)
================================

Left Side (Top to Bottom):
┌─────────────────┐
│ 3V3   [●]       │ ← 3.3V Output (connect to sensors)
│ GND   [●]       │ ← Ground (connect to sensors)
│ IO15  [●]       │
│ IO2   [●]       │
│ IO0   [●]       │
│ IO4   [●]───────┼─── DHT22 DATA (yellow wire)
│ IO16  [●]       │
│ IO17  [●]       │
│ IO5   [●]       │
│ IO18  [●]       │
│ IO19  [●]       │
│ IO21  [●]───────┼─── BMP280 SDA (blue wire)
│ IO22  [●]───────┼─── BMP280 SCL (green wire)
│ IO23  [●]       │
└─────────────────┘

Right Side (Top to Bottom):
┌─────────────────┐
│       [●] GND   │ ← Ground (optional)
│       [●] IO27  │
│       [●] IO26  │
│       [●] IO25  │
│       [●] IO33  │
│       [●] IO32  │
│       [●] IO35  │ ← Input only (ADC)
│       [●] IO34  ├─── LDR (white wire)
│       [●] SENSOR_VN │
│       [●] SENSOR_VP │
│       [●] EN    │
│       [●] 3V3   │ ← 3.3V (alt power pin)
│       [●] GND   │ ← Ground (alt)
│       [●] VIN   │ ← 5V Input (from USB)
└─────────────────┘
```

---

## I2C Bus Explanation

**What is I2C?**
- Inter-Integrated Circuit
- 2-wire communication protocol (SDA + SCL)
- Multiple sensors on same bus
- Each sensor has unique address

**I2C Connections on ESP32**:
```
ESP32          BMP280          (Future sensor)
GPIO 21 (SDA) ──┬── SDA           SDA
GPIO 22 (SCL) ──┬── SCL           SCL
                │                  │
              [Pull-up resistors in modules]
```

**Adding more I2C sensors**:
- Simply connect SDA to SDA, SCL to SCL
- No code changes needed (same pins)
- Each sensor must have different address
- Example: Add BME680 gas sensor (address 0x77)

---

## Troubleshooting Wiring

### Problem 1: ESP32 Won't Power On

**Symptoms**: No LEDs, no USB recognition

**Checks**:
- [ ] USB cable is data cable (not charge-only)
- [ ] USB port provides enough power (try different port)
- [ ] Check for short circuits (esp. power rails)
- [ ] Verify ESP32 is not damaged (test without sensors)

### Problem 2: DHT22 Not Reading

**Symptoms**: `ERROR:101:DHT22 not found`

**Checks**:
- [ ] DATA wire connected to GPIO 4 (not GPIO 2 or other)
- [ ] VCC connected to 3.3V (not 5V - can damage sensor)
- [ ] GND connected properly
- [ ] Wait 2 seconds after power-on (DHT22 init time)
- [ ] Try different DHT22 (sensor may be defective)

### Problem 3: BMP280 Not Found

**Symptoms**: `ERROR:102:BMP280 not found on I2C`

**Checks**:
- [ ] SDA wire to GPIO 21 (not swapped with SCL)
- [ ] SCL wire to GPIO 22
- [ ] SDO connected to GND (for 0x76 address)
- [ ] Power connections solid
- [ ] Run I2C scanner sketch to verify address:

```cpp
// I2C Scanner Code
#include <Wire.h>

void setup() {
  Wire.begin(21, 22);  // SDA, SCL
  Serial.begin(115200);
  Serial.println("I2C Scanner");
}

void loop() {
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(address, HEX);
    }
  }
  delay(5000);
}
```

**Expected output**: `Found device at 0x76`

### Problem 4: LDR Reads Constant Value

**Symptoms**: Light sensor always shows same ADC value

**Checks**:
- [ ] 10kΩ resistor in series (not parallel)
- [ ] LDR connected between resistor and ground
- [ ] Wire taps between resistor and LDR (voltage divider midpoint)
- [ ] Cover LDR with hand - value should drop significantly
- [ ] Shine flashlight on LDR - value should increase

**Test with multimeter**:
1. Measure voltage at GPIO 34 pin
2. Should be ~1.6V in room light
3. Cover LDR → voltage drops to ~0.3V
4. Shine light → voltage rises to ~3.0V

---

## Power Consumption Test

After wiring, measure power draw:

**With multimeter in series** (if available):
```
USB 5V ──[Multimeter]── ESP32 VIN

Expected: 150-200 mA
If higher: Check for short circuits
```

**Without multimeter**:
- Check ESP32 doesn't get hot (should be room temp)
- Blue LED on ESP32 should be on (power indicator)
- If any component gets hot: DISCONNECT IMMEDIATELY

---

## Wire Management Tips

### Keep it Clean
1. **Use short wires** - Easier to trace, less clutter
2. **Color code consistently**:
   - Red = 3.3V
   - Black = GND
   - Yellow/Blue/Green = Signals
3. **Route wires around edges** - Don't cross over middle
4. **Group by function** - Keep I2C wires together

### Secure Connections
1. **Push wires fully into breadboard** - Should be firm
2. **Avoid loose components** - Tap sensors to check if secure
3. **Don't overlap bare wires** - Prevents shorts
4. **Label connectors** - Use tape + marker for complex builds

---

## Pre-Flight Checklist

Before powering on, verify:

**Power**:
- [ ] 3.3V rail connected to ESP32 3V3 pin
- [ ] GND rail connected to ESP32 GND pin
- [ ] No shorts between 3.3V and GND (check with multimeter)

**DHT22**:
- [ ] VCC to 3.3V (red wire)
- [ ] DATA to GPIO 4 (yellow wire)
- [ ] GND to ground (black wire)

**BMP280**:
- [ ] VCC to 3.3V (red wire)
- [ ] GND to ground (black wire)
- [ ] SCL to GPIO 22 (green wire)
- [ ] SDA to GPIO 21 (blue wire)
- [ ] SDO to GND for address 0x76

**LDR**:
- [ ] 10kΩ resistor from 3.3V to midpoint
- [ ] LDR from midpoint to GND
- [ ] Wire from midpoint to GPIO 34

**Final Check**:
- [ ] All wires firmly seated in breadboard
- [ ] No loose components
- [ ] Sensor modules not touching (avoid shorts)
- [ ] USB cable ready to connect

---

## Next Steps

1. **Visual verification**: Take photo and compare to this guide
2. **Power on test**: Connect USB, check for blue LED on ESP32
3. **Upload code**: See [Setup Guide](setup_guide.md)
4. **Test sensors**: Follow [Testing Guide](testing_guide.md)

---

## Wiring Diagram Downloads

**Printable Versions** (for reference while building):
- Color diagram: [Download PDF]
- Black & white: [Download PDF]
- Fritzing file: [Download .fzz]

**Photo Gallery**:
- [Complete breadboard assembly - top view]
- [ESP32 closeup with pin labels]
- [Sensor connections detail]
- [Final assembled station]

---

## Document Version

**Version**: 1.0.0
**Last Updated**: 2025-10-29
**Tested On**: ESP32-DEVKIT-V1, DHT22, BMP280
