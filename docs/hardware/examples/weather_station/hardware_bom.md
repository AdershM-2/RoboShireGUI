# Weather Station - Bill of Materials (BOM)

**Last Updated**: 2025-10-29
**Total Cost**: ~$20 USD (minimal) to $30 USD (complete)

---

## Core Components (Required)

| Item | Quantity | Cost | Purpose | Purchase Links |
|------|----------|------|---------|----------------|
| **ESP32 DevKit** | 1 | $6-8 | Main microcontroller | [Amazon](https://amazon.com/s?k=esp32+devkit), [AliExpress](https://aliexpress.com/wholesale?SearchText=esp32) |
| **DHT22 Sensor** | 1 | $3-5 | Temperature & Humidity | [Amazon](https://amazon.com/s?k=dht22), [Adafruit](https://adafruit.com/product/385) |
| **Breadboard (400 tie)** | 1 | $2-3 | Prototyping | [Amazon](https://amazon.com/s?k=breadboard), [Adafruit](https://adafruit.com/product/64) |
| **Jumper Wires** | 1 set | $3-4 | Connections | [Amazon](https://amazon.com/s?k=jumper+wires) |
| **USB Cable (Micro-USB)** | 1 | $2-3 | Power & Programming | Local electronics store |

**Minimal Setup Total**: ~$18

---

## Additional Sensors (Recommended)

| Item | Quantity | Cost | Purpose | Purchase Links |
|------|----------|------|---------|----------------|
| **BMP280 Sensor** | 1 | $3-5 | Barometric Pressure | [Amazon](https://amazon.com/s?k=bmp280), [Adafruit](https://adafruit.com/product/2651) |
| **Photoresistor (LDR)** | 1 | $0.50 | Light Level | [Amazon](https://amazon.com/s?k=photoresistor+5mm) |
| **10kΩ Resistor** | 1 | $0.10 | Pull-down for LDR | Local electronics store |

**Complete Setup Total**: ~$22

---

## Optional Upgrades

| Item | Quantity | Cost | Purpose |
|------|----------|------|---------|
| **BME280** (instead of BMP280) | 1 | $8-10 | Pressure + Humidity (more accurate than DHT22) |
| **MQ-135 Gas Sensor** | 1 | $4-6 | Air quality (CO2, NH3, NOx) |
| **DS18B20 Waterproof** | 1 | $4-6 | Waterproof temperature probe |
| **UV Sensor (GUVA-S12SD)** | 1 | $5-8 | UV index measurement |
| **Rain Sensor** | 1 | $2-4 | Precipitation detection |
| **Soil Moisture Sensor** | 1 | $2-4 | For plant monitoring |

---

## Enclosure & Power (Optional)

| Item | Quantity | Cost | Purpose |
|------|----------|------|---------|
| **IP65 Enclosure** | 1 | $8-12 | Weatherproof housing |
| **5V 2A Power Supply** | 1 | $5-8 | Wall power (instead of USB) |
| **LiPo Battery 3.7V 2000mAh** | 1 | $8-12 | Portable power |
| **TP4056 Charging Module** | 1 | $1-2 | Battery charging circuit |
| **Solar Panel 5V 2W** | 1 | $8-15 | Solar charging |

**Complete Outdoor Station**: ~$50-60

---

## Tools Required

### Must Have
- Computer with USB port (for programming)
- Arduino IDE or PlatformIO (free software)
- Internet connection (for library downloads)

### Nice to Have
- Multimeter (for debugging, ~$15)
- Wire strippers (for custom cables, ~$10)
- Soldering iron (if making permanent connections, ~$20)
- Heat shrink tubing (for professional finish, ~$5)

---

## Component Details

### ESP32 DevKit (ESP32-WROOM-32)

**Specifications**:
- CPU: Dual-core Xtensa LX6 @ 240 MHz
- RAM: 520 KB SRAM
- Flash: 4 MB
- WiFi: 802.11 b/g/n (2.4 GHz)
- Bluetooth: BLE 4.2
- GPIO: 34 pins
- ADC: 18 channels (12-bit)
- I2C: 2 controllers
- SPI: 3 controllers
- UART: 3 controllers
- Operating Voltage: 3.3V (5V tolerant on some pins)
- Power Consumption: ~150 mA active, ~10 µA deep sleep

**Why ESP32?**
- Built-in WiFi for future wireless expansion
- More powerful than Arduino Uno
- More GPIOs for sensor expansion
- Same Arduino IDE compatibility
- Lower cost than Arduino ($6 vs $25)

**Alternatives**:
- **ESP8266 NodeMCU**: Cheaper ($4), less powerful, still has WiFi
- **Arduino Mega 2560**: More GPIOs, no WiFi, tried-and-true
- **Teensy 3.2/4.0**: Faster, better ADC, more expensive ($20-25)
- **Raspberry Pi Pico**: RP2040 chip, no WiFi (non-W version)

---

### DHT22 (AM2302) Temperature & Humidity Sensor

**Specifications**:
- Temperature Range: -40°C to 80°C
- Temperature Accuracy: ±0.5°C
- Humidity Range: 0-100% RH
- Humidity Accuracy: ±2% RH
- Sample Rate: 0.5 Hz (once per 2 seconds)
- Operating Voltage: 3.3-5V
- Current: 1-1.5 mA measuring, <60 µA standby
- Output: Single-wire digital (proprietary protocol)

**Pinout** (3-pin module):
1. VCC (red) - Power 3.3-5V
2. DATA (yellow) - Digital signal
3. GND (black) - Ground

**Why DHT22?**
- Affordable and widely available
- Simple single-wire interface
- Good accuracy for price point
- Widely supported Arduino libraries

**Alternatives**:
- **DHT11**: Cheaper ($2), less accurate (±2°C), 1% RH resolution
- **BME280**: More accurate (±1°C, ±3% RH), adds pressure, I2C, $8-10
- **SHT31**: Best accuracy (±0.3°C, ±2% RH), I2C, $10-15
- **DS18B20**: Temperature only, waterproof version available, 1-Wire

---

### BMP280 Barometric Pressure Sensor

**Specifications**:
- Pressure Range: 300-1100 hPa (9000m to -500m altitude)
- Pressure Accuracy: ±1 hPa (±0.12 m altitude)
- Temperature Range: -40°C to 85°C (built-in)
- Temperature Accuracy: ±1°C
- Interface: I2C (default 0x76, alt 0x77) or SPI
- Operating Voltage: 1.8-3.6V (module has 3.3V regulator)
- Current: 2.7 µA @ 1 Hz sampling

**Pinout** (6-pin module):
1. VCC - 3.3V or 5V (module regulated)
2. GND - Ground
3. SCL - I2C Clock
4. SDA - I2C Data
5. CSB - Chip Select (not used in I2C mode)
6. SDO - I2C address select (GND=0x76, VCC=0x77)

**Why BMP280?**
- Measures atmospheric pressure (weather prediction)
- Can calculate altitude
- I2C interface (easy to daisy-chain sensors)
- Very low power consumption
- Temperature sensor included (can replace DHT22 if needed)

**Alternatives**:
- **BME280**: Adds humidity sensor, $8-10 (best all-in-one)
- **BMP180**: Older generation, still works, slightly less accurate
- **BMP388**: Newer, more accurate (±0.4 hPa), more expensive ($12)
- **MS5611**: Aviation-grade, higher accuracy, $15-20

---

### Photoresistor (LDR) + Resistor

**Specifications**:
- Resistance in Light: 1-10 kΩ (depends on brightness)
- Resistance in Dark: 1 MΩ
- Response Time: ~10 ms
- Operating Voltage: Any (passive component)
- Lifetime: 10+ years

**Circuit**:
```
VCC (3.3V)
    │
   [10kΩ resistor]
    │
    ├──────> to ESP32 GPIO (ADC pin)
    │
   [LDR]
    │
   GND
```

**Why Photoresistor?**
- Dirt cheap ($0.50)
- No special protocols needed
- Analog output (easy to read with ADC)
- Works with any microcontroller

**Calculation**:
```cpp
int raw = analogRead(LIGHT_PIN);  // 0-4095 on ESP32
float voltage = raw * (3.3 / 4095.0);
// Higher voltage = more light
```

**Alternatives**:
- **BH1750**: Digital I2C light sensor, calibrated in lux, $2-3
- **TSL2561**: More accurate, lux output, I2C, $6-8
- **VEML7700**: High accuracy, I2C, $4-6

---

## Wiring Checklist

Before purchasing, ensure you have:
- [ ] ESP32 DevKit (with pins pre-soldered)
- [ ] DHT22 sensor (3-pin or 4-pin module)
- [ ] BMP280 module (breakout board with 6 pins)
- [ ] Breadboard (400+ tie points)
- [ ] Male-to-male jumper wires (at least 15 wires)
- [ ] USB cable (Micro-USB for ESP32)
- [ ] 10kΩ resistor for LDR (if using)
- [ ] Photoresistor (LDR)

---

## Where to Buy

### Budget Option (AliExpress/eBay)
**Pros**: Cheapest prices ($10-12 total)
**Cons**: 2-4 week shipping, quality varies
**Recommended for**: Bulk orders, hobbyists, non-urgent projects

### Standard Option (Amazon)
**Pros**: Fast shipping (1-2 days), easy returns
**Cons**: 50% more expensive ($18-22 total)
**Recommended for**: Quick projects, beginners, reliable parts

### Premium Option (Adafruit/SparkFun)
**Pros**: Best quality, excellent documentation, supports open-source
**Cons**: Most expensive ($25-30 total)
**Recommended for**: Educational use, critical projects, supporting makers

### Local Electronics Store
**Pros**: Same-day availability, expert advice, no shipping
**Cons**: Limited selection, highest prices
**Recommended for**: Urgent needs, hands-on shopping

---

## Pre-Assembled Kits

### RoboShire Weather Station Kit (Hypothetical)
If offered as complete kit:
- All sensors pre-tested
- Pre-cut jumper wires
- Printed wiring diagram
- USB cable included
- SD card with code
**Price**: $30-35 (saves 2-3 hours sourcing parts)

---

## Bulk Ordering (Classrooms)

For 10+ stations:

| Item | Unit Price | Bulk Price (10+) | Savings |
|------|------------|------------------|---------|
| ESP32 DevKit | $8 | $5 | 37% |
| DHT22 | $4 | $2.50 | 37% |
| BMP280 | $4 | $2.50 | 37% |
| Breadboard + Wires | $5 | $3 | 40% |
| **Total per station** | **$21** | **$13** | **38%** |

**10-pack cost**: ~$130 vs $210 retail (save $80)

**Contact educational suppliers**:
- Adafruit Education
- SparkFun Classroom Kits
- Digi-Key Education

---

## Substitutions & Alternatives

### If DHT22 is Unavailable
**Use**: DHT11 (cheaper, less accurate)
**Change**: None (same library, same code)
**Tradeoff**: ±2°C accuracy vs ±0.5°C

### If BMP280 is Unavailable
**Use**: BME280 (better - adds humidity!)
**Change**: Change I2C address in code if needed
**Tradeoff**: Costs $3-5 more, but better sensor

### If ESP32 is Unavailable
**Use**: Arduino Mega 2560 or ESP8266
**Change**: Pin assignments in code
**Tradeoff**: Arduino = no WiFi, ESP8266 = less powerful

### If Breadboard is Unavailable
**Use**: Perfboard + soldering
**Change**: Permanent connections (harder to debug)
**Tradeoff**: More work, but portable final product

---

## Power Consumption Calculation

**Active Mode** (all sensors reading):
```
ESP32:        150 mA @ 3.3V = 0.50 W
DHT22:        1.5 mA @ 3.3V = 0.005 W
BMP280:       2.7 mA @ 3.3V = 0.009 W
Total:        ~154 mA @ 3.3V = 0.51 W
```

**From USB**: 5V × 200 mA = 1 W available (plenty of headroom)

**From Battery**:
```
2000 mAh LiPo battery
2000 mAh / 154 mA = 13 hours runtime (active)
With deep sleep (10 µA): ~1000+ hours (41+ days)
```

---

## Storage and Organization

**Recommended storage**:
- Small plastic organizer box ($5)
- Anti-static bags for ESP32 (reuse packaging)
- Label compartments: "Sensors", "Wires", "Resistors"

**Inventory tracking**:
```
Components Inventory
--------------------
ESP32 DevKit:       Qty: __  Location: __
DHT22 Sensor:       Qty: __  Location: __
BMP280 Sensor:      Qty: __  Location: __
Breadboard:         Qty: __  Location: __
Jumper Wires:       Qty: __  Location: __
USB Cables:         Qty: __  Location: __
```

---

## Next Step

After ordering/receiving components, proceed to [Wiring Diagram](wiring_diagram.md) for assembly instructions.

---

## Shopping List Summary

**Print this checklist and take shopping**:

### Essential ($18-22)
- [ ] 1x ESP32 DevKit
- [ ] 1x DHT22 Temperature/Humidity Sensor
- [ ] 1x Breadboard (400 tie points)
- [ ] 1x Jumper wire set (40+ wires)
- [ ] 1x USB Micro cable

### Recommended Additions ($5-8)
- [ ] 1x BMP280 Pressure Sensor
- [ ] 1x Photoresistor (LDR)
- [ ] 1x 10kΩ Resistor

### Optional Upgrades ($15-25)
- [ ] 1x IP65 Enclosure
- [ ] 1x 5V 2A Power Supply
- [ ] 1x LiPo Battery + Charger

**Total Budget**: $18 (minimal) to $50 (complete outdoor station)

---

**Document Version**: 1.0.0
**Last Price Check**: 2025-10-29 (prices may vary)
