# Weather Station Testing Guide

**Time**: 15-20 minutes
**Goal**: Verify all components working correctly

---

## Test Checklist

- [ ] Arduino serial output
- [ ] Temperature readings valid (15-35°C indoor)
- [ ] Humidity readings valid (30-70% RH indoor)
- [ ] Pressure readings valid (980-1040 hPa)
- [ ] Light sensor responds to changes
- [ ] ROS2 topics publishing
- [ ] Data rates correct (~1 Hz)
- [ ] No errors in logs

---

## Test 1: Arduino Serial Monitor

### Procedure
1. Connect ESP32 via USB
2. Arduino IDE → Tools → Serial Monitor (115200 baud)
3. Observe output for 30 seconds

### Expected Output
```
STATUS:READY
TEMP:24.8
HUMIDITY:58.3
PRESSURE:1013.25
LIGHT:487
TEMP:24.9
HUMIDITY:58.4
...
```

### Validation
- ✅ Messages every ~1 second
- ✅ Temperature 15-35°C (indoor typical)
- ✅ Humidity 30-70% RH
- ✅ Pressure 980-1040 hPa (sea level typical)
- ✅ Light changes when covered/uncovered

### Troubleshooting
| Issue | Solution |
|-------|----------|
| No output | Check baud rate is 115200 |
| Garbage characters | Wrong baud rate or bad USB cable |
| TEMP: nan | DHT22 not connected or bad sensor |
| ERROR:102 | BMP280 wiring issue (check I2C) |

---

## Test 2: Sensor Response Tests

### Temperature Test
1. Blow warm air on DHT22 sensor
2. **Expected**: Temperature increases 1-3°C
3. Wait 30 seconds, should return to ambient

### Humidity Test
1. Breathe on DHT22 sensor
2. **Expected**: Humidity jumps 10-20%
3. Returns to normal after 1-2 minutes

### Pressure Test
1. Note current reading
2. Compare to local weather report (±5 hPa acceptable)
3. Pressure changes slowly (weather fronts)

### Light Test
1. Cover LDR with hand
2. **Expected**: Value drops by 50-80%
3. Shine flashlight on LDR
4. **Expected**: Value increases significantly

**All tests passing? Sensors working correctly!**

---

## Test 3: ROS2 Integration

### Start Serial Bridge
```bash
cd workspace
source install/setup.bash
ros2 run weather_station serial_bridge
```

**Expected**:
```
[INFO] [serial_bridge]: Connected to /dev/ttyUSB0 at 115200 baud
```

### Verify Topics Exist
```bash
# New terminal
source install/setup.bash
ros2 topic list
```

**Expected topics**:
- `/sensors/temperature`
- `/sensors/humidity`
- `/sensors/pressure`
- `/sensors/light`
- `/serial_bridge/status`

### Test Topic Data
```bash
# Temperature
ros2 topic echo /sensors/temperature --once
# Expected: data: 24.8

# Humidity
ros2 topic echo /sensors/humidity --once
# Expected: data: 58.3

# All topics at once
ros2 topic echo /sensors/temperature & \
ros2 topic echo /sensors/humidity & \
ros2 topic echo /sensors/pressure &
```

---

## Test 4: Data Rate Verification

### Check Publish Frequency
```bash
ros2 topic hz /sensors/temperature
```

**Expected**: `average rate: 1.000` (1 Hz)

### Check Message Count
```bash
# Count messages for 10 seconds
timeout 10 ros2 topic echo /sensors/temperature | grep "data:" | wc -l
```

**Expected**: ~10 messages

---

## Test 5: Data Quality Analysis

### Record Data Sample
```bash
ros2 bag record -o weather_test /sensors/temperature /sensors/humidity /sensors/pressure /sensors/light
# Record for 60 seconds, then Ctrl+C
```

### Playback and Analyze
```bash
ros2 bag play weather_test
# In another terminal:
ros2 topic echo /sensors/temperature > temp_data.txt
```

### Statistical Analysis (Python)
```python
#!/usr/bin/env python3
import re

with open('temp_data.txt', 'r') as f:
    values = [float(match.group(1)) for line in f for match in [re.search(r'data: ([\d.]+)', line)] if match]

print(f"Samples: {len(values)}")
print(f"Mean: {sum(values)/len(values):.2f}°C")
print(f"Min: {min(values):.2f}°C")
print(f"Max: {max(values):.2f}°C")
print(f"Range: {max(values) - min(values):.2f}°C")

# Expected: Range < 1°C over 1 minute (stable indoor)
```

---

## Test 6: Long-Term Stability

### 1-Hour Soak Test
```bash
# Log to file
ros2 topic echo /sensors/temperature > temperature_log.txt &
sleep 3600  # Wait 1 hour
kill %1
```

### Check for Issues
```bash
# Count total messages (should be ~3600)
grep "data:" temperature_log.txt | wc -l

# Check for anomalies
grep -E "data: (999|-999|nan|inf)" temperature_log.txt
# Should return nothing
```

**Pass criteria**:
- No missing data (3600 messages in 1 hour)
- No invalid values (nan, inf, extreme outliers)
- Data varies smoothly (no sudden jumps >5°C)

---

## Test 7: System Integration

### Visualize Data in RoboShire
1. Launch RoboShire GUI
2. Widgets → Data Plotter
3. Add `/sensors/temperature`
4. Add `/sensors/humidity`
5. **Expected**: Real-time updating graphs

### rqt_graph Visualization
```bash
ros2 run rqt_graph rqt_graph
```

**Expected node graph**:
```
[/serial_bridge] ----> /sensors/temperature
                 ----> /sensors/humidity
                 ----> /sensors/pressure
                 ----> /sensors/light
```

---

## Performance Benchmarks

| Metric | Target | Typical | Acceptable Range |
|--------|--------|---------|------------------|
| Temperature accuracy | ±0.5°C | ±1°C | ±2°C |
| Humidity accuracy | ±2% RH | ±3% RH | ±5% RH |
| Pressure accuracy | ±1 hPa | ±2 hPa | ±5 hPa |
| Publish rate | 1.0 Hz | 1.0 Hz | 0.9-1.1 Hz |
| Serial latency | <10 ms | 5-15 ms | <50 ms |
| Message loss | 0% | 0% | <0.1% |
| CPU usage (ESP32) | <50% | 20-30% | <70% |

---

## Troubleshooting Test Failures

### Temperature Reads Wrong
**Problem**: Shows 85°C or -999°C
**Solution**:
- Add 2 sec delay in Arduino setup()
- Check DHT22 wiring (DATA to GPIO 4)
- Replace DHT22 (may be defective)

### Topics Not Updating
**Problem**: `ros2 topic echo` shows nothing
**Solution**:
```bash
# Check serial bridge running
ros2 node list | grep serial

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Check Arduino still connected
ls /dev/tty*
```

### Data Rate Too Slow
**Problem**: `ros2 topic hz` shows 0.5 Hz instead of 1 Hz
**Solution**:
- Check Arduino loop delay (should be 1000 ms)
- Verify no blocking code in Arduino
- Check serial buffer not overflowing

---

## Acceptance Criteria

✅ **Pass**: All tests successful, system ready for deployment
⚠️ **Warning**: Minor issues but functional (document known issues)
❌ **Fail**: Critical issues, needs debugging

### Minimum Requirements to Pass
- [ ] All sensors read valid values
- [ ] ROS2 topics publish at 1 Hz
- [ ] No errors in 10-minute test run
- [ ] Data visualization works in GUI
- [ ] Temperature/humidity respond to changes

---

## Test Report Template

```markdown
# Weather Station Test Report

**Date**: 2025-10-29
**Tester**: [Your Name]
**Hardware**: ESP32 + DHT22 + BMP280 + LDR
**Software**: RoboShire v2.3.0, ROS2 Humble

## Results Summary
- Arduino Serial: ✅ Pass
- Sensor Response: ✅ Pass
- ROS2 Topics: ✅ Pass
- Data Rate: ✅ Pass (1.0 Hz)
- Long-Term: ✅ Pass (1 hour stable)
- Visualization: ✅ Pass

## Issues Found
None

## Recommendations
- Consider adding weatherproof enclosure for outdoor use
- Add data logging to SD card for offline operation

## Conclusion
System ready for deployment. All acceptance criteria met.
```

---

## Next Steps After Testing

**If all tests pass**:
1. Deploy to permanent location
2. Set up automated data logging
3. Create visualization dashboard
4. Try advanced project: [Differential Drive Robot](../differential_drive_robot/README.md)

**If issues found**:
1. Refer to [Troubleshooting Guide](../../TROUBLESHOOTING_HARDWARE.md)
2. Check [Hardware Documentation](../../README.md)
3. Post in community forums with test results

---

**Document Version**: 1.0.0
**Tested Hardware**: ESP32-DEVKIT-V1, DHT22, BMP280
**Last Test Date**: 2025-10-29
