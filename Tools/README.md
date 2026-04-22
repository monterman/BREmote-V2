# BREmote V2 Hardware Test Suite

Windows application for automated testing of BREmote TX and RX units.

## Features

- **Auto-detection**: Scans all COM ports and identifies BREmote devices
- **Device identification**: Automatically distinguishes TX (Transmitter) from RX (Receiver)
- **Automated tests**: Radio, display, sensors, interfaces (no user interaction needed)
- **Interactive tests**: User-guided tests with prompts to move throttle/press buttons
- **Radio link test**: End-to-end test matching TX output with RX input
- **JSON reports**: Exportable test results with timestamps and detailed results
- **GUI & CLI modes**: Interactive GUI or command-line operation

## Installation

```bash
# Install dependencies
pip install -r requirements.txt
```

## Usage

### GUI Mode (Recommended)

```bash
python bremote_test.py --gui
```

**GUI Buttons:**
- **🔍 Scan Ports**: Detects all connected BREmote devices
- **▶ Run Auto Tests**: Automated tests without user interaction
- **👤 Interactive Test**: Guided tests requiring user actions
- **🔗 Radio Link Test**: Tests TX→RX communication (requires both)
- **📄 Save Report**: Exports results to JSON file
- **🗑 Clear**: Clears the log window

### CLI Mode - Scan Only

```bash
python bremote_test.py --scan
```

### CLI Mode - Auto Test All Devices

```bash
python bremote_test.py
```

### CLI Mode - Interactive Tests

Prompts user to perform actions (move throttle, press buttons):

```bash
python bremote_test.py --interactive
# or
python bremote_test.py -i
```

**Interactive Tests for TX:**
- Center throttle position
- Full throttle forward
- Full brake
- Left toggle press
- Right toggle press
- USB mode combo (throttle + left toggle)

**Interactive Tests for RX:**
- VESC detection
- Radio reception
- PWM output (optional, with safety warning)

### CLI Mode - Radio Link Test

Tests the radio link between TX and RX by comparing TX output with RX received values:

```bash
# Default 10 second test
python bremote_test.py --link

# Custom duration
python bremote_test.py --link --duration 30
```

**Radio Link Metrics:**
- Packet loss percentage
- Value matching (throttle/steering differences)
- RSSI (signal strength)
- SNR (signal quality)
- Matched sample pairs

### CLI Mode - Test Specific Port

```bash
python bremote_test.py --port COM3
```

### CLI Mode - Save Report

```bash
python bremote_test.py --report test_results.json
```

### CLI Mode - Verbose Output

```bash
python bremote_test.py -v
```

## Test Coverage

### Automated Tests (TX)

| Test | Description | Commands Used |
|------|-------------|---------------|
| Radio TX | Verifies LoRa transmission | `?printPackets` |
| Display | Checks LED matrix interface | `?conf` |
| Hall Sensors | Throttle and toggle inputs | `?printInputs` |
| Analog | Battery voltage monitoring | `?conf` |
| RSSI | Signal strength monitoring | `?printRSSI` |
| SPIFFS | Configuration storage | `?conf` |

### Automated Tests (RX)

| Test | Description | Commands Used |
|------|-------------|---------------|
| Radio RX | Verifies LoRa reception | `?packets` |
| VESC | Motor controller interface | `?vesc` |
| PWM | Motor speed PWM output | `?pwm` |
| SPIFFS | Configuration storage | `?conf` |

### Interactive Tests (TX)

| Test | User Action | Verification |
|------|-------------|--------------|
| thr_center | Move throttle to center | Value 120-135 |
| thr_max | Full throttle forward | Value ≥200 |
| thr_min | Full brake | Value ≤55 |
| toggle_left | Press left toggle | Toggle state ON |
| toggle_right | Press right toggle | Toggle state ON |
| combo_usb | Throttle + left toggle | Both active |

### Interactive Tests (RX)

| Test | User Action | Verification |
|------|-------------|--------------|
| vesc_detect | Ensure VESC powered | VESC detected |
| radio_rx | Press TX buttons | Packets received |
| pwm_output | *(Optional)* | PWM active |

### Radio Link Test

| Metric | Description | Pass Criteria |
|--------|-------------|---------------|
| packet_loss | % of lost packets | <20% |
| throttle_diff | TX vs RX difference | ≤5 |
| steering_diff | TX vs RX difference | ≤5 |
| avg_rssi | Signal strength | >-100 dBm |
| matched_pairs | Correlated samples | ≥10 |

## Output Format

### Console Output - Interactive Test

```
============================================================
Interactive Test: BREmote TX on COM3
============================================================

🎯 Center Throttle
   Move throttle to CENTER position (neutral) and press Enter...
   Result: PASS - Throttle at center: 127

🎯 Full Throttle Forward
   Move throttle to FULL FORWARD and press Enter...
   Result: PASS - Throttle at max: 255

🎯 Full Brake
   Move throttle to FULL BRAKE and press Enter...
   Result: PASS - Throttle at brake: 0
```

### Console Output - Radio Link Test

```
============================================================
PHASE 1: Individual Device Tests
============================================================
...

============================================================
PHASE 2: Radio Link Integration Test
============================================================

🔗 Starting Radio Link Test (10.0s)...
   TX: COM3
   RX: COM4

📊 Radio Link Results:
  Samples: TX=105, RX=103, Matched=102
  Packet Loss: 2.9%
  Avg Throttle Diff: 0.50
  Avg Steering Diff: 0.30
  RSSI: -72 dBm (min: -75)
  SNR: 8.5 dB
  Result: PASS
  Details: Radio link working correctly
```

### JSON Report Format

```json
{
  "COM3": {
    "device_type": "tx",
    "port": "COM3",
    "timestamp": "2026-02-26T10:30:00",
    "tests": {
      "thr_center": {
        "result": "PASS",
        "details": "Throttle at center: 127"
      },
      "thr_max": {
        "result": "PASS", 
        "details": "Throttle at max: 255"
      }
    },
    "overall_result": "PASS"
  }
}
```

## Troubleshooting

### Device Not Detected

- Ensure the BREmote is powered on and USB is connected
- Check Windows Device Manager for the COM port
- Try different USB cables (some cables are power-only)
- Install CH340 or CP2102 drivers if needed

### Permission Denied

- Close any other programs using the COM port (Arduino IDE, PlatformIO, serial monitor)
- Run as Administrator if needed

### Tests Failing

- Verify the BREmote firmware is flashed correctly
- Check that debug output is enabled (`serialOff = false`)
- Ensure the device responds to `?conf` command

### Radio Link Test Failing

- Ensure TX and RX are paired/bound
- Check antennas are connected
- Verify TX and RX are on the same frequency (868MHz or 915MHz)
- Reduce distance between TX and RX
- Check for interference sources

## Firmware Requirements

For full functionality, the BREmote firmware should support these serial commands:

**TX Commands:**
- `?conf` - Print configuration
- `?printInputs` - Show throttle/toggle values
- `?printRSSI` - Show signal strength
- `?printPackets` - Show packet counters
- `quit` - Stop continuous output

**RX Commands:**
- `?conf` - Print configuration
- `?vesc` - VESC status
- `?pwm` - PWM status
- `?packets` - Radio statistics
- `?printReceived` - Show received values (for link test)

## Extending the Tests

To add custom interactive tests:

```python
def _interactive_custom_test(self, device: BREmoteDevice, gui_root=None) -> TestReport:
    tests = {}
    
    # Add your test
    tests["my_test"] = self._prompt_and_verify(
        "My Custom Test",
        "Instructions for user...",
        device,
        self._verify_my_test,
        gui_root
    )
    
    # ... create and return TestReport

def _verify_my_test(self, device: BREmoteDevice) -> Dict:
    response = device.send_command("?mycommand")
    if "expected" in response:
        return {"result": TestResult.PASS.value, "details": "Success"}
    return {"result": TestResult.FAIL.value, "details": "Failed"}
```

## Requirements

- Python 3.7+
- Windows 10/11
- USB-Serial drivers (CH340, CP2102, FTDI, etc.)
- BREmote V2 hardware with debug output enabled

## License

GPL 3.0 (same as BREmote project)
