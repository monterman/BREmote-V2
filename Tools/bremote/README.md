# BREmote Test Suite

Hardware test application for BREmote V2 TX and RX units.

## Installation

```bash
pip install pyserial
```

## Usage

### Command Line

```bash
# Scan for devices
python -m bremote --scan

# Run all tests (auto-detect devices)
python -m bremote

# Test specific port
python -m bremote --port COM3

# Run specific test type
python -m bremote --test all|radio|display|hall|analog|vesc|pwm

# WiFi/Web config tests only
python -m bremote --wifi

# Radio link integration test (requires TX+RX)
python -m bremote --link --duration 15

# Interactive tests (with user prompts)
python -m bremote --interactive

# Interactive tests (with user prompts)
python -m bremote --interactive

# Save report to file
python -m bremote --report results.json
```

### Python API

```python
from bremote import BREmoteTester, BREmoteDevice

# Create tester
tester = BREmoteTester()

# Scan for devices
tester.scan_ports()

# Run all tests
tester.run_all_tests()

# Run specific test types
tester.run_wifi_tests()
tester.run_integration_test(duration=10.0)

# Cleanup
tester.cleanup()
```

---

## Tests

### TX Tests (`tx_tests.py`)

| Test | Command | Description |
|------|---------|-------------|
| `radio` | `?state`, `?printPackets` | Verify radio subsystem active |
| `display` | `?state` | Verify LED matrix display working |
| `hall` | `?printInputs` | Test throttle/steering sensors |
| `analog` | `?state` | Verify ADC/battery monitoring |
| `rssi` | `?printRSSI` | Check radio signal strength |

### RX Tests (`rx_tests.py`)

| Test | Command | Description |
|------|---------|-------------|
| `radio` | `?printrssi` | Verify radio link status |
| `vesc` | `?printbat` | Test VESC/battery interface |
| `pwm` | `?printpwm` | Verify PWM output signals |

### WiFi Tests (`wifi_tests.py`)

| Test | Device | Commands | Description |
|------|--------|----------|-------------|
| `wifi_state` | TX | `?wifi`, `?wifistate` | Query WiFi status |
| `wifi_debug` | TX | `?wifidbg` | Test debug mode get/set |
| `wifi_timeout` | TX | `?wifips` | Test AP startup timeout |
| `wifi_version` | TX | `?wifiver` | Check web UI version |
| `wifi_error` | TX | `?wifierr` | Query last error |
| `wifi_onoff` | TX | `?wifi on/off` | Test enable/disable |
| `wifi_ui_update` | TX | `?wifiupd` | Force UI update to SPIFFS |

### Config Tests (`config_tests.py`)

| Test | Device | Commands | Description |
|------|--------|----------|-------------|
| `config_keys` | TX/RX | `?keys` | List all config keys |
| `config_get_set` | TX/RX | `?get`, `?set` | Test get/set values |
| `spiffs` | TX/RX | `?conf` | Verify SPIFFS storage |

### Link Test (`link_test.py`)

| Test | Description |
|------|-------------|
| `RadioLinkMonitor` | Correlates TX throttle/steering with RX received values over radio link. Measures packet loss, latency, RSSI, SNR. |

---

## Exit Charging Mode

The test suite automatically sends `?exitchg` to TX on startup to exit charge screen mode.

---

## Requirements

- Python 3.7+
- pyserial

---

## File Structure

```
Tools/bremote/
├── __init__.py           # Package exports
├── models.py             # Data classes
├── device.py            # Serial communication
├── runner.py            # Test orchestrator
└── tests/
    ├── __init__.py
    ├── tx_tests.py      # TX tests
    ├── rx_tests.py      # RX tests
    ├── wifi_tests.py    # WiFi tests
    ├── config_tests.py  # Config tests
    └── link_test.py    # Radio link correlation
```
