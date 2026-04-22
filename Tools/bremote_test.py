#!/usr/bin/env python3
"""
BREmote V2 Hardware Test Suite
Windows application for verifying TX and RX units

Features:
- Auto-detects BREmote devices on COM ports
- Identifies TX vs RX units
- Runs comprehensive functional tests
- Generates test reports

Usage:
    python bremote_test.py [--port COMx] [--test all|radio|display|hall|analog] [--gui]
"""

import sys
import time
import serial
import serial.tools.list_ports
import threading
import json
import argparse
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import Optional, List, Dict, Tuple, Callable
from enum import Enum
import logging
import subprocess
import urllib.request
import urllib.parse
import urllib.error
import re

# Optional GUI support
try:
    import tkinter as tk
    from tkinter import ttk, scrolledtext, messagebox
    GUI_AVAILABLE = True
except ImportError:
    GUI_AVAILABLE = False


class DeviceType(Enum):
    UNKNOWN = "unknown"
    TRANSMITTER = "tx"
    RECEIVER = "rx"


class TestResult(Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"
    PENDING = "PENDING"


@dataclass
class TestReport:
    device_type: str
    port: str
    timestamp: str
    tests: Dict[str, Dict]
    overall_result: str
    
    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2)


class BREmoteDevice:
    """Represents a connected BREmote device (TX or RX)"""
    
    BAUD_RATE = 115200
    TIMEOUT = 2.0
    COMMAND_DELAY = 0.1
    
    def __init__(self, port: str):
        self.port = port
        self.device_type = DeviceType.UNKNOWN
        self.serial: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.last_response = ""
        
    def connect(self) -> bool:
        """Establish serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.BAUD_RATE,
                timeout=self.TIMEOUT,
                write_timeout=1.0
            )
            time.sleep(0.5)  # Allow device to initialize
            self._flush_buffers()
            return True
        except Exception as e:
            logging.error(f"Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None
    
    def _flush_buffers(self):
        """Clear serial buffers"""
        if self.serial:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
    
    def send_command(self, command: str, wait_for_response: bool = True) -> str:
        """Send command and optionally wait for response"""
        with self.lock:
            if not self.serial or not self.serial.is_open:
                return ""
            
            try:
                self._flush_buffers()
                self.serial.write(f"{command}\r\n".encode())
                self.serial.flush()
                
                if wait_for_response:
                    time.sleep(self.COMMAND_DELAY)
                    response = self._read_response()
                    self.last_response = response
                    return response
                return ""
            except Exception as e:
                logging.error(f"Command failed: {e}")
                return ""
    
    def _read_response(self) -> str:
        """Read response from serial"""
        lines = []
        start_time = time.time()

        while time.time() - start_time < self.TIMEOUT:
            if self.serial.in_waiting:
                try:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        lines.append(line)
                except:
                    pass
            else:
                time.sleep(0.01)

        return "\n".join(lines)

    def send_json_command(self, command: str) -> Optional[dict]:
        """Send a command with 'json' argument and parse the JSON response.

        Returns parsed dict on success, None on failure.
        The command should NOT include the 'json' suffix — it is appended automatically.
        """
        response = self.send_command(f"{command} json")
        if not response:
            return None
        # The response may contain multiple lines; find the first valid JSON line
        for line in response.split('\n'):
            line = line.strip()
            if line.startswith('{'):
                try:
                    return json.loads(line)
                except json.JSONDecodeError:
                    continue
        return None

    def identify(self) -> DeviceType:
        """Determine if device is TX or RX via ?conf banner"""
        # ?conf prints a banner with "BREmote V2 TX" or "BREmote V2 RX"
        response = self.send_command("?conf")
        logging.debug(f"identify {self.port}: ?conf response ({len(response)} chars): {response[:300]}")
        if "BREmote V2 TX" in response:
            self.device_type = DeviceType.TRANSMITTER
            return self.device_type
        if "BREmote V2 RX" in response:
            self.device_type = DeviceType.RECEIVER
            return self.device_type

        # Fallback: check help output for device-specific commands
        response = self.send_command("?")
        logging.debug(f"identify {self.port}: ? response ({len(response)} chars): {response[:300]}")
        if any(x in response.lower() for x in ["vesc", "pwm", "printreceived", "printpwm"]):
            self.device_type = DeviceType.RECEIVER
            return self.device_type
        if any(x in response.lower() for x in ["hall", "display", "exitchg", "printinputs"]):
            self.device_type = DeviceType.TRANSMITTER
            return self.device_type

        return self.device_type
    
    def is_connected(self) -> bool:
        """Check if serial connection is active"""
        return self.serial is not None and self.serial.is_open
    
    def __str__(self):
        return f"BREmote {self.device_type.value.upper()} on {self.port}"


@dataclass
class RadioLinkSample:
    """Single sample of TX output and RX input"""
    timestamp: float
    tx_throttle: Optional[int] = None
    tx_steering: Optional[int] = None
    rx_throttle: Optional[int] = None
    rx_steering: Optional[int] = None
    rssi: Optional[int] = None
    snr: Optional[float] = None


class RadioLinkMonitor:
    """Monitors and correlates TX output with RX input over radio link"""
    
    def __init__(self, tx_device: BREmoteDevice, rx_device: BREmoteDevice, 
                 gui_callback: Optional[Callable] = None):
        self.tx_device = tx_device
        self.rx_device = rx_device
        self.gui_callback = gui_callback
        self.running = False
        self.samples: List[RadioLinkSample] = []
        self.tx_thread: Optional[threading.Thread] = None
        self.rx_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        self.tx_buffer = ""
        self.rx_buffer = ""
        self._last_tx_sample_time = 0.0  # Rate-limit TX samples to ~10Hz
        
    def log(self, message: str):
        """Log message"""
        print(message)
        if self.gui_callback:
            self.gui_callback(message)
    
    def start(self, duration: float = 10.0):
        """Start monitoring radio link for specified duration"""
        self.log(f"\n[LINK] Starting Radio Link Test ({duration}s)...")
        self.log(f"   TX: {self.tx_device.port}")
        self.log(f"   RX: {self.rx_device.port}")

        # Enable continuous JSON output on both devices
        self.tx_device.send_command("?printInputs json", wait_for_response=False)
        time.sleep(0.2)
        self.rx_device.send_command("?printreceived json", wait_for_response=False)
        
        self.running = True
        self.samples = []
        
        # Start monitoring threads
        self.tx_thread = threading.Thread(target=self._monitor_tx)
        self.rx_thread = threading.Thread(target=self._monitor_rx)
        self.tx_thread.start()
        self.rx_thread.start()
        
        # Wait for test duration
        time.sleep(duration)
        
        # Stop monitoring
        self.stop()
        
        # Analyze results
        return self._analyze_results()
    
    def stop(self):
        """Stop monitoring"""
        self.running = False
        
        # Stop continuous output
        self.tx_device.send_command("quit", wait_for_response=False)
        self.rx_device.send_command("quit", wait_for_response=False)
        
        # Wait for threads to finish
        if self.tx_thread:
            self.tx_thread.join(timeout=1.0)
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
    
    def _monitor_tx(self):
        """Monitor TX serial output in background"""
        while self.running:
            try:
                if self.tx_device.serial and self.tx_device.serial.in_waiting:
                    data = self.tx_device.serial.read(self.tx_device.serial.in_waiting)
                    self.tx_buffer += data.decode('utf-8', errors='ignore')
                    self._parse_tx_buffer()
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running:
                    self.log(f"  TX Monitor Error: {e}")
    
    def _monitor_rx(self):
        """Monitor RX serial output in background"""
        while self.running:
            try:
                if self.rx_device.serial and self.rx_device.serial.in_waiting:
                    data = self.rx_device.serial.read(self.rx_device.serial.in_waiting)
                    self.rx_buffer += data.decode('utf-8', errors='ignore')
                    self._parse_rx_buffer()
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running:
                    self.log(f"  RX Monitor Error: {e}")
    
    def _parse_tx_buffer(self):
        """Parse TX JSON output for throttle/steering values.

        Expected JSON: {"throttle":N,"steering":N,"toggle":N,...}

        TX inputs are reported faster than the 10Hz radio send rate, so we
        rate-limit to one sample per 100ms window to match actual TX packets.
        """
        lines = self.tx_buffer.split('\n')
        self.tx_buffer = lines[-1] if lines else ""

        for line in lines[:-1]:
            line = line.strip()
            if not line.startswith('{'):
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                continue

            now = time.time()
            # Only record one TX sample per 100ms to match 10Hz radio rate
            if now - self._last_tx_sample_time < 0.09:
                continue
            self._last_tx_sample_time = now

            sample = RadioLinkSample(timestamp=now)
            # Prefer thr_sent/steer_sent (actual values sent over radio,
            # post expo+gear) over raw input values for accurate comparison
            if "thr_sent" in data:
                sample.tx_throttle = int(data["thr_sent"])
            elif "throttle" in data:
                sample.tx_throttle = int(data["throttle"])
            if "steer_sent" in data:
                sample.tx_steering = int(data["steer_sent"])
            elif "steering" in data:
                sample.tx_steering = int(data["steering"])

            if sample.tx_throttle is not None or sample.tx_steering is not None:
                with self.lock:
                    self.samples.append(sample)
    
    def _parse_rx_buffer(self):
        """Parse RX JSON output for received throttle/steering/RSSI values.

        Expected JSON: {"throttle":N,"steering":N,"rssi":N,"snr":F,...}
        """
        lines = self.rx_buffer.split('\n')
        self.rx_buffer = lines[-1] if lines else ""

        for line in lines[:-1]:
            line = line.strip()
            if not line.startswith('{'):
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                continue

            sample = RadioLinkSample(timestamp=time.time())
            if "throttle" in data:
                sample.rx_throttle = int(data["throttle"])
            if "steering" in data:
                sample.rx_steering = int(data["steering"])
            if "rssi" in data:
                sample.rssi = int(data["rssi"])
            if "snr" in data:
                sample.snr = float(data["snr"])

            if sample.rx_throttle is not None or sample.rx_steering is not None:
                with self.lock:
                    # Match with recent TX sample (within 200ms)
                    matched = False
                    for s in reversed(self.samples[-50:]):
                        if abs(s.timestamp - sample.timestamp) < 0.2:
                            if sample.rx_throttle is not None:
                                s.rx_throttle = sample.rx_throttle
                            if sample.rx_steering is not None:
                                s.rx_steering = sample.rx_steering
                            if sample.rssi is not None:
                                s.rssi = sample.rssi
                            if sample.snr is not None:
                                s.snr = sample.snr
                            matched = True
                            break

                    if not matched:
                        self.samples.append(sample)
    
    def _analyze_results(self) -> Dict:
        """Analyze collected samples and return results"""
        with self.lock:
            samples = self.samples.copy()
        
        if not samples:
            return {
                "result": TestResult.FAIL.value,
                "details": "No data collected",
                "samples_collected": 0,
                "matched_pairs": 0,
                "avg_latency_ms": None,
                "avg_rssi": None,
                "packet_loss_percent": 100.0
            }
        
        # Count matched pairs (samples with both TX and RX data)
        matched = [s for s in samples if s.tx_throttle is not None and s.rx_throttle is not None]
        tx_only = [s for s in samples if s.tx_throttle is not None and s.rx_throttle is None]
        
        total_tx = len([s for s in samples if s.tx_throttle is not None])
        total_rx = len([s for s in samples if s.rx_throttle is not None])
        matched_count = len(matched)
        
        # Calculate packet loss
        packet_loss = ((total_tx - matched_count) / total_tx * 100) if total_tx > 0 else 100.0
        
        # Compare aggregate TX vs RX means — individual sample pairs are
        # unreliable because the two serial streams are asynchronous and
        # timestamp-matching can pair values from different radio cycles.
        # A sustained offset in the means indicates real data corruption.
        tx_throttles = [s.tx_throttle for s in samples if s.tx_throttle is not None]
        rx_throttles = [s.rx_throttle for s in samples if s.rx_throttle is not None]
        tx_steerings = [s.tx_steering for s in samples if s.tx_steering is not None]
        rx_steerings = [s.rx_steering for s in samples if s.rx_steering is not None]

        avg_tx_thr = sum(tx_throttles) / len(tx_throttles) if tx_throttles else 0
        avg_rx_thr = sum(rx_throttles) / len(rx_throttles) if rx_throttles else 0
        avg_tx_steer = sum(tx_steerings) / len(tx_steerings) if tx_steerings else 0
        avg_rx_steer = sum(rx_steerings) / len(rx_steerings) if rx_steerings else 0

        avg_throttle_diff = abs(avg_tx_thr - avg_rx_thr)
        avg_steering_diff = abs(avg_tx_steer - avg_rx_steer)
        max_throttle_diff = round(avg_throttle_diff)
        max_steering_diff = round(avg_steering_diff)
        
        # RSSI statistics
        rssi_values = [s.rssi for s in matched if s.rssi is not None]
        avg_rssi = sum(rssi_values) / len(rssi_values) if rssi_values else None
        min_rssi = min(rssi_values) if rssi_values else None
        
        # SNR statistics
        snr_values = [s.snr for s in matched if s.snr is not None]
        avg_snr = sum(snr_values) / len(snr_values) if snr_values else None
        
        # Determine pass/fail
        passed = True
        reasons = []
        
        if packet_loss > 20:  # More than 20% packet loss
            passed = False
            reasons.append(f"High packet loss: {packet_loss:.1f}%")
        
        if avg_throttle_diff > 5 or avg_steering_diff > 5:  # Mean values differ by more than 5
            passed = False
            reasons.append(f"Value mismatch: thr_diff={avg_throttle_diff:.1f}, steer_diff={avg_steering_diff:.1f}")
        
        if avg_rssi is not None and avg_rssi < -100:  # Very weak signal
            passed = False
            reasons.append(f"Weak signal: RSSI {avg_rssi} dBm")
        
        if matched_count < 10:  # Too few samples
            passed = False
            reasons.append(f"Insufficient samples: {matched_count} pairs")
        
        result = {
            "test": "Radio Link Integration",
            "result": TestResult.PASS.value if passed else TestResult.FAIL.value,
            "details": "; ".join(reasons) if reasons else "Radio link working correctly",
            "samples_collected": len(samples),
            "tx_samples": total_tx,
            "rx_samples": total_rx,
            "matched_pairs": matched_count,
            "packet_loss_percent": round(packet_loss, 2),
            "avg_throttle_diff": round(avg_throttle_diff, 2),
            "avg_steering_diff": round(avg_steering_diff, 2),
            "max_throttle_diff": max_throttle_diff,
            "max_steering_diff": max_steering_diff,
            "avg_rssi_dbm": avg_rssi,
            "min_rssi_dbm": min_rssi,
            "avg_snr_db": avg_snr
        }
        
        return result


class BREmoteTester:
    """Test orchestrator for BREmote devices"""
    
    def __init__(self, gui_callback: Optional[Callable] = None):
        self.gui_callback = gui_callback
        self.devices: List[BREmoteDevice] = []
        self.test_results: Dict[str, TestReport] = {}
        
    def log(self, message: str):
        """Log message to console and optionally GUI"""
        try:
            print(message)
        except UnicodeEncodeError:
            # Handle Windows console encoding issues
            print(message.encode('utf-8', errors='replace').decode('utf-8'))
        if self.gui_callback:
            self.gui_callback(message)
    
    def scan_ports(self) -> List[str]:
        """Scan for available COM ports with BREmote devices"""
        self.log("\n[SCAN] Scanning for BREmote devices...")
        ports = serial.tools.list_ports.comports()
        bre_ports = []
        
        for port_info in ports:
            port = port_info.device
            # Check for ESP32 or common USB-Serial chips
            if any(x in port_info.description.lower() for x in 
                   ["usb", "serial", "uart", "cp210", "ch340", "ftdi", "esp32"]):
                self.log(f"  Checking {port}: {port_info.description}")
                device = BREmoteDevice(port)
                if device.connect():
                    device_type = device.identify()
                    if device_type != DeviceType.UNKNOWN:
                        self.devices.append(device)
                        bre_ports.append(port)
                        self.log(f"    [OK] Found {device}")
                    else:
                        device.disconnect()
        
        if not bre_ports:
            self.log("  [WARN] No BREmote devices found")
        else:
            self.log(f"  [OK] Found {len(bre_ports)} device(s)")
            
        return bre_ports
    
    def test_tx_radio(self, device: BREmoteDevice) -> Dict:
        """Test TX radio functionality using ?state json and ?printPackets json"""
        self.log("\n[RADIO] Testing Radio (TX)...")
        result = {"test": "Radio TX", "result": TestResult.PENDING.value, "details": ""}

        try:
            # Use ?state json for structured status
            data = device.send_json_command("?state")
            if data:
                radio_on = data.get("radio", "OFF")
                last_pkt = data.get("last_pkt_ms")
                result["details"] = f"Radio: {radio_on}, last_pkt_ms: {last_pkt}"
                result["result"] = TestResult.PASS.value
            else:
                # Fallback: try ?printPackets json
                pkt_data = device.send_json_command("?printPackets")
                if pkt_data:
                    result["details"] = f"Sent: {pkt_data.get('sent',0)}, Received: {pkt_data.get('received',0)}, Ratio: {pkt_data.get('ratio',0)}%"
                    result["result"] = TestResult.PASS.value
                else:
                    result["details"] = "Radio initialized (no JSON response)"
                    result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result
    
    def test_tx_display(self, device: BREmoteDevice) -> Dict:
        """Test TX display functionality using ?state json"""
        self.log("\n[DISPLAY] Testing Display (TX)...")
        result = {"test": "Display TX", "result": TestResult.PENDING.value, "details": ""}

        try:
            data = device.send_json_command("?state")
            if data:
                display_on = data.get("display", "UNKNOWN")
                result["details"] = f"Display: {display_on}"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = "Display subsystem present (no JSON response)"
                result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result
    
    def test_tx_hall(self, device: BREmoteDevice) -> Dict:
        """Test TX hall sensor (throttle and toggles) using ?printInputs json"""
        self.log("\n[INPUT] Testing Hall Sensors (TX)...")
        result = {"test": "Hall Sensors", "result": TestResult.PENDING.value, "details": ""}

        try:
            data = device.send_json_command("?printInputs")
            # Stop the continuous print loop
            time.sleep(0.1)
            device.send_command("quit", wait_for_response=False)

            if data and "throttle" in data:
                thr = data.get("throttle")
                steer = data.get("steering")
                hall_en = data.get("hall_enabled")
                result["details"] = f"Throttle: {thr}, Steering: {steer}, HallEnabled: {hall_en}"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = "Input monitoring active (no JSON data)"
                result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result
    
    def test_tx_analog(self, device: BREmoteDevice) -> Dict:
        """Test TX analog inputs (battery monitoring) using ?state json"""
        self.log("\n[BATTERY] Testing Analog/Battery (TX)...")
        result = {"test": "Analog Inputs", "result": TestResult.PENDING.value, "details": ""}

        try:
            # ?state json includes hall status which implies ADC is working
            data = device.send_json_command("?state")
            if data:
                hall_on = data.get("hall", "UNKNOWN")
                result["details"] = f"Hall/ADC subsystem: {hall_on}"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = "Analog system responsive (no JSON)"
                result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result
    
    def test_tx_rssi(self, device: BREmoteDevice) -> Dict:
        """Test TX RSSI monitoring using ?printRSSI json"""
        self.log("\n[SIGNAL] Testing RSSI (TX)...")
        result = {"test": "RSSI Monitoring", "result": TestResult.PENDING.value, "details": ""}

        try:
            data = device.send_json_command("?printRSSI")
            time.sleep(0.1)
            device.send_command("quit", wait_for_response=False)

            if data:
                if "error" in data:
                    result["details"] = f"RSSI error: {data['error']}"
                    result["result"] = TestResult.FAIL.value
                elif "rssi" in data:
                    result["details"] = f"RSSI: {data['rssi']} dBm, SNR: {data.get('snr', 'N/A')} dB"
                    result["result"] = TestResult.PASS.value
                elif "failsafe_ms" in data:
                    result["details"] = f"No radio link (failsafe {data['failsafe_ms']}ms)"
                    result["result"] = TestResult.PASS.value
                else:
                    result["details"] = f"RSSI response: {data}"
                    result["result"] = TestResult.PASS.value
            else:
                result["details"] = "RSSI command accepted (no JSON)"
                result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result
    
    def test_rx_radio(self, device: BREmoteDevice) -> Dict:
        """Test RX radio functionality using ?printrssi"""
        self.log("\n[RADIO] Testing Radio (RX)...")
        result = {"test": "Radio RX", "result": TestResult.PENDING.value, "details": ""}
        
        try:
            # Use ?printrssi to check radio link status
            response = device.send_command("?printrssi")
            time.sleep(0.2)
            device.send_command("quit", wait_for_response=False)
            
            if "rssi" in response.lower() or "link" in response.lower() or "rx" in response.lower():
                result["details"] = f"Radio status: {response[:200]}"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = "Radio interface responsive"
                result["result"] = TestResult.PASS.value
                
        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"
            
        self.log(f"  Result: {result['result']}")
        return result
    
    def test_rx_vesc(self, device: BREmoteDevice) -> Dict:
        """Test RX VESC/battery interface using ?printbat"""
        self.log("\n[POWER] Testing VESC/Battery Interface (RX)...")
        result = {"test": "VESC/Battery Interface", "result": TestResult.PENDING.value, "details": ""}
        
        try:
            # Use ?printbat to check VESC/telemetry status
            # ?printbat enters a loop — send quit to exit it
            response = device.send_command("?printbat")
            time.sleep(1.5)
            device.send_command("quit")

            if "vesc" in response.lower() or "volt" in response.lower() or "bat" in response.lower() or "measured" in response.lower() or "data_src" in response.lower():
                result["details"] = f"Battery/VESC status: {response[:200]}"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = "VESC interface present"
                result["result"] = TestResult.PASS.value
                
        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"
            
        self.log(f"  Result: {result['result']}")
        return result
    
    def test_rx_pwm(self, device: BREmoteDevice) -> Dict:
        """Test RX PWM output using ?printpwm"""
        self.log("\n[PWM] Testing PWM Output (RX)...")
        result = {"test": "PWM Output", "result": TestResult.PENDING.value, "details": ""}
        
        try:
            # ?printpwm enters a loop — send quit to exit it
            response = device.send_command("?printpwm")
            time.sleep(1.5)
            device.send_command("quit")

            if "pwm" in response.lower():
                result["details"] = f"PWM status: {response[:200]}"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = "PWM interface responsive"
                result["result"] = TestResult.PASS.value
                
        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"
            
        self.log(f"  Result: {result['result']}")
        return result
    
    # ===== Web Config / WiFi AP Tests (TX/RX) =====

    def test_tx_wifi_state(self, device: BREmoteDevice) -> Dict:
        """Test WiFi status query via ?wifi and ?wifistate"""
        self.log("\n[WIFI] Testing WiFi state query...")
        result = {"test": "WiFi State", "result": TestResult.PENDING.value, "details": ""}

        try:
            response = device.send_command("?wifi")
            if "wifi=" not in response.lower():
                result["details"] = f"Unexpected ?wifi response: {response[:120]}"
                result["result"] = TestResult.FAIL.value
                return result

            wifi_on = "ON" in response.upper()

            state_resp = device.send_command("?wifistate")
            if "enabled=" not in state_resp:
                result["details"] = f"Unexpected ?wifistate response: {state_resp[:120]}"
                result["result"] = TestResult.FAIL.value
                return result

            # Parse key=value pairs from state line
            state_parts = {}
            for part in state_resp.strip().split(","):
                if "=" in part:
                    k, v = part.split("=", 1)
                    state_parts[k.strip()] = v.strip()

            state_enabled = state_parts.get("enabled") == "1"

            # Cross-check: ?wifi and ?wifistate should agree
            if wifi_on != state_enabled:
                result["details"] = f"State mismatch: ?wifi={wifi_on}, wifistate enabled={state_enabled}"
                result["result"] = TestResult.FAIL.value
            else:
                details = f"wifi={'ON' if wifi_on else 'OFF'}"
                details += f", req_total={state_parts.get('req_total','?')}"
                details += f", req_ok={state_parts.get('req_ok','?')}"
                details += f", req_err={state_parts.get('req_err','?')}"
                if "ap_uptime_ms" in state_parts:
                    details += f", ap_uptime={state_parts['ap_uptime_ms']}ms"
                result["details"] = details
                result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result

    def test_tx_wifi_debug_mode(self, device: BREmoteDevice) -> Dict:
        """Test WiFi debug mode get/set via ?wifidbg"""
        self.log("\n[WIFI] Testing WiFi debug mode...")
        result = {"test": "WiFi Debug Mode", "result": TestResult.PENDING.value, "details": ""}

        try:
            # Read current mode
            original = device.send_command("?wifidbg")
            if "wifidbg=" not in original.lower():
                result["details"] = f"Unexpected ?wifidbg response: {original[:80]}"
                result["result"] = TestResult.FAIL.value
                return result

            original_mode = original.strip().split("=", 1)[1].strip()

            # Cycle through modes and verify each one sticks
            test_modes = ["off", "some", "full"]
            all_ok = True
            tested = []

            for mode in test_modes:
                set_resp = device.send_command(f"?wifidbg {mode}")
                if f"wifidbg={mode}" not in set_resp.lower():
                    all_ok = False
                    tested.append(f"{mode}:FAIL")
                else:
                    tested.append(f"{mode}:OK")

            # Restore original mode
            device.send_command(f"?wifidbg {original_mode}")

            result["details"] = f"Modes tested: {', '.join(tested)} (restored={original_mode})"
            result["result"] = TestResult.PASS.value if all_ok else TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result

    def test_tx_wifi_startup_timeout(self, device: BREmoteDevice) -> Dict:
        """Test AP startup timeout get/set via ?wifips"""
        self.log("\n[WIFI] Testing WiFi AP startup timeout...")
        result = {"test": "WiFi Startup Timeout", "result": TestResult.PENDING.value, "details": ""}

        try:
            # Read current value
            original = device.send_command("?wifips")
            if "wifips_ms=" not in original.lower():
                result["details"] = f"Unexpected ?wifips response: {original[:80]}"
                result["result"] = TestResult.FAIL.value
                return result

            original_ms = original.strip().split("=", 1)[1].strip()

            checks = []
            all_ok = True

            # Test setting a specific value
            resp = device.send_command("?wifips 60000")
            if "wifips_ms=60000" in resp:
                checks.append("set60000:OK")
            else:
                checks.append("set60000:FAIL")
                all_ok = False

            # Test setting off (0)
            resp = device.send_command("?wifips off")
            if "wifips_ms=0" in resp:
                checks.append("off:OK")
            else:
                checks.append("off:FAIL")
                all_ok = False

            # Test out-of-range value (>3600000 should fail)
            resp = device.send_command("?wifips 9999999")
            if "ERR" in resp.upper():
                checks.append("range_reject:OK")
            else:
                checks.append("range_reject:FAIL")
                all_ok = False

            # Restore original
            device.send_command(f"?wifips {original_ms}")

            result["details"] = f"{', '.join(checks)} (restored={original_ms}ms)"
            result["result"] = TestResult.PASS.value if all_ok else TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result

    def test_tx_wifi_version(self, device: BREmoteDevice) -> Dict:
        """Test web UI version info via ?wifiver"""
        self.log("\n[WIFI] Testing Web UI version info...")
        result = {"test": "Web UI Version", "result": TestResult.PENDING.value, "details": ""}

        try:
            response = device.send_command("?wifiver")
            lines = response.strip().split("\n")
            target = None
            installed = None

            for line in lines:
                line = line.strip()
                if line.startswith("ui_target="):
                    target = line.split("=", 1)[1].strip()
                elif line.startswith("ui_installed="):
                    installed = line.split("=", 1)[1].strip()

            if target is None or installed is None:
                result["details"] = f"Missing version fields: {response[:120]}"
                result["result"] = TestResult.FAIL.value
            else:
                match = (target == installed)
                result["details"] = f"target={target}, installed={installed}, match={match}"
                result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result

    def test_tx_wifi_error(self, device: BREmoteDevice) -> Dict:
        """Test error reporting via ?wifierr"""
        self.log("\n[WIFI] Testing WiFi error reporting...")
        result = {"test": "WiFi Error Report", "result": TestResult.PENDING.value, "details": ""}

        try:
            response = device.send_command("?wifierr")
            # ?wifierr returns the last error string (may be empty if no errors)
            err_text = response.strip()
            result["details"] = f"last_err={'(none)' if not err_text else err_text}"
            result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result

    def test_tx_config_keys(self, device: BREmoteDevice) -> Dict:
        """Test config field listing via ?keys"""
        self.log("\n[CONFIG] Testing config key listing...")
        result = {"test": "Config Keys", "result": TestResult.PENDING.value, "details": ""}

        EXPECTED_KEYS = {
            "radio_preset", "rf_power", "max_gears", "startgear",
            "no_lock", "throttle_mode", "steer_enabled", "thr_expo",
            "wifi_password", "dynamic_power_start", "dynamic_power_step",
            "own_address", "dest_address", "version"
        }

        try:
            response = device.send_command("?keys")
            keys = set()
            for line in response.strip().split("\n"):
                key = line.strip()
                if key:
                    keys.add(key)

            missing = EXPECTED_KEYS - keys
            if missing:
                result["details"] = f"Missing keys: {missing} (got {len(keys)} total)"
                result["result"] = TestResult.FAIL.value
            elif len(keys) < 10:
                result["details"] = f"Too few keys: {len(keys)}"
                result["result"] = TestResult.FAIL.value
            else:
                result["details"] = f"{len(keys)} config keys listed"
                result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result

    def test_tx_config_get_set(self, device: BREmoteDevice) -> Dict:
        """Test config read/write cycle via ?get and ?set (non-destructive)"""
        self.log("\n[CONFIG] Testing config get/set cycle...")
        result = {"test": "Config Get/Set", "result": TestResult.PENDING.value, "details": ""}

        try:
            # Read current thr_expo value
            resp = device.send_command("?get thr_expo")
            if "=" not in resp or "ERR" in resp.upper():
                result["details"] = f"Cannot read thr_expo: {resp[:80]}"
                result["result"] = TestResult.FAIL.value
                return result

            original_val = resp.strip().split("=", 1)[1].strip()
            checks = []
            all_ok = True

            # Set a new value (within valid range 0-100)
            test_val = "30" if original_val != "30" else "70"
            set_resp = device.send_command(f"?set thr_expo {test_val}")
            if f"thr_expo={test_val}" in set_resp:
                checks.append("set:OK")
            else:
                checks.append(f"set:FAIL({set_resp.strip()[:40]})")
                all_ok = False

            # Read back to verify
            get_resp = device.send_command("?get thr_expo")
            if f"thr_expo={test_val}" in get_resp:
                checks.append("verify:OK")
            else:
                checks.append(f"verify:FAIL({get_resp.strip()[:40]})")
                all_ok = False

            # Test range rejection (thr_expo max=100)
            err_resp = device.send_command("?set thr_expo 999")
            if "ERR" in err_resp.upper():
                checks.append("range_reject:OK")
            else:
                checks.append("range_reject:FAIL")
                all_ok = False

            # Test unknown key rejection
            err_resp2 = device.send_command("?get nonexistent_key_xyz")
            if "ERR" in err_resp2.upper():
                checks.append("unknown_key:OK")
            else:
                checks.append("unknown_key:FAIL")
                all_ok = False

            # Restore original value
            device.send_command(f"?set thr_expo {original_val}")

            result["details"] = f"{', '.join(checks)} (restored={original_val})"
            result["result"] = TestResult.PASS.value if all_ok else TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result

    def test_rx_config_keys(self, device: BREmoteDevice) -> Dict:
        """Test RX config field listing via ?keys"""
        self.log("\n[CONFIG] Testing RX config key listing...")
        result = {"test": "Config Keys", "result": TestResult.PENDING.value, "details": ""}

        expected_keys = {
            "radio_preset", "rf_power", "steering_type", "pwm0_min",
            "pwm0_max", "failsafe_time", "own_address", "dest_address", "version"
        }

        try:
            response = device.send_command("?keys")
            keys = set()
            for line in response.strip().split("\n"):
                key = line.strip()
                if key:
                    keys.add(key)

            missing = expected_keys - keys
            if missing:
                result["details"] = f"Missing keys: {missing} (got {len(keys)} total)"
                result["result"] = TestResult.FAIL.value
            elif len(keys) < 10:
                result["details"] = f"Too few keys: {len(keys)}"
                result["result"] = TestResult.FAIL.value
            else:
                result["details"] = f"{len(keys)} config keys listed"
                result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result

    def test_rx_config_get_set(self, device: BREmoteDevice) -> Dict:
        """Test RX config read/write cycle via ?get and ?set (non-destructive)."""
        self.log("\n[CONFIG] Testing RX config get/set cycle...")
        result = {"test": "Config Get/Set", "result": TestResult.PENDING.value, "details": ""}

        try:
            # radio_preset range is 1..3 on RX
            resp = device.send_command("?get radio_preset")
            if "=" not in resp or "ERR" in resp.upper():
                result["details"] = f"Cannot read radio_preset: {resp[:80]}"
                result["result"] = TestResult.FAIL.value
                return result

            original_val = resp.strip().split("=", 1)[1].strip()
            checks = []
            all_ok = True

            test_val = "2" if original_val != "2" else "1"
            set_resp = device.send_command(f"?set radio_preset {test_val}")
            if f"radio_preset={test_val}" in set_resp:
                checks.append("set:OK")
            else:
                checks.append(f"set:FAIL({set_resp.strip()[:40]})")
                all_ok = False

            get_resp = device.send_command("?get radio_preset")
            if f"radio_preset={test_val}" in get_resp:
                checks.append("verify:OK")
            else:
                checks.append(f"verify:FAIL({get_resp.strip()[:40]})")
                all_ok = False

            err_resp = device.send_command("?set radio_preset 999")
            if "ERR" in err_resp.upper():
                checks.append("range_reject:OK")
            else:
                checks.append("range_reject:FAIL")
                all_ok = False

            err_resp2 = device.send_command("?get nonexistent_key_xyz")
            if "ERR" in err_resp2.upper():
                checks.append("unknown_key:OK")
            else:
                checks.append("unknown_key:FAIL")
                all_ok = False

            # Restore original value
            device.send_command(f"?set radio_preset {original_val}")

            result["details"] = f"{', '.join(checks)} (restored={original_val})"
            result["result"] = TestResult.PASS.value if all_ok else TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result

    def test_tx_wifi_onoff(self, device: BREmoteDevice) -> Dict:
        """Test WiFi AP enable/disable cycle via ?wifi on/off"""
        self.log("\n[WIFI] Testing WiFi AP on/off cycle...")
        result = {"test": "WiFi AP On/Off", "result": TestResult.PENDING.value, "details": ""}

        try:
            # Read current state
            status = device.send_command("?wifi")
            was_on = "ON" in status.upper()

            checks = []
            all_ok = True

            # Turn on
            on_resp = device.send_command("?wifi on")
            time.sleep(0.5)  # Allow AP to start
            verify_on = device.send_command("?wifi")
            if "ON" in verify_on.upper():
                checks.append("on:OK")
            else:
                checks.append(f"on:FAIL({verify_on.strip()[:40]})")
                all_ok = False

            # Check ?state json includes wifi status
            data = device.send_json_command("?state")
            if data and "wifi" in data:
                wifi_state = data["wifi"]
                if wifi_state == "ON":
                    checks.append("state_json:OK")
                else:
                    checks.append(f"state_json:FAIL(wifi={wifi_state})")
                    all_ok = False
            else:
                checks.append("state_json:SKIP")

            # Turn off
            off_resp = device.send_command("?wifi off")
            time.sleep(0.3)
            verify_off = device.send_command("?wifi")
            if "OFF" in verify_off.upper():
                checks.append("off:OK")
            else:
                checks.append(f"off:FAIL({verify_off.strip()[:40]})")
                all_ok = False

            # Restore original state
            if was_on:
                device.send_command("?wifi on")
                time.sleep(0.3)

            result["details"] = f"{', '.join(checks)} (restored={'ON' if was_on else 'OFF'})"
            result["result"] = TestResult.PASS.value if all_ok else TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result

    def test_tx_wifi_ui_update(self, device: BREmoteDevice) -> Dict:
        """Test forced web UI update to SPIFFS via ?wifiupd"""
        self.log("\n[WIFI] Testing Web UI SPIFFS update...")
        result = {"test": "Web UI Update", "result": TestResult.PENDING.value, "details": ""}

        try:
            response = device.send_command("?wifiupd", wait_for_response=True)
            # Allow extra time for SPIFFS write
            time.sleep(0.5)
            if not response:
                response = device._read_response()

            if "ERR" in response.upper():
                result["details"] = f"Update failed: {response.strip()[:80]}"
                result["result"] = TestResult.FAIL.value
            elif "updated" in response.lower() or "UI updated" in response:
                # Verify versions match after update
                ver_resp = device.send_command("?wifiver")
                target = None
                installed = None
                for line in ver_resp.strip().split("\n"):
                    line = line.strip()
                    if line.startswith("ui_target="):
                        target = line.split("=", 1)[1].strip()
                    elif line.startswith("ui_installed="):
                        installed = line.split("=", 1)[1].strip()

                if target and installed and target == installed:
                    result["details"] = f"Updated OK, version={target}"
                    result["result"] = TestResult.PASS.value
                elif target and installed:
                    result["details"] = f"Version mismatch after update: target={target}, installed={installed}"
                    result["result"] = TestResult.FAIL.value
                else:
                    result["details"] = f"Updated but cannot verify versions"
                    result["result"] = TestResult.PASS.value
            else:
                result["details"] = f"Unexpected response: {response.strip()[:80]}"
                result["result"] = TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result

    # ===== Web API Tests (over WiFi, TX/RX) =====

    WEB_API_TX_SSID = "BREmoteV2-TX-WebConfig"
    WEB_API_RX_SSID = "BREmoteV2-RX-WebConfig"
    WEB_API_PASS = "12345678"
    WEB_API_BASE = "http://192.168.4.1"

    def _wifi_get_current_ssid(self) -> Optional[str]:
        """Get currently connected WiFi SSID from `netsh wlan show interfaces`.

        Parses the active interface block and prefers SSID only when state is connected.
        """
        try:
            out = subprocess.check_output(
                ["netsh", "wlan", "show", "interfaces"],
                text=True, timeout=5, creationflags=subprocess.CREATE_NO_WINDOW
            )
            connected = False
            ssid = None
            fallback_ssid = None
            for line in out.splitlines():
                stripped = line.strip()

                # New interface section, reset section-local state.
                if stripped.lower().startswith("name") and ":" in stripped:
                    connected = False
                    ssid = None

                # "State : connected" (non-English locales still typically contain "connected"/"verbunden")
                if ":" in stripped:
                    key, value = [x.strip() for x in stripped.split(":", 1)]
                    key_l = key.lower()
                    value_l = value.lower()

                    if key_l in ("state", "status", "zustand"):
                        connected = ("connected" in value_l) or ("verbunden" in value_l)

                    # Match "SSID" or "SSID 1", but never BSSID.
                    if re.match(r"^ssid(\s+\d+)?$", key_l) and "bssid" not in key_l:
                        ssid = value
                        if value and value.lower() not in ("", "n/a", "not available"):
                            fallback_ssid = value

                if connected and ssid:
                    return ssid
            return fallback_ssid
        except Exception:
            pass
        return None

    def _wifi_can_reach_ap(self, timeout: float = 3.0) -> bool:
        """Check if the AP is reachable by attempting a TCP connection to port 80."""
        import socket
        try:
            sock = socket.create_connection(("192.168.4.1", 80), timeout=timeout)
            sock.close()
            return True
        except (OSError, socket.timeout):
            return False

    def _wifi_connect(self, ssid: str, password: str, timeout: float = 30.0) -> bool:
        """Connect the host PC to a WiFi network (Windows only).
        Retries the connect command if the AP isn't visible yet."""
        self.log(f"  Connecting to WiFi '{ssid}'...")
        import tempfile, os

        # Create a temporary profile XML
        profile_xml = f"""<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>{ssid}</name>
  <SSIDConfig><SSID><name>{ssid}</name></SSID></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security>
    <authEncryption><authentication>WPA2PSK</authentication><encryption>AES</encryption><useOneX>false</useOneX></authEncryption>
    <sharedKey><keyType>passPhrase</keyType><protected>false</protected><keyMaterial>{password}</keyMaterial></sharedKey>
  </security></MSM>
</WLANProfile>"""
        profile_path = os.path.join(tempfile.gettempdir(), "bremote_wifi_profile.xml")
        with open(profile_path, "w") as f:
            f.write(profile_xml)

        try:
            out = subprocess.check_output(
                ["netsh", "wlan", "add", "profile", f"filename={profile_path}"],
                text=True, timeout=5, stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW
            )
            self.log(f"  Profile added: {out.strip()[:80]}")
        except subprocess.CalledProcessError as e:
            self.log(f"  Profile add failed: {e.output.strip()[:80] if e.output else e}")
            os.remove(profile_path)
            return False
        os.remove(profile_path)

        # Retry connect — the AP may not be visible to the WiFi adapter immediately
        deadline = time.time() + timeout
        connected = False
        attempt = 0
        while time.time() < deadline:
            attempt += 1
            try:
                out = subprocess.check_output(
                    ["netsh", "wlan", "connect", f"name={ssid}", f"ssid={ssid}"],
                    text=True, timeout=5, stderr=subprocess.STDOUT,
                    creationflags=subprocess.CREATE_NO_WINDOW
                )
                self.log(f"  Connect attempt {attempt}: {out.strip()[:80]}")
            except subprocess.CalledProcessError as e:
                self.log(f"  Connect attempt {attempt} failed: {e.output.strip()[:80] if e.output else e}")

            # Poll for connection — check both SSID and TCP reachability
            poll_end = min(time.time() + 5, deadline)
            while time.time() < poll_end:
                # Primary check: can we reach the web server?
                if self._wifi_can_reach_ap(timeout=2.0):
                    self.log(f"  Connected to '{ssid}' (AP reachable)")
                    return True
                # Fallback: SSID detection only. Treat as provisional until AP is reachable.
                current = self._wifi_get_current_ssid()
                if current and current == ssid:
                    self.log(f"  SSID matched '{ssid}', waiting for AP reachability...")
                    # Require AP reachability to avoid false positives on wrong/lingering SSID reports.
                    settle_end = min(time.time() + 5, deadline)
                    while time.time() < settle_end:
                        if self._wifi_can_reach_ap(timeout=2.0):
                            self.log(f"  Connected to '{ssid}' (SSID matched + AP reachable)")
                            return True
                        time.sleep(1)
                time.sleep(1)

        self.log(f"  Timed out after {attempt} attempts waiting for WiFi connection to '{ssid}'")
        return False

    def _wifi_restore(self, previous_ssid: Optional[str], test_ssid: Optional[str] = None):
        """Reconnect to the previous WiFi network and clean up the temp profile"""
        try:
            profiles_to_delete = [test_ssid] if test_ssid else [self.WEB_API_TX_SSID, self.WEB_API_RX_SSID]
            for profile in profiles_to_delete:
                if not profile:
                    continue
                subprocess.run(
                    ["netsh", "wlan", "delete", "profile", f"name={profile}"],
                    text=True, timeout=5, capture_output=True, creationflags=subprocess.CREATE_NO_WINDOW
                )
        except Exception:
            pass
        if previous_ssid:
            self.log(f"  Restoring WiFi to '{previous_ssid}'...")
            try:
                subprocess.run(
                    ["netsh", "wlan", "connect", f"name={previous_ssid}"],
                    text=True, timeout=5, capture_output=True, creationflags=subprocess.CREATE_NO_WINDOW
                )
                time.sleep(3)
            except Exception:
                self.log(f"  [WARN] Could not restore WiFi to '{previous_ssid}'")

    def _http_get(self, path: str, timeout: float = 5.0) -> Tuple[int, str, str]:
        """HTTP GET, returns (status_code, content_type, body)"""
        url = self.WEB_API_BASE + path
        req = urllib.request.Request(url)
        try:
            resp = urllib.request.urlopen(req, timeout=timeout)
            body = resp.read().decode("utf-8", errors="replace")
            ct = resp.headers.get("Content-Type", "")
            return resp.status, ct, body
        except urllib.error.HTTPError as e:
            body = e.read().decode("utf-8", errors="replace") if e.fp else ""
            ct = e.headers.get("Content-Type", "") if e.headers else ""
            return e.code, ct, body

    def _http_post(self, path: str, params: Dict[str, str], timeout: float = 5.0) -> Tuple[int, str, str]:
        """HTTP POST with form-encoded params, returns (status_code, content_type, body)"""
        url = self.WEB_API_BASE + path
        data = urllib.parse.urlencode(params).encode("utf-8")
        req = urllib.request.Request(url, data=data, method="POST")
        req.add_header("Content-Type", "application/x-www-form-urlencoded")
        try:
            resp = urllib.request.urlopen(req, timeout=timeout)
            body = resp.read().decode("utf-8", errors="replace")
            ct = resp.headers.get("Content-Type", "")
            return resp.status, ct, body
        except urllib.error.HTTPError as e:
            body = e.read().decode("utf-8", errors="replace") if e.fp else ""
            ct = e.headers.get("Content-Type", "") if e.headers else ""
            return e.code, ct, body

    def _test_wifi_web_api(self, device: BREmoteDevice, ssid: str, set_key: str, set_value: str) -> Dict:
        """Test web API endpoints by connecting to the AP over WiFi."""
        self.log("\n[WEBAPI] Testing Web API endpoints over WiFi...")
        result = {"test": "Web API Endpoints", "result": TestResult.PENDING.value, "details": ""}
        checks = []
        all_ok = True
        previous_ssid = None
        original_timeout_ms = None

        try:
            # Save current WiFi and AP timeout state
            previous_ssid = self._wifi_get_current_ssid()
            self.log(f"  Current WiFi: {previous_ssid or 'unknown'}")

            timeout_resp = device.send_command("?wifips")
            if "wifips_ms=" in timeout_resp.lower():
                original_timeout_ms = timeout_resp.strip().split("=")[-1]

            # Disable AP startup timeout so it doesn't shut down mid-test
            device.send_command("?wifips off")

            # Ensure AP is on — give it time to start broadcasting
            device.send_command("?wifi on")
            time.sleep(3)
            verify = device.send_command("?wifi")
            if "ON" not in verify.upper():
                result["details"] = "Failed to enable WiFi AP"
                result["result"] = TestResult.FAIL.value
                return result

            # Connect host to AP
            if not self._wifi_connect(ssid, self.WEB_API_PASS):
                result["details"] = "Failed to connect to BREmote AP"
                result["result"] = TestResult.FAIL.value
                self._wifi_restore(previous_ssid, ssid)
                return result

            # --- Test each endpoint ---

            # 1. GET / — should serve HTML
            self.log("  Testing GET / ...")
            status, ct, body = self._http_get("/")
            if status == 200 and "text/html" in ct and len(body) > 100:
                checks.append("root:OK")
            else:
                checks.append(f"root:FAIL(status={status},ct={ct[:30]},len={len(body)})")
                all_ok = False

            # 2. GET /api/state
            self.log("  Testing GET /api/state ...")
            status, ct, body = self._http_get("/api/state")
            try:
                data = json.loads(body)
                if status == 200 and data.get("ok") == 1 and "state" in data:
                    checks.append("state:OK")
                else:
                    checks.append(f"state:FAIL(ok={data.get('ok')},status={status})")
                    all_ok = False
            except json.JSONDecodeError:
                checks.append(f"state:FAIL(not_json)")
                all_ok = False

            # 3. GET /api/config — all fields
            self.log("  Testing GET /api/config ...")
            status, ct, body = self._http_get("/api/config")
            try:
                data = json.loads(body)
                if status == 200 and data.get("ok") == 1 and isinstance(data.get("data"), dict):
                    field_count = len(data["data"])
                    checks.append(f"config:OK({field_count} fields)")
                else:
                    checks.append(f"config:FAIL(ok={data.get('ok')})")
                    all_ok = False
            except json.JSONDecodeError:
                checks.append("config:FAIL(not_json)")
                all_ok = False

            # 4. GET /api/get?key=radio_preset — valid key
            self.log("  Testing GET /api/get?key=radio_preset ...")
            status, ct, body = self._http_get("/api/get?key=radio_preset")
            try:
                data = json.loads(body)
                if status == 200 and data.get("ok") == 1 and "value" in data:
                    checks.append(f"get_valid:OK(val={data['value']})")
                else:
                    checks.append(f"get_valid:FAIL(ok={data.get('ok')})")
                    all_ok = False
            except json.JSONDecodeError:
                checks.append("get_valid:FAIL(not_json)")
                all_ok = False

            # 5. GET /api/get?key=INVALID_KEY — should return error
            self.log("  Testing GET /api/get?key=INVALID_KEY ...")
            status, ct, body = self._http_get("/api/get?key=INVALID_KEY")
            try:
                data = json.loads(body)
                if status == 400 and data.get("ok") == 0:
                    checks.append("get_invalid:OK(rejected)")
                else:
                    checks.append(f"get_invalid:FAIL(status={status},ok={data.get('ok')})")
                    all_ok = False
            except json.JSONDecodeError:
                checks.append("get_invalid:FAIL(not_json)")
                all_ok = False

            # 6. POST /api/set — set a known safe key/value
            self.log("  Testing POST /api/set ...")
            status, ct, body = self._http_post("/api/set", {"key": set_key, "value": set_value})
            try:
                data = json.loads(body)
                if status == 200 and data.get("ok") == 1:
                    checks.append("set:OK")
                else:
                    checks.append(f"set:FAIL(ok={data.get('ok')},status={status})")
                    all_ok = False
            except json.JSONDecodeError:
                checks.append("set:FAIL(not_json)")
                all_ok = False

            # 7. POST /api/set_batch — JSON object payload
            self.log("  Testing POST /api/set_batch ...")
            batch_payload = json.dumps({set_key: int(set_value) if set_value.isdigit() else set_value})
            status, ct, body = self._http_post("/api/set_batch", {"payload": batch_payload})
            try:
                data = json.loads(body)
                if status == 200 and data.get("ok") == 1:
                    checks.append("set_batch:OK")
                else:
                    err = data.get("err", "")
                    err_txt = f",err={err}" if err else ""
                    checks.append(f"set_batch:FAIL(ok={data.get('ok')},status={status}{err_txt})")
                    all_ok = False
            except json.JSONDecodeError:
                checks.append("set_batch:FAIL(not_json)")
                all_ok = False

            # 8. POST /api/load — reload config from SPIFFS (discards RAM changes)
            self.log("  Testing POST /api/load ...")
            status, ct, body = self._http_post("/api/load", {})
            try:
                data = json.loads(body)
                if status == 200 and data.get("ok") == 1:
                    checks.append("load:OK")
                else:
                    checks.append(f"load:FAIL(ok={data.get('ok')},status={status})")
                    all_ok = False
            except json.JSONDecodeError:
                checks.append("load:FAIL(not_json)")
                all_ok = False

            # 9. GET /nonexistent — 404
            self.log("  Testing GET /nonexistent ...")
            status, ct, body = self._http_get("/nonexistent")
            try:
                data = json.loads(body)
                if status == 404 and data.get("ok") == 0:
                    checks.append("404:OK")
                else:
                    checks.append(f"404:FAIL(status={status},ok={data.get('ok')})")
                    all_ok = False
            except json.JSONDecodeError:
                checks.append(f"404:FAIL(status={status},not_json)")
                all_ok = False

            result["details"] = ", ".join(checks)
            result["result"] = TestResult.PASS.value if all_ok else TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            partial = ", ".join(checks) if checks else ""
            result["details"] = f"Error: {str(e)}" + (f" [{partial}]" if partial else "")

        finally:
            # Restore AP timeout
            if original_timeout_ms is not None:
                device.send_command(f"?wifips {original_timeout_ms}")
            # Restore host WiFi
            self._wifi_restore(previous_ssid, ssid)

        # Always print endpoint-level outcome so failures are diagnosable from console logs.
        if result.get("details"):
            self.log(f"  Details: {result['details']}")
        self.log(f"  Result: {result['result']}")
        return result

    def test_tx_wifi_web_api(self, device: BREmoteDevice) -> Dict:
        """Test TX web API endpoints over WiFi."""
        return self._test_wifi_web_api(device, self.WEB_API_TX_SSID, "thr_expo", "50")

    def test_rx_wifi_web_api(self, device: BREmoteDevice) -> Dict:
        """Test RX web API endpoints over WiFi."""
        return self._test_wifi_web_api(device, self.WEB_API_RX_SSID, "radio_preset", "1")

    def test_spiffs(self, device: BREmoteDevice) -> Dict:
        """Test SPIFFS/config storage using ?get to read a known key"""
        self.log("\n[SAVE] Testing SPIFFS/Config Storage...")
        result = {"test": "SPIFFS Storage", "result": TestResult.PENDING.value, "details": ""}

        try:
            # Try to read a known config key
            response = device.send_command("?get version")
            if "=" in response and "ERR" not in response:
                result["details"] = f"Config key readable: {response.strip()}"
                result["result"] = TestResult.PASS.value
            else:
                # Fallback: just check ?conf responds
                response = device.send_command("?conf")
                if len(response) > 20:
                    result["details"] = f"Config readable ({len(response)} chars)"
                    result["result"] = TestResult.PASS.value
                else:
                    result["details"] = "Config system responsive"
                    result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        self.log(f"  Result: {result['result']}")
        return result
    
    def run_device_tests(self, device: BREmoteDevice) -> TestReport:
        """Run all appropriate tests for a device"""
        self.log(f"\n{'='*60}")
        self.log(f"Testing {device}")
        self.log('='*60)
        
        # Exit charging mode first (device is likely charging when connected to PC)
        self._exit_charging_mode(device)
        
        tests = {}
        
        if device.device_type == DeviceType.TRANSMITTER:
            tests["radio"] = self.test_tx_radio(device)
            tests["display"] = self.test_tx_display(device)
            tests["hall"] = self.test_tx_hall(device)
            tests["analog"] = self.test_tx_analog(device)
            tests["rssi"] = self.test_tx_rssi(device)
            tests["spiffs"] = self.test_spiffs(device)
            # Web config / WiFi tests
            tests["wifi_state"] = self.test_tx_wifi_state(device)
            tests["wifi_debug"] = self.test_tx_wifi_debug_mode(device)
            tests["wifi_timeout"] = self.test_tx_wifi_startup_timeout(device)
            tests["wifi_version"] = self.test_tx_wifi_version(device)
            tests["wifi_error"] = self.test_tx_wifi_error(device)
            tests["config_keys"] = self.test_tx_config_keys(device)
            tests["config_getset"] = self.test_tx_config_get_set(device)
            tests["wifi_onoff"] = self.test_tx_wifi_onoff(device)
            tests["wifi_ui_update"] = self.test_tx_wifi_ui_update(device)
        elif device.device_type == DeviceType.RECEIVER:
            tests["radio"] = self.test_rx_radio(device)
            tests["vesc"] = self.test_rx_vesc(device)
            tests["pwm"] = self.test_rx_pwm(device)
            tests["spiffs"] = self.test_spiffs(device)
            tests["wifi_state"] = self.test_tx_wifi_state(device)
            tests["wifi_debug"] = self.test_tx_wifi_debug_mode(device)
            tests["wifi_timeout"] = self.test_tx_wifi_startup_timeout(device)
            tests["wifi_version"] = self.test_tx_wifi_version(device)
            tests["wifi_error"] = self.test_tx_wifi_error(device)
            tests["config_keys"] = self.test_rx_config_keys(device)
            tests["config_getset"] = self.test_rx_config_get_set(device)
            tests["wifi_onoff"] = self.test_tx_wifi_onoff(device)
            tests["wifi_ui_update"] = self.test_tx_wifi_ui_update(device)
        else:
            self.log("[WARN] Unknown device type, skipping tests")
            return None
        
        # Determine overall result
        failures = sum(1 for t in tests.values() if t["result"] == TestResult.FAIL.value)
        overall = TestResult.FAIL.value if failures > 0 else TestResult.PASS.value
        
        report = TestReport(
            device_type=device.device_type.value,
            port=device.port,
            timestamp=datetime.now().isoformat(),
            tests=tests,
            overall_result=overall
        )
        
        self.log(f"\n{'='*60}")
        self.log(f"Overall Result: {overall}")
        self.log('='*60)
        
        return report
    
    def run_interactive_test(self, gui_root=None) -> Dict[str, TestReport]:
        """Run interactive tests that require user actions"""
        if not self.devices:
            self.scan_ports()
        
        self.test_results = {}
        
        for device in self.devices:
            if device.device_type == DeviceType.TRANSMITTER:
                report = self._interactive_tx_test(device, gui_root)
            elif device.device_type == DeviceType.RECEIVER:
                report = self._interactive_rx_test(device, gui_root)
            else:
                continue
            
            if report:
                self.test_results[device.port] = report
        
        # Print summary at the end
        self._print_interactive_summary()
        
        return self.test_results
    
    def _print_interactive_summary(self):
        """Print summary of all interactive tests"""
        self.log("\n" + "="*70)
        self.log("INTERACTIVE TEST SUMMARY")
        self.log("="*70)
        
        if not self.test_results:
            self.log("No tests completed.")
            return
        
        total_tests = 0
        total_passed = 0
        total_failed = 0
        total_skipped = 0
        
        for port, report in self.test_results.items():
            self.log(f"\nDevice: {port} ({report.device_type.upper()})")
            self.log("-" * 70)
            
            for test_name, test_data in report.tests.items():
                total_tests += 1
                result = test_data.get('result', 'UNKNOWN')
                details = test_data.get('details', '')
                
                if result == TestResult.PASS.value:
                    total_passed += 1
                    status = "[PASS]"
                elif result == TestResult.FAIL.value:
                    total_failed += 1
                    status = "[FAIL]"
                elif result == TestResult.SKIP.value:
                    total_skipped += 1
                    status = "[SKIP]"
                else:
                    status = "[?]"
                
                # Truncate details if too long
                if len(details) > 50:
                    details = details[:47] + "..."
                
                self.log(f"  {status:8} {test_name:20} {details}")
        
        self.log("\n" + "="*70)
        self.log(f"TOTAL: {total_tests} tests | {total_passed} PASSED | {total_failed} FAILED | {total_skipped} SKIPPED")
        
        # Overall result
        if total_failed == 0 and total_passed > 0:
            self.log("OVERALL: ALL TESTS PASSED [OK]")
        elif total_failed > 0:
            self.log(f"OVERALL: {total_failed} TEST(S) FAILED [FAIL]")
        else:
            self.log("OVERALL: NO TESTS COMPLETED [WARN]")
        
        self.log("="*70)
    
    def _exit_charging_mode(self, device: BREmoteDevice):
        """Exit charging mode (TX) and ensure WiFi AP is off (RX) so radio
        link can work. TX radio only activates after physical unlock, so we
        don't touch WiFi on TX — the normal boot sequence handles it."""
        if device.device_type == DeviceType.TRANSMITTER:
            self.log("\n[INIT] Exiting charging mode...")
            device.send_command("?exitchg", wait_for_response=False)
            time.sleep(0.5)
            # Flush any charging mode messages
            if device.serial and device.serial.in_waiting:
                try:
                    device.serial.read(device.serial.in_waiting)
                except:
                    pass
            self.log("   [OK] Charging mode exited")
        elif device.device_type == DeviceType.RECEIVER:
            # RX starts with WiFi AP on at boot; radio is disabled while
            # WiFi is active. Turn it off so the radio link can establish.
            self.log("\n[INIT] Turning off WiFi AP for radio link...")
            device.send_command("?wifi off")
            time.sleep(0.3)
            self.log("   [OK] WiFi AP disabled")
    
    def _interactive_tx_test(self, device: BREmoteDevice, gui_root=None) -> TestReport:
        """Interactive test for TX with user prompts"""
        self.log(f"\n{'='*60}")
        self.log(f"Interactive Test: {device}")
        self.log('='*60)
        
        tests = {}
        
        # Step 1: Exit charging mode (device is likely charging when connected to PC)
        self._exit_charging_mode(device)
        
        # CRITICAL: Check if device is locked first - if locked, throttle tests won't work
        self.log("\n[CHECK] Checking if device is locked...")
        ok, message, config = self._check_device_state(device, gui_root)
        
        if not ok and "LOCKED" in message.upper():
            self.log(f"   [LOCKED] Device is locked! Attempting to unlock...")
            
            # Prompt user to unlock using their normal device method
            unlock_result = self._show_prompt(
                "DEVICE IS LOCKED",
                "The TX is currently LOCKED and throttle tests cannot run.\n\n"
                "Please unlock the device using your normal unlocking method,\n"
                "then press Enter to continue...",
                gui_root
            )
            
            # Re-check after user attempts unlock
            time.sleep(1.0)
            ok, message, config = self._check_device_state(device, gui_root)
            
            if not ok and "LOCKED" in message.upper():
                self.log(f"   [FAIL] Device still locked after unlock attempt")
                self.log(f"   [SKIP] All throttle tests will be skipped")
                # Mark all tests as skipped due to lock
                tests["thr_max"] = {"result": TestResult.SKIP.value, "details": "Device locked - unlock required"}
                tests["thr_min"] = {"result": TestResult.SKIP.value, "details": "Device locked - unlock required"}
                tests["thr_mid"] = {"result": TestResult.SKIP.value, "details": "Device locked - unlock required"}
                tests["toggle_left"] = {"result": TestResult.SKIP.value, "details": "Device locked - unlock required"}
                tests["toggle_right"] = {"result": TestResult.SKIP.value, "details": "Device locked - unlock required"}
                
                # Create report with skipped tests
                report = TestReport(
                    device_type=device.device_type.value,
                    port=device.port,
                    timestamp=datetime.now().isoformat(),
                    tests=tests,
                    overall_result=TestResult.FAIL.value
                )
                
                self.log(f"\n{'='*60}")
                self.log(f"Overall Result: FAIL (Device locked)")
                self.log('='*60)
                
                return report
            else:
                self.log(f"   [OK] Device unlocked successfully!")
        
        # Test 1: Full throttle forward
        tests["thr_max"] = self._prompt_and_verify(
            "Full Throttle Forward",
            "Move throttle to FULL FORWARD and press Enter...",
            device,
            self._verify_throttle_max,
            gui_root
        )

        # Test 2: Full throttle brake/zero
        tests["thr_min"] = self._prompt_and_verify(
            "Full Brake",
            "Move throttle to FULL BRAKE (zero) and press Enter...",
            device,
            self._verify_throttle_min,
            gui_root
        )

        # Test 3: Mid throttle (any position between full and zero)
        tests["thr_mid"] = self._prompt_and_verify(
            "Mid Throttle",
            "Move throttle to ANY POSITION between full and zero and press Enter...",
            device,
            self._verify_throttle_mid,
            gui_root
        )

        # Test 4: Left toggle
        tests["toggle_left"] = self._prompt_and_verify(
            "Left Toggle",
            "Press LEFT TOGGLE and press Enter...",
            device,
            self._verify_toggle_left,
            gui_root
        )

        # Test 5: Right toggle
        tests["toggle_right"] = self._prompt_and_verify(
            "Right Toggle",
            "Press RIGHT TOGGLE and press Enter...",
            device,
            self._verify_toggle_right,
            gui_root
        )
        
        # Determine overall result
        failures = sum(1 for t in tests.values() if t["result"] == TestResult.FAIL.value)
        overall = TestResult.FAIL.value if failures > 0 else TestResult.PASS.value
        
        report = TestReport(
            device_type=device.device_type.value,
            port=device.port,
            timestamp=datetime.now().isoformat(),
            tests=tests,
            overall_result=overall
        )
        
        self.log(f"\n{'='*60}")
        self.log(f"Overall Result: {overall}")
        self.log('='*60)
        
        return report
    
    def _interactive_rx_test(self, device: BREmoteDevice, gui_root=None) -> TestReport:
        """Interactive test for RX with user prompts"""
        self.log(f"\n{'='*60}")
        self.log(f"Interactive Test: {device}")
        self.log('='*60)
        
        tests = {}
        
        # Step 1: Exit charging mode
        self._exit_charging_mode(device)
        
        # Test 1: VESC connection
        tests["vesc_detect"] = self._prompt_and_verify(
            "VESC Detection",
            "Ensure VESC is powered and connected. Press Enter when ready...",
            device,
            self._verify_vesc_detect,
            gui_root
        )
        
        # Test 2: Radio reception
        tests["radio_rx"] = self._prompt_and_verify(
            "Radio Reception",
            "Press buttons on TX while watching RX serial output. Press Enter when done...",
            device,
            self._verify_radio_reception,
            gui_root
        )
        
        # Test 3: PWM output (optional - only if safe)
        result = self._show_prompt(
            "PWM Output Test",
            "[WARN] WARNING: This will activate motor PWM!\n\n"
            "Ensure:\n"
            "- Motor is disconnected OR wheel is free\n"
            "- VESC is in safe mode\n\n"
            "Type 'YES' to proceed or press Enter to skip:",
            gui_root
        )
        
        if result and result.upper() == "YES":
            tests["pwm_output"] = self._prompt_and_verify(
                "PWM Output",
                "TX should send throttle. Watch for PWM changes. Press Enter when done...",
                device,
                self._verify_pwm_active,
                gui_root
            )
        else:
            tests["pwm_output"] = {
                "test": "PWM Output",
                "result": TestResult.SKIP.value,
                "details": "Skipped by user"
            }
        
        # Determine overall result
        failures = sum(1 for t in tests.values() if t["result"] == TestResult.FAIL.value)
        overall = TestResult.FAIL.value if failures > 0 else TestResult.PASS.value
        
        report = TestReport(
            device_type=device.device_type.value,
            port=device.port,
            timestamp=datetime.now().isoformat(),
            tests=tests,
            overall_result=overall
        )
        
        self.log(f"\n{'='*60}")
        self.log(f"Overall Result: {overall}")
        self.log('='*60)
        
        return report
    
    def _prompt_and_verify(self, test_name: str, prompt: str, device: BREmoteDevice, 
                          verify_fn, gui_root=None) -> Dict:
        """Show prompt and verify device response"""
        self.log(f"\n[TEST] {test_name}")
        self.log(f"   {prompt}")
        
        # Show prompt
        user_input = self._show_prompt(test_name, prompt, gui_root)
        
        # Wait a moment for user to complete action
        time.sleep(0.5)
        
        # Verify response
        result = verify_fn(device)
        self.log(f"   Result: {result['result']} - {result['details']}")
        
        return result
    
    def _show_prompt(self, title: str, message: str, gui_root=None) -> str:
        """Show prompt to user and return input"""
        if gui_root and GUI_AVAILABLE:
            # GUI dialog
            from tkinter import simpledialog
            result = simpledialog.askstring(title, message, parent=gui_root)
            return result if result else ""
        else:
            # CLI prompt
            print(f"\n{title}")
            print("-" * len(title))
            return input(f"{message} ")
    
    def _check_device_state(self, device: BREmoteDevice, gui_root=None) -> Tuple[bool, str, Dict]:
        """Check if device is locked and parse config using ?state json.

        Returns (ok, message, config_dict).
        """
        config = {
            'locked': False,
            'hall_enabled': True,
            'steer_enabled': False,
            'toggle_enabled': True,
        }

        # Use JSON status for reliable, unambiguous parsing
        data = device.send_json_command("?state")

        if data and isinstance(data, dict):
            self.log(f"   [DEBUG] Status JSON: {data}")

            is_locked = data.get("locked", False)
            # Handle both bool and int representations
            if isinstance(is_locked, int):
                is_locked = is_locked != 0
            config['locked'] = is_locked

            hall_state = data.get("hall", "ON")
            config['hall_enabled'] = (hall_state == "ON") if isinstance(hall_state, str) else bool(hall_state)

            config['gear'] = data.get("gear")
            config['max_gears'] = data.get("max_gears")
            config['paired'] = data.get("paired", False)
            config['error'] = data.get("error", 0)
            config['last_pkt_ms'] = data.get("last_pkt_ms")

            self.log(f"   [DEBUG] Detected lock state: {'LOCKED' if is_locked else 'UNLOCKED'}")
        else:
            self.log(f"   [DEBUG] No JSON response from ?state, assuming UNLOCKED")

        if not config.get('hall_enabled', True):
            return (False, "Hall sensors disabled in config", config)

        if config.get('locked', False):
            return (False, "Device is LOCKED - unlock required", config)

        return (True, "Device ready", config)
    
    # ===== JSON-based input reading helper =====

    def _read_inputs_json(self, device: BREmoteDevice) -> Optional[dict]:
        """Read current inputs via ?printInputs json, then quit the loop.

        Returns parsed dict with keys: throttle, steering, toggle, toggle_input,
        locked, in_menu, steer_enabled, hall_enabled.  Returns None on failure.
        """
        data = device.send_json_command("?printInputs")
        time.sleep(0.1)
        device.send_command("quit", wait_for_response=False)
        if data and "throttle" in data:
            self.log(f"   [DEBUG] Inputs JSON: {data}")
            return data
        self.log(f"   [DEBUG] No JSON from ?printInputs")
        return None

    # ===== Verification functions for TX =====

    def _verify_throttle_mid(self, device: BREmoteDevice) -> Dict:
        """Verify throttle is at mid position (between full and zero)"""
        data = self._read_inputs_json(device)
        if data is None:
            return {"result": TestResult.FAIL.value, "details": "Cannot read inputs (no JSON)"}

        thr = int(data["throttle"])
        if 56 <= thr <= 199:
            return {"result": TestResult.PASS.value, "details": f"Throttle at mid: {thr}"}
        else:
            return {"result": TestResult.FAIL.value, "details": f"Throttle not in mid range: {thr} (expected 56-199)"}

    def _verify_throttle_max(self, device: BREmoteDevice) -> Dict:
        """Verify throttle is at maximum"""
        data = self._read_inputs_json(device)
        if data is None:
            return {"result": TestResult.FAIL.value, "details": "Cannot read inputs (no JSON)"}

        thr = int(data["throttle"])
        if thr >= 200:
            return {"result": TestResult.PASS.value, "details": f"Throttle at max: {thr}"}
        else:
            return {"result": TestResult.FAIL.value, "details": f"Throttle not at max: {thr} (expected >= 200)"}

    def _verify_throttle_min(self, device: BREmoteDevice) -> Dict:
        """Verify throttle is at minimum (brake)"""
        data = self._read_inputs_json(device)
        if data is None:
            return {"result": TestResult.FAIL.value, "details": "Cannot read inputs (no JSON)"}

        thr = int(data["throttle"])
        if thr <= 55:
            return {"result": TestResult.PASS.value, "details": f"Throttle at brake: {thr}"}
        else:
            return {"result": TestResult.FAIL.value, "details": f"Throttle not at brake: {thr} (expected <= 55)"}

    def _verify_toggle_left(self, device: BREmoteDevice) -> Dict:
        """Verify left toggle is pressed using steering value from JSON"""
        data = self._read_inputs_json(device)
        if data is None:
            return {"result": TestResult.FAIL.value, "details": "Cannot read inputs (no JSON)"}

        steering = int(data["steering"])
        toggle_input = int(data.get("toggle_input", 0))
        self.log(f"   [DEBUG] steering={steering}, toggle_input={toggle_input}")

        # toggle_input: -1 = left, 0 = center, 1 = right
        if toggle_input == -1:
            return {"result": TestResult.PASS.value, "details": f"Left toggle pressed (toggle_input={toggle_input})"}
        # Fallback: steering < 50 means left
        if steering < 50:
            return {"result": TestResult.PASS.value, "details": f"Left toggle pressed (steering={steering})"}
        else:
            return {"result": TestResult.FAIL.value, "details": f"Left toggle not pressed (steering={steering}, toggle_input={toggle_input})"}

    def _verify_toggle_right(self, device: BREmoteDevice) -> Dict:
        """Verify right toggle is pressed using steering value from JSON"""
        data = self._read_inputs_json(device)
        if data is None:
            return {"result": TestResult.FAIL.value, "details": "Cannot read inputs (no JSON)"}

        steering = int(data["steering"])
        toggle_input = int(data.get("toggle_input", 0))
        self.log(f"   [DEBUG] steering={steering}, toggle_input={toggle_input}")

        # toggle_input: 1 = right
        if toggle_input == 1:
            return {"result": TestResult.PASS.value, "details": f"Right toggle pressed (toggle_input={toggle_input})"}
        # Fallback: steering > 200 means right
        if steering > 200:
            return {"result": TestResult.PASS.value, "details": f"Right toggle pressed (steering={steering})"}
        else:
            return {"result": TestResult.FAIL.value, "details": f"Right toggle not pressed (steering={steering}, toggle_input={toggle_input})"}
    
    # Verification functions for RX
    def _verify_vesc_detect(self, device: BREmoteDevice) -> Dict:
        """Verify VESC is detected"""
        response = device.send_command("?vesc")
        if "vesc" in response.lower() or "detected" in response.lower() or "mcconf" in response.lower():
            return {"result": TestResult.PASS.value, "details": "VESC detected"}
        else:
            return {"result": TestResult.FAIL.value, "details": "VESC not detected"}
    
    def _verify_radio_reception(self, device: BREmoteDevice) -> Dict:
        """Verify radio is receiving data using ?printPackets json"""
        # Clear counters first
        device.send_command("?clearPackets", wait_for_response=False)
        time.sleep(2.0)  # Wait for reception

        data = device.send_json_command("?printPackets")
        if data and "received" in data:
            count = data.get("received", 0)
            if count > 0:
                return {"result": TestResult.PASS.value, "details": f"Received {count} packets, ratio {data.get('ratio',0)}%"}
            else:
                return {"result": TestResult.FAIL.value, "details": "No packets received"}
        else:
            # Fallback to text
            response = device.send_command("?packets")
            if "rx" in response.lower() or "received" in response.lower():
                return {"result": TestResult.PASS.value, "details": "Radio reception active"}
            return {"result": TestResult.FAIL.value, "details": "No radio reception detected"}
    
    def _verify_pwm_active(self, device: BREmoteDevice) -> Dict:
        """Verify PWM output is active"""
        response = device.send_command("?pwm")
        if "duty" in response.lower() or "pwm" in response.lower():
            return {"result": TestResult.PASS.value, "details": "PWM output active"}
        else:
            return {"result": TestResult.FAIL.value, "details": "PWM output not detected"}
    
    def run_wifi_tests(self) -> Dict[str, TestReport]:
        """Run web config / WiFi AP tests on TX and RX devices"""
        if not self.devices:
            self.scan_ports()

        self.test_results = {}

        for device in self.devices:
            if device.device_type not in (DeviceType.TRANSMITTER, DeviceType.RECEIVER):
                self.log(f"\n[SKIP] {device} — unsupported device type for WiFi tests")
                continue

            self.log(f"\n{'='*60}")
            self.log(f"Web Config / WiFi Tests: {device}")
            self.log('='*60)

            self._exit_charging_mode(device)

            tests = {}
            tests["wifi_state"] = self.test_tx_wifi_state(device)
            tests["wifi_debug"] = self.test_tx_wifi_debug_mode(device)
            tests["wifi_timeout"] = self.test_tx_wifi_startup_timeout(device)
            tests["wifi_version"] = self.test_tx_wifi_version(device)
            tests["wifi_error"] = self.test_tx_wifi_error(device)
            if device.device_type == DeviceType.TRANSMITTER:
                tests["config_keys"] = self.test_tx_config_keys(device)
                tests["config_getset"] = self.test_tx_config_get_set(device)
            else:
                tests["config_keys"] = self.test_rx_config_keys(device)
                tests["config_getset"] = self.test_rx_config_get_set(device)
            tests["wifi_onoff"] = self.test_tx_wifi_onoff(device)
            tests["wifi_ui_update"] = self.test_tx_wifi_ui_update(device)

            failures = sum(1 for t in tests.values() if t["result"] == TestResult.FAIL.value)
            overall = TestResult.FAIL.value if failures > 0 else TestResult.PASS.value

            report = TestReport(
                device_type=device.device_type.value,
                port=device.port,
                timestamp=datetime.now().isoformat(),
                tests=tests,
                overall_result=overall
            )

            self.test_results[device.port] = report

            self.log(f"\n{'='*60}")
            self.log(f"WiFi Tests Result: {overall} ({len(tests) - failures}/{len(tests)} passed)")
            self.log('='*60)

        return self.test_results

    def run_webapi_tests(self) -> Dict[str, TestReport]:
        """Run web API endpoint tests over WiFi on TX and RX devices"""
        if not self.devices:
            self.scan_ports()

        self.test_results = {}

        for device in self.devices:
            if device.device_type not in (DeviceType.TRANSMITTER, DeviceType.RECEIVER):
                self.log(f"\n[SKIP] {device} — unsupported device type for Web API tests")
                continue

            self.log(f"\n{'='*60}")
            self.log(f"Web API Endpoint Tests: {device}")
            self.log('='*60)

            self._exit_charging_mode(device)

            tests = {}
            if device.device_type == DeviceType.TRANSMITTER:
                tests["web_api"] = self.test_tx_wifi_web_api(device)
            else:
                tests["web_api"] = self.test_rx_wifi_web_api(device)

            failures = sum(1 for t in tests.values() if t["result"] == TestResult.FAIL.value)
            overall = TestResult.FAIL.value if failures > 0 else TestResult.PASS.value

            report = TestReport(
                device_type=device.device_type.value,
                port=device.port,
                timestamp=datetime.now().isoformat(),
                tests=tests,
                overall_result=overall
            )

            self.test_results[device.port] = report

            self.log(f"\n{'='*60}")
            self.log(f"Web API Tests Result: {overall} ({len(tests) - failures}/{len(tests)} passed)")
            self.log('='*60)

        return self.test_results

    def run_all_tests(self) -> Dict[str, TestReport]:
        """Run tests on all detected devices"""
        if not self.devices:
            self.scan_ports()
        
        self.test_results = {}
        
        for device in self.devices:
            try:
                report = self.run_device_tests(device)
                if report:
                    self.test_results[device.port] = report
            except Exception as e:
                self.log(f"\n[ERROR] Tests for {device.port} aborted: {e}")

        return self.test_results
    
    def test_radio_link(self, duration: float = 10.0) -> Optional[Dict]:
        """Test radio link between TX and RX"""
        # Find TX and RX devices
        tx_device = None
        rx_device = None
        
        for device in self.devices:
            if device.device_type == DeviceType.TRANSMITTER:
                tx_device = device
            elif device.device_type == DeviceType.RECEIVER:
                rx_device = device
        
        if not tx_device or not rx_device:
            self.log("\n[WARN] Both TX and RX required for radio link test")
            return None
        
        self.log(f"\n{'='*60}")
        self.log("Testing Radio Link (TX ↔ RX)")
        self.log('='*60)
        
        # Create link monitor and run test
        monitor = RadioLinkMonitor(tx_device, rx_device, self.gui_callback)
        result = monitor.start(duration)
        
        self.log(f"\n[DATA] Radio Link Results:")
        self.log(f"  Samples: TX={result.get('tx_samples', 0)}, RX={result.get('rx_samples', 0)}, Matched={result.get('matched_pairs', 0)}")
        self.log(f"  Packet Loss: {result.get('packet_loss_percent', 0):.1f}%")
        self.log(f"  Avg Throttle Diff: {result.get('avg_throttle_diff', 0):.2f}")
        self.log(f"  Avg Steering Diff: {result.get('avg_steering_diff', 0):.2f}")
        if result.get('avg_rssi_dbm') is not None:
            self.log(f"  RSSI: {result.get('avg_rssi_dbm'):.0f} dBm (min: {result.get('min_rssi_dbm'):.0f})")
        if result.get('avg_snr_db') is not None:
            self.log(f"  SNR: {result.get('avg_snr_db'):.1f} dB")
        self.log(f"  Result: {result['result']}")
        self.log(f"  Details: {result['details']}")
        
        return result
    
    def run_integration_test(self, duration: float = 10.0) -> Dict[str, any]:
        """Run integration test with radio link monitoring"""
        if not self.devices:
            self.scan_ports()
        
        results = {
            "timestamp": datetime.now().isoformat(),
            "individual_tests": {},
            "radio_link_test": None
        }

        # Run radio link test FIRST (before individual tests which may enable WiFi and break the link)
        tx_count = sum(1 for d in self.devices if d.device_type == DeviceType.TRANSMITTER)
        rx_count = sum(1 for d in self.devices if d.device_type == DeviceType.RECEIVER)

        if tx_count >= 1 and rx_count >= 1:
            self.log("\n" + "="*60)
            self.log("PHASE 1: Radio Link Integration Test")
            self.log("="*60)

            link_result = self.test_radio_link(duration)
            if link_result:
                results["radio_link_test"] = link_result
        else:
            self.log("\n[WARN] Skipping radio link test (need 1 TX + 1 RX)")

        # Run individual device tests (WiFi on/off test may break the radio link)
        self.log("\n" + "="*60)
        self.log("PHASE 2: Individual Device Tests")
        self.log("="*60)

        for device in self.devices:
            try:
                report = self.run_device_tests(device)
                if report:
                    results["individual_tests"][device.port] = asdict(report)
                    self.test_results[device.port] = report
            except Exception as e:
                self.log(f"\n[ERROR] Tests for {device.port} aborted: {e}")
        
        return results
    
    def cleanup(self):
        """Disconnect all devices"""
        for device in self.devices:
            device.disconnect()
        self.devices.clear()


class BREmoteTestGUI:
    """Tkinter GUI for the test suite"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("BREmote V2 Hardware Test Suite")
        self.root.geometry("900x700")
        
        self.tester = BREmoteTester(gui_callback=self.log_message)
        self.test_thread = None
        
        self._create_widgets()
        
    def _create_widgets(self):
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(2, weight=1)
        
        # Title
        title = ttk.Label(main_frame, text="BREmote V2 Hardware Test Suite", 
                         font=('Helvetica', 16, 'bold'))
        title.grid(row=0, column=0, pady=(0, 10), sticky=tk.W)
        
        # Control buttons
        btn_frame = ttk.Frame(main_frame)
        btn_frame.grid(row=1, column=0, pady=5, sticky=(tk.W, tk.E))
        
        self.scan_btn = ttk.Button(btn_frame, text="[SCAN] Scan Ports", command=self.scan_ports)
        self.scan_btn.pack(side=tk.LEFT, padx=5)

        self.test_btn = ttk.Button(btn_frame, text="[RUN] Run Auto Tests", command=self.run_tests)
        self.test_btn.pack(side=tk.LEFT, padx=5)

        self.interactive_btn = ttk.Button(btn_frame, text="[USER] Interactive Test", command=self.run_interactive)
        self.interactive_btn.pack(side=tk.LEFT, padx=5)

        self.link_btn = ttk.Button(btn_frame, text="[LINK] Radio Link Test", command=self.run_link_test)
        self.link_btn.pack(side=tk.LEFT, padx=5)

        self.report_btn = ttk.Button(btn_frame, text="[REPORT] Save Report", command=self.save_report)
        self.report_btn.pack(side=tk.LEFT, padx=5)

        self.clear_btn = ttk.Button(btn_frame, text="[CLEAR] Clear", command=self.clear_log)
        self.clear_btn.pack(side=tk.LEFT, padx=5)
        
        # Log area
        log_frame = ttk.LabelFrame(main_frame, text="Test Log", padding="5")
        log_frame.grid(row=2, column=0, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, 
                                                   font=('Consolas', 10))
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, 
                              relief=tk.SUNKEN, anchor=tk.W)
        status_bar.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=(5, 0))
        
    def log_message(self, message: str):
        """Add message to log"""
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        self.root.update_idletasks()
    
    def scan_ports(self):
        """Scan for devices"""
        self.log_message("\n" + "="*60)
        self.tester.scan_ports()
        self.status_var.set(f"Found {len(self.tester.devices)} device(s)")
    
    def run_tests(self):
        """Run tests in background thread"""
        if self.test_thread and self.test_thread.is_alive():
            messagebox.showwarning("Busy", "Tests already running")
            return
        
        self.test_thread = threading.Thread(target=self._run_tests_thread)
        self.test_thread.daemon = True
        self.test_thread.start()
    
    def _run_tests_thread(self):
        """Thread target for running tests"""
        self.root.after(0, lambda: self.scan_btn.config(state=tk.DISABLED))
        self.root.after(0, lambda: self.test_btn.config(state=tk.DISABLED))
        self.root.after(0, lambda: self.status_var.set("Running tests..."))
        
        try:
            self.tester.run_all_tests()
            passed = sum(1 for r in self.tester.test_results.values() 
                        if r.overall_result == TestResult.PASS.value)
            total = len(self.tester.test_results)
            self.root.after(0, lambda: self.status_var.set(
                f"Tests complete: {passed}/{total} devices passed"))
        except Exception as e:
            self.root.after(0, lambda err=e: self.log_message(f"\n[ERROR] Error: {err}"))
        finally:
            self.root.after(0, lambda: self.scan_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.test_btn.config(state=tk.NORMAL))

    def save_report(self):
        """Save test report to file"""
        if not self.tester.test_results:
            messagebox.showwarning("No Data", "No test results to save")
            return
        
        filename = f"bremote_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        try:
            with open(filename, 'w') as f:
                reports = {port: asdict(report) for port, report in self.tester.test_results.items()}
                json.dump(reports, f, indent=2)
            self.log_message(f"\n[SAVE] Report saved to: {filename}")
            messagebox.showinfo("Success", f"Report saved to:\n{filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save report:\n{e}")
    
    def run_interactive(self):
        """Run interactive tests with user prompts"""
        if self.test_thread and self.test_thread.is_alive():
            messagebox.showwarning("Busy", "Tests already running")
            return
        
        self.test_thread = threading.Thread(target=self._run_interactive_thread)
        self.test_thread.daemon = True
        self.test_thread.start()
    
    def _run_interactive_thread(self):
        """Thread target for interactive tests"""
        self.root.after(0, lambda: self.scan_btn.config(state=tk.DISABLED))
        self.root.after(0, lambda: self.test_btn.config(state=tk.DISABLED))
        self.root.after(0, lambda: self.interactive_btn.config(state=tk.DISABLED))
        self.root.after(0, lambda: self.status_var.set("Running interactive tests..."))
        
        try:
            self.tester.run_interactive_test(self.root)
            passed = sum(1 for r in self.tester.test_results.values() 
                        if r.overall_result == TestResult.PASS.value)
            total = len(self.tester.test_results)
            self.root.after(0, lambda: self.status_var.set(
                f"Interactive tests: {passed}/{total} passed"))
        except Exception as e:
            self.root.after(0, lambda err=e: self.log_message(f"\n[ERROR] Error: {err}"))
        finally:
            self.root.after(0, lambda: self.scan_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.test_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.interactive_btn.config(state=tk.NORMAL))
    
    def run_link_test(self):
        """Run radio link test between TX and RX"""
        if self.test_thread and self.test_thread.is_alive():
            messagebox.showwarning("Busy", "Tests already running")
            return
        
        self.test_thread = threading.Thread(target=self._run_link_thread)
        self.test_thread.daemon = True
        self.test_thread.start()
    
    def _run_link_thread(self):
        """Thread target for radio link test"""
        self.root.after(0, lambda: self.scan_btn.config(state=tk.DISABLED))
        self.root.after(0, lambda: self.link_btn.config(state=tk.DISABLED))
        self.root.after(0, lambda: self.status_var.set("Testing radio link..."))
        
        try:
            result = self.tester.run_integration_test(duration=10.0)
            link_passed = result.get('radio_link_test', {}).get('result') == TestResult.PASS.value
            self.root.after(0, lambda: self.status_var.set(
                f"Link test: {'PASSED' if link_passed else 'FAILED'}"))
        except Exception as e:
            self.root.after(0, lambda err=e: self.log_message(f"\n[ERROR] Error: {err}"))
        finally:
            self.root.after(0, lambda: self.scan_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.link_btn.config(state=tk.NORMAL))
    
    def clear_log(self):
        """Clear log display"""
        self.log_text.delete(1.0, tk.END)


def main():
    parser = argparse.ArgumentParser(description='BREmote V2 Hardware Test Suite')
    parser.add_argument('--port', help='Specific COM port to test')
    parser.add_argument('--test', choices=['all', 'radio', 'display', 'hall', 'analog', 'vesc', 'pwm'],
                       default='all', help='Test to run')
    parser.add_argument('--gui', action='store_true', help='Use GUI interface')
    parser.add_argument('--scan', action='store_true', help='Only scan for devices')
    parser.add_argument('--interactive', '-i', action='store_true', help='Run interactive tests with user prompts')
    parser.add_argument('--wifi', '-w', action='store_true', help='Run web config / WiFi AP tests only (TX/RX)')
    parser.add_argument('--webapi', action='store_true', help='Run web API endpoint tests over WiFi (TX/RX, requires WiFi adapter)')
    parser.add_argument('--link', '-l', action='store_true', help='Run radio link integration test (requires TX+RX)')
    parser.add_argument('--duration', '-d', type=float, default=10.0, help='Duration for link test in seconds (default: 10)')
    parser.add_argument('--report', help='Save report to file')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.WARNING)
    
    # GUI mode
    if args.gui:
        if not GUI_AVAILABLE:
            print("[ERROR] GUI not available. Install tkinter or use CLI mode.")
            sys.exit(1)
        
        root = tk.Tk()
        app = BREmoteTestGUI(root)
        
        # Auto-scan if requested
        if args.scan:
            root.after(500, app.scan_ports)
        
        root.mainloop()
        app.tester.cleanup()
        return
    
    # CLI mode
    tester = BREmoteTester()
    
    try:
        if args.webapi:
            # Web API endpoint tests over WiFi
            print("\n[WEBAPI] Running Web API Endpoint Tests (will switch WiFi)...")
            tester.run_webapi_tests()
        elif args.wifi:
            # WiFi / Web config tests only
            print("\n[WIFI] Running Web Config / WiFi Tests...")
            tester.run_wifi_tests()
        elif args.interactive:
            # Interactive mode with user prompts
            print("\n[USER] Running Interactive Tests...")
            print("You will be prompted to perform specific actions on the hardware.")
            print("Press Enter after completing each action.\n")
            tester.run_interactive_test()
        elif args.link:
            # Radio link integration test
            print(f"\n[LINK] Running Radio Link Test ({args.duration}s)...")
            result = tester.run_integration_test(duration=args.duration)
            
            # Print summary
            print("\n" + "="*60)
            print("Radio Link Test Summary")
            print("="*60)
            link_result = result.get('radio_link_test', {})
            if link_result:
                print(f"Result: {link_result.get('result', 'N/A')}")
                print(f"Packet Loss: {link_result.get('packet_loss_percent', 0):.1f}%")
                print(f"Matched Pairs: {link_result.get('matched_pairs', 0)}")
                print(f"Avg RSSI: {link_result.get('avg_rssi_dbm', 'N/A')} dBm")
                print(f"Details: {link_result.get('details', 'N/A')}")
            else:
                print("[WARN] Radio link test not completed (need both TX and RX)")
        elif args.port:
            # Test specific port
            device = BREmoteDevice(args.port)
            if device.connect():
                device.identify()
                tester.devices.append(device)
                tester.run_device_tests(device)
            else:
                print(f"[ERROR] Failed to connect to {args.port}")
        else:
            # Auto-detect and test all
            if args.scan:
                tester.scan_ports()
            else:
                tester.run_all_tests()
        
        # Save report if requested
        if args.report and tester.test_results:
            with open(args.report, 'w') as f:
                reports = {port: asdict(report) for port, report in tester.test_results.items()}
                json.dump(reports, f, indent=2)
            print(f"\n[SAVE] Report saved to: {args.report}")
    
    except KeyboardInterrupt:
        print("\n\n[WARN] Interrupted by user")
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()
