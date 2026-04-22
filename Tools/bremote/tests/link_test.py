"""
BREmote Test Suite - Radio Link Test
Monitors and correlates TX output with RX input over radio link.
"""

import time
import json
import threading
from typing import Optional, Dict, Any, List
from dataclasses import dataclass

from ..device import BREmoteDevice
from ..models import TestResult


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
                 gui_callback: Optional[callable] = None):
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
        """Log message - callback handles printing"""
        if self.gui_callback:
            self.gui_callback(message)
        else:
            print(message)
    
    def start(self, duration: float = 10.0) -> Dict[str, Any]:
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
        self.tx_device.stop_continuous_output()
        self.rx_device.stop_continuous_output()
        
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
                    # Process all complete JSON objects
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
                    # Process all complete JSON objects
                    self._parse_rx_buffer()
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running:
                    self.log(f"  RX Monitor Error: {e}")
    
    def _parse_tx_buffer(self):
        """Parse TX JSON output for throttle/steering values.

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
        """Parse RX JSON output for received throttle/steering/RSSI values"""
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
                sample.rssi = int(float(data["rssi"]))
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
    
    def _analyze_results(self) -> Dict[str, Any]:
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
        
        # Count matched pairs and TX samples
        matched = [s for s in samples if s.tx_throttle is not None and s.rx_throttle is not None]
        total_rx = len([s for s in samples if s.rx_throttle is not None])
        total_tx = len([s for s in samples if s.tx_throttle is not None])
        matched_count = len(matched)
        
        # Calculate packet loss from captured TX samples.
        packet_loss = ((total_tx - matched_count) / total_tx * 100) if total_tx > 0 else 100.0
        
        # Compare aggregate TX vs RX means. Per-pair comparisons are noisy
        # because TX/RX serial streams are asynchronous.
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
        
        # SNR statistics
        snr_values = [s.snr for s in matched if s.snr is not None]
        avg_snr = sum(snr_values) / len(snr_values) if snr_values else None
        
        # Determine pass/fail
        passed = True
        reasons = []
        
        if packet_loss > 20:
            passed = False
            reasons.append(f"High packet loss: {packet_loss:.1f}%")
        
        if avg_throttle_diff > 5 or avg_steering_diff > 5:
            passed = False
            reasons.append(f"Value mismatch: thr_diff={avg_throttle_diff:.1f}, steer_diff={avg_steering_diff:.1f}")
        
        if avg_rssi is not None and avg_rssi < -100:
            passed = False
            reasons.append(f"Weak signal: RSSI {avg_rssi} dBm")
        
        if matched_count < 10:
            passed = False
            reasons.append(f"Insufficient samples: {matched_count} pairs")
        
        # Debug info
        debug_info = f"samples={len(samples)}, matched={matched_count}, tx={total_tx}"
        
        result = {
            "test": "Radio Link Integration",
            "result": TestResult.PASS.value if passed else TestResult.FAIL.value,
            "details": "; ".join(reasons) if reasons else f"Radio link working correctly ({debug_info})",
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
            "avg_snr_db": avg_snr
        }
        
        return result
