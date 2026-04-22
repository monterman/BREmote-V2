"""
BREmote Test Suite - Test Runner
Orchestrates test execution across multiple devices.
"""

import time
from typing import List, Dict, Any, Optional
from datetime import datetime

from .models import DeviceType, TestResult, TestReport
from .device import BREmoteDevice
from .tests import (
    TXTestSuite, RXTestSuite, WiFiTestSuite, 
    ConfigTestSuite, RadioLinkMonitor
)


class BREmoteTester:
    """Test orchestrator for BREmote devices"""
    
    def __init__(self):
        self.devices: List[BREmoteDevice] = []
        self.test_results: Dict[str, TestReport] = {}
        
    def log(self, message: str):
        """Log message to console"""
        try:
            print(message)
        except UnicodeEncodeError:
            print(message.encode('utf-8', errors='replace').decode('utf-8'))
    
    def scan_ports(self) -> List[str]:
        """Scan for available COM ports with BREmote devices"""
        self.log("\n[SCAN] Scanning for BREmote devices...")
        ports = BREmoteDevice.scan_ports()
        bremote_ports = []
        
        for port in ports:
            self.log(f"  Checking {port}...")
            device = BREmoteDevice(port)
            if device.connect():
                # Give device time to settle after connect
                time.sleep(0.5)
                
                device_type = device.identify()
                if device_type != DeviceType.UNKNOWN:
                    self.devices.append(device)
                    bremote_ports.append(port)
                    self.log(f"    Found: {device_type.value.upper()}")
                else:
                    device.disconnect()
                    self.log(f"    Unknown device")
            else:
                self.log(f"    Failed to connect")
        
        if not bremote_ports:
            self.log("  No BREmote devices found.")
        else:
            self.log(f"\n[SCAN] Found {len(bremote_ports)} device(s)")
        
        return bremote_ports
    
    def run_device_tests(self, device: BREmoteDevice) -> TestReport:
        """Run all tests for a single device"""
        self.log(f"\n{'='*60}")
        self.log(f"Testing: {device} ({device.device_type.value.upper()})")
        self.log('='*60)
        
        report = TestReport(
            device_type=device.device_type.value,
            port=device.port
        )
        
        if device.device_type == DeviceType.TRANSMITTER:
            report.tests.update(TXTestSuite.run_all(device, self.log))
            report.tests.update(ConfigTestSuite.run_all_tx(device, self.log))
        elif device.device_type == DeviceType.RECEIVER:
            report.tests.update(RXTestSuite.run_all(device, self.log))
            report.tests.update(ConfigTestSuite.run_all_rx(device, self.log))
        
        failures = sum(1 for t in report.tests.values() 
                     if t.get("result") == TestResult.FAIL.value)
        report.overall_result = TestResult.FAIL.value if failures > 0 else TestResult.PASS.value
        
        return report
    
    def run_all_tests(self) -> Dict[str, TestReport]:
        """Auto-detect and test all devices"""
        self.scan_ports()
        
        if not self.devices:
            self.log("\n[ERROR] No devices found. Exiting.")
            return {}
        
        self.test_results = {}
        
        for device in self.devices:
            report = self.run_device_tests(device)
            self.test_results[device.port] = report
        
        self._print_summary()
        return self.test_results
    
    def run_wifi_tests(self) -> Dict[str, TestReport]:
        """Run WiFi / Web config tests"""
        self.scan_ports()
        
        if not self.devices:
            self.log("\n[ERROR] No devices found.")
            return {}
        
        self.test_results = {}
        
        for device in self.devices:
            if device.device_type == DeviceType.TRANSMITTER:
                self.log(f"\n[WIFI] Running TX WiFi tests on {device.port}")
                report = TestReport(device_type="tx", port=device.port)
                report.tests.update(WiFiTestSuite.run_all_tx(device, self.log))
                self.test_results[device.port] = report
            elif device.device_type == DeviceType.RECEIVER:
                self.log(f"\n[WIFI] Running RX WiFi tests on {device.port}")
                report = TestReport(device_type="rx", port=device.port)
                report.tests.update(WiFiTestSuite.run_all_rx(device, self.log))
                self.test_results[device.port] = report
        
        self._print_summary()
        return self.test_results
    
    def run_integration_test(self, duration: float = 10.0) -> Dict[str, Any]:
        """Run TX↔RX radio link integration test"""
        self.scan_ports()
        
        tx_device = None
        rx_device = None
        
        for device in self.devices:
            if device.device_type == DeviceType.TRANSMITTER:
                tx_device = device
            elif device.device_type == DeviceType.RECEIVER:
                rx_device = device
        
        if not tx_device or not rx_device:
            self.log("\n[ERROR] Need both TX and RX for link test")
            return {}
        
        self.log("\n[INIT] Exiting charging mode on TX...")
        tx_device.send_command("?exitchg", wait_for_response=False)
        time.sleep(0.5)
        
        # Turn off WiFi on RX for radio link
        self.log("\n[INIT] Turning off WiFi on RX...")
        rx_device.send_command("?wifi off")
        time.sleep(0.5)
        
        # Check if TX is locked
        self.log("\n[INIT] Checking TX lock state...")
        tx_state = tx_device.send_json_command("?state json", timeout=2.0)
        is_locked = tx_state.get("locked", 1) if tx_state else True
        
        if is_locked:
            # Interactive unlock prompt with retry
            while True:
                self.log("\n" + "="*60)
                self.log("PRE-TEST SETUP REQUIRED")
                self.log("="*60)
                self.log("The TX is currently LOCKED.")
                self.log("Please move the throttle trigger to unlock the TX.")
                self.log("Once unlocked (display shows normal operation), press ENTER to continue...")
                try:
                    input()
                except EOFError:
                    self.log("  (Running in non-interactive mode, continuing...)")
                    break
                self.log("="*60)
                
                # Check again if unlocked
                self.log("\n[INIT] Verifying TX is unlocked...")
                tx_state = tx_device.send_json_command("?state json", timeout=2.0)
                is_locked = tx_state.get("locked", 1) if tx_state else True
                
                if not is_locked:
                    self.log("  TX is now unlocked [OK]")
                    break
                else:
                    self.log("  TX is still locked. Please try again.")
        else:
            self.log("  TX is already unlocked [OK]")
        
        monitor = RadioLinkMonitor(tx_device, rx_device, self.log)
        result = monitor.start(duration)
        
        return {"radio_link_test": result}
    
    def run_interactive(self):
        """Run interactive tests with user prompts"""
        self.scan_ports()
        
        if not self.devices:
            self.log("\n[ERROR] No devices found.")
            return
        
        for device in self.devices:
            if device.device_type == DeviceType.TRANSMITTER:
                self._interactive_tx_test(device)
            elif device.device_type == DeviceType.RECEIVER:
                self._interactive_rx_test(device)
    
    def _interactive_tx_test(self, device: BREmoteDevice):
        """Interactive test for TX with user prompts"""
        self.log(f"\n{'='*60}")
        self.log(f"Interactive Test: TX on {device.port}")
        self.log('='*60)
        
        # Exit charging mode
        self.log("\n[INIT] Exiting charging mode...")
        device.send_command("?exitchg", wait_for_response=False)
        time.sleep(0.5)
        
        # Check lock status
        data = device.send_json_command("?state")
        locked = False
        if data:
            locked = data.get("locked", False) or data.get("lock", False)
        
        if locked:
            self.log("\n[LOCKED] Device is locked!")
            self.log("Please unlock the device using your normal method,")
            self.log("then press Enter to continue...")
            input()
            time.sleep(1)
        
        tests = {}
        
        # Test 1: Full throttle forward
        tests["thr_max"] = self._prompt_test(
            device, "Full Throttle Forward",
            "Move throttle to FULL FORWARD and press Enter..."
        )
        
        # Test 2: Full brake/zero
        tests["thr_min"] = self._prompt_test(
            device, "Full Brake",
            "Move throttle to FULL BRAKE (zero) and press Enter..."
        )
        
        # Test 3: Mid throttle
        tests["thr_mid"] = self._prompt_test(
            device, "Mid Throttle",
            "Move throttle to MID position and press Enter..."
        )
        
        # Test 4: Left toggle
        tests["toggle_left"] = self._prompt_test(
            device, "Left Toggle",
            "Press LEFT TOGGLE and press Enter..."
        )
        
        # Test 5: Right toggle
        tests["toggle_right"] = self._prompt_test(
            device, "Right Toggle", 
            "Press RIGHT TOGGLE and press Enter..."
        )
        
        # Print results
        self.log("\n" + "="*60)
        self.log("INTERACTIVE TEST RESULTS - TX")
        self.log("="*60)
        for name, result in tests.items():
            self.log(f"  {name:20} {result}")
    
    def _interactive_rx_test(self, device: BREmoteDevice):
        """Interactive test for RX"""
        self.log(f"\n{'='*60}")
        self.log(f"Interactive Test: RX on {device.port}")
        self.log('='*60)
        
        self.log("\n[INFO] RX interactive tests:")
        self.log("  - Throttle: Check motor response")
        self.log("  - Toggle: Check auxiliary function")
        self.log("  Use ?printpwm to verify PWM output")
    
    def _prompt_test(self, device: BREmoteDevice, name: str, prompt: str) -> str:
        """Prompt user and verify input change"""
        self.log(f"\n[{name}]")
        self.log(prompt)
        input()
        
        # Stop any previous continuous output
        device.stop_continuous_output()
        
        # Read current inputs
        data = device.send_json_command("?printInputs")
        
        # Stop the continuous print loop
        device.stop_continuous_output()
        
        if data:
            thr = data.get("throttle", "N/A")
            steer = data.get("steering", "N/A")
            return f"PASS (thr={thr}, steer={steer})"
        return "PASS (input detected)"
    
    def _print_summary(self):
        """Print test summary"""
        self.log("\n" + "="*70)
        self.log("TEST SUMMARY")
        self.log("="*70)
        
        total_tests = 0
        total_passed = 0
        total_failed = 0
        
        for port, report in self.test_results.items():
            self.log(f"\nDevice: {port} ({report.device_type.upper()})")
            self.log("-" * 50)
            
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
                else:
                    status = "[?]"
                
                if len(details) > 40:
                    details = details[:37] + "..."
                
                self.log(f"  {status:8} {test_name:25} {details}")
        
        self.log("\n" + "="*70)
        self.log(f"TOTAL: {total_tests} tests | {total_passed} PASSED | {total_failed} FAILED")
        
        if total_failed == 0 and total_passed > 0:
            self.log("OVERALL: ALL TESTS PASSED")
        elif total_failed > 0:
            self.log(f"OVERALL: {total_failed} TEST(S) FAILED")
        
        self.log("="*70)
    
    def cleanup(self):
        """Disconnect all devices"""
        for device in self.devices:
            device.disconnect()
        self.devices = []
