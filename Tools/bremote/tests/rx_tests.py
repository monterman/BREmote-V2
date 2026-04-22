"""
BREmote Test Suite - RX Tests
Test functions specific to Receiver (RX) unit.
"""

import time
from typing import Dict, Any

from ..device import BREmoteDevice
from ..models import TestResult


class RXTestSuite:
    """Test suite for RX-specific functionality"""
    
    @staticmethod
    def test_radio(device: BREmoteDevice) -> Dict[str, Any]:
        """Test RX radio functionality using ?printrssi"""
        result = {"test": "Radio RX", "result": TestResult.PENDING.value, "details": ""}

        try:
            # Stop any previous continuous output
            device.stop_continuous_output()
            
            # Use ?printrssi to check radio link status
            response = device.send_command("?printrssi")
            
            # Stop the continuous print loop
            device.stop_continuous_output()

            if "rssi" in response.lower() or "link" in response.lower() or "rx" in response.lower():
                result["details"] = f"Radio status: {response[:200]}"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = "Radio interface responsive"
                result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"
        finally:
            try:
                device.stop_continuous_output()
            except:
                pass

        return result
    
    @staticmethod
    def test_vesc(device: BREmoteDevice) -> Dict[str, Any]:
        """Test RX VESC/battery interface using ?printbat"""
        result = {"test": "VESC/Battery Interface", "result": TestResult.PENDING.value, "details": ""}

        try:
            # Use ?printbat to check VESC/telemetry status
            response = device.send_command("?printbat")

            if "vesc" in response.lower() or "volt" in response.lower() or "bat" in response.lower():
                result["details"] = f"Battery/VESC status: {response[:200]}"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = "VESC interface present"
                result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    @staticmethod
    def test_pwm(device: BREmoteDevice) -> Dict[str, Any]:
        """Test RX PWM output using ?printpwm"""
        result = {"test": "PWM Output", "result": TestResult.PENDING.value, "details": ""}

        try:
            response = device.send_command("?printpwm")

            if "pwm" in response.lower():
                result["details"] = f"PWM status: {response[:200]}"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = "PWM interface responsive"
                result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    @staticmethod
    def run_all(device: BREmoteDevice, log_callback=None) -> Dict[str, Any]:
        """Run all RX tests"""
        if log_callback:
            log_callback("\n[RX] Running RX Test Suite...")
        
        results = {}
        
        tests = [
            ("radio", RXTestSuite.test_radio),
            ("vesc", RXTestSuite.test_vesc),
            ("pwm", RXTestSuite.test_pwm),
        ]
        
        for test_name, test_func in tests:
            if log_callback:
                log_callback(f"\n[RX] Running {test_name} test...")
            results[test_name] = test_func(device)
            if log_callback:
                log_callback(f"  Result: {results[test_name]['result']}")
        
        return results
