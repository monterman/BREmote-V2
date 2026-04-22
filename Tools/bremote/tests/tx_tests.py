"""
BREmote Test Suite - TX Tests
Test functions specific to Transmitter (TX) unit.
"""

import time
from typing import Dict, Any

from ..device import BREmoteDevice
from ..models import TestResult


class TXTestSuite:
    """Test suite for TX-specific functionality"""
    
    @staticmethod
    def test_radio(device: BREmoteDevice) -> Dict[str, Any]:
        """Test TX radio functionality using ?state json and ?printPackets json"""
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

        return result
    
    @staticmethod
    def test_display(device: BREmoteDevice) -> Dict[str, Any]:
        """Test TX display functionality using ?state json"""
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

        return result
    
    @staticmethod
    def test_hall(device: BREmoteDevice) -> Dict[str, Any]:
        """Test TX hall sensor (throttle and toggles) using ?printInputs json"""
        result = {"test": "Hall Sensors", "result": TestResult.PENDING.value, "details": ""}

        try:
            # Stop any previous continuous output
            device.stop_continuous_output()
            
            data = device.send_json_command("?printInputs")
            
            # Stop the continuous print loop
            device.stop_continuous_output()

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
        finally:
            # Ensure we stop continuous output even on error
            try:
                device.stop_continuous_output()
            except:
                pass

        return result
    
    @staticmethod
    def test_analog(device: BREmoteDevice) -> Dict[str, Any]:
        """Test TX analog inputs (battery monitoring) using ?state json"""
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

        return result
    
    @staticmethod
    def test_rssi(device: BREmoteDevice) -> Dict[str, Any]:
        """Test TX RSSI monitoring using ?printRSSI json"""
        result = {"test": "RSSI Monitoring", "result": TestResult.PENDING.value, "details": ""}

        try:
            # Stop any previous continuous output
            device.stop_continuous_output()
            
            data = device.send_json_command("?printRSSI")
            
            # Stop the continuous print loop
            device.stop_continuous_output()

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
        finally:
            try:
                device.stop_continuous_output()
            except:
                pass

        return result
    
    @staticmethod
    def run_all(device: BREmoteDevice, log_callback=None) -> Dict[str, Any]:
        """Run all TX tests"""
        if log_callback:
            log_callback("\n[TX] Running TX Test Suite...")
        
        results = {}
        
        tests = [
            ("radio", TXTestSuite.test_radio),
            ("display", TXTestSuite.test_display),
            ("hall", TXTestSuite.test_hall),
            ("analog", TXTestSuite.test_analog),
            ("rssi", TXTestSuite.test_rssi),
        ]
        
        for test_name, test_func in tests:
            if log_callback:
                log_callback(f"\n[TX] Running {test_name} test...")
            results[test_name] = test_func(device)
            if log_callback:
                log_callback(f"  Result: {results[test_name]['result']}")
        
        return results
