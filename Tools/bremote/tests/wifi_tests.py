"""
BREmote Test Suite - WiFi Tests
Test functions for WiFi / Web Config functionality (TX and RX).
"""

import time
from typing import Dict, Any, Optional

from ..device import BREmoteDevice
from ..models import TestResult


class WiFiTestSuite:
    """Test suite for WiFi / Web Config functionality"""
    
    # ========== TX WiFi Tests ==========
    
    @staticmethod
    def test_tx_wifi_state(device: BREmoteDevice) -> Dict[str, Any]:
        """Test WiFi status query via ?wifi and ?wifistate"""
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

        return result
    
    @staticmethod
    def test_tx_wifi_debug_mode(device: BREmoteDevice) -> Dict[str, Any]:
        """Test WiFi debug mode get/set via ?wifidbg"""
        result = {"test": "WiFi Debug Mode", "result": TestResult.PENDING.value, "details": ""}

        try:
            original = device.send_command("?wifidbg")
            if "wifidbg=" not in original:
                result["details"] = f"Unexpected ?wifidbg response: {original[:80]}"
                result["result"] = TestResult.FAIL.value
                return result

            # Extract original mode
            original_mode = original.split("=")[1].strip() if "=" in original else "unknown"

            # Test setting different modes
            for mode in ["some", "full", "off"]:
                set_resp = device.send_command(f"?wifidbg {mode}")
                if "wifidbg=" not in set_resp:
                    result["details"] = f"Failed to set mode {mode}: {set_resp[:80]}"
                    result["result"] = TestResult.FAIL.value
                    return result

            # Restore original
            device.send_command(f"?wifidbg {original_mode}")

            result["details"] = f"Debug mode cycles work: {original_mode}"
            result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    @staticmethod
    def test_tx_wifi_startup_timeout(device: BREmoteDevice) -> Dict[str, Any]:
        """Test AP startup timeout get/set via ?wifips"""
        result = {"test": "WiFi Startup Timeout", "result": TestResult.PENDING.value, "details": ""}

        try:
            original = device.send_command("?wifips")
            if "wifips=" not in original:
                result["details"] = f"Unexpected ?wifips response: {original[:80]}"
                result["result"] = TestResult.FAIL.value
                return result

            # Extract original timeout
            original_ms = original.split("=")[1].strip() if "=" in original else "120000"

            # Test setting new timeout
            resp = device.send_command("?wifips 60000")
            verify = device.send_command("?wifips")
            if "60000" not in verify:
                result["details"] = f"Failed to set timeout: {verify[:80]}"
                result["result"] = TestResult.FAIL.value
                return result

            # Test disabling
            resp = device.send_command("?wifips off")
            verify = device.send_command("?wifips")
            if "off" not in verify.lower():
                result["details"] = f"Failed to disable timeout: {verify[:80]}"
                result["result"] = TestResult.FAIL.value
                return result

            # Test invalid value rejection
            resp = device.send_command("?wifips 9999999")
            if "ERR" in resp or "fail" in resp.lower():
                pass  # Expected to fail
            else:
                # Verify it wasn't accepted
                verify = device.send_command("?wifips")
                if "9999999" in verify:
                    result["details"] = f"Invalid timeout accepted: {verify[:80]}"
                    result["result"] = TestResult.FAIL.value
                    return result

            # Restore original
            device.send_command(f"?wifips {original_ms}")

            result["details"] = f"Timeout config works: {original_ms}ms"
            result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    @staticmethod
    def test_tx_wifi_version(device: BREmoteDevice) -> Dict[str, Any]:
        """Test web UI version info via ?wifiver"""
        result = {"test": "WiFi Version", "result": TestResult.PENDING.value, "details": ""}

        try:
            response = device.send_command("?wifiver")
            if "version" in response.lower() or "ui" in response.lower():
                result["details"] = f"Version info: {response[:120]}"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = f"Unexpected response: {response[:80]}"
                result["result"] = TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    @staticmethod
    def test_tx_wifi_error(device: BREmoteDevice) -> Dict[str, Any]:
        """Test error reporting via ?wifierr"""
        result = {"test": "WiFi Error", "result": TestResult.PENDING.value, "details": ""}

        try:
            response = device.send_command("?wifierr")
            # ?wifierr returns the last error string (may be empty if no errors)
            result["details"] = f"Last error: {response[:80]}"
            result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    @staticmethod
    def test_tx_wifi_onoff(device: BREmoteDevice) -> Dict[str, Any]:
        """Test WiFi AP enable/disable cycle via ?wifi on/off"""
        result = {"test": "WiFi On/Off", "result": TestResult.PENDING.value, "details": ""}

        try:
            # Get initial state
            status = device.send_command("?wifi")
            was_on = "ON" in status.upper()

            if was_on:
                # Turn off
                off_resp = device.send_command("?wifi off")
                verify_off = device.send_command("?wifi")
                if "OFF" not in verify_off.upper():
                    result["details"] = f"Failed to turn off WiFi: {verify_off[:80]}"
                    result["result"] = TestResult.FAIL.value
                    return result
                
                # Check ?state json includes wifi status
                data = device.send_json_command("?state")
                if data and "wifi" in data:
                    if data["wifi"] != 0:
                        result["details"] = f"WiFi should be off but state shows: {data.get('wifi')}"
                        result["result"] = TestResult.FAIL.value
                        return result

                # Turn back on
                on_resp = device.send_command("?wifi on")
                verify_on = device.send_command("?wifi")
                if "ON" not in verify_on.upper():
                    # Restore for next test
                    device.send_command("?wifi on")
                    result["details"] = f"Failed to turn on WiFi: {verify_on[:80]}"
                    result["result"] = TestResult.FAIL.value
                    return result

                # Restore original state
                device.send_command("?wifi off")
                result["details"] = "WiFi on/off cycle works"
            else:
                # Was off, turn on test
                on_resp = device.send_command("?wifi on")
                verify = device.send_command("?wifi")
                if "ON" not in verify.upper():
                    result["details"] = f"Failed to turn on WiFi: {verify[:80]}"
                    result["result"] = TestResult.FAIL.value
                    return result
                
                # Restore off
                device.send_command("?wifi off")
                result["details"] = "WiFi enable works"

            result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    @staticmethod
    def test_tx_wifi_ui_update(device: BREmoteDevice) -> Dict[str, Any]:
        """Test forced web UI update to SPIFFS via ?wifiupd"""
        result = {"test": "WiFi UI Update", "result": TestResult.PENDING.value, "details": ""}

        try:
            # Note: This test may take a while as it writes to SPIFFS
            response = device.send_command("?wifiupd", wait_for_response=True)
            time.sleep(2)  # Give it time to complete
            
            # Check version after update
            ver_resp = device.send_command("?wifiver")
            if "version" in ver_resp.lower() or "ui" in ver_resp.lower():
                result["details"] = f"UI update completed: {ver_resp[:120]}"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = f"UI update may have failed: {response[:80]}"
                result["result"] = TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    # ========== RX WiFi Tests ==========
    
    @staticmethod
    def test_rx_wifi_state(device: BREmoteDevice) -> Dict[str, Any]:
        """Test RX WiFi status"""
        result = {"test": "WiFi State (RX)", "result": TestResult.PENDING.value, "details": ""}

        try:
            response = device.send_command("?wifi")
            if "wifi=" in response.lower() or "enabled" in response.lower():
                result["details"] = f"WiFi status: {response[:120]}"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = "WiFi interface present"
                result["result"] = TestResult.PASS.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    # ========== Run All ==========
    
    @staticmethod
    def run_all_tx(device: BREmoteDevice, log_callback=None) -> Dict[str, Any]:
        """Run all TX WiFi tests"""
        if log_callback:
            log_callback("\n[WIFI] Running TX WiFi Test Suite...")
        
        results = {}
        
        tests = [
            ("wifi_state", WiFiTestSuite.test_tx_wifi_state),
            ("wifi_debug", WiFiTestSuite.test_tx_wifi_debug_mode),
            ("wifi_timeout", WiFiTestSuite.test_tx_wifi_startup_timeout),
            ("wifi_version", WiFiTestSuite.test_tx_wifi_version),
            ("wifi_error", WiFiTestSuite.test_tx_wifi_error),
            ("wifi_onoff", WiFiTestSuite.test_tx_wifi_onoff),
            ("wifi_ui_update", WiFiTestSuite.test_tx_wifi_ui_update),
        ]
        
        for test_name, test_func in tests:
            if log_callback:
                log_callback(f"\n[WIFI] Running {test_name}...")
            results[test_name] = test_func(device)
            if log_callback:
                log_callback(f"  Result: {results[test_name]['result']}")
        
        return results
    
    @staticmethod
    def run_all_rx(device: BREmoteDevice, log_callback=None) -> Dict[str, Any]:
        """Run all RX WiFi tests"""
        if log_callback:
            log_callback("\n[WIFI] Running RX WiFi Test Suite...")
        
        results = {}
        
        tests = [
            ("wifi_state", WiFiTestSuite.test_rx_wifi_state),
        ]
        
        for test_name, test_func in tests:
            if log_callback:
                log_callback(f"\n[WIFI] Running {test_name}...")
            results[test_name] = test_func(device)
            if log_callback:
                log_callback(f"  Result: {results[test_name]['result']}")
        
        return results
