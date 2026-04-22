"""
BREmote Test Suite - Config Tests
Test functions for configuration get/set functionality (TX and RX).
"""

import time
from typing import Dict, Any

from ..device import BREmoteDevice
from ..models import TestResult


class ConfigTestSuite:
    """Test suite for configuration functionality"""
    
    # ========== TX Config Tests ==========
    
    @staticmethod
    def test_tx_config_keys(device: BREmoteDevice) -> Dict[str, Any]:
        """Test TX config keys listing via ?keys"""
        result = {"test": "TX Config Keys", "result": TestResult.PENDING.value, "details": ""}

        try:
            device.prepare_for_test()
            time.sleep(0.3)
            
            response = device.send_command("?keys", timeout=5.0)
            
            lines = response.split('\n')
            # TX returns key names one per line - filter for valid config key patterns
            import re
            key_lines = [l.strip() for l in lines if re.match(r'^[a-z_][a-z0-9_]*$', l.strip()) and len(l.strip()) > 2]
            
            if len(key_lines) >= 5:
                result["details"] = f"Found {len(key_lines)} config keys"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = f"Unexpected keys response: {response[:120]}"
                result["result"] = TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    @staticmethod
    def test_tx_config_get_set(device: BREmoteDevice) -> Dict[str, Any]:
        """Test TX config get/set via ?get and ?set"""
        result = {"test": "TX Config Get/Set", "result": TestResult.PENDING.value, "details": ""}

        try:
            device.prepare_for_test()
            time.sleep(0.3)
            
            get_resp = device.send_command("?get max_gears")
            if "max_gears" not in get_resp.lower() and "=" not in get_resp:
                result["details"] = f"GET failed: {get_resp[:80]}"
                result["result"] = TestResult.FAIL.value
                return result

            set_resp = device.send_command("?set max_gears 8")
            time.sleep(0.2)
            verify = device.send_command("?get max_gears")
            
            if "8" in verify:
                device.send_command("?set max_gears 10")
                result["details"] = "Config get/set works"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = f"SET value not persisted: {verify[:80]}"
                result["result"] = TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    # ========== RX Config Tests ==========
    
    @staticmethod
    def test_rx_config_keys(device: BREmoteDevice) -> Dict[str, Any]:
        """Test RX config keys listing via ?keys"""
        result = {"test": "RX Config Keys", "result": TestResult.PENDING.value, "details": ""}

        try:
            device.stop_continuous_output()
            time.sleep(1.0)
            device.flush()
            time.sleep(0.5)
            
            response = device.send_command("?keys", timeout=5.0)
            
            lines = response.split('\n')
            import re
            key_lines = [l.strip() for l in lines if re.match(r'^[a-z_][a-z0-9_]*$', l.strip()) and len(l.strip()) > 2]
            
            if len(key_lines) >= 5:
                result["details"] = f"Found {len(key_lines)} config keys"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = f"Unexpected keys response: {response[:120]}"
                result["result"] = TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    @staticmethod
    def test_rx_config_get_set(device: BREmoteDevice) -> Dict[str, Any]:
        """Test RX config get/set via ?get and ?set"""
        result = {"test": "RX Config Get/Set", "result": TestResult.PENDING.value, "details": ""}

        try:
            device.prepare_for_test()
            time.sleep(0.5)
            
            get_resp = device.send_command("?get failsafe_time")
            if "failsafe_time" not in get_resp.lower() and "=" not in get_resp:
                result["details"] = f"GET failed: {get_resp[:80]}"
                result["result"] = TestResult.FAIL.value
                return result

            set_resp = device.send_command("?set failsafe_time 2000")
            time.sleep(0.3)
            verify = device.send_command("?get failsafe_time")
            
            if "2000" in verify:
                device.send_command("?set failsafe_time 1000")
                result["details"] = "Config get/set works"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = f"SET value not persisted: {verify[:80]}"
                result["result"] = TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    # ========== SPIFFS Tests ==========
    
    @staticmethod
    def test_spiffs(device: BREmoteDevice) -> Dict[str, Any]:
        """Test SPIFFS config storage via ?conf"""
        result = {"test": "SPIFFS Config", "result": TestResult.PENDING.value, "details": ""}

        try:
            device.prepare_for_test()
            time.sleep(0.5)
            
            response = device.send_command("?conf")
            
            if "BREmote V2" in response:
                result["details"] = "SPIFFS config present"
                result["result"] = TestResult.PASS.value
            else:
                result["details"] = f"Unexpected conf response: {response[:80]}"
                result["result"] = TestResult.FAIL.value

        except Exception as e:
            result["result"] = TestResult.FAIL.value
            result["details"] = f"Error: {str(e)}"

        return result
    
    # ========== Run All ==========
    
    @staticmethod
    def run_all_tx(device: BREmoteDevice, log_callback=None) -> Dict[str, Any]:
        """Run all TX config tests"""
        if log_callback:
            log_callback("\n[CONFIG] Running TX Config Test Suite...")
        
        results = {}
        
        tests = [
            ("config_keys", ConfigTestSuite.test_tx_config_keys),
            ("config_get_set", ConfigTestSuite.test_tx_config_get_set),
            ("spiffs", ConfigTestSuite.test_spiffs),
        ]
        
        for test_name, test_func in tests:
            if log_callback:
                log_callback(f"\n[CONFIG] Running {test_name}...")
            results[test_name] = test_func(device)
            if log_callback:
                log_callback(f"  Result: {results[test_name]['result']}")
        
        return results
    
    @staticmethod
    def run_all_rx(device: BREmoteDevice, log_callback=None) -> Dict[str, Any]:
        """Run all RX config tests"""
        if log_callback:
            log_callback("\n[CONFIG] Running RX Config Test Suite...")
        
        results = {}
        
        tests = [
            ("config_keys", ConfigTestSuite.test_rx_config_keys),
            ("config_get_set", ConfigTestSuite.test_rx_config_get_set),
            ("spiffs", ConfigTestSuite.test_spiffs),
        ]
        
        for test_name, test_func in tests:
            if log_callback:
                log_callback(f"\n[CONFIG] Running {test_name}...")
            results[test_name] = test_func(device)
            if log_callback:
                log_callback(f"  Result: {results[test_name]['result']}")
        
        return results
