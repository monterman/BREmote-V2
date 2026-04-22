"""
BREmote Test Suite - Device Communication
Handles serial communication with BREmote TX/RX devices.
"""

import serial
import serial.tools.list_ports
import time
import json
import logging
from typing import Optional, Dict, Any, List

from .models import DeviceType

logger = logging.getLogger(__name__)


class BREmoteDevice:
    """Represents a BREmote TX or RX device connected via serial"""
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self.device_type = DeviceType.UNKNOWN
        self.identified = False
        self.response_buffer = ""
        
    def __str__(self) -> str:
        return f"BREmoteDevice({self.port}, {self.device_type.value})"
    
    def connect(self) -> bool:
        """Connect to device on serial port"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            # Give device time to initialize
            time.sleep(0.5)
            # Flush any stale data
            if self.serial.in_waiting:
                self.serial.read(self.serial.in_waiting)
            logger.info(f"Connected to {self.port}")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from device"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            logger.info(f"Disconnected from {self.port}")
    
    def is_connected(self) -> bool:
        """Check if device is connected"""
        return self.serial is not None and self.serial.is_open
    
    def identify(self) -> DeviceType:
        """Identify device type using serial commands"""
        if not self.is_connected():
            return DeviceType.UNKNOWN
        
        # First, stop any continuous output and flush - do it twice for reliability
        self.stop_continuous_output()
        time.sleep(0.5)
        self.stop_continuous_output()
        time.sleep(0.3)
        self.flush()
        
        # Use ?conf - it returns "BREmote V2 RX" or "BREmote V2 TX" which is definitive
        # Try up to 2 times if we get garbled response
        for attempt in range(2):
            response = self.send_command("?conf", wait_for_response=True, timeout=2.0)
            
            if response and "BREmote V2" in response:
                if "RX" in response:
                    self.device_type = DeviceType.RECEIVER
                    logger.info(f"Identified {self.port} as RX")
                    self.identified = True
                    return self.device_type
                elif "TX" in response:
                    self.device_type = DeviceType.TRANSMITTER
                    logger.info(f"Identified {self.port} as TX")
                    self.identified = True
                    return self.device_type
            
            if attempt == 0:
                self.stop_continuous_output()
                time.sleep(0.5)
                self.flush()
        
        self.identified = True
        return self.device_type
    
    def send_command(self, command: str, wait_for_response: bool = True, 
                    timeout: float = 2.0) -> str:
        """Send command and optionally wait for response"""
        if not self.is_connected():
            return ""
        
        # Send command
        full_command = f"?{command}\n" if not command.startswith("?") else f"{command}\n"
        self.serial.write(full_command.encode('utf-8'))
        
        if not wait_for_response:
            return ""
        
        # Wait for response
        start_time = time.time()
        response = ""
        last_data_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.serial.in_waiting:
                data = self.serial.read(self.serial.in_waiting)
                response += data.decode('utf-8', errors='ignore')
                last_data_time = time.time()
            else:
                # Wait a bit more after last data to allow response to complete
                if response and (time.time() - last_data_time) > 0.1:
                    break
                time.sleep(0.01)
        
        return response.strip()
    
    def stop_continuous_output(self):
        """Stop any continuous output commands (like ?printInputs)"""
        if not self.is_connected():
            return
        
        # Send quit command
        self.serial.write(b"quit\n")
        time.sleep(0.5)
        
        # Flush any remaining data in buffer
        self.flush()
    
    def flush(self):
        """Flush serial input buffer"""
        if self.is_connected() and self.serial.in_waiting:
            self.serial.read(self.serial.in_waiting)
    
    def prepare_for_test(self):
        """Prepare device for testing - stop continuous output and flush buffers"""
        self.stop_continuous_output()
        self.flush()
    
    def send_json_command(self, command: str, timeout: float = 2.0) -> Optional[Dict[str, Any]]:
        """Send command and parse JSON response"""
        response = self.send_command(command, wait_for_response=True, timeout=timeout)
        
        if not response:
            return None
        
        # Try to find JSON in response
        for line in response.split('\n'):
            line = line.strip()
            if line.startswith('{') and line.endswith('}'):
                try:
                    return json.loads(line)
                except json.JSONDecodeError:
                    continue
        
        return None
    
    @staticmethod
    def scan_ports() -> List[str]:
        """Scan for available COM ports with BREmote devices"""
        ports = serial.tools.list_ports.comports()
        bre_ports = []
        
        for port_info in ports:
            port = port_info.device
            # Check for ESP32 or common USB-Serial chips
            desc_lower = port_info.description.lower()
            if any(x in desc_lower for x in ["usb", "serial", "uart", "cp210", "ch340", "ftdi", "esp32"]):
                bre_ports.append(port)
        
        return bre_ports
    
    def read_line(self, timeout: float = 1.0) -> Optional[str]:
        """Read a single line from serial with timeout"""
        if not self.is_connected():
            return None
        
        start_time = time.time()
        line = ""
        
        while time.time() - start_time < timeout:
            if self.serial.in_waiting:
                char = self.serial.read(1)
                if char == b'\n':
                    return line.strip()
                line += char.decode('utf-8', errors='ignore')
            else:
                time.sleep(0.01)
        
        return line.strip() if line else None
