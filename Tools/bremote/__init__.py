"""
BREmote Test Suite
A comprehensive hardware testing application for BREmote V2 TX/RX units.

Usage:
    from bremote import BREmoteTester, BREmoteDevice
    
    tester = BREmoteTester()
    tester.scan_ports()
    tester.run_all_tests()
"""

from .models import DeviceType, TestResult, TestReport
from .device import BREmoteDevice
from .runner import BREmoteTester

__version__ = "2.2.4"

__all__ = [
    'DeviceType',
    'TestResult', 
    'TestReport',
    'BREmoteDevice',
    'BREmoteTester',
]
