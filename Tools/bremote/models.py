"""
BREmote Test Suite - Data Models
Data classes and enums used across the test suite.
"""

from dataclasses import dataclass, asdict, field
from enum import Enum
from typing import Optional, Dict, Any
from datetime import datetime


class DeviceType(Enum):
    """Device type enumeration"""
    UNKNOWN = "unknown"
    TRANSMITTER = "tx"
    RECEIVER = "rx"


class TestResult(Enum):
    """Test result enumeration"""
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"
    PENDING = "PENDING"


@dataclass
class TestReport:
    """Test report for a single device"""
    device_type: str
    port: str
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())
    tests: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    overall_result: str = TestResult.PENDING.value

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization"""
        return asdict(self)

    def add_test(self, name: str, result: TestResult, details: str = ""):
        """Add a test result"""
        self.tests[name] = {
            "result": result.value,
            "details": details
        }

    def get_summary(self) -> Dict[str, int]:
        """Get test summary counts"""
        summary = {r.value: 0 for r in TestResult}
        for test in self.tests.values():
            summary[test.get("result", "UNKNOWN")] += 1
        return summary
