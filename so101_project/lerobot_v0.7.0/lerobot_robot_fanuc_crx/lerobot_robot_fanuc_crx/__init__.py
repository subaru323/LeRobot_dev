"""FANUC CRX adapter for LeRobot v0.7.0.

Importing this package registers the `fanuc_crx` robot type.
"""

from .config_fanuc_crx import FanucCRXConfig
from .driver import CRXDriver, CRXStatus, ExistingSystemCRXDriver, MockCRXDriver, make_driver
from .fanuc_crx import FanucCRX

__all__ = [
    "CRXDriver",
    "CRXStatus",
    "ExistingSystemCRXDriver",
    "FanucCRX",
    "FanucCRXConfig",
    "MockCRXDriver",
    "make_driver",
]
