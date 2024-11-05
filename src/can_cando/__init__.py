"""Module provides the "python-can" plugin interface implementation
"CANdoBus" class for interfacing with the CANdo(ISO) physical devices."""

__version__ = "1.0.0"

__all__ = ["__version__", "CANdoBus"]

from can_cando.CANdo import CANdoBus
