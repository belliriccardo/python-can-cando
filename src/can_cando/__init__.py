"""Module provides the "python-can" plugin interface implementation "CANdoBus" class for interfacing with CANdo(ISO) physical devices."""

__version__ = "1.0.8"

__all__ = ["CANdoBus", "__version__"]

from can_cando.CANdo import CANdoBus
