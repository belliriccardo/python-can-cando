#--------------------------------------------------------------------------
#--------------------------------------------------------------------------
# THIS HEADER FILE CONTAINS THE FUNCTION IMPORTS AND CONSTANTS FOR THE
# FUNCTIONS EXPORTED BY THE CANdo API DLL - CANdo.dll v4.1
#--------------------------------------------------------------------------
#--------------------------------------------------------------------------
#  TITLE :- CANdo.dll header file - CANdoImport.py
#  AUTHOR :- Martyn Brown
#  DATE :- 22/01/21
#
#  DESCRIPTION :- Python import module for interfacing with the CANdo.dll.
#
#  UPDATES :-
#  22/01/21 Created
#
#  LICENSE :-
#  The SDK (Software Development Kit) provided for use with the CANdo device
#  is issued as FREE software, meaning that it is free for personal,
#  educational & commercial use, without restriction or time limit. The
#  software is supplied "as is", with no implied warranties or guarantees.
#
#  (c) 2021 Netronics Ltd. All rights reserved.
#------------------------------------------------------------------------------
import ctypes
#------------------------------------------------------------------------------
# CONSTANTS
#------------------------------------------------------------------------------
# CANdoOpen function constants
CANDO_CLOSED = 0  # USB channel closed
CANDO_OPEN = 1  # USB channel open
# CANdoSetState function constants
CANDO_STOP = 0  # Stop Rx/Tx of CAN messages
CANDO_RUN = 1  # Start Rx/Tx of CAN messages
# CANdo status message constants
CANDO_CLEAR = 0  # Status message flag clear
CANDO_SET = 1  # Status message flag set
# CANdoSetMode function constants
CANDO_NORMAL_MODE = 0  # Rx/Tx CAN mode
CANDO_LISTEN_ONLY_MODE = 1  # Rx only mode, no ACKs
CANDO_LOOPBACK_MODE = 2  # Tx internally looped back to Rx
# CANdo baud rate & bus load constants
CANDO_CLK_FREQ = 20000  # CANdo clk. freq. in kHz for baud rate calc.
CANDO_CLK_FREQ_HIGH = 40000  # CANdoISO & CANdo AUTO clk. freq. in kHz for baud rate calc.
CANDO_BRP_ENHANCED_OFFSET = 63  # BRP enhanced baud rate setting offset (CANdoISO & CANdo AUTO only)
# CANdo AUTO Module constants
CANDO_AUTO_V1_INPUT = 1  # V1 analogue I/P
CANDO_AUTO_V2_INPUT = 2  # V2 analogue I/P
CANDO_AUTO_MAX_NO_OF_TX_ITEMS = 10  # Max. no. of items in CAN Transmit store
CANDO_AUTO_ANALOG_INPUT_RESOLUTION = 0.0315  # Analogue I/P resolution 31.5mV
# CAN message constants
CANDO_ID_11_BIT = 0  # Standard 11 bit ID
CANDO_ID_29_BIT = 1  # Extended 29 bit ID
CANDO_DATA_FRAME = 0  # CAN data frame
CANDO_REMOTE_FRAME = 1  # CAN remote frame
# CANdo buffer lengths
CANDO_CAN_BUFFER_LENGTH = 2048  # Size of CAN message receive cyclic buffer
CANDO_MAX_STRING_SIZE = 256  # Max. no. of bytes in a string returned by the device
# CANdo H/W types
CANDO_TYPE_ANY = 0x0000  # Any H/W type
CANDO_TYPE_CANDO = 0x0001  # CANdo H/W type
CANDO_TYPE_CANDOISO = 0x0002  # CANdoISO H/W type
CANDO_TYPE_CANDO_AUTO = 0x0003  # CANdo AUTO H/W type
CANDO_TYPE_UNKNOWN = 0x8000  # Unknown H/W type
# CANdo status types
CANDO_NO_STATUS = 0  # No new status received
CANDO_DEVICE_STATUS = 1  # Device status received
CANDO_DATE_STATUS = 2  # Date status received
CANDO_BUS_LOAD_STATUS = 3  # Bus load status received
CANDO_SETUP_STATUS = 4  # CAN setup status received
CANDO_ANALOG_INPUT_STATUS = 5  # Analogue I/P status received
# CANdo USB PIDs
CANDO_PID = '8095'  # CANdo PID
CANDOISO_PID = '8660'  # CANdoISO PID
CANDO_AUTO_PID = '889B'  # CANdo AUTO PID
# CANdo function return codes
CANDO_SUCCESS = 0x0000  # All OK
CANDO_USB_DLL_ERROR = 0x0001  # WinUSB DLL error
CANDO_USB_DRIVER_ERROR = 0x0002  # WinUSB driver error
CANDO_NOT_FOUND = 0x0004  # CANdo not found
CANDO_IO_FAILED = 0x0008  # Failed to initialise USB I/O
CANDO_CONNECTION_CLOSED = 0x0010  # No CANdo channel open
CANDO_READ_ERROR = 0x0020  # USB read error
CANDO_WRITE_ERROR = 0x0040  # USB write error
CANDO_WRITE_INCOMPLETE = 0x0080  # Not all requested bytes written to CANdo
CANDO_BUFFER_OVERFLOW = 0x0100  # Overflow in cyclic buffer
CANDO_RX_OVERRUN = 0x0200  # Message received greater than max. message size
CANDO_RX_TYPE_UNKNOWN = 0x0400  # Unknown message type received
CANDO_RX_CRC_ERROR = 0x0800  # CRC mismatch
CANDO_RX_DECODE_ERROR = 0x1000  # Error decoding message
CANDO_INVALID_HANDLE = 0x2000  # Invalid device handle
CANDO_ERROR = 0x8000  # Non specific error
# CANdo CAN transmit repeat times
CANDO_REPEAT_TIME_OFF = 0  # Off 
CANDO_REPEAT_TIME_10MS = 1  # 10ms
CANDO_REPEAT_TIME_20MS = 2  # 20ms
CANDO_REPEAT_TIME_50MS = 3  # 50ms
CANDO_REPEAT_TIME_100MS = 4  # 100ms
CANDO_REPEAT_TIME_200MS = 5  # 200ms
CANDO_REPEAT_TIME_500MS = 6  # 500ms
CANDO_REPEAT_TIME_1S = 7  # 1s
CANDO_REPEAT_TIME_2S = 8  # 2s
CANDO_REPEAT_TIME_5S = 9  # 5s
CANDO_REPEAT_TIME_10S = 10  # 10s
#------------------------------------------------------------------------------
# TYPEDEFS
#------------------------------------------------------------------------------
TCANdoDeviceString = ctypes.c_char * CANDO_MAX_STRING_SIZE  # CANdo string type
PCANdoDeviceString = ctypes.POINTER(TCANdoDeviceString)  # Pointer type to CANdo string

# Type used to store device H/W type & S/N for connected CANdo
class TCANdoDevice(ctypes.Structure):
    _fields_ = [
        ('HardwareType', ctypes.c_int),
        ('SerialNo', TCANdoDeviceString)]

PCANdoDevice = ctypes.POINTER(TCANdoDevice)  # Pointer type to TCANdoDevice

# Type used to store USB info. for connected CANdo
class TCANdoUSB(ctypes.Structure):
    _fields_ = [
        ('TotalNo', ctypes.c_int),
        ('No', ctypes.c_int),
        ('OpenFlag', ctypes.c_bool),
        ('Description', TCANdoDeviceString),
        ('SerialNo', TCANdoDeviceString),
        ('Handle', ctypes.c_int)]

PCANdoUSB = ctypes.POINTER(TCANdoUSB)  # Pointer type to TCANdoUSB

# Type used to store a CAN message
class TCANdoCAN(ctypes.Structure):
    _fields_ = [
        ('IDE', ctypes.c_ubyte),
        ('RTR', ctypes.c_ubyte),
        ('ID', ctypes.c_uint),
        ('DLC', ctypes.c_ubyte),
        ('Data', ctypes.c_ubyte * 8),
        ('BusState', ctypes.c_ubyte),
        ('TimeStamp', ctypes.c_uint)]

# Type used as a cyclic buffer to store decoded CAN messages received from CANdo
class TCANdoCANBuffer(ctypes.Structure):
    _fields_ = [
        ('CANMessage', TCANdoCAN * CANDO_CAN_BUFFER_LENGTH),
        ('WriteIndex', ctypes.c_int),
        ('ReadIndex', ctypes.c_int),
        ('FullFlag', ctypes.c_bool)]

PCANdoCANBuffer = ctypes.POINTER(TCANdoCANBuffer)  # Pointer type to TCANdoCANBuffer

# Type used to store status information received from CANdo
class TCANdoStatus(ctypes.Structure):
    _fields_ = [
        ('HardwareVersion', ctypes.c_ubyte),
        ('SoftwareVersion', ctypes.c_ubyte),
        ('Status', ctypes.c_ubyte),
        ('BusState', ctypes.c_ubyte),
        ('TimeStamp', ctypes.c_uint),
        ('NewFlag', ctypes.c_ubyte)]

PCANdoStatus = ctypes.POINTER(TCANdoStatus)  # Pointer type to TCANdoStatus

#------------------------------------------------------------------------------
# Import CANdo library
lib = ctypes.windll.LoadLibrary('CANdo')
#------------------------------------------------------------------------------
# FUNCTION HANDLES & PROTOTYPES
#------------------------------------------------------------------------------
# Note : All functions return a ctypes.c_int containing a CANdo function return code
CANdoGetVersion = lib.CANdoGetVersion
CANdoGetVersion.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
CANdoGetPID = lib.CANdoGetPID
CANdoGetPID.argtypes = [ctypes.c_int, PCANdoDeviceString]
CANdoGetDevices = lib.CANdoGetDevices
CANdoGetDevices.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
CANdoOpen = lib.CANdoOpen
CANdoOpen.argtypes = [PCANdoUSB]
CANdoOpenDevice = lib.CANdoOpenDevice
CANdoOpenDevice.argtypes = [PCANdoUSB, PCANdoDevice]
CANdoClose = lib.CANdoClose
CANdoClose.argtypes = [PCANdoUSB]
CANdoFlushBuffers = lib.CANdoFlushBuffers
CANdoFlushBuffers.argtypes = [PCANdoUSB]
CANdoSetBaudRate = lib.CANdoSetBaudRate
CANdoSetBaudRate.argtypes = [PCANdoUSB, ctypes.c_ubyte, ctypes.c_ubyte, ctypes.c_ubyte, ctypes.c_ubyte, ctypes.c_ubyte,
                             ctypes.c_ubyte]
CANdoSetMode = lib.CANdoSetMode
CANdoSetMode.argtypes = [PCANdoUSB, ctypes.c_ubyte]
CANdoSetFilters = lib.CANdoSetFilters
CANdoSetFilters.argtypes = [PCANdoUSB, ctypes.c_uint, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_ubyte, ctypes.c_uint,
                            ctypes.c_uint, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_ubyte,
                            ctypes.c_uint, ctypes.c_ubyte, ctypes.c_uint]
CANdoSetState = lib.CANdoSetState
CANdoSetState.argtypes = [PCANdoUSB, ctypes.c_ubyte]
CANdoClearStatus = lib.CANdoClearStatus
CANdoClearStatus.argtypes = [PCANdoUSB]
CANdoRequestStatus = lib.CANdoRequestStatus
CANdoRequestStatus.argtypes = [PCANdoUSB]
CANdoRequestDateStatus = lib.CANdoRequestDateStatus
CANdoRequestDateStatus.argtypes = [PCANdoUSB]
CANdoRequestBusLoadStatus = lib.CANdoRequestBusLoadStatus
CANdoRequestBusLoadStatus.argtypes = [PCANdoUSB]
CANdoRequestSetupStatus = lib.CANdoRequestSetupStatus
CANdoRequestSetupStatus.argtypes = [PCANdoUSB]
CANdoRequestAnalogInputStatus = lib.CANdoRequestAnalogInputStatus
CANdoRequestAnalogInputStatus.argtypes = [PCANdoUSB]
CANdoAnalogStoreRead = lib.CANdoAnalogStoreRead
CANdoAnalogStoreRead.argtypes = [PCANdoUSB]
CANdoAnalogStoreWrite = lib.CANdoAnalogStoreWrite
CANdoAnalogStoreWrite.argtypes = [PCANdoUSB, ctypes.c_ubyte, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_ubyte, ctypes.c_ubyte,
                                  ctypes.c_double, ctypes.c_double, ctypes.c_ubyte, ctypes.c_ubyte]
CANdoAnalogStoreClear = lib.CANdoAnalogStoreClear
CANdoAnalogStoreClear.argtypes = [PCANdoUSB]
CANdoTransmitStoreRead = lib.CANdoTransmitStoreRead
CANdoTransmitStoreRead.argtypes = [PCANdoUSB]
CANdoTransmitStoreWrite = lib.CANdoTransmitStoreWrite
CANdoTransmitStoreWrite.argtypes = [PCANdoUSB, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_ubyte, ctypes.c_ubyte,
                                    ctypes.POINTER(ctypes.c_ubyte), ctypes.c_ubyte]
CANdoTransmitStoreClear = lib.CANdoTransmitStoreClear
CANdoTransmitStoreClear.argtypes = [PCANdoUSB]
CANdoTransmit = lib.CANdoTransmit
CANdoTransmit.argtypes = [PCANdoUSB, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_ubyte, ctypes.c_ubyte,
                          ctypes.POINTER(ctypes.c_ubyte), ctypes.c_ubyte, ctypes.c_ubyte]
CANdoReceive = lib.CANdoReceive
CANdoReceive.argtypes = [PCANdoUSB, PCANdoCANBuffer, PCANdoStatus]
#------------------------------------------------------------------------------
#------------------------------------------------------------------------------