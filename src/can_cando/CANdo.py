import logging
import sys
import threading
from ctypes import byref, c_int, c_ubyte, pointer
from pathlib import Path
from queue import Empty, Queue
from time import sleep, time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union

from can import BusABC, CanProtocol, Message
from can.broadcastmanager import CyclicSendTaskABC
from can.typechecking import AutoDetectedConfig, CanFilters

# TODO: REMOVE
sys.path.append(Path(__file__).resolve().parents[1].as_posix())
from can_cando.CANdoImport import (
    CAN_MSG_TIMESTAMP_RESOLUTION,
    CANDO_CAN_BUFFER_LENGTH,
    CANDO_DEVICE_STATUS,
    CANDO_ID_11_BIT,
    CANDO_ID_29_BIT,
    CANDO_MAX_FILTERS,
    CANDO_NO_STATUS,
    CANDO_PID,
    # CANDO_REPEAT_TIME_MAP,
    CANDO_RUN,
    CANDO_STOP,
    CANDOISO_PID,
    CANdoBusStsErr,
    CANdoCANBufferPtrType,
    CANdoClearStatus,
    CANdoClose,
    CANdoFlushBuffers,
    CANdoFuncRetCode,
    CANdoFuncRetCodeErr,
    CANdoGetDevices,
    CANdoGetPID,
    CANdoGetVersion,
    CANdoInitializationError,
    CANdoMode,
    CANdoOpenDevice,
    # CANdoRequestBusLoadStatus,
    # CANdoRequestDateStatus,
    CANdoOperationError,
    CANdoPIDType,
    CANdoReceive,
    CANdoRepeatTime,
    CANdoRequestStatus,
    CANdoSetBaudRate,
    CANdoSetFilters,
    CANdoSetMode,
    CANdoSetState,
    CANdoStatusPtrType,
    CANdoStsErr,
    CANdoTransmit,
    CANdoUSBPtrType,
    MsgDataType,
    MsgQueueType,
    TCANdoCANBuffer,
    TCANdoDevice,
    TCANdoDeviceString,
    TCANdoStatus,
    TCANdoUSB,
    cando_get_timing,
)

log = logging.getLogger("can.CANdo")
log.addHandler(logging.StreamHandler())
log.setLevel(logging.INFO)

if sys.version_info >= (3, 12):
    from typing import override  # novermin
else:

    def override(f: Callable[..., Any]) -> Callable[..., Any]:
        return f


class CANdoBus(BusABC):
    # No. of devices to look for
    NoOfDevices: c_int

    TCANdoDevices: Any
    CANdoDevices: Any
    PCANdoDevices: Any

    CANdoPID: CANdoPIDType

    CANdoUSB: TCANdoUSB
    CANdoUSBPtr: CANdoUSBPtrType

    # Create a buffer for the received CAN messages
    CANdoCANBuffer: TCANdoCANBuffer
    CANdoCANBufferPtr: CANdoCANBufferPtrType

    # Create a buffer for received CANdo status
    CANdoStatus: TCANdoStatus
    CANdoStatusPtr: CANdoStatusPtrType

    default_bitrate = 500000
    DataArrays = [c_ubyte * dlc for dlc in range(9)]

    @override
    def __init__(
        self,
        channel: Optional[int] = None,
        can_filters: Optional[CanFilters] = None,
        bitrate: Optional[int] = default_bitrate,
        is_candoiso: Optional[bool] = False,
        **kwargs: object,
    ) -> None:
        self.channel = channel
        self._filters = can_filters

        APIVersion = c_int()
        DLLVersion = c_int()
        DriverVersion = c_int()
        CANdoGetVersion(
            byref(APIVersion),
            byref(DLLVersion),
            byref(DriverVersion),
        )

        self.channel_info = (
            f"CANdoGetVersion >\n  API = v{APIVersion.value / 10}\n  "
            f"DLL = v{DLLVersion.value / 10}\n  "
            f"Driver = v{DriverVersion.value / 10}\n"
        )

        self._detect_cando_iso()

        if channel is None:
            # Channel discovery mode
            return

        if self.NoOfDevices.value == 0:
            raise CANdoInitializationError("No CANdo(ISO) devices detected!")

        if channel < 0:
            raise CANdoInitializationError(f"Invalid channel number: {channel}, must be >= 0")

        if channel >= self.NoOfDevices.value:
            raise CANdoInitializationError(
                f"Invalid channel number: {channel}, only {self.NoOfDevices.value} devices detected and values start at 0",
            )

        self.is_candoiso = is_candoiso
        self.bit_timing = cando_get_timing(bitrate or self.default_bitrate)

        # CANdoOpen function
        self.CANdoUSB = TCANdoUSB()  # Create an instance of TCANdoUSB
        self.CANdoUSBPtr = pointer(self.CANdoUSB)
        self.CANdoCANBuffer = TCANdoCANBuffer()
        self.CANdoCANBufferPtr = pointer(self.CANdoCANBuffer)
        self.CANdoStatus = TCANdoStatus()
        self.CANdoStatusPtr = pointer(self.CANdoStatus)

        self.cando_open_device(channel)

        if not self.CANdoUSB.OpenFlag:
            raise CANdoInitializationError("Device not open!")

        # Ensure that when we request the device's status,
        # the receiving thread does not interfere.
        self._recv_lock = threading.Lock()

        self.set_timings()
        # Normal mode: Rx/Tx CAN mode
        self.set_mode(CANdoMode.CANDO_NORMAL_MODE)

        self.filters_active = False
        # Will be set to True if filters are correctly set
        self.set_filters(can_filters)

        self.flush_buffers()
        sleep(0.01)

        self.cando_clear_status()
        self.set_state(CANDO_RUN)

        # channel_info present in BusABC
        self.channel_info += self.get_status_description()

        self.t_start = time()

        self.msg_queue: MsgQueueType = Queue()

        self._is_shutdown = False
        self._can_protocol: CanProtocol = CanProtocol.CAN_20

        self._recv_thread = threading.Thread(target=self._recv_t, daemon=True)
        self._recv_thread.start()

    def cando_open_device(self, channel: int) -> None:
        """Allow a connection to a specific CANdo device.

        Opens a connection to the CANdo device specified by the hardware type &
        serial number passed into the function via the CANdoDevicePointer
        parameter. If successful the function returns a handle to the connected
        device, together with the device description & the serial number, via the
        CANdoUSBPointer parameter. In addition, the hardware type of the
        connected device is returned via the CANdoDevicePointer parameter.

        The CANdoDevice structure passed into the function contains fields for
        hardware type & serial number. To uniquely specify a CANdo device for
        connection, both fields must be populated. To select a particular hardware
        type only, the serial number maybe set to an empty string (“”), the function
        will then connect to the next free device matching the selected hardware
        type. To select a device with a specific serial number regardless of hardware
        type, the hardware type maybe set to CANDO_TYPE_ANY. A call to this
        function with the serial number set to an empty string & the hardware type
        set to CANDO_TYPE_ANY is equivalent to a call to the CANdoOpen(...)
        function, except that the hardware type of the connected device is returned
        via the CANdoDevicePointer parameter.
        """
        # Open a connection to the selected device
        if CANdoOpenDevice(self.CANdoUSBPtr, byref(self.CANdoDevices[channel])) == CANdoFuncRetCode.CANDO_SUCCESS:
            log.debug(
                f"Connection open to {self.CANdoUSB.Description.decode('utf-8')}, S/N {self.CANdoUSB.SerialNo.decode('utf-8')}",
            )
        else:
            raise CANdoInitializationError("Device unavailable")

    def set_timings(self) -> None:
        # Wrapper for BusABC
        self.cando_set_baudrate(
            self.bit_timing.sjw,
            self.bit_timing.brp,
            self.bit_timing.phseg1,
            self.bit_timing.phseg2,
            self.bit_timing.propseg,
            self.bit_timing.sam,
        )

    def cando_set_baudrate(
        self,
        sjw: int,
        brp: int,
        phseg1: int,
        phseg2: int,
        propseg: int,
        sam: int,
    ) -> None:
        """Set the CAN bus baud rate (BR) & bit sampling point (SP).

        :param sjw: - sync. jump width, 0 - 3. (This is the width of the synchronisation jump
        used by the CAN module to achieve synchronisation. 0 = 1 jump bit ... 3 = 4
        jump bits)

        :param brp: - baud rate prescaler, 0 - 63 for a CANdo device, 0 - 31 & 64 - 127 for
        a CANdoISO or CANdo AUTO device (See description below)

        :param phseg1: - phase segment 1, 0 - 7 (See description below)

        :param phseg2: - phase segment 2, 0 - 7 (See description below)

        :param propseg: - propagation segment, 0 - 7 (See description below)

        :param sam: - number of samples per data bit, 0 - 1 (0 = Sample each data bit
        once, 1 = Sample each data bit thrice)

        NOTE : All these CAN register values are 0 based, so that 0 actually equals
        1. For example, a PHSEG1 value of 3 equates to a 4 PHSEG1 segments in
        the CAN bit timing. The equations shown below all expect the 0 based
        values.

        According to the following equation:

        BR = 20000000 / 2 x (BRP + 1) x (4 +PROPSEG + PHSEG1 + PHSEG2)

        NOTE : For a CANdo device, the BRP may be programmed between 0 - 63.
        For a CANdoISO or CANdo AUTO device the BRP is restricted to between 0
        - 31. Within these ranges the BR maybe calculated using the equation
        shown above. The minimum programmable baud rate for a CANdo device is
        6.25k & for a CANdoISO or CANdo AUTO device is 12.5k baud.

        SP = (3 + PROPSEG + PHSEG1) x 100 / (4 + PROPSEG + PHSEG1 +
        PHSEG2)

        Some rules apply with respect to the values entered into these equations
        due to their interdependence upon one another, as described below -

        PROPSEG + PHSEG1 + 1 >= PHSEG2
        PROPSEG + PHSEG1 + PHSEG2 >= 4
        PHSEG2 >= SJW

        The table below gives typical settings for some common baud rates:
        _______________________________________
        |  BR   SP  BRP PROPSEG PHSEG1 PHSEG2 |
        |  50k  70%   9     4      7      5   |
        | 125k  75%   4     2      7      3   |
        | 250k  75%   1     5      7      4   |
        | 500k  70%   0     4      7      5   |
        |   1M  80%   0     1      4      1   |
        |_____________________________________|

        For a CANdoISO or CANdo AUTO device, the baud rate may be prog-
        rammed with a higher setting resolution using a BRP in the range 64 - 127.
        In this case the baud rate is calculated using the equation shown below -

        BR = 40000000 / {2 x (BRP - 63) x (4 + PROPSEG + PHSEG1 + PHSEG2)}

        This allows additional baud rates to be programmed that are not available for
        a CANdo device. For example, programming with a BRP value of 64 -

        800k = 40000000 / {2 x (64 - 63) x (4 + 7 + 7 + 7)}

        The sample point calculation remains the same as before.

        NOTE : The baud rate settings are stored internally in the CANdo unit in
        non-volatile memory. This ensures that the unit powers up with the last
        programmed baud rate settings automatically. The internal memory is only
        updated if the baud rate parameters specified in the CANdoSetBaudRate(...)
        function differ from those already stored. If the baud rate parameters do
        differ, then the unit can take up to 100ms to store the new settings. During
        this period the unit is unable to accept any new commands.

        NOTE : The values used for BRP, PROPSEG, PHSEG1 & PHSEG2 in the
        above equations for the SDK are all 1 less than the corresponding values
        shown in the CANdo Application 'CAN Setup' page. The values shown in the
        'CAN Setup' page all start at 1, while those used in the SDK all start at 0.
        """
        if (
            CANdoSetBaudRate(
                self.CANdoUSBPtr,
                sjw,
                brp,
                phseg1,
                phseg2,
                propseg,
                sam,
            )
            != CANdoFuncRetCode.CANDO_SUCCESS
        ):
            raise CANdoInitializationError("Failed to set baud rate!")

        # The programmer's guide state that if the new settings differ from the current settings, the device can take up
        # to 100ms to store the new settings, during which time the device will not accept to any new commands.
        sleep(0.1)

    def set_mode(self, mode: CANdoMode) -> None:
        """Set the decice's operating mode.

        :param mode: operating mode, 0 - 2 (0 = Normal, 1 = Listen only, 2 = Loopback)
        """
        ret_val = CANdoFuncRetCode(CANdoSetMode(self.CANdoUSBPtr, mode))

        if ret_val != CANdoFuncRetCode.CANDO_SUCCESS:
            log.error(f"Failed to set mode: {mode}, error code: {ret_val}")
            raise CANdoInitializationError(f"Failed to set mode: {mode}")

        sleep(0.01)  # Sleep for 10ms if the mode was changed

    def flush_buffers(self) -> None:
        """Flushes the internal USB read & write buffers.

        :raises ValueError: if the device is not open
        """
        if not self.CANdoUSB.OpenFlag:
            raise CANdoOperationError("Device not open!")

        if CANdoFlushBuffers(self.CANdoUSBPtr) != CANdoFuncRetCode.CANDO_SUCCESS:
            log.error("Failed to flush buffers!")

    def cando_set_filters(
        self,
        Rx1Mask: int = 0,
        Rx1IDE1: int = CANDO_ID_29_BIT,
        Rx1Filter1: int = 0,
        Rx1IDE2: int = CANDO_ID_11_BIT,
        Rx1Filter2: int = 0,
        Rx2Mask: int = 0,
        Rx2IDE1: int = CANDO_ID_29_BIT,
        Rx2Filter1: int = 0,
        Rx2IDE2: int = CANDO_ID_11_BIT,
        Rx2Filter2: int = 0,
        Rx2IDE3: int = CANDO_ID_29_BIT,
        Rx2Filter3: int = 0,
        Rx2IDE4: int = CANDO_ID_11_BIT,
        Rx2Filter4: int = 0,
    ) -> None:
        """Set the CANdo(ISO) filters.

        :param Rx1Mask: - receive buffer 1 mask
        :param Rx1IDE1: - flag to indicate Rx1Mask & Rx1Filter1 values are either 11 or 29 bit values (0 = 11 bit ID, 1 = 29 bit ID)
        :param Rx1Filter1: - receive buffer 1, filter 1 (See Appendix B for further details)
        :param Rx1IDE2: - flag to indicate Rx1Filter2 value is either an 11 or 29 bit value (0 = 11 bit ID, 1 = 29 bit ID)
        :param Rx1Filter2: - receive buffer 1, filter 2 (See Appendix B for further details)
        :param Rx2Mask: - receive buffer 2 mask
        :param Rx2IDE1: - flag to indicate Rx2Mask & Rx2Filter1 values are either 11 or 29 bit values (0 = 11 bit ID, 1 = 29 bit ID)
        :param Rx2Filter1: - receive buffer 2, filter 1 (See Appendix B for further details)
        :param Rx2IDE2: - flag to indicate Rx2Filter2 value is either an 11 or 29 bit value (0 = 11 bit ID, 1 = 29 bit ID)
        :param Rx2Filter2: - receive buffer 2, filter 2 (See Appendix B for further details)
        :param Rx2IDE3: - flag to indicate Rx2Filter3 value is either an 11 or 29 bit value (0 = 11 bit ID, 1 = 29 bit ID)
        :param Rx2Filter3: - receive buffer 2, filter 3 (See Appendix B for further details)
        :param Rx2IDE4: - flag to indicate Rx2Filter4 value is either an 11 or 29 bit value (0 = 11 bit ID, 1 = 29 bit ID)
        :param Rx2Filter4: - receive buffer 2, filter 4 (See Appendix B for further details)
        """
        ret = CANdoSetFilters(
            self.CANdoUSBPtr,
            Rx1Mask,
            Rx1IDE1,
            Rx1Filter1,
            Rx1IDE2,
            Rx1Filter2,
            Rx2Mask,
            Rx2IDE1,
            Rx2Filter1,
            Rx2IDE2,
            Rx2Filter2,
            Rx2IDE3,
            Rx2Filter3,
            Rx2IDE4,
            Rx2Filter4,
        )

        if ret != CANdoFuncRetCode.CANDO_SUCCESS:
            raise CANdoInitializationError(f"Failed to set CANdo(ISO) filters: {CANdoFuncRetCodeErr(ret)}")

        # Sleep for 10ms as stated in the programmer's guide
        sleep(0.01)

    def cando_request_status(self) -> None:
        """Request the CAN bus & internal CANdo status.

        After sending this command to CANdo, the status is transmitted back to the
        PC in <1ms. To read the status, call the CANdoReceive(...) function &
        interrogate the NewFlag & status information within the TCANdoStatus
        parameter.

        (A status message is automatically sent back to the PC if an error is
        detected on the CAN bus, or if there is an internal system error within the
        CANdo device.)
        """

    def cando_get_date_status(self) -> None:
        """Request the date of manufacture of the CANdo device.

        After sending this command to CANdo, the date status is transmitted back to
        the PC in <1ms. To read the status, call the CANdoReceive(...) function &
        interrogate the NewFlag & status information within the TCANdoStatus
        parameter.
        """
        # TODO: Use "CANdoRequestDateStatus" function

    def cando_request_bus_load_status(self) -> None:
        """Request the CAN bus loading as calculated by connected device.

        The bus load is calculated every second, while the device is running.

        After sending this command to the device, the bus load status is transmitted
        back to the PC in <1ms. To read the status, call the CANdoReceive(...)
        function & interrogate the NewFlag & status information within the
        TCANdoStatus parameter.
        """
        # TODO: Use the "CANdoRequestBusLoadStatus" function

    def get_status_description(self) -> str:
        """Request the CAN bus & internal CANdo status.

        After sending this command to CANdo, the status is transmitted back to the
        PC in <1ms. To read the status, call the CANdoReceive(...) function &
        interrogate the NewFlag & status information within the TCANdoStatus
        parameter.

        (A status message is automatically sent back to the PC if an error is
        detected on the CAN bus, or if there is an internal system error within the
        CANdo device.)
        """
        sts = "Status unknown"
        with self._recv_lock:  # Ensure that the receiving thread does not interfere
            if CANdoRequestStatus(self.CANdoUSBPtr) == CANdoFuncRetCode.CANDO_SUCCESS:
                # Wait for device to reply; in the programmer's guide it's mentioned that it takes < 1ms to get a response.
                sleep(0.001)
                self.CANdoStatus.NewFlag = CANDO_NO_STATUS  # Clear status flag

                # Receive status message
                if (
                    CANdoReceive(
                        self.CANdoUSBPtr,
                        self.CANdoCANBufferPtr,
                        self.CANdoStatusPtr,
                    )
                    == CANdoFuncRetCode.CANDO_SUCCESS
                ) and self.CANdoStatus.NewFlag == CANDO_DEVICE_STATUS:  # type: ignore
                    # Display new device status
                    sts = "  {} S/N {} H/W v{}, S/W v{}, Status = {}, Bus State = {}\n".format(
                        self.CANdoUSB.Description.decode("utf-8"),
                        self.CANdoUSB.SerialNo.decode("utf-8"),
                        self.CANdoStatus.HardwareVersion / 10,
                        self.CANdoStatus.SoftwareVersion / 10,
                        self.CANdoStatus.Status,
                        self.CANdoStatus.BusState,
                    )

                    sts += f"Status Err: {CANdoStsErr(self.CANdoStatus.Status)}\n"
                    sts += f"Bus State Err: {CANdoBusStsErr(self.CANdoStatus.BusState)}\n"

        return sts

    def print_status(self) -> None:
        log.info(self.get_status_description())

    def cando_clear_status(self) -> None:
        """Clear the internal system status within the CANdo device.

        The CAN busstatus is not cleared by this function, as this is determined by the state of the
        CAN bus & CAN module only.
        """
        ret_val = CANdoFuncRetCode(CANdoClearStatus(self.CANdoUSBPtr))

        if ret_val != CANdoFuncRetCode.CANDO_SUCCESS:
            log.error(f"Failed to clear status, error code: {ret_val}")

    def set_state(self, state: int) -> None:
        """Set the run state.

        :param state: 0 - 1 (0 = Stop, 1 = Run)

        Stop - disables reception & transmission of messages on the CAN bus. Also
        disables the CAN transceiver.

        Run - enables transmission & reception of CAN messages. Also resets the
        message timestamp & enables the CAN transceiver.
        """
        if state not in (CANDO_STOP, CANDO_RUN):
            raise CANdoOperationError(f"Invalid state: {state}")

        ret_val = CANdoFuncRetCode(CANdoSetState(self.CANdoUSBPtr, state))

        if ret_val != CANdoFuncRetCode.CANDO_SUCCESS:
            log.error(f"Failed to set state to {state}, error code: {ret_val}")
            raise CANdoOperationError(f"Failed to set state to {state}!")

        sleep(0.01)  # Sleep for 10ms

    def _detect_cando_iso(self) -> None:
        # CANdoGetDevices & CANdoGetPID functions
        self.NoOfDevices = c_int(10)  # Look for up to 10 CANdo devices
        # Create an array type of TCANdoDevice for those devices
        self.TCANdoDevices = TCANdoDevice * self.NoOfDevices.value
        self.CANdoDevices = self.TCANdoDevices()  # Create an instance of TCANdoDevices array
        # Get a pointer to the TCANdoDevices array
        self.PCANdoDevices = pointer(self.CANdoDevices)
        self.CANdoPID = TCANdoDeviceString()
        if CANdoGetDevices(self.PCANdoDevices, byref(self.NoOfDevices)) == CANdoFuncRetCode.CANDO_SUCCESS:
            for CANdoNo in range(self.NoOfDevices.value):
                if CANdoGetPID(CANdoNo, byref(self.CANdoPID)) == CANdoFuncRetCode.CANDO_SUCCESS:
                    # PID
                    log.debug(f"  Device no. {CANdoNo} >\n    PID = 0x{self.CANdoPID.value.decode('utf-8').upper()}")
                    if self.CANdoPID.value not in [CANDO_PID, CANDOISO_PID]:
                        log.warning(f"    Device no. {CANdoNo} is not a CANdo(ISO) device!")

                    # H/W type & S/N
                    log.debug(
                        f"    H/W type = {self.CANdoDevices[CANdoNo].HardwareType}\n    "
                        f"S/N = {self.CANdoDevices[CANdoNo].SerialNo.decode('utf-8')}\n",
                    )

    def cando_transmit(
        self,
        IDExtended: int,
        ID: int,
        RTR: int,
        DLC: int,
        Data: MsgDataType,
        BufferNo: int = 0,  # Send instantly by default
        RepeatTime: CANdoRepeatTime = CANdoRepeatTime.CANDO_REPEAT_TIME_OFF,  # No repeat by default
    ) -> int:
        """Transmit a message on the CAN bus.

        The transmitter includes sixteen buffers (0 - 15), one single shot (buffer 0) &
        fifteen (buffers 1 - 15) repeat buffers. A message loaded into buffer 0 is
        transmitted once only. The repeat buffers once loaded with a CAN message,
        repeat the transmission at the rate specified by the RepeatTime parameter.
        The repeat buffers allow for accurate repetitive transmissions on the CAN
        bus with no overhead on the PC.

        :param IDExtended:
            flag to indicate 11 or 29 bit message ID, 0 - 1 (0 = 11 bit ID, 1 = 29 bit ID)

        :param ID:
            CAN message ID, 0 - 7FF (11 bit ID) or 0 - 1FFFFFFF (29 bit ID)

        :param RTR:
            remote frame flag, 0 - 1 (0 = Data, 1 = Remote frame)

        :param DLC:
            Data Length Code, 0 - 8 (No. of data bytes in message)

        :param Data:
            byte array containing data to be transmitted (Note: A null pointer is
            permitted for a remote frame transmission, see note below)

        :param BufferNo:
            buffer within CANdo to use for the transmission, 0 - 15. Buffer 0
            transmits directly to the CAN bus. Buffers 1 - 15 are specialised repeat
            buffers that allow accurate timed transmissions of repetitive messages. To
            transmit a message at a specified rate, select one of the buffers 1 - 15 & set
            the RepeatTime parameter to one of the values specified below. Once
            loaded, a repeat buffer transmits the message in the buffer repetitively, at
            the rate specified by the RepeatTime parameter. To stop a repeat buffer
            transmitting, set the RepeatTime to 0.

        :param RepeatTime:
            the message transmit repeat time, 0 - 10. (Only applies to the
            repeat buffers 1 - 15, set to 0 for buffer 0). Values are as follows:

        0 Off
        1 10ms
        2 20ms
        3 50ms
        4 100ms
        5 200ms
        6 500ms
        7 1000ms
        8 2000ms
        9 5000ms
        10 10000ms
        """
        if not self.CANdoUSB.OpenFlag:
            raise CANdoOperationError("Device not open!")

        return int(
            CANdoTransmit(
                self.CANdoUSBPtr,  # Device handle pointer
                IDExtended,
                ID,
                RTR,
                DLC,
                Data,
                int(BufferNo),
                int(RepeatTime),
            ),
        )

    def cando_stop_all_periodic_transmissions(self) -> None:
        """Stop all periodic transmissions on the CAN bus."""
        if not self.CANdoUSB.OpenFlag:
            raise CANdoOperationError("Device not open!")

        for buf_no in range(1, 16):
            ret = self.cando_transmit(
                0,
                0,
                0,
                0,
                self.DataArrays[0](),
                buf_no,
                CANdoRepeatTime.CANDO_REPEAT_TIME_OFF,
            )
            if ret != CANdoFuncRetCode.CANDO_SUCCESS:
                log.error(f"Failed to stop transmission on buffer {buf_no}, error code: {CANdoFuncRetCodeErr(ret)}")

    def _recv_t(self) -> None:
        # Local copy to avoid global lookups
        ts_resolution = CAN_MSG_TIMESTAMP_RESOLUTION

        while not self._is_shutdown:
            has_received_messages = False

            with self._recv_lock:  # Do not interfere with status requests
                # Clear status flag
                self.CANdoStatus.NewFlag = CANDO_NO_STATUS

                ret = CANdoReceive(self.CANdoUSBPtr, self.CANdoCANBufferPtr, self.CANdoStatusPtr)
                if ret != CANdoFuncRetCode.CANDO_SUCCESS:
                    raise CANdoOperationError(f"Failed to receive messages, error code: {CANdoFuncRetCodeErr(ret)}")

                # log.debug(f"Status: {CANdoStsErr(self.CANdoStatus.Status)}, \nBus State: {CANdoBusStsErr(self.CANdoStatus.BusState)}")

                while self.CANdoCANBuffer.ReadIndex != self.CANdoCANBuffer.WriteIndex or self.CANdoCANBuffer.FullFlag:
                    has_received_messages = True

                    arbitration_id = self.CANdoCANBuffer.CANMessage[self.CANdoCANBuffer.ReadIndex].ID
                    ide = self.CANdoCANBuffer.CANMessage[self.CANdoCANBuffer.ReadIndex].IDE
                    rtr = self.CANdoCANBuffer.CANMessage[self.CANdoCANBuffer.ReadIndex].RTR
                    dlc = self.CANdoCANBuffer.CANMessage[self.CANdoCANBuffer.ReadIndex].DLC
                    data = bytearray(self.CANdoCANBuffer.CANMessage[self.CANdoCANBuffer.ReadIndex].Data[:dlc])

                    # # Unused for now; however, check for the "state" property and "BusState" enum  in bus.py
                    # bus_state = self.CANdoCANBuffer.CANMessage[self.CANdoCANBuffer.ReadIndex].BusState

                    # Timestamp - a timestamp indicating the receive time of the message since starting CANdo.
                    timestamp = (self.CANdoCANBuffer.CANMessage[self.CANdoCANBuffer.ReadIndex].TimeStamp * ts_resolution) + self.t_start

                    # Increment index onto next circular buffer element
                    self.CANdoCANBuffer.ReadIndex = (self.CANdoCANBuffer.ReadIndex + 1) % CANDO_CAN_BUFFER_LENGTH
                    self.CANdoCANBuffer.FullFlag = False  # No longer full

                    self.msg_queue.put((timestamp, arbitration_id, bool(ide), rtr, dlc, data))

                    if self.CANdoStatus.NewFlag == CANDO_DEVICE_STATUS:  # type: ignore
                        # log.debug(f"  Status = {self.CANdoStatus.Status}, Bus State = {self.CANdoStatus.BusState}")
                        self.CANdoStatus.NewFlag = CANDO_NO_STATUS  # Clear status flag

            if not has_received_messages:
                # No messages were received, sleep for a short time OUTSIDE the lock
                sleep(0.01)

    @override
    def flush_tx_buffer(self) -> None:
        """Discard every message that may be queued in the output buffer(s)."""
        self.flush_buffers()

    @override
    @staticmethod
    def _detect_available_configs() -> List[AutoDetectedConfig]:
        interfaces: List[AutoDetectedConfig] = []

        bus = CANdoBus(None)  # Channel discovery mode
        if bus.NoOfDevices.value > 0:
            interfaces = [AutoDetectedConfig(interface="cando", channel=c) for c in range(bus.NoOfDevices.value)]

        bus.shutdown()  # Close connection
        del bus

        return interfaces

    @override
    def send(self, msg: Message, timeout: Optional[float] = None) -> None:
        if not self.CANdoUSB.OpenFlag:
            raise CANdoOperationError("Device not open!")

        # DataArray = c_ubyte * msg.dlc  # Create a data array type

        tx_res = self.cando_transmit(
            int(msg.is_extended_id),
            msg.arbitration_id,
            int(msg.is_remote_frame),
            msg.dlc,
            # DataArray(*msg.data),  # Create & initialise an instance of the data array
            self.DataArrays[msg.dlc](*msg.data),  # Create & initialise an instance of the data array
        )

        if tx_res != CANdoFuncRetCode.CANDO_SUCCESS:
            raise CANdoOperationError(f"Failed to transmit message, error code(s): {CANdoFuncRetCodeErr(tx_res)}")

    @override
    def _recv_internal(self, timeout: Optional[float]) -> Tuple[Optional[Message], bool]:
        try:
            msg = self.msg_queue.get(timeout=timeout)
            return (
                Message(
                    timestamp=msg[0],
                    arbitration_id=msg[1],
                    is_extended_id=msg[2],
                    is_remote_frame=msg[3],
                    dlc=msg[4],
                    data=msg[5],
                    channel=self.channel,
                ),
                # Filters weren't set => all messages are received
                self.filters_active,
            )
        except Empty:
            return (None, False)
        except Exception as e:
            log.exception("Error receiving message!")
            raise CANdoOperationError from e

    @override
    def set_filters(self, filters: Optional[CanFilters] = None) -> None:
        if filters is None:
            return

        if len({f["can_mask"] for f in filters}) > CANDO_MAX_FILTERS:
            raise CANdoInitializationError(f"CANdo(ISO) device supports only {CANDO_MAX_FILTERS} filter masks!")

        flt_msk_map: Dict[int, List[Tuple[int, int]]] = {}
        for flt in filters:
            can_id = flt["can_id"]
            can_mask = flt["can_mask"]
            extended = int(flt.get("extended", False))  # type: ignore

            if can_mask not in flt_msk_map:
                flt_msk_map[can_mask] = []

            flt_msk_map[can_mask].append((extended, can_id))

        masks = list(flt_msk_map.keys())

        # Which mask has the longest list of IDs? We do this so that masks[0] is the
        # mask with the least IDs, and masks[1] is the mask with the most IDs.
        masks.sort(key=lambda x: len(flt_msk_map[x]))

        Rx1Mask: int = masks[0]
        Rx1MskFlt = flt_msk_map[Rx1Mask]

        Rx1IDE1, Rx1Filter1 = Rx1MskFlt[0]

        # Default values
        Rx1IDE2: int = CANDO_ID_11_BIT
        Rx1Filter2: int = 0

        if len(Rx1MskFlt) > 1:
            Rx1IDE2, Rx1Filter2 = Rx1MskFlt[1]

        if len(Rx1MskFlt) > 2:
            log.error(f"CANdo(ISO) device only supports 2 IDs in the first masking filter, ignoring the rest: {Rx1MskFlt[2:]}")

        # Default values
        Rx2Mask: int = 0
        Rx2IDE1: int = CANDO_ID_29_BIT
        Rx2Filter1: int = 0
        Rx2IDE2: int = CANDO_ID_11_BIT
        Rx2Filter2: int = 0
        Rx2IDE3: int = CANDO_ID_29_BIT
        Rx2Filter3: int = 0
        Rx2IDE4: int = CANDO_ID_11_BIT
        Rx2Filter4: int = 0

        if len(masks) > 1:
            Rx2Mask = masks[1]
            Rx2MskFlt = flt_msk_map[Rx2Mask]
            Rx2IDE1, Rx2Filter1 = Rx2MskFlt[0]

            if len(Rx2MskFlt) > 1:
                Rx2IDE2, Rx2Filter2 = Rx2MskFlt[1]

            if len(Rx2MskFlt) > 2:
                Rx2IDE3, Rx2Filter3 = Rx2MskFlt[2]

            if len(Rx2MskFlt) > 3:
                Rx2IDE4, Rx2Filter4 = Rx2MskFlt[3]

            if len(Rx2MskFlt) > 4:
                log.error(f"CANdo(ISO) device only supports 4 IDs in the second masking filter, ignoring the rest: {Rx2MskFlt[4:]}")

        self.cando_set_filters(
            Rx1Mask=Rx1Mask,
            Rx1IDE1=Rx1IDE1,
            Rx1Filter1=Rx1Filter1,
            Rx1IDE2=Rx1IDE2,
            Rx1Filter2=Rx1Filter2,
            Rx2Mask=Rx2Mask,
            Rx2IDE1=Rx2IDE1,
            Rx2Filter1=Rx2Filter1,
            Rx2IDE2=Rx2IDE2,
            Rx2Filter2=Rx2Filter2,
            Rx2IDE3=Rx2IDE3,
            Rx2Filter3=Rx2Filter3,
            Rx2IDE4=Rx2IDE4,
            Rx2Filter4=Rx2Filter4,
        )
        # If we got here, no exceptions were raised, so filters are active
        self.filters_active = True

    @override
    def shutdown(self) -> None:
        try:
            super().shutdown()
        except Exception:
            log.exception("Error during shutdown!")

        if hasattr(self, "_recv_thread") and self._recv_thread.is_alive():
            self._recv_thread.join()

        # CANdo connection is open, so close
        if hasattr(self, "CANdoUSB"):
            if self.CANdoUSB.OpenFlag:
                res = CANdoClose(self.CANdoUSBPtr)
                if res == CANdoFuncRetCode.CANDO_SUCCESS:
                    log.debug("Connection closed successfully.")
                else:
                    log.error(f"Failed to close connection, error code: {CANdoFuncRetCodeErr(res)}")
            else:
                log.debug("Connection to device is already closed!")

    @override
    def send_periodic(
        self,
        msgs: Union[Message, Sequence[Message]],
        period: float,
        duration: Optional[float] = None,
        store_task: bool = True,
        modifier_callback: Optional[Callable[[Message], None]] = None,
    ) -> CyclicSendTaskABC:
        if not self.CANdoUSB.OpenFlag:
            raise CANdoOperationError("Device not open!")
        log.warning(
            "To send periodic messages using CANdo(ISO)'s internal specialized repeat buffers, check \"cando_transmit\"'s implementation.",
        )
        return super().send_periodic(msgs, period, duration, store_task, modifier_callback)

    @override
    def stop_all_periodic_tasks(self, remove_tasks: bool = True) -> None:
        if hasattr(self, "CANdoUSB"):
            self.cando_stop_all_periodic_transmissions()
        super().stop_all_periodic_tasks(remove_tasks=remove_tasks)


def main() -> None:
    interfaces = CANdoBus._detect_available_configs()  # type: ignore

    if len(interfaces) == 0:
        print("No CANdo(ISO) devices detected!")
        return

    print("Available CANdo(ISO) channels:")
    for i in interfaces:
        print(f"  {i}")
    selected = int(input("Select channel: ") or "0")

    bus = CANdoBus(selected)

    try:
        while True:
            _in = input(
                "Press 't' to transmit a message, 'r' to receive a message,'s' to see the device's status and 'q' or 'Ctrl-C' to quit: ",
            )
            if _in == "q":
                break
            if _in == "s":
                bus.print_status()
            if _in == "t":
                bus.send(
                    Message(
                        arbitration_id=0x1500,
                        data=[1, 2, 3, 4, 5, 6, 7, 8],
                        is_extended_id=True,
                    ),
                )
            if _in == "r":
                msg = bus.recv(timeout=1)
                if msg is not None:
                    print(msg)
                else:
                    print("No message received...")
    except KeyboardInterrupt:
        pass
    bus.shutdown()


if __name__ == "__main__":
    main()
