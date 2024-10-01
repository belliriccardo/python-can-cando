import logging
import threading
from ctypes import byref, c_int, c_ubyte, pointer
from queue import Empty, Queue
from time import sleep, time
from typing import Any, List, Optional, Tuple

from can import BusABC, Message
from can.typechecking import AutoDetectedConfig, CanFilters
from CANdoImport import (
    CAN_MSG_TIMESTAMP_RESOLUTION,
    CANDO_CAN_BUFFER_LENGTH,
    CANDO_DEVICE_STATUS,
    CANDO_ID_11_BIT,
    CANDO_ID_29_BIT,
    CANDO_LISTEN_ONLY_MODE,
    CANDO_LOOPBACK_MODE,
    CANDO_NO_STATUS,
    CANDO_NORMAL_MODE,
    CANDO_RUN,
    CANDO_STOP,
    CANDO_SUCCESS,
    CANDOISO_PID,
    CANdoCANBufferPtrType,
    CANdoClearStatus,
    CANdoClose,
    CANdoFlushBuffers,
    CANdoGetDevices,
    CANdoGetPID,
    CANdoGetVersion,
    CANdoOpenDevice,
    CANdoPIDType,
    CANdoReceive,
    CANdoRequestStatus,
    CANdoSetBaudRate,
    CANdoSetFilters,
    CANdoSetMode,
    CANdoSetState,
    CANdoStatusPtrType,
    CANdoTransmit,
    CANdoUSBPtrType,
    MsgQueueType,
    TCANdoCANBuffer,
    TCANdoDevice,
    TCANdoDeviceString,
    TCANdoStatus,
    TCANdoUSB,
    candoiso_get_timing,
)

log = logging.getLogger("can.CANDoISO")
log.addHandler(logging.StreamHandler())
log.setLevel(logging.DEBUG)


class CANDoISO(BusABC):
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

    def __init__(
        self,
        channel: Optional[int] = None,
        can_filters: Optional[CanFilters] = None,
        bitrate: Optional[int] = default_bitrate,
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

        log.debug(
            f"CANdoGetVersion >\n  API = v{APIVersion.value / 10}\n  "
            f"DLL = v{DLLVersion.value / 10}\n  "
            f"Driver = v{DriverVersion.value / 10}\n",
        )

        self._available_configs = self._detect_cando_iso()

        if channel is None:
            # Channel discovery mode
            return

        if self.NoOfDevices.value == 0:
            raise ValueError("No CANdoISO devices detected!")

        if channel < 0:
            raise ValueError(f"Invalid channel number: {channel}, must be >= 0")

        if channel >= self.NoOfDevices.value:
            raise ValueError(
                f"Invalid channel number: {channel}, "
                f"only {self.NoOfDevices.value} devices detected and values start at 0",
            )

        self.bit_timing = candoiso_get_timing(bitrate or self.default_bitrate)

        # CANdoOpen function
        self.CANdoUSB = TCANdoUSB()  # Create an instance of TCANdoUSB
        self.CANdoUSBPtr = pointer(self.CANdoUSB)
        self.CANdoCANBuffer = TCANdoCANBuffer()
        self.CANdoCANBufferPtr = pointer(self.CANdoCANBuffer)
        self.CANdoStatus = TCANdoStatus()
        self.CANdoStatusPtr = pointer(self.CANdoStatus)

        self.open_device(channel)

        if not self.CANdoUSB.OpenFlag:
            raise ValueError("Device not open!")

        self.set_timings()
        # Normal mode: Rx/Tx CAN mode
        self.set_mode(CANDO_NORMAL_MODE)
        sleep(0.01)  # Sleep for 10ms

        self.set_filters(can_filters)
        sleep(0.01)  # Sleep for 10ms

        self.flush_buffers()
        sleep(0.01)  # Sleep for 10ms

        self.clear_status()
        self.set_state(CANDO_RUN)
        sleep(0.01)  # Sleep for 10ms
        self.print_status()

        self.t_start = time()

        self.msg_queue: MsgQueueType = Queue()

        self._is_shutdown = False

        self._recv_thread = threading.Thread(target=self._recv_t, daemon=True)
        self._recv_thread.start()

        log.debug("CANdoISO configured and started successfully!")

    def open_device(self, channel: int) -> None:
        # Open a connection to the selected device
        if CANdoOpenDevice(self.CANdoUSBPtr, byref(self.CANdoDevices[channel])) == CANDO_SUCCESS:
            log.debug(
                f"Connection open to {self.CANdoUSB.Description.decode('utf-8')}, "
                f"S/N {self.CANdoUSB.SerialNo.decode('utf-8')}",
            )
        else:
            raise ValueError("Device unavailable")

    def set_timings(self) -> None:
        # CANdo connection is open, so configure for a CAN baud rate of 250k & set the filters to receive all messages
        # SJW - sync. jump width, 0 - 3. (This is the width of the synchronisation jump used by the CAN module to
        # achieve synchronisation. 0 = 1 jump bit … 3 = 4 jump bits)
        # BRP - baud rate prescaler, 0 - 63 for a CANdo device, 0 - 31 & 64 - 127 for a CANdoISO or CANdo AUTO device
        # (See description below)
        # PHSEG1 - phase segment 1, 0 - 7 (See description below)
        # PHSEG2 - phase segment 2, 0 - 7 (See description below)
        # PROPSEG - propagation segment, 0 - 7 (See description below)
        # SAM - number of samples per data bit, 0 - 1 (0 = Sample each data bit once, 1 = Sample each data bit thrice)
        if (
            CANdoSetBaudRate(
                self.CANdoUSBPtr,
                self.bit_timing.sjw,
                self.bit_timing.brp,
                self.bit_timing.phseg1,
                self.bit_timing.phseg2,
                self.bit_timing.propseg,
                self.bit_timing.sam,
            )
            != CANDO_SUCCESS
        ):
            raise ValueError("Failed to set baud rate!")

        sleep(0.01)  # Sleep for 10ms

    def set_mode(self, mode: int) -> None:
        if mode not in (
            CANDO_NORMAL_MODE,
            CANDO_LISTEN_ONLY_MODE,
            CANDO_LOOPBACK_MODE,
        ):
            raise ValueError(f"Invalid mode: {mode}")

        if CANdoSetMode(self.CANdoUSBPtr, mode) != CANDO_SUCCESS:
            raise ValueError(f"Failed to set mode: {mode}")

        sleep(0.01)  # Sleep for 10ms

    def flush_buffers(self) -> None:
        if not self.CANdoUSB.OpenFlag:
            raise ValueError("Device not open!")

        if CANdoFlushBuffers(self.CANdoUSBPtr) != CANDO_SUCCESS:
            log.error("Failed to flush buffers!")

    def set_filters(self, filters: Optional[CanFilters] = None) -> None:
        log.warning("Filters are not implemented yet, all messages will be received")
        # CANdoUSBPointer - pointer to TCANdoUSB structure
        # Rx1Mask - receive buffer 1 mask
        # Rx1IDE1 - flag to indicate Rx1Mask & Rx1Filter1 values are either 11 or 29 bit values (0 = 11 bit ID, 1 = 29 bit ID)
        # Rx1Filter1 - receive buffer 1, filter 1 (See Appendix B for further details)
        # Rx1IDE2 - flag to indicate Rx1Filter2 value is either an 11 or 29 bit value (0 = 11 bit ID, 1 = 29 bit ID)
        # Rx1Filter2 - receive buffer 1, filter 2 (See Appendix B for further details)
        # Rx2Mask - receive buffer 2 mask
        # Rx2IDE1 - flag to indicate Rx2Mask & Rx2Filter1 values are either 11 or 29 bit values (0 = 11 bit ID, 1 = 29 bit ID)
        # Rx2Filter1 - receive buffer 2, filter 1 (See Appendix B for further details)
        # Rx2IDE2 - flag to indicate Rx2Filter2 value is either an 11 or 29 bit value (0 = 11 bit ID, 1 = 29 bit ID)
        # Rx2Filter2 - receive buffer 2, filter 2 (See Appendix B for further details)
        # Rx2IDE3 - flag to indicate Rx2Filter3 value is either an 11 or 29 bit value (0 = 11 bit ID, 1 = 29 bit ID)
        # Rx2Filter3 - receive buffer 2, filter 3 (See Appendix B for further details)
        # Rx2IDE4 - flag to indicate Rx2Filter4 value is either an 11 or 29 bit value (0 = 11 bit ID, 1 = 29 bit ID)
        # Rx2Filter4 - receive buffer 2, filter 4 (See Appendix B for further details)
        for can_filter in filters or []:
            can_id = can_filter["can_id"]
            can_mask = can_filter["can_mask"]
            extended = int(can_filter.get("extended", False))
            if (
                CANdoSetFilters(
                    self.CANdoUSBPtr,
                    0,
                    CANDO_ID_29_BIT,
                    0,
                    CANDO_ID_11_BIT,
                    0,
                    0,
                    CANDO_ID_29_BIT,
                    0,
                    CANDO_ID_11_BIT,
                    0,
                    CANDO_ID_29_BIT,
                    0,
                    CANDO_ID_11_BIT,
                    0,
                )
                != CANDO_SUCCESS
            ):
                raise ValueError(f"Failed to set filter: {can_filter}")

    def get_status_description(self) -> str:
        sts = "Status unknown"
        # if CANdoRequestStatus(self.CANdoUSBPtr) == CANDO_SUCCESS:
        if CANdoRequestStatus(pointer(self.CANdoUSB)) == CANDO_SUCCESS:
            # Wait for device to reply
            sleep(0.01)  # Sleep for 10ms
            self.CANdoStatus.NewFlag = CANDO_NO_STATUS  # Clear status flag

            # Receive status message
            if (
                CANdoReceive(
                    self.CANdoUSBPtr,
                    self.CANdoCANBufferPtr,
                    self.CANdoStatusPtr,
                )
                == CANDO_SUCCESS
            ) and self.CANdoStatus.NewFlag == CANDO_DEVICE_STATUS:  # type: ignore
                # Display new device status
                sts = "  {} S/N {} H/W v{}, S/W v{}, Status = {}, Bus State = {}".format(
                    self.CANdoUSB.Description.decode("utf-8"),
                    self.CANdoUSB.SerialNo.decode("utf-8"),
                    self.CANdoStatus.HardwareVersion / 10,
                    self.CANdoStatus.SoftwareVersion / 10,
                    self.CANdoStatus.Status,
                    self.CANdoStatus.BusState,
                )

        return sts

    def print_status(self) -> None:
        log.debug(self.get_status_description())

    def clear_status(self) -> None:
        CANdoClearStatus(self.CANdoUSBPtr)

    def set_state(self, state: int) -> None:
        if state not in (CANDO_STOP, CANDO_RUN):
            raise ValueError(f"Invalid state: {state}")

        if CANdoSetState(self.CANdoUSBPtr, state) != CANDO_SUCCESS:
            raise ValueError(f"Failed to set state to {state}!")

    def _detect_cando_iso(self) -> None:
        # CANdoGetDevices & CANdoGetPID functions
        self.NoOfDevices = c_int(5)  # Look for up to 5 CANdo devices
        # Create an array type of TCANdoDevice for those devices
        self.TCANdoDevices = TCANdoDevice * self.NoOfDevices.value
        self.CANdoDevices = self.TCANdoDevices()  # Create an instance of TCANdoDevices array
        # Get a pointer to the TCANdoDevices array
        self.PCANdoDevices = pointer(self.CANdoDevices)
        self.CANdoPID = TCANdoDeviceString()
        if CANdoGetDevices(self.PCANdoDevices, byref(self.NoOfDevices)) == CANDO_SUCCESS:
            for CANdoNo in range(self.NoOfDevices.value):
                if CANdoGetPID(CANdoNo, byref(self.CANdoPID)) == CANDO_SUCCESS:
                    # PID
                    log.debug(f"  Device no. {CANdoNo} >\n    PID = 0x{self.CANdoPID.value.decode('utf-8').upper()}")
                    if self.CANdoPID.value != CANDOISO_PID:
                        log.warning(f"    Device no. {CANdoNo} is not a CANdoISO device!")

                    # H/W type & S/N
                    log.debug(
                        f"    H/W type = {self.CANdoDevices[CANdoNo].HardwareType}\n    "
                        f"S/N = {self.CANdoDevices[CANdoNo].SerialNo.decode('utf-8')}\n",
                    )

    @staticmethod
    def _detect_available_configs() -> List[AutoDetectedConfig]:
        interfaces: List[AutoDetectedConfig] = []

        bus = CANDoISO(None)
        bus._detect_cando_iso()  # noqa: SLF001
        if bus.NoOfDevices.value > 0:
            interfaces = [AutoDetectedConfig(interface="candoiso", channel=c) for c in range(bus.NoOfDevices.value)]

        del bus

        return interfaces

    def shutdown(self) -> None:
        super().shutdown()
        self._is_shutdown = True
        self._recv_thread.join()

        # CANdo connection is open, so close
        if not (self.CANdoUSB.OpenFlag and CANdoClose(self.CANdoUSBPtr) == CANDO_SUCCESS):
            log.error("Failed to close connection!")
        else:
            log.debug("Connection closed successfully.")

    def send(self, msg: Message, timeout: Optional[float] = None) -> None:
        if not self.CANdoUSB.OpenFlag:
            raise ValueError("Device not open!")

        DataArray = c_ubyte * msg.dlc  # Create a data array type
        Data = DataArray(*msg.data)  # Create & initialise an instance of the data array

        tx_res = CANdoTransmit(
            self.CANdoUSBPtr,  # Device handle pointer
            int(msg.is_extended_id),  # Extended ID flag
            msg.arbitration_id,  # CAN ID
            int(msg.is_remote_frame),
            msg.dlc,  # Message's dlc
            Data,
            # Buffer No - buffer within CANdo to use for the transmission, 0 - 15.
            # Buffer 0 transmits directly to the CAN bus.
            # Buffers 1 - 15 are specialised repeat buffers that allow accurate timed
            # transmissions of repetitive messages.
            0,
            # RepeatTime - the message transmit repeat time, 0 - 10.
            # (Only applies to the repeat buffers 1 - 15, set to 0 for buffer 0)
            0,
        )

        if tx_res == CANDO_SUCCESS:
            pass
            # log.debug("Message transmitted successfully")

    def _recv_t(self) -> None:
        # Local copy to avoid global lookups
        ts_resolution = CAN_MSG_TIMESTAMP_RESOLUTION

        while not self._is_shutdown:
            # Clear status flag
            self.CANdoStatus.NewFlag = CANDO_NO_STATUS

            # Receive CAN messages
            if CANdoReceive(self.CANdoUSBPtr, self.CANdoCANBufferPtr, self.CANdoStatusPtr) != CANDO_SUCCESS:
                raise ValueError("Error receiving messages")

            read_idx, write_idx, full_flag = (
                self.CANdoCANBuffer.ReadIndex,
                self.CANdoCANBuffer.WriteIndex,
                self.CANdoCANBuffer.FullFlag,
            )

            # print(
            #     f"Status: {CANdoStsErr(self.CANdoStatus.Status)}, "
            #     f"Bus State: {CANdoBusStsErr(self.CANdoStatus.BusState)}",
            # )

            if read_idx == write_idx and not full_flag:
                # No messages were received, sleep for a short time
                sleep(0.005)
                continue

            # CANdoReceive success
            while read_idx != write_idx or full_flag:
                arbitration_id = self.CANdoCANBuffer.CANMessage[read_idx].ID
                ide = self.CANdoCANBuffer.CANMessage[read_idx].IDE
                rtr = self.CANdoCANBuffer.CANMessage[read_idx].RTR
                dlc = self.CANdoCANBuffer.CANMessage[read_idx].DLC
                data = bytearray(self.CANdoCANBuffer.CANMessage[read_idx].Data[:dlc])
                # bus_state = self.CANdoCANBuffer.CANMessage[read_idx].BusState  # Unused for now

                # Timestamp - a timestamp indicating the receive time of the message since starting CANdo.
                # The timestamp resolution is 25.6us per bit.
                timestamp = (self.CANdoCANBuffer.CANMessage[read_idx].TimeStamp * ts_resolution) + self.t_start

                if (read_idx + 1) < CANDO_CAN_BUFFER_LENGTH:
                    # Increment index onto next slot
                    read_idx += 1
                else:
                    read_idx = 0

                self.CANdoCANBuffer.FullFlag = False  # No longer full

                msg = Message(
                    timestamp=timestamp,
                    arbitration_id=arbitration_id,
                    is_extended_id=bool(ide),
                    is_remote_frame=rtr,
                    dlc=dlc,
                    data=data,
                )
                self.msg_queue.put(msg)

                if self.CANdoStatus.NewFlag == CANDO_DEVICE_STATUS:  # type: ignore
                    # Display new device status
                    # log.debug("  Status = {0}, Bus State = {1}".format(self.CANdoStatus.Status, self.CANdoStatus.BusState))  # noqa: E501
                    self.CANdoStatus.NewFlag = CANDO_NO_STATUS  # Clear status flag

            self.CANdoCANBuffer.ReadIndex = read_idx

    def _recv_internal(self, timeout: Optional[float]) -> Tuple[Optional[Message], bool]:
        try:
            return (self.msg_queue.get(timeout=timeout), True)
        except Empty:
            return (None, False)
        except Exception:
            log.exception("Error receiving message!")
            raise


def main() -> None:
    interfaces = CANDoISO._detect_available_configs()  # noqa: SLF001  #type: ignore

    if len(interfaces) == 0:
        print("No CANdoISO devices detected!")
        return

    print("Available interfaces:")
    for i in interfaces:
        print(f"  {i}")
    selected = int(input("Select interface: "))

    bus = CANDoISO(selected)

    try:
        while True:
            _in = input(
                "Press 't' to transmit a message, 'r' to receive a message,"
                "'s' to see the device's status and 'q' or 'Ctrl-C' to quit: ",
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
