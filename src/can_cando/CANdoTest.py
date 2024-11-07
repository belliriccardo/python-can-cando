import logging
import sys
from pathlib import Path

from can import Message

sys.path.append(Path(__file__).resolve().parents[1].as_posix())


from can_cando import CANdoBus
from can_cando.CANdo import log

log.setLevel(logging.DEBUG)


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
    except Exception:
        log.exception("Exiting...")
    bus.shutdown()


if __name__ == "__main__":
    main()
