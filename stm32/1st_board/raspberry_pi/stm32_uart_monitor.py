#!/usr/bin/env python3
"""
STM32 NUCLEO <-> Raspberry Pi USB UART monitor.

STM32 sends 'S' on button press; this script logs RX and sends 'O' or 'N'.
"""

import argparse
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    print("pyserial is required: pip3 install pyserial")
    sys.exit(1)

CMD_START = b"S"
RSP_OK = b"O"
RSP_NG = b"N"


def log(message: str) -> None:
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{timestamp}] {message}", flush=True)


def format_byte(data: bytes) -> str:
    if len(data) != 1:
        return repr(data)
    value = data[0]
    char = chr(value) if 32 <= value < 127 else "."
    return f"'{char}' (0x{value:02X})"


def choose_response(mode: str) -> bytes:
    if mode == "O":
        return RSP_OK
    if mode == "N":
        return RSP_NG

    while True:
        choice = input("Respond with O or N: ").strip().upper()
        if choice == "O":
            return RSP_OK
        if choice == "N":
            return RSP_NG
        print("Invalid input. Enter O or N.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Monitor STM32 UART traffic on Raspberry Pi"
    )
    parser.add_argument(
        "-p",
        "--port",
        default="/dev/ttyACM0",
        help="Serial port (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "-r",
        "--response",
        choices=["O", "N", "ask"],
        default="O",
        help="Reply mode: fixed O, fixed N, or ask on each event",
    )
    args = parser.parse_args()

    log(f"Opening {args.port} at {args.baud} baud")
    log("Waiting for STM32 button press (RX: 'S')")
    log("Press Ctrl+C to stop")

    try:
        with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
            ser.reset_input_buffer()

            while True:
                data = ser.read(1)
                if not data:
                    continue

                if data == CMD_START:
                    log(f"RX << {format_byte(data)}")
                    reply = choose_response(args.response)
                    ser.write(reply)
                    ser.flush()
                    log(f"TX >> {format_byte(reply)}")
                else:
                    log(f"RX << unexpected {format_byte(data)}")

    except serial.SerialException as exc:
        log(f"Serial error: {exc}")
        log("Check USB cable, port name, and dialout group permission")
        return 1
    except KeyboardInterrupt:
        log("Stopped")
        return 0


if __name__ == "__main__":
    sys.exit(main())
