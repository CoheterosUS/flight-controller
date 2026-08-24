import serial
import sys

HEADER_LSB = 0xFE
HEADER_MSB = 0xCA
FOOTER = 0xBE

COMMANDS = {
    "reset": 0x01,
    "abort": 0x02,
    "calibrate": 0x03,
    "drogue": 0x04,
}


def build_packet(command_id):
    return bytes([HEADER_LSB, HEADER_MSB, command_id, 0x00, FOOTER])


def main():
    if len(sys.argv) < 3:
        print(f"Usage: python {sys.argv[0]} <COM_PORT> <command>")
        print(f"Commands: {', '.join(COMMANDS.keys())}")
        sys.exit(1)

    port = sys.argv[1]
    command_name = sys.argv[2].lower()

    if command_name not in COMMANDS:
        print(f"Unknown command: {command_name}")
        print(f"Available: {', '.join(COMMANDS.keys())}")
        sys.exit(1)

    packet = build_packet(COMMANDS[command_name])

    with serial.Serial(port, 115200, timeout=1) as ser:
        ser.write(packet)
        print(f"Sent {command_name} -> {packet.hex(' ')}")


if __name__ == "__main__":
    main()
