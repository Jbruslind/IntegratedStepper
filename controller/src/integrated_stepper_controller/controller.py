import ctypes
import logging
import sys
import time

import can

# CLOWN WORLD
logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger(__name__)

class Header(ctypes.BigEndianStructure):
    _pack_ = 1
    _fields_ = [
        ('_pad',        ctypes.c_uint8,   4),
        ('data_type',   ctypes.c_uint8,   4),
        ('command',     ctypes.c_uint8     ),
        ('motor_id',    ctypes.c_uint16,  4),
        ('bus_node_id', ctypes.c_uint16, 12),
    ]
    def __repr__(self) -> str:
        return (f'<Header: bus_node_id={self.bus_node_id:03x} '
                f'motor_id={self.motor_id:02x} '
                f'command={self.command:02x} '
                f'data_type={self.data_type:x}>')

UIDType = 6 * ctypes.c_byte
VersionType = 2 * ctypes.c_byte

class UIDVersionPayload(ctypes.BigEndianStructure):
    _pack_ = 1
    _fields_ = [
        ('uid',     UIDType    ),
        ('version', VersionType),
    ]

class Controller(can.Listener):
    def __init__(self, bus: can.BusABC) -> None:
        self.bus = bus
        self.next_bus_id = 69

    def on_message_received(self, msg: can.Message) -> None:
        raw_hdr = msg.arbitration_id.to_bytes(length=4, byteorder='big')
        hdr = Header.from_buffer_copy(raw_hdr)

        log.debug('received message: %s, payload=%s', hdr, msg.data.hex())

        match hdr.command:
            case 0xf1:    # CMD_CUST_REQ_BUS
    # which is less overhead????
                resp_hdr = Header(
                    data_type=8,
                    command=0xf2, # CMD_CUST_BUS_ID
                    motor_id=0,
                    bus_node_id=self.next_bus_id,
                )
                aid = int.from_bytes(bytes(resp_hdr), byteorder='big')
    # or (data_type << 24) | (command << 16) | (motor_id << 12) | bus_node_id
    # and that's why we use C
                m = can.Message(arbitration_id=aid, data=msg.data)
                log.debug('sending message: %s, payload=%s',
                          resp_hdr, m.data.hex())
                self.bus.send(m)

                self.next_bus_id += 1

invocation = 'controller'

def main() -> None:
    if len(sys.argv) != 3:
        print(f'usage: {invocation} CAN_IFACE BPS')
        sys.exit(1)

    channel = sys.argv[1]
    bitrate = int(sys.argv[2])

    # sudo ip link set can0 up type can bitrate 1000000
    log.debug('attempting connection to SocketCAN interface %s at %s bps',
              channel, bitrate)
    with (can.Bus(interface='socketcan', channel=channel, bitrate=bitrate)
          as bus):
        controller = Controller(bus)
        can.Notifier(bus, [controller])

        # not really sure how kernel deals with open CAN sockets on termination
        # ctrl-C at ur own risk
        time.sleep(120.0)
