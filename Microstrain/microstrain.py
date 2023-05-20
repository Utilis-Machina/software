r"""Library for LORD Microstrain 3DM-GX4-45 navigation unit.

This library provides a software interface built around the MIP interface of the
device. It has not been tested against the current generation of devices, but
would likely work since the structure is the mostly the same.

This serial library communicates using the MIP packet format, which consits of
header, payload, and checksum. The header defines which type of interaction is
happening - either general commands, IMU, GPS, or EKF data. The payload is
divided into fields, that also contain a descriptor to explain what kind of data
is included. The classes were designed to make it easy to translate between the
documentation and code.

Helper dataclasses are introduced:
  MipsField: are strung together as a list and packed into the payload portion
    of the packet.
  MipsPacket: the top level container that describes the payload and holds all
    the data.
  ReplyFormats: provide a mapping between packets and the data to make it easier
    to decode.
"""
import collections
from dataclasses import dataclass
import itertools
import serial
import struct
import time
from typing import Optional

# The sync headers are always the same.
SYNC1 = 0x75
SYNC2 = 0x65
SYNC_MSG = bytearray([SYNC1, SYNC2])

def fletcher_checksum(packet: bytes) -> bytes:
    """Calculates fletcher checksum for bytes in packet."""
    check_sum = [sum(packet), sum(itertools.accumulate(packet))]
    return bytearray([0xFF & check_sum[0], 0xFF & check_sum[1]])

@dataclass
class MipsField:
    """Container that represents the payload data in the MipsPacket.
    
    For convenience, the data held in the MipsField is represented in both byte
    and as a list of integers. This gives some flexibility in how it is used in
    the code.

    Attributes:
      field_len: the length of the field in bytes.
      field_desc: the field descriptor explaining the data contents.
      data: the data of the field as a list of integers.
      data_bytes: the data of the field in byte format.
    """
    field_len: int
    field_desc: int
    data: Optional[list[int]] = None
    data_bytes: Optional[bytes] = None

    def __init__(self, field_len: int, field_desc: int,
                 data: Optional[list[int]] = None,
                 data_bytes: Optional[bytes] = None):
        self.field_len = field_len
        self.field_desc = field_desc
        if data is not None:
            # Make sure it's a bit array for consistency with read messages.
            self.data = list(data)
            self.data_bytes = bytearray(data)
        elif data_bytes is not None:
            self.data_bytes = data_bytes
            self.data = [i for i in data_bytes]
        else:
            self.data = []
            self.data_bytes = b''

    def __len__(self):
        """Returns length of field data."""
        data_len = len(self.data) if self.data else 0
        return 2 + data_len
    
    def __iter__(self):
        """Returns the contents of class."""
        content = [self.field_len, self.field_desc]
        if self.data:
            content += self.data
        return iter(content)
    
    @property
    def as_bytes(self) -> bytes:
        field_bytes = bytearray([self.field_len, self.field_desc])
        field_bytes += self.data_bytes
        return field_bytes

@dataclass
class MipsPacket:
    """Container for communication with the device.
    
    This class is responsible for properly composing the messages to send to the
    device. It stores the essential information only, skipping always needed
    items like the SYNC and checksum components.

    Attributes:
      desc_set: the descriptor set byte, explaining which message catalog is
        being sent to the device.
      payload: a list of MipsField objects, which contain the data to send or
        what was received from the device.
    """
    desc_set: int
    payload: list[MipsField]

    def __init__(self, desc_set: int, payload: list[MipsField]):
        self.desc_set = desc_set
        self.payload = payload
        self.payload_len = sum([len(f) for f in self.payload])

    @property
    def as_bytes(self) -> bytes:
        """Returns the full packet info as bytes."""
        field_bytes = []
        for entry in self.payload:
            field_bytes.extend(entry)
        packet = bytearray([SYNC1, SYNC2, self.desc_set, self.payload_len]
                           + field_bytes)
        packet += fletcher_checksum(packet)
        return packet

class ReplyFormats:
    """Container of messages and mapping to formats for unpacking."""
    # The ACK/NACK message consists of a command echo and error code.
    # This comes back when the field description is 0xf1.
    Ack = collections.namedtuple('Ack', ['cmd_echo', 'error_code'])

    # The device information, note there are lots of spaces in these
    # fields.
    DevInfo = collections.namedtuple('DevInfo',[
        'fw_ver', 'model_name', 'model_number', 'serial_num', 'lot_num',
        'dev_opts'])

    byte_formats = {
        0x81: ('<H16s16s16s16s16s', DevInfo),
        0x83: ('<I', int),
        0xf1: ('<BB', Ack),
    }

    @classmethod
    def process_field(self, mips_field: MipsField) -> tuple[any,...]:
        """Returns processed bytes according to field description."""
        fmt, obj = self.byte_formats[mips_field.field_desc]
        return obj(*struct.unpack(fmt, mips_field.data_bytes)) 


class Microstrain3DM:
    """An interface for Microstrain 3D motion devices."""
    
    def __init__(self, port: str ='COM5', baudrate: int = 115200):
        self._ser = serial.Serial(port=port, baudrate=baudrate)
        # These are for debugging the raw traffic.
        self._read_buffer = collections.deque(maxlen=10)
        self._write_buffer = collections.deque(maxlen=10)

    def _split_payload_to_fields(self, payload: bytes) -> list[MipsField]:
        """Takes payload bytes and divides them into fields."""
        index = 0
        last_byte = len(payload)
        field_data = []
        while index < last_byte:
            field_len = payload[index]
            field_desc = payload[index + 1]
            data = payload[index + 2:index + field_len]
            field_data.append(MipsField(field_len, field_desc, data_bytes=data))
            index += field_len
        return field_data
    
    def _read_one_packet(self) -> MipsPacket:
        """Returns the first MIP packet found on the serial connection."""
        self._ser.read_until(SYNC_MSG)
        desc_set = self._ser.read(1)
        payload_len = self._ser.read(1)
        payload = self._ser.read(int.from_bytes(payload_len, 'little'))
        checksum = self._ser.read(2)
        self._read_buffer.append(desc_set + payload_len + payload + checksum)
        field_data = self._split_payload_to_fields(payload)
        packet = MipsPacket(int.from_bytes(desc_set, 'little'), field_data)
        return packet
    
    def _write(self, value: bytes):
        """Writes data to device and buffers for debug purposes."""
        self._ser.write(value)
        self._write_buffer.append(value)

    def _send_command(self, packet: MipsPacket) -> MipsPacket:
        """Sends command to device and provides response."""
        self._write(packet.as_bytes)
        time.sleep(0.001)
        return self._read_one_packet()
    
    def _send_and_parse_reply(self, packet: MipsPacket
                              ) -> list[collections.namedtuple]:
        """Sends command and returns device reply object."""
        response = self._send_command(packet)
        print(f'Received: {response}')
        parsed_fields = []
        for field in response.payload:
            parsed_fields.append(ReplyFormats.process_field(field))
        return parsed_fields

    def dump_read_buffer(self) -> collections.deque:
        return self._read_buffer

    def dump_write_buffer(self) -> collections.deque:
        return self._write_buffer
    
    def device_ping(self) -> collections.namedtuple:
        """Sends ping command to device and returns ack message."""
        data = self._send_and_parse_reply(
            MipsPacket(0x01, [MipsField(0x02, 0x01)]))
        return data[0]
    
    def device_reset(self) -> collections.namedtuple:
        """Resets device and returns ack message."""
        data = self._send_and_parse_reply(
            MipsPacket(0x01, [MipsField(0x02, 0x7e)]))
        return data[0]
    
    def device_idle(self) -> collections.namedtuple:
        """Sets device into idle mode, which interrupts streaming or sleep."""
        data = self._send_and_parse_reply(
            MipsPacket(0x01, [MipsField(0x02, 0x02)]))
        return data[0]
    
    def device_resume(self) -> collections.namedtuple:
        """Sets device into previous or default mode."""
        data = self._send_and_parse_reply(
            MipsPacket(0x01, [MipsField(0x02, 0x06)]))
        return data[0]
    
    def device_info(self) -> collections.namedtuple:
        """Returns information for device."""
        data = self._send_and_parse_reply(
            MipsPacket(0x01, [MipsField(0x02, 0x03)]))
        return data[1]
    
    def device_built_in_test(self) -> int:
        """Runs built-in-test and returns results integer.
        
        For a properly functioning device we expect the result to be zero. The
        other potential options are (Byte#, bit#):
          Byte 1, Processor Board
            b1: WDT Reset
          Byte 2, Sensor Board
            b1: IMU communication fault
            b2: Magnetometer fault
            b3: Pressure sensor fault
          Byte 3, GPS
            b1: Power fault
            b2: Communication Fault
            b3: Solution Fault
          Byte 4, Kalman Filter
            b1: Solution fault
        """
        data = self._send_and_parse_reply(
            MipsPacket(0x01,[MipsField(0x02, 0x05)]))
        return data[1]
