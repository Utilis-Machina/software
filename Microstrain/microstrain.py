r"""Library for LORD Microstrain 3DM-GX4-45 navigation unit.

This library provides a software interface built around the MIP protocol of the
device. It has not been tested against the current generation of devices, but
would likely work since the structure is the mostly the same.

This serial library communicates using the MIP packet format, which consists of
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

The device shows up under ttyACM*, so the following udev rule is suggested to
be added to 88-utilis.rules to make a specific device name on linux (without
the line break):
ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", 
  ATTRS{product}=="Lord Inertial Sensor", MODE="0660", SYMLINK+="ustrain"

Usage example:
unit = Microstrain3DM()
# Collect scaled accelerometer data at 250 Hz (500 Hz max).
unit.set_msg_fmt(Datamessages.IMU, descriptors=[0x04], rate_hz=[250.])
data = unit.collect_data_stream(10.)  # Start streaming data for 10 seconds.

# The data can be processed locally, or written out to csv with:
unit.write_stream_data()
"""
import enum
import collections
from dataclasses import dataclass
import itertools
import serial
import struct
import time
from typing import Any, Optional
import os

# The sync headers are always the same, and indicate the start of a packet.
SYNC1 = 0x75
SYNC2 = 0x65
SYNC_MSG = bytearray([SYNC1, SYNC2])

class MessageFormatSelectors(enum.Enum):
    """These are the possible options when setting up a data message format."""
    NEW = 1
    READ_BACK = 2
    SAVE = 3
    LOAD = 4
    RESET = 5

class DataMessages(enum.Enum):
    """Small class to bundle interactions with the different message formats.
    
    Attributes:
      msg_ind: the packet descriptor for a message of that type.
      fmt_cmd: the field descriptor portion of the format command.
      poll_cmd: the field desecriptor portion of the poll command.  
    """
    IMU = (0x80, 0x08, 0x01)
    GPS = (0x81, 0x09, 0x02)
    EKF = (0x82, 0x0a, 0x03)

    def __init__(self, msg_ind, fmt_cmd, poll_cmd):
        self.msg_ind = msg_ind
        self.fmt_cmd = fmt_cmd
        self.poll_cmd = poll_cmd

def fletcher_checksum(packet: bytes) -> bytes:
    """Calculates fletcher checksum for bytes in packet."""
    check_sum = [sum(packet), sum(itertools.accumulate(packet))]
    return bytearray([0xff & check_sum[0], 0xff & check_sum[1]])

@dataclass
class MipsField:
    """Container that represents the payload data in the MipsPacket.
    
    For convenience, the data held in the MipsField is represented in both byte
    and as a list of integers. This gives some flexibility in how it is used in
    the code, and makes it easier to reference the documentation.

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
        # Ensure we load the data from one source, and make sure the alternate
        # form is consistent.
        if data is not None:
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
        # Additional 2 for field length and descriptor byte.
        return 2 + data_len
    
    def __iter__(self):
        """Returns the contents of class (used for unpacking)."""
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
      recv_time: a field that can be used to store time a streamed packet
        arrived, used for decoding and writing out messages.
    """
    desc_set: int
    payload: list[MipsField]
    recv_time: Optional[float] = None

    def __init__(self, desc_set: int, payload: list[MipsField]):
        self.desc_set = desc_set
        self.payload = payload
        self.payload_len = sum([len(f) for f in self.payload])

    @property
    def as_bytes(self) -> bytes:
        """Returns the full packet info as bytes (used to send to device)."""
        field_bytes = []
        for entry in self.payload:
            field_bytes.extend(entry)
        packet = bytearray([SYNC1, SYNC2, self.desc_set, self.payload_len]
                           + field_bytes)
        packet += fletcher_checksum(packet)
        return packet
    
    def convert_payload(self) -> list[collections.namedtuple]:
        """Converts payload fields to their respective objects."""
        entries = []
        for f in self.payload:
            value = ReplyFormats.process_field(self.desc_set, f)
            entries.append(value)
        return entries
    
    def flatten_payload(self) -> list[Any]:
        """Flattens values in payload fields into a list."""
        processed_fields = self.convert_payload()
        return list(itertools.chain(*processed_fields))


class ReplyFormats:
    """Container of messages and mapping to formats for unpacking.
    
    This class contains a number of helper classes introduced to make it more
    obvious what's in the data received from the device. It maps the format
    to the struct unpacking instructions, as well as additional helpful info
    like the units or info of the terms.

    This class is intended to operate on individual fields, and contains
    class methods to:
        - Unpack a field's byte data into an object
        - Retrieve units for the field
    """
    # The ACK/NACK message consists of a command echo and error code.
    # This comes back when the field description is 0xf1.
    Ack = collections.namedtuple('Ack', ['cmd_echo', 'error_code'])

    # The device information, note there are lots of spaces in these
    # fields.
    DevInfo = collections.namedtuple('DevInfo',[
        'fw_ver', 'model_name', 'model_number', 'serial_num', 'lot_num',
        'dev_opts'])
    
    DataFmt = collections.namedtuple('DataFmt',
                                     ['descriptor', 'rate_decimation'])

    # Each set of information stores the format, class, and units of the data.
    base_set = {
        0x81: ('>H16s16s16s16s16s', DevInfo),  # Device info.
        0x83: ('>I', int),  # Built-in self test.
        0xf1: ('>BB', Ack),  # (N)Ack message.
    }

    Vector = collections.namedtuple('Vector', ['x', 'y', 'z'])
    Quat = collections.namedtuple('Quat', ['q0', 'q1', 'q2', 'q3'])
    Euler = collections.namedtuple('Euler', ['roll', 'pitch', 'yaw'])
    Matrix = collections.namedtuple('Matrix', ['M11', 'M12', 'M13',
                                               'M21', 'M22', 'M23',
                                               'M31', 'M32', 'M33'])
    Timestamp = collections.namedtuple('GpsTimestamp',
                                       ['time_of_week', 'week_num', 'flags'])
    AmbPressure = collections.namedtuple('AmbPressure', ['pressure'])

    imu_set = {
        0x04: ('>fff', Vector),  # Accelerometer vector.
        0x05: ('>fff', Vector),  # Gyro vector.
        0x06: ('>fff', Vector),  # Magnetometer vector.
        0x07: ('>fff', Vector),  # Delta theta vector.
        0x08: ('>fff', Vector),  # Delta velocity vector.
        0x09: ('>fffffffff', Matrix),  # CF orientation matrix.
        0x0a: ('>ffff', Quat),  # Complimentary filter (CF) quaternion.
        0x0c: ('>fff', Euler),  # CF Euler Angles.
        0x10: ('>fff', Vector),  # CF stabilized mag vector.
        0x11: ('>fff', Vector),  # CF stabilized accel vector.
        0x12: ('>dHH', Timestamp),  # GPS correlation timestamp.
        0x17: ('>f', AmbPressure),  # Scaled ambient pressure.
    }

    imu_units = {
        0x04: 3*['Accel g'],
        0x05: 3*['Gyro rad/s'],
        0x06: 3*['Mag gauss'],
        0x07: 3*['Delta Theta rad'],
        0x08: 3*['Delta Vel g*s'],
        0x09: 9*['n/a'],
        0x0a: ['q0', 'q1', 'q2', 'q3'],
        0x0c: 3*['rad'],
        0x10: 3*['CF Mag gauss'],
        0x11: 3*['CF Accel g'],
        0x12: ['sec', 'week', 'n/a'],
        0x17: ['milliBar'],     
    }

    PosLLH = collections.namedtuple(
        'PosLLH',['lat', 'lon', 'ellipsoid_ht', 'msl_ht',
                  'horizontal_acc', 'vertical_acc', 'flags'
    ])
    PosECEF = collections.namedtuple(
        'PosECEF', ['x', 'y', 'z', 'position_acc', 'flags'])
    VelNED = collections.namedtuple('VelNED', [
        'north', 'east', 'down', 'speed', 'ground_speed', 'heading',
        'speed_acc', 'heading_acc', 'flags'
    ])
    VelECEF = collections.namedtuple('VelECEF',[
        'x', 'y', 'z', 'velocity_acc', 'flags'
    ])
    Dilution = collections.namedtuple('Dilution', [
        'geometric_dop', 'position_dop', 'horizontal_dop', 'vertical_dop',
        'time_dop', 'northing_dop', 'easting_dop', 'flags'
    ])
    UTC = collections.namedtuple('UTC', [
        'year', 'month', 'day', 'hour', 'minute', 'second', 'msec', 'flags'
    ])
    GPSTime = collections.namedtuple('GPSTime', [
        'tow', 'week_num', 'flags'
    ])
    ClockInfo = collections.namedtuple('ClockInfo', [
        'clock_bias', 'clock_drift', 'acc_est', 'flags'
    ])
    GPSFix = collections.namedtuple('GPSFix', [
        'fix_type', 'num_sol_SVs', 'fix_flags', 'valid_flags'
    ])
    SvInfo = collections.namedtuple('SvInfo', [
        'channel', 'sv_id', 'carrier_noise_ratio', 'az', 'el', 'sv_flags',
        'valid_flags'
    ])
    GpsHwStatus = collections.namedtuple('GpsHwStatus', [
        'sensor_state', 'antenna_state', 'antenna_power', 'flags'
    ])

    gps_set = {
        0x03: ('>ddddffH', PosLLH),
        0x04: ('>dddfH', PosECEF),
        0x05: ('>ffffffffH', VelNED),
        0x06: ('>ffffH', VelECEF),
        0x07: ('>fffffffH', Dilution),
        0x08: ('>HBBBBBIH', UTC),
        0x09: ('>dHH', GPSTime),
        0x0a: ('>dddH', ClockInfo),
        0x0b: ('>BBHH', GPSFix),
        0x0c: ('>BBH2s2sHH', SvInfo),
        0x0d: ('>BBBH', GpsHwStatus)
    }

    gps_units = {
        0x03: ['deg', 'deg', 'm', 'm', 'm', 'm', 'n/a'],
        0x04: 4*['m'] + ['n/a'],
        0x05: 5*['m/s'] + ['deg', 'm/s', 'deg', 'n/a'],
        0x06: 4*['m/s'] + ['n/a'],
        0x07: 8*['n/a'],
        0x08: ['years', 'months', 'days', 'hrs', 'min', 'sec', 'ms', 'n/a'],
        0x09: ['sec', 'week', 'n/a'],
        0x0a: ['sec', 'sec/sec', 'sec', 'n/a'],
        0x0b: ['n/a', 'count'] + 2*['n/a'],
        0x0c: ['chan_num', 'sv_id_num', 'dBHz', 'deg', 'deg'] + 2*['n/a'],
        0x0d: 4*['n/a']    
    }

    format_map = {
        0x01: base_set,
        0x0c: base_set,
        DataMessages.IMU.msg_ind: imu_set,
        DataMessages.GPS.msg_ind: gps_set,
    }

    @classmethod
    def process_field(self, desc_set: int,
                      mips_field: MipsField) -> tuple[any,...]:
        """Returns processed bytes according to field description."""
        message_set = self.format_map[desc_set]
        fmt, obj = message_set[mips_field.field_desc]
        return obj(*struct.unpack(fmt, mips_field.data_bytes)) 
    
    @classmethod
    def get_field_units(
        self, desc_set: int, mips_field: MipsField) -> list[str]:
        """Returns units for MipsField provided."""
        if desc_set == DataMessages.IMU.msg_ind:
            units = self.imu_units
        elif desc_set == DataMessages.GPS.msg_ind:
            units = self.gps_units
        return units[mips_field.field_desc]


def process_mips_packets(
        packets: list[MipsPacket]) -> list[collections.namedtuple]:
    """Converts fields of MipsPackets to objects."""
    return list(itertools.chain.from_iterable(
        [p.convert_payload() for p in packets]))


def divide_mips_packets(
        packets: list[MipsPacket]) -> tuple[MipsPacket, MipsPacket, MipsPacket]:
    """Separates MipsPackets into IMU, GPS, and EKF formats."""
    imu_data = []
    gps_data = []
    ekf_data = []
    for p in packets:
        if p.desc_set == DataMessages.IMU.msg_ind:
            imu_data.append(p)
        elif p.desc_set == DataMessages.GPS.msg_ind:
            gps_data.append(p)
        elif p.desc_set == DataMessages.EKF.msg_ind:
            ekf_data.append(p)
    return imu_data, gps_data, ekf_data 


class Microstrain3DM:
    """An interface for Microstrain 3D motion devices."""

    # Base rates for decimation of data from unit, specific to GX4 model.
    IMU_RATE = 500.0
    EKF_RATE = 500.0
    GPS_RATE = 4.0
    
    def __init__(self, port: str ='COM5', baudrate: int = 115200):
        self._ser = serial.Serial(port=port, baudrate=baudrate)
        # These are for debugging the raw traffic.
        self._read_buffer = collections.deque(maxlen=10)
        self._write_buffer = collections.deque(maxlen=10)
        # Dictionaries to map data collection frequencies to data messages.
        self._imu_fmt = None
        self._gps_fmt = None
        self._ekf_fmt = None
        # This is storage for data collected during streaming.
        self._stream_results = None  # Packets received.

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
        """Returns the first MipsPacket found on the serial connection."""
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
        return process_mips_packets([response])

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
    
    def _generate_field_for_descriptors(
            self, cmd_desc: int, base_rate: float, selector: int,
            descriptors: list[int], rate_hz: list[float]) -> MipsField:
        """Prepares descriptors to send in message format command."""
        num_descriptors = len(descriptors)
        field_data = [selector, num_descriptors]
        for i in range(num_descriptors):
            # Each field contains the descriptor number, and the decimation
            # factor as 2 bytes.
            desc_rate = max(int(base_rate / rate_hz[i]), 1)
            field_data.extend([descriptors[i],
                               desc_rate >> 8, desc_rate & 0xff])
        return MipsField(4 + 3 * num_descriptors, cmd_desc, field_data)
    
    def _get_max_rate(self, msg_type: DataMessages) -> float:
        if msg_type == DataMessages.IMU:
            base_rate = self.IMU_RATE
        elif msg_type == DataMessages.GPS:
            base_rate = self.GPS_RATE
        else:
            base_rate = self.EKF_RATE
        return base_rate
    
    def set_msg_fmt(
            self,
            msg_type: DataMessages = DataMessages.IMU,
            descriptors: list[int] = [0x04],
            rate_hz: list[float] = [500.]) -> MipsField:
        """Sets data format for device msg_type specified.
        
        All data formats are configured with this command. The device can be
        configured to stream data, and the command can define under what
        conditions the settings will be in effect (i.e. at startup).

        Args:
          msg_type: which data message the format is configured for.
          descriptors: A list of the codes for which measurements to send, see
            ResponseFormats for details.
          rate_hz: the rate to collect the data at to calculate the decimation.
            Can either be one number, or a list of rates, one per descriptor.
        Returns:
          The MipsField ACK response from device.
        """
        num_descriptors = len(descriptors)
        desc_hz = (rate_hz if num_descriptors == len(rate_hz)
            else num_descriptors * list(rate_hz))
        self._add_descriptors_and_rates(msg_type, descriptors, desc_hz)
        fmt_payload = self._generate_field_for_descriptors(
            msg_type.fmt_cmd, self._get_max_rate(msg_type),
            MessageFormatSelectors.NEW.value, descriptors, desc_hz)
        data = self._send_and_parse_reply(MipsPacket(0x0c, [fmt_payload]))
        return data[0]
    
    def _add_descriptors_and_rates(self, msg_type: DataMessages,
                                   descriptors: list[int],
                                   desc_hz: list[float]):
        """Stores descriptors for message type to help divide data later.
        
        Note: this will overwrite previous values.
        """
        keys = set(desc_hz)
        update_dict = {}       
        for k in keys:
            desc_for_rate = [d for d, r in zip(descriptors, desc_hz) if r == k]
            update_dict[k] = desc_for_rate
        if msg_type == DataMessages.IMU:
            self._imu_fmt = update_dict
        elif msg_type == DataMessages.GPS:
            self._gps_fmt = update_dict
        else:
            self._ekf_fmt = update_dict
    
    def get_msg_fmt(self,
                       msg_type: DataMessages = DataMessages.IMU) -> MipsField:
        """Returns the message format and decimation factors requested."""
        cmd = MipsField(0x04, msg_type.fmt_cmd,
                        [MessageFormatSelectors.READ_BACK.value, 0x00])
        data = self._send_command(MipsPacket(0x0c, [cmd]))
        # The format will be returned in a field after the ack message, and
        # consist of repeated messages.
        formats = data.payload[1]
        # The number of descriptors is stored in location 0, then the repeated
        # format begins.
        return [ReplyFormats.DataFmt(*t) for t in
                struct.iter_unpack('>BH', formats.data_bytes[1:])]
    
    def save_msg_fmt(self,
                        msg_type: DataMessages = DataMessages.IMU) -> MipsField:
        """Saves the current settings to apply at startup."""
        cmd = MipsField(0x04, msg_type.fmt_cmd,
                        [MessageFormatSelectors.SAVE.value, 0x00])
        data = self._send_command(MipsPacket(0x0c, [cmd]))
        return data.convert_payload()
    
    def load_msg_fmt(
            self, msg_type: DataMessages = DataMessages.IMU) -> MipsField:
        """Reloads saved settings onto device."""
        cmd = MipsField(0x04, msg_type.fmt_cmd,
                        [MessageFormatSelectors.LOAD.value, 0x00])
        data = self._send_command(MipsPacket(0x0c, [cmd]))
        return data.convert_payload()
    
    def reset_msg_fmt(
            self, msg_type: DataMessages = DataMessages.IMU) -> MipsField:
        """Resets the message format to factory defaults."""
        cmd = MipsField(0x04, msg_type.fmt_cmd,
                        [MessageFormatSelectors.RESET.value, 0x00])
        data = self._send_command(MipsPacket(0x0c, [cmd]))
        # Clear the stored version of the format.
        self._imu_fmt = None
        self._gps_fmt = None
        self._ekf_fmt = None
        return data.convert_payload()
    
    def poll_data(
            self, msg_type: DataMessages = DataMessages.IMU) -> MipsField:
        """Gets a set of values from device data type in current format."""
        # The poll request suppresses the normal (n)ack reply with option = 1.
        data = self._send_and_parse_reply(
            MipsPacket(0x0c,
                       [MipsField(0x04, msg_type.poll_cmd, [0x01, 0x00])]))
        return data

    def collect_data_stream(
            self,
            duration_sec: float = 10.) -> tuple[list[float], list[MipsPacket]]:
        """Collects packets for a time and returns data split to fields.
        
        Prior to calling this, you must have put a format onto the device. This
        function will handle starting and stopping the data streaming.

        Args:
          duration_sec: the amount of time to collect data for.
        Returns:
          The list of objects read from device.
        """
        self.device_resume()
        t_end = time.time() + duration_sec
        payload_recv = []
        while (t_now := time.time()) < t_end:
            # Read all packets available.
            while self._ser.in_waiting:
                new_packet = self._read_one_packet()
                new_packet.recv_time = t_now
                payload_recv.append(new_packet)
            time.sleep(0.001)
        self.device_idle()
        self._stream_results = payload_recv
        
        return payload_recv
    
    def _create_file_header_for_fmt(
            self,
            msg_type: DataMessages,
            desc_fmt: dict[float, list[int]]) -> dict[float, str]:
        """Makes a string useable for file header for received data."""
        name = msg_type.name.lower()
        headers = {}
        for rate, descriptors in desc_fmt.items():
            data_map = getattr(ReplyFormats, name + '_set')
            data_units = getattr(ReplyFormats, name + '_units')
            # Create header for file.
            hdr_line = []
            for d in descriptors:
                # Look up the data type and units.
                object = data_map[d][1]
                units = data_units[d]
                # Join for column name.
                hdr = [f'{v} ({u})' for v, u in zip(object._fields, units)]
                hdr_line.extend(hdr)
            headers[rate] = hdr_line
        return headers
    
    def write_stream_data(self, folder: str = './'):
        """Writes stream data to files."""
        # Divide packets by data message type, keep time.
        processed_data = divide_mips_packets(self._stream_results)
        message_types = (DataMessages.IMU, DataMessages.GPS, DataMessages.EKF)
        messages = (self._imu_fmt, self._gps_fmt, self._ekf_fmt)

        # For each data message collection, pick ones at same rate.
        for i in range(3):
            msg, fmt, data = message_types[i], messages[i], processed_data[i]
            if fmt is None:
                continue
            headers = self._create_file_header_for_fmt(msg, fmt)
            for rate, descriptors in fmt.items():
                fname = f'stream_{msg.name.lower()}_{rate}_hz.txt'
                entries = [['time (s)'] + headers[rate]]
                for d in data:
                    t = str(d.recv_time)
                    for f, val in zip(d.payload, d.convert_payload()):
                        # For each payload field, check if it is at this rate.
                        if f.field_desc in descriptors:
                            # Store time once per entry, this keeps messages
                            # at a lower rate from getting the faster time
                            # interval of another measurement.
                            if t not in entries[-1]:
                                entries.append([t])
                            entries[-1].extend(val)
                # Write to file.
                with open(os.path.join(folder, fname), 'w') as dest:
                    for entry in entries:
                        dest.write(','.join(map(str, entry)) + '\n')
