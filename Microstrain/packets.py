r"""Helper library for handling packets from the microstrain device.

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
import enum
import itertools
import logging
import struct
from typing import Any, Optional

# The sync headers are always the same, and indicate the start of a packet.
SYNC1 = 0x75
SYNC2 = 0x65
SYNC_MSG = bytearray([SYNC1, SYNC2])

class DataMessages(enum.Enum):
    """Small class to bundle interactions with the different message formats.
    
    Attributes:
      msg_ind: the packet descriptor for a message of that type.
      fmt_cmd: the field descriptor portion of the format command.
      poll_cmd: the field desecriptor portion of the poll command.  
    """
    IMU = (0x80, 0x08, 0x01, 0x06)
    GPS = (0x81, 0x09, 0x02, 0x07)
    EKF = (0x82, 0x0a, 0x03, 0x0b)

    def __init__(self, msg_ind, fmt_cmd, poll_cmd, rate_cmd):
        self.msg_ind = msg_ind
        self.fmt_cmd = fmt_cmd
        self.poll_cmd = poll_cmd
        self.rate_cmd = rate_cmd

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

    def __init__(self, desc_set: int, payload: list[MipsField],
                 recv_time: Optional[float] = None):
        self.desc_set = desc_set
        self.payload = payload
        self.payload_len = sum([len(f) for f in self.payload])
        self.recv_time = recv_time

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
            try:
              value = ReplyFormats.process_field(self.desc_set, f)
              entries.append(value)
            except KeyError:
              logging.debug('Failed to process field %s', f.data)
        return entries
    
    def flatten_payload(self) -> list[Any]:
        """Flattens values in payload fields into a list."""
        processed_fields = self.convert_payload()
        return list(itertools.chain(*processed_fields))
    
    def as_log_str(self) -> str:
        t = struct.pack('>d', self.recv_time)
        return t + self.as_bytes

def split_payload_to_fields(payload: bytes) -> list[MipsField]:
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
    Vector = collections.namedtuple('Vector', ['x', 'y', 'z'])
    base_set = {
        0x81: ('>H16s16s16s16s16s', DevInfo),  # Device info.
        0x83: ('>I', int),  # Built-in self test.
        0x9b: ('>fff', Vector),  # Gyro bias.
        0xf1: ('>BB', Ack),  # (N)Ack message.
    }

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

    GpsLLH = collections.namedtuple(
        'GpsLLH',['lat', 'lon', 'ellipsoid_ht', 'msl_ht',
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
        'time_of_week', 'week_num', 'flags'
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
        0x03: ('>ddddffH', GpsLLH),
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

    FilterState = collections.namedtuple(
        'FilterState', ['state', 'dynamics_mode', 'flags'])
    EkfLLH = collections.namedtuple(
        'EkfLLH',['lat', 'lon', 'ellipsoid_ht', 'flags'])
    EkfVelNED = collections.namedtuple(
        'EkfVelNED', ['north', 'east', 'down', 'flags'])
    EkfQuat = collections.namedtuple('EkfQuat', Quat._fields + ('flags',))
    EkfMatrix = collections.namedtuple('EkfMatrix', Matrix._fields + ('flags',))
    EkfEuler = collections.namedtuple('EkfEuler', Euler._fields + ('flags',))
    EkfVector = collections.namedtuple('EkfVector', Vector._fields + ('flags',))
    UcLLH = collections.namedtuple('UcLLH', 
        [f'uc_{f}_1sig' for f in ['north', 'east', 'down']] + ['flags'])
    UcNED = collections.namedtuple('UcNED', UcLLH._fields)
    UcEuler = collections.namedtuple('UcLLH', 
        [f'uc_{f}_1sig' for f in ['roll', 'pitch', 'yaw']] + ['flags'])
    UcVec = collections.namedtuple('UcVec',
        ['uc_x_1sig', 'uc_y_1sig', 'uc_z_1sig', 'flags'])
    Wgs84 = collections.namedtuple('Wgs84', ['grav_mag', 'flags'])
    UcQuat = collections.namedtuple('UcQuat',
        [f'uc_{f}_1sig' for f in ['q0', 'q1', 'q2', 'q3']] + ['flags'])
    EkfHeading = collections.namedtuple('EkfHeading',
        ['true_heading', 'heading_uc_1sig', 'source', 'flags'])
    AtmModel = collections.namedtuple('AtmModel',
        ['geometric_alt', 'geopotential_alt', 'temp', 'pressure', 'density',
         'flags'])
    MagModel = collections.namedtuple('MagModel', 
        ['intensity_north', 'intensity_east', 'intensity_down', 'inclination',
        'declination', 'flags'])
    PressureAlt = collections.namedtuple('PressureAlt', ['pressure', 'flags'])

    ekf_set = {
        0x01: ('>dddH', EkfLLH),  # LLH position.
        0x02: ('>fffH', EkfVelNED),  # NED velocity.
        0x03: ('>ffffH', EkfQuat),  # Orientation quaternion.
        0x04: ('>fffffffffH', EkfMatrix),  # Orientation matrix.
        0x05: ('>fffH', EkfEuler),  # Orientation euler angles.
        0x06: ('>fffH', EkfVector),  # Gyro bias.
        0x07: ('>fffH', EkfVector),  # Accel bias.
        0x08: ('>fffH', UcLLH),  # LLH position uncertainty.
        0x09: ('>fffH', UcNED),  # NED velocity uncertainty.
        0x0a: ('>fffH', UcEuler),  # Attitude uncertainty euler angles.
        0x0b: ('>fffH', UcVec),  # Gyro bias uncertainty.
        0x0c: ('>fffH', UcVec),  # Accel bias uncertainty.
        0x0d: ('>fffH', EkfVector),  # Linear acceleration.  
        0x0e: ('>fffH', EkfVector),  # Compensated angular rate.
        0x0f: ('>fH', Wgs84),  # WGS84 local gravity magnitude.
        0x10: ('>HHH', FilterState),  # Filter status.
        0x11: ('>dHH', GPSTime),  # GPS timestamp.
        0x12: ('>ffffH', UcQuat),  # Attitude uncertainty, quat elements.
        0x13: ('>fffH', EkfVector),  # Gravity vector.
        0x14: ('>ffHH', EkfHeading),  # Heading update source state.
        0x15: ('>fffffH', MagModel),  # Magnetic model solution.
        0x16: ('>fffH', EkfVector),  # Gyro scale factor.
        0x17: ('>fffH', EkfVector),  # Accel scale factor.
        0x18: ('>fffH', UcVec),  # Gyro scale factor uncertainty.
        0x19: ('>fffH', UcVec),  # Accel scale factor uncertainty.
        0x1c: ('>fffH', EkfVector),  # Compensated linear acceleration.
        0x20: ('>fffffH', AtmModel),  # Standard atmosphere model.
        0x21: ('>fH', PressureAlt),  # Pressure altitude.
        0x30: ('>fffH', EkfVector),  # GPS Antenna offset correction.
        0x31: ('>fffH', UcVec),  # GPS Antenna offset correction uncertainty.
    }

    ekf_units = {
        0x01: 2*['deg'] + ['m', 'n/a'],
        0x02: 3*['m/s'] + ['n/a'],
        0x03: 5*['n/a'],
        0x04: 10*['n/a'],
        0x05: 3*['rad'] + ['n/a'],
        0x06: 3*['rad/s'] + ['n/a'],
        0x07: 3*['m/s2'] + ['n/a'],
        0x08: 3*['m'] + ['n/a'],
        0x09: 3*['m/s'] + ['n/a'],
        0x0a: 3*['rad'] + ['n/a'],
        0x0b: 3*['rad/s'] + ['n/a'],
        0x0c: 3*['m/s2'] + ['n/a'],
        0x0d: 3*['m/s2'] + ['n/a'], 
        0x0e: 3*['rad/s'] + ['n/a'],
        0x0f: ['m/s2', 'n/a'],
        0x10: 3*['n/a'],
        0x11: ['sec', 'week', 'n/a'],
        0x12: 5*['n/a'],
        0x13: 3*['m/s2'] + ['n/a'],
        0x14: 2*['rad'] + 2*['n/a'],
        0x15: 3*['gauss'] + 2*['rad'] + ['n/a'],
        0x16: 3*['Gyro SF %/100'] + ['n/a'],
        0x17: 3*['Acc SF %/100'] + ['n/a'],
        0x18: 3*['Gyro SF %/100'] + ['n/a'],
        0x19: 3*['Acc SF %/100'] + ['n/a'],
        0x1c: 3*['m/s2'] + ['n/a'],
        0x20: ['m', 'm', 'decC', 'mBar', 'kg/m3', 'n/a'],
        0x21: ['m', 'n/a'],
        0x30: 3*['m'] + ['n/a'],
        0x31: 3*['m'] + ['n/a'],
    }

    format_map = {
        0x01: base_set,  # Back commands for all devices.
        0x0c: base_set,  # To pick up ack/nack for 3DM commands.
        0x0d: base_set,  # To pick up ack/nack for EKF commands.
        DataMessages.IMU.msg_ind: imu_set,
        DataMessages.GPS.msg_ind: gps_set,
        DataMessages.EKF.msg_ind: ekf_set,
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
        elif desc_set == DataMessages.EKF.msg_ind:
            units = self.ekf_units
        return units[mips_field.field_desc]
    
    @classmethod
    def decode(self, data: collections.namedtuple) -> list[str]:
        """Converts data message into human readable form."""
        decoder_funcs = {
            self.FilterState.__name__: self._decode_ekf_status,
            self.GPSFix.__name__: self._decode_gps_fix_information,
            self.GpsHwStatus.__name__: self._decode_gps_hw_status,
        }
        func = decoder_funcs[type(data).__name__]
        return func(data)

    @classmethod
    def _apply_decoder(self, decoder: dict[int, str], val: int) -> list[str]:
        """Returns human readable description of integer value."""
        messages = []
        for key, msg in decoder.items():
            if key & val:
                messages.append(msg)
        return messages
    
    @classmethod
    def _decode_ekf_status(self, status: FilterState) -> list[str]:
        """Translates EKF messages into human readable form."""
        run_decoder = {
            0x0001: 'IMU Unavailable',
            0x0002: 'GPS Unavailable',
            0x0008: 'Matrix Singularity in calculation',
            0x0010: 'Position Covariance High Warning',
            0x0020: 'Velocity Covariance High Warning',
            0x0040: 'Attitude Covariance High Warning',
            0x0080: 'NAN in Solution',
            0x0100: 'Gyro bias estimate high warning',
            0x0200: 'Accel bias estimate high warning',
            0x0400: 'Gyro scale factor estimate high warning',
            0x0800: 'Accel scale factor estimate high warning',
            0x2000: 'GPS Antenna Offset Correction estimate high warning'
            }
        init_decoder = {
            0x1000: 'Attitude not initialized',
            0x2000: 'Position & Velocity not initialized'
            }
        mode_decoder = {
            0x00: 'startup',
            0x01: 'init',
            0x02: 'run',
            0x03: 'error'
        }
        state_str = self._apply_decoder(mode_decoder, status.state)[0]
        logging.debug(f'EKF in {state_str} phase.')
 
        decoder = {0x01: init_decoder,
                   0x02: run_decoder,
                   0x03: run_decoder}[status.state]
        return self._apply_decoder(decoder, status.flags)
    
    @classmethod
    def _decode_gps_fix_information(self, status: GPSFix) -> list[str]:
        """Parses fields of GPS fix information."""
        valid_decoder = {
            0x0001: "Fix type valid",
            0x0002: "Number of SVs valid",
            0x0004: "Fix flags valid",
        }
        type_decoder = {
            0x00: '3D Fix',
            0x01: '2D Fix',
            0x02: 'Time only',
            0x03: 'None',
            0x04: 'Invalid',
        }
        flag_decoder = {
            0x0001: 'SBAS corrections used',
            0x0002: 'Differential (DGPS) corrections used'
        }
        messages = self._apply_decoder(type_decoder, status.fix_type)
        messages.extend(self._apply_decoder(flag_decoder, status.fix_flags))
        messages.extend(self._apply_decoder(valid_decoder, status.valid_flags))
        return messages

    @classmethod
    def _decode_gps_hw_status(self, status: GpsHwStatus) -> list[str]:
        """Parses fields of GPS HW Status information."""
        valid_decoder = {
            0x0001: 'Sensor state valid',
            0x0002: 'Antenna state valid',
            0x0004: 'Antenna power valid',
        }
        sensor_state = {
            0x00: 'Sensor off',
            0x01: 'Sensor on',
            0x02: 'Sensor state unknown',
        }
        antenna_state = {
            0x01: 'Antenna init',
            0x02: 'Antenna short',
            0x03: 'Antenna open',
            0x04: 'Antenna good',
            0x05: 'Antenna state unknown',
        }
        antenna_power = {
            0x00: 'Antenna off',
            0x01: 'Antenna on',
            0x02: 'Antenna power unknown',
        }
        messages = self._apply_decoder(sensor_state, status.sensor_state)
        messages.extend(self._apply_decoder(antenna_state, status.antenna_state))
        messages.extend(self._apply_decoder(antenna_power, status.antenna_power))
        messages.extend(self._apply_decoder(valid_decoder, status.flags))
        return messages


def fletcher_checksum(packet: bytes) -> bytes:
    """Calculates fletcher checksum for bytes in packet."""
    check_sum = [sum(packet), sum(itertools.accumulate(packet))]
    return bytearray([0xff & check_sum[0], 0xff & check_sum[1]])


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
