r"""Library for LORD Microstrain 3DM-GX4-45 navigation unit.

This library provides a software interface built around the MIP protocol of the
device. It has not been tested against the current generation of devices, but
would likely work since the structure is the mostly the same.

This serial library communicates using the MIP packet format, which consists of
header, payload, and checksum. The header defines which type of interaction is
happening - either general commands, IMU, GPS, or EKF data. The payload is
divided into fields, that also contain a descriptor to explain what kind of data
is included. The classes were designed to make it easy to translate between the
documentation and code. See the packets library for more details.

The device shows up under ttyACM*, so the following udev rule is suggested to
be added to 88-utilis.rules to make a specific device name on linux (without
the line break):
ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", 
  ATTRS{product}=="Lord Inertial Sensor", MODE="0660", SYMLINK+="ustrain"

Usage example:
from packets import DataMessages
unit = Microstrain3DM()
# Collect scaled accelerometer data at 250 Hz (500 Hz max).
unit.set_msg_fmt(DataMessages.IMU, descriptors=[0x04], rate_hz=[250.])
data = unit.collect_data_stream(10.)  # Start streaming data for 10 seconds.

# The data can be processed locally, or written out to csv with:
unit.write_stream_data()
"""
import enum
import collections
import logging
import packets
import serial
import struct
import time
from typing import BinaryIO, Optional, Union
import os

class FunctionSelectors(enum.Enum):
    """These are the possible options when setting up a data message format."""
    NEW = 1
    READ_BACK = 2
    SAVE = 3
    LOAD = 4
    RESET = 5

class EkfDynamicsMode(enum.Enum):
    PORTABLE = 1  # Low acceleration applications.
    AUTOMOTIVE = 2  # Low vertical acceleration, wheeled vehicle dynamics.
    AIRBORNE = 3  # Airborne up to 2 Gs.
    AIRBORNE_HIGH_G = 4  # Airborn up to 4 Gs.

class EkfHeadingUpdate(enum.Enum):
    DISABLE = 0
    MAG = 1
    GPS = 2
    EXT = 3

class Microstrain3DM:
    """An interface for Microstrain 3D motion devices."""

    # Base rates for decimation of data from unit, specific to GX4 model.
    IMU_RATE = 500.0
    EKF_RATE = 500.0
    GPS_RATE = 4.0

    # Supported baud rates.
    BAUD_RATES = (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)
    
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
        # Flush any leftovers.
        self._ser.flush()

    def _collect_packet_from_source(
            self,
            source: Union[BinaryIO, serial.Serial]) -> packets.MipsPacket:
        """Collects packet from binary stream, used for files or serial.
        
        This allows for reuse of binary data reading from previously recorded
        files or live serial data. Source will be either the class serial
        object or a file descriptor.

        Args:
          source: the location of data, which has a read method.
        Returns:
          The next MipsPacket from the data stream.
        """
        sync = b''
        # It would be nice to use read_until, but that's only available for
        # serial objects, and this function is reused for data stored in files.
        while tmp := source.read(1):
            sync += tmp
            if sync.endswith(packets.SYNC_MSG):
                break
        data = source.read(2)
        desc_set, payload_len = struct.unpack('>BB', data)
        payload = source.read(payload_len)
        checksum = source.read(2)
        field_data = packets.split_payload_to_fields(payload)
        self._read_buffer.append(data + payload + checksum)
        packet = packets.MipsPacket(desc_set, field_data)
        return packet
    
    def _read_one_packet(self) -> packets.MipsPacket:
        """Returns the first MipsPacket found on the serial connection."""
        packet = self._collect_packet_from_source(self._ser)
        packet.recv_time = time.time()
        return packet
    
    def _write(self, value: bytes):
        """Writes data to device and buffers for debug purposes."""
        self._ser.write(value)
        self._write_buffer.append(value)

    def _send_command(self, packet: packets.MipsPacket) -> packets.MipsPacket:
        """Sends command to device and provides response."""
        self._write(packet.as_bytes)
        time.sleep(0.001)
        return self._read_one_packet()
    
    def _send_and_parse_reply(self, packet: packets.MipsPacket
                              ) -> list[collections.namedtuple]:
        """Sends command and returns device reply object."""
        response = self._send_command(packet)
        return packets.process_mips_packets([response])

    def dump_read_buffer(self) -> collections.deque:
        return self._read_buffer

    def dump_write_buffer(self) -> collections.deque:
        return self._write_buffer
    
    def device_ping(self) -> collections.namedtuple:
        """Sends ping command to device and returns ack message."""
        data = self._send_and_parse_reply(
            packets.MipsPacket(0x01, [packets.MipsField(0x02, 0x01)]))
        return data[0]
    
    def device_reset(self) -> collections.namedtuple:
        """Resets device and returns ack message."""
        data = self._send_and_parse_reply(
            packets.MipsPacket(0x01, [packets.MipsField(0x02, 0x7e)]))
        return data[0]
    
    def device_idle(self) -> collections.namedtuple:
        """Sets device into idle mode, which interrupts streaming or sleep."""
        data = self._send_and_parse_reply(
            packets.MipsPacket(0x01, [packets.MipsField(0x02, 0x02)]))
        return data[0]
    
    def device_resume(self) -> collections.namedtuple:
        """Sets device into previous or default mode."""
        data = self._send_and_parse_reply(
            packets.MipsPacket(0x01, [packets.MipsField(0x02, 0x06)]))
        return data[0]
    
    def device_info(self) -> collections.namedtuple:
        """Returns information for device."""
        data = self._send_and_parse_reply(
            packets.MipsPacket(0x01, [packets.MipsField(0x02, 0x03)]))
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
            packets.MipsPacket(0x01,[packets.MipsField(0x02, 0x05)]))
        return data[1]
    
    def device_baud(self, baud_rate = 115200,
                    func_sel: FunctionSelectors = FunctionSelectors.NEW
    ) -> collections.namedtuple:
        """Sets baud rate on device."""
        valid_rates = self.BAUD_RATES
        if baud_rate not in valid_rates:
            raise ValueError(f'Baud rate must be one of {valid_rates}')
        if baud_rate != self._ser.baudrate:
            # Only send if this will be a change on the hardware side.
            field_bytes = struct.pack('>BI', func_sel.value, baud_rate)
            cmd = packets.MipsPacket(0x0c, [packets.MipsField(0x07, 0x40,
                                              data_bytes=field_bytes)])
            self._send_and_parse_reply(cmd)
            time.sleep(0.25)
            self._ser.close()
            self._baud_rate = baud_rate
            self._ser.baudrate = baud_rate
            self._ser.open()

    def device_base_rate(self, msg_type: packets.DataMessages) -> int:
        """Queries and returns base rate for msg on device."""
        cmd = packets.MipsPacket(0x0c,
                                 [packets.MipsField(0x02, msg_type.rate_cmd)])
        response = self._send_command(cmd)
        # This response comes back with a duplicate field descriptor (0x83), so
        # process it locally.
        data_field = response.payload[1]
        rate_hz = struct.unpack('>H', data_field.data_bytes)
        return rate_hz[0]
    
    def _generate_field_for_descriptors(
            self, cmd_desc: int, base_rate: float, selector: int,
            descriptors: list[int], rate_hz: list[float]) -> packets.MipsField:
        """Prepares descriptors to send in message format command."""
        num_descriptors = len(descriptors)
        field_data = [selector, num_descriptors]
        for i in range(num_descriptors):
            # Each field contains the descriptor number, and the decimation
            # factor as 2 bytes. Allow this to be zero to cover the case of
            # polling data in a provided format.
            desc_rate = int(base_rate / rate_hz[i])
            field_data.extend([descriptors[i],
                               desc_rate >> 8, desc_rate & 0xff])
        return packets.MipsField(4 + 3 * num_descriptors, cmd_desc, field_data)

    def _get_max_rate(self, msg_type: packets.DataMessages) -> float:
        if msg_type == packets.DataMessages.IMU:
            base_rate = self.IMU_RATE
        elif msg_type == packets.DataMessages.GPS:
            base_rate = self.GPS_RATE
        else:
            base_rate = self.EKF_RATE
        return base_rate
    
    def set_msg_fmt(
            self,
            msg_type: packets.DataMessages = packets.DataMessages.IMU,
            descriptors: list[int] = [0x04],
            rate_hz: list[float] = [500.]) -> packets.MipsField:
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
            FunctionSelectors.NEW.value, descriptors, desc_hz)
        data = self._send_and_parse_reply(
            packets.MipsPacket(0x0c, [fmt_payload]))
        return data[0]
    
    def _add_descriptors_and_rates(self, msg_type: packets.DataMessages,
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
        if msg_type == packets.DataMessages.IMU:
            self._imu_fmt = update_dict
        elif msg_type == packets.DataMessages.GPS:
            self._gps_fmt = update_dict
        else:
            self._ekf_fmt = update_dict
    
    def get_msg_fmt(
            self, msg_type: packets.DataMessages = packets.DataMessages.IMU
            ) -> packets.MipsField:
        """Returns the message format and decimation factors requested."""
        cmd = packets.MipsField(0x04, msg_type.fmt_cmd,
                                [FunctionSelectors.READ_BACK.value, 0x00])
        data = self._send_command(packets.MipsPacket(0x0c, [cmd]))
        # The format will be returned in a field after the ack message, and
        # consist of repeated messages.
        formats = data.payload[1]
        # The number of descriptors is stored in location 0, then the repeated
        # format begins.
        return [packets.ReplyFormats.DataFmt(*t) for t in
                struct.iter_unpack('>BH', formats.data_bytes[1:])]
    
    def save_msg_fmt(
            self, msg_type: packets.DataMessages = packets.DataMessages.IMU
            ) -> packets.MipsField:
        """Saves the current settings to apply at startup."""
        cmd = packets.MipsField(0x04, msg_type.fmt_cmd,
                                [FunctionSelectors.SAVE.value, 0x00])
        data = self._send_command(packets.MipsPacket(0x0c, [cmd]))
        return data.convert_payload()
    
    def load_msg_fmt(
            self, msg_type: packets.DataMessages = packets.DataMessages.IMU
            ) -> packets.MipsField:
        """Reloads saved settings onto device."""
        cmd = packets.MipsField(0x04, msg_type.fmt_cmd,
                                [FunctionSelectors.LOAD.value, 0x00])
        data = self._send_command(packets.MipsPacket(0x0c, [cmd]))
        return data.convert_payload()
    
    def reset_msg_fmt(
            self, msg_type: packets.DataMessages = packets.DataMessages.IMU
            ) -> packets.MipsField:
        """Resets the message format to factory defaults."""
        cmd = packets.MipsField(0x04, msg_type.fmt_cmd,
                                [FunctionSelectors.RESET.value, 0x00])
        data = self._send_command(packets.MipsPacket(0x0c, [cmd]))
        # Clear the stored version of the format.
        self._imu_fmt = None
        self._gps_fmt = None
        self._ekf_fmt = None
        return data.convert_payload()
    
    def poll_data(
            self, msg_type: packets.DataMessages = packets.DataMessages.IMU,
            suppress_nack: int = 1,  # Default to no nack.
            descriptors: Optional[list[int]] = None
            ) -> list[collections.namedtuple]:
        """Gets a set of values from device data type in current format."""
        if descriptors is not None:
            # Create the field data for the requested format using zero as the
            # base rate to make sure the descriptor gets set to the appropriate
            # reserve value of 0.
            num_descriptors = len(descriptors)
            fmt_field = self._generate_field_for_descriptors(
                msg_type.poll_cmd, 0, suppress_nack, descriptors,
                num_descriptors * [1])
        else:
            fmt_field = packets.MipsField(0x04, msg_type.poll_cmd,
                                          [suppress_nack, 0x00])
        # The poll request suppresses the normal (n)ack reply with option = 1.
        data = self._send_and_parse_reply(packets.MipsPacket(0x0c, [fmt_field]))
        return data
    
    def ekf_auto_init(self, func_sel: FunctionSelectors,
                      enable: int = 1) -> collections.namedtuple:
        """Enables auto-initialization of filter on device.
        
        If enabled (1), the filter will auto-initialize with the heading source
        provided.

        Args:
          func_sel: how to apply the setting.
          enable: the value to enable (1) or disable (0) auto-initialize.
        """
        cmd = packets.MipsPacket(
            0x0d, [packets.MipsField(0x04, 0x19,[func_sel.value, enable])])
        data = self._send_and_parse_reply(cmd)
        return data[0]
    
    def ekf_euler_init(
            self, euler_vec: Optional[packets.ReplyFormats.Vector] = None
            ) -> collections.namedtuple:
        """Initializes EKF attitude with euler angles provided."""
        if euler_vec is None:
            # Fetch angles from device.
            euler_vec = self.poll_data(descriptors=[0x0c])[0]
        logging.debug(f'Initializing ekf with {euler_vec}')
        vec_bytes = struct.pack('>fff', *euler_vec)
        cmd = packets.MipsPacket(
            0x0d, [packets.MipsField(0x0e, 0x02, data_bytes=vec_bytes)])
        data = self._send_and_parse_reply(cmd)
        return data[0]

    def ekf_heading_source(
            self, func_sel: FunctionSelectors,
            option: EkfHeadingUpdate) -> collections.namedtuple:
        """Sets source of heading updates for EKF.
        
        Select the heading source, then enable auto-initialization of the
        filter to transition into run mode from init.

        Args:
          func_sel: how the setting shall be applied.
          option: the heading update source to use.
        Returns:
          The ack/nack message from the command.
        """
        cmd = packets.MipsPacket(
            0x0d, [packets.MipsField(0x04, 0x18,
                                     [func_sel.value, option.value])])
        data = self._send_and_parse_reply(cmd)
        return data[0]       
    
    def capture_gyro_bias(self, sample_duration_ms: int):
        """Captures gyro bias to device and returns value.
        
        Note - the device should be still during this operation. The vector
        of gyro bias will be returned when complete.

        Args:
          sample_duration_ms: the amount of time to collect bias for.
        Returns:
          The vector of bias calculated for the gyro.
        """
        if not (1000 <= sample_duration_ms <= 30000):
            raise ValueError(f'Time span {sample_duration_ms} not in range.')
        sample_bytes = struct.pack('>H', sample_duration_ms)
        cmd = packets.MipsPacket(
            0x0c, [packets.MipsField(0x04, 0x39, data_bytes=sample_bytes)])
        logging.debug(f'Sleeping for {sample_duration_ms} ms to capture bias.')
        # Response will show up after the sleep period.
        response = self._send_command(cmd)
        # Return the ack + bias vector estimated by the filter.
        return response.convert_payload()

    def set_dynamics_mode(
            self, mode: EkfDynamicsMode,
            func_selector: FunctionSelectors) -> collections.namedtuple:
        cmd = packets.MipsPacket(0x0d, [packets.MipsField(0x04, 0x10,
                                          [func_selector.value, mode.value])])
        data = self._send_and_parse_reply(cmd)
        return data[0]
    
    def get_single_packet(self) -> packets.MipsPacket:
        """Gets and returns a single packet as a string if available.
        
        This method is intended for external callers to see if a packet is
        available and grab it if so. If not it returns quickly to allow for
        other async data collection elsewhere.
        """
        if self._ser.in_waiting:
            new_packet = self._read_one_packet()
            return new_packet

    def collect_data_stream(
            self,
            duration_sec: float = 10.) -> list[packets.MipsPacket]:
        """Collects packets for a time and returns data split to fields.
        
        Prior to calling this, you must have put a format onto the device. This
        function will handle starting and stopping the data streaming. Note that
        on windows, timing precision limits us to about 50 Hz, the full 500 Hz
        is achievable on linux.

        Args:
          duration_sec: the amount of time to collect data for.
        Returns:
          The list of objects read from device.
        """
        self.device_resume()
        t_end = time.time() + duration_sec
        logging.info(f'Will stop data collection at {t_end}')
        payload_recv = []
        while time.time() < t_end:
            # Read all packets available.
            while self._ser.in_waiting:
                payload_recv.append(self.get_single_packet())
            time.sleep(0.001)
            logging.debug('outer loop')
        self.device_idle()
        self._stream_results = payload_recv
        logging.debug('Microstrain stream complete.')
        return payload_recv
    
    def load_data_stream(self, file_path: str):
        """Loads file of collected binary data into the class stream results.
        
        During data collection, the output of the device is logged as binary
        data directly to disk for analysis later. This function allows the class
        to be reloaded with a set of data, to take advantage of the write
        functions defined for streams already.

        Args:
          file_path: the location of the binary file with microstrain data.
        """
        self._stream_results = []
        logging.debug(f'Loading data stream from {file_path}.')
        with open(file_path, 'rb') as f:
            while t := f.read(8):
                new_packet = self._collect_packet_from_source(f)
                new_packet.recv_time = struct.unpack('>d', t)[0]
                self._stream_results.append(new_packet)
    
    def _create_file_header_for_fmt(
            self,
            msg_type: packets.DataMessages,
            desc_fmt: dict[float, list[int]]) -> dict[float, str]:
        """Makes a string useable for file header for received data."""
        name = msg_type.name.lower()
        headers = {}
        for rate, descriptors in desc_fmt.items():
            data_map = getattr(packets.ReplyFormats, name + '_set')
            data_units = getattr(packets.ReplyFormats, name + '_units')
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
        processed_data = packets.divide_mips_packets(self._stream_results)
        message_types = (packets.DataMessages.IMU, packets.DataMessages.GPS,
                         packets.DataMessages.EKF)
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
                    new_entry = [t]
                    for f, val in zip(d.payload, d.convert_payload()):
                        # For each payload field, check if it is at this rate.
                        if f.field_desc in descriptors:
                            # Store time once per entry, this keeps messages
                            # at a lower rate from getting the faster time
                            # interval of another measurement.
                            new_entry.extend(val)
                    if len(new_entry) > 1:
                        entries.append(new_entry)
                # Write to file.
                with open(os.path.join(folder, fname), 'w') as dest:
                    for entry in entries:
                        dest.write(','.join(map(str, entry)) + '\n')
