r"""Serial interface wrapper to PWRCheck+ device using the binary interface.

This library is a platform independent serial interface to the PowerCheck+
power monitor device by West Mountain Radio.
Copyright (C) 2023  Stephen Sulack

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Communication is done via micro USB, which also powers the device. If
a voltage source is available it will run off that instead.

In order to make it easier to manage on linux, it is recommended to
create a udev rule in /etc/udev/rules.d (suggest 88-utilis.rules), but
without any linebreaks:
ACTION=="add", ATTRS{idVendor}=="2405", ATTRS{idProduct}=="0001",
ATTRS{product}=="PWRCheck+", MODE="0660", SYMLINK+="PwrCheck"

This will make sure whenever one is plugged in /dev/PwrCheck will be an
available device. The code was tested on windows and linux, using the
FW version 1.27.
"""
import collections
import os
import serial
import struct
import time
from typing import Optional

NACK_SLEEP = 0.2


class PowerCheck:
    """Implements a serial wrapper for PWRCheck+ HW.

    This class uses the binary interface available for the device.
    It's also possible to use the serial interface as any user
    could in an interactive serial console, however, navigating the
    menus and filtering the serial output complicates the logic.

    Attributes:
      config: A Config namedtuple, current settings on device.
      status: A Status namedtuple, current readings of sensors.
      unit_info: A UnitInfo namedtuple, software information for device.
    """
    # The byte mapping and fields for unit info message.
    UNIT_INFO_FMT = '<BBB'
    # Note that the versions are tracked in hex on the display, so
    # version 1.27 has a integer value of 39 for minor.
    UnitInfo = collections.namedtuple(
        'UnitInfo', ['major', 'minor', 'revision'])

    # The byte information for the repeated dump message.
    DUMP_FMT = '<Ii'
    DumpInfo = collections.namedtuple('DumpInfo', ['voltage_mv', 'current_ma'])

    # General byte packet structure.
    PacketInfo = collections.namedtuple('PacketInfo', ['type', 'payload'])

    # The byte mapping and fields for the config message. The config
    # message can be sent in two different formats, but the longer
    # one is used here, though not much is done with the display.
    CONFIG_FMT = '<BBIIIIBiiiiBHHHHHHHHHHBB'
    Config = collections.namedtuple('Config', [
        'backlight_timeout_s',  # Values: 1-240, 0 no timeout.
        'log_interval_s',  # Values: 1-120, 0 to disable.
        'over_voltage_mv',  # Values: 1-100000, 0 to disable.
        'under_voltage_mv',  # Same as overvoltage.
        'over_current_ma',  # Values: 1-40000, 0 to disable.
        'max_ma_hrs',  # Values: 0 - 4000000000.
        'alarm_notify_s',  # Values: 1-60, 0 for continuous.
        'current_adjustment',
        'current_offset',
        'voltage_adj',
        'voltage_offset',
        'unused_checksum',
        'fast_log_interval_ms',  # Values: 25-999, 0 to disable.
        # The next settings are related to the screen configuration,
        # and define the rgb colors of various features on PwrCheck+.
        'foreground_rgb',
        'background_rgb',
        'volt_bar_graph_rgb',
        'amp_bar_graph_rgb',
        'watt_bar_graph_rgb',
        'amp_hr_bar_graph_rgb',
        'volt_graph_rgb',
        'amp_graph_rgb',
        'alarm_rgb',
        'back_light_on_duty',  # Brightness of screen when on.
        'back_light_off_duty',]  # Brightness of screen when dimmed.
    )

    # The byte mapping and fields for the status message.
    STATUS_FMT = '<IiIiIIIIiiIII'
    Status = collections.namedtuple('Status', [
        'voltage_mv',  # Raw voltage.
        'current_ma',  # Raw  current.
        'calibrated_voltage_mv',  # Calibrated voltage.
        'calibrated_current_ma',  # Calibrated current.
        'calibrated_power_mw',  # Calibrated wattage.
        'ma_hours',
        'max_voltage_mv',
        'min_voltage_mv',
        'max_current_ma',
        'min_current_ma',
        'quality_mv',  # Peak to peak variance of voltage.
        'forward_ma_hrs',  # Source to load flow.
        'reverse_ma_hrs']  # Load to source flow.
    )

    # Selects values from the status message to log to file when streaming.
    _LOG_VALUES = ('calibrated_voltage_mv', 'calibrated_current_ma', 'ma_hours',
                   'quality_mv', 'forward_ma_hrs', 'reverse_ma_hrs')

    def __init__(self, port: str = 'COM3', baudrate: int = 115200,
                 monitor_file_location: Optional[str] = None):
        """Sets up a power check device using interface information provided.

        Args:
          port: the path to device on machine.
          baudrate: the rate to communicate with the device.
          monitor_file_location: if provided, will open a file to receive
            streamed data.
        """
        # Open device.
        self._ser = serial.Serial(
            port=port, baudrate=baudrate, timeout=10, xonxoff=True)
        # Open file if requested.
        self._monitor_file = None
        if monitor_file_location:
            self.set_status_file(monitor_file_location)
        # Number of entries to put in file.
        self._num_log_values = len(self._LOG_VALUES)

        # The next few items cache values requested from the device.
        self._last_config = None
        self._last_status = None
        self._unit_info = None

        # These are for debugging the raw traffic.
        self._read_buffer = collections.deque(maxlen=10)
        self._write_buffer = collections.deque(maxlen=10)

        # Clear out any leftover information.
        self._ser.flush()

    @property
    def config(self) -> Config:
        if self._last_config is None:
            self.get_config()
        return self._last_config

    @property
    def status(self) -> Status:
        if self._last_status is None:
            self.get_status()
        return self._last_status

    @property
    def unit_info(self) -> UnitInfo:
        if self._unit_info is None:
            self.get_unit_info()
        return self._unit_info

    def _write(self, value: bytes):
        """Writes and stores raw serial values."""
        self._ser.write(value)
        self._write_buffer.append(value)

    def dump_read_buffer(self) -> collections.deque:
        return self._read_buffer

    def dump_write_buffer(self) -> collections.deque:
        return self._write_buffer

    def dump_eeprom(self) -> list[tuple[int, int]]:
        """Initiates and manages dump of eeprom on device.

        When the unit EEPROM is dumped, payloads will be assembled to send the
        data in chunks. From testing this seems to be 12 entries at a time, so
        with 8 bytes per entry, the checksum, and packet type, they come in up
        to 98 byte payload chunks. The last packet indicates EOF by sending an
        empty payload.

        The unit can store more than 750K points, so this can take a while.

        Returns:
          A list of tuples, the voltage and current values from dump.
        """
        request_dump = self._make_packet(0xa2)
        # We need to send an accept and complete nack after each block of data.
        nack_accept = self._nack_packet()
        nack_complete = self._nack_packet(1)
        data = []
        cur_packet = self.PacketInfo(0x0, b'\x01')
        # Initiate the transfer.
        self._write(request_dump)
        # Read one packet at a time until we reach EOF.
        while cur_packet.payload:
            time.sleep(0.001)
            cur_packet = self._read_one_packet(type=0xa3)
            data.append(cur_packet.payload)
            self._write(nack_accept)
            self._write(nack_complete)
        return self._process_dump_raw_data(data)

    def _read_one_packet(self, type: Optional[int] = None) -> PacketInfo:
        """Reads one packet from the device.
        
        Thif function will either receive the first packet available, or
        the first packet of 'type' argument it sees.

        Args:
          type: the integer packet type to look for.
        Return:
          The PacketInfo (type and payload) of the response.
        """
        while 1:
            # Read to start of packet.
            self._ser.read_until(b'\x80')
            packet_len = self._ser.read(1)
            packet_data = self._ser.read(int.from_bytes(packet_len, 'little'))
            packet_checksum = packet_data[-1]
            # Validate all data received.
            if packet_checksum != sum(packet_len + packet_data[:-1]) & 0xff:
                print('Checksum mismatch:')
                print(packet_len + packet_data)
                time.sleep(0.25)  # Sleep to trigger a resend.
            else:
                self._read_buffer.append(packet_len + packet_data)
            # Separate the type and remove the checksum from the end.
            packet_result = self.PacketInfo(packet_data[0], packet_data[1:-1])
            if packet_result.type == type or type is None:
                break
        return packet_result

    def _process_dump_raw_data(self,
                               dump_bytes: bytes) -> list[tuple[int, int]]:
        """Processes the raw bytes received during an EEPROM data dump."""
        values = []
        # Skip last value, which is null.
        for p in dump_bytes[:-1]:
            values += list(struct.iter_unpack(self.DUMP_FMT, p))
        return values

    def _make_packet(self, msg_type: int,
                     payload: Optional[bytes] = None) -> bytes:
        """Makes a binary packet to send to device."""
        hdr = 0x80
        # Length, checksum are required for all packets.
        packet_length = 2
        packet = [hdr, packet_length, msg_type]
        if payload is not None:
            packet[1] += len(payload)
            packet.extend(payload)
        # Add checksum to end now that we know the complete message.
        packet.append(sum(packet[1:]) & 0xff)
        return bytes(packet)

    def _process_packet(self, fmt: str, packet: PacketInfo):
        """Processes packet according to format and returns values.

        Args:
          fmt: the struct fmt string to use for unpacking.
          packet: the bytes to process from the unit.
        Returns:
          The extracted values from bytes after applying fmt.
        """
        return struct.unpack_from(fmt, packet.payload)

    def _nack_packet(self, code: int = 0x0) -> bytes:
        """Returns a nack packet to send to device.

        Args:
          code: the status to relay.
            0 - packet accepted, link layer acceptance of packet.
            1 - operation complete.
            2 - operation cancel, used to stop dump of flash.
            ff - invalid command.
        """
        packet_type = 0xf0
        return self._make_packet(packet_type, code.to_bytes(1, 'little'))

    def _send_nack(self, code: int = 0x0):
        """Sends an acknowledgement of packet."""
        self._write(self._nack_packet(code))

    def _packet_handshake(
            self, packet_type: int, data_fmt: str,
            data_type: collections.namedtuple) -> collections.namedtuple:
        """Requests data from device and returns processed data.

        For the get methods, there is a basic pattern to repeat:
        1. Send packet requesting packet type N.
        2. Receive response of type N+1 from device.
        3. Respond with ACK for packet within 250ms to prevent resends of
          packet.

        Args:
          packet_type: the numeric code of the packet being requested.
          data_fmt: a format string with the byte unpacking for message.
          data_type: a namedtuple to unpack the results into.
        Returns:
          An instance of the data_type provided.
        """
        request = self._make_packet(packet_type)
        self._write(request)
        time.sleep(NACK_SLEEP)
        response = self._read_one_packet(packet_type + 1)
        raw_packet = self._process_packet(data_fmt, response)
        self._send_nack()
        return data_type(*raw_packet)

    def get_unit_info(self):
        """Gets unit firmware information (0xf1)."""
        self._unit_info = self._packet_handshake(
            0xf1, self.UNIT_INFO_FMT, self.UnitInfo)
        return self._unit_info

    def get_config(self) -> Config:
        """Gets config (0xa4) for device and returns values."""
        self._last_config = self._packet_handshake(
            0xa4, self.CONFIG_FMT, self.Config)
        return self._last_config

    def set_config(self, **kwargs):
        """Sets configuration items provided from config namedtuple.

        This function can be used to set config using the fields of the
        config namedtuple as kwargs, in any number. For example:
          unit.set_config(backlight_timeout_s=10, log_interval_s=1)

        First it fetches the current config, modifies the fields requested,
        and then sends them back to the device.
        """
        # Build the message based off last known values.
        packet_type = 0xa6
        config_dict = self.config._asdict()
        for key, val in kwargs.items():
            config_dict[key] = val
        # Convert back to namedtuple to order fields properly.
        new_config = self.Config(**config_dict)
        payload = struct.pack(self.CONFIG_FMT, *new_config)
        # Add 9 trailing unpopulated bytes per design doc.
        payload += 6 * b'\x00'  # 5 reserved bytes + 1 ignored checksum byte.
        request = self._make_packet(packet_type, payload)
        self._write(request)
        time.sleep(NACK_SLEEP)
        accept = self._read_one_packet()  # Read acceptance.
        complete = self._read_one_packet()  # Read complete.
        if accept.payload == b'\x00' and complete.payload == b'\x01':
            # If the packet was accepted and completed update cached config.
            self._last_config = new_config

    def get_status(self) -> Status:
        """Gets measurements status (0xa0) measured on device."""
        self._last_status = self._packet_handshake(
            0xa0, self.STATUS_FMT, self.Status)
        return self._last_status

    def set_status_file(self, status_path: str):
        """Creates a file to receive streamed unit data.

        This can be called after the class is constructed. If this
        is run and stream is called, the values collected from the
        unit will be copied to this file.

        Args:
          status_path: the location to put streamed data for device.
        """
        # If a folder, create a default name.
        if os.path.isdir(status_path):
            fname = 'pwrcheck_status_{}.txt'.format(
                time.strftime("%Y%m%d-%H%M%S"))
            monitor_path = os.path.join(status_path, fname)
        else:
            if os.path.exists(os.path.dirname(status_path)):
                monitor_path = status_path
        self._monitor_file = open(monitor_path, 'w')
        self._monitor_file.write(
            'time,' + ','.join(self._LOG_VALUES) + '\n')

    def stream_status(self, sample_period_s: float = 1.0,
                      num_samples: int = 10, echo_on: bool = False):
        """Streams status from device to screen and/or file."""
        write_fmt = '{:f}' + ',{:d}'*self._num_log_values + '\n'
        for i in range(num_samples):
            status = self.get_status()
            if echo_on:
                print(status)
            time.sleep(sample_period_s)
            if self._monitor_file:
                new_entry = [time.time()] + [getattr(status, e)
                                             for e in self._LOG_VALUES]
                self._monitor_file.write((write_fmt).format(*new_entry))

    def clear_log_data(self):
        """Clears log data stored on EEPROM of device."""
        packet_type = 0xa8
        # Bits are:
        #  b0: Amp hours.
        #  b1: Min voltage.
        #  b2: Max voltage.
        #  b3: Min current.
        #  b4: Max Current.
        #  b5: Quality.
        #  b6: Clear all log data.
        payload = struct.pack('<B', 0b1000000)
        request = self._make_packet(packet_type, payload)
        self._write(request)
        time.sleep(NACK_SLEEP)
        accept = self._read_one_packet()  # Acceptance of command.
        complete = self._read_one_packet()  # Completion of command.
