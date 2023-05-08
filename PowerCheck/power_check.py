r"""Serial interface wrapper to PWRCheck+ device using the binary interface.

This python interface provides configuration and data collection functions
for the West Mountain Radio Powercheck+ device. The device can be powered via
micro-USB or when connected to a power supply.

The power check device has a serial interface that is used in this wrapper.
In order to make it easier to manage on linux, it is recommended to
create a udev rule in /etc/udev/rules.d (suggest 88-utilis.rules), but
without any linebreaks:
ACTION=="add", ATTRS{idVendor}=="2405", ATTRS{idProduct}=="0001",
 ATTRS{product}=="PWRCheck+", MODE="0660", SYMLINK+="PwrCheck"

This will make sure whenever one is plugged in /dev/PwrCheck will be an
available device. The code was tested on windows and linux.
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
    UnitInfo = collections.namedtuple(
        'UnitInfo', ['major', 'minor', 'revision'])

    # The byte mapping and fields for the config message.
    CONFIG_FMT = '<BBIIIIBiiiiBH'
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
        'fast_log_interval_ms']  # Values: 25-999, 0 to disable.
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

    # Selects which values from the status message to log to file when streaming.
    _LOG_VALUES = ('calibrated_voltage_mv', 'calibrated_current_ma', 'ma_hours',
                   'quality_mv', 'forward_ma_hrs', 'reverse_ma_hrs')

    def __init__(self, port: str, baudrate: int = 115200, monitor_file_location: Optional[str] = None):
        """Sets up a power check device using interface information provided.

        Args:
          port: the path to device on machine.
          baudrate: the rate to communicate with the device.
          monitor_file_location: if provided, will open a file to receive streamed data.
        """
        # Open device.
        self._ser = serial.Serial(
            port=port, baudrate=baudrate, timeout=10, xonxoff=True)
        # Open file if requested.
        self._monitor_file = None
        if monitor_file_location:
            self.set_status_file(monitor_file_location)
        self._num_log_values = len(self._LOG_VALUES)  # Number of entries to put in file.

        # The next few items cache values requested from the device.
        self._last_config = None
        self._last_status = None
        self._unit_info = None

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

    def _make_packet(self, msg_type: int, payload: Optional[bytes] = None) -> bytes:
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

    def _process_packet(self, fmt: str, start_byte: int, packet: bytes):
        """Processes packet and returns values.
        
        Convenience method to index into packet, extract values, and return
        the converted result.

        Args:
          fmt: the struct fmt string to use for unpacking.
          start_byte: the type of message, which proceeds the payload to process.
          packet: the bytes to process from the unit.
        Returns:
          The extracted values from bytes after applying fmt.

        """
        return struct.unpack_from(fmt, packet, offset=packet.index(start_byte) + 1)

    def _nack_packet(self, code: int = 0x0) -> bytes:
        """Gets a nack packet to send to device.
        
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
        self._ser.write(self._nack_packet(code))

    def _packet_handshake(self, packet_type: int, data_fmt: str, 
                          data_type: collections.namedtuple) -> collections.namedtuple:
        """Requests data from device and returns processed data type class.

        For the get methods, there is a basic pattern to repeat:
        1. Send packet requesting packet type N.
        2. Receive response of type N+1 from device.
        3. Respond with ACK for packet within 250ms to prevent resends of packet.

        Args:
          packet_type: the numeric code of the packet being requested.
          data_fmt: a format string with the byte unpacking for message.
          data_type: a namedtuple to unpack the results into.
        Returns:
          An instance of the data_type provided.
        """
        request = self._make_packet(packet_type)
        self._ser.write(request)
        time.sleep(NACK_SLEEP)
        response = self._ser.read_all()
        print(f'Response: {response}')
        raw_packet = self._process_packet(data_fmt, packet_type + 1, response)
        self._send_nack()
        return data_type(*raw_packet)

    def get_config(self) -> Config:
        """Gets config (0xa4) for device and returns values."""
        self._last_config = self._packet_handshake(0xa4, self.CONFIG_FMT, self.Config)
        return self._last_config

    def set_config(self, **kwargs):
        """Sets configuration items provided from config namedtuple."""
        # Build the message based off last known values.
        packet_type = 0xa6
        config_dict = self.config._asdict()
        for key, val in kwargs.items():
            config_dict[key] = val
        # Convert back to namedtuple.
        new_config = self.Config(**config_dict)
        print(f'Now config = {new_config}')
        # Create payload.
        payload = struct.pack(self.CONFIG_FMT, *new_config)
        # Add 9 trailing unpopulated bytes.
        payload += 9 * b'\x00'
        request = self._make_packet(packet_type, payload)
        print(f'Send: {request}')
        self._ser.write(request)
        time.sleep(NACK_SLEEP)
        response = self._ser.read_all()  # Should get a nack + complete message.
        print(f'Received: {response}')

    def get_status(self):
        """Gets status (0xa0) measured on device."""
        self._last_status = self._packet_handshake(0xa0, self.STATUS_FMT, self.Status)
        return self._last_status
    
    def set_status_file(self, status_path: str):
        """Creates a file to receive streamed unit data.
        
        This can be defined after the object is constructed. If this
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

    def stream_status(self, sample_period_s: float = 1.0, num_samples: int = 10):
        """Streams status from device."""
        for i in range(num_samples):
            status = self.get_status()
            print(status)
            time.sleep(sample_period_s)
            if self._monitor_file:
                new_entry = [time.time()] + [getattr(status, e) for e in self._LOG_VALUES]
                self._monitor_file.write(
                    ('{:f}' + ',{:d}'*self._num_log_values + '\n').format(*new_entry))

    def get_unit_info(self):
        """Gets unit firmware information (0xf1)."""
        self._unit_info = self._packet_handshake(0xf1, self.UNIT_INFO_FMT, self.UnitInfo)
        return self._unit_info

    def reset_all_counters(self):
        """Resets system counters for specified element."""
        packet_type = 0xa8
        # Bits are:
        #  b0: Amp hours.
        #  b1: Min voltage.
        #  b2: Max voltage.
        #  b3: Min current.
        #  b4: Max Current.
        #  b5: Quality.
        #  b6: Clear all log data.
        payload = struct.pack('<B', 0b100000)
        request = self._make_packet(packet_type, payload)
        print(f'Send: {request}')
        self._ser.write(request)
        time.sleep(NACK_SLEEP)
        response = self._ser.read_all()  # Should get a nack.
        print(f'Received: {response}')
