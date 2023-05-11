from unittest import mock
import power_check
import pytest

# Mock out serial for all tests since we won't talk to HW.
@pytest.fixture(autouse=True)
def serial_mock():
    with mock.patch.object(power_check.serial, 'Serial') as _fixture:
        yield _fixture

@pytest.mark.byte_tests
def test_make_packet_for_expected_structure():
    unit = power_check.PowerCheck(port='/any/port/')
    packet = unit._make_packet(0xff)
    # Packets start with preamble byte, size (counts all to the right)
    # the packet type, and a checksum (addition of all bytes).
    expected_packet = b'\x80\x02\xff\x01'
    assert packet == expected_packet

@pytest.mark.byte_tests
def test_process_packet_applies_format():
    unit = power_check.PowerCheck(port='/any/port')
    sample_packet = power_check.PowerCheck.PacketInfo(
        type=0xed, payload=b'\x12\x34\x56\x78\x9a\xbc\xde')
    values = unit._process_packet('<BHI', sample_packet)
    assert values == (0x12, 0x5634, 0xdebc9a78)

# Test for process packet working with each structure and sample messages.

@pytest.mark.byte_tests
@pytest.mark.parametrize(
    'code, expect', [
        (0x0, b'\x80\x03\xf0\x00\xf3'),
        (0x1, b'\x80\x03\xf0\x01\xf4'),
        (0x2, b'\x80\x03\xf0\x02\xf5')
    ]
)
def test_send_nack(code: int, expect: bytes):
    unit = power_check.PowerCheck(port='/any/port')
    unit._send_nack(code)
    unit._ser.write.assert_called_with(expect)

@pytest.mark.byte_tests
def test_packet_handshake_sequence(mocker):
    mock_make = mocker.patch('power_check.PowerCheck._make_packet')
    mock_read_one = mocker.patch('power_check.PowerCheck._read_one_packet')
    unit = power_check.PowerCheck(port=None)
    mock_read_one.return_value = power_check.PowerCheck.PacketInfo(type=0xf2, payload=b'\x01\x02\x03')
    packet_type = 0xf1
    packet_fmt = unit.UNIT_INFO_FMT
    packet_class = unit.UnitInfo
    result = unit._packet_handshake(packet_type, packet_fmt, packet_class)
    mock_make.assert_has_calls([mock.call(packet_type), mock.call(0xf0, b'\x00')])
    assert result == unit.UnitInfo(1, 2, 3)

@pytest.mark.byte_tests
def test_get_config_uses_correct_arguments(mocker):
    """This just makes sure the packet types and formats are matched."""
    mock_handshake = mocker.patch('power_check.PowerCheck._packet_handshake')
    unit = power_check.PowerCheck(port=None).get_config()
    mock_handshake.assert_called_once_with(
        0xa4, power_check.PowerCheck.CONFIG_FMT, power_check.PowerCheck.Config)

@pytest.mark.byte_tests
def test_get_status_uses_correct_arguments(mocker):
    mock_handshake = mocker.patch('power_check.PowerCheck._packet_handshake')
    unit = power_check.PowerCheck(port=None).get_status()
    mock_handshake.assert_called_once_with(
        0xa0, power_check.PowerCheck.STATUS_FMT, power_check.PowerCheck.Status)

@pytest.mark.byte_tests  
def test_get_unit_info_uses_correct_arguments(mocker):
    mock_handshake = mocker.patch('power_check.PowerCheck._packet_handshake')
    unit = power_check.PowerCheck(port=None).get_unit_info()
    mock_handshake.assert_called_once_with(
        0xf1, power_check.PowerCheck.UNIT_INFO_FMT, power_check.PowerCheck.UnitInfo)
    
@pytest.mark.byte_tests  
def test_set_config_makes_correct_packet_one_setting(mocker):
    mock_packet = mocker.patch('power_check.PowerCheck._make_packet')
    unit = power_check.PowerCheck(port=None)
    unit._last_config = power_check.PowerCheck.Config(*range(13))
    unit.set_config(log_interval_s=10)
    # The payload is constructed to match the Config object, using the range input
    # values. The start of each field can be seen in the message below, except the
    # log_interval_s, which is overwritten in the set_config command.
    expected_payload = (b'\x00\x0a\x02\x00\x00\x00\x03\x00\x00\x00\x04\x00\x00\x00'
                        b'\x05\x00\x00\x00\x06\x07\x00\x00\x00\x08\x00\x00\x00\x09\x00\x00\x00'
                        b'\x0a\x00\x00\x00\x0b\x0c\x00' + 9 * b'\x00')
    assert mock_packet.call_args_list[0] == mock.call(0xa6, expected_payload)

@pytest.mark.byte_tests  
def test_set_config_makes_correct_packet_multi_settings(mocker):
    mock_packet = mocker.patch('power_check.PowerCheck._make_packet')
    unit = power_check.PowerCheck(port=None)
    unit._last_config = power_check.PowerCheck.Config(*range(13))
    unit.set_config(backlight_timeout_s=33, max_ma_hrs=0x11223344)
    # The payload is constructed to match the Config object, using the range input
    # values. The start of each field can be seen in the message below, except the
    # fields provided. Note also the bytes are little endian.
    expected_payload = (b'\x21\x01\x02\x00\x00\x00\x03\x00\x00\x00\x04\x00\x00\x00'
                        b'\x44\x33\x22\x11\x06\x07\x00\x00\x00\x08\x00\x00\x00\x09\x00\x00\x00'
                        b'\x0a\x00\x00\x00\x0b\x0c\x00' + 9 * b'\x00')
    assert mock_packet.call_args_list[0] == mock.call(0xa6, expected_payload)
 
@pytest.mark.byte_tests
def test_clear_log_sends_proper_message():
    unit = power_check.PowerCheck(port=None)
    unit.clear_log_data()
    unit._ser.write.assert_called_with(b'\x80\x03\xa8\x40\xeb')

@pytest.mark.byte_tests
def test_dump_eeprom(mocker):
    mock_read_one = mocker.patch('power_check.PowerCheck._read_one_packet')
    mock_read_one.side_effect = (
        power_check.PowerCheck.PacketInfo(0xa3, b'\x44\x33\x22\x11\x40\x30\x20\x10'),
        power_check.PowerCheck.PacketInfo(0xa3, b'')
    )
    expect = [(0x11223344, 0x10203040)]
    unit = power_check.PowerCheck(port=None)
    results = unit.dump_eeprom()
    calls = unit._ser.write.call_args_list
    # Confirm message to start dump of data.
    assert calls[0] == mock.call(b'\x80\x02\xa2\xa4')
    # Confirm an accept and complete per transmission.
    assert calls[1:] == 2 * [mock.call(unit._nack_packet(0)), mock.call(unit._nack_packet(1))]
    assert results == expect

def test_stream_status(mocker):
    status_packet = power_check.PowerCheck.Status(*range(13))
    mock_status = mocker.patch('power_check.PowerCheck.get_status',
                               return_value=status_packet)
    mock_time = mocker.patch('power_check.time.time', return_value=1)
    mock_sleep = mocker.patch('power_check.time.sleep')
    unit = power_check.PowerCheck(port=None)
    unit._monitor_file = mock.Mock()
    stream_values_expected = 2
    stream_sleep = 0.1
    unit.stream_status(stream_sleep, stream_values_expected)
    assert unit._monitor_file.write.call_count == stream_values_expected
    mock_sleep.assert_has_calls(2 * [mock.call(stream_sleep)])

def test_config_property_returns_last_config():
    unit = power_check.PowerCheck(port=None)
    config = power_check.PowerCheck.Config(*range(13))
    unit._last_config = config
    assert unit.config == config

def test_status_property_returns_last_status():
    unit = power_check.PowerCheck(port=None)
    status = power_check.PowerCheck.Status(*range(13))
    unit._last_status = status
    assert unit.status == status

def test_unit_info_property_returns_last_unit_info():
    unit = power_check.PowerCheck(port=None)
    info = power_check.PowerCheck.UnitInfo(1, 2, 3)
    unit._unit_info = info
    assert unit.unit_info == info

@pytest.mark.parametrize(
    'property, func_to_call', [
        ('config', 'get_config'),
        ('status', 'get_status'),
        ('unit_info', 'get_unit_info')
    ]
)
def test_config_property_requests_config_if_none(mocker, property, func_to_call):
    unit = power_check.PowerCheck(port=None)
    mock_func = mocker.patch('power_check.PowerCheck.' + func_to_call)
    _unused_property = getattr(unit, property)
    mock_func.assert_called_once()

@pytest.mark.parametrize(
    'file_or_directory, is_file', [
        ('./', False),  # In this case the file will be autonamed, no format required.
        ('./test.txt', True),
    ]
)
def test_status_file_opens_correctly(mocker, file_or_directory, is_file):
    mock_open = mocker.patch('builtins.open')
    unit = power_check.PowerCheck(port=None)
    unit.set_status_file(file_or_directory)
    assert mock_open.call_args[0][0].startswith(file_or_directory)
