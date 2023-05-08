from unittest import mock
import power_check
import pytest

# Mock out serial for all tests since we won't talk to HW.
@pytest.fixture(scope='session', autouse=True)
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
    values = unit._processs_packet('<BHI', 0x80, b'\x80\x12\x34\x56\x78\x9A\xBC\xDE')
    assert values == (0x12, 0x5634, 0xDEBC9A78)

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
    mock_make = mocker.patch('power_check2.PowerCheck._make_packet')
    unit = power_check.PowerCheck(port=None)
    unit._ser.read_all.return_value = b'\x80\x05\xf2\x01\x02\x03\x0b'
    packet_type = 0xf1
    packet_fmt = unit.UNIT_INFO_FMT
    packet_class = unit.UnitInfo
    result = unit._packet_handshake(packet_type, packet_fmt, packet_class)
    mock_make.assert_has_calls([mock.call(packet_type), mock.call(0xf0, b'\x00')])
    assert result == unit.UnitInfo(1, 2, 3)

@pytest.mark.byte_tests
def test_get_config_uses_correct_arguments(mocker):
    """This just makes sure the packet types and formats are matched."""
    mock_handshake = mocker.patch('power_check2.PowerCheck._packet_handshake')
    unit = power_check.PowerCheck(port=None).get_config()
    mock_handshake.assert_called_once_with(
        0xa4, power_check.PowerCheck.CONFIG_FMT, power_check.PowerCheck.Config)

@pytest.mark.byte_tests
def test_get_status_uses_correct_arguments(mocker):
    mock_handshake = mocker.patch('power_check2.PowerCheck._packet_handshake')
    unit = power_check.PowerCheck(port=None).get_status()
    mock_handshake.assert_called_once_with(
        0xa0, power_check.PowerCheck.STATUS_FMT, power_check.PowerCheck.Status)

@pytest.mark.byte_tests  
def test_get_unit_info_uses_correct_arguments(mocker):
    mock_handshake = mocker.patch('power_check2.PowerCheck._packet_handshake')
    unit = power_check.PowerCheck(port=None).get_unit_info()
    mock_handshake.assert_called_once_with(
        0xf1, power_check.PowerCheck.UNIT_INFO_FMT, power_check.PowerCheck.UnitInfo)
    
def test_stream_status(mocker):
    status_packet = power_check.PowerCheck.Status(
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13)
    mock_status = mocker.patch('power_check2.PowerCheck.get_status',
                               return_value=status_packet)
    mock_time = mocker.patch('power_check2.time.time', return_value=1)
    mock_sleep = mocker.patch('power_check2.time.sleep')
    unit = power_check.PowerCheck(port=None)
    unit._monitor_file = mock.Mock()
    stream_values_expected = 2
    stream_sleep = 0.1
    unit.stream_status(stream_sleep, stream_values_expected)
    assert unit._monitor_file.write.call_count == stream_values_expected
    mock_sleep.assert_has_calls(2 * [mock.call(stream_sleep)])
