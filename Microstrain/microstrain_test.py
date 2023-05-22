from unittest import mock
import microstrain
import pytest

# Mock out serial for all tests since there is no HW involved.
@pytest.fixture(autouse=True)
def serial_mock():
    with mock.patch.object(microstrain.serial, 'Serial') as _fixture:
        yield _fixture

def test_mips_field_as_bytes_int_construction():
    sample = microstrain.MipsField(0x06, 0xff, [0x0f, 0x0d, 0x0b, 0x09])
    assert sample.as_bytes == b'\x06\xff\x0f\x0d\x0b\x09'

def test_mips_data_from_field_bytes_construction():
    sample = microstrain.MipsField(0x06, 0x11, data_bytes=b'\x02\x04\x06\x08')
    assert sample.data == [0x02, 0x04, 0x06, 0x08]

def test_mips_field_as_iter():
    sample = microstrain.MipsField(0x06, 0x11, data_bytes=b'\x02\x04\x06\x08')
    assert list(sample) == [0x06, 0x11, 0x02, 0x04, 0x06, 0x08]

@pytest.mark.parametrize(
    'payload_bytes, field_list', [
        # Device reset example - single field case.
        (b'\x04\xf1\x7e\x00', [
            microstrain.MipsField(0x04, 0xf1, [0x7e, 0x00])
        ]),
        # Save IMU/estimation format - multiple field case.
        (b'\x04\x08\x03\x00\x04\x0a\x03\x00',
         [microstrain.MipsField(0x04, 0x08, [0x03, 0x00]),
          microstrain.MipsField(0x04, 0x0a, [0x03, 0x00])]),
        # Initialize attitude - note data must be passed in one byte at
        # a time, even though the 3 values are each 4 bytes.
        (b'\x0e\x02\xba\xe3\xed\x9b\x3c\x7d\x6d\xdf\xbf\x85\x5c\xf5', [
            microstrain.MipsField(0x0e, 0x02, [0xba, 0xe3, 0xed, 0x9b,
                                               0x3c, 0x7d, 0x6d, 0xdf,
                                               0xbf, 0x85, 0x5c, 0xf5])]),
        ]
)
def test_split_fields_from_payload_data(payload_bytes, field_list):
    unit = microstrain.Microstrain3DM()
    result = unit._split_payload_to_fields(payload_bytes)
    assert result == field_list

@pytest.mark.parametrize(
    'packet, expected_checksum', [
        (b'\x75\x65\x01\x02\x02\x01', b'\xe0\xc6'),  # Ping example.
        (b'\x75\x65\x01\x04\x04\xf1\x01\x00', b'\xd5\x6a'),  # Ping response.
        (b'\x75\x65\x0c\x0a\x0a\x01\x00\x02\x04\x00\x00\x05\x00\x00',
         b'\x06\x27')  # Poll IMU.
    ]
)
def test_checksum_calculation(packet, expected_checksum):
    check_sum = microstrain.fletcher_checksum(packet)
    assert check_sum == expected_checksum

def test_read_one_packet():
    packet = b'\x01\x0a\x04\xf1\x05\x00\x06\x83\x01\x02\x03\x04\x68\x7d'
    unit = microstrain.Microstrain3DM()
    # Slice the read sequence from the packet to make sure they all are
    # read as bytes instead of ints.
    return_vals = (packet[:1], packet[1:2], packet[2:-2], packet[-2:])
    unit._ser.read.side_effect = return_vals
    result = unit._read_one_packet()
    mips_expected = microstrain.MipsPacket(0x01,[
        microstrain.MipsField(0x04, 0xf1, [0x05, 0x00]),
        microstrain.MipsField(0x06, 0x83, [1, 2, 3, 4])])
    assert result == mips_expected

def test_ping_command(mocker):
    error_expected = 0x00
    # Test from the read level to make sure the helper _send_and_parse_reply
    # works.
    mock_read = mocker.patch('microstrain.Microstrain3DM._read_one_packet')
    mock_read.return_value = microstrain.MipsPacket(
        b'\x01', [microstrain.MipsField(0x04, 0xf1,
                                        data=[0x01, error_expected])])
    unit = microstrain.Microstrain3DM()
    response = unit.device_ping()  # Ack response received.
    unit._ser.write.assert_called_with(b'\x75\x65\x01\x02\x02\x01\xe0\xc6')
    assert response.error_code == error_expected

# Each of these byte streams come from the examples in the data communication
# protocol manual. Make sure each command sends the proper byte sequence. The
# complicated response parsing are handled in other test cases.
@pytest.mark.parametrize(
    'command, expected_bytes', [
        ('device_reset', b'\x75\x65\x01\x02\x02\x7e\x5d\x43'),
        ('device_idle', b'\x75\x65\01\x02\x02\x02\xe1\xc7'),
        ('device_resume', b'\x75\x65\x01\x02\x02\x06\xe5\xcb'),
        ('device_info', b'\x75\x65\x01\x02\x02\x03\xe2\xc8'),
        ('device_built_in_test', b'\x75\x65\x01\x02\x02\x05\xe4\xca')
    ]
)
def test_device_command(mocker, command, expected_bytes):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    getattr(unit, command)()
    send_packet = mock_send.call_args[0][0]
    assert send_packet.as_bytes == expected_bytes   
