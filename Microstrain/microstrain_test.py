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
    'msg_type, field_len, field_desc, data_bytes, expect', [
        (microstrain.DataMessages.IMU.msg_ind, 0x0e, 0x04,
         b'\x3e\x7a\x63\xa0xbb\x8e\x3b\x29\x7f\xe5\xbf7f',
         ['Accel g', 'Accel g', 'Accel g']),
        (microstrain.DataMessages.GPS.msg_ind, 0x08, 0x0b,
         b'\x00\x01\x00\x02\x00\x03', ['n/a', 'count', 'n/a', 'n/a'])]
)
def test_get_units_for_data_message(msg_type, field_len, field_desc, data_bytes,
                                    expect):
    field_data = microstrain.MipsField(field_len, field_desc,
                                       data_bytes=data_bytes)
    units = microstrain.ReplyFormats.get_field_units(msg_type, field_data)
    assert units == expect

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

def test_process_mips_packet():
    imu_fields = [
        microstrain.MipsField(
        0x0e, 0x04, data_bytes=b'?\x80\x00\x00@\x00\x00\x00@@\x00\x00'),
        microstrain.MipsField(0x06, 0x17, data_bytes=b'?\x80\x00\x00'),
        microstrain.MipsField(
        0x12, 0x0a, data_bytes=b'?\x80\x00\x00' + 12*b'\x00')]
    gps_fields = [microstrain.MipsField(0x08, 0x0b, data = [0, 1, 0, 2, 0, 3])]
    imu_desc = microstrain.DataMessages.IMU.msg_ind
    gps_desc = microstrain.DataMessages.GPS.msg_ind
    packets = [microstrain.MipsPacket(imu_desc, imu_fields),
               microstrain.MipsPacket(gps_desc, gps_fields)]
    result = microstrain.process_mips_packets(packets)
    assert result == [microstrain.ReplyFormats.Vector(1.0, 2.0, 3.0),
                      microstrain.ReplyFormats.AmbPressure(1.0),
                      microstrain.ReplyFormats.Quat(1.0, 0, 0, 0),
                      microstrain.ReplyFormats.GPSFix(0, 1, 2, 3)]
    
def test_flatten_payload():
    gps_fields = [
        microstrain.MipsField(0x08, 0x0b, data = [0, 1, 0, 2, 0, 3]),
        microstrain.MipsField(0x0f, 0x08,
                              data=[0, 4, 5, 6, 7, 8, 9, 0, 0, 0, 10, 0, 11])]
    gps_desc = microstrain.DataMessages.GPS.msg_ind
    packet = microstrain.MipsPacket(gps_desc, gps_fields)
    assert packet.flatten_payload() == [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

def test_divide_mips_packets():
    imu_packet = microstrain.MipsPacket(
        microstrain.DataMessages.IMU.msg_ind,
        [microstrain.MipsField(0x06, 0x17, data_bytes=b'?\x80\x00\x00')])
    gps_packet = microstrain.MipsPacket(
        microstrain.DataMessages.GPS.msg_ind,
        [microstrain.MipsField(0x08, 0x0b, data = [0, 1, 0, 2, 0, 3])]
    )
    ekf_packet = microstrain.MipsPacket(
        microstrain.DataMessages.EKF.msg_ind,
        [microstrain.MipsField(0x02, 0xff)]
    )
    packets = [imu_packet, gps_packet, ekf_packet, gps_packet, imu_packet,
               imu_packet]
    imu, gps, ekf = microstrain.divide_mips_packets(packets)
    assert (len(imu), len(gps), len(ekf)) == (3, 2, 1)

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
        0x01, [microstrain.MipsField(0x04, 0xf1,
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

def test_set_imu_format(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.set_msg_fmt(microstrain.DataMessages.IMU, [0x04, 0x05], [50.])
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0c\x0a\x0a\x08\x01\x02\x04\x00\x0a\x05\x00\x0a\x22\xa0'
    assert send_packet.as_bytes == expect

def test_set_gps_format(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.set_msg_fmt(microstrain.DataMessages.GPS, [0x03, 0x05], [1.])
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0c\x0a\x0a\x09\x01\x02\x03\x00\x04\x05\x00\x04\x16\x85'
    assert send_packet.as_bytes == expect

def test_set_estimation_filter_format(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.set_msg_fmt(microstrain.DataMessages.EKF, [0x01, 0x02], [500.])
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0c\x0a\x0a\x0a\x01\x02\x01\x00\x01\x02\x00\x01\x0c\x6a'
    assert send_packet.as_bytes == expect  

def test_get_msg_format_for_gps(mocker):
    mock_cmd = mocker.patch('microstrain.Microstrain3DM._send_command')
    mock_cmd.return_value = microstrain.MipsPacket(0x0c,[
        microstrain.MipsField(0x04, 0xf1, [0x09, 0x00]),
        microstrain.MipsField(0x06, 0x81, [0x01, 0x03, 0x00, 0x04])
    ])
    unit = microstrain.Microstrain3DM()
    formats = unit.get_msg_fmt(microstrain.DataMessages.GPS)
    send_packet = mock_cmd.call_args[0][0]
    expect = b'\x75\x65\x0c\x04\x04\x09\x02\x00\xf9\xf6'
    assert send_packet.as_bytes == expect
    assert formats == [microstrain.ReplyFormats.DataFmt(0x03, 0x04)]

def test_save_msg_format_for_estimation_filter(mocker):
    mock_cmd = mocker.patch('microstrain.Microstrain3DM._send_command')
    unit = microstrain.Microstrain3DM()
    unit.save_msg_fmt(microstrain.DataMessages.EKF)
    send_packet = mock_cmd.call_args[0][0]
    expect = b'\x75\x65\x0c\x04\x04\x0a\x03\x00\xfb\xfb'
    assert send_packet.as_bytes == expect

def test_load_msg_format_for_imu(mocker):
    mock_cmd = mocker.patch('microstrain.Microstrain3DM._send_command')
    unit = microstrain.Microstrain3DM()
    unit.load_msg_fmt(microstrain.DataMessages.IMU)
    send_packet = mock_cmd.call_args[0][0]
    expect = b'\x75\x65\x0c\x04\x04\x08\x04\x00\xfa\xf7'
    assert send_packet.as_bytes == expect

def test_reset_msg_format_for_gps(mocker):
    mock_cmd = mocker.patch('microstrain.Microstrain3DM._send_command')
    unit = microstrain.Microstrain3DM()
    unit.reset_msg_fmt(microstrain.DataMessages.GPS)
    send_packet = mock_cmd.call_args[0][0]
    expect = b'\x75\x65\x0c\x04\x04\x09\x05\x00\xfc\xfc'
    assert send_packet.as_bytes == expect

def test_poll_data_for_estimation_filter(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.poll_data(microstrain.DataMessages.EKF)
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0c\x04\x04\x03\x01\x00\xf2\xe2'
    assert send_packet.as_bytes == expect

def test_create_headers_for_file():
    unit = microstrain.Microstrain3DM()
    header = unit._create_file_header_for_fmt(
        microstrain.DataMessages.IMU, {1.0: [0x04, 0x06], 2.0: [0x17]})
    expect = {1.0: ['x (Accel g)', 'y (Accel g)', 'z (Accel g)',
                    'x (Mag gauss)', 'y (Mag gauss)', 'z (Mag gauss)'],
              2.0: ['pressure (milliBar)']}
    assert header == expect

def test_collect_data_stream(mocker):
    mock_resume = mocker.patch('microstrain.Microstrain3DM.device_resume')
    mock_idle = mocker.patch('microstrain.Microstrain3DM.device_idle')
    mocker.patch('microstrain.Microstrain3DM._read_one_packet')
    mock_time = mocker.patch('microstrain.time')
    mock_time.time.side_effect = [0, 1, 2]
    unit = microstrain.Microstrain3DM()
    unit._ser.in_waiting = False
    unit.collect_data_stream(2)
    # Really want to ensure that the device is managed correctly.
    mock_resume.assert_called_once()
    mock_idle.assert_called_once()

def test_write_stream_data(mocker):
    mock_open = mocker.patch('builtins.open')
    unit = microstrain.Microstrain3DM()
    unit._stream_results = [
        microstrain.MipsPacket(
            microstrain.DataMessages.IMU.msg_ind,
            [microstrain.MipsField(0x06, 0x17, data_bytes=b'?\x80\x00\x00')])]
    unit._imu_fmt = {1.0: [0x17]}
    unit.write_stream_data()
    # Make sure import info in file name.
    file_destination = mock_open.call_args[0][0]
    assert all(['1.0' in file_destination, 'imu' in file_destination])
    
