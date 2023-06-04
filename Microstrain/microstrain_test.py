from unittest import mock
import microstrain
import packets
import pytest

# Mock out serial for all tests since there is no HW involved.
@pytest.fixture(autouse=True)
def serial_mock():
    with mock.patch.object(microstrain.serial, 'Serial') as _fixture:
        yield _fixture

def test_read_one_packet():
    packet = b'\x01\x0a\x04\xf1\x05\x00\x06\x83\x01\x02\x03\x04\x68\x7d'
    unit = microstrain.Microstrain3DM()
    # Slice the read sequence from the packet to make sure they all are
    # read as bytes instead of ints.
    return_vals = (packet[:1], packet[1:2], packet[2:-2], packet[-2:])
    unit._ser.read.side_effect = return_vals
    result = unit._read_one_packet()
    mips_expected = packets.MipsPacket(0x01,[
        packets.MipsField(0x04, 0xf1, [0x05, 0x00]),
        packets.MipsField(0x06, 0x83, [1, 2, 3, 4])])
    assert result == mips_expected

def test_ping_command(mocker):
    error_expected = 0x00
    # Test from the read level to make sure the helper _send_and_parse_reply
    # works.
    mock_read = mocker.patch('microstrain.Microstrain3DM._read_one_packet')
    mock_read.return_value = packets.MipsPacket(
        0x01, [packets.MipsField(0x04, 0xf1,
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

def test_device_base_rate(mocker):
    mock_cmd = mocker.patch('microstrain.Microstrain3DM._send_command')
    mock_cmd.return_value = packets.MipsPacket(
        0x0c,
        [packets.MipsField(0x04, 0xf1, [0x06, 0x00]),
         packets.MipsField(0x04, 0x83, [0x00, 0x64])])
    unit = microstrain.Microstrain3DM()
    rate = unit.device_base_rate(packets.DataMessages.IMU)
    cmd_packet = mock_cmd.call_args[0][0]
    assert cmd_packet.as_bytes == b'\x75\x65\x0c\x02\x02\x06\xf0\xf7'
    assert rate == 100

def test_set_baud_rate(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit._baud_rate = 9600
    unit.device_baud(115200)
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0c\x07\x07\x40\x01\x00\x01\xc2\x00\xf8\xda'
    assert send_packet.as_bytes == expect 

def test_set_baud_raises_for_invalid_rate():
    with pytest.raises(ValueError):
        microstrain.Microstrain3DM().device_baud(0)

def test_set_imu_format(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.set_msg_fmt(packets.DataMessages.IMU, [0x04, 0x05], [50.])
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0c\x0a\x0a\x08\x01\x02\x04\x00\x0a\x05\x00\x0a\x22\xa0'
    assert send_packet.as_bytes == expect

def test_set_gps_format(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.set_msg_fmt(packets.DataMessages.GPS, [0x03, 0x05], [1.])
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0c\x0a\x0a\x09\x01\x02\x03\x00\x04\x05\x00\x04\x16\x85'
    assert send_packet.as_bytes == expect

def test_set_estimation_filter_format(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.set_msg_fmt(packets.DataMessages.EKF, [0x01, 0x02], [500.])
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0c\x0a\x0a\x0a\x01\x02\x01\x00\x01\x02\x00\x01\x0c\x6a'
    assert send_packet.as_bytes == expect

def test_set_dynamics_mode(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.set_dynamics_mode(microstrain.EkfDynamicsMode.PORTABLE,
                           microstrain.FunctionSelectors.NEW)
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0d\x04\x04\x10\x01\x01\x01\x10'
    assert send_packet.as_bytes == expect    

def test_get_msg_format_for_gps(mocker):
    mock_cmd = mocker.patch('microstrain.Microstrain3DM._send_command')
    mock_cmd.return_value = packets.MipsPacket(0x0c,[
        packets.MipsField(0x04, 0xf1, [0x09, 0x00]),
        packets.MipsField(0x06, 0x81, [0x01, 0x03, 0x00, 0x04])
    ])
    unit = microstrain.Microstrain3DM()
    formats = unit.get_msg_fmt(packets.DataMessages.GPS)
    send_packet = mock_cmd.call_args[0][0]
    expect = b'\x75\x65\x0c\x04\x04\x09\x02\x00\xf9\xf6'
    assert send_packet.as_bytes == expect
    assert formats == [packets.ReplyFormats.DataFmt(0x03, 0x04)]

def test_save_msg_format_for_estimation_filter(mocker):
    mock_cmd = mocker.patch('microstrain.Microstrain3DM._send_command')
    unit = microstrain.Microstrain3DM()
    unit.save_msg_fmt(packets.DataMessages.EKF)
    send_packet = mock_cmd.call_args[0][0]
    expect = b'\x75\x65\x0c\x04\x04\x0a\x03\x00\xfb\xfb'
    assert send_packet.as_bytes == expect

def test_load_msg_format_for_imu(mocker):
    mock_cmd = mocker.patch('microstrain.Microstrain3DM._send_command')
    unit = microstrain.Microstrain3DM()
    unit.load_msg_fmt(packets.DataMessages.IMU)
    send_packet = mock_cmd.call_args[0][0]
    expect = b'\x75\x65\x0c\x04\x04\x08\x04\x00\xfa\xf7'
    assert send_packet.as_bytes == expect

def test_reset_msg_format_for_gps(mocker):
    mock_cmd = mocker.patch('microstrain.Microstrain3DM._send_command')
    unit = microstrain.Microstrain3DM()
    unit.reset_msg_fmt(packets.DataMessages.GPS)
    send_packet = mock_cmd.call_args[0][0]
    expect = b'\x75\x65\x0c\x04\x04\x09\x05\x00\xfc\xfc'
    assert send_packet.as_bytes == expect

def test_poll_data_for_estimation_filter(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.poll_data(packets.DataMessages.EKF)
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0c\x04\x04\x03\x01\x00\xf2\xe2'
    assert send_packet.as_bytes == expect

def test_poll_data_for_imu_format(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.poll_data(packets.DataMessages.IMU, suppress_nack=0,
                   descriptors=[0x04, 0x05])
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0c\x0a\x0a\x01\x00\x02\x04\x00\x00\x05\x00\x00\x06\x27'
    assert send_packet.as_bytes == expect   

def test_ekf_auto_initialize(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.ekf_auto_init(microstrain.FunctionSelectors.NEW, 1)
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0d\x04\x04\x19\x01\x01\x0a\x2b'
    assert send_packet.as_bytes == expect

def test_ekf_euler_initialize(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.ekf_euler_init(packets.ReplyFormats.Vector(0, 0, 0))
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0d\x0e\x0e\x02' + 12*b'\x00' + b'\x05\x6f'
    assert send_packet.as_bytes == expect

def test_ekf_heading_update_source(mocker):
    mock_send = mocker.patch('microstrain.Microstrain3DM._send_and_parse_reply')
    unit = microstrain.Microstrain3DM()
    unit.ekf_heading_source(microstrain.FunctionSelectors.NEW,
                            microstrain.EkfHeadingUpdate.MAG)
    send_packet = mock_send.call_args[0][0]
    expect = b'\x75\x65\x0d\x04\x04\x18\x01\x01\x09\x28'
    assert send_packet.as_bytes == expect

def test_capture_gyro_bias_raises_for_out_of_range_value(mocker):
    with pytest.raises(ValueError):
        microstrain.Microstrain3DM().capture_gyro_bias(0)

def test_capture_gyro_bias(mocker):
    mock_cmd = mocker.patch('microstrain.Microstrain3DM._send_command')
    mock_sleep = mocker.patch('microstrain.time.sleep')
    unit = microstrain.Microstrain3DM()
    unit.capture_gyro_bias(0x2710)  # 10 seconds.
    cmd_packet = mock_cmd.call_args[0][0]
    expect = b'\x75\x65\x0c\x04\x04\x39\x27\x10\x5e\xe0'
    assert cmd_packet.as_bytes == expect

def test_create_headers_for_file():
    unit = microstrain.Microstrain3DM()
    header = unit._create_file_header_for_fmt(
        packets.DataMessages.IMU, {1.0: [0x04, 0x06], 2.0: [0x17]})
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
        packets.MipsPacket(
            packets.DataMessages.IMU.msg_ind,
            [packets.MipsField(0x06, 0x17, data_bytes=b'?\x80\x00\x00')])]
    unit._imu_fmt = {1.0: [0x17]}
    unit.write_stream_data()
    # Make sure import info in file name.
    file_destination = mock_open.call_args[0][0]
    assert all(['1.0' in file_destination, 'imu' in file_destination])
    
