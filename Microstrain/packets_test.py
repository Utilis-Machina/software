from unittest import mock
import packets
import pytest

def test_mips_field_as_bytes_int_construction():
    sample = packets.MipsField(0x06, 0xff, [0x0f, 0x0d, 0x0b, 0x09])
    assert sample.as_bytes == b'\x06\xff\x0f\x0d\x0b\x09'

def test_mips_data_from_field_bytes_construction():
    sample = packets.MipsField(0x06, 0x11, data_bytes=b'\x02\x04\x06\x08')
    assert sample.data == [0x02, 0x04, 0x06, 0x08]

def test_mips_field_as_iter():
    sample = packets.MipsField(0x06, 0x11, data_bytes=b'\x02\x04\x06\x08')
    assert list(sample) == [0x06, 0x11, 0x02, 0x04, 0x06, 0x08]

@pytest.mark.parametrize('name', ['imu', 'gps', 'ekf'])
def test_formats_and_units_same_length(name):
    fmts = getattr(packets.ReplyFormats, name + '_set')
    units = getattr(packets.ReplyFormats, name + '_units')
    for ind, vals in units.items():
        num_units = len(vals)
        cur_fmt, cur_obj = fmts[ind]
        # We want to ignore things like the byte symbol, and numbers that
        # specify the byte length of strings when comparing the two. Thus apply
        # a filter.
        num_fmts = len(list(filter(str.isalpha, cur_fmt)))
        # To unpack properly we need one field per number returned.
        num_fields = len(cur_obj._fields)
        assert num_fmts == num_units, f'{name}[{ind}] mismatch!'
        assert num_fields == num_units, f'{name}[{ind}] field mismatch!'

@pytest.mark.parametrize(
    'msg_type, field_len, field_desc, data_bytes, expect', [
        (packets.DataMessages.IMU.msg_ind, 0x0e, 0x04,
         b'\x3e\x7a\x63\xa0xbb\x8e\x3b\x29\x7f\xe5\xbf7f',
         ['Accel g', 'Accel g', 'Accel g']),
        (packets.DataMessages.GPS.msg_ind, 0x08, 0x0b,
         b'\x00\x01\x00\x02\x00\x03', ['n/a', 'count', 'n/a', 'n/a']),
         (packets.DataMessages.EKF.msg_ind, 0x10, 0x30,
          14*b'\x00', ['m', 'm', 'm', 'n/a'])]
)
def test_get_units_for_data_message(msg_type, field_len, field_desc, data_bytes,
                                    expect):
    field_data = packets.MipsField(field_len, field_desc,
                                       data_bytes=data_bytes)
    units = packets.ReplyFormats.get_field_units(msg_type, field_data)
    assert units == expect

def test_decode_ekf_status_init_mode():
    status = packets.ReplyFormats.FilterState(1, 1, 0x3000)
    result = packets.ReplyFormats.decode(status)
    assert len(result) == 2

def test_decode_ekf_status_run_mode():
    status = packets.ReplyFormats.FilterState(2, 1, 0x2ffb)
    result = packets.ReplyFormats.decode(status)
    assert len(result) == 12

def test_decode_gps_fix_info():
    status = packets.ReplyFormats.GPSFix(0, 7, 0, 7)
    result = packets.ReplyFormats.decode(status)
    assert len(result) == 3

def test_decode_gps_hw_status():
    status = packets.ReplyFormats.GpsHwStatus(1, 4, 1, 7)
    result = packets.ReplyFormats.decode(status)
    assert len(result) == 7

@pytest.mark.parametrize(
    'payload_bytes, field_list', [
        # Device reset example - single field case.
        (b'\x04\xf1\x7e\x00', [
            packets.MipsField(0x04, 0xf1, [0x7e, 0x00])
        ]),
        # Save IMU/estimation format - multiple field case.
        (b'\x04\x08\x03\x00\x04\x0a\x03\x00',
         [packets.MipsField(0x04, 0x08, [0x03, 0x00]),
          packets.MipsField(0x04, 0x0a, [0x03, 0x00])]),
        # Initialize attitude - note data must be passed in one byte at
        # a time, even though the 3 values are each 4 bytes.
        (b'\x0e\x02\xba\xe3\xed\x9b\x3c\x7d\x6d\xdf\xbf\x85\x5c\xf5', [
            packets.MipsField(0x0e, 0x02, [0xba, 0xe3, 0xed, 0x9b,
                                               0x3c, 0x7d, 0x6d, 0xdf,
                                               0xbf, 0x85, 0x5c, 0xf5])]),
        ]
)
def test_split_fields_from_payload_data(payload_bytes, field_list):
    result = packets.split_payload_to_fields(payload_bytes)
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
    check_sum = packets.fletcher_checksum(packet)
    assert check_sum == expected_checksum

def test_process_mips_packet():
    imu_fields = [
        packets.MipsField(
        0x0e, 0x04, data_bytes=b'?\x80\x00\x00@\x00\x00\x00@@\x00\x00'),
        packets.MipsField(0x06, 0x17, data_bytes=b'?\x80\x00\x00'),
        packets.MipsField(
        0x12, 0x0a, data_bytes=b'?\x80\x00\x00' + 12*b'\x00')]
    gps_fields = [packets.MipsField(0x08, 0x0b, data = [0, 1, 0, 2, 0, 3])]
    imu_desc = packets.DataMessages.IMU.msg_ind
    gps_desc = packets.DataMessages.GPS.msg_ind
    packet_list = [packets.MipsPacket(imu_desc, imu_fields),
               packets.MipsPacket(gps_desc, gps_fields)]
    result = packets.process_mips_packets(packet_list)
    assert result == [packets.ReplyFormats.Vector(1.0, 2.0, 3.0),
                      packets.ReplyFormats.AmbPressure(1.0),
                      packets.ReplyFormats.Quat(1.0, 0, 0, 0),
                      packets.ReplyFormats.GPSFix(0, 1, 2, 3)]
    
def test_flatten_payload():
    gps_fields = [
        packets.MipsField(0x08, 0x0b, data = [0, 1, 0, 2, 0, 3]),
        packets.MipsField(0x0f, 0x08,
                              data=[0, 4, 5, 6, 7, 8, 9, 0, 0, 0, 10, 0, 11])]
    gps_desc = packets.DataMessages.GPS.msg_ind
    packet = packets.MipsPacket(gps_desc, gps_fields)
    assert packet.flatten_payload() == [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

def test_divide_mips_packets():
    imu_packet = packets.MipsPacket(
        packets.DataMessages.IMU.msg_ind,
        [packets.MipsField(0x06, 0x17, data_bytes=b'?\x80\x00\x00')])
    gps_packet = packets.MipsPacket(
        packets.DataMessages.GPS.msg_ind,
        [packets.MipsField(0x08, 0x0b, data = [0, 1, 0, 2, 0, 3])]
    )
    ekf_packet = packets.MipsPacket(
        packets.DataMessages.EKF.msg_ind,
        [packets.MipsField(0x02, 0xff)]
    )
    packet_list = [imu_packet, gps_packet, ekf_packet, gps_packet, imu_packet,
               imu_packet]
    imu, gps, ekf = packets.divide_mips_packets(packet_list)
    assert (len(imu), len(gps), len(ekf)) == (3, 2, 1)
