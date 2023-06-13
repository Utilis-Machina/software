r"""Simple binary to test microstrain IMU library with HW.

If testing on windows you will need to make sure you have a working
driver first. There is one available for download on the LORD Microstrain
site - see the README markdown for more details.

The hardware just needs to be connected via USB or USB->RS232 converter
in order to run the code. Verify the correct port to use and pass it in
as an argument if needed.
"""
import argparse
import microstrain
import packets
import sys

LICENSE_INFO = """
run_microstrain  Copyright (C) 2023 Stephen Sulack
This program comes with ABSOLUTELY NO WARRANTY.
This is free software, and you are welcome to redistribute it
under certain conditions; see the license file on github for details.
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    # Note value will be stored under first argument provided.
    # Basic commands for device.
    parser.add_argument('-ping', action='store_true',
                        help='Pings device to verify communication.')
    parser.add_argument('-info', action='store_true',
                        help='Get model and serial info from device.')
    parser.add_argument('-test', action='store_true',
                        help='Run built-in test and print results.')
    parser.add_argument('-rates', action='store_true',
                        help='Get base rates for device.')
    parser.add_argument('-baud_test', action='store_true',
                        help='Test setting different baud rates.')
    # IMU commands.
    parser.add_argument('-imu', action='store_true',
                        help='Get accelerometer data from IMU.')
    parser.add_argument('-imu_all', action='store_true',
                        help='Get all data from IMU.')
    parser.add_argument('-imu_stream', action='store_true',
                        help='Stream data from IMU.')
    parser.add_argument('-sample_sec', type=float, default=5.0,
                        help='Number of seconds to collect data for.')
    parser.add_argument('-pressure', action='store_true',
                        help='Get pressure data from IMU.') 
    # GPS commands.
    parser.add_argument('-gps', action='store_true',
                        help='Get GPS data from unit.')
    parser.add_argument('-gps_all', action='store_true',
                        help='Get all data from GPS.')
    # EKF commands.
    parser.add_argument('-ekf', action='store_true',
                        help='Get EKF data from unit.')
    parser.add_argument('-ekf_euler_init', action='store_true',
                        help='Initialize EKF with Euler angles.')
    parser.add_argument('-ekf_all', action='store_true',
                        help='Get all EKF data from unit.')
    # Field testing cases.
    parser.add_argument('-stream_test', action='store_true',
                        help='Stream data to file in multiple formats.')
    parser.add_argument('-stream_reset', action='store_true',
                        help='Reset message formats on device.')
    parser.add_argument('-stream_high_rate', action='store_true',
                        help='Collect high rate data from IMU.')
    parser.add_argument('-gyro_bias', action='store_true',
                        help='Collect and store gyro bias for IMU.')    
    # These are generic settings for the device connection.
    parser.add_argument('-baud', choices=[9600, 19200, 38400, 57600, 115200,
                                          230400, 460800, 921600],
                        type=int, default=115200, help='Baud rate for device.')
    parser.add_argument('-port', default='COM5',
                        help=('Specify port for machine (COM3 for windows, '
                              '/dev/ustrain linux)'))
    # This is for debugging, to see the raw traffic on serial during the run.
    parser.add_argument('-raw', action='store_true',
                        help='Dump raw byte read/writes from session.') 
    args = parser.parse_args()

    # If no arguments, print copyright notice and help.
    if len(sys.argv) == 1:
        print(LICENSE_INFO)
        parser.print_help()
        exit(0)

    unit = microstrain.Microstrain3DM(port=args.port, baudrate=args.baud)
    if args.ping:
        print(f'Ping device and return error codes.')
        print(f'Device Response: {unit.device_ping()}')
    if args.info:
        print(f'Get info from device.')
        print(unit.device_info())
    if args.test:
        print(f'Running built-in-test for device.')
        print(f'Result: {unit.device_built_in_test()}')
    if args.rates:
        print('Collecting base rates for device.')
        imu = unit.device_base_rate(packets.DataMessages.IMU)
        gps = unit.device_base_rate(packets.DataMessages.GPS)
        ekf = unit.device_base_rate(packets.DataMessages.EKF)
        print(f'IMU: {imu}, GPS: {gps}, EKF: {ekf} Hz')
    if args.baud_test:
        new_rate = 460800
        print(f'Setting baud rate to {new_rate}')
        unit.device_baud(new_rate)
        print('Rate changed, trying ping. Note - persists until powered off.')
        print(f'{unit.device_ping()}')
    if args.imu:
        print(f'Collecting data from IMU.')
        unit.set_msg_fmt(descriptors=[0x04, 0x11])
        print(f'Result: {unit.poll_data()}')
    if args.pressure:
        print(f'Checking ambient pressure.')
        unit.set_msg_fmt(descriptors=[0x17])
        print(f'Result: {unit.poll_data()}')
    if args.imu_all:
        print(f'Collecting example data from IMU.')
        unit.set_msg_fmt(descriptors=[4, 5, 6, 7, 8, 9, 10, 12, 16, 17, 18, 23],
                         rate_hz = [1.0])
        print(f'Result: {unit.poll_data()}')
    if args.imu_stream:
        print(f'Streaming IMU data for {args.sample_sec} seconds.')
        unit.set_msg_fmt(descriptors=[0x04, 0x10], rate_hz = [1.0, 2.0])
        data = unit.collect_data_stream(args.sample_sec)
        print(f'Received {len(data)} samples.')
        processed_data = packets.process_mips_packets(data)
        print(f'Result: {processed_data}')
    if args.stream_reset:
        print(f'Reset message formats.')
        unit.reset_msg_fmt(packets.DataMessages.IMU)
        unit.reset_msg_fmt(packets.DataMessages.GPS)
        unit.reset_msg_fmt(packets.DataMessages.EKF)
    if args.stream_test:
        print(f'Streaming data for {args.sample_sec} seconds.')
        unit.set_msg_fmt(packets.DataMessages.IMU,
                         [0x04, 0x10], rate_hz = [1.0, 2.0])
        unit.set_msg_fmt(packets.DataMessages.GPS, [0x03], [1.0])
        unit.set_msg_fmt(packets.DataMessages.EKF,
                         [0x10, 0x06, 0x0b], [5.0])
        data = unit.collect_data_stream(args.sample_sec)
        unit.write_stream_data()
        print(f'Results written to file.')
    if args.stream_high_rate:
        print(f'Streaming data for {args.sample_sec} seconds.')
        unit.set_msg_fmt(packets.DataMessages.IMU,
                         [0x04], rate_hz = [50.])
        data = unit.collect_data_stream(args.sample_sec)
        unit.write_stream_data()
        print(f'Results written to file.')
    if args.gps:
        print(f'Collecting example data from GPS.')
        unit.set_msg_fmt(packets.DataMessages.GPS, [4], [1.0])
        print(f'Result: {unit.poll_data(packets.DataMessages.GPS)}')
    if args.gps_all:
        print(f'Collecting example data from GPS.')
        unit.set_msg_fmt(packets.DataMessages.GPS,
                         [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], rate_hz = [0.1])
        print(f'Result: {unit.poll_data(packets.DataMessages.GPS)}')
    if args.ekf:
        print(f'Collecting EKF data.')
        unit.set_msg_fmt(packets.DataMessages.EKF, [0x10], [1.0])
        data = unit.poll_data(packets.DataMessages.EKF)[0]
        print(f'{data}')
        data_messages = packets.ReplyFormats.decode(data)
        print(f'Result: {data_messages}')
    if args.ekf_euler_init:
        print(f'Initialize EKF with Euler angles.')
        unit.ekf_euler_init()
    if args.ekf_all:
        print(f'Collecting lots of EKF data.')
        unit.set_msg_fmt(packets.DataMessages.EKF,
                         [16, 17, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
                          12, 13, 14, 15, 18, 19, 20, 21, 22, 23, 24], [1.0])
        data = unit.poll_data(packets.DataMessages.EKF)
        print(f'{data}')            
    if args.gyro_bias:
        print(f'Sampling gyro bias for device for {args.sample_sec} seconds.')
        bias = unit.capture_gyro_bias(int(args.sample_sec * 1e3))
        print(bias)

    if args.raw:
        print('Writes from command.')
        print(unit.dump_write_buffer())
        print('Reads from command.')
        print(unit.dump_read_buffer())
