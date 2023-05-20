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
    # These are generic settings for the device connection.
    parser.add_argument('-baud', choices=[9600, 19200, 38400, 57600, 115200], type=int,
                        default=115200, help='Baud rate for device.')
    parser.add_argument('-port', default='COM5',
                        help='Specify port for machine (COM3 for windows, /dev/PwrCheck linux)')

    # This is for debugging, to see the raw traffic on serial during the run.
    parser.add_argument('-raw', action='store_true',
                        help='Dump raw byte read/write transactions from session.') 
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

    if args.raw:
        print('Writes from command.')
        print(unit.dump_write_buffer())
        print('Reads from command.')
        print(unit.dump_read_buffer())
