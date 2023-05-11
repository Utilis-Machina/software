r"""Simple binary to test power_check functionality with hardware."""
import argparse
import power_check
import sys
import time

LICENSE_INFO = """
run_power_check  Copyright (C) 2023 Stephen Sulack
This program comes with ABSOLUTELY NO WARRANTY.
This is free software, and you are welcome to redistribute it
under certain conditions; see the license file for details.
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    # Note value will be stored under first name provided.
    parser.add_argument('-config', action='store_true',
                        help='Prints config on device.')
    parser.add_argument('-status', '-r', action='store_true',
                        help='Returns current measurements from system.')
    parser.add_argument('-info', action='store_true',
                        help='Prints firmware info of device.')
    parser.add_argument('-reset', action='store_true',
                        help='Clears log data from device.')   
    parser.add_argument('-set_logging_sec', type=int, dest='logging_sec',
                        help='Sets logging interval')
    parser.add_argument('-over_voltage', type=int, dest='over_voltage',
                        help='Sets over voltage alarm limit.')
    parser.add_argument('-dump', '-d', action='store_true',
                        help='Dumps logged data from EEPROM.')
    parser.add_argument('-stream_status', action='store_true',
                        help='Logs and writes data to file.')
    parser.add_argument('-samples', type=int, default=1,
                        help='Number of samples to collect while streaming.')
    parser.add_argument('-baud', choices=[9600, 19200, 38400, 57600, 115200], type=int,
                        default=115200, help='Baud rate for device.')
    parser.add_argument('-port', default='COM3',
                        help='Specify port for machine (COM3 for windows, /dev/PwrCheck linux)')
    parser.add_argument('-raw', action='store_true',
                        help='Dump raw byte read/write transactions from session.')    
    args = parser.parse_args()

    # If no arguments, print copyright notice and help.
    if len(sys.argv) == 1:
        print(LICENSE_INFO)
        parser.print_help()
        exit(0)

    unit = power_check.PowerCheck(port=args.port, baudrate=args.baud)
    if args.config:
        print(f'Print current settings from device.')
        print(unit.config)
    elif args.info:
        print(f'Print device firmware information.')
        print(unit.get_unit_info())
    elif args.reset:
        print(f'Reset device counters.')
        print(unit.clear_log_data())
    elif args.status:
        print(f'Getting status from device.')
        print(unit.status)
    elif args.stream_status:
        print(f'Streaming values to file.')
        unit.set_status_file('./')
        unit.stream_status(num_samples=args.samples)
    elif args.dump:
        print(f'Dumping eeprom from unit.')
        eeprom_data = unit.dump_eeprom()
        print(f'Found {len(eeprom_data)} values: {eeprom_data}')
    elif args.logging_sec is not None:
        print(f'Applying logging interval of {args.logging_sec}')
        unit.set_config(log_interval_s=args.logging_sec)
        time.sleep(0.5)
        print('Verifying new setting.')
        print(unit.config)
    elif args.over_voltage is not None:
        print(f'Applying overvoltage value of {args.over_voltage}')
        unit.set_config(over_voltage_mv=args.over_voltage)
        print('Verifying new setting.')
        print(unit.get_config)

    # For viewing the raw traffic from the command.
    if args.raw:
        print('Writes from command.')
        print(unit.dump_write_buffer())
        print('Reads from command.')
        print(unit.dump_read_buffer())
