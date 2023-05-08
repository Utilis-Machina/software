r"""Simple program to try power check library with actual hardware.

This file is used to test the power_check library with an actual unit
hooked up to the computer. This was tested on both Windows and a
Raspberry Pi.
"""
import argparse
import power_check

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    # Note value will be stored under first name provided.
    parser.add_argument('-baud', choices=[9600, 19200, 38400, 57600, 115200], type=int,
                        default=115200, help='Baud rate for device.')
    parser.add_argument('-port', default='COM3',
                        help='Specify port for machine (COM3 for windows, /dev/PwrCheck linux)')
    parser.add_argument('-settings', action='store_true',
                        help='Prints settings on device.')
    parser.add_argument('-info', action='store_true',
                        help='Prints firmware info of device.')  
    parser.add_argument('-set_logger', type=int, dest='logging_sec',
                        help='Sets logging interval')
    parser.add_argument('-over_voltage', type=int, dest='over_voltage',
                        help='Sets over voltage alarm limit.')
    parser.add_argument('-status', '-r', action='store_true',
                        help='Dumps logged data.')
    parser.add_argument('-stream_status', action='store_true',
                        help='Logs and writes data to file.')
    args = parser.parse_args()

    unit = power_check.PowerCheck(port=args.port, baudrate=args.baud)
    if args.settings:
        print(f'Print current settings from device.')
        print(unit.config)
    elif args.info:
        print(f'Print device firmware information.')
        print(unit.get_unit_info())
    elif args.status:
        print(f'Getting status from device.')
        print(unit.status)
    elif args.stream_status:
        print(f'Streaming values to file.')
        unit.set_status_file('./')
        unit.stream_status()
    # For the setting cases, call program a second time with flag "settings" to check.
    elif args.logging_sec is not None:
        print(f'Applying logging interval of {args.logging_sec}')
        unit.set_config(log_interval_s=args.logging_sec)
    elif args.over_voltage is not None:
        print(f'Applying overvoltage value of {args.over_voltage}')
        unit.set_config(over_voltage_mv=args.over_voltage)     
