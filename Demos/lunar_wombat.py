"""Library of data collection cases for lunar wombat demonstration.

Currently, the data collection computer is a Raspberry Pi, which manages data
collection of power and inertial sensing data. The code operates in 3 phases:
- initialization of the equipment and environment
- asynchronous data collection
- post processing of data

Currently, the operating model of the code covers:
- Prep and ready: create objects needed and verify the equipment is in the
  proper state to run the experiment.
- System identification: collect high speed data during operations to perform
  spectral analysis on the excavator.
- Transport: while the excavator is transiting between locations, collect basic
  navigation telemetry.

The file uses asyncio to manage the collection of the multiple data streams. The
data collection functions are defined here to keep the functions located
together. If for some reason data collection needs to end early it is OK to
terminate the program with a SIGINT. Generally, try to run this in a screen
program (like tmux) or via nohup over ssh to allow for easy interrupts.

Sample call:
python3 ./lunar_wombat.py -mode route -results_dir ./data -remove_bias
"""
import argparse
import asyncio
import logging
import microstrain
import os
import packets
import power_check
import sys
from typing import Any
import time

# Name of file to hold the binary traffic from the microstrain device.
INS_FILE = 'ins_raw.txt'
# The other raw data set from the run is the logging messages.
LOG_FILE = 'log.txt'


def create_parser() -> argparse.ArgumentParser:
    """Creates an argument parser to manage flags for the experiment."""
    parser = argparse.ArgumentParser()
    # Operational phase data collection.
    parser.add_argument('-mode', type=str,
                      choices=['prep', 'ready', 'sysid', 'route'])
    # Experimental config.
    parser.add_argument('-collection_sec', type=float, default=5.0,
                    help='Number of seconds to collect data for.')
    parser.add_argument('-results_dir', type=str, default='./',
                        help='Directory to store results from collection.')
    # Configuration flags.
    parser.add_argument('-ustrain_baud',
                        choices=microstrain.Microstrain3DM.BAUD_RATES,
                        type=int, default=115200, help='Baud rate for device.')
    parser.add_argument('-ustrain_port', default='/dev/ustrain',
                        help=('Specify port for machine'))
    parser.add_argument('-pwrcheck_port', default='/dev/PwrCheck',
                        help=('Specify port for machine'))
    # Process flags.
    parser.add_argument('-remove_bias', action='store_true',
                        help='Estimate gyro bias (at rest) for navigation.')
    return parser


def process_args() -> argparse.Namespace:
    """Process args or print help if no flags are provided."""
    parser = create_parser()
    args = parser.parse_args()

    if len(sys.argv) == 1:
        # Mode must be defined for binary to run.
        parser.print_help()
        exit(0)
    return args


def get_results_dir_path(args):
    """Returns absolute path to results directory."""
    if args.results_dir.startswith('.'):
        # Relative path, make absolute.
        cwd = os.getcwd()
        abs_path = os.path.join(cwd, args.results_dir[2:]) 
    else:
        abs_path = args.results_dir
    return abs_path


def prep_environment(args: argparse.Namespace) -> tuple[Any]:
    """Creates and configures objects needed for data collection and files."""
    ins = microstrain.Microstrain3DM(args.ustrain_port)
    pwr = power_check.PowerCheck(args.pwrcheck_port)

    # In case streaming is in progress, reset state.
    ins.device_idle()

    # Set baud rate requested, note we always start at the default.
    ins.device_baud(args.ustrain_baud)

    # Make sure the data will have some place to go.
    results_path = get_results_dir_path(args)
    if not os.path.exists(args.results_dir):
        os.makedirs(results_path)

    # Setup logging for the run, everything higher than the level in the
    # basicConfig will be written to the log file.
    log_file = os.path.join(results_path, LOG_FILE)
    logging.basicConfig(filename=log_file, level=logging.DEBUG)
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)

    logging.info(f'Prepping system for {args.mode} operation.')
    if args.mode == 'prep':
        # Configure data storage on power check as a backup.
        pwr.set_config(log_interval_s=5)  # Log power every 5 seconds.
        # Clear formats just in case.
        ins.reset_msg_fmt(packets.DataMessages.IMU)
        ins.reset_msg_fmt(packets.DataMessages.GPS)
        ins.reset_msg_fmt(packets.DataMessages.EKF)
        # Set max rate for data collection.
        ins.device_baud(921600)
    elif args.mode == 'sysid':
        # Configure high rate acquisition for system characterization.
        ins.set_msg_fmt(descriptors=[0x04, 0x05], rate_hz=[500.])
        pwr.set_config(foreground_rgb=0x1f)  # Set screen blue to indicate sysid.
    elif args.mode == 'route':
        # Collect general telemetry during transport.
        # Prepare the filter.
        if args.remove_bias:
          ins.capture_gyro_bias(10000)  # Capture bias for 10 seconds.
        # Note for automative mode the X axis of the device needs to align with
        # the direction of forward travel.
        ins.set_dynamics_mode(microstrain.EkfDynamicsMode.AUTOMOTIVE,
                              microstrain.FunctionSelectors.NEW)
        # Use complimentary filter euler angles to initialize attitude.
        ins.ekf_euler_init()
        # Configure data for streaming.
        # CF Accel vector.
        ins.set_msg_fmt(packets.DataMessages.IMU, [0x11], [1.])
        # LLH position, GPS time.
        ins.set_msg_fmt(packets.DataMessages.GPS, [0x03, 0x09], [1.])
        # LLH, NED velocity, LLH uncertainty + Euler angles,
        # angular rate, accel scale factor and uncertainty.
        ins.set_msg_fmt(packets.DataMessages.EKF,
                        [0x01, 0x02, 0x08, 0x05, 0x0a, 0x0e, 0x17, 0x19], [1.])
        # Setup files for power output, since it writes directly.
        pwr.set_status_file(args.results_dir)
        pwr.set_config(foreground_rgb=0x07e0)  # Set screen green to indicate route.

    return ins, pwr


def system_ready(ins: microstrain.Microstrain3DM):
    """Provides information on hardware readiness and device status."""
    logging.info(f'Built in test OK: {ins.device_built_in_test() == 0}')
    gps_fix_info = ins.poll_data(
        packets.DataMessages.GPS,
        descriptors=[0x0b])
    gps_hw_status = ins.poll_data(
        packets.DataMessages.GPS,
        descriptors=[0x0d]
    )
    filter_status = ins.poll_data(
        packets.DataMessages.EKF,
        descriptors=[0x10]
    )
    logging.info(f'GPS satellite fix info: {gps_fix_info}')
    logging.info(packets.ReplyFormats.decode(gps_fix_info[0]))
    logging.info(f'GPS HW status: {gps_hw_status}')
    logging.info(packets.ReplyFormats.decode(gps_hw_status[0]))
    logging.info(f'Filter status: {filter_status}')
    logging.info(packets.ReplyFormats.decode(filter_status[0]))


async def print_timer(args: argparse.Namespace, num_updates: int = 10):
    """Periodic function to keep track of time during the experiment."""
    sleep_int = args.collection_sec / num_updates
    t_start = time.time()
    t_end = t_start + args.collection_sec
    logging.debug(f'Timer sleep {sleep_int} for {num_updates} cycles.')
    while (t_now := time.time()) < t_end:
        logging.info(f'Elapsed {t_now - t_start} of {args.collection_sec} sec.')
        await asyncio.sleep(sleep_int)


def get_ins_file(args) -> str:
    """Returns the full system path to the raw data file for microstrain."""
    return os.path.join(get_results_dir_path(args), INS_FILE)


async def ins_data(ins: microstrain.Microstrain3DM, args: argparse.Namespace):
    """Async routine to handle data collection for microstrain."""
    t_end = time.time() + args.collection_sec
    ins.device_resume()
    with open(get_ins_file(args), 'wb') as f:
        while time.time() < t_end:
            # Read all packets available, if no serial bytes are available it
            # will return None.
            new_packet = ins.get_single_packet()
            if new_packet:
                f.write(new_packet.as_log_str())
            await asyncio.sleep(0.001)
        ins.device_idle()


async def pwr_data(pwr: power_check.PowerCheck, args: argparse.Namespace):
    """Async routine to handle data collection for power check device."""
    t_end = time.time() + args.collection_sec
    while time.time() < t_end:
        # Get a status message and write to file.
        status_str = pwr.get_single_status()
        logging.debug(status_str)
        await asyncio.sleep(1.0)


async def data_collect(ins: microstrain.Microstrain3DM,
                       pwr: power_check.PowerCheck,
                       args: argparse.Namespace):
    """Manages the data gathering from all the async functions."""
    await asyncio.gather(
        print_timer(args), pwr_data(pwr, args), ins_data(ins, args))


def run_experiment(args: argparse.Namespace):
    """Runs data collection and restores equipment to initial state."""
    logging.info(f'Starting data collection for {args.collection_sec} sec.')
    try:
        asyncio.run(data_collect(ins, pwr, args))
    except KeyboardInterrupt:
        logging.warning('Keyboard interrupt ended collection early.')
        ins.device_idle()  # Put IMU to sleep to quiet the serial stream.
    # Convert raw data stream into MipsPacket objects.
    ins.load_data_stream(get_ins_file(args))
    # Process and sort results into csv files for analysis.
    ins.write_stream_data(get_results_dir_path(args))
    # Restore baud to default.
    ins.device_baud(115200)
    # Disable data logging on power check.
    pwr.set_config(log_interval_s=0)
    pwr.set_config(foreground_rgb=0xfff)  # Reset to default, white.

if __name__ == "__main__":
    args = process_args()
    ins, pwr = prep_environment(args)

    if args.mode == 'ready':
        system_ready(ins)

    # Run data collection if requested.
    if args.mode in ['sysid', 'route']:
        run_experiment(args)

    logging.info('Data collection complete!')
