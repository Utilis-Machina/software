"""Plotting and analysis library for lunar wombat data.

This library provides individual run and batch processing capabilities to
easily visualize data. Some different plot types are provided as examples to
build off of for future analysis.

If directory is defined with underscores in the final folder, it will process
a single run, for example:
python3 ./wombat_analysis.py -results_dir Data\20230719\20230719_100929_route

For visualization in Google Earth the -kml_file argument can be added. That will
write the file to the directory and can then be imported as part of a project
in Google Earth.

For a batch run, provide a path one level higher, i.e:
python3 ./wombat_analysis.py -results_dir Data\20230719

The code will get the contents of the folder provided, pull out any path that
seems to be a sysid or route data collection, and create the plots. It will
report back any errors encountered since sometimes the field data is incomplete
in some way.
"""
import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import re
import subprocess
from typing import Optional

FIG_SIZE = (12, 9)
TIME_KEY = 'Elapsed Time (s)'

# Apply a list of longitude, latitude, and altitude values, separated by
# newlines to complete this formatable string.
# Ref: https://developers.google.com/kml/documentation/kmlreference#linestring
KML_FMT = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Paths</name>
    <description>Path taken during data collection.</description>
    <Style id="yellowLineGreenPoly">
      <LineStyle>
        <color>7f00ffff</color>
        <width>4</width>
      </LineStyle>
      <PolyStyle>
        <color>7f00ff00</color>
      </PolyStyle>
    </Style>
    <Placemark>
      <name>Absolute Extruded</name>
      <description>Transparent green wall with yellow outlines</description>
      <styleUrl>#yellowLineGreenPoly</styleUrl>
      <LineString>
        <extrude>1</extrude>
        <tessellate>0</tessellate>
        <altitudeMode>clamptoground</altitudeMode>
        <coordinates>
          {}
        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>"""

def llh_to_kml_str(
        df: pd.DataFrame,
        cols: list[str] = ['lon (deg)', 'lat (deg)', 'ellipsoid_ht (m)']):
    """Takes a pandas dataframe and returns a KML string for Google Earth."""
    # Convert columns needed to a csv to write into the string.
    llh_csv = df.to_csv(columns=cols, header=False, index=False)
    # Return string with correct padding to match kml indentation.
    return KML_FMT.format(llh_csv.replace('\r\n', '\n' + 10*' ').rstrip())


def fft_amplitude(signal: np.ndarray, period_sec: float):
    """Returns frequency and amplitude of signal series."""
    num_samples = len(signal)
    # At halfway point the frequencies shift from positive to negative.
    half_samples = int(np.ceil(num_samples / 2.))
    fft_amp = np.fft.fft(signal)
    fft_freq = np.fft.fftfreq(num_samples, period_sec)
    pos_freq = fft_freq[:half_samples]
    pos_fft_amp = np.abs((2.0 / num_samples) * fft_amp[:half_samples])
    return pos_freq, pos_fft_amp


def plot_fft_dataframe(fft_data_df: pd.DataFrame, fft_freq_df: pd.DataFrame,
                       cols: list[str], path: str, time_key: str,
                       plot_type: str):
    """Plots and saves figures for dataframes with sys id info."""
    _, axes = plt.subplots(2, 1, figsize=FIG_SIZE)
    fft_data_df.plot(x=time_key, y=cols, ax=axes[0])
    fft_freq_df.plot(x='freq (hz)', y=cols, ax=axes[1])
    for ax in axes:
        ax.grid(True)
    plt.savefig(f'{path}/fft_{plot_type}.png')


def get_path_for_file_type(folder: str, file_type: str) -> Optional[os.DirEntry]:
    """Returns file object for file of type requested."""
    for f in os.scandir(folder):
        if f.is_file():
            if file_type in f.name and f.name.endswith('txt'):
                return f
    else:
        print(f'No file available for {file_type} in {folder}')
    return None


def get_dataframe_for_type(folder: str, file_type: str) -> pd.DataFrame:
    """Gets dataframe for file type requested."""
    df_path = get_path_for_file_type(folder, file_type)
    if df_path is None:
        # See if reprocessing works on the folder.
        print(f'Reprocessing required for {folder}')
        cmd = ['python3', './lunar_wombat.py', '-reprocess', '-results_dir',
               folder]
        proc = subprocess.run(cmd, shell=True, capture_output=True)
        if proc.returncode == 0:
            print('Reprocessing successful!')
        else:
            print('Reprocessing failed.')
        # Try to get path again.
        df_path = get_path_for_file_type(folder, file_type)
    df = pd.read_csv(df_path.path)
    df[TIME_KEY] = df['time (s)'] - df.loc[0, 'time (s)']
    return df


def load_pwrcheck_dataframe(path: str):
    """Loads and sanitzes power check data into a dataframe."""
    pwr_file = get_path_for_file_type(path, 'pwr')
    pwr_df_raw = pd.read_csv(pwr_file.path)
    # The system runs ~24V batteries with 40 amp fuses, so all values should
    # be consistent with that. Some logged values are outside these limits, so
    # apply a filter with padding to find the invalid lines.
    pwr_df = pwr_df_raw.loc[
        (np.abs(pwr_df_raw['calibrated_voltage_mv']) < 5e4) & 
        (np.abs(pwr_df_raw['calibrated_current_ma']) < 1e5), :].copy()
    pwr_df['Volts (V)'] = pwr_df['calibrated_voltage_mv'] * 1e-3
    pwr_df['Current (A)'] = pwr_df['calibrated_current_ma'] * 1e-3
    pwr_df['Quality (V)'] = pwr_df['quality_mv'] * 1e-3
    pwr_df[TIME_KEY] = pwr_df['time'] - pwr_df.iloc[0]['time']

    return pwr_df


def plot_power_dataframe(pwr_df: pd.DataFrame, dir_path: str):
    """Plots and saves figures of power data for a run."""
    _, axes = plt.subplots(2, 1, figsize=FIG_SIZE)
    # Voltage with quality variance as errorbar.
    axes[0].errorbar(x=pwr_df[TIME_KEY], y=pwr_df['Volts (V)'],
                     yerr=pwr_df['Quality (V)'] / 2, marker='o', ecolor='green')
    axes[0].set_ylabel('Volts (V)')
    axes[0].grid(True)
    pwr_df.plot(x=TIME_KEY, y='Current (A)', xlabel='CPU Time (sec)',
                ylabel='Current (A)', marker='o', ax=axes[1], grid=True)
    plt.savefig(f'{dir_path}/pwr_check.png')


def plot_sysid(path: str):
    """Plots and saves figures for a system identification collection."""
    imu_file = get_path_for_file_type(path, 'imu')
    imu_df = get_dataframe_for_type(path, 'imu')
    # Determine the sampling rate from the file name.
    sample_hz = float(re.findall('\d+.\d+', os.path.basename(imu_file.path))[0])

    # Copy out the data to evaluate.
    accel_cols = ['x (Accel g)', 'y (Accel g)', 'z (Accel g)']
    gyro_cols = ['x (Gyro rad/s)', 'y (Gyro rad/s)', 'z (Gyro rad/s)']
    fft_data_df = imu_df[accel_cols + gyro_cols].copy()

    time_key = 'Elapsed GPS Time (s)'
    fft_data_df[time_key] = (imu_df['time_of_week (sec)'] -
                             imu_df.loc[0, 'time_of_week (sec)'])
    period = 1.0 / sample_hz
    expected_time = np.arange(min(fft_data_df[time_key]),
                              max(fft_data_df[time_key]), period)
    fft_freq_df = pd.DataFrame()
    for col_name, data in fft_data_df.items():
        if col_name == time_key:
            continue
        # Remove DC bias.
        data -= data.mean()
        # Fill data gaps to reduce high frequency noise in the data.
        data = np.interp(expected_time, fft_data_df[time_key], data)
        fft_freq, fft_freq_df[col_name] = fft_amplitude(data, period)
    fft_freq_df['freq (hz)'] = fft_freq
    
    plot_fft_dataframe(fft_data_df, fft_freq_df, accel_cols, path,
                       time_key, plot_type='accel')
    plot_fft_dataframe(fft_data_df, fft_freq_df, gyro_cols, path,
                       time_key, plot_type='gyro')


def plot_route(path: str, make_kml: bool = False):
    """Plots and saves figures for a route data collection."""
    imu_df = get_dataframe_for_type(path, 'imu')
    gps_df = get_dataframe_for_type(path, 'gps')
    ekf_df = get_dataframe_for_type(path, 'ekf')
    pwr_df = load_pwrcheck_dataframe(path)
    
    # GPS ground track plot, using plot methods from dataframe.
    gps_ax = gps_df.plot(x='lon (deg)', y='lat (deg)', ylabel='lon (deg)',
                         figsize=FIG_SIZE, grid=True)
    ekf_df.plot(x='lon (deg)', y='lat (deg)', ax=gps_ax)
    gps_ax.legend(['gps', 'ekf'])
    gps_ax.grid(True)
    plt.savefig(f'{path}/ekf_vs_gps_track.png')

    # Accelerometer plots, from the complementary filter.
    _, axes = plt.subplots(2, 1, figsize=FIG_SIZE)
    imu_df.plot(y=['x (CF Accel g)', 'y (CF Accel g)'], ax=axes[0])
    imu_df.plot(y=['z (CF Accel g)'], ax=axes[1])
    axes[0].grid(True)
    axes[1].grid(True)
    plt.savefig(f'{path}/cf_accel.png')

    # Plot of EKF state estimate.
    acc_cols = ['x (%/100)' , 'y (%/100)', 'z (%/100)']
    acc_uc = ['uc_x_1sig (%/100)', 'uc_y_1sig (%/100)', 'uc_z_1sig (%/100)']
    _, est_ax = plt.subplots(3, 1, figsize=FIG_SIZE)
    for i in range(3):
        est_ax[i].errorbar(x=ekf_df[TIME_KEY],
                       y=ekf_df[acc_cols[i]], yerr=ekf_df[acc_uc[i]],
                       marker='o', ecolor='red')
        est_ax[i].grid(True)
        est_ax[i].set_ylabel(acc_cols[i])
    est_ax[2].set_xlabel(TIME_KEY)
    est_ax[0].set_title('EKF State Estimates w/Uncertainties')
    plt.savefig(f'{path}/accel_state.png')

    # Plot incline angle versus current (2 axis plot example).
    _, ax = plt.subplots(figsize=FIG_SIZE)
    color = 'r'
    ax.set_xlabel('Elapsed CPU Time (sec)')
    ax.set_ylabel('z (CF Accel g)', color=color)
    imu_df.plot(x=TIME_KEY, y='z (CF Accel g)', color=color, ax=ax,
                legend=False)
    ax.tick_params(axis='y', color=color)
    ax2 = ax.twinx()
    color = 'b'
    ax2.set_ylabel('Current (Amps)', color=color)
    ax2.plot(pwr_df[TIME_KEY], pwr_df['Current (A)'], color=color)
    ax2.tick_params(axis='y', color=color)
    ax.grid(True)
    plt.savefig(f'{path}/current_vs_angle.png')

    # 3D plot visualization of GPS.
    _, ax = plt.subplots(subplot_kw={"projection": "3d"}, figsize=FIG_SIZE)
    x, y, z = ('lon (deg)', 'lat (deg)', 'ellipsoid_ht (m)')
    ax.plot(ekf_df[x], ekf_df[y], ekf_df[z])
    ax.set_xlabel(x)
    ax.set_ylabel(y)
    ax.set_zlabel(z)
    plt.savefig(f'{path}/plot_3d.png')

    if make_kml:
        # Make a kml file using the gps data loaded.
        with open(os.path.join(path, 'route.kml'), 'w') as f:
            f.write(llh_to_kml_str(ekf_df))


def process_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('-results_dir', type=str, default='./',
                    help='Directory to process results from run.')
    parser.add_argument('-kml_file', action='store_true',
                        help='Make KML file for route run.')
    return parser.parse_args()


if __name__ == '__main__':
    args = process_args()
    # Make path always end with a platform correct slash for os dirname.
    path_arg = os.path.join(args.results_dir, '')
    # Determine how many runs to be processed.
    if '_' in os.path.dirname(path_arg):
        paths = [path_arg]
        print('Single run processing.')
    else:
        all_paths = [f.path for f in os.scandir(path_arg) if f.is_dir()]
        # For now, only process these select runs.
        paths = [p for p in all_paths if p.endswith(('sysid', 'route'))]
        print('Batch processing.')

    for path in paths:
        print(f'Processing path: {path}')
        try:
            # Power system values during the run, should always be available.
            pwr_df = load_pwrcheck_dataframe(path)
            plot_power_dataframe(pwr_df, path)

            # Custom plotting based on collection type.
            if 'sysid' in path:
                plot_sysid(path)
            elif 'route' in path:
                plot_route(path, args.kml_file)
            else:
                print(f'SKIP unsupported run type.')
            plt.close('all')  # Close any opened figures to free memory.
        except (KeyError, AttributeError) as e:
            # Let user know what went wrong for further investigation.
            print(f'FAIL with {type(e).__name__}')
            print(e)
