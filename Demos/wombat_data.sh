#!/bin/bash
# This script waits until a short button press is recorded and then
# kicks off data collection of the equipment for the experiment.

# Home base for all data and scripts.
SCRIPT_DIR=/home/utilis

# First, block until the press is registered, 3 counts with 0.5 sec sleeps.
${SCRIPT_DIR}/wombat_gpio_monitor.sh 3 0.5 "telemetry begin"

echo "Starting data collection at $(date)"

wombat_script=${SCRIPT_DIR}/lunar_wombat.py
wombat_devs="-ustrain_port=/dev/ustrain -pwrcheck_port=/dev/PwrCheck"

# Commands to cycle through during the experiment.
wombat_cmds=("prep"
  "ready"
  "route -remove_bias -collection_sec 10"
  "sysid -collection_sec 60"
)

# Create results variable for today's date.
RESULTS=${SCRIPT_DIR}/$(date +%Y%m%d)

# Cycle through commands.
for cmd in "${wombat_cmds[@]}"
do
  trial="${cmd%% *}"  # Extract first term from cmd passed in.
  results_dir="${RESULTS}/$(date +%Y%m%d_%H%M%S)_${trial}"
  full_cmd="${wombat_script} ${wombat_devs} -mode ${cmd} -results_dir=${results_dir}"
  echo "RUNNING: ${full_cmd}"
  python3 ${full_cmd}
done

echo "Data collection complete!"
