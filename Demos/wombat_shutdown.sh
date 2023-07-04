#!/bin/bash
# This script looks for a switch press of 8+ seconds and initiates.
# a shutdown of the RPi after killing any data collection. See the
# wombat_gpio_monitor shell script for info on the physical setup.

# Display system status.
echo "System temperature is $(vcgencmd measure_temp)"

# Wait for keypress of requested duration.
/home/utilis/wombat_gpio_monitor.sh 8 1 "8sec Shutdown Press"

# Send an interrupt and wait for the process to complete.
PID=`pgrep python3`
kill -INT ${PID}
tail --pid=${PID} -f /dev/null

# Data processing complete, safe to proceed.
echo "SHUTTING DOWN SYSTEM!"
shutdown now
