#!/bin/bash
# This script looks for a switch press for user requested durations and
# returns when complete. This can be used to block a script until the button
# event desired happens on the system. This script is customized for the
# lunar wombat demonstration. GPIO24 is pulled down to ground via a 10Kohm
# resistor to pin 20. The switch connects V3.3 via pin 17 to the GPIO.
# Sample calls:
#  Look for a 3 cycle press, at 1 second period, with message "3 second press"
#  ./wombat_gpio_monitor.sh 3 1 "3 second press"

# Basic script params. 
CNTR_MAX=${1:-3}
CNTR_SLEEP=${2:-0.5}
CNTR_MSG=${3:-"short press"}

# Select the pin for switch input. 
IN_SWITCH=24 

# Useful constants. 
GPIO=/sys/class/gpio
GPIO_EXPORT="${GPIO}/export"
GPIO_UNEXPORT="${GPIO}/unexport" 
OUTPUT="out" 
INPUT="in"

# Local variables.
in_pin="${GPIO}/gpio${IN_SWITCH}"
in_dir="${in_pin}/direction"
in_val="${in_pin}/value" 

# Set input pin to input mode, only if it doesn't exist already.
if [[ ! -e ${in_pin} ]]; then
  echo "Setting up ${in_pin} on system."
  echo "${IN_SWITCH}" > "${GPIO_EXPORT}"
  echo "${INPUT}" > "${in_dir}"
else
  echo "gpio${IN_SWITCH} already created."
fi

echo "Starting watch for ${CNTR_MSG} at $(date)"
echo "Requested ${CNTR_MAX} counts at ${CNTR_SLEEP} sec intervals."

# Blocking loop while we wait for requested button presses.
cntr=${CNTR_MAX}
while [[ ${cntr} -ge 0 ]]
do
  state=`cat ${in_val}`
  if [[ ${state} -eq 1 ]]; then
    # If button pressed, advance countdown.
    cntr=$((cntr-1))
  else
    # Reset the counter.
    cntr=${CNTR_MAX}
  fi
  if [[ ${cntr} -eq $((CNTR_MAX-1)) ]]; then
    echo "Push detected at $(date)"
  fi
  sleep ${CNTR_SLEEP}
done

echo "${CNTR_MSG} complete!"
