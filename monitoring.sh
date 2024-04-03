#!/bin/bash

# Path to python script
PYTHON_SCRIPT="/home/pi/tradeshow_sensors/test_bme.py"

# Path to heartbeat file
HEARTBEAT_FILE="/home/pi/tradeshow_sensors/heartbeat.txt"

# Time interval to check for updates in seconds
CHECK_INTERVAL=3

# Function to check if heartbeat file has been updated recently
check_timestamp() {
    current_time=$(date +%s)
    modification_time=$(stat -c %Y "$HEARTBEAT_FILE")
    time_difference=$((current_time - modification_time))

    if [ "$time_difference" -le 15 ]; then
        return 0  # File has been updated recently
    else
        return 1  # File has not been updated recently
    fi
}

# Main loop to continuously check and restart the python script if needed
while true; do
	# Check if heartbeat file has been updated
	if ! check_timestamp; then
		echo "Python script stuck, restarting"
		pkill -f "$PYTHON_SCRIPT"
		python3 "$PYTHON_SCRIPT" &
	fi
	# Sleep before checking again
        sleep "$CHECK_INTERVAL"
done
