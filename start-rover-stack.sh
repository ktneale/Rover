#!/bin/bash

cleanup() {
    echo "Ctrl+C detected. Cleaning up..."
    killall -9 python3
    killall -9 janus
    exit 1
}

trap cleanup SIGINT

cd ~/rover2

logs_base_dir="/tmp/rover/logs"

rm -rf $logs_base_dir
mkdir -p $logs_base_dir

controller_log_path="$logs_base_dir/controller.log"
uart_log_path="$logs_base_dir/uart.log"
lidar_log_path="$logs_base_dir/lidar.log"
udp_monitor_path="$logs_base_dir/udp-monitor.log"
websocket_relay_path="$logs_base_dir/websocket_relay.log"

stdbuf -oL python3 -m rover.control.rover_uart5 --cmd-bind-host 127.0.0.1  --cmd-bind-port 5001 --pub-host 10.42.0.213  --pub-port 5002  --imu-poll-s 1.5 --status-poll-s 1.5 --telemetry-pub-host 127.0.0.1 --telemetry-pub-port 8012 >> "$uart_log_path" 2>&1 & 
#stdbuf -oL python3 -m rover.control.rover_uart --serial /dev/serial0 --heartbeat 10   >> "$uart_log_path" 2>&1 &
stdbuf -oL python3 -m rover.control.controller --initial-mode track --teleop-timeout-s 0.35 --manual-override-timeout-s 0.4 --auto-timeout-s 0.25 --heartbeat 10 >> "$controller_log_path" 2>&1 &
#stdbuf -oL python3 -m rover.sensors.lidar_headless --port /dev/ttyUSB0 >> "$lidar_log_path" 2>&1 &

stdbuf -oL python3 -m rover.sensors.lidar_headless_slam \
  --port /dev/ttyUSB0 \
  --slam-scan-host 10.42.0.213 \
  --debug-sectors \
  --slam-scan-port 8030 \
  --debug-revolution-scans >> "$lidar_log_path" 2>&1 &

stdbuf -oL python3 -m rover.tools.udp_monitor --port 5002 --pretty >> "$udp_monitor_path" 2>&1 &
stdbuf -oL python3 -m rover.tools.websocket-bridge >> "$websocket_relay_path" 2>&1 &

echo "Rover stack started. Run Rover Run!"

while true; do
    sleep 5 
done

#EOF
