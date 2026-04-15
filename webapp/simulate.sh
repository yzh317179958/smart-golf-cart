#!/bin/bash
# Simulate car MQTT messages for Web APP testing
# Run on VPS: bash webapp/simulate.sh

echo "=== Golf Cart Simulator ==="
echo "Simulating car GPS + status messages..."
echo "Open http://144.202.105.250:8080 on your phone to see"
echo "Press Ctrl+C to stop"
echo ""

# Starting position (from path_graph.json waypoint 1)
LAT=22.658400
LON=114.224740

# Send initial mode
mosquitto_pub -h localhost -t "golf/cart_01/status/mode" -m "FOLLOWING"
mosquitto_pub -h localhost -t "golf/cart_01/status/path_stats" -m '{"waypoint_count":61,"edge_count":64,"total_distance_m":350,"labeled_count":0}'

# Subscribe to commands from app in background
mosquitto_sub -h localhost -t "golf/cart_01/cmd/#" -v &
SUB_PID=$!
trap "kill $SUB_PID 2>/dev/null; echo 'Stopped.'; exit" INT

i=0
while true; do
  # Simulate GPS drift (small random movement)
  DLAT=$(python3 -c "import random; print(f'{random.uniform(-0.00002, 0.00002):.7f}')")
  DLON=$(python3 -c "import random; print(f'{random.uniform(-0.00002, 0.00002):.7f}')")
  LAT=$(python3 -c "print(f'{$LAT + $DLAT:.7f}')")
  LON=$(python3 -c "print(f'{$LON + $DLON:.7f}')")

  # GPS
  mosquitto_pub -h localhost -t "golf/cart_01/status/gps" \
    -m "{\"lat\":$LAT,\"lon\":$LON,\"alt\":50.0,\"status\":0}"

  # Heartbeat every 5 iterations
  if (( i % 5 == 0 )); then
    mosquitto_pub -h localhost -t "golf/cart_01/status/heartbeat" -m "online"
  fi

  ((i++))
  sleep 1
done
