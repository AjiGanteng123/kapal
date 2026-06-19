#!/bin/bash
echo "=== Killing ROS2 + Gazebo ==="
pkill -f "gz sim" 2>/dev/null
pkill -f "gzserver" 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null
pkill -f "ros2 run" 2>/dev/null
pkill -f "node_navigasi" 2>/dev/null
pkill -f "node_deteksi" 2>/dev/null
pkill -f "sim_bridge" 2>/dev/null
pkill -f "viewer" 2>/dev/null
pkill -f "parameter_bridge" 2>/dev/null
echo "Done. Log saved at /tmp/navigasi_log.csv"
