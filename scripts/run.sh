#!/bin/bash
set -e
source ~/robot_ws/install/setup.bash
echo "=== Build ==="
colcon build --packages-select asv1 2>&1 | tail -5
echo ""
echo "=== Launch ==="
ros2 launch asv1 asv1.launch.py
