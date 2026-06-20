#!/bin/bash
set -e

# Load environment variables from .env (if exists)
if [ -f ~/robot_ws/.env ]; then
    export $(cat ~/robot_ws/.env | grep -v '^#' | xargs)
    echo "[INFO] Loaded environment variables from .env"
fi

source ~/robot_ws/install/setup.bash
echo "=== Build ==="
colcon build --packages-select asv1 2>&1 | tail -5
echo ""
echo "=== Launch ==="
ros2 launch asv1 asv1.launch.py
