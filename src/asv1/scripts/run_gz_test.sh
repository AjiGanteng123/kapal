#!/bin/bash
source /opt/ros/jazzy/setup.bash
exec gz sim -r -s -v 3 "$@" > /tmp/gz_run.log 2>&1
