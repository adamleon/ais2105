#!/bin/bash

source /opt/ros/humble/setup.bash && colcon build

tail -f /dev/null