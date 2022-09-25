#!/bin/bash

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$AVT_HOME/devel/setup.bash"

roslaunch traffic_light_fetcher traffic_light_fetcher.launch
