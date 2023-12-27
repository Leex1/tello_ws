# /usr/bin/env bash

rostopic pub --once /tello/takeoff std_msgs/Empty "{ }"

read -p "enter to land"

rostopic pub --once /tello/land std_msgs/Empty "{ }"