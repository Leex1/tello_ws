# /usr/bin/env bash

rostopic pub /tello/land std_msgs/Empty "{ }"

read -p "enter to land"

rostopic pub /tello/takeoff std_msgs/Empty "{ }"