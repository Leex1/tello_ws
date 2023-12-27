# /usr/bin/env bash

read -p "enter to land"

rosbag record /commond/pose /command_trajectory /mavros/vision_pose/pose /tello/cmd_vel /slot/path