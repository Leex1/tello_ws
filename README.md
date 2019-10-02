# tello_driver

## 1. Overview
Communicating with the Tello drone can be done either using official SDK or one of the unofficial libraries. The unofficial libraries originated from the reverse-engineering the raw packages broadcasted by the Tello. This ROS package is build on top of the unofficial TelloPy library. The TelloPy library is used since at this moment it offers more functionalities than the official SDK or any other unofficial library. 

Developing of the tello_driver ROS package is inspired by [tello_driver](https://github.com/anqixu/tello_driver), which by now diverged considerately from the original work. Furthermore, development of this ROS package pursues not to modify the TelloPy library, but instead apply any modification or addition to the ros_driver package in an encapsulated manner. This prevents breaking functionalities when updating the TelloPy library.

### Installation
$ cd <CATKIN_WS/SRC>
$ git clone https://github.com/hanyazou/TelloPy/tree/develop-0.7.0
$ cd TelloPy
$ pip install av numpy --user
$ python setup.py install --user
$ cd ..
$ git clone https://github.com/appie-17/tello_driver.git
$ git clone https://github.com/tilk/h264_image_transport.git
$ cd ..
$ catkin_make
$ source devel/setup.bash

### Launch

* Turn on Tello drone
* Connect to drone's WiFi access point (```TELLO_XXXXXX)```
* $ roslaunch tello_driver tello_node.launch


## 2. Nodes

### tello_driver_node.py
Main node running as interface for the TelloPy library

#### Subscribed topics
/tello/cmd_vel
/tello/emergency
/tello/fast_mode
/tello/flattrim
/tello/flip
/tello/land
/tello/palm_land
/tello/takeoff
/tello/throw_takeoff

### Published topics
/tello/camera/camera_info [sensor_msgs/CameraInfo]
/tello/image_raw [sensor_msgs/Image]
/tello/imag/raw/h264 [h264_image_transport/H264Packet]
/tello/odom [nav_msgs/Odometry]
/tello/status [tello_driver/TelloStatus]

### Services

### Parameters
~/image_raw/compressed/format
~/image_raw/compressed/jpeg_quality
~/image_raw/compressed/png_level
~/tello_driver_node/connect_timeout_sec
~/tello_driver_node/fixed_video_rate
~/tello_driver_node/local_cmd_client_port
~/tello_driver_node/local_vid_server_port
~/tello_driver_node/stream_h264_video
~/tello_driver_node/tello_cmd_server_port
~/tello_driver_node/tello_ip
~/tello_driver_node/vel_cmd_scale
~/tello_driver_node/video_req_sps_hz


## 3. Troubleshooting

## 4. Notes

## 5. WIP

## 6. License

**DISCLAIMER: This package is an work-in-progress. I take no responsibility for any consequences of you using this software. The documentation might be broken, and features and API are considered VOLATILE presently.**

ROS driver wrapper for DJI/Ryze Tello drone

Node: [src/tello_driver_node.py](src/tello_driver_node.py)

Topics:
* `~cmd_vel`: [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
* `~fast_mode`: [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~image_raw`: [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
* `~takeoff`: [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~throw_takeoff`: [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~land`: [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~palm_land`: [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~flattrim`: [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~flip`: [std_msgs/Uint8](http://docs.ros.org/api/std_msgs/html/msg/UInt8.html)

Parameters:
* `~tello_ip`
* `~tello_cmd_port`
* `~client_port`
* `~connect_timeout_sec`

## Installation
* `$ cd <CATKIN_WS/SRC>`
* `$ git clone https://github.com/anqixu/TelloPy.git`
* `$ cd TelloPy`
* `$ sudo -H pip2 install -e .`
* `$ cd ..`
* `$ git clone https://github.com/anqixu/h264_image_transport.git`
* `$ git clone https://github.com/anqixu/tello_driver.git`
* `$ cd ..`
* `$ rosdep install h264_image_transport`
* skip this step: `$ # rosdep install tello_driver # not working currently`
* `$ catkin build tello_driver`

Optionally, install the [following udev rules](https://github.com/anqixu/sixad_rumble/blob/master/misc/10-gamepads.rules) for PS3 gamepads; see instructions in comments on top of file.

## Running the driver

* turn on drone and wait for its front lights to blink amber
* connect WiFi to drone's access point (e.g. `TELLO_######`)
* `$ roslaunch tello_driver launch/tello_node.launch`

To see the camera:
* `$ rosrun rqt_image_view rqt_image_view /tello/image_raw/compressed`

## Tele-operate the drone using a wired DualShock 3 gamepad

First check out the following and possibly adjust parameters / code / mappings:
* [launch/joy_teleop.launch](launch/joy_teleop.launch)
* [launch/logger.launch](launch/logger.launch)
* [src/gamepad_marshall_node.py](src/gamepad_marshall_node.py)

Now run:
* `$ roslaunch tello_driver devel.launch`

## Connecting to multiple drones

It is possible to connect to multiple Tello drones by using multiple USB WiFi dongles and a [Docker container running UDP proxy servers](wifi_docker_proxy).

## Known bugs
* Sometimes, perhaps when taking off without moving gamepad analog sticks / sending commands to `/tello/cmd_vel`, further cmd_vel will not work; fix by restarting node, moving gamepad analog sticks / send a message to `/tello/cmd_vel` FIRST, then takeoff
