# tello_driver

## 1. Overview
Communicating with the Tello drone can be done either using official [Tello SDK](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf) or one of the unofficial libraries. The unofficial libraries originated from the reverse-engineering the raw packages broadcasted by the Tello. This ROS package is build on top of the unofficial [TelloPy](https://github.com/hanyazou/TelloPy) library. The [TelloPy](https://github.com/hanyazou/TelloPy) library is used at this moment since it offers more functionalities than the official [Tello SDK](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf) or any other unofficial library. 

Developing of the tello_driver ROS package is inspired by [tello_driver](https://github.com/anqixu/tello_driver), which by now diverged considerately from the original work. Furthermore, development of this ROS package pursues not to modify the TelloPy library, but instead apply any modification or addition to the ros_driver package in an encapsulated manner. This prevents breaking functionalities when updating the TelloPy library.

### Installation
* ```$ cd <CATKIN_WS/SRC>```
* ```$ git clone https://github.com/hanyazou/TelloPy/tree/develop-0.7.0```
* ```$ cd TelloPy```
* ```$ pip install av numpy --user```
* ```$ python setup.py install --user```
* ```$ cd ..```
* ```$ git clone https://github.com/appie-17/tello_driver.git```
* ```$ cd ..```
* ```$ catkin_make```
* ```$ source devel/setup.bash```

### Launch

* Turn on Tello drone
* Connect to drone's WiFi access point (```TELLO_XXXXXX)```
* ```$ roslaunch tello_driver tello_node.launch```


## 2. Nodes

### tello_driver_node.py
Main node running as interface for the TelloPy library

#### Subscribed topics
* ```/tello/cmd_vel``` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
* ```/tello/emergency``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/fast_mode``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/flattrim``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/flip``` [std_msgs/Uint8](http://docs.ros.org/api/std_msgs/html/msg/UInt8.html)
* ```/tello/land``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/palm_land``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/takeoff``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/throw_takeoff``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)

### Published topics
* ```/tello/camera/camera_info``` [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
* ```/tello/image_raw``` [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
* ```/tello/imag/raw/h264``` [h264_image_transport/H264Packet](https://github.com/tilk/h264_image_transport/blob/master/msg/H264Packet.msg)
* ```/tello/odom``` [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
* ```/tello/status``` [tello_driver/TelloStatus](https://github.com/appie-17/tello_driver/blob/development/msg/TelloStatus.msg)

### Services

### Parameters
* ```~/image_raw/compressed/format```
* ```~/image_raw/compressed/jpeg_quality```
* ```~/image_raw/compressed/png_level```
* ```~/tello_driver_node/connect_timeout_sec```
* ```~/tello_driver_node/fixed_video_rate```
* ```~/tello_driver_node/local_cmd_client_port```
* ```~/tello_driver_node/local_vid_server_port```
* ```~/tello_driver_node/stream_h264_video```
* ```~/tello_driver_node/tello_cmd_server_port```
* ```~/tello_driver_node/tello_ip```
* ```~/tello_driver_node/vel_cmd_scale```
* ```~/tello_driver_node/video_req_sps_hz```


## 3. Troubleshooting

## 4. Notes

## 5. WIP

## 6. License


