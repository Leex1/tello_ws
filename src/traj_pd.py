#!/usr/bin/env python

import math

# from numpy import vectorize
# from pygame import Vector3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Path
from tf.transformations import quaternion_matrix

class DroneControllerNode:
    def __init__(self):
        rospy.init_node('drone_controller_node')
        self.pose_subscriber = rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.pose_callback)
        self.cmd_vel_publisher = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.cmd_traj_publisher = rospy.Publisher('/command_trajectory', MultiDOFJointTrajectory, queue_size=10)
        self.cmd_pose_publisher=rospy.Publisher('/command/pose',PoseStamped,queue_size=10)
        self.path_publisher=rospy.Publisher('/slot/path',Path,queue_size=10)

        # 时间戳，用于修改轨迹
        self.t0=rospy.get_time()
        # 添加当前状态。
        self.current_pose_ = PoseStamped()
        self.desired_pose_ = PoseStamped()
        # (x,y,z) * I = (x,y,z)* R
        self.cmd_vel_ = Twist()      
        self.cmd_vel_b = Twist() 
        self.error_linear_x = 0.0 
        self.error_linear_y = 0.0 
        self.error_linear_z = 0.0 
        self.last_error_linear_x = 0.0 
        self.last_error_linear_y = 0.0 
        self.last_error_linear_z = 0.0 

        self.error_sum_linear_x = 0.0
        self.error_sum_linear_y = 0.0
        self.error_sum_linear_z = 0.0


        # 状态信号
        self.traj_point = True

        self.path=Path()

        # 圆轨迹的直径和，角速度。
        self.desired_radius = 1.0  # Desired radius of circle trajectory
        self.desired_angular_speed = 0.2  # Desired angular velocity of drone
    
    #num min max
    def limit_value(self, value, min_limit, max_limit):
        return max(min(value, max_limit), min_limit)
    
    def translate(self, matrix, cmd_vel):
        cmd_vel_b = np.dot(np.linalg.inv(matrix),np.array([cmd_vel.x, cmd_vel.y, cmd_vel.z]))
        return cmd_vel_b
        


    def pose_callback(self, pose_stamped_msg):
        self.current_pose_ = pose_stamped_msg
        current_pose = pose_stamped_msg.pose

        self.path.header=pose_stamped_msg.header
        self.path.poses.append(pose_stamped_msg)
        self.path_publisher.publish(self.path)
        

    # def traj_callback(self):
        # current_pose = self.current_pose_ 
        # Calculate desired position using a circle trajectory
        
        # 追踪a点轨迹
        if(self.traj_point):
            print("start to set point!")
            self.desired_pose_.pose.position.x = 1.0
            self.desired_pose_.pose.position.y = -2.5
            self.desired_pose_.pose.position.z = 1.5
            self.desired_pose_.pose.orientation = current_pose.orientation

        # 追踪圆轨迹
        if(abs(current_pose.position.x - self.desired_pose_.pose.position.x)<0.3 
           and abs(current_pose.position.y - self.desired_pose_.pose.position.y)<0.3 
           and self.traj_point):
            self.t0=rospy.get_time()
            self.traj_point = False

        if(self.traj_point==False):
            print("start to follow circle!")
            self.desired_pose_.pose.position.x = self.desired_radius * math.cos(self.desired_angular_speed * (rospy.get_time()-self.t0))
            self.desired_pose_.pose.position.y = self.desired_radius * math.sin(self.desired_angular_speed * (rospy.get_time()-self.t0)) - 2.5
            # self.desired_pose_.pose.position.z = current_pose.position.z
            self.desired_pose_.pose.orientation = current_pose.orientation

        # Calculate desired linear and angular velocities using PD control law
        kp_linear = 1.0  # Proportional gain for linear velocity control
        kd_linear = 0.5  # Derivative gain for linear velocity control
        kp_angular = 1.0  # Proportional gain for angular velocity control
        kd_angular = 0.5  # Derivative gain for angular velocity control
        ki_linear = 0.0001
        ki_angular = 0.0001

        self.error_linear_x = self.desired_pose_.pose.position.x - current_pose.position.x
        self.error_linear_y = self.desired_pose_.pose.position.y - current_pose.position.y
        self.error_linear_z = self.desired_pose_.pose.position.z - current_pose.position.z

        error_angular = self.desired_angular_speed - current_pose.orientation.z  # Assuming orientation.z represents yaw angle

        desired_linear_x = kp_linear * self.error_linear_x + kd_linear * (self.error_linear_x - self.last_error_linear_x) + ki_linear * self.error_sum_linear_x
        desired_linear_y = kp_linear * self.error_linear_y + kd_linear * (self.error_linear_y - self.last_error_linear_y) + ki_linear * self.error_sum_linear_y
        # desired_angular = kp_angular * error_angular - kd_angular * current_pose.angular.z

        self.last_error_linear_x = self.error_linear_x
        self.last_error_linear_y = self.error_linear_y
        self.last_error_linear_z = self.error_linear_z

        # 限制幅度，1000以内。
        self.error_sum_linear_x = self.limit_value(self.error_sum_linear_x, 1000, -1000) + self.error_linear_x
        self.error_sum_linear_y = self.limit_value(self.error_sum_linear_y, 1000, -1000) + self.error_linear_y
        self.error_sum_linear_z = self.limit_value(self.error_sum_linear_z, 1000, -1000) + self.error_linear_z
        
        # 计算cmd_vel
        self.cmd_vel_.linear.x = desired_linear_x * 1.5
        self.cmd_vel_.linear.y = desired_linear_y * 1.5
        self.cmd_vel_.linear.z = self.error_linear_z

        # self.cmd_vel_.linear.z = 0.0
        self.cmd_vel_.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.cmd_vel_)
        print("/tello/cmd_vel",self.cmd_vel_)

        # 获取旋转矩阵
        Orientation = [ self.current_pose_.pose.orientation.x, 
                        self.current_pose_.pose.orientation.y, 
                        self.current_pose_.pose.orientation.z, 
                        self.current_pose_.pose.orientation.w, 
        ]
        rotation_matrix = quaternion_matrix(Orientation)[:3, :3]
        self.cmd_vel_b.linear = self.translate(rotation_matrix, self.cmd_vel_.linear)

        


        # Publish the desired velocity and command pose in a MultiDOFJointTrajectory message
        traj_msg = MultiDOFJointTrajectory()
        point_msg = MultiDOFJointTrajectoryPoint()
        point_msg.transforms.append(self.desired_pose_)
        self.desired_pose_.header.frame_id='map'

        # point_msg.velocities.append(Twist(linear=Vector3(x=desired_linear_x, y=desired_linear_y)))
        # point_msg.velocities.append(Twist(angular=Vector3(z=desired_angular)))
        traj_msg.points.append(point_msg)
        self.cmd_traj_publisher.publish(traj_msg)
        self.cmd_pose_publisher.publish(self.desired_pose_)


if __name__ == '__main__':
    try:
        drone_controller_node = DroneControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass