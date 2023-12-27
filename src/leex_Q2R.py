import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix

def pose_callback(msg):
    # Extract quaternion from the PoseStamped message
    quaternion = [msg.pose.orientation.x, msg.pose.orientation.y,
                  msg.pose.orientation.z, msg.pose.orientation.w]

    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_matrix(quaternion)[:3, :3]

    # Now you have the rotation matrix
    print("Rotation Matrix:")
    print(rotation_matrix)

if __name__ == "__main__":
    rospy.init_node("pose_listener")

    # Assuming your PoseStamped topic is "/drone/pose"
    pose_topic = "/drone/pose"
    
    # Set up a TF2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Subscribe to the PoseStamped topic
    rospy.Subscriber(pose_topic, PoseStamped, pose_callback)

    rospy.spin()
