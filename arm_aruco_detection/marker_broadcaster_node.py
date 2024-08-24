from launch_ros.actions import Node
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from aruco_opencv_msgs.msg import MarkerPose

class marker_broadcaster(Node):

    def __init__(self):
        super().__init__('marker_frame_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.marker_subscription = self.create_subscription(MarkerPose, '/aruco_opencv_msgs/msg/ArucoDetection', self.marker_pose, 10)
        self.marker_subscription

    def marker_pose(self, msg: MarkerPose):
        MarkerTransform = TransformStamped()
        MarkerTransform.header.stamp = self.get_clock().now().to_msg()
        MarkerTransform.header.frame_id = 'marker_frame'

        MarkerTransform.transform.translation.x = msg.pose.position.x
        MarkerTransform.transform.translation.y = msg.pose.position.y
        MarkerTransform.transform.translation.z = msg.pose.position.z

        MarkerTransform.transform.rotation.x = msg.pose.orientation.x
        MarkerTransform.transform.rotation.y = msg.pose.orientation.y
        MarkerTransform.transform.rotation.z = msg.pose.orientation.z
        MarkerTransform.transform.rotation.w = msg.pose.orientation.w

        self.tf_broadcaster(MarkerTransform)



def main():
    try:
        rclpy.init()
        node = marker_broadcaster()
        rclpy.spin(node)
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

        
        


            

    
    

