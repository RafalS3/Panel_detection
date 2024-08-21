from launch_ros.actions import Node
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from aruco_opencv_msgs.msg import MarkerPose

class marker_broadcaster(Node):

    def __init__(self):
        super().__init__('marker_frame_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_subscription = self.create_subscription(MarkerPose, '/d435_arm/aruco_detections', self.marker_pose, )
        self.marker_subscription

    def marker_pose(self, msg: MarkerPose):
        marker_data = MarkerPose()
        marker_data = msg
        self.tf_broadcaster.sendTransform(marker_data)


def main():
    try:
        with rclpy.init():
            node = marker_broadcaster()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

        
        


            

    
    

