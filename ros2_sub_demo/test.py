#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image

from rclpy.node import Node

from tf2_ros import TransformBroadcaster, TransformStamped, TransformListener, Buffer, LookupException, ExtrapolationException, ConnectivityException
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library


class CarShapeNode(Node):

    def __init__(self):
        super().__init__('CarShapeNode')
        self.scan_sub = self.create_subscription(PointCloud2,'scan_matched_points2', self.scan_sub,  qos_profile_sensor_data)
        self.map_sub = self.create_subscription(OccupancyGrid,'map', self.map_cb,  qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image,'camera1/image_raw', self.image_cb, qos_profile_sensor_data)

        self.br = CvBridge() # for image conversion
        timer_period = 0.1

        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.set_parameters(
            [Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        qos_profile = QoSProfile(depth=10)
      
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer,self, qos=qos_profile)


        self.get_logger().info('Started')

    def scan_sub(self, msg: PointCloud2):
        pcd_list = list(point_cloud2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True))


    def image_cb(self, data: Image):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

    def map_cb(self, msg: OccupancyGrid):
        pass
        
    def timer_cb(self):
        now = self.get_clock().now()
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', now, rclpy.duration.Duration(seconds=1, nanoseconds= 500))
            print(trans)
         
        except (LookupException, LookupError, ConnectionAbortedError, ConnectionError, ConnectionRefusedError, ConnectionResetError,ExtrapolationException,ConnectivityException) as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    try:
        shape_node = CarShapeNode()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(shape_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            shape_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()