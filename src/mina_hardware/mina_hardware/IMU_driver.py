#!/usr/bin/env python3

# EPFL AI Team - SAPIEN

# Author: , 2026

"""
TODO: CREATE DESCRIPTION OF THE NODE
"""

import rclpy
from rclpy.node import Node

class IMUDriverNode(Node):
    def __init__(self):
        super().__init__("imu_driver")
        

        self.get_logger().info("IMU driver has been started")

def main(args=None):
    rclpy.init(args=args)
    node = IMUDriverNode()

    rclpy.spin(node) # keep alive this node
    rclpy.shutdown() # kill this node


if __name__ == "__main__":
    main()