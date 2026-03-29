#!/usr/bin/env python3

# EPFL AI Team - SAPIEN

# Author: , 2026

"""
TODO: CREATE DESCRIPTION OF THE NODE
"""

import rclpy
from rclpy.node import Node

class CANDriverNode(Node):
    def __init__(self):
        super().__init__("can_driver")
        

        self.get_logger().info("CAN driver has been started")

def main(args=None):
    rclpy.init(args=args)
    node = CANDriverNode()

    rclpy.spin(node) # keep alive this node
    rclpy.shutdown() # kill this node


if __name__ == "__main__":
    main()