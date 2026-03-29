#!/usr/bin/env python3

# EPFL AI Team - SAPIEN

# Author: , 2026

"""
TODO: CREATE DESCRIPTION OF THE NODE
"""

import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    def __init__(self):
        super().__init__("node_name")
        

        self.get_logger().info("Node has been started")

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()

    rclpy.spin(node) # keep alive this node
    rclpy.shutdown() # kill this node


if __name__ == "__main__":
    main()