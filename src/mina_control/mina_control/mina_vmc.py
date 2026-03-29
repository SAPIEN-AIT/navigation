#!/usr/bin/env python3

# EPFL AI Team - SAPIEN

# Author: , 2026

"""
TODO: CREATE DESCRIPTION OF THE NODE
"""

import rclpy
from rclpy.node import Node

class VirtualModelControllerNode(Node):
    def __init__(self):
        super().__init__("mina_vmc")
        self.get_logger().info("VMC has been started")
        
        is_simulated = self.get_parameter('use_sim_time').get_parameter_value().bool_value
 


def main(args=None):
    rclpy.init(args=args)
    node = VirtualModelControllerNode()

    rclpy.spin(node) # keep alive this node
    node.destroy_node() # clean up before shutting down
    rclpy.shutdown() # kill this node


if __name__ == "__main__":
    main()