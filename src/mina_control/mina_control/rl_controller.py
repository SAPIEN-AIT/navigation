#!/usr/bin/env python3

# EPFL AI Team - SAPIEN

# Author: , 2026

"""
TODO: CREATE DESCRIPTION OF THE NODE
"""

import rclpy
from rclpy.node import Node

class RLControllerNode(Node):
    def __init__(self):
        super().__init__("rl_controller")
        self.get_logger().info("RL controller has been started")

        self.declare_parameter('policy_path', "../policies/model.onnx")
        
        is_simulated = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        policy_path = self.get_parameter("policy_path").get_parameter_value().string_value
        
        # TEST: this will be removed
        if is_simulated:
            self.get_logger().warning("Policy is in: " + policy_path)
        else:
            self.get_logger().warning("Policy is in: " + policy_path)

def main(args=None):
    rclpy.init(args=args)
    node = RLControllerNode()

    rclpy.spin(node) # keep alive this node
    node.destroy_node() # clean up before shutting down
    rclpy.shutdown() # kill this node


if __name__ == "__main__":
    main()