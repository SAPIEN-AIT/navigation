#!/usr/bin/env python3

# EPFL AI Team - SAPIEN
# Author: , 2026

"""
TODO: CREATE DESCRIPTION OF THE NODE
"""

import rclpy
from rclpy.node import Node

class FSMNode(Node):
    def __init__(self):
        super().__init__("mina_fsm")
        self.get_logger().info("FSM has been started") 
                
        is_simulated = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        
        # TEST: this will be removed
        if is_simulated:
            self.get_logger().warning("SIMULATION mode")
        else:
            self.get_logger().warning("REAL HARDWARE mode")


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()

    rclpy.spin(node) # keep alive this node
    node.destroy_node() # clean up before shutting down
    rclpy.shutdown() # kill this nodeill this node


if __name__ == "__main__":
    main()