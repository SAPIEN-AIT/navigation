#!/usr/bin/env python3

# EPFL AI Team - SAPIEN
# Author: Mattia Prandi, 2026

"""
DESCRIPTION:
    Acts as a centralized multiplexer and command smoother for humanoid robot 
    control. It routes commands from multiple control sources to the robot 
    actuators, allowing independent switching for arms and legs while 
    ensuring mechanical safety through signal blending.

INPUT:
    - /<control_source>/arm_action (std_msgs/Float64MultiArray): 10-element 
      position commands for the arms from a specific controller.

    - /<control_source>/leg_action (std_msgs/Float64MultiArray): 12-element 
      position commands for the legs from a specific controller.

    - /switch_control (mina_interfaces/srv/SwitchControl): Service to trigger 
      a transition to a different arm or leg control source.

OUTPUT:
    - /cmd_action (std_msgs/Float64MultiArray): The final, unified 22-element 
      command stream (concatenated arms and legs) published at a fixed frequency.

LOGIC:
    1. Dynamically initializes subscribers for all controllers defined in the 
       'control_sources' parameter.

    2. Maintains a buffer of the latest commands received from every source.

    3. On a service request, it validates that the target source is active and 
       within a safe joint-delta threshold (max_safe_delta) to prevent jumps.

    4. Executes a Linear Interpolation (LERP) transition between the current 
       state and the new target over a specified 'transition_duration'.
       
    5. Continuous Loop: Blends active signals and publishes the 22-element 
       concatenated array to ensure the low-level hardware remains active.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from mina_interfaces.srv import SwitchControl # Make sure to compile this custom service
import numpy as np

class DynamicSwitchNode(Node):
    def __init__(self):
        super().__init__("dynamic_switch")
        
        # --- Parameters ---
        # List of available controller names (e.g., ['locomotion', 'il_policy', 'vmc'])
        self.declare_parameter('control_sources', ['locomotion'])
        # Transition duration in seconds to blend commands
        self.declare_parameter('transition_duration', 0.5)
        # Control loop frequency in Hz
        self.declare_parameter('control_frequency', 50.0)
        # Maximum allowed joint difference (radians) to accept a switch
        self.declare_parameter('max_safe_delta', 1.0)
        
        self.sources = self.get_parameter('control_sources').get_parameter_value().string_array_value
        self.transition_duration = self.get_parameter('transition_duration').get_parameter_value().double_value
        self.control_rate = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.max_delta = self.get_parameter('max_safe_delta').get_parameter_value().double_value
        
        # --- State Variables ---
        self.latest_arm_msgs = {source: None for source in self.sources}
        self.latest_leg_msgs = {source: None for source in self.sources}
        
        # Default to the first control_sources in the list if available
        default_controller = self.sources[0] if self.sources else ""
        self.active_arm_source = default_controller
        self.active_leg_source = default_controller
        
        # Blending state
        self.is_transitioning = False
        self.transition_start_time = 0.0
        self.blend_start_arms = np.zeros(10)
        self.blend_start_legs = np.zeros(12)
        
        # --- Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(Float64MultiArray, "/cmd_action", 10)
        
        self.arm_subs = {}
        self.leg_subs = {}
        
        for source in self.sources:
            # Using lambda with default argument to capture the source name correctly in the callback
            self.arm_subs[source] = self.create_subscription(
                Float64MultiArray, 
                f"/{source}/arm_action", 
                lambda msg, s=source: self.arm_callback(msg, s), 
                10
            )
            self.leg_subs[source] = self.create_subscription(
                Float64MultiArray, 
                f"/{source}/leg_action", 
                lambda msg, s=source: self.leg_callback(msg, s), 
                10
            )
            
        # --- Services ---
        self.srv = self.create_service(SwitchControl, '/switch_control', self.switch_control_callback)
        
        # --- Main Control Loop ---
        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info(f"Dynamic switch started. Available sources: {self.sources}")

    def arm_callback(self, msg: Float64MultiArray, source: str):
        """Stores the latest arm action from a specific controller."""
        if len(msg.data) == 10:
            self.latest_arm_msgs[source] = np.array(msg.data)
        else:
            self.get_logger().warn(f"[{source}] Invalid arm action length: {len(msg.data)}. Expected 10.")

    def leg_callback(self, msg: Float64MultiArray, source: str):
        """Stores the latest leg action from a specific controller."""
        if len(msg.data) == 12:
            self.latest_leg_msgs[source] = np.array(msg.data)
        else:
            self.get_logger().warn(f"[{source}] Invalid leg action length: {len(msg.data)}. Expected 12.")

    def switch_control_callback(self, request, response):
        """Handles requests to switch the active controllers for arms and legs."""
        req_arm = request.arm_target
        req_leg = request.leg_target
        
        # 1. Validation: Check if requested sources exist
        if req_arm not in self.sources or req_leg not in self.sources:
            response.success = False
            response.message = f"Invalid source requested. Available: {self.sources}"
            self.get_logger().error(response.message)
            return response
            
        # 2. Validation: Check if requested sources are actually publishing data
        if self.latest_arm_msgs[req_arm] is None or self.latest_leg_msgs[req_leg] is None:
            response.success = False
            response.message = "Requested source has not published any data yet (Dead node?)."
            self.get_logger().error(response.message)
            return response

        # 3. Validation: Safety delta check (Prevent explosive jumps)
        # Compute the difference between current output and the new target
        current_target_arms = self.latest_arm_msgs[self.active_arm_source]
        current_target_legs = self.latest_leg_msgs[self.active_leg_source]
        
        # If this is the very first loop, allow it. Otherwise check deltas.
        if current_target_arms is not None and current_target_legs is not None:
            new_target_arms = self.latest_arm_msgs[req_arm]
            new_target_legs = self.latest_leg_msgs[req_leg]
            
            arm_delta = np.max(np.abs(current_target_arms - new_target_arms))
            leg_delta = np.max(np.abs(current_target_legs - new_target_legs))
            
            if arm_delta > self.max_delta or leg_delta > self.max_delta:
                response.success = False
                response.message = f"Safety rejected: Delta too large (Arm:{arm_delta:.2f}, Leg:{leg_delta:.2f} > Limit:{self.max_delta})"
                self.get_logger().error(response.message)
                return response

        # --- Apply Switch and Start Transition ---
        # Save the exact current state to blend from
        self.blend_start_arms = current_target_arms if current_target_arms is not None else self.latest_arm_msgs[req_arm]
        self.blend_start_legs = current_target_legs if current_target_legs is not None else self.latest_leg_msgs[req_leg]
        
        self.active_arm_source = req_arm
        self.active_leg_source = req_leg
        
        self.is_transitioning = True
        self.transition_start_time = self.get_clock().now().nanoseconds / 1e9
        
        response.success = True
        response.message = f"Switching... Arms -> {req_arm}, Legs -> {req_leg}"
        self.get_logger().info(response.message)
        
        return response

    def control_loop(self):
        """Runs at fixed frequency. Blends and publishes the final cmd_action."""
        target_arms = self.latest_arm_msgs[self.active_arm_source]
        target_legs = self.latest_leg_msgs[self.active_leg_source]
        
        # Do not publish if we haven't received data from the active sources yet
        if target_arms is None or target_legs is None:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Output arrays
        final_arms = target_arms
        final_legs = target_legs
        
        # Apply blending if in transition phase
        if self.is_transitioning:
            elapsed_time = current_time - self.transition_start_time
            alpha = elapsed_time / self.transition_duration
            
            if alpha >= 1.0:
                self.is_transitioning = False
                alpha = 1.0
                
            # Linear interpolation (LERP): (1 - alpha) * start + alpha * target
            final_arms = (1.0 - alpha) * self.blend_start_arms + alpha * target_arms
            final_legs = (1.0 - alpha) * self.blend_start_legs + alpha * target_legs

        # Construct and publish the final combined message (22 elements)
        out_msg = Float64MultiArray()
        out_msg.data = np.concatenate((final_arms, final_legs)).tolist()
        self.cmd_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()