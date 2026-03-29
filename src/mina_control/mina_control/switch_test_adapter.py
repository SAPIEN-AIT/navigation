#!/usr/bin/env python3

# EPFL AI Team - SAPIEN
# Author: Mattia Prandi, 2026

"""
DESCRIPTION:
    Acts as a bridge and data adapter between GUI-based joint state inputs, 
    a dynamic policy switcher, and the RViz visualization tool. It converts 
    named joint messages into ordered arrays for processing and reconstructs 
    them for feedback. Used only for testing

INPUT:
    - /policy_1/gui_joints (sensor_msgs/JointState): Joint positions from GUI 1.
    - /policy_2/gui_joints (sensor_msgs/JointState): Joint positions from GUI 2.
    - /cmd_action (std_msgs/Float64MultiArray): A 22-element blended/selected 
      action array from the dynamic switch.

OUTPUT:
    - /policy_1/arm_action (std_msgs/Float64MultiArray): 10-element arm array for P1.
    - /policy_1/leg_action (std_msgs/Float64MultiArray): 12-element leg array for P1.
    - /policy_2/arm_action (std_msgs/Float64MultiArray): 10-element arm array for P2.
    - /policy_2/leg_action (std_msgs/Float64MultiArray): 12-element leg array for P2.
    - /joint_states (sensor_msgs/JointState): Final reconstructed state for RViz.

LOGIC:
    1. Subscribes to JointState messages from two separate GUIs.
    2. Maps incoming named joints to fixed-index 10-element (arms) and 12-element 
       (legs) arrays to ensure consistency for policy consumption.
    3. Publishes these indexed arrays to their respective policy topics.
    4. Receives a combined 22-element array from the dynamic switcher.
    5. Re-attaches the original joint names to the switch output and publishes 
       it to RViz for real-time visualization.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class SwitchTestAdapter(Node):
    def __init__(self):
        super().__init__("switch_test_adapter")
        
        self.arm_joints = [
            'arm_left_elbow_roll_joint', 'arm_left_elbow_pitch_joint', 'arm_left_shoulder_yaw_joint', 'arm_left_shoulder_roll_joint', 'arm_left_shoulder_pitch_joint',
            'arm_right_elbow_roll_joint', 'arm_right_elbow_pitch_joint', 'arm_right_shoulder_yaw_joint', 'arm_right_shoulder_roll_joint', 'arm_right_shoulder_pitch_joint'
        ] # Must be 10 elements
        
        self.leg_joints = [
            'leg_left_ankle_roll_joint', 'leg_left_ankle_pitch_joint', 'leg_left_knee_pitch_joint', 'leg_left_hip_pitch_joint', 'leg_left_hip_yaw_joint', 'leg_left_hip_roll_joint',
            'leg_right_ankle_roll_joint', 'leg_right_ankle_pitch_joint', 'leg_right_knee_pitch_joint', 'leg_right_hip_pitch_joint', 'leg_right_hip_yaw_joint', 'leg_right_hip_roll_joint'
        ] # Must be 12 elements
        
        # Publishers to send data TO the dynamic switch
        self.p1_arm_pub = self.create_publisher(Float64MultiArray, "/policy_1/arm_action", 10)
        self.p1_leg_pub = self.create_publisher(Float64MultiArray, "/policy_1/leg_action", 10)
        
        self.p2_arm_pub = self.create_publisher(Float64MultiArray, "/policy_2/arm_action", 10)
        self.p2_leg_pub = self.create_publisher(Float64MultiArray, "/policy_2/leg_action", 10)
        
        # Publisher to send the final blended data TO RViz
        self.rviz_pub = self.create_publisher(JointState, "/joint_states", 10)
        
        # Subscribers to read data FROM the two GUIs
        self.create_subscription(JointState, "/policy_1/gui_joints", self.gui_1_cb, 10)
        self.create_subscription(JointState, "/policy_2/gui_joints", self.gui_2_cb, 10)
        
        # Subscriber to read data FROM the dynamic switch
        self.create_subscription(Float64MultiArray, "/cmd_action", self.switch_cb, 10)

        self.get_logger().info("Test Adapter started. Waiting for GUI inputs...")

    def extract_arrays(self, msg: JointState):
        """Extracts ordered arm and leg positions from a JointState message."""
        arm_positions = [0.0] * 10
        leg_positions = [0.0] * 12
        
        # Map the received names to their positions
        joint_dict = dict(zip(msg.name, msg.position))
        
        for i, name in enumerate(self.arm_joints):
            if name in joint_dict:
                arm_positions[i] = joint_dict[name]
                
        for i, name in enumerate(self.leg_joints):
            if name in joint_dict:
                leg_positions[i] = joint_dict[name]
                
        return arm_positions, leg_positions

    def gui_1_cb(self, msg: JointState):
        arm_pos, leg_pos = self.extract_arrays(msg)
        self.p1_arm_pub.publish(Float64MultiArray(data=arm_pos))
        self.p1_leg_pub.publish(Float64MultiArray(data=leg_pos))

    def gui_2_cb(self, msg: JointState):
        arm_pos, leg_pos = self.extract_arrays(msg)
        self.p2_arm_pub.publish(Float64MultiArray(data=arm_pos))
        self.p2_leg_pub.publish(Float64MultiArray(data=leg_pos))

    def switch_cb(self, msg: Float64MultiArray):
        """Receives the blended 22-element array and sends it to RViz."""
        if len(msg.data) != 22:
            return
            
        rviz_msg = JointState()
        rviz_msg.header.stamp = self.get_clock().now().to_msg()
        # Combine names and assign the blended positions
        rviz_msg.name = self.arm_joints + self.leg_joints
        rviz_msg.position = list(msg.data)
        
        self.rviz_pub.publish(rviz_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SwitchTestAdapter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()