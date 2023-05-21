#!/usr/bin/env python3
import sys
import math
import rclpy
from std_msgs.msg import Float64
import jetmax_kinematics
from jetmax_control.srv import IK #, IKRequest, IKResponse
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray

NUM_OF_JOINTS = 9

class EffortControllerService(Node):
    def __init__(self):
        super().__init__('effort_controller_service')
        # Create a service for the effort controller (This node works as the service server)
        self.srv = self.create_service(IK, '/jetmax_control/effort_controller_service', self.effort_controller_callback)

        # Create a publisher for the effort controller
        self.publisher = self.create_publisher(Float64MultiArray, 'effort_controller', 10)
        self.get_logger().info("Effort Controller Service is ready")

    def effort_controller_callback(self, request, response):
        '''
        This function is called when the effort controller service is called.
        It calculates the inverse kinematics and publishes the joint efforts to the effort controller.
        Input:  request (IKRequest): Containing desired end effector efforts
        Output: response (IKResponse): Containing success status
        Publish
        '''
        # Get the request
        x = request.x
        y = request.y
        z = request.z
        roll = request.roll
        pitch = request.pitch
        yaw = request.yaw
        # Calculate the inverse kinematics
        joint_angles = jetmax_kinematics.inverse_kinematics(x, y, z, roll, pitch, yaw)
        # Create a message for the effort controller
        msg = Float64MultiArray()
        msg.data = joint_angles
        # Publish the message
        self.publisher.publish(msg)
        # Return the response
        response.success = True
        return response
    
    def publish_joint_efforts(self, joint_efforts):
        msg = Float64MultiArray()
        msg.data = joint_efforts
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    effort_controller_service = EffortControllerService()
    rclpy.spin(effort_controller_service)
    effort_controller_service.destroy_node()
    rclpy.shutdown()
