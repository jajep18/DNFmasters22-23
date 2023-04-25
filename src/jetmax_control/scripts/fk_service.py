#!/usr/bin/env python3
import sys
import math
import rclpy
import jetmax_kinematics
from jetmax_control.srv import FK
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NUM_OF_JOINTS = 9
joints_publishers = []
fk_service = None

class JetmaxFKService(Node):
    def __init__(self):
        super().__init__('jetmax_fk_service')
        self.fk_service = self.create_service(FK, '/jetmax_control/forward_kinematics', self.fk_callback)
        self.get_logger().info("Jetmax FK service created")
        self.res = FK.Response()

        # Create a publisher to /joint_position_controller/commands with type std_msgs/msg/Float64MultiArray
        self.pub = self.create_publisher(Float64MultiArray, '/joint_position_controller/commands', 5)
        self.get_logger().info("Publisher created")

    def fk_callback(self, request, response):
        """
        This callback function executes whenever a forward_kinematics service is requested
        @param request: FKRequest
        @return: FKResponse
        """
        angles = [request.angle_rotate, request.angle_left, request.angle_right]
        fk_result = jetmax_kinematics.forward_kinematics(angles)
        
        if fk_result:
            self.get_logger().info("FK succeeded! Result: {}".format(fk_result))
            joint_angles = [deg * math.pi / 180 for deg in fk_result]
            # Set the arm angles / Publish the joint angles to the robot
            self.publish_joint_angles(joint_angles)
            self.res.success = True
        else:
            self.get_logger().info("FK failed! Result: {}".format(fk_result))
            #response.result = False
            self.res.success = False

        return self.res
    
    def publish_joint_angles(self, joint_angles):
        self.get_logger().info("Publishing joint angles...")
        msg = Float64MultiArray()
        msg.data = joint_angles
        self.pub.publish(msg)
        self.get_logger().info("Published joint angles: %s" % msg.data)


    def result_response(self, result):
        self.get_logger().info("Result: %s" % result.result,
                                 "Status: %s" % result.status)          


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    # Create the node
    node = JetmaxFKService()

    # Spin the node
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
