#!/usr/bin/env python3
import sys
import math
import rclpy
from std_msgs.msg import Float64
import jetmax_kinematics
from jetmax_control.srv import IK #, IKRequest, IKResponse
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray, Float32MultiArray

NUM_OF_JOINTS = 9
NUM_OF_ACTIVE_JOINTS = 3
joints_publishers = []
ik_service = None

class JetmaxIKService(Node):
    def __init__(self):
        super().__init__('jetmax_ik_service')
        self.ik_service = self.create_service(IK, '/jetmax_control/inverse_kinematics', self.ik_callback)
        self.get_logger().info("Jetmax IK service created")
        self.res = IK.Response()

        # Create a publisher to /joint_position_controller/commands
        self.pub = self.create_publisher(Float64MultiArray, '/joint_position_controller/commands', 1)
        self.get_logger().info("Joint publisher created")

        # Create a publisher to /datalog_IK
        self.pub_datalog = self.create_publisher(Float32MultiArray, '/datalog_IK', 10)
        self.get_logger().info("Datalog publisher created")

        # Create dummy FK and IK results to pad the datalogging when kinematics fail
        self.dummy_fk = [0] * NUM_OF_JOINTS
        self.dummy_ik = [0] * NUM_OF_ACTIVE_JOINTS

    def ik_callback(self, request, response):
        """
        This callback function executes whenever a inverse_kinematics service is requested
        @param request: IKRequest
        @return: IKResponse
        """
        position = [request.x, request.y, request.z]
        ik_result= jetmax_kinematics.inverse_kinematics(position)
        #self.get_logger().info("Publishing for position: {}".format(position))
        self.get_logger().info("Recieved IK request: x: %.4f, y: %.4f, z: %.4f" % (request.x, request.y, request.z))
        if ik_result:
            fk_result = jetmax_kinematics.forward_kinematics(ik_result)
            if fk_result:
                joint_angles = [deg * math.pi / 180 for deg in fk_result]
                # Set the arm joint angles / Publish the joint angles to the robot
                self.publish_joint_angles(joint_angles)
                self.res.success = True
                self.publish_datalog(position, ik_result, fk_result, 1)
            else:
                self.get_logger().info("FK(on IK) failed! Result: {}".format(fk_result))
                self.res.success = False
                self.publish_datalog(position, ik_result, self.dummy_fk, 0)
        else:
            self.get_logger().info("IK failed! Result: {}".format(ik_result))
            self.res.success = False
            self.publish_datalog(position, self.dummy_ik, self.dummy_fk, -1)

        return self.res
    
    def publish_joint_angles(self, joint_angles):
        msg = Float64MultiArray()
        msg.data = joint_angles
        self.pub.publish(msg)
        self.get_logger().info("Published joint angles: %s" % msg.data)

    def publish_datalog(self, position_cartesian, ik_result, fk_result, success):
        msg = Float32MultiArray()
        msg.data = [position_cartesian, ik_result, fk_result, success]
        self.pub_datalog.publish(msg)
        self.get_logger().info("Published datalog: %s" % msg.data)

    # def __del__(self):
    #     self.get_logger().info("Destroying Jetmax IK service")
    #     self.ik_service.destroy()
    
    # def destroy_node(self):
    #     self.ik_service.destroy()
    #     super().destroy_node()


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)
    
    # Create the node
    node = JetmaxIKService()

    # Spin the node
    rclpy.spin(node)

    # Destroy the node (explicitly)
    #node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()