#!/usr/bin/env python3
import sys
import math
import rclpy
from std_msgs.msg import Float64
import jetmax_kinematics
from jetmax_control.srv import IK #, IKRequest, IKResponse
from rclpy.node import Node
import numpy as np

NUM_OF_JOINTS = 9
joints_publishers = []
ik_service = None

class JetmaxIKService(Node):
    def __init__(self):
        super().__init__('jetmax_ik_service')
        self.ik_service = self.create_service(IK, '/jetmax_control/inverse_kinematics', self.ik_callback)
        self.get_logger().info("Jetmax IK service created")
        self.res = IK.Response()

    def ik_callback(self, request, response):
        """
        This callback function executes whenever a inverse_kinematics service is requested
        @param request: IKRequest
        @return: IKResponse
        """
        global joints_publishers
        position = request.x, request.y, request.z
        ik_result= jetmax_kinematics.inverse_kinematics(position)
        if ik_result:
            fk_result = jetmax_kinematics.forward_kinematics(ik_result)
            if fk_result:
                joint_angles = [deg * math.pi / 180 for deg in fk_result]
                # Set the arm joint angles
                for i in range(NUM_OF_JOINTS):
                    joints_publishers[i].publish(np.float64(joint_angles[i]))
                self.res.success = True
            else:
                self.get_logger().info("FK(on IK) failed! Result: {}".format(fk_result))
                self.res.success = False
        else:
            self.get_logger().info("IK failed! Result: {}".format(ik_result))
            self.res.success = False

        return self.res

    # def __del__(self):
    #     self.get_logger().info("Destroying Jetmax IK service")
    #     self.ik_service.destroy()
    
    # def destroy_node(self):
    #     self.ik_service.destroy()
    #     super().destroy_node()


def main(args=None):
    # Joint publishers: These are used to publish the joint angles to the joint controllers
    #global joints_publishers
    # for i in range(NUM_OF_JOINTS):
    #     pub = node.create_publisher(np.float64, "/jetmax/joint{}_position_controller/command".format(i + 1))
    #     joints_publishers.append(pub)

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