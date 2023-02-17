#!/usr/bin/env python3
import sys
import math
import rclpy
import jetmax_kinematics
from jetmax_control.srv import FK
import numpy as np
from rclpy.node import Node

NUM_OF_JOINTS = 9
joints_publishers = []
fk_service = None

class JetmaxFKService(Node):
    def __init__(self):
        super().__init__('jetmax_fk_service')
        self.fk_service = self.create_service(FK, '/jetmax_control/forward_kinematics', self.fk_callback)
        self.get_logger().info("Jetmax FK service created")
        self.res = FK.Response()


    def fk_callback(self, request, response):
        """
        This callback function executes whenever a forward_kinematics service is requested
        @param request: FKRequest
        @return: FKResponse
        """
        global joints_publishers
        angles = [request.angle_rotate, request.angle_left, request.angle_right]
        fk_result = jetmax_kinematics.forward_kinematics(angles)
        
        if fk_result:
            self.get_logger().info("FK succeeded! Result: {}".format(fk_result))
            joint_angles = [deg * math.pi / 180 for deg in fk_result]
            # for i in range(NUM_OF_JOINTS):
            #     joints_publishers[i].publish(np.float64(joint_angles[i]))
            self.res.success = True
            # self.res.angle_rotate = fk_result[0]
            # self.res.angle_left = fk_result[1]
            # self.res.angle_right = fk_result[2]
        else:
            self.get_logger().info("FK failed! Result: {}".format(fk_result))
            #response.result = False
            self.res.success = False

        return self.res


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    # Create the node
    node = JetmaxFKService()

    # Spin the node
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()
    
    # for i in range(NUM_OF_JOINTS): # This 
    #     pub = node.create_publisher(np.float64,
    #         "/jetmax/joint{}_position_controller/command".format(i + 1), 10)
    #     joints_publishers.append(pub)

if __name__ == "__main__":
    main()
