#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from jetmax_control.srv import IK #, IKRequest(use IK.Request instead in ROS2)
import time
import numpy as np

class JetmaxIKClient(Node):
    def __init__(self):
        super().__init__('ik_client_movement_test')
        self.get_logger().info("IKClientTest (Movement Test) node created")
    
        # Create a client to send a request to the service
        self.cli = self.create_client(IK, '/jetmax_control/inverse_kinematics')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.get_logger().info("Service is available")

        # Create a request
        self.req = IK.Request()

    def send_request(self, _x: float, _y: float, _z: float):
        self.req.x = float(_x)
        self.req.y = float(_y)
        self.req.z = float(_z)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info("Result: %s" % self.future.result().success)
        else:
            self.get_logger().info("Service call failed %r" % (self.future.exception(),))
        return self.future.result()

def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    # Create the node
    node = JetmaxIKClient()

    # Home: (0, 150, 50) ?
    # Send a request to move home
    # node.get_logger().info("Sending request to move home...")
    # position = [0, -175, 120]
    # # position = [120, 0, 200]
    # # position = [0, 100, 200]
    # node.get_logger().info("Position: {}".format(position))
    # response = node.send_request(position[0], position[1], position[2])
    # node.get_logger().info("Response: %s" % response.success)

    # for loop from 120 to 200 in 10 steps
    # for i in range(120, 200, 5):
    #     time.sleep(0.5)
    #     position = [0, -175, i]
    #     node.get_logger().info("Moving to Position: {}".format(position))
    #     response = node.send_request(position[0], position[1], position[2])
    #     node.get_logger().info("Response: %s" % response.success)

    # x_min = -0.5
    # x_max = 0.5
    # y_min = -0.5
    # y_max = 0.5
    # z_min = 0.1
    # z_max = 0.5

    # x_min = -166.6666717529297
    # x_max =  166.6666717529297
    # y_min = -166.6666717529297  
    # y_max =  166.6666717529297
    # z_min =  100
    # z_max =  322

    x_min = -300
    x_max =  300
    y_min = -300  
    y_max =  300
    z_min =  100
    z_max =  400

    n_points = 25
    # Generate a grid of points
    x_points = np.linspace(x_min, x_max, n_points) # X points between x_min and x_max
    y_points = np.linspace(y_min, y_max, n_points) # X points between y_min and y_max   
    z_points = np.linspace(z_min, z_max, n_points) # X points between z_min and z_max



    # Scale




    # Loop through x, y, z
    for x in x_points:
        for y in y_points:
            for z in z_points:

                position = [x, y, z] 
                # print("Position: ", position)
                node.get_logger().info("Moving to Position: {}".format(position))
                response = node.send_request(position[0], position[1], position[2])
                node.get_logger().info("Response: %s" % response.success)
                

    # Spin the node
    #rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()