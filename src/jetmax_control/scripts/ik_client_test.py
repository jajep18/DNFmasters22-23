#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from jetmax_control.srv import IK #, IKRequest(use IK.Request instead in ROS2)
import time

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
    for i in range(120, 200, 5):
        time.sleep(0.5)
        position = [0, -175, i]
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