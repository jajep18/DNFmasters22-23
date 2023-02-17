#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from jetmax_control.srv import IK #, IKRequest(use IK.Request instead in ROS2)

class JetmaxIKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.get_logger().info("IKClient (moving) node created")
    
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

    # Send a request using the nodes client
    node.get_logger().info("Sending request...")
    response = node.send_request(0, 150, 50)
    node.get_logger().info("Response: %s" % response.success)

    # Spin the node
    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

    
# Old code
# rclpy.init()
# node = rclpy.create_node('go_home')
# moving = node.create_client("/jetmax_control/inverse_kinematics", IK)
# x, y, z = [0, 150, 50]
# while z < 200:
#     z += 1
#     moving(IKRequest(x, y, z))
#     rclpy.sleep(0.05)
