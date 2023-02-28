#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
# Import the service message used by the service /switch for setting the gripper state
from std_srvs.srv import SetBool

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')
        self.get_logger().info("Gripper client node created")
    
        # Create a client to send a request to the service
        self.cli = self.create_client(SetBool, '/jetmax_switch')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.get_logger().info("Service is available")

        # Create a request
        self.req = SetBool.Request()

    def send_request(self, _data: bool):
        self.req.data = _data
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
    node = GripperClient()

    # # Send request spanning the entire workspace
    # node.get_logger().info("Sending request to open gripper...")
    # response = node.send_request(True)
    # # response = node.send_request(False)
    # node.get_logger().info("Response: %s" % response.success)
    

    # Spin the node
    # rclpy.spin(node)

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
