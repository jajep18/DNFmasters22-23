#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from jetmax_control.srv import FK, FKRequest # FKRequest, FKResponse
#from jetmax_fk import fk_callback

class GoHomeClient(Node):
    def __init__(self):
        super().__init__('go_home')
        self.get_logger().info("GoHomeClient node created")
        
        # Create a client to send a request to the service
        self.cli = self.create_client(FK, '/jetmax_control/forward_kinematics')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.get_logger().info("Service is available")

        # Create a request
        self.req = FK.Request()
        
    def send_request(self, _angle_rotate: float, _angle_left: float, _angle_right: float):
        self.req.angle_rotate = float(_angle_rotate)
        self.req.angle_left   = float(_angle_left)
        self.req.angle_right  = float(_angle_right)
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
    node = GoHomeClient()

    # Send a request using the nodes client
    node.get_logger().info("Sending request...")
    response = node.send_request(90, 90, 0)
    node.get_logger().info("Response: %s" % response.success)

    # Spin the node
    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

## DO it like this in ROS2
# https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide-Python.html
#add_two_ints = node.create_client(AddTwoInts, 'add_two_ints')
#while not add_two_ints.wait_for_service(timeout_sec=1.0):    
#resp = add_two_ints.call_async(req)
#rclpy.spin_until_future_complete(node, resp)
