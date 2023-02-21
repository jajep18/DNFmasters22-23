#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from jetmax_control.srv import FK # FKRequest, FKResponse (use FK.Request and FK.Response in ROS2)
#from jetmax_fk import fk_callback
# Import std_msgs/msg/Float64MultiArray
from std_msgs.msg import Float64MultiArray
import time

class JetmaxFKClient(Node):
    def __init__(self):
        super().__init__('fk_client')
        self.get_logger().info("FKClient (go_home) node created")
        
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
    node = JetmaxFKClient()

    # Send a request using the nodes client
    # node.get_logger().info("Sending request...")
    # response = node.send_request(90, 90, 0)
    # node.get_logger().info("Response: %s" % response.success)

    # Send a series of request with increasing joint angles
    for i in range(0, 240, 1):
        node.get_logger().info("Sending request on joint 1...")
        response = node.send_request(i, 90, 0)
        # Wait for some time / Delay
        time.sleep(0.1)
    
    # Repeat with second joint angle
    for i in range(30, 160, 1):
        node.get_logger().info("Sending request on joint 2...")
        response = node.send_request(80, i, 0)
        # Wait for some time / Delay
        time.sleep(0.1)

    # Repeat with third joint angle
    for i in range(-20, 90, 1):
        node.get_logger().info("Sending request on joint 3...")
        response = node.send_request(90, 90, i)
        # Wait for some time / Delay
        time.sleep(0.1)
        

    # Spin the node
    #rclpy.spin(node)

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

## Old code (ROS1)
# if __name__ == "__main__":
#     rospy.init_node("go_home")
#     go_home = rospy.ServiceProxy("/jetmax_control/forward_kinematics", FK) #Node der kalder en service. Skal laves om til ROS2
#     ret = go_home(FKRequest(90, 90, 0))
#     rospy.logdebug(ret)
