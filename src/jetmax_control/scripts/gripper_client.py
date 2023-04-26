#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
# Import the service message used by the service /switch for setting the gripper state
from std_srvs.srv import SetBool
from std_msgs.msg import Bool


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
        self.get_logger().info("Request created")
        # Create a subscription to '/jetmax_grasping
        self.subscription = self.create_subscription(Bool, '/jetmax_grasping', self.listener_callback, 1)
        self.get_logger().info("Subscription created")
        
        # Initialize loop_rate to 1 Hz - This allows loop_rate.sleep() to be called in the callbacks
        self.loop_rate = self.create_rate(0.4, self.get_clock()) # 0.25 Hz = 1 time pr 4 seconds

        # Manual throttle
        self.throttle_count = 0
        
    def listener_callback(self, msg):
        if self.throttle_count < 200:
            self.throttle_count += 1
            return
        else:
            self.throttle_count = 0
        self.get_logger().info("I heard on /jetmax_grasping: %s" % msg.data)
        #Add a delay to the callback
        # print("Taking a sleep")
        # self.loop_rate.sleep()
        # print("Waking up")
        # Send request to the service
        # if msg.data:
        #     self.get_logger().info("Finally - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - ")
        #     self.get_logger().info("Sending request to close gripper...")
        #     response = self.send_request(False)
        #     #self.get_logger().info("Response: %s" % response.success)
        # else:
        #     self.get_logger().info("Sending request to open gripper...")
        #     response = self.send_request(True)
        #     #self.get_logger().info("Response: %s" % response.success)

        # Add a delay to the callback
        # self.loop_rate.sleep()
        
    def send_request(self, _data: bool):
        self.req.data = _data
        # self.get_logger().info("Sending request to set gripper to: %s (status of grip)" % self.req.data)
        # return self.cli.call(self.req)

        self.future = self.cli.call_async(self.req)
        self.get_logger().info("Request sent!")
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("Request completed!")
        if self.future.result() is not None:
            self.get_logger().info("Result: %s" % self.future.result().success)
            self.get_logger().info("Message: %s" % self.future.result().message)
        else:
            self.get_logger().info("Service call failed. Exception: %r" % (self.future.exception(),))
        return self.future.result()

def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    # Create the node
    node = GripperClient()

    my_bool = True
    node.get_logger().info("Main: Sending request to set gripper to: %s (status of grip)" % my_bool)
    response = node.send_request(my_bool)
    #node.get_logger().info("Main: Response: %s" % response.success)

    
    # #Use MultiThreadedExecutor to enable multiple callbacks to be processed simultaneously
    # my_executor = rclpy.executors.MultiThreadedExecutor()
    # try:
    #     rclpy.spin(node, executor=my_executor)
    # except KeyboardInterrupt:
    #     pass
    # finally: # This is to make sure the program exits cleanly0
    #     # Destroy the node explicitly
    #     # (optional - otherwise it will be done automatically
    #     # when the garbage collector destroys the node object)
    #     node.destroy_node()
    #     rclpy.shutdown()


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
