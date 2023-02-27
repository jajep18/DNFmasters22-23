#!/usr/bin/env python3
import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from jetmax_control.ik_client.py import JetmaxIKClient
from jetmax_control.scripts.gripper_client import GripperClient
from jetmax_control.scripts.fk_client import JetmaxFKClient


# This node is responsible for receiving the TCP transform from the camera 
# and reciving the real world pose of the balls from the camera
# Should then send the robot to the ball and pick it up

class MovementCommandCenter(Node):
    def __init__(self):
        super().__init__('movement_command_center')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._timer = self.create_timer(0.1, self.get_transform)
        self._subscriber = self.create_subscription(TransformStamped, 'base_TCP_transform', self.get_transform, 10)
        
        # create a service to send commands to the robot
        self._ik_client = JetmaxIKClient()
        self._gripper_client = GripperClient()
        self._transform = None

    def get_transform(self, msg):
        if msg is not None:
            # Check if header time is older than 5 seconds
            if (msg.header.stamp.sec + 5) < self.get_clock().now().sec:
                self.get_logger().info('Transform is older than 5 seconds')
                return
            else:
                self._transform = msg.transform
                self.get_logger().info('Recieved transform: %s' % self._transform)
        else:
            self.get_logger().info('No transform recieved')

def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    # Create the node
    node = MovementCommandCenter()
    
    # Spin the node
    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
