#!/usr/bin/env python3
import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# This node is responsible for receiving the TCP transform from the camera 
# and reciving the real world pose of the balls from the camera
# Should then send the robot to the ball and pick it up

class MovementCommandCenter(Node):
    def __init__(self):
        super().__init__('movement_command_center')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._timer = self.create_timer(0.1, self.command_robot)
        self._subscriper = self.create_subscription(TransformStamped, 'cam_TCP_transform', self.command_robot, 10)
        # create a service to send commands to the robot
        

    def command_robot(self, ):
        #do stuff
        pass
