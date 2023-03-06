#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pytorch as pt




class DNF_Node(Node):
    def __init__(self):
        super().__init__('dnf_node')
        self.subscriber = self.create_subscription(String, 'keywords', self.subscriber_callback, 1)



