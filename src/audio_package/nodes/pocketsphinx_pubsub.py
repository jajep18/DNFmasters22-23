#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# from pocketsphinx import Decoder
# import argparse
# import wave


class Pocketsphinx_Pubsub(Node):

    def __init__(self):
        super().__init__('pocketsphinx_node')
        self.publisher_ = self.create_publisher(String, 'keywords_topic', 10)
        timer_period = 15  # seconds
        self.timer = self.create_timer(timer_period, self.kwspotting_callback)
        self.i = 0
        

    def kwspotting_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    pocketsphinx_node = Pocketsphinx_Pubsub()
    rclpy.spin(pocketsphinx_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pocketsphinx_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()