#!/usr/bin/env python3
import math

from geometry_msgs.msg import Twist, TransformStamped, Transform

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrameListener(Node):

    def __init__(self):
        super().__init__('frame_listener')

        # Define target frame
        self._target_frame = 'Link9'
        self._source_frame = 'Base'
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Create a publisher for the transformation result
        self._publisher = self.create_publisher(TransformStamped, 'base_TCP_transform', 1)

        # Create a timer 
        self._timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        from_frame_rel = self._target_frame
        to_frame_rel = self._source_frame
        try:
            when = self.get_clock().now() - rclpy.time.Duration(seconds=2.0)
            t = self.tf_buffer.lookup_transform_full(
                target_frame=to_frame_rel,
                target_time=rclpy.time.Time(),
                source_frame=from_frame_rel,
                source_time=when,
                fixed_frame='world',
                timeout=rclpy.duration.Duration(seconds=0.05))
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Create a TransformStamped message
        msg = TransformStamped()
        # Set the header
        # Set time stamp to current time to avoid TF error "Lookup would require extrapolation into the future"
         
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = to_frame_rel
        msg.child_frame_id = from_frame_rel

        # Set the transform
        msg.transform.translation.x = t.transform.translation.x
        msg.transform.translation.y = t.transform.translation.y
        msg.transform.translation.z = t.transform.translation.z
        # Set the rotation
        msg.transform.rotation.x = t.transform.rotation.x
        msg.transform.rotation.y = t.transform.rotation.y
        msg.transform.rotation.z = t.transform.rotation.z
        msg.transform.rotation.w = t.transform.rotation.w
        # Publish the message
        self._publisher.publish(msg)



def main(args=None):
    rclpy.init()
    node = FrameListener()

    rclpy.spin(node)

    rclpy.shutdown()

        
if __name__ == "__main__":
    main()