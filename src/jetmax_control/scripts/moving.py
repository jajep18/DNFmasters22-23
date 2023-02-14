#!/usr/bin/env python3
import rclpy
from jetmax_control.srv import IK, IKRequest


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('go_home')
    moving = node.create_client("/jetmax_control/inverse_kinematics", IK)
    x, y, z = [0, 150, 50]
    while z < 200:
        z += 1
        moving(IKRequest(x, y, z))
        rclpy.sleep(0.05)
