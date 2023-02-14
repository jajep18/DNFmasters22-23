#!/usr/bin/env python3
import rclpy
from jetmax_control.srv import FK, FKRequest, FKResponse

if __name__ == "__main__":
    rclpy.init_node("go_home")
    go_home = rclpy.ServiceProxy("/jetmax_control/forward_kinematics", FK) #Node der kalder en service. Skal laves om til ROS2
    ret = go_home(FKRequest(90, 90, 0))
    rclpy.logdebug(ret)
    rclpy.shutdown("go_home")
   