#!/usr/bin/env python3
import sys
import math
import rclpy
from std_msgs.msg import Float64
import jetmax_kinematics
from jetmax_control.srv import IK #, IKRequest, IKResponse

NUM_OF_JOINTS = 9
joints_publishers = []
ik_service = None


# This callback function executes whenever a inverse_kinematics service is requested
def ik_callback(req):
    global joints_publishers
    position = req.x, req.y, req.z
    ik_result= jetmax_kinematics.inverse_kinematics(position)
    if ik_result:
        fk_result = jetmax_kinematics.forward_kinematics(ik_result)
        if fk_result:
            joint_angles = [deg * math.pi / 180 for deg in fk_result]
            # Set the arm joint angles
            for i in range(NUM_OF_JOINTS):
                joints_publishers[i].publish(Float64(joint_angles[i]))
            return [True, ]
    return [False, ]

def main():
    global joints_publishers, fk_service
    rclpy.init()
    node = rclpy.create_node('ik_jetmax')
    
    for i in range(NUM_OF_JOINTS):
        pub = node.create_publisher(np.float64, "/jetmax/joint{}_position_controller/command".format(i + 1))
        joints_publishers.append(pub)

    fk_service = node.create_service(IK, "/jetmax_control/inverse_kinematics", ik_callback)
    node.get_logger().info("Ready to send joint angles")
    try:
        rclpy.spin()
    except Exception as e:
        rclpy.logerr(e)
        sys.exit(-1)
    
if __name__ == "__main__":
    main()