#!/usr/bin/env python3
import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import time
import asyncio #Implements sleep in a non-blocking manner
import threading

from enum import Enum

# from jetmax_control.scripts.ik_client import JetmaxIKClient
# from jetmax_control.scripts.gripper_client import GripperClient
# from jetmax_control.scripts.fk_client import JetmaxFKClient

from jetmax_control.srv import IK
from jetmax_control.srv import FK
from std_srvs.srv import SetBool

from custom_msgs.msg import TriangulatedCircleInfoArr

class Color(Enum):
    RED = 0
    GREEN = 1
    BLUE = 2
# This node is responsible for receiving the TCP transform from the camera 
# and reciving the real world pose of the balls from the camera
# Should then send the robot to the ball and pick it up

class MovementCommandCenter(Node):
    def __init__(self):
        super().__init__('movement_command_center')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        #self._timer = self.create_timer(0.1, self.get_transform)
        self._subscriber_trans = self.create_subscription(TransformStamped, 'base_TCP_transform', self.get_transform, 10)
        self._subscriber_coords = self.create_subscription(TriangulatedCircleInfoArr, 'triangulated_circle_topic', self.get_coordinates, 10)
        self._transform = None
        self._ball_coordinates =[[None, None, None], [None, None, None], [None, None, None]] # Red, Green, Blue

        # Create a client for the IK service
        self._ik_client = self.create_client(IK, '/jetmax_control/inverse_kinematics')
        while not self._ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("IK Service not available, waiting again...")
        self.get_logger().info("IK Service is available")

        # # Create a client for the FK service
        # self._fk_client = self.create_client(FK, '/jetmax_control/forward_kinematics')
        # while not self._fk_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("FK Service not available, waiting again...")
        # self.get_logger().info("FK Service is available")

        # Create a client for the gripper service
        self._gripper_client = self.create_client(SetBool, '/jetmax_switch')
        while not self._gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Gripper Service not available, waiting again...")
        self.get_logger().info("Gripper Service is available")

        # Create requests for the clients
        self._ik_req = IK.Request()
        self._gripper_req = SetBool.Request()
        # self._fk_req = FK.Request()

    def send_ik_request(self, _x: float, _y: float, _z: float):
        scale = 1000
        self._ik_req.x = float(scale * _x)
        self._ik_req.y = float(scale * _y)
        self._ik_req.z = float(scale * _z)
        self.get_logger().info("Sending IK request with x: %s, y: %s, z: %s" % (self._ik_req.x, self._ik_req.y, self._ik_req.z))
        self.future = self._ik_client.call_async(self._ik_req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=5)
        if self.future.result() is not None:
            self.get_logger().info("IK Request sent, Result: %s" % self.future.result().success)
        else:
            self.get_logger().info("IK Request failed %r" % (self.future.exception(),)) 
            return self.future.exception()
        return self.future.result().success
    
    def send_gripper_request(self, _open_data: bool):
        self._gripper_req.data = _open_data
        self.future = self._gripper_client.call_async(self._gripper_req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=5)
        if self.future.result() is not None:
            self.get_logger().info("Gripper Request sent, Result: %s" % self.future.result().success)
        else:
            self.get_logger().info("Gripper Request failed %r" % (self.future.exception(),)) 
            return self.future.exception()
        return self.future.result().success
    
    # def send_fk_request(self, _angle_rotate: float, _angle_left: float, _angle_right: float):
    #     self._fk_req.angle_rotate = _angle_rotate
    #     self._fk_req.angle_left = _angle_left
    #     self._fk_req.angle_right = _angle_right
    #     self.future = self._fk_client.call_async(self._fk_req)
    #     rclpy.spin_until_future_complete(self, self.future)
    #     if self.future.result() is not None:
    #         self.get_logger().info("FK Request sent, Result: %s" % self.future.result().result)
    #     else:
    #         self.get_logger().info("FK Request failed %r" % (self.future.exception(),)) 
    #     return self.future.result().result
        
        

    def get_transform(self, msg):
        if msg is not None:
            # Check if header time is older than 5 seconds
            if (msg.header.stamp.sec + 5) < (self.get_clock().now()).nanoseconds / 1e9:
                self.get_logger().info('Transform is older than 5 seconds')
                return
            else:
                self._transform = msg.transform
                #self.get_logger().info('Recieved transform: %s' % self._transform)
        else:
            self.get_logger().info('No transform recieved')
            
    def get_coordinates(self, msg):
        # Get the coordinates of the ball from the camera
        if msg is not None:
            for circles in msg.circles:
                if circles.color == 'red':
                    self._ball_coordinates[0] = [circles.x, circles.y, 0.0]
                elif circles.color == 'green':
                    self._ball_coordinates[1] = [circles.x, circles.y, 0.0]
                elif circles.color == 'blue':
                    self._ball_coordinates[2] = [circles.x, circles.y, 0.0]
        else: 
            self.get_logger().info('No coordinates recieved')

    def send_movement_command(self, color: str):
        
        # Check if color is valid
        if color != 'red' and color != 'green' and color != 'blue':
            self.get_logger().info('Invalid color passed')
            return
        
        # Check if transform is available
       # if self._transform is None:
        #    self.get_logger().info('No transform available')
        #    return
        
        # Get the coordinates of the ball from the color passed
        coordinates = self._ball_coordinates[Color[color.upper()].value]

        # Check if coordinates are available
        if coordinates[0] is None or coordinates[1] is None or coordinates[2] is None:
            self.get_logger().info('No coordinates available')
            return
        
        
        
        # Calculate the x, y, z coordinates of the ball
        
        # Create a transform for the coordinates
        #coor_trans = TransformStamped()
        #coor_trans.x = coordinates[0]
        #coor_trans.y = coordinates[1]
        #coor_trans.z = 0
        
        #coor_trans.rotation.x = 0
        #coor_trans.rotation.y = 0
        #coor_trans.rotation.z = 0
        
        #transformed_coor = coor_trans * self._transform
        
        #rot = [self._transform.rotation.x, self._transform.rotation.y, self._transform.rotation.z, self._transform.rotation.w]
        #trans = [self._transform.translation.x, self._transform.translation.y, self._transform.translation.z]

       # coordinates = coordinates * self._transform.rotation + self._transform.translation
       
        coordinates[2] = 0.025*2  + 0.08 # Z coordinate is always 0.05m, as r = 0.025m + 0.15m for the height of the table
        
        # Offset on y because of large triangulation error
        coordinates[1] = coordinates[1] + 0.038

        self.get_logger().info('Coordinates: %s' % coordinates)
        
        # Send the coordinates to the IK service
        return self.send_ik_request(coordinates[0], coordinates[1], coordinates[2])


        
        

def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    

    # Create the node
    node = MovementCommandCenter()
    
    # # Spin the node
    # rclpy.spin(node)

    # Spin in a seperate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Create rate object to "sleep" in a non-blocking manner
    #rate = node.create_rate(0.5, node.get_clock())

    #rate.sleep()
    
    # Move the arm outta the goddang way
    node.get_logger().info('Moving arm outta the way')
    position = [0.120, 0.0, 0.200] # Summer home position
    node.send_ik_request(position[0], position[1], position[2])
    node.get_logger().info('Arm moved outta the way')
    
    while node._ball_coordinates[0][0] is None:
        node.get_logger().info('Waiting for ball coordinates')
        time.sleep(0.5)


    # rclpy.sleep(2)
    # Test the gripper
    time.sleep(2)
    # Send the movement command as a test
    node.get_logger().info('Sending movement command for red ball')
    node.send_movement_command('red') # Valid color - should work
    node.get_logger().info('Turn on the big suck')
    node.send_gripper_request(True)
    time.sleep(0.5)
    
    node.get_logger().info('Moving arm up')
    position = node._ball_coordinates[Color["RED"].value]
    position[2] = 0.025*2  + 0.2 
    position[1] = position[1] + 0.038

    node.get_logger().info('Arm above ball')
    time.sleep(5)

    node.get_logger().info('Turn off the big suck')
    node.send_gripper_request(False)

    time.sleep(4)

    # for i in range(0, 10):
    #     # rate.sleep()
    #     time.sleep(2)
    #     node.get_logger().info('Sending movement command for red ball')
    #     node.send_movement_command('red')
    #     node.get_logger().info('Exited movement cmd for red ball')
        
    #     time.sleep(2)
    #     node.get_logger().info('Sending movement command for green ball')
    #     node.send_movement_command('green')
    #     node.get_logger().info('Exited movement cmd for green ball')

    #     time.sleep(2)
    #     node.get_logger().info('Sending movement command for blue ball')
    #     node.send_movement_command('blue')
    #     node.get_logger().info('Exited movement cmd for blue ball')
        
        

    # Spin the node
    #rclpy.spin(node)

    # Destroy the node explicitly (optional)
    #node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
