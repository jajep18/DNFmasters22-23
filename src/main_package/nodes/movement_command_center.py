#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import transform_pubsub #Our own transform publisher node

import time
import threading
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

# Using this for import guide: https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/#How_we%E2%80%99ll_manage_to_setup_Python_and_Cpp_nodes_in_the_same_ROS2_package
from main_package.movement import *

from enum import Enum

# Custom interfaces:
from jetmax_control.srv import IK
from jetmax_control.srv import FK
from std_srvs.srv import SetBool
from custom_msgs.srv import Decision
from custom_msgs.msg import TriangulatedCircleInfoArr

# Enums (Action and target color)
from main_package.actions import Action
class Color(Enum):
    RED     = 0
    GREEN   = 1
    BLUE    = 2
    NONE    = 3

    def __int__(self):
        return self.value

# This node is responsible for receiving the TCP transform from the camera 
# and reciving the real world pose of the balls from the camera
# Should then send the robot to the ball and pick it up
class MovementCommandCenter(Node):
    def __init__(self):
        super().__init__('movement_command_center')
        # self._tf_buffer = Buffer()
        # self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        # self._subscriber_trans = self.create_subscription(TransformStamped, 'base_TCP_transform', self.get_tcp_transform, 10)

        # Create callback groups
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self._cb_none = None
        self._cb_timer= rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # Create a subscriber to the triangulated circles
        self._subscriber_coords = self.create_subscription(TriangulatedCircleInfoArr, 'triangulated_circles', self.get_coordinates, 10, callback_group=self.callback_group)
        self._transform = None
        self._ball_coordinates =[[None, None, None], [None, None, None], [None, None, None]] # Red, Green, Blue
        # Todo: Above would support 3 or fewer balls, but we should make it support any number of balls

        # Create a client for the IK service
        self._ik_client = self.create_client(IK, '/jetmax_control/inverse_kinematics', callback_group=self.callback_group)
        while not self._ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("IK Service not available, waiting again...")
        self.get_logger().info("IK Service is available")

        # # Create a client for the FK service
        # self._fk_client = self.create_client(FK, '/jetmax_control/forward_kinematics')
        # while not self._fk_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("FK Service not available, waiting again...")
        # self.get_logger().info("FK Service is available")

        # Create a client for the gripper service
        self._gripper_client = self.create_client(SetBool, '/jetmax_switch', callback_group=self.callback_group)
        while not self._gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Gripper Service not available, waiting again...")
        self.get_logger().info("Gripper Service is available")
        
        # Create a client for the decision service
        # self._decision_client = self.create_client(Decision, '/DNF/decision')
        # while not self._decision_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Decision Service not available, waiting again...")
        # self.get_logger().info("Decision Service is available")

        # Create a timer to call for decision making service, and perform the determined action
        timer_period = 10  # seconds
        # Get the clock of the node 
        self._clock = self.get_clock()
        self._timer = self.create_timer(timer_period, self.timer_callback, callback_group=self._cb_timer)
        # self._timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.callback_group, clock=self._clock)

            
        
        # Create requests for the clients
        self._ik_req = IK.Request()
        self._gripper_req = SetBool.Request()
        # self._fk_req = FK.Request()
        self._decision_req = Decision.Request()
        self._reqid_count = 0

        # Create position storing variables
        self.home_position      = np.array([0.120, 0.0, 0.200])
        self.current_position   = np.array([0.120, 0.0, 0.200])
        self.above_offset = 0.1

        # Create gripper variable. True = open (sucking), False = closed
        self.gripper_state = True

        self._count_actions = 0 # 7 actions
        
    def send_decision_request(self):  
        self._decision_req.requestid = self._reqid_count # For debugging purposes
        self.get_logger().info("Sending Decision request with id: %s..." % (self._decision_req.requestid))
        self._reqid_count += 1
        
        self.future = self._decision_client.call_async(self._decision_req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=5)
        if self.future.result() is not None:
            self.get_logger().info("Decision Request sent, Result: %s" % self.future.result().success)
        else:
            self.get_logger().info("Decision Request failed %r" % (self.future.exception(),)) 
            return self.future.exception()
        return self.future.result()
        

    # async def send_ik_request(self, _x: float, _y: float, _z: float):
    def send_ik_request(self, _x: float, _y: float, _z: float):
        scale = 1000
        self._ik_req.x = float(scale * _x)
        self._ik_req.y = float(scale * _y)
        self._ik_req.z = float(scale * _z)
        self.get_logger().info("Sending IK request with x: %.2f, y: %.2f, z: %.2f" % (self._ik_req.x, self._ik_req.y, self._ik_req.z))
        # self.get_logger().info("Sending IK request with x: %s, y: %s, z: %s" % (self._ik_req.x, self._ik_req.y, self._ik_req.z))
        self.future = self._ik_client.call_async(self._ik_req)


        if self.executor is None:
            rclpy.spin_until_future_complete(self, self.future, timeout_sec=2)
        else:
            self.executor.spin_until_future_complete(self.future, timeout_sec=0.5)
        
        # rclpy.spin_until_future_complete(self, self.future, timeout_sec=5)
        #self.executor.spin_until_future_complete(self.future)


        if self.future.result() is not None:
            self.get_logger().info("IK Request sent, Result: %s" % self.future.result().success)
        else:
            self.get_logger().info("IK Request failed %r" % (self.future.exception(),)) 
            return self.future.exception()
        return self.future.result().success
        # return await self._ik_client.call_async(self._ik_req)
    
    def send_gripper_request(self, _open_data: bool):
        self.get_logger().info("Sending Gripper: %s. Gripper is currently in state: %s" % (_open_data, self.gripper_state))
        self._gripper_req.data = _open_data
        self.future = self._gripper_client.call_async(self._gripper_req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=5)
        if self.future.result() is not None:
            self.get_logger().info("Gripper Request sent, Result: %s" % self.future.result().success)
            self.gripper_state = _open_data
        else:
            self.get_logger().info("Gripper Request failed %r" % (self.future.exception(),)) 
            return self.future.exception()
        return self.future.result().success

    def get_tcp_transform(self, msg):
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
                    #self.get_logger().info('Recieved coordinates for red ball')
                elif circles.color == 'green':
                    self._ball_coordinates[1] = [circles.x, circles.y, 0.0]
                    #self.get_logger().info('Recieved coordinates for green ball')
                elif circles.color == 'blue':
                    self._ball_coordinates[2] = [circles.x, circles.y, 0.0]
                    #self.get_logger().info('Recieved coordinates for blue ball')
                else:
                    self.get_logger().info('Recieved coordinates for unknown ball')
        else: 
            self.get_logger().info('No coordinates recieved')
    
    def action_pickup(self, pick_pos):
        '''
        Move to the pick up position and turn on gripper
        @param pick_pos: The position to pick up at in the form [x, y, z]
        '''

        above_pick_pos = [pick_pos[0], pick_pos[1], pick_pos[2] + self.above_offset]
        
        # Move above the pick up position
        self.action_move(above_pick_pos)
        # Move to the pick up position
        self.action_move(pick_pos)
        # Turn on the gripper
        self.action_grasp()
        # Move above the pick up position
        self.action_move(above_pick_pos)
        
        self.get_logger().info('Pick up successful')

    def action_move(self, move_pos):
        '''
        Move to the given position
        @param move_pos: The position to move to in the form [x, y, z]
        '''
        # Check if positions are defined
        if move_pos[0] is None or move_pos[1] is None or move_pos[2] is None:
            self.get_logger().info('No coordinates available')
            return 
        elif self.current_position[0] is None or self.current_position[1] is None or self.current_position[2] is None:
            self.get_logger().info('No current position available')
            return
        
        self.get_logger().info('Moving to (%s, %s, %s)' % (move_pos[0], move_pos[1], move_pos[2]))
        path = linear_interpolation(start_pos=self.current_position, end_pos=move_pos, max_vel= 0.5, sample_rate=200)#= 500)
        
        # Loop through the path and send movement commands
        for positions in path:
            self.send_ik_request(positions[0], positions[1], positions[2])
            time.sleep(0.08)
            
        self.get_logger().info('Move successful')
        time.sleep(3)
        self.current_position = move_pos
        
    def action_placedown(self):
        '''
        Move to the place position and turn off gripper
        Place position should be directly below the current position (todo: is this final?)
        @param place_pose: The position to place down at in the form [x, y, z]
        '''
        above_place_pos = [self.current_position[0], self.current_position[1], self.current_position[2]]
        place_pos = [self.current_position[0], self.current_position[1], self.current_position[2] - self.above_offset]

        self.get_logger().info('Placing down at (%s, %s, %s)' % (place_pos[0], place_pos[1], place_pos[2]))
        # Move to the place position and turn off gripper
        self.action_move(place_pos)
        self.send_gripper_request(False)
        # Move above the place position
        self.action_move(above_place_pos)
        # Move to home position
        self.action_move(self.home_position)
        
    def action_grasp(self):
        '''
        Turn on gripper (start sucking)
        '''
        self.send_gripper_request(True)
        self.get_logger().info('Suction on')
        
    def action_release(self):
        '''
        Turn off gripper (stop sucking)
        '''
        self.send_gripper_request(False)
        self.get_logger().info('Suction off')

    def action_movedir(self, target_pos, place_offset_x = 0.0, place_offset_y = 0.0):
        '''
        Pick up target ball. Move left 5cm. Place down target ball.
        '''
        self.get_logger().info('Moving left')
        pick_pos        = [target_pos[0], target_pos[1], target_pos[2]]
        above_pick_pos  = [target_pos[0], target_pos[1], target_pos[2] + self.above_offset]
        place_pos       = [target_pos[0] + place_offset_x, target_pos[1] + place_offset_y, target_pos[2]]
        above_place_pos = [target_pos[0] + place_offset_x, target_pos[1] + place_offset_y, target_pos[2] + self.above_offset]

        # Move to above pick up position
        self.action_move(above_pick_pos)
        # Move to pick up position
        self.action_move(pick_pos)
        # Turn on gripper
        self.send_gripper_request(True)
        # Move above pick up position
        self.action_move(above_pick_pos)
        # Move left 5cm (Above place position)
        self.action_move(above_place_pos)
        # Move to place position
        self.action_move(place_pos)
        # Turn off gripper
        self.send_gripper_request(False)
        # Move above place position
        self.action_move(above_place_pos)
        # Move to home position
        self.action_move(self.home_position)
        
    
    def perform_action(self, action:Enum, target:Enum):
        '''
        Takes an action as input and adds the action to a path if necessary.
        Target: what the target object for the action is (in color for simplification)
        '''
        try:
            target_pos = self._ball_coordinates[int(target)]
            #self.get_logger().info('Target: %s, Ball coordinates: %s' % (target.name, target_pos))
        except:
            # Print the targets int value, and the length of the ball coordinates
            self.get_logger().info('Exception occured during target_pos')
            self.get_logger().info('Target: %s, Ball coordinates: %s' % (target.name, len(self._ball_coordinates)))
            return

        
        # If target_pos is NoneType, return
        if target_pos[0] is None or target_pos[1] is None or target_pos[2] is None:
            self.get_logger().info('No coordinates available')
            return
        
        # Print which action is going to be performed, along with what the target is
        self.get_logger().info('Performing action: %s on the %s ball.' % (action.name, target.name))  
        # Print the target position
        self.get_logger().info('Target position: (%.2f, %.2f, %.2f)' % (target_pos[0], target_pos[1], target_pos[2]))


        # Switch case for the action (Manually, only using python 3.8.10)
        if action == Action.PICK_UP:
            self.action_pickup(pick_pos=target_pos)
        elif action == Action.PLACE_DOWN:
            self.action_placedown()
        elif action == Action.MOVE_LEFT:
            self.action_movedir(target_pos=target_pos, place_offset_x=-0.05)
        elif action == Action.MOVE_RIGHT:
            self.action_movedir(target_pos=target_pos, place_offset_x= 0.05)
        elif action == Action.MOVE_BACK:
            self.action_movedir(target_pos=target_pos, place_offset_y=-0.05)
        elif action == Action.MOVE_FORWARD:
            self.action_movedir(target_pos=target_pos, place_offset_y= 0.05)
        elif action == Action.GRASP:
            self.action_grasp()
        elif action == Action.RELEASE:
            self.action_release()
        else:
            self.get_logger().info('Invalid action passed')
            return

        self.get_logger().info('Action performed')
        
    def timer_callback(self):
        '''
        Timer callback function. Runs every 10(?) seconds. Has 2 steps:
        - Send a request for decision making.
        - Perform the decided action.
        '''
        self.get_logger().info('- - - - - - - - - - - - - - - - - - - - - - - - ')
        self.get_logger().info('Timer callback function called. Count: %s' % self._count_actions)
        

        # Send request for decision making
        # self.get_logger().info('Sending request for decision making')
        # response = self.send_decision_request()
        # self.get_logger().info('Response action: %s. Response target: %s' % (response.actions, response.targets))
        # action = response.actions
        # target = response.targets
        # action = Action(self._count_actions % 7)
        action = Action.PICK_UP
        target = Color.RED
        
        # Currenty action
        self.get_logger().info('Action: %s, Target: %s' % (action.name, target.name))

        if self._count_actions != 0:
            self.get_logger().info('This is not the first action. Quitting the timer cb')
            self.get_logger().info('- - - - - - - - - - - - - - - - - - - - - - - - ')
            return

        self._count_actions += 1

        # Perform the decided action
        self.perform_action(action, target)

        #self._timer.reset()
        #self.get_logger().info('Is timer active: %s' % self._timer.is_active())

        # self.get_logger().info('Timer stress test commencing (waste time for 10 seconds)')
        # time.sleep(10)
        
        self.get_logger().info('Is timer ready: %s' % self._timer.is_ready())
        self.get_logger().info('Is timer canceled: %s' % self._timer.is_canceled())
        self.get_logger().info('Timer callback function finished. Count: %s' % (self._count_actions-1))
        self.get_logger().info('- - - - - - - - - - - - - - - - - - - - - - - - ')


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    # Create the node
    node = MovementCommandCenter()

    # Move the arm to the home position
    node.get_logger().info("Sending request to move home...")
    position = [0.120, 0.0, 0.200]
    node.get_logger().info("Position: {}".format(position))
    response = node.send_ik_request(position[0], position[1], position[2])
    node.get_logger().info("Response: %s" % response)

    time.sleep(0.5)

    # node.get_logger().info("Sending request to move slightly...")
    # position = [0.120, 0.10, 0.200]
    # node.get_logger().info("Position: {}".format(position))
    # response = node.send_ik_request(position[0], position[1], position[2])
    # node.get_logger().info("Response: %s" % response)

    # Delay
    # time.sleep(2)
    node.get_logger().info("Testing actions - - - - - - - - - - - - - - - - - - - -")

    # Spin / Start the node
    # create a single threaded executor to prevent multiple threads from being created
    
    #my_executor = SingleThreadedExecutor()
    my_executor = MultiThreadedExecutor()
    my_executor.add_node(node)
    my_executor.spin()

    # # Print 10 times
    # for i in range(10):
    #     print("Spin once...")
    #     my_executor.spin_once()
    #     print("Spinned once")

    rclpy.shutdown()
    # try:
    #     my_executor = MultiThreadedExecutor()
    #     my_executor.add_node(node)
        
    #     try:
    #         # my_executor.spin_until_future_complete( future = node.get_clock().now(), node = node.timer_callback(), timeout_sec= rclpy.duration.Duration(seconds=1))
    #         #my_executor.spin_once_until_future_complete(node.get_clock().now(), node.timer_callback(), (node.get_clock().now() + rclpy.duration.Duration(seconds=1)) )
    #         # my_executor.spin()
    #         #rclpy.spin_until_future_complete(node = node, executor=my_executor, future=node.timer_callback(), timeout_sec=1)
    #         #my_executor.spin()
    #         rclpy.spin(node, executor=my_executor)
    #     except KeyboardInterrupt:
    #         pass
    #     finally:
    #         my_executor.shutdown()
    #         node.destroy_node()
    # finally:
    #     rclpy.shutdown()
    
    # rclpy.spin(node)
    # node.perform_action(Action.PICK_UP, Color.RED)



    # Test the motion planning of the arm:
    #node.get_logger().info("Testing movement.py")

    #start_pos = np.array([      0.120,   0.0,      0.2])
    #pickup_target = np.array([  0.0,    -0.18,      0.1])
    #place_target = np.array([   0.1,    -0.18,      0.1])
    #height_offset = 0.03

    #test_path = create_path(start_pos, pickup_target, place_target, height_offset)
    #
    #write_path_to_csv(test_path, "test_path.csv")
    #
    #
    #for i in range(0, len(test_path)):
    #    #node.get_logger().info("Sending request for path: {}".format(test_path[i]))
    #    for j in range(0, len(test_path[i])):
    #        #node.get_logger().info("Sending request to move to position: {}".format(test_path[i][j]))
    #        node.send_ik_request(test_path[i][j][0], test_path[i][j][1], test_path[i][j][2])
    #        time.sleep(0.01)
    #
    # # Spin the node
    # rclpy.spin(node)

    # Spin in a seperate thread
    # thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    # thread.start()

    # Create rate object to "sleep" in a non-blocking manner
    #rate = node.create_rate(0.5, node.get_clock())
    # for i in range(15):
    #     node.get_logger().info('Sending decision request')
    #     action_resp = node.send_decision_request()
    #     node.get_logger().info('Decision request response: %s' % action_resp)
    #     time.sleep(5)
    # node.get_logger().info('Sending decision request')
    # node.send_decision_request()


    # Spin the node
    #rclpy.spin(node)

    # Destroy the node explicitly (optional)
    #node.destroy_node()
    #rclpy.shutdown()
    #thread.join()


if __name__ == "__main__":
    main()
