#!/usr/bin/env python3
'''
IK Datalogging node. Created on 01/02/2023
This is a data logging node. It listens to strings published to the topic /datalog_IK and writes them to a file, 1 row per string.
'''
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray
import os
import glob

import numpy as np
import rclpy

from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import ModelStates

from custom_msgs.msg import TriangulatedCircleInfo, TriangulatedCircleInfoArr


class DataLogTriangulation(Node):
    def __init__(self):
        super().__init__('datalog_triangulation_node')

        # Create subscripton to topic /ktriangulated_circles
        self.subscriber = self.create_subscription(TriangulatedCircleInfoArr, '/triangulated_circles', self.subscriber_callback, qos_profile=2)
        self.get_logger().info('Data log Tri. node has been launched.')
        
        # Count the amount of files in the ik_datalogs folder (For a unique new filename)
        dir_path = os.path.abspath("./src/data_logging/datalogs/triangulation_datalogs")
        glob_path = os.path.join(dir_path, "*.csv")
        n_files = len(glob.glob(glob_path))
        n_files = str(n_files).zfill(4) # Fill with zeros to get a 4 digit number
        # Create a unique filename
        rec_filename  = "triangulation_recording_" + str(n_files) + ".csv"
        self.filename = dir_path + "/" + rec_filename
        # Create a new file with the unique filename
        self.file = open(self.filename, "w")
        self.file.write("x,y,color,bgr_mean(1),bgr_mean(2),bgr_mean(3),bgr_var(1),bgr_var(2),bgr_var(3),gt_x,gt_y\n")
        self.file.close()
        # Last received message
        self.last_msg = None


        # Get the position of the red ball
        # ros2 service call /gazebo_msgs/set_entity_state gazebo_msgs/srv/SetEntityState "state: {name: red_ball, pose: {position:{x: 1.0,y: 0.0,z: 0.0}}, reference_frame: world}"

        # Create a suscriber to the topic /gazebo_msgs/model_states
        self.subscriber_model_states = self.create_subscription(ModelStates, '/gazebo_msgs/model_states', self.subscriber_callback_states, qos_profile=2)

        self.ball_position = None

        # Set the position of the red ball
        # ros2 service call /gazebo_msgs/set_entity_state gazebo_msgs/srv/SetEntityState "state: {name: red_ball, pose: {position:{x: 1.0,y: 0.0,z: 0.0}}, reference_frame: world}"
        self.min_x = -0.33
        self.max_x = 0.33
        self.min_y = -0.33
        self.max_y = -0.09
        self.step = 0.05 # try 6
        self.service_client = self.create_client(SetEntityState, '/gazebo_msgs/set_entity_state')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        
        # How to move the ball
        self.req = SetEntityState.Request()
        self.req.state.name = "red_ball"
        # self.req.state.pose.position.x = 1.0
        # self.req.state.pose.position.y = 0.0
        # self.req.state.pose.position.z = 0.0
        self.req.state.reference_frame = "world"
        # self.future = self.service_client.call_async(self.req)
        # self.future.add_done_callback(self.service_callback)

        # Create a list of ball positions to cycle through
        self.ball_positions = []
        for x in np.arange(self.min_x, self.max_x, self.step):
            for y in np.arange(self.min_y, self.max_y, self.step):
                self.ball_positions.append([x,y])
        self.ball_position_index = 0
        
        self.ball_pos_wait_count = 0
        # # Move the ball to the first position
        self.send_ball_to_position(self.ball_positions[self.ball_position_index][0], self.ball_positions[self.ball_position_index][1])
        self.ball_position_index += 1
        self.get_logger().info('Ball is moved to first position.')

    def send_ball_to_position(self, x, y):
        self.req.state.pose.position.x = x
        self.req.state.pose.position.y = y
        self.req.state.pose.position.z = 0.035
        self.future = self.service_client.call_async(self.req)
        # self.future.add_done_callback(self.service_callback)


    def subscriber_callback_states(self, msg):
        '''
        Input msg: ModelStates
        Output: Saves the position of the red ball as a private variable
        '''
        # Get the position of the red ball
        for i in range(len(msg.name)):
            if msg.name[i] == "red_ball":
                self.ball_position = msg.pose[i].position
                #self.get_logger().info('Ball position is: ' + str(self.ball_position))
                return

 
    def subscriber_callback(self, msg):
        '''
        Input msg: TriangulatedCircleInfoArray
        Output: Writes triangulated circles to a csv file.
        '''
        # If the last message is the same as the current message, don't write it to the file
        # if self.last_msg == msg:
        #     return
        
        if self.ball_position == None:
            self.get_logger().info('Ball position is None')
            return
        
        # if self.last_msg != msg: 
        # Open the file in append mode
        self.file = open(self.filename, "a")
        
        # Print triangulated circles to csv
        for i in range(len(msg.circles)):
            self.file.write(str(msg.circles[i].x))
            self.file.write(",")
            self.file.write(str(msg.circles[i].y))
            self.file.write(",")
            self.file.write(str(msg.circles[i].color))
            self.file.write(",")
            self.file.write(str(msg.circles[i].bgr_mean[0]))
            self.file.write(",")
            self.file.write(str(msg.circles[i].bgr_mean[1]))
            self.file.write(",")
            self.file.write(str(msg.circles[i].bgr_mean[2]))
            self.file.write(",")
            self.file.write(str(msg.circles[i].bgr_var[0]))
            self.file.write(",")
            self.file.write(str(msg.circles[i].bgr_var[1]))
            self.file.write(",")
            self.file.write(str(msg.circles[i].bgr_var[2]))
            self.file.write(",")
            self.file.write(str(self.ball_position.x))
            self.file.write(",")
            self.file.write(str(self.ball_position.y))
            self.file.write("\n")
            
        # # print each element of the msg.data array to the file, comma seperated
        # for i in range(len(msg.data)):
        #     self.file.write(str(msg.data[i]))
        #     if i != len(msg.data)-1:
        #         self.file.write(",")
        # self.file.write("\n")

        # Save the last received message
        self.last_msg = msg

        # Close the file
        self.file.close()

        # Check self.ball_pos_wait_count, if it is 10 or more, switch the ball position to the one marked by index self.ball_position_index
        if self.ball_pos_wait_count >= 10:
            self.ball_pos_wait_count = 0
            self.get_logger().info('Move debug.')
            self.send_ball_to_position(self.ball_positions[self.ball_position_index][0], self.ball_positions[self.ball_position_index][1])
            self.ball_position_index += 1
            self.get_logger().info('Moving the ball.')
        else:
            self.ball_pos_wait_count += 1
            self.get_logger().info('No movement.')


        


    # # The desctructor, when the object is destroyed:
    # def __del__(self):
    #     self.driver.close()
    #     self.get_logger().info('Data log Keyword destruction started.')
    #     self.file.close()
    #     self.get_logger().info('Data log Keyword node has been destroyed.')

def main(args=None):
    # Initialize the node
    rclpy.init(args=args)
    
    # Create the node
    node = DataLogTriangulation()

    # Spin the node
    rclpy.spin(node)

    # Destroy the node (explicitly)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    