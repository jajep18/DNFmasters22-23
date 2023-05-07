#!/usr/bin/env python3
'''
IK Datalogging node. Created on 01/02/2023
This is a data logging node. It listens to strings published to the topic /datalog_IK and writes them to a file, 1 row per string.
'''
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import os
import glob

import numpy as np
import rclpy


class DataLogIK(Node):
    def __init__(self):
        super().__init__('datalog_ik_node')
        self.subscriber = self.create_subscription(Float32MultiArray, '/datalog_IK', self.subscriber_callback, qos_profile=10)
        self.get_logger().info('Data log IK node has been launched.')
        

        # Count the amount of files in the ik_datalogs folder (For a unique new filename)
        dir_path = os.path.abspath("./src/data_logging/datalogs/ik_datalogs")
        glob_path = os.path.join(dir_path, "*.csv")
        n_files = len(glob.glob(glob_path))
        n_files = str(n_files).zfill(4) # Fill with zeros to get a 4 digit number
        
        # Create a unique filename
        rec_filename = "IK_recording_" + str(n_files) + ".csv"
        filename = dir_path + "/" + rec_filename

        # Create a new file with the unique filename
        self.file = open(filename, "w")
        #self.file = open("datalog_ik.txt", "w")
        self.file.write("x,y,z,0,1,2,0,1,2,3,4,5,6,7,8\n")

 
    def subscriber_callback(self, msg):
        '''
        Input: Float32MultiArray msg with the following data:
            Robot TCP position          x,
                                        y,
                                        z, 
            IK solution active joints   0, 1, 2     (active angles [rotate, angle left, angle right])
            FK solution joint angles    0, 1, 2, 3, 4, 5, 6. 
            Successful movement         1 if successful, 0 or -1 if not (fk or ik failed).
        Output: Writes same string msg to file, csv comma seperated.
        '''
        # print each element of the msg.data array to the file, comma seperated
        for i in range(len(msg.data)):
            self.file.write(str(msg.data[i]))
            if i != len(msg.data)-1:
                self.file.write(",")
        self.file.write("\n")
        self.get_logger().info('IK datalog has been written to file.')

    # The desctructor, when the object is destroyed:
    def __del__(self):
        self.file.close()
        self.get_logger().info('Data log IK node has been destroyed.')

def main(args=None):
    rclpy.init(args=args)           # Initialize the node
    datalog_ik_node = DataLogIK()   # Create the node
    rclpy.spin(datalog_ik_node)     # Spin the node
    datalog_ik_node.destroy_node()  # Destroy the node (explicitly)
    rclpy.shutdown()                # Shutdown rclpy
    