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


class DataLogKeywords(Node):
    def __init__(self):
        super().__init__('datalog_keywords_node')
        # Create subscripton to topic /keywords
        self.subscriber = self.create_subscription(Int8MultiArray, '/keywords', self.subscriber_callback, qos_profile=2)
        self.get_logger().info('Data log KW node has been launched.')
        

        # Count the amount of files in the ik_datalogs folder (For a unique new filename)
        dir_path = os.path.abspath("./src/data_logging/datalogs/keyword_datalogs")
        glob_path = os.path.join(dir_path, "*.csv")
        n_files = len(glob.glob(glob_path))
        n_files = str(n_files).zfill(4) # Fill with zeros to get a 4 digit number
        
        # Create a unique filename
        rec_filename = "keyword_recording_" + str(n_files) + ".csv"
        filename = dir_path + "/" + rec_filename

        # Create a new file with the unique filename
        self.file = open(filename, "w")
 
    def subscriber_callback(self, msg):
        '''
        Input msg: keywords in a int8multiarray
        Output: Writes keywords (ints) to a csv file.
        '''
        # print each element of the msg.data array to the file, comma seperated
        for i in range(len(msg.data)):
            self.file.write(str(msg.data[i]))
            if i != len(msg.data)-1:
                self.file.write(",")
        self.file.write("\n")


    # The desctructor, when the object is destroyed:
    def __del__(self):
        self.file.close()
        self.get_logger().info('Data log Keyword node has been destroyed.')

def main(args=None):
    rclpy.init(args=args)                   # Initialize the node
    datalog_kw_node = DataLogKeywords()     # Create the node
    rclpy.spin(datalog_kw_node)             # Spin the node
    datalog_kw_node.destroy_node()          # Destroy the node (explicitly)
    rclpy.shutdown()                        # Shutdown rclpy
    