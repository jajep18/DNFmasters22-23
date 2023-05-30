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
        self.get_logger().info('Data log KW node has been launched.')

        # Create subscripton to topic /keywords
        self.subscriber = self.create_subscription(Int8MultiArray, '/keywords', self.subscriber_callback, qos_profile=2)

        # Count the amount of files in the ik_datalogs folder (For a unique new filename)
        dir_path = os.path.abspath("./src/data_logging/datalogs/keyword_datalogs")
        glob_path = os.path.join(dir_path, "*.csv")
        n_files = len(glob.glob(glob_path))
        n_files = str(n_files).zfill(4) # Fill with zeros to get a 4 digit number
        
        # Create a unique filename
        rec_filename = "keyword_recording_" + str(n_files) + ".csv"
        self.filename = dir_path + "/" + rec_filename

        # Create a new file with the unique filename
        self.file = open(self.filename, "w")
        self.file.write("keywords (variable len)\n")
        self.file.close()
 
    def subscriber_callback(self, msg):
        '''
        Input msg: keywords in a int8multiarray
        Output: Writes keywords (ints) to a csv file.
        '''
        # Open the file in append mode
        self.file = open(self.filename, "a")

        # print each element of the msg.data array to the file, comma seperated
        for i in range(len(msg.data)):
            self.file.write(str(msg.data[i]))
            if i != len(msg.data)-1:
                self.file.write(",")
        self.file.write("\n")

        # Close the file
        self.file.close()

    # # The desctructor, when the object is destroyed:
    # def __del__(self):
    #     self.file.close()
    #     self.get_logger().info('Data log Keyword node has been destroyed.')

def main(args=None):
    # Initialize the node
    rclpy.init(args=args)
    
    # Create the node
    node = DataLogKeywords()

    # Spin the node
    rclpy.spin(node)

    # Destroy the node (explicitly)
    #node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    