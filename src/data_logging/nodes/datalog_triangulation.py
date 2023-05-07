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
        self.file.write("x,y,color,bgr_mean(1),bgr_mean(2),bgr_mean(3),bgr_var(1),bgr_var(2),bgr_var(3)\n")
        self.file.close()

        # Last received message
        self.last_msg = None
 
    def subscriber_callback(self, msg):
        '''
        Input msg: TriangulatedCircleInfoArray
        Output: Writes triangulated circles to a csv file.
        '''
        # If the last message is the same as the current message, don't write it to the file
        if self.last_msg == msg:
            return
        
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
            self.file.write("\n")

        # print each element of the msg.data array to the file, comma seperated
        for i in range(len(msg.data)):
            self.file.write(str(msg.data[i]))
            if i != len(msg.data)-1:
                self.file.write(",")
        self.file.write("\n")

        # Save the last received message
        self.last_msg = msg

        # Close the file
        self.file.close()


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
    