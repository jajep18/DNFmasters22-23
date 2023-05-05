'''
IK Datalogging node. Created on 01/02/2023
This is a data logging node. It listens to strings published to the topic /datalog_IK and writes them to a file, 1 row per string.
'''
from rclpy.node import Node
from std_msgs.msg import String
import os
import glob


class DataLogIK(Node):
     def __init__(self):
         super().__init__('datalog_ik_node')
         self.subscriber = self.create_subscription(String, '/datalog_IK', self.subscriber_callback, qos_profile=10)
         self.get_logger().info('Data log IK node has been launched.')
         self.file = open("datalog_ik.txt", "w")
         self.file.write("x,y,z,0,1,2,3,4,5,6\n")
 
     def subscriber_callback(self, msg):
        '''
        Input: String, comma seperated data. 
            Robot TCP position          x,
                                        y,
                                        z, 
            IK solution joint angles    0, 1, 2, 3, 4, 5, 6. 
            Successful movement         1 if successful, 0 if not.
        Output: Writes same string msg to file
        '''
        if msg is None:
            return
        self.file.write(msg.data + "\n")
        self.get_logger().info('Writing to file: "%s"' % msg.data)

        # Count the amount of files in the audio_files folder (For a unique new filename)
        dir_path = os.path.abspath("./src/data_logging/ik_datalogs")
        glob_path = os.path.join(dir_path, "*.wav")
        n_files = len(glob.glob(glob_path))
        n_files = str(n_files).zfill(4) # Fill with zeros to get a 4 digit number
        
        # Create a unique filename
        filename = "IK_recording_" + str(n_files) + ".wav"
        wav_output_filename = dir_path + "/" + filename
    