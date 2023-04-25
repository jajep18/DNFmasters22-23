#!/usr/bin/env python3
# Sensor specific publisher node. 
#   Publishes the data from the microphone sensor. 
#   Currently set up with an example from a accelerometer sensor.
#   https://docs.edgeimpulse.com/experts/machine-learning-prototype-projects/ros2-part1-pubsub-node
#   The sensor specific code is marked with "Sensor specific" comments.
#   The code marked "Always" is the same for all sensor nodes.
#   The code marked "Board specific" is specific to the board used.

# Imports
#import board # Sensor specific
import rclpy # Always
from rclpy.node import Node # Always
from std_msgs.msg import Float32MultiArray # Sensor specific
from std_msgs.msg import String 

import os
import glob
import wave
#import adafruit_mpu6050 # Sensor specific

# Include the system library: import sys
import sys

import pyaudio

# Node class
class MicSensorNode(Node):
    def __init__(self):
        super().__init__('mic_sensor_node')
        #self.i2c = board.I2C() # Board specific
        #self.mpu = adafruit_mpu6050.MPU6050(self.i2c) # MPU: This is the accelerometer sensor.
        #self.publisher_ = self.create_publisher(Float32MultiArray, 'mic_sensor', 10) # Sensor specific

        self.publisher = self.create_publisher(String, 'audio_file', 1)

        self.frequency = 0.2 # Freq. pr. second; See "Impulse design" block in EI project
        timer_period = 1 / self.frequency # Calculate period

        # self.timer = self.create_timer(timer_period, self.dummy_callback) # Create timer for callback
        self.timer = self.create_timer(timer_period, self.save_mic_rec) # Create timer for callback

    def dummy_callback(self):
        # Dummy callback function for testing
        msg = String()
        # V- Name of audio file in audio_files folder for PocketSphinx
        # msg.data = "synthetic_movetheredballright" # This one works!    
        # msg.data = "synthetic_movethegreenballleft" # This one works!
        # msg.data = "synthetic_movethegreenballright" # This one works!
        # msg.data = "synthetic_movetheblueballright" # This one works!
        # msg.data = "synthetic_movetheblueballleft" # This one works!
        # msg.data = "synthetic_movetheredballleft" # This one works!
        msg.data = "synthetic_data/synthetic_movetheredballup"
        self.publisher.publish(msg)
        self.get_logger().info('Publishing the filename: "%s"' % msg.data)
        
    def save_mic_rec(self):
        self.get_logger().info("Starting recording call...")
        form_1 = pyaudio.paInt16
        chans = 1 # 1 channel for mono
        samp_rate = 48000 # 48kHz sampling rate
        chunk = 1024 # 2^12 samples for buffer
        record_secs = 5 # seconds to record
        dev_index = 2 # device index found by p.get_device_info_by_index(ii)

        # devnull = os.open(os.devnull, os.O_WRONLY)
        # old_stderr = os.dup(2)
        # sys.stderr.flush()
        # os.dup2(devnull, 2)
        # os.close(devnull)
        # self.get_logger().info("Finished redirecting stderr to devnull. Output is now suppressed.")
        # self.get_logger().info("Ran devnull & flush test...")

        try:
            audio = pyaudio.PyAudio() # create pyaudio instantiation
            # create pyaudio stream
            stream = audio.open(format = form_1,rate = samp_rate,channels = chans, \
                                input_device_index = dev_index,input = True, \
                                frames_per_buffer=chunk)
        except:
            self.get_logger().info("Error opening audio stream")
            return
        self.get_logger().info("Recording...")
        frames = []

        # loop through stream and append audio chunks to frame array
        for ii in range(0,int((samp_rate/chunk)*record_secs)):
            data = stream.read(chunk)
            frames.append(data)
        self.get_logger().info("Finished recording")

        # stop the stream, close it, and terminate the pyaudio instantiation
        stream.stop_stream()
        stream.close()
        audio.terminate()
        
        # Count the amount of files in the audio_files folder (For a unique new filename)
        dir_path = os.path.abspath("./src/audio_package/audio_files")
        glob_path = os.path.join(dir_path, "*.wav")
        n_files = len(glob.glob(glob_path))
        n_files = str(n_files).zfill(4) # Fill with zeros to get a 4 digit number
        
        # Create a unique filename
        filename = "audio_recording_" + str(n_files) + ".wav"
        wav_output_filename = dir_path + "/" + filename
        # Save the audio frames as .wav file
        wavefile = wave.open(wav_output_filename,'w')
        
        wavefile.setnchannels(chans)
        wavefile.setsampwidth(audio.get_sample_size(form_1))
        wavefile.setframerate(samp_rate)
        wavefile.writeframes(b''.join(frames))
        wavefile.close()

        # Publish the filename
        msg = String()
        msg.data = "audio_recording_" + str(n_files)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing the filename: "%s"' % msg.data)
        

def main(args=None):
    #Always: Init node, create node, spin node, destroy node, shutdown node
    rclpy.init(args=args)
    mic_sensor_node = MicSensorNode()
    rclpy.spin(mic_sensor_node)
    mic_sensor_node.destroy_node()
    rclpy.shutdown()