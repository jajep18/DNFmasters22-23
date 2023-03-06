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
#import adafruit_mpu6050 # Sensor specific

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
        #self.timer = self.create_timer(timer_period, self.timer_callback) # Create timer for callback
        self.timer = self.create_timer(timer_period, self.dummy_callback) # Create timer for callback

    def dummy_callback(self):
        # Dummy callback function for testing
        msg = String()
        msg.data = "Move red ball right"
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        
    def timer_callback(self):
        msg = Float32MultiArray() # Sensor specific
        msg.data = [self.mpu.acceleration] # Sensor specific
        self.publisher_.publish(msg) # Sensor specific
        self.get_logger().info('Publishing: "%s"' % msg.data) # Sensor specific

def main(args=None):
    #Always: Init node, create node, spin node, destroy node, shutdown node
    rclpy.init(args=args)
    mic_sensor_node = MicSensorNode()
    rclpy.spin(mic_sensor_node)
    mic_sensor_node.destroy_node()
    rclpy.shutdown()