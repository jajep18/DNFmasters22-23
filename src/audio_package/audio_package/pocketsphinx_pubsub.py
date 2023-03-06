#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from pocketsphinx import Decoder
import argparse
import wave

import random

from vocabulary import Words

from std_msgs.msg import Int8MultiArray

class Pocketsphinx_Pubsub(Node):

    def __init__(self):
        super().__init__('pocketsphinx_node')
        self.publisher = self.create_publisher(Int8MultiArray, 'keywords', 10)
        self.publisher_explicit = self.create_publisher(String, 'keywords_explicit', 10)

        self.subscriber = self.create_subscription(String, 'audio_file', self.subscriber_callback, 1)
        timer_period = 3  # seconds
        self.timer_pub      = self.create_timer(timer_period, self.kwspotting_callback)
        self.timer_pub_expl = self.create_timer(timer_period, self.kwspotting_callback_explicit)
        #self.timer_sub      = self.create_timer(timer_period_sub, self.subscriber_callback)

        self.audio_data = ""
        self.words = [Words['ERROR'].value]
        

    def kwspotting_callback(self):
        kw_msg = Int8MultiArray()
        #print self.words for debugging
        #self.get_logger().info('Words: "%s"' % self.words)
        for i in range(len(self.words)):
            kw_msg.data.append(self.words[i] )
        self.publisher.publish(kw_msg)
        self.get_logger().info('Publishing: "%s"' % kw_msg.data)

    def kwspotting_callback_explicit(self):
        msg = String()
        msg.data = self.audio_data
        self.publisher_explicit.publish(msg)


    # This function is currently a dummy function. It needs to take actual audio, and perform the keyword spotting.    
    def subscriber_callback(self, msg):
        '''
        Input msg: audio file as a String
        Output: Split string into words
        '''
        if msg is None:
            return
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.audio_data = msg.data # Save audio data to a string with everything
        split_msg = msg.data.split(" ") # Split string into words by spacing
        words = [] # Empty list
        for word in split_msg:
            self.get_logger().info('Word: "%s"' % word)
            words.append(self.check_word_in_dictionary(word.upper())) # Check if word is in dictionary
        self.words = words # Save words to a list

        
    def check_word_in_dictionary(self, word):
        '''
        Input: Word to check
        Output: True if word is in dictionary, False if not
        '''
        #if (word) in Words enum class:
        try:
            Words[word.upper()]
        except KeyError:
            self.get_logger().info('Word NOT in dictionary: "%s"' % word)
            tmp = "ERROR"
            detected_word = Words[tmp.upper()].value
            return detected_word
        
        self.get_logger().info('Word IS in dictionary: "%s"' % word)
        detected_word = Words[word.upper()].value # Convert to upper case and get value
        return detected_word

        # if Words.has_member_key(word):
        #     self.get_logger().info('Word IS in dictionary: "%s"' % word)
        #     detected_word = Words(word.upper()).value # Convert to upper case and get value
        #     return detected_word
        # else:
        #     self.get_logger().info('Word NOT in dictionary: "%s"' % word)
        #     return Words("ERROR").value # Return error if word is not in dictionary



def main(args=None):
    rclpy.init(args=args)

    pocketsphinx_node = Pocketsphinx_Pubsub()
    rclpy.spin(pocketsphinx_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pocketsphinx_node.destroy_node() #ZOOM gidder ikke starte for mig----------------------------------------
    rclpy.shutdown()


if __name__ == '__main__':
    main()