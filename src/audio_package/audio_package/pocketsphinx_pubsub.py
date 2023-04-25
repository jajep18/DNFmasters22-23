#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from pocketsphinx import Decoder
import argparse
import wave

import random

import os

from vocabulary import Words, Keywords
from pathlib import Path

from std_msgs.msg import Int8MultiArray

class Pocketsphinx_Pubsub(Node):

    def __init__(self):
        super().__init__('pocketsphinx_node')
        self.publisher = self.create_publisher(Int8MultiArray, 'keywords', 1)
        self.publisher_explicit = self.create_publisher(String, 'keywords_explicit', 1)

        self.subscriber = self.create_subscription(String, 'audio_file', self.subscriber_callback, 1)
        timer_period = 3  # seconds
        self.timer_pub      = self.create_timer(timer_period, self.kwspotting_callback)
        self.timer_pub_expl = self.create_timer(timer_period, self.kwspotting_callback_explicit)
        #self.timer_sub      = self.create_timer(timer_period_sub, self.subscriber_callback)

        self.audio_data = ""
        self.words = [Words['ERROR'].value]
        self.audio_recieved = False
        self.audio_recieved_explicit = False
        

    def kwspotting_callback(self):
        if self.audio_recieved == False:
            return
        
        kw_msg = Int8MultiArray()
        #print self.words for debugging
        #self.get_logger().info('Words: "%s"' % self.words)
        for i in range(len(self.words)):
            kw_msg.data.append(self.words[i] )
        self.publisher.publish(kw_msg)
        self.get_logger().info('Publishing: "%s"' % kw_msg.data)

        self.audio_recieved = False

    def kwspotting_callback_explicit(self):
        if self.audio_recieved_explicit == False:
            return
        
        msg = String()
        msg.data = self.audio_data
        self.publisher_explicit.publish(msg)
        self.get_logger().info('Publishing (explicit): "%s"' % msg.data)

        self.audio_recieved_explicit = False


    # This function is currently a dummy function. It needs to take actual audio from the mic node, and perform the keyword spotting.    
    def subscriber_callback(self, msg):
        '''
        Input msg: audio file as a String
        Output: Split string into words
        '''
        if msg is None:
            return
        self.get_logger().info('I heard the filename: "%s"' % msg.data)

        # Get filepath from msg.data ("audio_file") filename
        dir_path = os.path.abspath("./src/audio_package/audio_files")
        filename = dir_path + "/" + msg.data + ".wav"
        # dict_path = os.path.abspath("./src/audio_package/resource/cmudict-en-us.dict")
        dict_path = os.path.abspath("./src/audio_package/resource/cmudict-reduced.dict")
        

        filepath = Path(filename)
        if filepath.is_file():
           # self.get_logger().info('File exists')
        
            # Open audio file
            try:
                with wave.open(filename, 'rb') as audio:
                    # decoder = Decoder(samprate=audio.getframerate())#, dict=None)
                    decoder = Decoder(samprate=audio.getframerate(), dict=None)
                    # decoder.load_dict('cmudict-en-us.dict')
                    decoder.load_dict(dict_path)
                    decoder.start_utt()
                    decoder.process_raw(audio.getfp().read(), full_utt=True)
                    decoder.end_utt()
                    self.get_logger().info('Detected keywords: %s' % decoder.hyp().hypstr)
                    
                    # Convert audio data (string) to list of words
                    self.audio_data = decoder.hyp().hypstr # Save audio data to a string for the explicit publisher (debugging and logging purposes)
                    split_msg = decoder.hyp().hypstr.split(" ") # Split string into words

                    # Check if words are in dictionary
                    words = [] # Empty list
                    for word in split_msg:
                        self.get_logger().info('Word: "%s"' % word)
                        words.append(self.check_word_in_dictionary(word.upper())) # Check if word is in dictionary

                    # Check if words are in keyword dictionary
                    keywords = [] # Empty list
                    for word in split_msg:
                        self.get_logger().info('Word: "%s"' % word)
                        keyword = self.check_word_in_keywords(word.upper()) # Check if word is in keyword dictionary
                        self.get_logger().info('Keyword: "%s" - Error value: "%s"' % (keyword, Keywords['ERROR'].value))
                        if keyword != Keywords['ERROR'].value:
                            keywords.append(keyword)

                    self.words = keywords #words # Save words to a list
                    self.audio_recieved = True
                    self.audio_recieved_explicit = True
            except:
                self.get_logger().info('Could not open audio file')
                self.get_logger().info('File path: "%s"' % filename)
        else: 
            self.get_logger().info('File does not exist')
            self.get_logger().info('File path: "%s"' % filename)



        
    def check_word_in_dictionary(self, word):
        '''
        Input: Word to check
        Output: True if word is in dictionary, False if not

        Dictionary is defined in vocabulary.py (class Words)
        Contains all words that pocketsphinx should detect
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
    
    def check_word_in_keywords(self, word):
        '''
        Input: Word to check
        Output: True if word is in keyword dictionary, False if not

        Keyword dictionary is defined in vocabulary.py (class Keywords)
        Contains all keywords that the robot should react to
        '''
        try:
            Keywords[word.upper()]
        except KeyError:
            self.get_logger().info('Word NOT in keyword dictionary: "%s"' % word)
            tmp = "ERROR"
            detected_word = Keywords[tmp.upper()].value
            return detected_word
        
        self.get_logger().info('Word IS in keyword dictionary: "%s"' % word)
        detected_word = Keywords[word.upper()].value # Convert to upper case and get value
        return detected_word


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