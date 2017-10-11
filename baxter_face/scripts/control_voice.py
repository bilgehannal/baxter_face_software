#!/usr/bin/env python

'''
@Author: Bilgehan NAL
This file is a publisher node. 
This node listens the people and provides with to control with voice.
Google speech recognition api is used to reconite the voice.
'''

import sys
import os
import getpass
import termios
import contextlib
import rospy
from std_msgs.msg import String
import Voice_Recogniser

# This function is used for key listener.
@contextlib.contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

def main():
    rospy.init_node('voice_publisher', anonymous=True)
    pub = rospy.Publisher('display_chatter', String, queue_size=40) #Display chatter publisher defined.
    # Recogniser initilazition: parameter is path of txt file of commands
    recogniser = Voice_Recogniser.Voice_Recogniser("/home/{}/ros_ws/src/baxter_face/scripts/data/voice_command.txt".format(getpass.getuser()))
    print 'exit with ^C or ESC'
    # Detecting the push any key on keyboard
    with raw_mode(sys.stdin):
        try:
            while True:
                ch = sys.stdin.read(1)
                if not ch or ch == chr(4) or ord(ch) == 27: # Closing program control
                    break
                if ord(ch) == 32: #space key detection
                    print("Baxter is ready to listen you")
                    command = recogniser.process(recogniser.listen_language(recogniser.TURKEY)) #command
                    print ("applicated command: {}".format(command))
                    pub.publish(command) #publishing the command
        except (KeyboardInterrupt, EOFError):
            pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass