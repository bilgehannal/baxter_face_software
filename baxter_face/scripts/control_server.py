#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
@Author: Bilgehan NAL
This python file is a socket server.
This server listens the messages sent to determined port
then, publish the given data to determined topic
'''

import socket
import rospy
from std_msgs.msg import String
import re
import fcntl
import struct
import sys
import getpass
import io

commands = {'---': '---'}


def main():
    port = 8080 # Default Port 
    topic = 'display_chatter' # Default Topic
    topic, port = externelArgumant(topic, port) # if there is any external topic or port, use them

    rospy.init_node('voice_publisher', anonymous=True)
    updateDictionary("/home/{}/ros_ws/src/baxter_face/scripts/data/voice_command.txt".format(getpass.getuser()))
    pub = rospy.Publisher(topic, String, queue_size=40) # Display chatter publisher defined.
    # Socket variables
    TCP_IP = ""
    TCP_PORT = port
    BUFFER_SIZE = 256  # Normally 1024, but we want fast response
    # Communication variables display
    print "ip: {}".format(get_ip_address('eth0'))
    print "port: {}".format(TCP_PORT)
    print "topic: {}".format(topic)
    
    

    # Socket Creation
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    conn, addr = s.accept()
    socket.setdefaulttimeout(1)
    print 'Connection address:', addr
    
    # Listen message
    while 1:
        try:
            data = conn.recv(BUFFER_SIZE) # message given from client stop the server
            data = data.decode('latin-1').encode('utf-8')
            if not data or "exit" in data:  # To stop the server
                print "Program is closing"
                break
            data = data.replace('\0', '')
            for index in data:
                print "String: {}".format(ord(index))
            print u"received data: {}".format(data)

            conn.send("data: {}\n".format(data))  # echo
            if(isSpeech(data)):
                pub.publish(process(getSpeechMessage(data)))
            else:
                pub.publish(data) # Publish to topic
        except socket.timeout:
            print "No data is detected"
    conn.close()

def isSpeech(msg):
    msgs = msg.split("*") #this array keeps all variables
    if len(msgs) > 1:
        return True
    else:
        return False 

def getSpeechMessage(msg):
    msgs = msg.split("*")
    if len(msgs) > 1:
        return msgs[1]
    else:
        return  msgs[0] 



# IP address taken
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

# Externel topic or port information control:
def externelArgumant(topic, port):
    args = sys.argv
    length = len(args)
    '''
    1 argumant -> nothing
    2 argumants -> topic
    3 argumants -> topic and port
    '''
    if length == 1:
        return topic, port
    elif length == 2:
        newPort = int(args[1])
        return topic, newPort
    elif length == 3:
        newTopic = args[2]
        newPort = int(args[1])
        return newTopic, newPort
    return topic, port

def updateDictionary(path):
        #Read data line by line to a list from the txt file.
        with io.open(path, 'r', encoding='utf-8') as file:
            my_list = file.readlines()
        #Seperating waited speech and commands. These two things are seperated by a character of dot(.)
        for row in my_list:
            command = row.encode('utf-8').split(".")
            if len(command) == 2:
                commands[command[0]] = command[1]
                print ("Key: {}, Value: {}".format(command[0], command[1]))

def process(string):
    enc = sys.getdefaultencoding()
    result = "Speech could not be processed" #Default message
    string = string.lower()#All cases are converted to lower case
    # Search the commands in dictionary
    for key in commands.keys():
        # if the key is substring of string -> key is our commands.
        if key in string:
            result = commands[key]
            break
    return result.rstrip().lower().encode('utf_8')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

