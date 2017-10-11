#!/usr/bin/env python

"""
@Author: Bilgehan NAL
This Wobbler class is for the moving of the had joint.
"""


import argparse
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION


class Wobbler(object):

    def __init__(self):
        """
        'Wobbles' the head
        """
        self._done = False
        self._head = baxter_interface.Head()
        self.tolerance = baxter_interface.HEAD_PAN_ANGLE_TOLERANCE
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        print "Wobbler is initilized"

    """ enable robot """

    def enable(self):
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        if not self._rs.state().enabled:
            print("Enabling robot... ")
            self._rs.enable()
    
    def disable(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
       
        print("\nExiting example...")
        if self._done:
            self.set_neutral()
        if self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def set_neutral(self):
        # Sets the head back into a neutral pose
        self._head.set_pan(0.0)

    def wobbleSlow(self, angle) :
        # Sets the head place to a gaven position with a given speed
        print baxter_interface.HEAD_PAN_ANGLE_TOLERANCE
        if angle > 1.5 :
            angle = 1.5
        elif angle < -1.5 :
            angle = -1.5
       
        currentAngle = currentAngle = self.getPosition()
        sign = 0
        
        if angle > currentAngle:
            sign = 1
        else:
            sign = -1

        control_rate = rospy.Rate(100)
        
        while not abs(angle - currentAngle) <= 0.1:
            currentAngle = currentAngle + sign*0.07
            #print "calculated angle: {}".format(currentAngle)
            self._head.set_pan(currentAngle, speed=0.3, timeout=0)
            #self._head.set_pan(currentAngle)
            control_rate.sleep()
            currentAngle = self.getPosition()
           # print "current: {}, sign: {}".format(currentAngle, sign)     
        
        

    def wobble(self, angle) :
        # Sets the head place to a gaven position
        if angle > 1.5 :
            angle = 1.5
        elif angle < -1.5 :
            angle = -1.5
        if self._rs.state().enabled:
            print "Wobbling is apllicating"
            self._head.set_pan(angle)

    def getPosition(self):
        # get the angle of the baxter's head's in the current time.
        return self._head.pan()

    def isEnable(self):
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        return self._rs.state().enabled
        
