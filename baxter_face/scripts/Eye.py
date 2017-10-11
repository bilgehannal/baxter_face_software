#!/usr/bin/env python

'''
@Author: Bilgehan NAL
This file has a class which defines the shape and movment of the baxter's eyes.
'''

from PIL import Image
import math

class Eye:
    
    def __init__(self):

        self.eyes = Image.open("data/baxter_eye.png")   # eye image
        self.positionX = 0  # keeps the x position as a coordinate
        self.positionY = 0  # keeps the y position as a coordinate

    '''Eye movement functions'''
        
    #  This function set the new position of the eyes (in the radius)
    def lookExactCoordinate(self, x, y):
        #print "Look: ", x, "  -  ",  y
        if self.dotPosition(x, y) <= 1:
            self.setPositionX(x)
            self.setPositionY(y)
        else: 
            rate = self.scaleRateOfRadius(x, y, self.calculateRadiusOfCircleFromGivenPoints(90, 120, self.calculateAngle(x,y)))
            self.setPositionX(int(x*rate))
            self.setPositionY(int(y*rate))
    
    #  This function calculates the position of the eye in the instant frame
    def lookWithMotionCalculation(self, x, y, destinationX, destinationY, totalTime, instantTime):
    
        if totalTime > instantTime:
            diffirenceOfX = destinationX - x
            newPositionDiffirenceX = float(diffirenceOfX)/totalTime * instantTime
            newPositionX = int(x + newPositionDiffirenceX)

            diffirenceOfY = destinationY - y
            newPositionDiffirenceY = float(diffirenceOfY)/totalTime * instantTime
            newPositionY = int(y + newPositionDiffirenceY)
            self.lookExactCoordinate(newPositionX, newPositionY)
            return True
        else: 
            self.lookExactCoordinate(destinationX, destinationY)
            return False


    ''' Ellipse Calculations'''

    def calculateRadiusOfCircleFromGivenPoints(self, a, b,theta):
        firstPart = float(a) * float(b)
        secondPart = (a**2) * (math.sin(theta)**2) + (b**2) * (math.cos(theta)**2)
        secondPart = math.sqrt(secondPart)
        return firstPart / secondPart

    def calculateAngle(self, x, y):
        if x != 0:
            return math.atan(float(y)/float(x))
        else: 
            return math.atan(float(y)/float(x+0.00001))

    '''This function returns the result of ellipse formula according to x and y positions.
    If the result between [0, 1]. Eyes are in the eyes space'''
    
    def dotPosition(self, x, y):
        radiusX = 80
        radiusY = 120
        return (float(x**2) / radiusX**2) + (float(y**2) / radiusY**2)  
        # Formula of an ellipse

    # This function calculates the rate of scaling
    def scaleRateOfRadius(self, x, y, idealRadius): 
        radius = math.sqrt(x**2 + y**2)
        rate = idealRadius / radius
        return rate
    


    # Encapsulation
    
    def getEyes(self):
        return self.eyes
    
    def setPositionX(self, positionX):
        self.positionX = positionX

    def getPositionX(self):
        return self.positionX

    def setPositionY(self, positionY):
        self.positionY = positionY

    def getPositionY(self):
        return self.positionY

   


