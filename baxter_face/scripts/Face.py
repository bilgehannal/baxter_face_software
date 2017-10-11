#!/usr/bin/env python

'''
@Author: Bilgehan NAL
This file has a class which defines the face of baxter
Face class has other part of the face objects.
'''

'''
Baxter Face Descriptions:

Skin, Mouth and Eyebrow has multiple shapes.
Skin has 5 -> [0, 5]
Mouth has 7 -> [0, 6]
Eyebrow has 5 -> [0, 4]

Skin :: 1->5 : yellow skin -> red skin
        -> 6 is sleeping skin
Mouth ::
    0 -> angry mouth
    1 -> boring mouth
    2 -> confused mouth
    3 -> sad mouth
    4 -> smiling tooth mouth
    5 -> smiling mouth
    6 -> smiling open mouth
Eyebrow ::
    0 -> normal eyebrow
    1 -> ( ): eyebrow
    2 -> One eyebrow in the high
    3 -> angry eyebrow
    4 -> Kucuk Emrah Eyebrow

Coordinate range for x: [-80, 80]
Coordinate range for y: [-120, 120]

'''

from PIL import Image
import Skin
import Mouth
import Eyebrow
import Eye
import Eyelid
from numpy import array
import timeit
import cv2
import os
import random
import getpass
import time


class Face:

    def __init__(self):
        # determine the path and set the default path place
        os.chdir(r'/home/{}/ros_ws/src/baxter_face/scripts'.format(getpass.getuser()))
        ''' Parts of the face of baxter are defined.'''
        self.backgroundImage = Image.open("data/baxter_background.png") # Background behind the eyes
        # Face partions objects
        self.skin = Skin.Skin(5) # range: [0, 5]
        self.mouth = Mouth.Mouth(2) # range: [0, 6]
        self.eyebrow = Eyebrow.Eyebrow(1) # range: [0, 3]
        self.eye = Eye.Eye()
        self.eyelid = Eyelid.Eyelid()
        self.eyelid.setPosition(-330)
        self.eyesCoordinateX = self.eye.getPositionX()
        self.angleOfView = 0.25
    # buildFace function is combining the all face parts together.

    def buildFace(self):
        # Merging the layers
        faceImage = self.backgroundImage.copy()
        faceImage.paste(self.eye.getEyes(), (int(self.eye.getPositionX()), int(self.eye.getPositionY())), self.eye.getEyes())
        faceImage.paste(self.eyelid.getEyelid(), (0, self.eyelid.getPosition()), self.eyelid.getEyelid())
        faceImage.paste(self.skin.getSkin(), (0, 0), self.skin.getSkin())
        faceImage.paste(self.mouth.getMouth(), (0, 0), self.mouth.getMouth())
        faceImage.paste(self.eyebrow.getEyebrow(), (0, 0), self.eyebrow.getEyebrow())
        image = array(faceImage)
        return image

    def show(self, publish):
        image = self.buildFace()
        publish(image)

    # Reposition of the eyes of the baxter
    # This function provide with the eyes' simulation movement
    def lookWithMotion(self, cv2, destinationX, destinationY, time, publish):
        """
        Look with motion is a looking style with an animation
        Animation is generated like this:
            Eyes go to the given coordinates in a fiven time. in a loop
        """
        startTime = timeit.default_timer()
        currentTime = timeit.default_timer()
        x = self.eye.getPositionX()
        y = self.eye.getPositionY()

        while(self.eye.lookWithMotionCalculation(x, y, destinationX, destinationY, time, currentTime-startTime)):
            image = self.buildFace()
            publish(image) # this part is for the baxter's face
            currentTime = timeit.default_timer()

    """ 
    Dynamic looking functions are recalculates the x value according to the head of the Baxter's position
    if the goal coordinate is not in the angle of view of Baxter. -> Wobbling the head joint
    """

    def lookWithMotionDynamic(self, cv2, destinationX, destinationY, time, publish, wobbler):
        # if it is not initilized don't applicate the function
        if wobbler != None: 
            # taking head position as a coordinate
            headPositionRadian = wobbler.getPosition()
            headPositionCoordinate = self.radianToCoordinate(headPositionRadian)
            # control for goal coordinate is not in the angle of view of Baxter
            if abs(destinationX - headPositionCoordinate) > self.radianToCoordinate(self.angleOfView):
                # wobbling -> look at the given coordinates physicly
                print "Wobbling to: ", destinationX
                wobbler.wobble(self.coordinateToRadian(destinationX))
                self.eye.lookExactCoordinate(0, destinationY)
                image = self.buildFace()
                publish(image)
            else:
                # Normal looking with eyes with an animation
                destinationX = destinationX - headPositionCoordinate
                self.lookWithMotion(cv2, destinationX, destinationY, time, publish)

    def lookExactCoordinateDynamic(self, destinationX, destinationY, publish, wobbler):
        # Looking the given coordinate according to the position of the head.
        if wobbler != None: 
            # taking head position as a coordinate
            headPositionRadian = wobbler.getPosition()
            headPositionCoordinate = self.radianToCoordinate(headPositionRadian)
            # control for goal coordinate is not in the angle of view of Baxter
            if abs(destinationX - headPositionCoordinate) > self.radianToCoordinate(self.angleOfView):
                # wobbling -> look at the given coordinates physicly
                print "Wobbling to: ", destinationX
                wobbler.wobble(self.coordinateToRadian(destinationX))
                self.eye.lookExactCoordinate(0, destinationY)  
            else:
                # Normal looking with eyes with an animation
                destinationX = destinationX - headPositionCoordinate
                self.eye.lookExactCoordinate(destinationX, destinationY)
            image = self.buildFace()
            publish(image)

    
    """
    Winkmove functions sets the position of the eyelid with an animation.
    """

    def winkMove(self, cv2, destinationPosition, time, publish):
        # Animation initial values
        startTime = timeit.default_timer()
        currentTime = timeit.default_timer()
        position = self.eyelid.getPosition()
        # Animation part
        while(self.eyelid.moveCalculation(position, destinationPosition, time, currentTime-startTime)):
            image = self.buildFace()
            publish(image)
            currentTime = timeit.default_timer()
            
    def wink(self, cv2, publish):
        firstPosition = self.eyelid.getPosition()
        self.winkMove(cv2, 0, 0.3, publish)
        self.winkMove(cv2, firstPosition, 0.2, publish)
        self.eyelid.setPosition(firstPosition)
        self.show(publish)

    # Encapsulation

    def getSkin(self):
        return self.skin

    def getMouth(self):
        return self.mouth

    def getEyebrow(self):
        return self.eyebrow

    def getEye(self):
        return self.eye

    def getEyelid(self):
        return self.eyelid

    def getBackgroundImage(self):
        return self.backgroundImage


    # Emotions

    def showEmotion(self, mouthIndex, eyebrowIndex, cv2, publish):
        self.mouth.setMouth(mouthIndex)
        self.eyebrow.setEyebrow(eyebrowIndex)
        self.show(publish)

    def sleep(self, cv2, publish):
       self.winkMove(cv2, 0, 0.6, publish) # Eyelids are not seen.
       self.skin.setSkin(5) # range: [0, 5]
       self.showEmotion(1, 1, cv2, publish)

    def wakeUp(self, cv2, publish):
        self.winkMove(cv2, -330, 0.8, publish) # Eyelids are not seen.
        self.skin.setSkin(3) # range: [0, 5]
        self.showEmotion(5, 0, cv2, publish)

    def emotion_default(self, cv2, publish):
       self.winkMove(cv2, -330, 0.3, publish) # Eyelids are not seen.
       self.skin.setSkin(2)
       self.showEmotion(5, 0, cv2, publish)
    
    def emotion_happy(self, cv2, publish):
        mouthArray = [4, 6]
        eyeBrowArray = [0, 1]
        self.winkMove(cv2, -330, 0.3, publish) # Eyelids are not seen.
        self.skin.setSkin(2)
        mouthIndex = random.choice(mouthArray)
        eyebrowIndex = random.choice(eyeBrowArray)
        self.showEmotion(mouthIndex, eyebrowIndex, cv2, publish)
        
    def emotion_angry(self, cv2, publish):
        mouthArray = [0, 3]
        eyeBrowArray = [2, 3]
        self.winkMove(cv2, -330, 0.3, publish) # Eyelids are not seen.
        self.skin.setSkin(4)
        mouthIndex = random.choice(mouthArray)
        eyebrowIndex = random.choice(eyeBrowArray)
        self.showEmotion(mouthIndex, eyebrowIndex, cv2, publish)

    def emotion_confused(self, cv2, publish):
        mouthArray = [2]
        eyeBrowArray = [0, 1]
        self.winkMove(cv2, -330, 0.3, publish) # Eyelids are not seen.
        self.skin.setSkin(3)
        mouthIndex = random.choice(mouthArray)
        eyebrowIndex = random.choice(eyeBrowArray)
        self.showEmotion(mouthIndex, eyebrowIndex, cv2, publish)

    def emotion_sad(self, cv2, publish):
        mouthArray = [1, 3]
        eyeBrowArray = [4]
        self.winkMove(cv2, -330, 0.3, publish) # Eyelids are not seen.
        self.skin.setSkin(1)
        mouthIndex = random.choice(mouthArray)
        eyebrowIndex = random.choice(eyeBrowArray)
        self.showEmotion(mouthIndex, eyebrowIndex, cv2, publish)

    def emotion_panic(self, cv2, publish):
        mouthArray = [2]
        eyeBrowArray = [1]
        self.winkMove(cv2, -330, 0.3, publish) # Eyelids are not seen.
        self.skin.setSkin(0)
        mouthIndex = random.choice(mouthArray)
        eyebrowIndex = random.choice(eyeBrowArray)
        self.showEmotion(mouthIndex, eyebrowIndex, cv2, publish)

    def emotion_bored(self, cv2, publish):
        mouthArray = [1]
        eyeBrowArray = [0, 2, 3]
        self.winkMove(cv2, -150, 0.3, publish) # Eyelids are in the middle of the eyes.
        self.skin.setSkin(2)
        mouthIndex = random.choice(mouthArray)
        eyebrowIndex = random.choice(eyeBrowArray)
        self.showEmotion(mouthIndex, eyebrowIndex, cv2, publish)

    def emotion_crafty(self, cv2, publish):
        mouthArray = [4, 6]
        eyeBrowArray = [2, 3]
        self.winkMove(cv2, -330, 0.3, publish) # Eyelids are not seen.
        self.skin.setSkin(3)
        mouthIndex = random.choice(mouthArray)
        eyebrowIndex = random.choice(eyeBrowArray)
        self.showEmotion(mouthIndex, eyebrowIndex, cv2, publish)

    def testAllImages(self, cv2, publish):
        for index in range(6):
            self.skin.setSkin(index)
            self.show(publish)
        for index in range(7):
            self.showEmotion(index, 0, cv2, publish)
            time.sleep(0.1)
        for index in range(5):
            self.showEmotion(1, index, cv2, publish)
            time.sleep(0.1)


    """ Head Joint move calculations """

    def coordinateToRadian(self, theta) :
        return (3 * theta) / 160.0

    def radianToCoordinate(self, coordinate) :
        return (160 * coordinate) / 3

