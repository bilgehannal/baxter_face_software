#!/usr/bin/env python

'''
@Author: Bilgehan NAL
This file is a subscriber node which listens a topic type:String
Waited String messages:
Emotions:
    -> "default"
    -> "happy"
    -> "sad"
    -> "angry"
    -> "confused"
    -> "panic"
    -> "bored"
    -> "crafty"
Actions:
    -> "look_<x>_<y>"
    -> "look_<x>_<y>_<time>"
    -> "skin_<number>"
    -> "human_follow_on"
    -> "human_follow_off"
    -> "arm_follow_off"
    -> "left_arm_follow_on"
    -> "right_arm_follow_on"
    -> "dynamic_look_<x>_<y>"
    -> "dynamic_human_follow_on"
    -> "wake_up"
    -> "sleep"
Wobbling:
    -> "enable"
    -> "disable" 
    -> "wobble_<angle>" angle should be between[-1.5, 1.5]
Other:
    -> "exit"
    -> "wake_up"
    -> "sleep"


'''

import os
import sys
import rospy
import timeit
import cv2
import cv_bridge
import Face
from sensor_msgs.msg import Image, PointCloud
from std_msgs.msg import String
import threading
import head_wobbler
from baxter_core_msgs.msg import EndpointState
import math

""" Variable Decleration """

wobbler = None

face = Face.Face()
humanFollowControl = False # if this variable will be true, baxter follows the humans 
dynamicControl = False
armFollowControl = False # if this variable will be true, baxter follows its determined arm
isItLeftArm = True
defaultMsg = "This message is for controlling the msgs (is it the same with previous one?)"

# helpers: handle the noise while following the arm
xAxisLeft = 0
yAxisLeft = 0
xAxisRight = 0
yAxisRight = 0
c = 0.75 #This variable keeps the distance between origin and head

# helpers: handle the noise while following the human
elementOfHandleList = 0
coor = 0
isSystemRun = True
oldCoor = 0
xOld = 0
pKOld = 1
sizeOfHandleList = 35
handleList = [0]*35

def isInAvailablePercentage(minimum, current, percentage):
    rangeOfPercentage = percentage / 100.0
    if abs(current-minimum) < (minimum * rangeOfPercentage):
        return True
    else:
        return False

# publish image is a function which displays the image given with parameter. Image Type: Numpy array
def publish_image(img):
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="rgba8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)

# Statistical Functions
def mode(numbers) :
    largestCount = 0
    modes = []
    for x in numbers:
        if x in modes:
            continue
        count = numbers.count(x)
        if count > largestCount:
            del modes[:]
            modes.append(x)
            largestCount = count
        elif count == largestCount:
            modes.append(x)
    return modes[0]

def stddev(lst):
    mean = float(sum(lst)) / len(lst)
    return (float(reduce(lambda x, y: x + y, map(lambda x: (x - mean) ** 2, lst))) / len(lst))**0.5

def mean(listX) :
    return sum(listX) / len(listX)

def eliminateOutliers(listOfData, limit) :
    stdDeviation = stddev(listOfData)
    meanOfList = mean(listOfData)

    for element in listOfData :
        if stdDeviation != 0 :
            zScore = abs(element - meanOfList) / stdDeviation
            if zScore > limit :
                del element
    return listOfData

""" Callback Functions """

def callback_Command(data):
    global isSystemRun
    global defaultMsg
    global humanFollowControl
    global armFollowControl
    global isItLeftArm
    global dynamicControl
    msg = data.data.lower() # All lethers were made in small to compare.
    print "recieved msg is : {}".format(msg)
    msgs = msg.split("_") #this array keeps all variables
   
    # Messages and actions

    if len(msgs) == 1 and msg != defaultMsg:
        
        if msgs[0] == "default" :
            face.emotion_default(cv2, publish_image)
            print "Default Emotion is applicated"

        if msgs[0] == "happy" :
            face.emotion_happy(cv2, publish_image)
            print "Emotion happy is applicated"
            
        elif msgs[0] == "angry" :
            face.emotion_angry(cv2, publish_image)
            print "Emotion angry is applicated"

        elif msgs[0] == "confused" :
            face.emotion_confused(cv2, publish_image)
            print "Emotion confused is applicated"

        elif msgs[0] == "sad" :
            face.emotion_sad(cv2, publish_image)
            print "Emotion sad is applicated"

        elif msgs[0] == "panic" :
            face.emotion_panic(cv2, publish_image)
            print "Emotion panic is applicated"

        elif msgs[0] == "bored" :
            face.emotion_bored(cv2, publish_image)
            print "Emotion bored is applicated"

        elif msgs[0] == "crafty" :
            face.emotion_crafty(cv2, publish_image)
            print "Emotion crafty is applicated"

        elif msgs[0] == "exit" :
            print "Program is closing..."
            face.sleep(cv2, publish_image)
            rospy.sleep(1)
            print "Program is closed"
            isSystemRun = False
            sys.exit()

        elif msgs[0] == "enable" :
            wobbler.enable()
            wobbler.wobble(0.0)
            print "Wobbler is enabled and wobbled to 0.0"

        elif msgs[0] == "disable" :
            wobbler.disable()
            print "Wobbler is disabled"

        elif msgs[0] == "sleep" :
            face.sleep(cv2, publish_image)
            print "Sst! Baxter is sleeping right now"
            
        defaultMsg = msg
    
    elif len(msgs) == 2 and msg != defaultMsg :
        
        if msgs[0] == "skin" :
            numberOfSkin = int(msgs[1]) 
            face.skin.setSkin(numberOfSkin)
            face.show(publish_image)
        
        elif msgs[0] == "wobble" :
            angle = float(msgs[1])
            wobbler.wobble(angle)
            print "Wobbling is applicated"

        elif msgs[0] == "wake" and msgs[1] == "up" :
            face.wakeUp(cv2, publish_image)
            print "Baxter woke up"

        defaultMsg = msg

    elif len(msgs) == 3 and msg != defaultMsg :
        if msgs[0] == "look" :
            x = int(msgs[1])
            y = int(msgs[2])
            face.lookWithMotion(cv2, x, y, 0.5, publish_image)

        elif msgs[0] == "human" and msgs[1] == "follow" and msgs[2] == "on" :
            face.lookWithMotion(cv2, 0, 0, 0.5, publish_image)
            humanFollowControl = True
            armFollowControl = False
            dynamicControl = False
            wobbler.enable()
            wobbler.wobble(0.0)
            print "Human following mod on"

        elif msgs[0] == "human" and msgs[1] == "follow" and msgs[2] == "off" :
            humanFollowControl = False
            dynamicControl = False
            print "Human following mod off"
            face.lookWithMotion(cv2, 0, 0, 0.5, publish_image)
            dynamicControl = False
            wobbler.enable()
            wobbler.wobble(0.0)

        elif msgs[0] == "arm" and msgs[1] == "follow" and msgs[2] == "off" :
            armFollowControl = False
            print "Arm following mod off"
            face.lookWithMotion(cv2, 0, 0, 0.5, publish_image)
            dynamicControl = False
            wobbler.enable()
            wobbler.wobble(0.0)
        defaultMsg = msg
    
    elif len(msgs) == 4 and msg != defaultMsg :
        if msgs[0] == "look" :
            x = int(msgs[1])
            y = int(msgs[2])
            second = float(msgs[3])
            face.lookWithMotion(cv2, x, y, second, publish_image)

        elif msgs[0] == "dynamic" and msgs[1] == "look" :
            x = int(msgs[2])
            y = int(msgs[3])
            face.lookWithMotionDynamic(cv2, x, y, 0.5, publish_image, wobbler)

        elif msgs[0] == "dynamic" and msgs[1] == "human" and msgs[2] == "follow" and msgs[3] == "on" :
            humanFollowControl = True
            armFollowControl = False
            dynamicControl = True
            print "Human following mod on"

        elif msgs[0] == "left" and msgs[1] == "arm" and msgs[2] == "follow" and msgs[3] == "on" :
            humanFollowControl = False
            armFollowControl = True
            isItLeftArm = True
            dynamicControl = False
            wobbler.enable()
            wobbler.wobble(0.0)
            print "Left arm following mod on"

        elif msgs[0] == "right" and msgs[1] == "arm" and msgs[2] == "follow" and msgs[3] == "on" :
            humanFollowControl = False
            armFollowControl = True
            isItLeftArm = False
            dynamicControl = False
            wobbler.enable()
            wobbler.wobble(0.0)
            print "Right arm following mod on"
        defaultMsg = msg
    
    elif len(msgs) == 5 and msg != defaultMsg :
        if msgs[0] == "dynamic" and msgs[1] == "right" and msgs[2] == "arm" and msgs[3] == "follow" and msgs[4] == "on" :
            humanFollowControl = False
            armFollowControl = True
            isItLeftArm = False
            dynamicControl = True
            print "Dynamic right arm following mod on"

        if msgs[0] == "dynamic" and msgs[1] == "left" and msgs[2] == "arm" and msgs[3] == "follow" and msgs[4] == "on" :
            humanFollowControl = False
            armFollowControl = True
            isItLeftArm = True
            dynamicControl = True
            print "Dynamic left arm following mod on"
        defaultMsg = msg


# this function for the human following
def callback_human_follow(msg):

    global xOld 
    global pKOld 
    global coor
    global elementOfHandleList
    global oldCoor
    sonarIDs = msg.channels[0].values 
    sonarDistances = msg.channels[1].values
    #r is a standart deviation of the sonar sensors' values.
    r = 0.50635561 

    #arrayOfSonarID is sensor shoul be proccessed
    arrayOfSonarID = humanFollowNoiseElimination(sonarIDs, sonarDistances) 
    numberOfData = len(arrayOfSonarID)
    
    if numberOfData > 0:
        meanOfSonarID = mean(arrayOfSonarID)
        # Kalman Filter Part
        K = pKOld / (pKOld + r) 
        x = xOld + K*(meanOfSonarID-xOld)
        pK = (1-K) * pKOld
        prob = 0.03 # Prob value determines that how much measured value effect the kalman filter value.
        x = (x * (1-prob)) + meanOfSonarID*prob # Result of the kalman filter

        # Meaning of the last 35 value
        handleList[elementOfHandleList] = x
        elementOfHandleList += 1
        elementOfHandleList %= sizeOfHandleList

        # Output of the value
        xOld = x
        pKOld = pK
        
        value = int(mean(handleList) * 26.67)
        oldCoor = coor
        coor = value #Coor is the coordinate of the object according to robot's eye
        #print "Coor: {}, SumOfSensors: {}".format(coor, sum(arrayOfSonarID))

def humanFollowNoiseElimination(sonarIDs, sonarDistances) :
    arrayOfSonarID = []
    numberOfData = len(sonarIDs)
    counter = 0
    minimumIndex = 0
    maximumDistance = 2 # maximum distance as a unit of meter
    percentageRate = 30 # to understand the object in front of it

    # determine the minimum index
    for index in range(numberOfData):
        if (sonarIDs[index] <= 3 and sonarIDs[index] >= 0) or (sonarIDs[index] >= 9 and sonarIDs[index] <= 11):
            if sonarDistances[index] < sonarDistances[minimumIndex]:
                minimumIndex = index
    
    # Determining the values will be proccesed
    for index in range(numberOfData):
        if sonarIDs[index] <= 3 and sonarIDs[index] >= 0:
            if sonarDistances[index] < maximumDistance and isInAvailablePercentage(sonarDistances[minimumIndex], sonarDistances[index], percentageRate):
                levelOfSonar = float(sonarIDs[index])*(-1) # resizing the value between [-3, 3]
                arrayOfSonarID.append(levelOfSonar)
                counter += 1
                continue
        
        elif sonarIDs[index] >= 9 and sonarIDs[index] <= 11:
            if sonarDistances[index] < maximumDistance and isInAvailablePercentage(sonarDistances[minimumIndex], sonarDistances[index], percentageRate):
                levelOfSonar = (12-float(sonarIDs[index])) # resizing the value between [-3, 3]
                arrayOfSonarID.append(levelOfSonar)
                counter += 1
                continue

    # Eliminate the outliers
    if counter > 0 :
        arrayOfSonarID = eliminateOutliers(arrayOfSonarID, 1.3)
    
    return arrayOfSonarID

def callback_left_arm_follow(msg) :
    global xAxisLeft
    global yAxisLeft
    # taken the coordinates
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    # angle calculation of y axis
    yAxisAngle = math.atan(abs(x)/(z-c)) 
    if yAxisAngle < 0:
        yAxisAngle = (-3.14/2)-yAxisAngle
    else:
        yAxisAngle = (3.14/2)-yAxisAngle
    if isItLeftArm == True :
        yAxisLeft = (-76.394) * ( yAxisAngle )
        xAxisLeft = (57.294) * ( math.atan(y/abs(x)) )

def callback_right_arm_follow(msg):
    global xAxisRight
    global yAxisRight
    # taken the coordinates
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    # angle calculation of y axis
    yAxisAngle = math.atan(abs(x)/(z-c)) 
    if yAxisAngle < 0:
        yAxisAngle = (-3.14/2)-yAxisAngle
    else:
        yAxisAngle = (3.14/2)-yAxisAngle

    if isItLeftArm == False :
        yAxisRight = (-76.394) * ( yAxisAngle )
        xAxisRight = (57.294) * ( math.atan(y/abs(x)) )

    """ Main Functions """

def main():
    global wobbler
    print "entered main part..."
    wobbler = head_wobbler.Wobbler()
    face.testAllImages(cv2, publish_image)
    face.sleep(cv2, publish_image)
    rospy.Subscriber('/robot/sonar/head_sonar/state', PointCloud, callback_human_follow)
    rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, callback_left_arm_follow)
    rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, callback_right_arm_follow)
    rospy.Subscriber('display_chatter', String, callback_Command)
    rospy.spin()
    return 0

def main_loop() :
    global isSystemRun
    rospy.sleep(2)
    rate = rospy.Rate(10) #10 times in a second (loop frequency)
    #These time keepers for the eyelid
    referenceTime = timeit.default_timer()
    currentTime = timeit.default_timer()
    print "entered main loop part..."

    while not rospy.is_shutdown() :
        
        # Blink for each 5 seconds.
        currentTime = timeit.default_timer()
        if currentTime - referenceTime > 5:
            face.wink(cv2, publish_image)
            referenceTime = timeit.default_timer()
            print "wink motion is applicated"

        if humanFollowControl == True :
            if oldCoor != face.eye.getPositionX():
                if dynamicControl == False :
                    face.eye.lookExactCoordinate(coor, 0)
                    face.show(publish_image)
                else : 
                    face.lookExactCoordinateDynamic(cv2, coor, 0, publish_image, wobbler)
                    face.show(publish_image)

        elif armFollowControl == True :
            if dynamicControl == False :
                if isItLeftArm:
                    face.eye.lookExactCoordinate(int(xAxisLeft), int(yAxisLeft))
                else:
                    face.eye.lookExactCoordinate(int(xAxisRight), int(yAxisRight))
            else:
                if isItLeftArm:
                    face.lookExactCoordinateDynamic(int(xAxisLeft), int(yAxisLeft), publish_image, wobbler)
                else:
                    face.lookExactCoordinateDynamic(int(xAxisRight), int(yAxisRight), publish_image, wobbler)
            face.show(publish_image)

        if isSystemRun == False :
            sys.exit()
    
    face.show(publish_image)
    isSystemRun = False

if __name__ == '__main__' :

    rospy.init_node('rsdk_xdisplay_image', anonymous=True)
    
    threadMain = threading.Thread(name='listener', target=main)
    threadMainLoop = threading.Thread(name='main_loop', target=main_loop)

    try:
        threadMain.daemon = True
        threadMainLoop.daemon = True
        threadMainLoop.start()
        threadMain.start()
    except (KeyboardInterrupt, SystemExit):
        cleanup_stop_thread()
        sys.exit()

    except :
        print "Unable to start thread"
    while 1 :
        if isSystemRun == False :
            break
        pass
