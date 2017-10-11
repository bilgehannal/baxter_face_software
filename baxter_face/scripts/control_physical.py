#!/usr/bin/env python

'''
@Author: Bilgehan NAL
This file is a publisher node. 
This node listens the data came from the buttons of baxter.
According to this data user can control the baxter's face.

'''

import rospy
import baxter_interface
from std_msgs.msg import String
import time

# commands are the emotions as a list
commands = ['default',
'happy',
'sad',
'confused',
'angry',
'panic',
'crafty',
'bored']

'''
getEmotion function returns the key of the emotions according to
the situation of the wheel.
'''
def getEmotion(value):
    numberOfEmotions = len(commands)
    rangeOfEmotions = 256.0/float(numberOfEmotions)
    return int(float(value)/rangeOfEmotions)

def main():
    rospy.init_node("control", anonymous=True)
    pub = rospy.Publisher('display_chatter', String, queue_size=40) #Display chatter publisher defined.

    # This variables fot helping that which action will be chosed
    indexOfEmotion = 0
    isWakeUp = False
    isFollow = False
    emotion = 0
    isEnable = False

    '''
    Navigators are declared.
    -> Navigators on the arms are used for changing the emotions. 
    Okay buttons of the arm navigators are used for enable or disable robot
    -> Navigators on the torses are unsed for the arm following action.
    Also they are used for two actions. (Sleep and wake up)
    '''
    navLeftArm = baxter_interface.Navigator('left')
    navRightArm = baxter_interface.Navigator('right')
    navLeftTorso = baxter_interface.Navigator('torso_left')
    navRightTorso = baxter_interface.Navigator('torso_right')

    print "Controller is enable..."

    while not rospy.is_shutdown():
        # Arm navigators okay button to enable and disable
        if navLeftTorso._state.buttons[0] or navRightTorso._state.buttons[0]:
            if not isEnable:
                pub.publish("enable")
                isEnable = True
                print "enable"
            else:
                pub.publish("disable")
                isEnable = False
                print "disable"
        #Left arm up button to wake up
        elif navLeftArm._state.buttons[1]:
            pub.publish("wake_up")
            isWakeUp = True
            print "wake_up"
        #Left arm down button to sleep
        elif navLeftArm._state.buttons[2]:
            pub.publish("sleep")
            isWakeUp = False
            print "sleep"
        #Right arm buttons to follow arms
        elif navRightArm._state.buttons[1]:
            if isFollow:
                pub.publish("arm_follow_off")
                isFollow = False
                print "arm_follow_off"
            else:
                pub.publish("dynamic_left_arm_follow_on")
                isFollow = True
                print "dynamic_left_arm_follow_on"

        elif navRightArm._state.buttons[2]:
            if isFollow:
                pub.publish("arm_follow_off")
                isFollow = False
                print "arm_follow_off"
            else:
                pub.publish("dynamic_right_arm_follow_on")
                isFollow = True
                print "dynamic_right_arm_follow_on"
        else:
            # Wheel Control
            currentEmotion = getEmotion(navLeftArm._state.wheel)
            if not emotion == currentEmotion and isWakeUp:
                pub.publish(commands[currentEmotion])
                emotion = currentEmotion
                print commands[currentEmotion]
                print currentEmotion
            continue
        
        print "Wait for 0.3 secs "
        time.sleep(0.3)
        print "Okay:"

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass