#!/usr/bin/env python

PACKAGE = 'navigation'
NODE = 'userinput'                  #REMEMBER: avoid using space

import rospy
from std_msgs.msg import Bool
from navigation.msg import *

class UserInput:
    def __init__(self):
        #Node Initialization
        rospy.init_node(NODE, anonymous = True)

        #Set Publisher
        #To publish a set of 4 desired position
        self.targetPos_pub = rospy.Publisher("navigation/position", poseArray, queue_size = 10)

        #Set Subscriber
        #To ask for whether the robot has reached the destination: false or true
        self.situation_sub = rospy.Subscriber("navigation/situation", Bool, self.getSituation)

        #Variable Initialization
        #Initialize the target position
        self.targetPos = poseArray()
        self.targetPos.poses = [pose(), pose(), pose(), pose()]
        #Initialize the situation
        self.targetReached = Bool()
        
    #Get the current situation: has the robot reached the final position
    def getSituation(self, msg):
        self.targetReached = msg
        
        print self.targetReached.data               #debugging

        if self.targetReached.data:
            print "The robot has reached the final destination"
            self.readInput()
            self.targetPos_pub.publish(self.targetPos)
        else:
            #pass
            print "Please wait for more minute"

    #Ask the user to input 4 desired position
    def readInput(self):
        for i in range(4):
            print "Position % d" % (i+1)
            try:
                self.targetPos.poses[i].x = float(raw_input("Enter x-position: "))
            except ValueError:
                self.targetPos.poses[i].x = 0.0
            try:
                self.targetPos.poses[i].y = float(raw_input("Enter y-position: "))
            except ValueError:
                self.targetPos.poses[i].y = 0.0
            try:
                self.targetPos.poses[i].theta = float(raw_input("Enter theta (rad): "))
            except ValueError:
                self.targetPos.poses[i].theta = 0.0
        
        #Debugging
        for i in range(4):
            print self.targetPos.poses[i].x
            print self.targetPos.poses[i].y
            print self.targetPos.poses[i].theta

if __name__ == '__main__':
    userinput = UserInput()
    
    rospy.spin()
