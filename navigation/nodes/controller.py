#!/usr/bin/env python

PACKAGE = 'navigation'
NODE = 'controller'
EPS_LINEAR = 0.2
EPS_ANGULAR = 0.2

import numpy as np
import sys
from time import sleep

#ROS DEPENDENCIES
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from navigation.msg import *

class Controller:
    def __init__(self):
        #Node Initialization
        rospy.init_node(NODE)

        #Set Publisher
        #To publish the current situation of the robot
        self.situation_pub = rospy.Publisher("navigation/situation", Bool, queue_size = 10)
        #To publish the velocity to the robot
        self.setVelocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

        #Set Subscriber
        #To get a set of desired position from user
        self.targetPos_sub = rospy.Subscriber("navigation/position", poseArray, self.getTargetPos)
        #To get the current position of the robot
        self.currentPos_sub = rospy.Subscriber("/odom", Odometry, self.getPosition)

        #Variable Initialization
        #Initialize the situation
        self.targetReached = Bool()
        self.targetReached.data = True
        #Initialize the position
        self.targetPos = poseArray()
        self.targetPos.poses = [pose(), pose(), pose(), pose(), pose()]     #First element is the initial position, the others are desired position
        self.totalPoint = 5 
        self.currentPos = pose()
        self.subset = 0                         #Route is divided into 4 subsets
        #Initialize the velocity
        self.velocity = Twist()
    
    #Check the current situation of the robot
    def getPosition(self, msg):
        self.currentPos.x = msg.pose.pose.position.x
        self.currentPos.y = msg.pose.pose.position.y

        #The output orientation of odometer is in quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.currentPos.theta = yaw

        #Debug
        #print "Current pos X = %0.2f" %(self.currentPos.x)
        #print "Current pos Y = %0.2f" %(self.currentPos.y)
        #print "Current theta = %0.2f" %(self.currentPos.theta)

        self.setVelocity(self.targetReached.data)

        if not self.targetReached.data:
            self.checkSituation()

    def checkSituation(self):
        #Check the position of the robot (check per subpoint)
        #print self.subset
        distX = abs(self.currentPos.x - self.targetPos.poses[self.subset + 1].x)
        distY = abs(self.currentPos.y - self.targetPos.poses[self.subset + 1].y)
        if distX < EPS_LINEAR and distY < EPS_LINEAR:
            print "Finish destination %d" %(self.subset + 1)
            if self.subset < self.totalPoint - 2:
                self.subset += 1
            else:
                self.subset = 0
                self.targetReached.data = True   
                self.situation_pub.publish(self.targetReached)          #Ask for new set of data as the target has been reached  
        
    #Get a set of desired position from the user then set the shortest route
    def getTargetPos(self, msg):
        self.targetReached.data = False
        self.situation_pub.publish(self.targetReached)                  #Tell the userinterface to stop asking for more input

        self.targetPos = msg
        self.setRoute()

    def setRoute(self):
        #Insert the initial position to index 0 of the array
        self.targetPos.poses.insert(0, self.currentPos)         
        
        #Calculate the shortest distance
        for i in range(self.totalPoint - 2):
            refValue = self.targetPos.poses[i]
            closest = sys.float_info.max
            index = 0
            for j in range(i + 1, self.totalPoint):
                dist = np.sqrt(np.square(self.targetPos.poses[j].x - refValue.x) + np.square(self.targetPos.poses[j].y - refValue.y))
                if dist < closest:
                    closest = dist
                    index = j
            if index == i + 1:
                pass
            else:
                #swap value
                self.targetPos.poses[i+1], self.targetPos.poses[index] = self.targetPos.poses[index], self.targetPos.poses[i+1]
        
        #Debugging
        #print "SELECTED ROUTE: "
        #for point in self.targetPos.poses:
        #    print point.x,
        #    print point.y,
        #    print point.theta
    
    def setVelocity(self, finish):
        if not finish:
            #Calculate the angle and the distance to reach linear position
            deg, dist = self.calcDegDist(self.currentPos, self.targetPos.poses[self.subset + 1])
        
            #Debug
            #print "degree %d = %0.2f" %(self.subset, deg)
            #print "distance %d = %0.2f" %(self.subset, dist)
            #print "Current theta = %0.2f" %(self.currentPos.theta)
            #print "Target theta = %0.2f" %(deg)

            deltaTheta = self.currentPos.theta - deg
            
            if abs(deltaTheta) < EPS_ANGULAR:
                #Move linear
                self.velocity.angular.z = 0.0
                self.velocity.linear.x = EPS_LINEAR * 2
            else:
                if deltaTheta > 0:
                    const = -1
                else:
                    const = 1
                #Rotate
                self.velocity.linear.x = 0.0
                self.velocity.angular.z = EPS_ANGULAR * 2 * const
        else:           #if finish
            self.velocity.linear.x = 0.0
            self.velocity.linear.y = 0.0
            self.velocity.angular.z = 0.0

            ##NOTES TO MYSELF
            #current position is taken regularly. so check whether the robot has reached the sub-route (a point)
            #if so, then change the initPos to that point and the destPos to the next point
            #should we make a global index holder; to which desired point are we going now?
            #it's better to model marks in the simulation. that way the user will know which position they want on the environment
            #and which route the robot will take --> if time is available, of course. don't think about it now..

    def calcDegDist(self, curPos, destPos):
        #Calculate distance
        dist = np.sqrt(np.square(destPos.x - curPos.x) + np.square(destPos.y - curPos.y))

        #Calculate angle
        deg = np.arctan2((destPos.y - curPos.y),(destPos.x - curPos.x))
        
        #NOTES TO MYSELF
        #use arctan2 instead of arctan. It gives the right quadrant and it accepts infinite so we don't need to check 
        #whether the divider is 0

        return deg, dist

if __name__ == '__main__':
    controller = Controller()
    
    try:                    #ROS takes some time to notify the publishers and subscribers of the topics.
        sleep(5)
        controller.situation_pub.publish(controller.targetReached)
    except rospy.ROSInterruptException:
        raise e

    while not rospy.is_shutdown():
        controller.setVelocity_pub.publish(controller.velocity)
        sleep(1)

