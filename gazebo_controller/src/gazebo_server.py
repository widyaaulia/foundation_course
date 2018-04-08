#!/usr/bin/env python

import rospy
import math
import numpy
import tf
from gazebo_controller.srv import position
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class server:
    def __init__(self):
        #initialize the server node
        rospy.init_node("gazebo_server")

        #Display in the console the message
        rospy.loginfo("Server initiated")

        # Tolerance in term of postion and angle
        self.pos_tolerant = 0.2
        self.ang_tolerant = 0.01

        # Create current and target position
        self.current_pose = Odometry()

        # Create velocity
        self.target_vel = Twist()

        # Create subscriber for pose from gazebo
        self.current_pose_sub = rospy.Subscriber("odom", Odometry, self.current_pose_callback)
        
        # Create publisher
        self.target_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

        ser = rospy.Service("gazebo_ser",position,self.func_move)

    def current_pose_callback(self, msg):
        self.current_pose = msg
        pass

    # Calculate distance between current position and target positions
    def distance_calculate(self,array_x,array_y):
        distance=[0]*len(array_x) 
        for i in range(0,len(array_x)):
            current_x = self.current_pose.pose.pose.position.x
            current_y = self.current_pose.pose.pose.position.y
            target_x = array_x[i]
            target_y = array_y[i]
            dx = numpy.abs(current_x - target_x)
            dy = numpy.abs(current_y - target_y)
            distance[i] = math.sqrt(dx*dx + dy*dy)
        return distance

    # Find target position with minimum distance then move the robot
    def move_then_delete(self,target_array_x,target_array_y,target_array_theta):
        current_x = self.current_pose.pose.pose.position.x
        current_y = self.current_pose.pose.pose.position.y
        quar= (self.current_pose.pose.pose.orientation.x,self.current_pose.pose.pose.orientation.y,self.current_pose.pose.pose.orientation.z,self.current_pose.pose.pose.orientation.w)
        current_theta =tf.transformations.euler_from_quaternion(quar)[2]
        distance=self.distance_calculate(target_array_x,target_array_y)
        min_travel=(min(distance))
        min_index=numpy.argmin(distance)
        self.func_perform(target_array_x[min_index],target_array_y[min_index],target_array_theta[min_index])
        target_array_x.pop(min_index)
        target_array_y.pop(min_index)
        target_array_theta.pop(min_index)
        return target_array_x,target_array_y,target_array_theta

    # Move robot to target x -y
    def position_move(self,target_x,target_y):
        self.target_vel.linear.x = 0.5
        self.target_vel.angular.z = 0
        self.target_vel_pub.publish(n.target_vel)
        while True:
            current_x = self.current_pose.pose.pose.position.x
            current_y = self.current_pose.pose.pose.position.y
            dx = numpy.abs(current_x - target_x)
            dy = numpy.abs(current_y - target_y)
            if ((dx < self.pos_tolerant) and (dy < self.pos_tolerant)):
                break
    
    # Move robot to target theta
    def theta_rotation(self, target_theta):
        self.target_vel.linear.x = 0
        self.target_vel.angular.z = 0.5
        self.target_vel_pub.publish(n.target_vel)
        while True:
            quar= (self.current_pose.pose.pose.orientation.x,self.current_pose.pose.pose.orientation.y,self.current_pose.pose.pose.orientation.z,self.current_pose.pose.pose.orientation.w)
            current_theta =tf.transformations.euler_from_quaternion(quar)[2]
            dtheta = numpy.abs(target_theta - current_theta)
            if (dtheta < self.ang_tolerant):
                break
    
    # Move the robot to defined position
    def func_perform(self,target_x, target_y, target_theta):
        self.target_vel.linear.x = 0
        self.target_vel.angular.z = 0.0
        self.target_vel_pub.publish(n.target_vel)
        current_x = self.current_pose.pose.pose.position.x
        current_y = self.current_pose.pose.pose.position.y
        quar= (self.current_pose.pose.pose.orientation.x,self.current_pose.pose.pose.orientation.y,self.current_pose.pose.pose.orientation.z,self.current_pose.pose.pose.orientation.w)
        current_theta =tf.transformations.euler_from_quaternion(quar)[2]
        dx = numpy.abs(current_x - target_x)
        dy = numpy.abs(current_y - target_y)
        dtheta = numpy.abs(target_theta - current_theta)
        distance = math.sqrt(dx*dx + dy*dy)
        angle = numpy.arctan2(target_y - current_y, target_x -current_x)
        
        # Check if (x,y) is okay
        if ((dx < self.pos_tolerant) and (dy < self.pos_tolerant)):
            self.target_vel.linear.x = 0.0
            # We reached the correct (x,y), rotate to theta
            if (dtheta > self.ang_tolerant):
                self.theta_rotation(target_theta)
            else:
                self.target_vel.angular.z = 0.0
                self.target_vel_pub.publish(n.target_vel)
        else:
            angle = numpy.arctan2((target_y - current_y),( target_x - current_x))
            if (abs(angle - current_theta) >self.ang_tolerant):
                self.target_vel.linear.x = 0
                self.target_vel.angular.z = 0.5
                self.target_vel_pub.publish(n.target_vel)
                while True:
                    current_x = self.current_pose.pose.pose.position.x
                    current_y = self.current_pose.pose.pose.position.y
                    quar= (self.current_pose.pose.pose.orientation.x,self.current_pose.pose.pose.orientation.y,self.current_pose.pose.pose.orientation.z,self.current_pose.pose.pose.orientation.w)
                    current_theta =tf.transformations.euler_from_quaternion(quar)[2]
                    if (abs(angle - current_theta) < self.ang_tolerant):
                        break
                self.position_move(target_x,target_y)
                self.theta_rotation(target_theta)
            else:
                self.position_move(target_x,target_y)

        self.target_vel.linear.x = 0
        self.target_vel.angular.z = 0.0
        self.target_vel_pub.publish(n.target_vel)
        print "Success"
        time.sleep(5)
    
    # Move robot to all 4 target positions
    def func_move(self,request):
        target_array_x=list(request.x)
        target_array_y=list(request.y)
        target_array_theta=list(request.theta)
        
        for i in range(0,4):
            target_array_x,target_array_y,target_array_theta=self.move_then_delete(target_array_x,target_array_y,target_array_theta)
            res = True
        return res

if __name__ == '__main__':
    n = server()

    rospy.spin()