#!/usr/bin/env python

from gazebo_controller.srv import position
import rospy

class client:
    def __init__(self):
        #Initialize the client node
        rospy.init_node("gazebo_client")

        #Show in the console the message
        rospy.loginfo("Client initiated")

        #Pause the node untill the service specified(service_label) is active
        rospy.wait_for_service("gazebo_ser")

    cli = rospy.ServiceProxy("gazebo_ser", position)

if __name__=='__main__':
    n = client()

    while not rospy.is_shutdown():
        try:
            x_str=raw_input("Enter 4 int value for x: ")
            x = [int(x_val) for x_val in x_str.split()]
        except ValueError:
            rospy.logerr("Invalid input")
            x = 0
        try:
            y_str=raw_input("Enter 4 int value for y: ")
            y = [int(y_val) for y_val in y_str.split()]
        except ValueError:
            rospy.logerr("Invalid input")
            y = 0
        try:
            theta_str=raw_input("Enter 4 int value for orientation: ")
            theta = [float(theta_val) for theta_val in theta_str.split()]
        except ValueError:
            rospy.logerr("Invalid input")
            theta = 0
        
        result = n.cli(x,y, theta)
        print result.res
        rospy.spin()