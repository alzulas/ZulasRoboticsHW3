#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from intro_to_robotics.image_converter import ToOpenCV, depthToOpenCV
from math import *
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros
import copy

LINEAR_VELOCITY_MINIMUM_THRESHOLD = 0.2
ANGULAR_VELOCITY_MINIMUM_THRESHOLD = 0.4
L = .23
R = .035


#this function does our image processing
#returns the location and "size" of the detected object
def process_image(image):
    #convert color space from BGR to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #create bounds for our color filter
    lower_bound = np.array([0, 10, 10])
    upper_bound = np.array([10,255,255])

    #execute the color filter, returns a binary black/white image
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    #display the results of the color filter
    cv2.imshow("image_mask", mask)

    #calculate the centroid of the results of the color filer
    M = cv2.moments(mask)
    location = None
    magnitude = 0
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        magnitude = M['m00']
        location = (cx-320, cy-240) #scale so that 0,0 is center of screen
        #draw a circle image where we detected the centroid of the object
        cv2.circle(image, (cx,cy), 3, (0,0,255), -1)

    #display the original image with the centroid drawn on the image
    cv2.imshow("processing result", image)

    #waitKey() is necessary for making all the cv2.imshow() commands work
    cv2.waitKey(1)
    return location, magnitude


class Node:
    def __init__(self):
        #register a subscriber callback that receives images
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1)

        #create a publisher for sending commands to turtlebot
        self.movement_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    #this function wll get called every time a new image comes in
    #all logic occurs in this function
    def image_callback(self, ros_image):
        # convert the ros image to a format openCV can use
        cv_image = np.asarray(ToOpenCV(ros_image))

        #run our vision processing algorithm to pick out the object
        #returns the location (x,y) of the object on the screen, and the
        #"size" of the discovered object. Size can be used to estimate
        #distance
        #None/0 is returned if no object is seen
        location, magnitude = process_image(cv_image)

        #log the processing results
        rospy.logdebug("image location: {}\tmagnitude: {}".format(location, magnitude))
	print ("location = %s magnitude = %d" % (location, magnitude))

        ###########
        # Insert turtlebot controlling logic here!
        ###########
        cmd = Twist()
	#x is positive if the turtle bot is heading to the left of the object
	#x is negative if the turtle bot is heading to the right of the object
	if location[0] < -30: #If the x in your location is negative, you should move left
		print "I am to heading to the right of the object"
		linear_vel = (R/2) * (13 + 12) #Robots x, moving left
        	ang_vel = (R/L) * (13- 12) #Robots theta, turning a little to the left
		cmd.linear.x = linear_vel #Post to the twist command
		cmd.angular.z = ang_vel
	elif location[0] > 30:#If the x in your location is positive, you should move right
		print "I am to heading to the left of the object"
		linear_vel = (R/2) * (12 + 13) #Robots x, moving right
        	ang_vel = (R/L) * (12- 13) #Robots theta, turning a little to the right
		cmd.linear.x = linear_vel #Post to the twist command
		cmd.angular.z = ang_vel 
	else: 	#if you're between -30 and + 30, stay true
		print "I am mostly centered on the object"
		linear_vel = (R/2) * (15 + 15) #Robots x #don't turn any
        	ang_vel = (R/L) * (15- 15) #Robots theta
		cmd.linear.x = linear_vel #Post to twist
		cmd.angular.z = ang_vel 

	if location[1] > -5: #If you're close to the object time to make a corner
		print "Got to a corner!"
		linear_vel = (R/2) * (1.75 + -1.75) #Robots x doesn't move forward
        	ang_vel = (R/L) * (1.75 - -1.75) #Robots theta does a full turn
		cmd.linear.x = linear_vel #Post to twist
		cmd.angular.z = ang_vel 

        #publish command to the turtlebot
        self.movement_pub.publish(cmd)



if __name__ == "__main__":
    rospy.init_node("lab2_example")
    node = Node()

    #this function loops and returns when the node shuts down
    #all logic should occur in the callback function
    rospy.spin()
