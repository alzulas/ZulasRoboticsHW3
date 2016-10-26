import rospy
import tf
import numpy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from math import *
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros
import copy

#run roscore before starting gazebo

LINEAR_VELOCITY_MINIMUM_THRESHOLD = 0.2
ANGULAR_VELOCITY_MINIMUM_THRESHOLD = 0.4
L = .23
R = .035


class GoForward():
    def __init__(self):
        # initiliaze
	rospy.init_node('GoForward', anonymous=False)

	# tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot CTRL + C")

	# What function to call when you ctrl + c    
	rospy.on_shutdown(self.shutdown)
        
	# Create a publisher which can "talk" to TurtleBot and tell it to move
	# Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
	self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

	#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
	r = rospy.Rate(10);

	# as long as you haven't ctrl + c keeping doing...
	while not rospy.is_shutdown():
	    for x in range(0, 4): #forward, rotate, forward, rotate, forward, rotate
		  #To get the turtle bot to move exactly .25 meters both wheels have to spin at 7.14 radians/sec
		  #then to get it to 2 meters, do that 8 timess.
		  self.spinWheels(7.14, 7.14, 8)
		  #Next, rotate 90 degrees.
		  #I prfered to do this by using the rotate function, and instead calculating angular momentum
		  rotate(self, .5, 1.35, True)
		  print("1.35")
	    self.spinWheels(7.14, 7.14, 8) #Last wall of the square
	    shutdown()#Stop
	    r.sleep
	r.sleep()

    def shutdown(self):
        # stop turtlebot
       	rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

    def spinWheels(self, u1, u2, time): #speed of wheel one, speed of wheel two
        linear_vel = (R/2) * (u1 + u2) #Robots x
        ang_vel = (R/L) * (u1 - u2) #Robots theta 

        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = ang_vel

        begin = rospy.get_rostime().secs # first time point of robot

        #while haven't reached time
        while(rospy.get_rostime().secs - begin < time):
            self.cmd_vel.publish(twist_msg)
            #publish twist_msg

        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self.cmd_vel.publish(twist_msg)
        #publish a new twist message to make the robot stop


#So, I used the rotate function that is available through the people who created the turtle bot package. http://edu.gaitech.hk/turtlebot/turtlebot-tutorials.html
#It does a lot of things, and it makes you include a lot of #includes to make it work.
#It takes your turtle bot, the velocity you want to turn at, the radians you want to turn, and whether you want clockwise or counter clockwise (BOOL)
#Then it stops the turtle bot, turns the turtle bot as much as you're asking as fast as you're asking.
def rotate(self,angular_velocity,radians,clockwise):
    rotateMessage = Twist()
    #declare tf transform
    listener = tf.TransformListener()
    #init_transform: is the transformation before starting the motion
    init_transform = geometry_msgs.msg.TransformStamped()
    #current_transformation: is the transformation while the robot is moving
    current_transform = geometry_msgs.msg.TransformStamped()
    angle_turned = 0.0

    angular_velocity = (-angular_velocity, ANGULAR_VELOCITY_MINIMUM_THRESHOLD)[angular_velocity > ANGULAR_VELOCITY_MINIMUM_THRESHOLD]

    while(radians < 0):
        radians += 2* pi

    while(radians > 2* pi):
        radians -= 2* pi
    listener.waitForTransform("/base_footprint", "/odom", rospy.Time(0), rospy.Duration(10.0) )
    (trans,rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
    #listener.lookupTransform("/base_footprint", "/odom", rospy.Time(0),init_transform)

    init_transform.transform.translation = trans
    init_transform.transform.rotation =rot

    #since the rotation is only in the Z-axes
    #start_angle = tf.transformations.#0.5 * sqrt(rot[2] ** 2)
    euler = tf.transformations.euler_from_quaternion(rot)
    roll = euler[0]
    pitch = euler[1]
    start_angle = euler[2]
    rotateMessage.linear.x = rotateMessage.linear.y = 0.0
    rotateMessage.angular.z = angular_velocity

    if(clockwise):
        rotateMessage.angular.z = -rotateMessage.angular.z


    loop_rate = rospy.Rate(20)
    while True: 
        rospy.loginfo("Turtlebot is Rotating")

        self.cmd_vel.publish(rotateMessage)

        loop_rate.sleep()

        #rospy.Duration(1.0)
        try:

            #wait for the transform to be found
            listener.waitForTransform("/base_footprint", "/odom", rospy.Time(0), rospy.Duration(10.0) )
            #Once the transform is found,get the initial_transform transformation.
            #listener.lookupTransform("/base_footprint", "/odom",rospy.Time(0))
            (trans,rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.Duration(1.0)
        current_transform.transform.translation = trans
        current_transform.transform.rotation =rot

        #since the rotation is only in the Z-axes
        #end_angle = 0.5 * sqrt( rot[2] ** 2)
        euler = tf.transformations.euler_from_quaternion(rot)
        roll = euler[0]
        pitch = euler[1]
        end_angle = euler[2]
        angle_turned = abs(end_angle - start_angle)
        print "angle_turned: %s" %angle_turned
        if (angle_turned > radians):
            break

if __name__ == '__main__':
    GoForward()

