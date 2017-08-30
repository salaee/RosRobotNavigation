#!/usr/bin/env python  

#
# Code for Following a target based on the paper
# "Semi-Autonomous navigation of a robotic wheelchair" by 
# Antonis ArgyrosPantelis GeorgiadisPanos TrahaniasDimitris Tsakiris
# Implemented by: Ghassem Alaee
#

import rospy
import math
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pi
import copy
import os

sranges = []
angle_min = 0
angle_increment = 0


def radian_to_degree(r):
    return r * (180 / math.pi)

def get_angle_phi(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    phi = math.atan2(dy,dx)
    return phi


def get_twist(x, y, theta, xT, yT):
    twist = Twist()
    # Compute a twist message
    os.system('clear')
    print "------- Following a target -------"
    print "Parameters:"
    print " "
    # calculating r and phi
    A = math.pow(xT-x, 2)
    B = math.pow(yT-y, 2)
    r = math.sqrt(A + B)
    print "r = {0}".format(r)
    angle = get_angle_phi(x,y,xT,yT)
    phi =  angle - theta
    print "phi = {0}".format(phi)

    # Point N is a point with '2 meters' distance to the target
    # Xmn and Ymn are distance of point n to robot
    # These are the parameters determining the position 
    # where the target will be held with respect to the wheelchair
    Ymn = 1
    Xmn = 1

    print "Xmn = {0}".format(Xmn)  
    print "Ymn = {0}".format(Ymn)

    # point N = the virtual point which we have to get to this point and
    #           and keep a constant distance to 'target'
    Xn = x + (Xmn * math.cos(theta)) - Ymn * math.sin(theta)
    Yn = y + (Xmn * math.sin(theta)) + Ymn * math.cos(theta)

    print "Xn = {0}".format(Xn)
    print "Yn = {0}".format(Yn)

    # Tracking error
    eX = (Xmn * math.cos(theta)) - (Ymn * math.sin(theta))  - (r * math.cos(theta + phi)) 
    eY = (Xmn * math.sin(theta)) - (Ymn * math.cos(theta))  - (r * math.sin(theta + phi)) 

    print "eX = {0}".format(eX)
    print "eY = {0}".format(eY)

    # parameters:
    k = 1
    print "k = {0}".format(k) 

    # Control Law
    # based on sensory information (r,phi):
    v = -k * (Xmn * (Xmn - (r * math.cos(phi)) + Ymn * (Ymn - (r * math.sin(phi))))) / Xmn
    w = -k * (Ymn - r * (math.sin(phi))) / Xmn

    print "v = {0}".format(v)
    print "w = {0}".format(w) 

    twist.linear.x = v
    twist.angular.z = w

    print "------------------------"
    return twist


if __name__ == '__main__':
    rospy.init_node('follow_target')

    # Create a transform listener.
    listener = tf.TransformListener()

    # Create a publisher so that we can output command velocities.
    cmd_vel_publisher = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        try:
            # Determine the transform from the /odom frame to the 
            # /base_footprint frame.  /odom represents the robot's oodometric
            # frame of reference.  Since the odometry is perfect in the autolab
            # Stage world, /odom remains coincident with the global reference
            # frame.  /base_footprint represents the pose of the robot's base.
            (trans,rot) = listener.lookupTransform('/robot_0/odom', '/robot_0/base_footprint', \
                                                   rospy.Time(0))

            (trans_t,rot_t) = listener.lookupTransform('/robot_1/odom', '/robot_1/base_footprint', \
                                                   rospy.Time(0))
        except (tf.LookupException, \
                tf.ConnectivityException, tf.ExtrapolationException):
            # The /odom frame doesn't come into existence right away, which
            # will lead to an exception being generated from lookupTransform.
            continue

        # Get the 2-D position and orientation of the robot with respect to the
        # global frame.
        x = trans[0]
        y = trans[1]
        euler_angles = euler_from_quaternion(rot)
        theta = euler_angles[2]

        # Get 2-D positionof 'target' with respect to the global frame
        xT = trans_t[0]
        yT = trans_t[1]


        # Get a twist object from the controller. 
        twist = get_twist(x, y, theta, xT, yT)

        # Publish the twist message produced by the controller.
        cmd_vel_publisher.publish(twist)

        rate.sleep()


