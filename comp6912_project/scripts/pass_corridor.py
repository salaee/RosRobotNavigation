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
import datetime

sranges = []
angle_min = 0
angle_increment = 0
# store start time
s_time = datetime.datetime.now()

def find_shortest_ray(ranges, start, end):
    print "shortest ray: ",
    l = len(ranges)
    ray_ranges = []
    for i in range(start, end):
        ray_ranges.append(ranges[i])
        
    m = min(ray_ranges)
    print m
    print "index: ",
    i = ranges.index(m)
    print i
    return m, i

def radian_to_degree(r):
    return r * (180 / math.pi)

def degree_to_radian(d):
    return d * (math.pi / 180)

def radian_of_ray(n, theta):
    ray = angle_min + n * angle_increment
    t = theta
    return ray + t

def find_longest_ray(ranges):
    l = len(ranges)
    ray = max(ranges)
    i = ranges.index(ray)
    return i

def get_twist(x, y, theta, ranges):
    twist = Twist()
    # Compute a twist message
    os.system('clear')
    print "------- Moving in the middle of corridor -------"
    print "Parameters:"
    print " "


    # distance to left side of robot with 35 degree from where it is headed
    #phi_1 = radian_of_ray(64,theta)
    d1 = ranges[64]
    print "d1 = {0}".format(d1) 
    # distance to right side of robot with 35 degree from where it is headed
    #phi_2 = radian_of_ray(35,theta)
    d2 = ranges[35]
    print "d2 = {0}".format(d2)

    inverse_d1 = 1 / d1
    inverse_d2 = 1 / d2

    phi = degree_to_radian(35.15)
    k1 = 2.5
    v = 0.6
    w = - k1 * v * math.sin(phi) * (inverse_d1 - inverse_d2)
    print "k1 = {0}".format(k1)
    print "v = {0}".format(v)
    print "w = {0}".format(w)

    h_ray = ranges[49]
    print "distance to an obstacle: {0}".format(h_ray)

    # If there isn't any obstacle in front of robot
    if h_ray > 1:
        twist.linear.x = v
        twist.angular.z = w
    else:
        index = find_longest_ray(ranges)
        r = radian_of_ray(index, theta)
        print index
        twist.linear.x = 0
        twist.angular.z = abs(r)

    now_time = datetime.datetime.now()
    e_time = now_time - s_time
    print e_time

    print "------------------------"
    return twist

def scan_callback(scan):
    global sranges, angle_increment, angle_min

    angle_increment = scan.angle_increment
    angle_min = scan. angle_min
    sranges = copy.copy(scan.ranges)


if __name__ == '__main__':
    rospy.init_node('pass_corridor')

    # Subscribe to /base_scan
    rospy.Subscriber('/robot_0/base_scan', LaserScan, scan_callback)

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

        # Get a twist object from the controller. 
        twist = get_twist(x, y, theta, sranges)

        # Publish the twist message produced by the controller.
        cmd_vel_publisher.publish(twist)

        rate.sleep()


