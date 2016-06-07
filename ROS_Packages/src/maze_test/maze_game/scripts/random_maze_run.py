#!/usr/bin/env python

import roslib;

roslib.load_manifest('maze_game')
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import numpy as np
from math import pow, sqrt
import re
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


class RandomMazeRun():
    def __init__(self):
        rospy.init_node('random_maze_run', anonymous=True)

        self.time_between_points = []

        # Set up the important locations
        self.locations = dict()

        self.locations["forward"] = Pose()
        self.locations["home"] = Pose(Point(2.0686, 7.26749, 0),
                                      Quaternion(*quaternion_from_euler(0, 0, -math.pi / 4, "sxyz")))
        self.locations["p0"] = Pose(Point(6.730, 6.0, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi, "sxyz")))
        self.locations["p1"] = Pose(Point(5.730, 6.0, 0),
                                    Quaternion(*quaternion_from_euler(0, 0, -math.pi / 2, "sxyz")))
        self.locations["p2"] = Pose(Point(5.730, 4.600, 0), Quaternion(*quaternion_from_euler(0, 0, 0, "sxyz")))
        self.locations["p3"] = Pose(Point(6.230, 4.600, 0),
                                    Quaternion(*quaternion_from_euler(0, 0, math.pi / 2, "sxyz")))
        self.locations["p4"] = Pose(Point(6.230, 5.500, 0), Quaternion(*quaternion_from_euler(0, 0, 0, "sxyz")))
        self.locations["p5"] = Pose(Point(6.730, 5.500, 0),
                                    Quaternion(*quaternion_from_euler(0, 0, -math.pi / 2, "sxyz")))
        self.locations["p6"] = Pose(Point(6.730, 2.900, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi, "sxyz")))
        self.locations["p7"] = Pose(Point(6.230, 2.900, 0),
                                    Quaternion(*quaternion_from_euler(0, 0, math.pi / 2, "sxyz")))
        self.locations["p8"] = Pose(Point(6.230, 4.100, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi, "sxyz")))
        self.locations["p9"] = Pose(Point(5.730, 4.100, 0),
                                    Quaternion(*quaternion_from_euler(0, 0, -math.pi / 2, "sxyz")))
        self.locations["p10"] = Pose(Point(5.730, 2.900, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi, "sxyz")))
        self.locations["p11"] = Pose(Point(5.230, 2.900, 0),
                                     Quaternion(*quaternion_from_euler(0, 0, math.pi / 2, "sxyz")))
        self.locations["p12"] = Pose(Point(5.230, 4.600, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi, "sxyz")))
        self.locations["p13"] = Pose(Point(4.330, 4.600, 0),
                                     Quaternion(*quaternion_from_euler(0, 0, math.pi / 2, "sxyz")))
        self.locations["p14"] = Pose(Point(4.330, 5.100, 0), Quaternion(*quaternion_from_euler(0, 0, 0, "sxyz")))
        self.locations["p15"] = Pose(Point(5.230, 5.100, 0),
                                     Quaternion(*quaternion_from_euler(0, 0, math.pi / 2, "sxyz")))
        self.locations["p16"] = Pose(Point(5.230, 6.000, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi, "sxyz")))
        self.locations["p17"] = Pose(Point(2.100, 6.000, 0),
                                     Quaternion(*quaternion_from_euler(0, 0, -math.pi / 2, "sxyz")))
        self.locations["p18"] = Pose(Point(2.100, 5.100, 0), Quaternion(*quaternion_from_euler(0, 0, 0, "sxyz")))
        self.locations["p19"] = Pose(Point(3.830, 5.100, 0),
                                     Quaternion(*quaternion_from_euler(0, 0, -math.pi / 2, "sxyz")))
        self.locations["p20"] = Pose(Point(3.830, 4.600, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi, "sxyz")))
        self.locations["p21"] = Pose(Point(2.100, 4.600, 0),
                                     Quaternion(*quaternion_from_euler(0, 0, -math.pi / 2, "sxyz")))
        self.locations["p22"] = Pose(Point(2.100, 3.400, 0), Quaternion(*quaternion_from_euler(0, 0, 0, "sxyz")))
        self.locations["p23"] = Pose(Point(2.600, 3.400, 0),
                                     Quaternion(*quaternion_from_euler(0, 0, math.pi / 2, "sxyz")))
        self.locations["p24"] = Pose(Point(2.600, 4.100, 0), Quaternion(*quaternion_from_euler(0, 0, 0, "sxyz")))
        self.locations["p25"] = Pose(Point(4.730, 4.100, 0),
                                     Quaternion(*quaternion_from_euler(0, 0, -math.pi / 2, "sxyz")))
        self.locations["p26"] = Pose(Point(4.730, 2.900, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi, "sxyz")))
        self.locations["p27"] = Pose(Point(4.000, 2.900, 0),
                                     Quaternion(*quaternion_from_euler(0, 0, math.pi / 2, "sxyz")))
        self.locations["p28"] = Pose(Point(4.000, 3.550, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi, "sxyz")))
        self.locations["p29"] = Pose(Point(3.255, 3.550, 0),
                                     Quaternion(*quaternion_from_euler(0, 0, -math.pi / 2, "sxyz")))
        self.locations["p30"] = Pose(Point(3.255, 2.900, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi, "sxyz")))
        self.locations["p31"] = Pose(Point(2.100, 2.900, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi, "sxyz")))

        # publisher for control manual override (in the quite likely event of erratic behavior)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)

        # Subscribe to move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Tf listener for getting the robots current pose
        self.tl = tf.TransformListener()
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_current_pose)

        # Shutdown on shutdown signal
        rospy.on_shutdown(self.shutdown)

        # Log what we're doing
        rospy.loginfo("Waiting for move_base action server...")

        # Delay for 60 seconds to allow the action server to come up
        self.move_base.wait_for_server(rospy.Duration(60))

        # Log that we (hopefully) connected to the action server
        rospy.loginfo("Connected to move_base action server!")

        # Store the initial pose set by user in RViz
        initial_pose = PoseWithCovarianceStamped()

        # Variables to keep track of the distance traveled this run
        self.distance = 0
        self.target_distance = 0
        self.current_location = Pose()
        self.last_location = Pose()
        self.start_time = rospy.Time.now()
        self.goal = MoveBaseGoal()

        # Get the initial pose from RViz
        rospy.loginfo("Waiting for AMCL pose...")
        rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
        last_location = self.current_location
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.update_initial_pose)

        # Do we have the inital pose?
        while initial_pose.header.stamp == "":
            rospy.sleep(1)

        # Inform user we have recieved the pose and are ready for commands
        rospy.loginfo("AMCL Pose recieved. Awaiting input...")
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose = self.locations["p0"]
        self.goal.target_pose.header.stamp = rospy.Time().now()

        # Begin the main loop
        while not rospy.is_shutdown():

            # Prompt user for input
            command = raw_input(
                "Distance: %.2f m\nr = reset distance\n'X' in cm = move forward 'X' cm\nInput: " % (self.distance))

            if command == 'r':
                self.distance = 0
                self.last_location = self.current_location
            elif (command in self.locations):
                self.sendGoal(command)
            elif (command == "run maze"):
                self.run_maze()
            else:
                print(command)
                m = re.match(r"\d+|-\d+", command)
                self.target_distance = (float(m.group(0)) / 100)
                self.compute_distance_components()
                self.sendGoal("forward")

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
        self.locations["forward"] = initial_pose.pose

    def update_current_pose(self, cur_pose):
        self.cur_location = cur_pose.pose.pose
        self.locations["forward"] = cur_pose.pose.pose

    def compute_distance_components(self):
        rot_angle = euler_from_quaternion(
            [self.cur_location.orientation.x, self.cur_location.orientation.y, self.cur_location.orientation.z,
             self.cur_location.orientation.w])[2]
        x_comp = self.target_distance * math.cos(rot_angle)
        y_comp = self.target_distance * math.sin(rot_angle)
        self.locations["forward"].position.x += x_comp
        self.locations["forward"].position.y += y_comp
        self.locations["forward"].orientation = self.cur_location.orientation

    def sendGoal(self, location):

        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose = self.locations[location]
        self.goal.target_pose.pose.position.x
        self.goal.target_pose.pose.position.y
        self.goal.target_pose.header.stamp = rospy.Time().now()
        # Send the goal!
        self.move_base.send_goal(self.goal)
        self.last_location = self.goal

    def run_maze(self):
        startTime = rospy.Time().now()
        for x in range(1, 32):
            print "Sending robot to p%d" % (x)
            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.pose = self.locations["p%d" % (x)]
            self.goal.target_pose.header.stamp = rospy.Time().now()
            # Send the goal and wait for the robot to get there
            self.move_base.send_goal_and_wait(self.goal)
            self.time_between_points.append(((rospy.Time.now() - startTime).secs - (self.goal.target_pose.header.stamp - startTime).secs))
            self.last_location = self.goal
        endTime = rospy.Time().now() - startTime
        print ("Runtime was %i %i" % (endTime.secs, endTime.nsecs))

        for x in range(0, len(self.time_between_points)):
            print "time @ {}: {}".format(x, self.time_between_points[x])

    def shutdown(self):
        rospy.loginfo("Stopping the robot and shuting down the node...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


def trunc(f, n):
    # Truncates or pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])


if __name__ == '__main__':
    try:
        RandomMazeRun()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutdown complete")
