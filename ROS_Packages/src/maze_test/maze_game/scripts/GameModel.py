#!/usr/bin/env python

import rospkg
import Orange
from Orange.data import *
from Orange.feature import *
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
import actionlib

import RunData
import Robot
from Robot import RobotState


class GameModel:

    def __init__(self):

        self.robots = {} # Holds the Robot objects for the robots we are representing in the game keyed by their name
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10) # Publishes the maze as a Visualization Marker message, used primarily by rosbridge
        self.maze_points = Marker()

        # Data recording
        self.averageRunTimesVector = []# Holds the average times
        self.averageRunTime = rospy.Duration(0)
        self.averageHealthVector = []
        self.averageHealth = 0
        self.expectedPoints = [] # The points read using readPoints. They are used in the disparity calculation.

        self.readPoints()

        rospy.loginfo("GameModel ready!")

        self.runDataVector = [] #Stores all the RunData objects since the last export.

    def readPoints(self):
        print "Reading the points...\n"

        points_arr = []
        # path = rospkg.get_ros_package_path("maze_game") + "/include/points.txt"
        path = "/home/matt/cognitive_stress/ROS_Packages/src/maze_game/include/points.txt"
        print path + "\n"
        infile = open(path)

        for line in infile:

            line = line[:-1]
            split = line.split("  ")
            # Read the coords in as float (they must be space delimited!)
            x = float(split[0])
            y = float(split[1])
            z = float(split[2])

            p = Point

            p.x = x
            p.y = y
            p.z = z

            points_arr.append(p)

        # Set up the visualization markers
        rospy.loginfo("Building visualization markers...")
        self.maze_points.header.frame_id = "/map"
        self.maze_points.header.stamp = rospy.Time.now()
        self.maze_points.ns = "maze_walls"
        self.maze_points.action = Marker.ADD
        self.maze_points.pose.orientation.w = 1.0
        self.maze_points.id = 0
        self.maze_points.type = Marker.LINE_STRIP
        self.maze_points.scale.x = 0.05
        self.maze_points.scale.y = 0.05
        self.maze_points.color.r = 1.0
        self.maze_points.color.a = 1.0

        for i in range(len(points_arr)):
            self.expectedPoints.append(points_arr[i])
            self.maze_points.points.append(points_arr[i])

            if i < len(points_arr) - 1:
                self.maze_points.points.append(points_arr[i + 1])

    def addRobot(self, robotName):
        print "Adding robot: " + robotName + "\n"
        # Commented out for testing without creating new robot objects, DO NOT DELETE!

        toAdd = Robot.make_robot(robotName, self)
        self.robots.update(robotName, toAdd)

        # List robots
        i = 0
        print "---------- Robots List ----------\n"
        for key in self.robots:
            print("%i) %s\n", i, key)
            i += 1

    def removeRobot(self, robotName):
        print("Removing robot %s\n", robotName)

        self.robots.__delitem__(robotName)

        # List robots
        j = 0
        print "---------- Robots List ----------\n"
        for key in self.robots:
            print("%i) %s\n", j, key)
            j += 1

    def startRobot(self, robotName):
        ac = actionlib.SimpleActionClient("/" + robotName + "/move_base", MoveBaseAction)

        while not ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for move base action server...")

        self.robots[robotName].setState(RobotState.MOVING_TO_START)

        msg = MoveBaseGoal()

        msg.target_pose.header.frame_id = "map"
        msg.target_pose.header.stamp = rospy.Time.now()

        msg.target_pose.pose.position.x = 6.730
        msg.target_pose.pose.position.y =  6.0
        msg.target_pose.pose.position.z  = 0
        msg.target_pose.pose.orientation.x = 0
        msg.target_pose.pose.orientation.y  = 0
        msg.target_pose.pose.orientation.z = .993607
        msg.target_pose.pose.orientation.w = -0.112894

        rospy.loginfo("Sending %s to start of the maze...", robotName)
        ac.send_goal(msg)
        ac.stop_tracking_goal()

    def restartRobot(self, robot_name):
        # Check for the robots existence

        if self.robots.has_key(robot_name):
            self.robots[robot_name].resetRobot()
            rospy.loginfo("Successfully reset %s", robot_name)
        else:
            rospy.logwarn("No robot names %s to reset. ", robot_name)

    def checkRobotsPosition(self):

    # rospy.loginfo("Checking robots from GameModel.")

        for key, value in self.robots.iteritems():

        # self is not executed, seems to crash while adding a new robot...
        # rospy.loginfo("Checking robot %s", entry.first)

            if value.getCurState() == RobotState.MOVING_TO_START or value.getCurState() == RobotState.RUNNING_MAZE:
                # rospy.loginfo("%s has started, checking distance.", entry.first)
                value.checkDistance()

    def getExpPointAt(self, i):
        return self.expectedPoints[i]

    def getAverageScores(self):
        return [self.averageRunTime, self.averageHealth]

    def submitAndUpdateAverageScores(self, name, times, healthPoints, givenPoints, expectedPoints, inputDelay, trustScores):
        self.runDataVector.append(RunData.make_run_data(name, False, times, healthPoints, givenPoints, expectedPoints, inputDelay, trustScores))

        # Compute average duration
        self.averageRunTimesVector.append(rospy.Duration(times[-1:] - times[0]))
        self.averageRunTime = rospy.Duration((self.averageRunTime + self.averageRunTimesVector[-1:]).toSec() / len(self.averageRunTimesVector))

        # Compute average health
        self.averageHealthVector.append(healthPoints[-1:] - healthPoints.front())

    def exportRunData(self, robotName):

        # Vector to contain the robot(s) that we are asked to export data for
        toExport = []

        rospy.loginfo("Exporting data for %s...", robotName)

        # Check to see what we need to export
        if not robotName == "ALL":

            # Check for the robots existence (I realize this is redundant).

            if not len(self.runDataVector) == 0:
                for dat in self.runDataVector:
                    if dat.robotName == robotName:
                        toExport.append(dat)
        else:
            toExport = self.runDataVector

        for entry in toExport:

            path = rospkg.get_ros_package_path("maze_game")
            path += "/export/" + entry.timestamp + "_" + entry.robotName + ".tab"

            # Normalize the trust values

            maxTrust = max(entry.trustScores)
            minTrust = min(entry.trustScores)

            for i in range(len(entry.trustScores)):
                entry.trustScores[i] = 1 - (entry.trustScores[i] - minTrust) / (maxTrust - minTrust)

            Name, RunTime, RunBefore, GivenX, GivenY, ExpectedX, ExpectedY, Disparity, DamagePercent, TimeDelay, TrustValue = [Orange.feature.Discreate(x) for x in ['name', 'runtime', 'run before', 'given x', 'given y', 'expected x', 'expected y', 'disparity', 'percent damage', 'time delay', 'trust value']]
            dom = Domain([Name, RunTime, RunBefore, GivenX, GivenY, ExpectedX, ExpectedY, Disparity, DamagePercent, TimeDelay, TrustValue])
            data = Table(dom, [entry.robotName, entry.runMazeBefore, [p.x for p in entry.givenPoints], [p.y for p in entry.givenPoints], [p.x for p in entry.expectedPoints], [p.y for p in entry.expectedPoints], entry.disparity, entry.healthAtGoal, entry.inputDelay, entry.trustScores])

            print data

            data.save(path)

        # Clear the runData we just exported?
        return True

    def publishMazeMarkers(self):
        self.marker_pub.publish(self.maze_points)

    def getRobot(self, name):
        return self.robots[name]

def make_model():
    return GameModel()