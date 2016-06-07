#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionGoal
from tf import TransformListener
from tf.transformations import quaternion_from_euler
import math
from std_srvs.srv import Empty
from maze_game.srv import *
import numpy as np
import tf
import sys

# /*! \relates Robot
#  * Enum Indicated the robots current state to help facilitate transitions from one operation to the next.
# */
class RobotState:
    IDLE, MOVING_TO_START, RUNNING_MAZE, FINISHED_MAZE = range(4)
#/< Used to indicate that the robot is not doing anything and is ready to be re-tasked
#/< Indicates that the robot is moving to the start of the maze
# #/< Indicates that the robot is running the maze and should not be interrupted
# #/< Indicates that the robot has finished the maze. This is used primarily for score logging and a future re-queue mechanism

# class GameModel:
# /**
#*\author Matthew Atkins
#
#*This class is used to represent an individual robot in the current game.
#*The object interacts with ROS directly to update itself and should inform the MazeGame object
#*associated with this game about relevant events such as taking "damage". It does provide topics for
#*sending information, probably to rosbridge, about itself for things like updating the webUI.
#
#*The class is designed to support multiple robots using multiple Robot object instances. This class IS NOT
#*designed to operate as an independent ROS node a robot should be created with service call a running maze_game
#*node using the addRobot service. This service call requires a string robot_name that will be used to uniquely
#*identify a given robot. Each will be given their own ROS namespace to operate in to avoid accidentally updating
#*information incorrectly. The namespace is determine by the value of robot_name from the service call.
#
#*For example, to use two robots at the same time, robot1 and robot2 respectively, then the following namespaces
#*will be created:
#
#*robot1 -> robot1/move_base/goal<br>
#*robot2 -> robot2/move_base/goal
#
#*If only one robot is to be used, a service call to "/maze_game/add_robot" with robot_name set to "" will result in no namespace being
#*prepended.
#*

class Robot:
    def __init__(self, robotName, modelPtr):
        #Perform additional setup
        self.name = robotName
        self.model = modelPtr
        rospy.loginfo("subscribing to /%s/move_base/current_goal\n", self.name)
        # MYSTERIOUS LINE
        self.sub = rospy.subscribe("/" + self.name + "/move_base/current_goal", 10, self.newGoalCallback, self.pointDex)
        self.pointDex = 0
        self.health = 100000
        #Distance from centerline (walls) variables
        self.distance = 0
        self.oldDistance = 0
        self.curGoal = self.model.getExpPointAt(0)
        self.curState = RobotState.IDLE
        self.started = False
        # MYSTERIOUS LINE
        self.addRobotService = rospy.advertiseService(self, "/maze_game/submit_scores_" + self.name, self.submitScores)
        #Check for parameters
        self.goal_tolerance = 0.2
        self.curState = RobotState.MOVING_TO_START #/< The robot's current state, initially set to IDLE.
        self.distance = np.inf #/< The robot's distance from the center of the maze's path
        self.oldDistance = np.inf #/< The previously observed distance from the center of the maze's path
        self.health = 1000 #/< Used to determine how 'good' a run through the maze was and approximately how long a robot was in contact with an obstacle.
        self.curGoal = None #/< The current goal used to calculate which path segment the distance calculation should be performed with respect to. Also used to compute the disparity between the robot's own notion of what is a 'good' goal and what the user actually give it.
        self.model = None #/< A pointer the GameModel to add this Robot instance to
        self.arivedAtLastGoal = None
        self.goal_tolerance = 0.2 #/< This is the value that check distance uses to see if the robot is within the target "zone" around it's current goal (stored in curGoal). It's important to know that this is the RADUIUS and not the diameter of the target zone.

        #Score recording and submission
        self.givenPoints = [] #/< Keeps track of the points that are given by the user
        self.expectedPoints = [] #/< Keeps track of the points that are expected by the robot
        self.goalArivalTimestamps = [] #/< Keeps track of when a Robot arrives at a certain goal/point in the maze. First point should be the time the maze was started, last should be the completion time.
        self.healthAtGoal = [] #/< Keeps track of what the robot's health was as it arrives at every goal.
        self.inputDelay = [] #/< Stores the time in seconds between when the robot arrives at a goal and when the user sends the next navigation goal.
        self.trustScores = [] #/< The UN-normalized trust scores for each input
        if rospy.hasParam(robotName + "/goal_tolerance"):
            rospy.loginfo("%s had parameter 'goal_tolerance' in public node handle", self.name)
            rospy.getParam(self.name + "/goal_tolerance", self.goal_tolerance)
        elif rospy.hasParam("goal_tolerance"):
            rospy.loginfo("%s had parameter 'goal_tolerance' in private handle", self.name)
            rospy.getParam("goal_tolerance", self.goal_tolerance)
        else:
            rospy.loginfo("%s did not have a parameter name goal_tolerance! Defaulting to .2!", self.name)
            self.goal_tolerance = .2

        self.disparity = []
        self.goalArivalTimestamps = []

    def submitScores(self, req, res):
        self.callSubmitScoresService()
        return True

    def newGoalCallback(self):
        pass

    def resetRobot(self):
        self.started = False
        self.curState = RobotState.IDLE
        self.pointDex = 0
        self.health = 100000
        self.distance = 0.0
        self.oldDistance = 0.0
        self.curGoal = self.model.expectedPoints.at(0)
        self.disparity.x = []
        self.goalArivalTimestamps = []
        self.healthAtGoal = []

    def setState(self, state):
        self.curState = state

    def getCurState(self):
        return self.curState

    def isStarted(self):
        return self.started

    def setGameModel(self, modelVar):
        self.model = modelVar

    def getName(self):
        return self.name

    def addDispPoint(self, point):
        self.disparity.append(point)

    def getAverageDisp(self, returned):
        for i in range(len(self.disparity)):
            returned[i] += self.disparity[i].z
        return returned

    def getStartTime(self):
        if self.curState == RobotState.RUNNING_MAZE:
            if not len(self.goalArivalTimestamps) == 0:
                return rospy.Time(self.goalArivalTimestamps[0])
        else:
            return rospy.Time(-1.0)


    def getRunDuration(self):
        if self.curState == RobotState.RUNNING_MAZE:
            if not self.goalArivalTimestamps == 0:
                return rospy.Duration(self.goalArivalTimestamps[len(self.goalArivalTimestamps) - 1] - self.goalArivalTimestamps[0])
        else:
            return rospy.Duration(-1.0)


    def callback(self, pointRec, timeRec):
        # geometry_msgs::Point toAdd #The disparity point to add to the list of disparity points in self.dispPoints
        target = self.model.getExpPointAt(self.pointDex)
        #Record goal input delay
        #ros::Duration inputDelay = self.arivedAtLastGoal - msg->header.stamp
        delay = rospy.Duration(self.arivedAtLastGoal - timeRec)
        self.inputDelay.append(delay.to_sec())
        rospy.loginfo("Goal input delay was %.6f", rospy.Duration(delay.to_sec()))
        # Add this point to givenPoints and add the expected point to expectePoints
        #   self.givenPoints.append(msg->pose.position)
        self.givenPoints.append(pointRec)
        self.expectedPoints.append(target)
        # std::cout << "exp point from model x: " << exp.x << " y: " << exp.y << " z:" << exp.z << '\n'
        # Store the disparity data in toAdd
        # toAdd.x = fabs(self.curGoal.x - msg->pose.position.x)
        # toAdd.y = fabs(self.curGoal.y - msg->pose.position.y)
        toAdd = Point()
        toAdd.x = abs(target.x - pointRec.x)
        toAdd.y = abs(target.y - pointRec.y)
        toAdd.z = abs(pow(toAdd.x, 2.0) + pow(toAdd.y, 2.0))
        #Perform evaluation of the user's input using the "trust" function
        trustValue = (toAdd.z + ((10000.0 - self.health)/ 10000.0) - (10.0 * delay.toSec()))
        self.trustScores.append(trustValue)
        rospy.loginfo("Calculated trust: %.6f\n\td = %.6f\n\th = %6.f\n\ttd = %.6f", trustValue, toAdd.z, ((10000.0 - self.health)/ 10000.0), (10.0 * delay.toSec()))
        # rospy.loginfo("Expected (%.6f, %.6f) got (%.6f, %.6f)", self.curGoal.x, self.curGoal.y, msg->pose.position.x, msg->pose.position.y)
        rospy.loginfo("Expected (%.6f, %.6f) got (%.6f, %.6f)", target.x, target.y, pointRec.x, pointRec.y)
        rospy.loginfo("Calculated Disparity: x: %.6f, y: %.6f, total: %.6f\n", toAdd.x, toAdd.y, toAdd.z)
        self.addDispPoint(toAdd)
        avgDisp = 0.
        for i in range(len(self.disparity)):
            avgDisp += self.disparity[i].z
        rospy.loginfo("Average disparity so far: %.6f\n", avgDisp)

    def getDistanceToMazeWalls(self, i, p3transform):
        p1 = Point()
        p2 = Point()
        p3 = Point()
        # distance, numerator, denominator
        if i <= 0:
            i = 1
        elif i >= len(self.model.expectedPoints):
            i = len(self.model.expectedPoints) - 1

            p1 = self.model.getExpPointAt(i - 1)
            p2 = self.model.getExpPointAt(i)

            p3.x = p3transform.pose.position.x
            p3.y = p3transform.pose.position.y
            p3.z = p3transform.pose.position.z

            #ensure that we are able to compute the distance to the line segment
            if p1.x == p2.x:
                if p3.y < min(p1.y, p2.y) or p3.y > max(p1.y, p2.y):
                    return -1.0
        else:
            if p3.x < min(p1.x, p2.x) or p3.x > max(p1.x, p2.x):
                return -1.0

                #Formulas
                # distance = |(y2 - y1)p3.x - (x2 - x1)p3.y + x2*y1 - x1*y2| / sqrt( (y2 - y1)^2 + (x2 - y1)^2)
                #distance = ( fabs( (p2.y - p1.y) * p3.getOrigin().x() - (p2.x - p1.x) * p3.getOrigin().y() + p2.x*p1.y - p2.y*p1.x ) / sqrt( pow(p2.y - p1.y, 2.0) + pow(p2.x - p1.x, 2.0) ) )

        numerator = abs( (p2.y - p1.y) * p3.x - (p2.x - p1.x) * p3.y + p2.x*p1.y - p2.y*p1.x)
        denominator = math.sqrt(pow(p2.y - p1.y, 2.0) + pow(p2.x - p1.x, 2.0) )
        distance = numerator / denominator
        return distance

    def checkDistance(self):

        #See if the Robot is in the IDLE state, and do nothing if it is
        if self.curState == RobotState.IDLE:
            return

        # geometry_msgs::PoseStamped pBase, pMap
        pBase = PoseStamped()
        pMap = PoseStamped()
        pBase.header.frame_id = self.name + "/base_link"
        pBase.pose.position.x = 0.0
        pBase.pose.position.y = 0.0
        pBase.pose.orientation = Quaternion(*quaternion_from_euler(0., 0., 0., "sxyz"))# createQuaternionMsgFromYaw(0.0)

        # tf::TransformListener tfListener
        current_transform = rospy.Time.now()

        try:
            TransformListener.waitForTransform(pBase.header.frame_id, "/map", rospy.Time(), rospy.Duration(10.0))
            TransformListener.getLatestCommonTime(pBase.header.frame_id, "/map", current_transform, None)
            pBase.header.stamp = current_transform
            #transform the robot's pose, pBase, into the map frame, pMap. pMap will contain these coordinates
            #according to the TF data available at time "current_transform"
            TransformListener.transformPose("/map", pBase, pMap)
        except tf.Exception as e:
            rospy.logerr("%s", e.what())
            rospy.Duration(1.0).sleep()

        distance = self.getDistanceToMazeWalls(self.pointDex, pMap)

        if distance >= self.oldDistance:
            self.oldDistance = distance

        # Only perform the distance pentalty check if the robot is actually running the maze
        if distance > self.goal_tolerance and self.curState == RobotState.RUNNING_MAZE:
            self.health -= 1
            rospy.loginfo("%s: Ouch!\n\tHealth: %i\n\tDistance: %.6f\n", self.name,self.health, distance)

        # See if we have reached the next goal
        rospy.loginfo("%s is %.6f from goal x: %.3f, y: %.3f, z: %.3f", self.name, math.sqrt( pow(self.curGoal.x - pMap.pose.position.x, 2.0) + pow(self.curGoal.y - pMap.pose.position.y, 2.0) ), self.curGoal.x, self.curGoal.y, self.curGoal.z)

        if math.sqrt(pow(self.curGoal.x - pMap.pose.position.x, 2.0) + pow(self.curGoal.y - pMap.pose.position.y, 2.0)) <= self.goal_tolerance:
            self.arivedAtLastGoal = rospy.Time.now()

            rospy.loginfo("%s has reached goal at pointDex = %i", self.name, self.pointDex)
            self.pointDex += 1
            rospy.loginfo("%s's pointDex is now: %i", self.name, self.pointDex)

            #		rospy.loginfo("PointDex: %i\n", pointDex)

            #Have we finished the maze?
            if self.pointDex == len(self.model.expectedPoints):
                rospy.loginfo("%s completed the maze!", self.name)
                self.curState = RobotState.FINISHED_MAZE
                self.started = False

                #Submit our scores
                self.callSubmitScoresService()
                return

            #Get the next goal from the model and save the status information for the goal we have just arrived at
            curGoal = self.model.getExpPointAt(self.pointDex)
            self.oldDistance = 0.
            self.goalArivalTimestamps.append(rospy.Time.now().toSec())
            self.healthAtGoal.append(self.health)

    def callSubmitScoresService(self):
        #	ros::ServiceClient client = self.public_nh.serviceClient<maze_game::SubmitScores>("submit_score")

        #	maze_game::SubmitScores::Request req
        #	req.robot_name = self.name
        #	req.goal_arival_times = self.goalArivalTimestamps
        #	req.damage_at_goal = self.healthAtGoal

        #	maze_game::SubmitScores::Response res

        lengths = [len(self.healthAtGoal), len(self.givenPoints), len(self.expectedPoints), len(self.inputDelay), len(self.trustScores)]

        for i in range(len(lengths)):
            rospy.loginfo("lengths @ %i: %i", i, lengths[i])

        maxLength = max(lengths)
        rospy.loginfo("maxLength: %i", maxLength)


        if len(self.goalArivalTimestamps) < maxLength:
            rospy.loginfo("goalArivalTimeStamps size: %i. Resizing...", len(self.goalArivalTimestamps))
            for i in range(len(self.goalArivalTimestamps) - 1, maxLength, 1):
                self.goalArivalTimestamps.append(-86401.0) #24 * 60 * 60 - 1

        if len(self.healthAtGoal) < maxLength:
            rospy.loginfo("healthAtGoal size: %i. Resizing...", len(self.healthAtGoal))
            for i in range(len(self.healthAtGoal) - 1, maxLength, 1):
                self.healthAtGoal.append(- (sys.maxint-1))

        nullPoint = Point()
        nullPoint.x = -86401.0
        nullPoint.y = -86401.0
        nullPoint.z = -86401.0

        if len(self.givenPoints) < maxLength:
            rospy.loginfo("givenPoints size: %i. Resizing...", len(self.givenPoints))
            for i in range(len(self.givenPints)-1, maxLength, 1):
                self.givenPoints.append(nullPoint)

        if len(self.expectedPoints < maxLength):
            rospy.loginfo("expectedPoints size: %i. Resizing...", len(self.expectedPoints))
            for i in range(len(self.expectedPints)-1, maxLength, 1):
                self.expectedPoints.append(nullPoint)

        if len(self.inputDelay) < maxLength:
            rospy.loginfo("inputDelay size: %i. Resizing...", self.inputDelay.size())
            for i in range(len(self.inputDelay)-1, maxLength, 1):
                self.inputDelay.append(-86401.0)

        if len(self.trustScores) < maxLength:
            rospy.loginfo("trustScores size: %i. Resizing...", self.trustScores.size())
            for i in range(len(self.trustScores)-1, maxLength, 1):
                self.trustScores.append(-86401.0)

        self.model.submitAndUpdateAverageScores(self.name, self.goalArivalTimestamps, self.healthAtGoal, self.givenPoints, self.expectedPoints, self.inputDelay, self.trustScores)

    def newGoalCallback(self, msg, pointDex):
        self.callback(msg.pose.position, msg.header.stamp)
        self.checkDistance()

def make_robot(name, modelPtr):
	return Robot(name, modelPtr)