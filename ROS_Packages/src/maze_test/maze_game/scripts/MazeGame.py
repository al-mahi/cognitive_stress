#!/usr/bin/env python

import rospy
import roslib
#roslib.load_manifest('maze_game')
from maze_game.srv import *
import GameModel
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseActionGoal
from visualization_msgs.msg import Marker
import math

PACKAGE = 'maze_game'


class MazeGame:

    def __init__(self):

        self.start_robots = []

        rospy.loginfo("Starting new maze_game node...")
        rospy.loginfo("I'm not C++, I'm PYTHON! YAY!")

        self.addRobotService = rospy.Service("add_robot", AddRobot, self.addRobot)
        self.removeRobotService = rospy.Service("remove_robot", RemoveRobot, self.removeRobot)
        self.startRobotService = rospy.Service("start_robot", StartRobot, self.startRobot)
        # restartRobotService = rospy.Service("restart_robot", )
        self.requesetMazePoints = rospy.Service("request_maze_points", RequestMazeMarkers, self.requestMazeMarkers)

        # Score Averaging Services
        self.requestAverageScores  = rospy.Service("request_average_scores", RequestAverageScores, self.requestAverageScores)
        self.submitScore  = rospy.Service("submit_score", SubmitScores, self.submitScores)
        self.exportRunData  = rospy.Service("export_run_data", ExportRunData, self.exportRunData)

        #See if there are any roboealthPoints, std::vector<geometry_msgs::Point> givenPoints, std::vector<geometry_msgs::Point> expectedPoints, std::vector<ros::Time> inputDelay>) {ts to initialize with
        start_robots = []
        if rospy.has_param("start_robots"):

            rospy.get_param("start_robots", self.start_robots)

            for i in range(len(start_robots)):
                self.model.addRobot(start_robots[i])

        rospy.loginfo("Checking Robots...")

        self.model = GameModel.make_model()

        while not rospy.is_shutdown():
            # rospy.loginfo("MazeGame Spinning.")
            self.model.checkRobotsPosition()
            rospy.sleep(1.0)
            rospy.spin()

    def addRobot(self, request):

        self.model.addRobot(request.robot_name)
        return AddRobotResponse("True")

    def removeRobot(self, request):

        self.model.removeRobot(request.robot_name)
        return RemoveRobotResponse("True")

    def startRobot(self, request):
        self.model.startRobot(request.robot_name)
        return StartRobotResponse("True")

    # def restartRobot(std_msgs::String &req, std_msgs::String &res):
    #
    # if (!request.data.empty()) {
    #   self.model.restartRobot(request.data)
    #   response.data = "True"
    # else {
    # ROS_WARN("No name was given for robot to restart!")
    # response.data = "false"
    #
    #
    # return True
    #

    def requestMazeMarkers(self, request):
        self.model.publishMazeMarkers()
        return RequestMazeMarkersResponse("True")

    def requestAverageScores(self, request):

        avgScores = self.model.getAverageScores()
        response =RequestAverageScoresResponse()
        response.avgRuntime = avgScores[0]
        response.avgDamage = avgScores[1]
        return response

    def submitScores(self, request):
        # Comented out until service type has been updated. This may also be removed as it seems unnecessary
        # self.model.submitAndUpdateAverageScores(request.robot_name, request.goal_arival_times, request.damage_at_goal)
        return True

    def exportRunData(self, request):
        rospy.loginfo("Calling data export from MazeGame!")
        return ExportRunDataResponse(self.model.exportRunData(request.robotName))

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('maze_game')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node = MazeGame()
    except rospy.ROSInterruptException: pass