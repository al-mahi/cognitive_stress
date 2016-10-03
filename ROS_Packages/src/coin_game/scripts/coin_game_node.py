#!/usr/bin/env python

from enum import Enum
import random
import math
import rospy
import roslib
from roslib import packages
from visualization_msgs.msg import *
import actionlib
from actionlib_msgs.msg import *
from std_srvs.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
from coin_game_msgs.srv import *

__author__ = 'matt'


class SegmentDirection(Enum):
    VERTICAL = 0
    HORIZONTAL = 1
    TOP_TO_BOTTOM = 2
    BOTTOM_TO_TOP = 3
    LEFT_TO_RIGHT = 4
    RIGHT_TO_LEFT = 5


class CoinGameNode():

    def __init__(self):
        self.current_coins = {}
        self.coin_publisher = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

        self.robotNodes = []
        self.maze_path_points = []
        self.read_points()

        add_robot_service = rospy.Service("add_robot", RobotNamed, self.add_robot)
        remove_robot_service = rospy.Service("remove_robot", RobotNamed, self.remove_robot)
        start_robot_service = rospy.Service("start_robot", RobotNamed, self.start_robots)
        get_points_for_indices_service = rospy.Service("points_for_indices", PathPointsForIndices, self.path_points_for_indices)
        spawn_new_coin_service = rospy.Service("spawn_new_coin", SpawnNewCoin, self.spawn_new_coin)
        remove_coin_service = rospy.Service("remove_coin", RobotNamed, self.remove_coin)
        start_game_service = rospy.Service("start_game", Empty, self.start_game)

        self.rgb = {
            "miranda": [1.0, 0.0, 0.0],
            "trinculo": [0.541176471, 0.168627451, 0.88627451],
            "caliban": [1.0, 0.5, 0.0],
            "ferdinand": [0.0, 1.0, 0.0],
            "prospero": [0.3, 0.3, 1.0],
            "ariel": [1.0, 0.843137255, 0.0]
        }

        self.robot_start_points = {"trinculo": Pose(Point(6.73, 6.0, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi))),
                                   "miranda": Pose(Point(6.73, 2.8, 0), Quaternion(*quaternion_from_euler(0, 0, math.pi))),
                                   "ferdinand": Pose(Point(2.1, 6.0, 0), Quaternion(*quaternion_from_euler(0, 0, 0))),
                                   "prospero": Pose(Point(2.1, 2.8, 0), Quaternion(*quaternion_from_euler(0, 0, 0)))
                                   }

        self.start_point_vacant = [True, True, True, True]

        rospy.on_shutdown(self.clean_up)

        # Start the robots
        if rospy.has_param("~number_of_robots"):
            self.number_of_robots = rospy.get_param("~number_of_robots")
        else:
            self.number_of_robots = 1

        self.countdown_timer = rospy.Timer
        self.countdown_marker = Marker()
        self.game_start_countdown = 5

        while not rospy.is_shutdown():
            rospy.spin()

    def add_robot(self, req):
        self.robotNodes.append(req.robotName)
        start_req = RobotNamedRequest()
        start_req.robotName = req.robotName
        self.start_robots(start_req)

        return RobotNamedResponse("Done")

    def remove_robot(self, req):
        if self.robotNodes.count(req.robotName) != 0:
            self.robotNodes.remove(req.robotName)
        else:
            rospy.logwarn("No robot by the name " + req.robotName + " to remove.")

    def start_robots(self, req):
        if self.robotNodes.count(req.robotName) == 0:
            rospy.logwarn("No robot by the name " + req.robotName + " to start.")
            return RobotNamedResponse("ERROR!")
        print "Debug[M] @coin_game_node robot name: ", req.robotName
        move_base = actionlib.SimpleActionClient("/" + req.robotName + "/move_base", MoveBaseAction)

        while not move_base.wait_for_server(rospy.Duration(5)):
            print "Waiting for action server..."

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        while True:
            # test_point = self.start_point_vacant[random.randint(0, 3)]
            test_point = random.randint(0, 3)
            print "Debug[M] @coin_game_node: Searching for vacant point... ", req.robotName, "try ", test_point, " vacancy ", self.start_point_vacant
            if self.start_point_vacant[test_point]:
                self.start_point_vacant[test_point] = False
                # goal.target_pose.pose = self.robot_start_points[test_point]
                break

        goal.target_pose.pose = self.robot_start_points[req.robotName]

        rospy.loginfo("Sending " + req.robotName + " to the start of the maze!")
        move_base.send_goal(goal)
        move_base.stop_tracking_goal()
        return RobotNamedResponse("Done")

    def start_game(self, req):
        try:
            for robot in self.robotNodes:
                rospy.wait_for_service("/" + robot + "/start_timer", 10)
                call_start_timer = rospy.ServiceProxy("/" + robot + "/start_timer", Empty)
                call_start_timer.call(EmptyRequest())
        except rospy.ServiceException, e:
            print "Service call to start_timer failed: {}".format(e)

        return EmptyResponse()

    def begin_countdown(self, req):

        self.countdown_marker.header.frame_id = "map"
        self.countdown_marker.header.stamp = rospy.Time.now()
        self.countdown_marker.ns = "countdown"
        self.countdown_marker.id = random.randint
        self.countdown_marker.type = Marker.TEXT_VIEW_FACING
        self.countdown_marker.text = "5"
        self.countdown_marker.action = Marker.ADD
        self.countdown_marker.pose.position.x = 4.3
        self.countdown_marker.pose.position.y = 4.3
        self.countdown_marker.pose.position.z = 3.0
        self.countdown_marker.pose.orientation.x = 0.0
        self.countdown_marker.pose.orientation.y = 0.0
        self.countdown_marker.pose.orientation.z = 0.0
        self.countdown_marker.pose.orientation.w = 1.0
        self.countdown_marker.scale.x = 1.0
        self.countdown_marker.scale.y = 1.0
        self.countdown_marker.scale.z = 5.0
        self.countdown_marker.color.a = 1.0
        self.countdown_marker.color.r = 1.0
        self.countdown_marker.color.g = 1.0
        self.countdown_marker.color.b = 1.0
        self.coin_publisher.publish(self.countdown_marker)

        self.countdown_timer = rospy.Timer(rospy.Rate(1), self.countdown)

    def countdown(self, event):

        if self.game_start_countdown >= 0:
            self.game_start_countdown -= 1
            self.countdown_marker.text = str(self.game_start_countdown)
            self.countdown_marker.action = Marker.MODIFY
        else:
            self.countdown_timer.shutdown()
            self.game_start_countdown = 0
            self.countdown_marker.text = str(self.game_start_countdown)
            self.countdown_marker.action = Marker.DELETE

        self.coin_publisher.publish(self.countdown_marker)

    def read_points(self):
        points_file = open(roslib.packages.get_pkg_subdir("coin_game", "include") + "/points.txt")

        print "Reading points file..."
        i = 0

        for line in points_file:
            split = line.split("  ")
            #print split

            to_add = Point()

            to_add.x = float(split[0])
            to_add.y = float(split[1])
            to_add.z = float(split[2])

            self.maze_path_points.append(to_add)
            i += 1

        print "Done! Read {} points.".format(i)

    def path_points_for_indices(self, req):

        if req.index_a >= len(self.maze_path_points):
            req.index_a = len(self.maze_path_points) - 1
        elif req.index_a < 0:
            req.index_a = 0

        if req.index_b >= len(self.maze_path_points):
            req.index_b = len(self.maze_path_points) - 1
        elif req.index_b < 0:
            req.index_b = 0

        resp = PathPointsForIndicesResponse()
        resp.point_a = self.maze_path_points[req.index_a]
        resp.point_b = self.maze_path_points[req.index_b]

        return resp

    def spawn_new_coin(self, req):

        spawn_location = Point()

        if req.index_a >= len(self.maze_path_points):
            req.index_a = len(self.maze_path_points) - 1
        elif req.index_a < 0:
            req.index_a = 0

        if req.index_b >= len(self.maze_path_points):
            req.index_b = len(self.maze_path_points) - 1
        elif req.index_b < 0:
            req.index_b = 0

        spawn_segment_index_a = random.randrange(0, len(self.maze_path_points) - 1, 1)
        spawn_segment_index_b = spawn_segment_index_a + 1

        orientation = self.get_path_segment_orientation(spawn_segment_index_a, spawn_segment_index_b)

        if orientation[0] == SegmentDirection.HORIZONTAL:

            spawn_location.y = self.maze_path_points[spawn_segment_index_a].y

            if orientation[1] == SegmentDirection.LEFT_TO_RIGHT:
                spawn_location.x = random.uniform(self.maze_path_points[spawn_segment_index_a].x, self.maze_path_points[spawn_segment_index_b].x)
            else:
                spawn_location.x = random.uniform(self.maze_path_points[spawn_segment_index_b].x, self.maze_path_points[spawn_segment_index_a].x)
        else:

            spawn_location.x = self.maze_path_points[spawn_segment_index_a].x

            if orientation[1] == SegmentDirection.TOP_TO_BOTTOM:
                spawn_location.y = random.uniform(self.maze_path_points[spawn_segment_index_a].y, self.maze_path_points[spawn_segment_index_b].y)
            else:
                spawn_location.y = random.uniform(self.maze_path_points[spawn_segment_index_b].y, self.maze_path_points[spawn_segment_index_a].y)

        self.add_new_coin_marker(spawn_location, req.robot_name)

        return SpawnNewCoinResponse(spawn_location)

    # Determines if a path segment defined by two points is vertical or horizontal
    def get_path_segment_orientation(self, index_a, index_b):

        toReturn = []

        point_a = self.maze_path_points[index_a]
        point_b = self.maze_path_points[index_b]

        if point_a.x == point_b.x:
            toReturn.append(SegmentDirection.VERTICAL)

            if point_a.y > point_b.y:
                toReturn.append(SegmentDirection.TOP_TO_BOTTOM)
            else:
                toReturn.append(SegmentDirection.BOTTOM_TO_TOP)

        else:
            toReturn.append(SegmentDirection.HORIZONTAL)

            if point_a.x > point_b.x:
                toReturn.append(SegmentDirection.RIGHT_TO_LEFT)
            else:
                toReturn.append(SegmentDirection.LEFT_TO_RIGHT)

        return toReturn

    def add_new_coin_marker(self, location, robot_name):

        rgb = self.rgb.get(robot_name)

        coin_marker = Marker()

        coin_marker.header.frame_id = "map"
        coin_marker.header.stamp = rospy.Time.now()
        coin_marker.ns = robot_name
        coin_marker.id = len(self.current_coins)
        coin_marker.type = Marker.CYLINDER
        coin_marker.action = Marker.ADD
        coin_marker.pose.position = location
        coin_marker.pose.position.z = .1
        coin_marker.pose.orientation.x = 0.0
        coin_marker.pose.orientation.y = 0.0
        coin_marker.pose.orientation.z = 0.0
        coin_marker.pose.orientation.w = 1.0
        coin_marker.scale.x = .2
        coin_marker.scale.y = .2
        coin_marker.scale.z = .01
        coin_marker.color.a = 1.0
        coin_marker.color.r = rgb[0]
        coin_marker.color.g = rgb[1]
        coin_marker.color.b = rgb[2]
        self.coin_publisher.publish(coin_marker)
        self.current_coins.update({robot_name: coin_marker})

    def remove_coin(self, req):
        coin_marker = self.current_coins.get(req.robotName)
        coin_marker.action = Marker.DELETE
        self.coin_publisher.publish(coin_marker)
        return RobotNamedResponse("Done")

    def clean_up(self):
        for key in self.current_coins.keys():
            self.remove_coin(RobotNamedRequest(key))

if __name__ == "__main__":
    rospy.init_node("coin_game_node")
    try:
        cgn = CoinGameNode()
    except rospy.ROSInterruptException:
        pass
