#!/usr/bin/env python
import rospy

class RunData(object):

    timestamp = rospy.Time(0)
    robotName = ""
    runMazeBefore = False
    goalArrivalTimes = []
    healthAtGoal = []
    givenPoints = []
    expectedPoints = []
    inputDelay = []
    trustScores =[]

    def __init__(self, robotName, runMazeBefore, goalArrivalTimes, healthAtGoal, givenPoints, expectedPoints, inputDelay, trustScores):
        self.timestamp = rospy.Time.now() # Time stamp indicating when the data was saved by the robot
        self.robotName = robotName # Name of the robot that submitted the data
        self.runMazeBefore = runMazeBefore # Flag indicating if the user has run the maze before
        self.goalArrivalTimes = goalArrivalTimes# Time stamps for when the robot arrived a particular point, indexed by point
        self.healthAtGoal = healthAtGoal # The health a robot had when it arrived a particular point in the maze, indexed by point
        self.givenPoints = givenPoints# Keeps track of the points that are given by the user
        self.expectedPoints = expectedPoints# Keeps track of the points that are expected by the robot
        self.inputDelay = inputDelay
        self.trustScores = trustScores # Vector of the unnormalized trust scores

def make_run_data(robotName, runMazeBefore, goalArrivalTimes, healthAtGoal, givenPoints, expectedPoints, inputDelay, trustScores):
    return RunData(timestamp, robotName, runMazeBefore, goalArrivalTimes, healthAtGoal, givenPoints, expectedPoints, inputDelay, trustScores)