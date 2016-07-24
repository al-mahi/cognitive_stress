#!/usr/bin/env python

import Orange
import Orange.feature
import Orange.classification
import Orange.data
import Orange.utils
import Orange.wrappers
import pickle
import rospy
import roslib
from roslib import packages
from maze_game.srv import *

__author__ = 'matthew'


class TrustEvaluationServer():

    def __init__(self):

        classifier_file = open(roslib.packages.get_pkg_subdir("coin_game", "include") + "/svm.pck")
        self.SVM = pickle.load(classifier_file)
        classifier_file.close()
        if rospy.has_param("/trust_evaluation_server/min_disparity"):
            self.min_disparity = rospy.get_param("/trust_evaluation_server/min_disparity")
        else:
            self.min_disparity = 0.0

        if rospy.has_param("/trust_evaluation_server/max_disparity"):
            print "Found max_disparity parameter!"
            self.max_disparity = rospy.get_param("/trust_evaluation_server/max_disparity")
        else:
            self.max_disparity = 1.0

        if rospy.has_param("/trust_evaluation_server/min_percent_damage"):
            self.min_percent_damage = rospy.get_param("/trust_evaluation_server/min_percent_damage")
        else:
            self.min_percent_damage = 0.0

        if rospy.has_param("/trust_evaluation_server/max_percent_damage"):
            self.max_percent_damage = rospy.get_param("/trust_evaluation_server/max_percent_damage")
        else:
            self.max_percent_damage = 1.0

        if rospy.has_param("/trust_evaluation_server/min_arrival_delay"):
            self.min_arrival_delay = rospy.get_param("/trust_evaluation_server/min_arrival_delay")
        else:
            self.min_arrival_delay = 0.0

        if rospy.has_param("/trust_evaluation_server/max_arrival_delay"):
            self.max_arrival_delay = rospy.get_param("/trust_evaluation_server/max_arrival_delay")
        else:
            self.max_arrival_delay = 1.0

        self.s = rospy.Service('evaluate_trust', TrustEvaluation, self.evaluate_trust)
        print "Trust Evaluation Server ready..."
        rospy.spin()

    def evaluate_trust(self, req):
        print "Evaluating trust..."
        print "\tDisparity: " + str(req.disparity)
        print "\tPercent Damage: " + str(req.percentDamage)
        print "\tArrival Delay: " + str(req.goalArrivalDelay)

        # check given values and clamp them if they are out of bounds
        if req.disparity > self.max_disparity:
            req.disparity = self.max_disparity
        elif req.disparity < self.min_disparity:
            req.disparity = self.min_disparity

        if req.percentDamage > self.max_percent_damage:
            req.percentDamage = self.max_percent_damage
        elif req.percentDamage < self.min_percent_damage:
            req.percentDamage = self.min_percent_damage

        if req.goalArrivalDelay > self.max_arrival_delay:
            req.goalArrivalDelay = self.max_arrival_delay
        elif req.goalArrivalDelay < self.min_arrival_delay:
            req.goalArrivalDelay = self.min_arrival_delay

        # Normalize input values
        normalized_values = [(req.disparity - self.min_disparity)/(self.max_disparity - self.min_disparity),
                             (req.percentDamage - self.min_percent_damage)/(self.max_percent_damage - self.min_percent_damage),
                             (req.goalArrivalDelay - self.min_arrival_delay)/(self.max_arrival_delay - self.min_arrival_delay)
                            ]
        print "noramlized values"

        domain = self.SVM.domain

        # print "Min disparity: " + str(self.min_disparity) +  "Max disparity: " + str(self.max_disparity)

        for x in range(0, 3):
            print "normalized @ " + str(x) + ": " + str(normalized_values[x])

        toClassify = Orange.data.Instance(domain, [normalized_values[0], normalized_values[1], normalized_values[2], None])
        # print "constructed data instance"

        result = self.SVM(toClassify)

        print result

        if result == True:
            return TrustEvaluationResponse(True)
        else:
            return TrustEvaluationResponse(False)

if __name__ == "__main__":
    rospy.init_node("trust_evaluation_server")
    try:
        trustServer = TrustEvaluationServer()
    except rospy.ROSInterruptException:
        pass
