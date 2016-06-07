__author__ = 'matthew'

import Orange
import Orange.data
import pickle
import rospy
from maze_game.srv import *

SVM = pickle.load(file("svm.pck"))

min_disparity = 0.0
max_disparity = 0.0
min_percent_damage = 0.0
max_percent_damage = 0.0
min_arrival_delay = 0.0
max_arrival_delay = 0.0

def evaluate_trust(req):
    print "Evaluating trust..."
    print "\tDisparity: " + req.disparity
    print "\tPercent Damage: " + req.percentDamage
    print "\tArrival Delay: " + req.goalArrivalDelay

    Orange.data.Table()

    return TrustEvaluationResponse(True)

def trust_evaluation_server():
    rospy.init_node("trust_evaluation_server")
    if rospy.has_param("/trust_evaluation_server/min_disparity"):
        min_disparity = rospy.get_param("/trust_evaluation_server/min_disparity")
    else:
        min_disparity = 0.0

    if rospy.has_param("/trust_evaluation_server/max_disparity"):
        max_disparity = rospy.get_param("/trust_evaluation_server/max_disparity")
    else:
        max_disparity = 1.0

    if rospy.has_param("/trust_evaluation_server/min_percent_damage"):
        min_percent_damage = rospy.get_param("/trust_evaluation_server/min_percent_damage")
    else:
        min_percent_damage = 0.0

    if rospy.has_param("/trust_evaluation_server/max_percent_damage"):
        max_percent_damage = rospy.get_param("/trust_evaluation_server/max_percent_damage")
    else:
        max_percent_damage = 1.0

    if rospy.has_param("/trust_evaluation_server/min_arrival_delay"):
        min_arrival_delay = rospy.get_param("/trust_evaluation_server/min_arrival_delay")
    else:
        min_arrival_delay = 0.0

    if rospy.has_param("/trust_evaluation_server/max_arrival_delay"):
        max_arrival_delay = rospy.get_param("/trust_evaluation_server/max_arrival_delay")
    else:
        max_arrival_delay = 1.0
    s = rospy.Service('evaluate_trust', TrustEvaluation, evaluate_trust)
    print "Trust Evaluation Server ready..."
    rospy.spin()

if __name__ == "__main__":
    trust_evaluation_server()