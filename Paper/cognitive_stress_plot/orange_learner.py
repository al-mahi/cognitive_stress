#!/usr/bin/python

import Orange.data
import Orange.regression
import Orange.feature
import random
import math
import pickle

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats
import os


def pred_vs_coder(num_robots):
    path = "/home/alien/Conferences/HRI-17-LBR/cognitive_stress/ROS_Packages/src/maze_game/export" \
           "/orange_training_set_rm_irraticness.tab"
    # path = "/home/alien/Conferences/HRI-17-LBR/cognitive_stress/ROS_Packages/src/maze_game/export" \
    #        "/orange_training_set.tab"
    original_data = Orange.data.Table(path)
    data = [d for d in original_data]

    # test = Orange.data.Table(random.sample(data, 5))
    # train = Orange.data.Table([d for d in data if d not in test])
    train = Orange.data.Table([d for d in data])

    rf = Orange.ensemble.forest.RandomForestLearner(train)
    rf.name = "rf"
    domain = rf.domain
    # pickle.dump(rf, open("random_forest_learner.pck", "wb"))
    # print "pred obs"
    # for d in test:
    #     print d, "%.1f %.1f" % (rf(d), d.get_class())
    #
    # print "end!!"

    # classifier_file = open("random_forest_learner.pck")
    # random_forest_model = pickle.load(classifier_file)
    # domain = random_forest_model.domain

    "------------------------------------------------------------------------------------------------------------------"

    dir_name = "{}_robot".format(num_robots)
    model_predictions = {}
    coder_evaluations = {}
    # see https://www.zephyranywhere.com/media/pdf/BH_DS_P-BioHarness3-Data-Sheet_20120919_V02.pdf
    heart_rate = {}
    breathing_rate = {}
    posture = {}
    activity_level = {}
    ecg_amplitude = {}

    for root, dirs, files in os.walk(dir_name):
        if len(files) > 0:
            for fname in files:
                if fname.startswith("."): continue
                path = "{}/{}".format(root, fname)
                # print("reading file {}".format(path))
                with open(path, "r") as csvf:
                    data = csvf.read()
                    rows = data.split("\n")
                    # print("{} rows".format(len(rows)))
                    # print("second row looks like {}".format(rows[1]))
                    # print("split gives {}".format(rows[1].split(",")))
                    if fname.startswith("coder_") or fname.startswith("evaluation"):
                        # rows.pop(0)
                        for row in rows:
                            r = row.split(",")
                            if len(r) < 2: break
                            coder_evaluations[r[0]] = (float(r[1]))
                    elif fname.startswith("Summary"):
                        rows.pop(0)
                        for row in rows:
                            r = row.split(",")
                            if len(r) < 2: break
                            heart_rate[r[5]] = float(r[7])
                            breathing_rate[r[5]] = float(r[8])
                            posture[r[5]] = float(r[10])
                            activity_level[r[5]] = float(r[11])
                            ecg_amplitude[r[5]] = float(r[18])
                    elif fname.startswith("output"):
                        rows.pop(0)
                        for row in rows:
                            r = row.split(",")
                            if len(r) < 2: break
                            test_data = Orange.data.Instance(domain, [float(r[0]),
                                                                      # float(r[1]),
                                                                      float(r[2]), None])
                            cognitive_load_estimate = rf(test_data)
                            model_predictions[r[4]] = (-1. * cognitive_load_estimate)

    ax = plt.subplot(2, 1, num_robots)
    if num_robots == 2:
        plt.xlabel("Models Estimation of Stress Level")
    plt.ylabel("Coders Evaluation\nof Stress Level")
    time_predictions = set(model_predictions.keys())
    time_observation = set(coder_evaluations.keys())
    common_times = time_predictions & time_observation
    predictions = np.array([model_predictions.get(t) for t in common_times])
    observations = np.array([coder_evaluations.get(t) for t in common_times]) / 100.
    # plt.scatter(x, predictions, color="red", label="predictions")
    plt.scatter(predictions, observations, color="blue", label="predictions vs\nevaluation")
    # y = mx + c
    m_pred, c_pred = np.polyfit(predictions, observations, 1)
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred * X_plot + c_pred, 'b--', label="least sqr")

    rho1, pv = scipy.stats.pearsonr(predictions, observations)
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    textstr = "correlation\n $\\rho =$ {:.3}\n".format(rho1)
    # place a text box in upper left in axes coords
    ax.text(0.85, 0.3, textstr, transform=ax.transAxes, fontsize='medium',
            verticalalignment='top', bbox=props)
    legend = plt.legend(loc='upper left', shadow=True, fontsize='medium')
    plt.title("{} Robot".format(num_robots))


if __name__ == "__main__":
    pred_vs_coder(1)
    pred_vs_coder(2)
    ext = "eps"
    plt.tight_layout()
    plt.savefig("plots/coder_eval_1_2.{}".format(ext), format=ext)
    plt.show()
