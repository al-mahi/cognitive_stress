#!/usr/bin/python
from __future__ import print_function
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats
import os


def pred_vs_coder(num_robots):
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
        if len(files)>0:
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
                    elif fname.startswith("output_"):
                        rows.pop(0)
                        for row in rows:
                            r = row.split(",")
                            if len(r) < 2: break
                            model_predictions[r[4]] = (-1. * float(r[3]))

    time_predictions = set(model_predictions.keys())
    #123
    #456
    #789
    #...

    # ------------------------------------------------------------------------------------------------------------------
    ax = plt.subplot(2, 1, num_robots)
    plt.ylabel("Physiological Evaluation\nof Stress Level")
    time_observation_breat = set(breathing_rate.keys())
    common_times_breat = time_predictions & time_observation_breat
    predictions_breat = np.array([model_predictions.get(t) for t in common_times_breat])
    observations_breat = np.array([breathing_rate.get(t) for t in common_times_breat])
    plt.scatter(predictions_breat, observations_breat, color="g", label="predictions vs\nbreathing rate")
    # observations_breat /= observations_breat.max()
    # y = mx + c
    m_pred, c_pred = np.polyfit(predictions_breat, observations_breat, 1)
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred * X_plot + c_pred, 'g--', label="least sqr breathing")
    rho, pv = scipy.stats.pearsonr(predictions_breat, observations_breat)
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    textstr = "correlation\n $\\rho =$ {:.3}\n".format(rho)
    # place a text box in upper left in axes coords
    ax.text(0.85, 0.3, textstr, transform=ax.transAxes, fontsize='medium',
            verticalalignment='top', bbox=props)
    legend = plt.legend(loc='upper left', shadow=True, fontsize='medium')
    plt.title("{} Robot".format(num_robots))
    # plt.title("{} Robots".format(num_robots))

    plt.ylabel("Physiological Evaluation\nof Stress Level")
    if num_robots == 2:
        plt.xlabel("Models Estimation of Stress Level")

# fig = plt.figure(figsize=(4, 8))
pred_vs_coder(1)
pred_vs_coder(2)
# pred_vs_coder(3)
ext = "eps"
plt.savefig("prediction_vs_b_p_2.{}".format(ext), format=ext)
plt.show()


