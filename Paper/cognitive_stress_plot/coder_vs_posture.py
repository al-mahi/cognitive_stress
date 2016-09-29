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
                            coder_evaluations[r[0]] = (float(r[1]))/100.
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
                    else:
                        rows.pop(0)
                        for row in rows:
                            r = row.split(",")
                            if len(r) < 2: break
                            model_predictions[r[4]] = (-1. * float(r[3]))

    time_predictions = set(coder_evaluations.keys())
    #123
    #456
    #789
    #...
    # ------------------------------------------------------------------------------------------------------------------
    ax = plt.subplot(1, 2, 0 + num_robots)
    plt.xlabel("coder evaluation of stress")
    if num_robots==1:
        plt.ylabel("observed posture")
    time_observation_postu = set(posture.keys())
    common_times_postu = time_predictions & time_observation_postu
    predictions_postu = np.array([coder_evaluations.get(t) for t in common_times_postu])
    observations_postu = np.array([posture.get(t) for t in common_times_postu]) / -360.
    # observations_postu /= observations_postu.max()
    plt.scatter(predictions_postu, observations_postu, color="c", label="posture")
    # y = mx + c
    # print(time_observation_postu)
    # print(common_times_postu)
    m_pred, c_pred = np.polyfit(predictions_postu, observations_postu, 1)
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred * X_plot + c_pred, 'c--', label="least sqr posture")
    rho, pv = scipy.stats.pearsonr(predictions_postu, observations_postu)
    textstr = "correlation\n$\\rho =$ {:.3}\n".format(rho)
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    # place a text box in upper left in axes coords
    ax.text(0.66, 0.20, textstr, transform=ax.transAxes, fontsize='medium',
            verticalalignment='top', bbox=props)

    legend = plt.legend(loc='upper left', shadow=True, fontsize='medium')
    plt.title("{} Robots".format(num_robots))

fig = plt.figure(figsize=(8, 4))
pred_vs_coder(1)
pred_vs_coder(2)
ext = "eps"
plt.tight_layout()
plt.savefig("coder_vs_posture.{}".format(ext), format=ext)
plt.show()


