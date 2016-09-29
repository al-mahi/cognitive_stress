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
                    else:
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
    ax = plt.subplot(6, 3, 0+num_robots)
    plt.xlabel("Models Estimation of Stress Level")
    plt.ylabel("Physiological Evaluation of Stress Level")
    time_observation_heart = set(heart_rate.keys())
    common_times_heart = time_predictions & time_observation_heart
    predictions_heart = np.array([model_predictions.get(t) for t in common_times_heart])
    observations_heart = np.array([heart_rate.get(t) for t in common_times_heart])
    # observations_heart /= observations_heart.max()
    plt.scatter(predictions_heart, observations_heart, color="b", label="heart rate")
    # y = mx + c
    m_pred, c_pred = np.polyfit(predictions_heart, observations_heart, 1)
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred*X_plot + c_pred, 'b--', label="least sqr heart")
    rho, pv = scipy.stats.pearsonr(predictions_heart, observations_heart)
    textstr = "heart $\\rho =$ {:.3}\n".format(rho)
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    # place a text box in upper left in axes coords
    ax.text(0.7, 0.2, textstr, transform=ax.transAxes, fontsize='medium',
            verticalalignment='top', bbox=props)

    legend = plt.legend(loc='upper left', shadow=True, fontsize='medium')
    plt.title("{} Robots".format(num_robots))

    # ------------------------------------------------------------------------------------------------------------------
    ax = plt.subplot(6, 3, 3 + num_robots)
    time_observation_breat = set(breathing_rate.keys())
    common_times_breat = time_predictions & time_observation_breat
    predictions_breat = np.array([model_predictions.get(t) for t in common_times_breat])
    observations_breat = np.array([breathing_rate.get(t) for t in common_times_breat])
    plt.scatter(predictions_breat, observations_breat, color="g", label="breathing rate")
    # observations_breat /= observations_breat.max()
    # y = mx + c
    m_pred, c_pred = np.polyfit(predictions_breat, observations_breat, 1)
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred * X_plot + c_pred, 'g--', label="least sqr breathing")
    rho, pv = scipy.stats.pearsonr(predictions_breat, observations_breat)
    textstr = "breathing $\\rho =$ {:.3}\n".format(rho)
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    # place a text box in upper left in axes coords
    ax.text(0.7, 0.2, textstr, transform=ax.transAxes, fontsize='medium',
            verticalalignment='top', bbox=props)

    legend = plt.legend(loc='upper left', shadow=True, fontsize='medium')
    plt.title("{} Robots".format(num_robots))

    # ------------------------------------------------------------------------------------------------------------------
    ax = plt.subplot(6, 3, 6 + num_robots)
    time_observation_postu = set(posture.keys())
    common_times_postu = time_predictions & time_observation_postu
    predictions_postu = np.array([model_predictions.get(t) for t in common_times_postu])
    observations_postu = np.array([posture.get(t) for t in common_times_postu]) / -360.
    # observations_postu /= observations_postu.max()
    plt.scatter(predictions_postu, observations_postu, color="c", label="posture")
    # y = mx + c
    m_pred, c_pred = np.polyfit(predictions_postu, observations_postu, 1)
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred * X_plot + c_pred, 'c--', label="least sqr posture")
    rho, pv = scipy.stats.pearsonr(predictions_postu, observations_postu)
    textstr = "posture $\\rho =$ {:.3}\n".format(rho)
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    # place a text box in upper left in axes coords
    ax.text(0.7, 0.2, textstr, transform=ax.transAxes, fontsize='medium',
            verticalalignment='top', bbox=props)

    legend = plt.legend(loc='upper left', shadow=True, fontsize='medium')
    plt.title("{} Robots".format(num_robots))
    # ------------------------------------------------------------------------------------------------------------------
    ax = plt.subplot(6, 3, 9 + num_robots)
    time_observation_activ = set(activity_level.keys())
    common_times_activ = time_predictions & time_observation_activ
    predictions_activ = np.array([model_predictions.get(t) for t in common_times_activ])
    observations_activ = np.array([activity_level.get(t) for t in common_times_activ])
    # observations_activ /= observations_activ.max()
    plt.scatter(predictions_activ, observations_activ, color="m", label="activity")
    # y = mx + c
    m_pred, c_pred = np.polyfit(predictions_activ, observations_activ, 1)
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred * X_plot + c_pred, 'c--', label="least sqr activity")
    rho, pv = scipy.stats.pearsonr(predictions_activ, observations_activ)
    textstr = "activity $\\rho =$ {:.3}\n".format(rho)
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    # place a text box in upper left in axes coords
    ax.text(0.7, 0.2, textstr, transform=ax.transAxes, fontsize='medium',
            verticalalignment='top', bbox=props)

    legend = plt.legend(loc='upper left', shadow=True, fontsize='medium')
    plt.title("{} Robots".format(num_robots))
    # ------------------------------------------------------------------------------------------------------------------

    ax = plt.subplot(6, 3, 12 + num_robots)
    time_observation_ecgAm = set(ecg_amplitude.keys())
    common_times_ecgAm = time_predictions & time_observation_ecgAm
    predictions_ecgAm = np.array([model_predictions.get(t) for t in common_times_ecgAm])
    observations_ecgAm = np.array([ecg_amplitude.get(t) for t in common_times_ecgAm])
    # observations_ecgAm /= observations_ecgAm.max()
    plt.scatter(predictions_ecgAm, observations_ecgAm, color="r", label="ECG amplitude")
    # y = mx + c
    m_pred, c_pred = np.polyfit(predictions_ecgAm, observations_ecgAm, 1)
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred * X_plot + c_pred, 'c--', label="least sqr ECG")
    rho, pv = scipy.stats.pearsonr(predictions_ecgAm, observations_ecgAm)
    textstr = "ECG $\\rho =$ {:.3}\n".format(rho)
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    # place a text box in upper left in axes coords
    ax.text(0.7, 0.2, textstr, transform=ax.transAxes, fontsize='medium',
            verticalalignment='top', bbox=props)

    legend = plt.legend(loc='upper left', shadow=True, fontsize='medium')
    plt.title("{} Robots".format(num_robots))

    # --------------------------physiological observation------------------------------------

    ax = plt.subplot(6, 3, 15+num_robots)
    plt.xlabel("Models Estimation of Stress Level")
    plt.ylabel("Coders Evaluation of Stress Level")

    time_predictions = set(model_predictions.keys())
    time_observation = set(coder_evaluations.keys())
    common_times = time_predictions & time_observation
    predictions = np.array([model_predictions.get(t) for t in common_times])
    observations = np.array([coder_evaluations.get(t) for t in common_times])/100.

    # plt.scatter(x, predictions, color="red", label="predictions")
    plt.scatter(predictions, observations, color="blue", label="human observation")
    # y = mx + c
    m_pred, c_pred = np.polyfit(predictions, observations, 1)
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred*X_plot + c_pred, 'b--', label="least sqr of plotted points.")


    rho1, pv = scipy.stats.pearsonr(predictions, observations)
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    textstr = "Correlation\npearson $\\rho =$ {:.3}\n".format(rho1)
    # place a text box in upper left in axes coords
    ax.text(0.7, 0.2, textstr, transform=ax.transAxes, fontsize='medium',
            verticalalignment='top', bbox=props)
    legend = plt.legend(loc='upper left', shadow=True, fontsize='medium')
    plt.title("{} Robots".format(num_robots))

fig = plt.figure(figsize=(30, 60))
pred_vs_coder(1)
pred_vs_coder(2)
pred_vs_coder(3)
ext = "eps"
plt.savefig("prediction_vs_human_observation.{}".format(ext), format=ext)
plt.show()


