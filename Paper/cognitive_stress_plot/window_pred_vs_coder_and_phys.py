#!/usr/bin/python
from __future__ import print_function
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats
import os
import datetime as dt


class datum:
    def __init__(self, time, value):
        self._t = time
        self._v = value

    @property
    def time(self):
        return self._t

    @time.setter
    def time(self, value):
        self._v = value

    @property
    def level(self):
        return self._v

    @level.setter
    def level(self, value):
        self._v = value

def str_time(s):
    """
    :type s: str
    """
    s_date = (s.split("-")[0]).split("_")
    s_time = (s.split("-")[1]).split("_")
    YY = int(s_date[0])
    MM = int(s_date[1])
    DD = int(s_date[2])
    hh = int(s_time[0])
    mm = int(s_time[1])
    ss = int(s_time[2])
    return dt.datetime(YY,MM,DD,hh,mm,ss)


def pred_vs_coder(num_robots):
    dir_name = "{}_robot".format(num_robots)
    model_predictions = []
    coder_evaluations = []
    # see https://www.zephyranywhere.com/media/pdf/BH_DS_P-BioHarness3-Data-Sheet_20120919_V02.pdf
    heart_rate = []
    breathing_rate = []
    posture = []
    activity_level = []
    ecg_amplitude = []

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
                            coder_evaluations.append(datum(str_time(r[0]),float(r[1])))
                    elif fname.startswith("Summary"):
                        rows.pop(0)
                        for row in rows:
                            r = row.split(",")
                            if len(r) < 2: break
                            heart_rate.append(datum(str_time(r[5]), float(r[7])))
                            breathing_rate.append(datum(str_time(r[5]), float(r[8])))
                            posture.append(datum(str_time(r[5]), float(r[10])))
                            activity_level.append(datum(str_time(r[5]), float(r[11])))
                            ecg_amplitude.append(datum(str_time(r[5]), float(r[18])))
                    else:
                        rows.pop(0)
                        for row in rows:
                            r = row.split(",")
                            if len(r) < 2: break
                            model_predictions.append(datum(str_time(r[4]), (-1. * float(r[3]))))

    model_predictions = sorted(model_predictions, key=lambda ob: ob.time)
    coder_evaluations = sorted(coder_evaluations, key=lambda ob: ob.time)
    heart_rate = sorted(heart_rate, key=lambda ob: ob.time)
    breathing_rate = sorted(breathing_rate, key=lambda ob: ob.time)
    posture = sorted(posture, key=lambda ob: ob.time)
    activity_level = sorted(activity_level, key=lambda ob: ob.time)
    ecg_amplitude = sorted(ecg_amplitude, key=lambda ob: ob.time)

    def windowed_prediction(predictions):
        window = 5.  # 5*2 = 10 sec
        max_preditions = predictions
        for i in range(len(predictions)):
            left = i - 1
            right = i + 1
            max_stress = predictions[i].level
            while left >= 0:
                if (predictions[i].time - predictions[left].time).total_seconds() > window or left < (i-2):
                    break
                if predictions[left].level > max_stress:
                    max_stress = predictions[left].level
                left -= 1
            while right < min((i+2), len(predictions)):
                if (predictions[right].time - predictions[i].time).total_seconds() > window:
                    break
                if predictions[right].level > max_stress:
                    max_stress = predictions[left].level
                right += 1
            max_preditions[i].level = max_stress
        return max_preditions

    predictions = windowed_prediction(model_predictions)

    def get_phy_data(pred, observations):
        x = []
        y = []
        for p in pred:
            for ob in observations:
                if np.isclose(np.fabs((p.time - ob.time).total_seconds()), 0.):
                    x.append(p.level)
                    y.append(ob.level)
        return x, y
    # ------------------------------------------------------------------------------------------------------------------
    ax = plt.subplot(6, 3, 0+num_robots)
    plt.xlabel("Models Estimation of Stress Level")
    plt.ylabel("Physiological Evaluation of Stress Level")

    predictions_heart, observations_heart = get_phy_data(predictions, heart_rate)
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
    predictions_breat, observations_breat = get_phy_data(predictions, breathing_rate)
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
    predictions_postu, observations_postu = get_phy_data(predictions, posture)
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
    predictions_activ, observations_activ = get_phy_data(predictions, activity_level)
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
    predictions_ecgAm, observations_ecgAm = get_phy_data(predictions, ecg_amplitude)
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

    predictions, observations = get_phy_data(predictions, coder_evaluations)

    # plt.scatter(x, predictions, color="red", label="predictions")
    plt.scatter(predictions, observations, color="blue", label="human observation")
    # y = mx + c
    m_pred, c_pred = np.polyfit(predictions, observations, 1)
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred*X_plot + c_pred, 'b--', label="least sqr pred.")

    rho1, pv = scipy.stats.pearsonr(predictions, observations)
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    textstr = "Correlation\npearson $\\rho =$ {:.3}\n".format(rho1)
    # place a text box in upper left in axes coords
    ax.text(0.7, 0.2, textstr, transform=ax.transAxes, fontsize='medium',
            verticalalignment='top', bbox=props)

    plt.title("{} Robots".format(num_robots))

fig = plt.figure(figsize=(30, 60))
pred_vs_coder(1)
pred_vs_coder(2)
pred_vs_coder(3)
ext = "eps"
plt.savefig("windowed_prediction_vs_human_observation.{}".format(ext), format=ext)
plt.show()


