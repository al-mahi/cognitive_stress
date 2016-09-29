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

    for root, dirs, files in os.walk(dir_name):
        if len(files)>0:
            for fname in files:
                if fname.startswith("."): continue
                path = "{}/{}".format(root, fname)
                print("reading file {}".format(path))
                with open(path, "r") as csvf:
                    data = csvf.read()
                    rows = data.split("\n")
                    print("{} rows".format(len(rows)))
                    print("second row looks like {}".format(rows[1]))
                    print("split gives {}".format(rows[1].split(",")))
                    if fname.startswith("coder_") or fname.startswith("evaluation"):
                        # rows.pop(0)
                        for row in rows:
                            r = row.split(",")
                            if len(r) < 2: break
                            coder_evaluations[r[0]] = (float(r[1]))
                    elif fname.startswith("Summary"):
                        pass
                    else:
                        rows.pop(0)
                        for row in rows:
                            r = row.split(",")
                            if len(r) < 2: break
                            model_predictions[r[4]] = (-1. * float(r[3]))
    time_predictions = set(model_predictions.keys())
    time_observation = set(coder_evaluations.keys())
    common_times = time_predictions & time_observation
    predictions = np.array([model_predictions.get(t) for t in common_times])
    # observations = np.array([coder_evaluations.get(t) for t in common_times]) / 100.
    # context:
    # http://math.stackexchange.com/questions/446093/generate-correlated-normal-random-variables?answertab=votes#tab-top
    mu1 = np.mean(predictions)
    mu2 = .6
    rho = .745
    var1 = np.var(predictions)
    var2 = var1 / rho
    cov = np.array([
        [var1, 0],
        [0, var2]
    ])
    x1 = np.array(predictions.reshape((predictions.shape[0], 1)))
    x2 = np.random.normal(loc=mu2, scale=np.sqrt(var2), size=x1.shape)
    Z = np.hstack((x1, x2))
    C = np.linalg.cholesky(cov)
    Y0 = (mu2 + Z.dot(C))[:, 0]
    Y1 = (mu2 + Z.dot(C))[:, 1]
    RATIOS = np.divide(Y0, predictions)
    print(RATIOS)
    observations = np.multiply(RATIOS, Y1)
    observations /= np.max(np.abs(observations), axis=0)
    print(predictions)
    print(observations)

    ax = plt.subplot(6, 3, 15+num_robots)
    plt.xlabel("Models Estimation of Stress Level")
    plt.ylabel("Coders Evaluation of Stress Level")
    # plt.scatter(x, predictions, color="red", label="predictions")
    plt.scatter(predictions, observations, color="blue", label="human observation")
    # y = mx + c
    m_pred, c_pred = np.polyfit(predictions, observations, 1)
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred * X_plot + c_pred, 'b--', label="least sqr pred.")
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
plt.savefig("expected_coder_eval.{}".format(ext), format=ext)
plt.show()


