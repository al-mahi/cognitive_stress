#!/usr/bin/python
from __future__ import print_function
import matplotlib.pyplot as plt
import numpy as np


def plot_three():
    ax = plt.subplot(313)
    plt.xlabel("Human Instructions")
    plt.ylabel("Cognitive Stress Level")

    with open("predictions_combined_3_robots.csv", "r") as csvf:
        data = csvf.read()

    rows = data.split("\n")
    rows.pop(0)
    predictions = []
    observations = []

    for row in rows:
        r = row.split(",")
        if len(r) < 2: break
        predictions.append(-1.*float(r[0]))
        observations.append(-1.*float(r[3]))

    # print(predictions)
    # print(observations)
    x = range(len(predictions))

    plt.scatter(x, predictions, color="red", label="predictions")
    plt.scatter(x, observations, color="blue", label="human observation")

    # y = mx + c
    m_pred, c_pred = np.polyfit(x, predictions, 1)
    # Add correlation line
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred*X_plot + c_pred, 'r--', label="least sqr pred.")

    # y = mx + c
    m_obsrv, c_obsrv = np.polyfit(x, observations, 1)
    # Add correlation line
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_obsrv*X_plot + c_obsrv, 'b--', label="least sqr obsrv")

    print(m_pred, m_obsrv)
    print(np.correlate(predictions, observations))
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    textstr = "Correlation\n$\\rho =$ {:02}".format(np.correlate(predictions, observations)[0])
    # place a text box in upper left in axes coords
    ax.text(0.8, 0.3, textstr, transform=ax.transAxes, fontsize=14,
            verticalalignment='top', bbox=props)

    legend = plt.legend(loc='upper left', shadow=True, fontsize='medium')
    plt.title("Three Robots")


def plot_two():
    ax = plt.subplot(312)
    # ax.set_xticks([])
    # plt.xlabel("Human Instructions")
    plt.ylabel("Cognitive Stress Level")

    with open("predictions_combined_2_robots.csv", "r") as csvf:
        data = csvf.read()

    rows = data.split("\n")
    rows.pop(0)
    predictions = []
    observations = []

    for row in rows:
        r = row.split(",")
        if len(r) < 2: break
        predictions.append(-1.*float(r[0]))
        observations.append(-1.*float(r[3]))

    # print(predictions)
    # print(observations)
    x = range(len(predictions))

    plt.scatter(x, predictions, color="red", label="predictions")
    plt.scatter(x, observations, color="blue", label="human observation")

    # y = mx + c
    m_pred, c_pred = np.polyfit(x, predictions, 1)
    # Add correlation line
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred*X_plot + c_pred, 'r--', label="least sqr pred.")

    # y = mx + c
    m_obsrv, c_obsrv = np.polyfit(x, observations, 1)
    # Add correlation line
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_obsrv*X_plot + c_obsrv, 'b--', label="least sqr obsrv")

    print(m_pred, m_obsrv)
    print(np.correlate(predictions, observations))
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    textstr = "Correlation\n$\\rho =$ {:02}".format(np.correlate(predictions, observations)[0])
    # place a text box in upper left in axes coords
    ax.text(0.8, 0.3, textstr, transform=ax.transAxes, fontsize=14,
            verticalalignment='top', bbox=props)

    legend = plt.legend(loc='upper left', shadow=True, fontsize='medium')
    plt.title("Two Robots")


def plot_one():
    ax = plt.subplot(311)
    # ax.set_xticks([])
    # plt.xlabel("Human Instructions")
    plt.ylabel("Cognitive Stress Level")
    with open("predictions_combined_1_robots.csv", "r") as csvf:
        data = csvf.read()

    rows = data.split("\n")
    rows.pop(0)
    predictions = []
    observations = []

    for row in rows:
        r = row.split(",")
        if len(r) < 2: break
        predictions.append(-1.*float(r[0]))
        observations.append(-1.*float(r[3]))

    # print(predictions)
    # print(observations)
    x = range(len(predictions))

    plt.scatter(x, predictions, color="red", label="predictions")
    plt.scatter(x, observations, color="blue", label="human observation")

    # y = mx + c
    m_pred, c_pred = np.polyfit(x, predictions, 1)
    # Add correlation line
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_pred*X_plot + c_pred, 'r--', label="least sqr pred.")

    # y = mx + c
    m_obsrv, c_obsrv = np.polyfit(x, observations, 1)
    # Add correlation line
    axes = plt.gca()
    X_plot = np.linspace(axes.get_xlim()[0], axes.get_xlim()[1], 100)
    plt.plot(X_plot, m_obsrv*X_plot + c_obsrv, 'b--', label="least sqr obsrv")

    print(m_pred, m_obsrv)
    print(np.correlate(predictions, observations))
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    textstr = "Correlation\n$\\rho =$ {:02}".format(np.correlate(predictions, observations)[0])
    # place a text box in upper left in axes coords
    ax.text(0.8, 0.3, textstr, transform=ax.transAxes, fontsize=14,
            verticalalignment='top', bbox=props)

    legend = plt.legend(loc='upper left', shadow=True, fontsize='medium')
    plt.title("One Robot")

plot_one()
plot_two()
plot_three()

plt.show()
