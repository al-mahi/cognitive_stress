"""
Demo of the new boxplot functionality
"""

import numpy as np
import matplotlib.pyplot as plt

import pandas as pd

manual1 = [353,350,374,392,351,172,234]
manual2 = [376,354,523,281,341,228,382]
auto1 = [236,213,112,119]
auto2 = [236,168,218,318,271,306,225,268,118,454]
data = [manual1, auto1, manual2, auto2]
labels=["One Robot", "One Robot", "Two Robots", "Two Robots"]
boxes = plt.boxplot(data,labels=labels, meanline=True, patch_artist=True)
colors = ["lightgreen", "lightblue", "lightgreen", "lightblue"]
for patch, color in zip(boxes['boxes'], colors):
    patch.set_facecolor(color)

plt.figtext(.6, .75, 'Manual',
            backgroundcolor="lightgreen", color='white',
            size='medium')
plt.figtext(.6, .8, 'Autonomous Assistance on\nCognitive Load Estimate',
            backgroundcolor="lightblue",
            color='white', weight='roman', size='medium')

plt.legend()
plt.savefig("plots/BoxWiskerTimesCompMaualVsAuto2.eps", format='eps')
plt.show()
