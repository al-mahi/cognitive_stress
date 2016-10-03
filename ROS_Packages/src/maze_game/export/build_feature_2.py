#!/usr/bin/env python

import os
import roslib
import pickle
import datetime
from pprint import pprint
import copy
import re
from Orange import *

classifier_file = open(roslib.packages.get_pkg_subdir("coin_game", "include") + "/random_forest_2_with_bool.pck")
random_forest_model = pickle.load(classifier_file)
domain = random_forest_model.domain

features = {}
robots_in_group = {}
robot_names = ["miranda", "trinculo", "ferdinand"]

def robot_count(x):
    return {
        'one': '1',
        'two': '2',
        'three': '3'
    }.get(x, '0')


def build(group, pass_remaining):

    name = robots_in_group[group][pass_remaining - 1]

    with open("{}_robot/output_{}.csv".format(group, group), "w") as out:
        for e in features[group]:

            # Calculate new CLE and save it
            test_data = Orange.data.Instance(domain, [float(e[0]), float(e[1]), float(e[2]), int(e[5] == name), int(e[6]), None])
            cognitive_load_estimate = random_forest_model(test_data)

            out_string = "{},{},{},{},{},{},{}\n".format(e[0], e[1], e[2], cognitive_load_estimate, e[4].replace('\n', ""), str(e[5] == name), e[6])
            out.write(out_string)

    pass_remaining -= 1
    if pass_remaining == 0:
        return
    else:
        build(group, pass_remaining)


def extract(num_robots):

    directory = "{}_robot".format(num_robots)

    for root, dirs, files in os.walk(directory):

        for f in files:
            if f.startswith("."): continue
            if not f.endswith(".csv") or not (f.startswith("trinculo") or f.startswith("miranda") or f.startswith("ferdinand")):
                continue

            robot_name = re.match("(.+?)_(?:.+?)\.csv", f).group(1)

            path = "{}/{}".format(root, f)

            with open(path, "r") as csv_data:

                # Append the dir name as it also indicates the number of robots
                r_count = str(num_robots)

                if r_count not in robots_in_group:
                    robots_in_group[r_count] = []
                    robots_in_group[r_count].append(robot_name)
                elif robot_name not in robots_in_group[r_count]:
                    robots_in_group[r_count].append(robot_name)

                if not features.keys().__contains__(r_count):
                    features[r_count] = []

                # Skip first line
                rows = csv_data.readlines()[1:]
                rows = [r.split(',') for r in rows]

                print rows

                # Save the time for the first row since it will be filtered, but will still need
                # it's delta time
                t = 0.0
                if len(rows) > 0:
                    t = float(rows[0][2].replace('\n', ""))

                if len(rows) > 0:
                    # Save the first time event
                    rows[0][2] = str(t)

                    for i in range(1, len(rows)):
                        t += float(rows[i][2])
                        rows[i][2] = str(t)

                    # Append placeholder for commend recipient and number of robots
                    for i in range(0, len(rows)):
                        rows[i] = [s.replace('\r\n', '') for s in rows[i]]

                        rows[i].append(robot_name.replace('\n', ''))
                        # Append the number of robots in this run
                        rows[i].append(r_count)

                    features[r_count].extend(rows)

for d in range(1, 4):
    extract(d)

for g in range(1, 4):
    build(str(g), g)

classifier_file.close()
