#!/usr/bin/env python

import os
from pprint import pprint
import copy
import re

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

    with open("output.csv", "a") as out:
        for e in features[group]:
            out_string = "{},{},{},{},{},{}\n".format(e[0], e[1], e[2], e[3], str(e[4] == name), e[5])
            out.write(out_string)

    pass_remaining -= 1
    if pass_remaining == 0:
        return
    else:
        build(group, pass_remaining)


def extract(directory):

    for root, dirs, files in os.walk(directory):

        for f in files:

            if not f.endswith(".csv"):
                continue

            robot_name = re.match("(?:.+?)_(.+?)\.csv", f).group(1)

            path = "{}/{}".format(directory, f)

            with open(path, "r") as csv_data:

                # Append the dir name as it also indicates the number of robots
                r_count = directory.split('/')
                r_count = robot_count(r_count[len(r_count) - 1])

                if r_count not in robots_in_group:
                    robots_in_group[r_count] = []
                    robots_in_group[r_count].append(robot_name)
                elif robot_name not in robots_in_group[r_count]:
                    robots_in_group[r_count].append(robot_name)

                if not features.keys().__contains__(r_count):
                    features[r_count] = []

                # Skip first line
                rows = csv_data.readlines()[1:]
                rows = [r.split(',')[13:17] for r in rows]

                print rows

                # Save the time for the first row since it will be filtered, but will still need
                # it's delta time
                t = 0.0
                if len(rows) > 0:
                    t = float(rows[0][2].replace('\n', ""))

                # Filter values and clean up the data
                rows = [x for x in rows if not (x.__contains__("*\r\n") or x.__contains__("*"))]

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



def sanity_check():
    x = []
    print x

    for i in range(0, 5):
        x.append(i)

    print x

    for i in range(5, 10):
        x.append(i)

    print x

base_path = "model1/"
for d in ["one", "two", "three"]:
    extract(base_path + d)

for g in range(1, 4):
    build(str(g), g)
