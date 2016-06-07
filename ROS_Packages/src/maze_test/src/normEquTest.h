/*
 * normEquTest.h
 *
 *  Created on: Mar 30, 2015
 *      Author: matt
 */

#ifndef MAZE_TEST_SRC_NORMEQUTEST_H_
#define MAZE_TEST_SRC_NORMEQUTEST_H_

#include <ros/ros.h>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <boost/program_options.hpp>

bool usePoints;
std::vector<std::vector<double> > points;
std::vector<std::vector<double> > pointsFromFile;

void readPoints();
void computeDisparity();

#endif /* MAZE_TEST_SRC_NORMEQUTEST_H_ */
