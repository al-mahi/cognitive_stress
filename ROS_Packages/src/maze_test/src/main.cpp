/*
 * main.cpp
 *
 *  Created on: Feb 20, 2015
 *      Author: matt
 */

#include <stdio.h>
#include <vector>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

std::vector<geometry_msgs::Point> expectedPoints;
std::vector<geometry_msgs::Point> dispPoints;
geometry_msgs::Point toAdd;
int pointDex;
double disp[3];

void callback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg) {

	geometry_msgs::Point robot, exp;
	robot = msg->goal.target_pose.pose.position;
	exp = expectedPoints.at(pointDex);

	printf("NEW GOAL RECIEVED!\n");

	disp[0] = (exp.x - robot.x);
	disp[1] = (exp.y - robot.y);
	disp[2] = sqrt((pow(disp[0], 2.0) + pow(disp[1],2.0)));

	toAdd.x = disp[0];
	toAdd.y = disp[1];
	toAdd.z = disp[3];
	dispPoints.push_back(toAdd);

	printf("Calculated Disparity: x: %.6f, y: %.6f, total: %.6f\npointDex: %i\n", disp[0], disp[1], disp[2], pointDex);
	pointDex++;
}

int main(int argv, char** argc) {

	pointDex = 0;

	//Read the points
	std::cout << "Reading the points...\n";

	std::vector<geometry_msgs::Point> points_arr;

	std::ifstream infile;
	infile.open("/home/matt/ros_testing/src/odom_measure/include/points.txt");

	if (!infile.is_open()) {
		std::cerr << "Error! Could not open points file";
		return -1;
	}

	std::string line;

	while (std::getline(infile, line)) {

		std::stringstream ss(line);

		float x, y, z;

		//Read the coords in as float (they must be space delimited!)
		ss >> x >> y >> z;

		geometry_msgs::Point p;

		p.x = x;
		p.y = y;
		p.z = z;

		points_arr.push_back(p);
	}

	for (uint32_t i = 0; i < points_arr.size(); i++) {
		std::cout << "i = " << i << "\n";
		expectedPoints.push_back(points_arr.at(i));
	}

	//setup ros
	ros::init(argv, argc, "maze_game");

	//init node

	ros::NodeHandle n;
	ros::Subscriber sub;
	sub = n.subscribe("move_base/goal", 10, &callback);
	ros::spin();
}

