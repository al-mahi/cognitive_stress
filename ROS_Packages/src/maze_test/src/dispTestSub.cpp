/*
 * dispTestSub.cpp
 *
 *  Created on: Feb 21, 2015
 *      Author: matt
 */

#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <math.h>

geometry_msgs::Point point;

void callback(const geometry_msgs::Point::ConstPtr& msg) {

	double dx, dy, dt;

	dx = abs(point.x -msg->x);
	dy = abs(point.y - msg->y);
	dt = sqrt( pow(msg->x, 2.0) + pow(msg->y, 2.0) );

	printf("dx:%.6f, dy:%.6f, dt:%.6f\n", dx, dy, dt);
}

int main(int argv, char** argc) {

	point.x = 0.0;
	point.y = 0.0;
	point.z = 0.0;

	ros::init(argv, argc, "dispPubSub");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("dispPub", 1, &callback);
	ros::spin();
}
