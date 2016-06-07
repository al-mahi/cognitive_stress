/*
 * dispTestPub.cpp
 *
 *  Created on: Feb 21, 2015
 *      Author: matt
 */

#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

int main(int argv, char** argc) {
	char buf[256];
	double x, y, z;
	geometry_msgs::Point toPub;

	ros::init(argv, argc, "dispTestPublisher");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<geometry_msgs::Point>("dispPub", 1);

	while(n.ok()) {

		printf("Read:\n");
		std::cin >> x >> y >> z;

		toPub.x = x;
		toPub.y = y;
		toPub.z = z;

		printf("x: %.6f, y:%.6f, z:%.6f\n", x, y, z);

		pub.publish(toPub);
	}

	return 0;
}
