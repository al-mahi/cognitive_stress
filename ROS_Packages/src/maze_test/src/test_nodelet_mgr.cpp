/*
 * test_nodelet_mgr.cpp
 *
 *  Created on: Jun 4, 2015
 *      Author: matt
 */

#include <ros/ros.h>
#include <nodelet/loader.h>
#include <string>

int main(int argc, char** argv) {

	ros::init(argc, argv, "test_nodelet_mgr");

	nodelet::Loader nodelet;
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;
	std::string nodelet_name = ros::this_node::getName();
	nodelet.load(nodelet_name, "maze_test/test_nodelet", remap, nargv);
	ros::spin();

	return EXIT_SUCCESS;
}
