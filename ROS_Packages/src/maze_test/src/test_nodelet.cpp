/*
 * test_nodelet.cpp
 *
 *  Created on: Jun 4, 2015
 *      Author: matt
 */

#include "test_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

namespace maze_test {

	TestNodelet::TestNodelet() {
		NODELET_INFO("Starting a test nodelet...");
	}

	void TestNodelet::onInit() {

		nh_ = getNodeHandle();
		private_nh_ = getPrivateNodeHandle();

		pub_ = nh_.advertise<std_msgs::String>("test_nodelet", 10);

		sub_ = nh_.subscribe("nodelet_chat", 10, &TestNodelet::nodeletChatCb, this);
	}

	void TestNodelet::sayHello() {
		printf("Hello! I'm a nodelet!");
	}

	void TestNodelet::echo(std::string phrase) {
		printf("Nodelet echos: %s", phrase.c_str());
	}

	std::string TestNodelet::areYouDumb() {
		return "No. What kind of question is that?";
	}

	void TestNodelet::nodeletChatCb(const std_msgs::StringConstPtr &msg) {

		NODELET_INFO("Nodelet callback got: %s", msg->data.c_str());
	}
} //namespace maze_test

PLUGINLIB_DECLARE_CLASS(maze_test, TestNodelet, maze_test::TestNodelet, nodelet::Nodelet);
