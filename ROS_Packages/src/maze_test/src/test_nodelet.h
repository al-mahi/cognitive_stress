/*
 * test_nodelet.h
 *
 *  Created on: Jun 4, 2015
 *      Author: matt
 */

#ifndef MAZE_TEST_SRC_TEST_NODELET_H_
#define MAZE_TEST_SRC_TEST_NODELET_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>

namespace maze_test {

	class TestNodelet : public nodelet::Nodelet {

	public:
		TestNodelet();
		std::string changeMe;

	private:
		virtual void onInit();
		void sayHello();
		void echo(std::string phrase);
		std::string areYouDumb();

		void nodeletChatCb(const std_msgs::StringConstPtr &msg);

		ros::NodeHandle nh_, private_nh_;
		ros::Subscriber sub_;
		ros::Publisher pub_;
	};

} // namespace maze_test

#endif /* MAZE_TEST_SRC_TEST_NODELET_H_ */
