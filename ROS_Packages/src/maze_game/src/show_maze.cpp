#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

int main(int argc, char **argv) {

    std::vector<geometry_msgs::Point> points_arr, points_arr_right;

    std::string path;
    path = ros::package::getPath("maze_game").append("/include/left_points.txt");

    std::ifstream infile;
    infile.open(path.c_str());

    if (!infile.is_open()) {
        ROS_ERROR("Error! Could not open left points file!");
        return 1;
    }

    std::string line;

    while (std::getline(infile, line)) {

        std::stringstream ss(line);

        float x, y, z;

        //Read the coords in as float (they must be space delimted!)
        ss >> x >> y >> z;

        geometry_msgs::Point p;

        p.x = x;
        p.y = y;
        p.z = z;

        points_arr.push_back(p);
    }

    infile.close();

    path = ros::package::getPath("maze_game").append("/include/right_points.txt");
    infile.open(path.c_str());

    if (!infile.is_open()) {
        ROS_ERROR("Error! COuld not open right points file!");
        return 1;
    }

    line = "";

    while (std::getline(infile, line)) {
        std::stringstream ss(line);

        float x, y, z;

        ss >> x >> y >> z;

        geometry_msgs::Point p;

        p.x = x;
        p.y = y;
        p.z = z;

        points_arr_right.push_back(p);
    }

    //Bring up the node
    ros::init(argc, argv, "maze_markers");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ROS_INFO("Node started with name maze_markers...\n");

    //Form visualization messages

    //Upper wall
    visualization_msgs::Marker upper_wall_line_list, lower_wall_line_list;
    upper_wall_line_list.header.frame_id = lower_wall_line_list.header.frame_id = "/map";
    upper_wall_line_list.header.stamp = lower_wall_line_list.header.stamp = ros::Time::now();
    upper_wall_line_list.ns = "maze_wall_upper";
    lower_wall_line_list.ns = "maze_wall_lower";
    upper_wall_line_list.action = lower_wall_line_list.action = visualization_msgs::Marker::ADD;
    upper_wall_line_list.pose.orientation.w = lower_wall_line_list.pose.orientation.w = 1.0;

    upper_wall_line_list.id = 0;
    lower_wall_line_list.id = 1;

    upper_wall_line_list.type = visualization_msgs::Marker::LINE_STRIP;
    lower_wall_line_list.type = visualization_msgs::Marker::LINE_STRIP;

    upper_wall_line_list.scale.x = 0.05;
    upper_wall_line_list.scale.y = 0.05;
    lower_wall_line_list.scale.x = 0.05;
    lower_wall_line_list.scale.y = 0.05;

    upper_wall_line_list.color.b = lower_wall_line_list.color.b = 1.0;
    upper_wall_line_list.color.a = lower_wall_line_list.color.a = 1.0;

    //std::cout << "points_arr.size() = " << points_arr.size() << "\n";
    //Create the points themselves
    for (uint32_t i = 0; i < points_arr.size(); i++) {
        //std::cout << "i = " << i << "\n";
        upper_wall_line_list.points.push_back(points_arr.at(i));
        lower_wall_line_list.points.push_back(points_arr_right.at(i));

        if (i < points_arr.size() - 1) {
            upper_wall_line_list.points.push_back(points_arr.at(i + 1));
            lower_wall_line_list.points.push_back(points_arr_right.at(i + 1));
            //std::cout << "Added points " << i << " and " << i+1 << " to line list.\n";
        }
    }

    ros::Rate r(1);

    while (ros::ok()) {
        marker_pub.publish(upper_wall_line_list);
        marker_pub.publish(lower_wall_line_list);
        r.sleep();
    }
}








