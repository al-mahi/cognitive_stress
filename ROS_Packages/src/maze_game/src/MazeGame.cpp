#include "MazeGame.h"
//-------added by Mahi-----------
#include "franticness.h"
//-------------------------------

int main(int argc, char **argv) {

    ros::init(argc, argv, "maze_game");
    ros::NodeHandle nh("~");

    ROS_INFO("Starting new maze_game node...");

    MazeGame game = MazeGame(&nh);

    ros::ServiceServer addRobotService = nh.advertiseService("add_robot", &MazeGame::addRobot, &game);
    ros::ServiceServer removeRobotService = nh.advertiseService("remove_robot", &MazeGame::removeRobot, &game);
    ros::ServiceServer startRobotService = nh.advertiseService("start_robot", &MazeGame::startRobot, &game);
    //ros::ServiceServer restartRobotService = nh.advertiseService("restart_robot", &MazeGame::restartRobot, &game);
    ros::ServiceServer requesetMazePoints = nh.advertiseService("request_maze_points", &MazeGame::requestMazeMarkers,
                                                                &game);

    //Score Averaging Services
    ros::ServiceServer requestAverageScores = nh.advertiseService("request_average_scores",
                                                                  &MazeGame::requestAverageScores, &game);
    ros::ServiceServer submitScore = nh.advertiseService("submit_score", &MazeGame::submitScores, &game);
    ros::ServiceServer exportRunData = nh.advertiseService("export_run_data", &MazeGame::exportRunData, &game);

    //See if there are any roboealthPoints, std::vector<geoPREV_MOUSE_POSX = data->event.u.keyButtonPointer.rootX;metry_msgs::Point> givenPoints, std::vector<geometry_msgs::Point> expectedPoints, std::vector<ros::Time> inputDelay>) {ts to initialize with
    std::vector<std::string> start_robots;
    if (nh.hasParam("start_robots")) {
        nh.getParam("start_robots", start_robots);
        for (int i = 0; i < start_robots.size(); i++) {
            game.model->addRobot(start_robots[i]);
        }
    }

    //------added by Mahi------------
    printf("!!!! Thread Mouse click !!!!");
    std::thread t(mouse_events);
    //-------------------------------

    ROS_INFO("Checking Robots...");
    while (nh.ok()) {
//		ROS_INFO("MazeGame Spinning.");
        game.model->checkRobotsPosition();
        ros::Rate(1.0).sleep();
        ros::spinOnce();
    }

    //-------added by Mahi-----------
    t.detach();
    t.~thread();
    printf("!!!! Terminated Thread Mouse click !!!!");
    //-------------------------------
    std::cout << "Returning!\n";
    return 0;
}

MazeGame::MazeGame(ros::NodeHandle *n) {
    ROS_INFO("Creating model...\n");
    //init members
    this->nh = *n;
    this->model = new GameModel(n);
}

bool MazeGame::addRobot(maze_game::AddRobot::Request &req, maze_game::AddRobot::Response &res) {
    model->addRobot(req.robot_name);
    res.done = "true";
    return true;
}

bool MazeGame::removeRobot(maze_game::RemoveRobot::Request &req, maze_game::RemoveRobot::Response &res) {

    model->removeRobot(req.robot_name);
    res.done = "true";
    return true;
}

bool MazeGame::startRobot(maze_game::StartRobot::Request &req, maze_game::StartRobot::Response &res) {
    model->startRobot(req.robot_name);
    res.done = "true";
    return true;
}

//bool MazeGame::restartRobot(std_msgs::String &req, std_msgs::String &res) {
//
//	if (!req.data.empty()) {
//		model->restartRobot(req.data);
//		res.data = "true";
//	} else {
//		ROS_WARN("No name was given for robot to restart!");
//		res.data = "false";
//	}
//
//	return true;
//}

bool MazeGame::requestMazeMarkers(maze_game::RequestMazeMarkers::Request &req,
                                  maze_game::RequestMazeMarkers::Response &res) {
    model->publishMazeMarkers();
    res.done = "done";
    return true;
}

bool MazeGame::requestAverageScores(maze_game::RequestAverageScores::Request &req,
                                    maze_game::RequestAverageScores::Response &res) {

    std::pair<ros::Duration, int32_t> avgScores = model->getAverageScores();
    res.avgRuntime = avgScores.first;
    res.avgDamage = avgScores.second;
    return true;
}

bool MazeGame::submitScores(maze_game::SubmitScores::Request &req, maze_game::SubmitScores::Response &res) {

    //Comented out until service type has been updated. This may also be removed as it seems unnecessary
    //model->submitAndUpdateAverageScores(req.robot_name, req.goal_arival_times, req.damage_at_goal);
    return true;
}

bool MazeGame::exportRunData(maze_game::ExportRunData::Request &req, maze_game::ExportRunData::Response &res) {

    ROS_INFO("Calling data export from MazeGame!");
    res.success = model->exportRunData(req.robotName);
    return res.success;
}
