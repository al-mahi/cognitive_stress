#include "GameModel.h"
#include "Robot.h"

GameModel::GameModel(ros::NodeHandle *n) {
	this->nh = *n;
	readPoints();
	readExpectedArrivalTimes();

	ROS_INFO("GameModel ready!");
}

void GameModel::readPoints() {
	std::cout << "Reading the points...\n";

	std::vector<geometry_msgs::Point> points_arr;

	std::ifstream infile;
	std::string path = ros::package::getPath("maze_game").append("/include/points.txt");
	std::cout << path.c_str() << "\n";
	infile.open(path.c_str());

	if (!infile.is_open()) {
		std::cerr << "Error! Could not open points file";
		return;
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

	//Set up the visualization markers
	ROS_INFO("Building visualization markers...");
	this->marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	this->maze_points.header.frame_id = "/map";
	this->maze_points.header.stamp = ros::Time::now();
	this->maze_points.ns = "maze_walls";
	this->maze_points.action = visualization_msgs::Marker::ADD;
	this->maze_points.pose.orientation.w = 1.0;
	this->maze_points.id = 0;
	this->maze_points.type = visualization_msgs::Marker::LINE_STRIP;
	this->maze_points.scale.x = 0.05;
	this->maze_points.scale.y = 0.05;
	this->maze_points.color.r = 1.0;
	this->maze_points.color.a = 1.0;

	for (uint32_t i = 0; i < points_arr.size(); i++) {
		expectedPoints.push_back(points_arr.at(i));
		this->maze_points.points.push_back(points_arr.at(i));

		if (i < points_arr.size() - 1) {
			this->maze_points.points.push_back(points_arr.at(i + 1));
		}
	}
}

void GameModel::readExpectedArrivalTimes() {
	ROS_INFO("Reading the arrival Times...");

	std::ifstream infile;
	std::string path = ros::package::getPath("maze_game").append("/include/expectedTimes.txt");
	std::cout << path.c_str() << "\n";
	infile.open(path.c_str());

	if (!infile.is_open()) {
		std::cerr << "Error! Could not open expected times file";
		return;
	}

	std::string line;

	while (std::getline(infile, line)) {

		std::stringstream ss(line);

		double time;

		//Read the coords in as float (they must be space delimited!)
		ss >> time;

		this->expectedArrivalTimes.push_back(time);
	}
}


void GameModel::addRobot(std::string name) {
	printf("Adding robot: %s\n", name.c_str());
// Commented out for testing without creating new robot objects, DO NOT DELETE!

	Robot* toAdd = new Robot(name, &nh, this);
	robots.insert(std::make_pair(name, *toAdd));

	//List robots
	int i = 0;
	std::cout << "---------- Robots List ----------\n";
	for(auto &entry : this->robots) {
		printf("%i) %s\n", i, entry.first.c_str());
		i++;
	}
}

void GameModel::removeRobot(std::string name) {
	printf("Removing robot %s\n", name.c_str());

	this->robots.erase(name);

	//List robots
	int j = 0;
	std::cout << "---------- Robots List ----------\n";
	for(auto &entry : this->robots) {
		printf("%i) %s\n", j, entry.first.c_str());
		j++;
	}
}

void GameModel::startRobot(std::string robotName) {


	MoveBaseClient ac("/" + robotName + "/move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for move base action server...");
	}

	this->robots.at(robotName).setState(RobotState::MOVING_TO_START);

	move_base_msgs::MoveBaseGoal msg;

	msg.target_pose.header.frame_id = "map";
	msg.target_pose.header.stamp = ros::Time::now();

	msg.target_pose.pose.position.x = 6.730;
	msg.target_pose.pose.position.y =  6.0;
	msg.target_pose.pose.position.z  = 0;
	msg.target_pose.pose.orientation.x = 0;
	msg.target_pose.pose.orientation.y  = 0;
	msg.target_pose.pose.orientation.z = .993607;
	msg.target_pose.pose.orientation.w = -0.112894;

	ROS_INFO("Sending %s to start of the maze...", robotName.c_str());
	ac.sendGoal(msg);
	ac.stopTrackingGoal();
}

void GameModel::restartRobot(std::string robot_name) {

	//Check for the robots existence
	std::map<std::string, Robot>::iterator it = this->robots.find(robot_name);

	if (it != this->robots.end()) {
		this->robots.at(robot_name).resetRobot();
		ROS_INFO("Successfully reset %s", robot_name.c_str());
	} else {
		ROS_WARN("No robot names %s to reset. ", robot_name.c_str());
	}
}

Robot* GameModel::getRobot(std::string name) {

	return &robots.at(name);
}

void GameModel::checkRobotsPosition() {

//	ROS_INFO("Checking robots from GameModel.");

	for (auto &entry : this->robots) {

		//This is not executed, seems to crash while adding a new robot...
//		ROS_INFO("Checking robot %s", entry.first.c_str());

		if (entry.second.getCurState() == RobotState::MOVING_TO_START || entry.second.getCurState() == RobotState::RUNNING_MAZE) {
//			ROS_INFO("%s has started, checking distance.", entry.first.c_str());
			entry.second.checkDistance();
		}
	}
}

geometry_msgs::Point GameModel::getExpPointAt(int i) {
	return expectedPoints.at(i);
}

std::pair<ros::Duration, int32_t> GameModel::getAverageScores() {
	std::pair<ros::Duration, int32_t> toReturn = std::make_pair(this->averageRunTime, this->averageHealth);
	return toReturn;
}

void GameModel::submitAndUpdateAverageScores(std::string name, std::vector<double> times, std::vector<int32_t> healthPoints, std::vector<double> percentDamage, std::vector<geometry_msgs::Point> givenPoints, std::vector<geometry_msgs::Point> expectedPoints, std::vector<geometry_msgs::Point> disparity, std::vector<double> inputDelay, std::vector<double> trustScores, std::map<int, std::vector<double>> goalFrequencyForPoints) {

	//Record run completion
	this->runDataVector.push_back(RunData(name, false, times, healthPoints, percentDamage, givenPoints, expectedPoints, disparity, inputDelay, trustScores, goalFrequencyForPoints));

	//Compute average duration
	this->averageRunTimesVector.push_back(ros::Duration(times.back() - times.front()));
	this->averageRunTime = ros::Duration((this->averageRunTime + this->averageRunTimesVector.back()).toSec() / this->averageRunTimesVector.size());

	//Compute average health
	this->averageHealthVector.push_back(healthPoints.back() - healthPoints.front());
	this->averageHealth = (this->averageHealth + this->averageHealthVector.back()) / this->averageHealthVector.size();
}

std::string processDoubleValue(double val) {

	std::stringstream ss;

	if (val == -86401.0) {
		return std::string("*");
	} else {
		ss << val;
		return ss.str();
	}
}

void GameModel::computeValuesForDataSample(GameModel::RunData& data) {

	for (int i = data.disparity.size() - 1; i >= 0; i--) {

		//Fix first row
		if (i == 0) {
			data.goalArivalTimes[i] -= data.goalArivalTimes[0];
			data.percentDamage[i] = 0.0;
			data.goalArrivalDelay[i] = 0.0;
			break;
		}

		//Compute arrival times
		data.goalArivalTimes[i] -= data.goalArivalTimes[0];
		printf("Comp. goalArrivalTime @ %i: %.6f\n", i, data.goalArivalTimes[i]);

		//Compute Percent Damage Values
		data.percentDamage[i] = double((data.healthAtGoal[0] - data.healthAtGoal[i]) / data.healthAtGoal[0]);
		printf("Comp. percentDamage @ %i: %.6f\n", i, data.percentDamage[i]);

		//Compute Goal arrival delay
		data.goalArrivalDelay[i] = this->expectedArrivalTimes[i] - data.goalArivalTimes[i];
		printf("Comp. goalArrivalDelay @ %i: %.6f\n", i, data.goalArrivalDelay[i]);

		//Compute trust score (unnormalized trust value)
		data.trustScores[i] = data.disparity[i].z + data.percentDamage[i] + data.goalArrivalDelay[i];
		printf("Comp. trustScore @ %i: %.6f\n", i, data.trustScores[i]);
	}
}

bool GameModel::exportRunData(std::string robotName) {

	//Vector to contain the robot(s) that we are asked to export data for
	std::vector<RunData> toExport;

	ROS_INFO("Exporting data for %s...", robotName.c_str());

	//Check to see what we need to export
	if (robotName != "ALL") {

		//Check for the robots existence (this is redundant I realize).
		std::vector<RunData>::iterator it = std::find_if(this->runDataVector.begin(), this->runDataVector.end(),
				[&robotName](const RunData& data) {
					return data.robotName.compare(robotName) == 0;
				});

		if (it != this->runDataVector.end()) {
			for (RunData dat : this->runDataVector) {
				if (dat.robotName.compare(robotName) == 0) {
					toExport.push_back(dat);
				}
			}
		}
	} else {
		toExport = this->runDataVector;
	}

	//File to export to
	std::ofstream outFile;

	for (RunData data : toExport) {

		computeValuesForDataSample(data);

		std::string path = ros::package::getPath("maze_game");
		path.append(std::string("/export/" + std::to_string(data.timestamp) + "_" + data.robotName + ".csv"));
		outFile.open(path.c_str(), std::ios_base::app);

		if (!outFile.is_open()) {
			ROS_ERROR("Could not open output path at: %s", path.c_str());
			return false;
		}

		std::string csvToExport = "Robot,Runtime,Run Before,Arrival Time,Health at Goal,Percent Damage,Given x,Given y,Expected x,Expected y,Disparity,Goal Arrival Delay,Trust Value\n";

		//Normalize the trust values (start at index 1 since we do not wish to include the first scores as it will always be 1)
		double maxTrust = data.trustScores[1];
		double minTrust = data.trustScores[1];

		for (double e : data.trustScores) {
			if (e != -86401.0) {

				if (e > maxTrust) {
					maxTrust = e;
				} else if (e < minTrust) {
					minTrust = e;
				}

			}
		}

//		maxTrust = *std::max_element(data.trustScores.begin(), data.trustScores.end());
//		minTrust = *std::min_element(data.trustScores.begin(), data.trustScores.end());

		ROS_INFO("maxTrust: %.6f, minTrust: %.6f", maxTrust, minTrust);

		for (int i = 0; i < data.trustScores.size(); i++) {
			data.trustScores[i] = ((data.trustScores[i] - minTrust)/ (maxTrust - minTrust));
		}

		for (int i = 0; i < data.trustScores.size(); i++) {

			std::string curRow = "";

			//Some values only have actual data on the first row
			if (i == 0) {
				curRow.append(data.robotName
							  + ","
							  + std::to_string(data.goalArivalTimes.back() - data.goalArivalTimes.front())
							  + ","
							  + (data.runMazeBefore ? "True" : "False")
							  + ","
						);
			} else {
				curRow.append("*,*,*,");
			}

			curRow.append(processDoubleValue(data.goalArivalTimes[i]) + ",");

			if (data.healthAtGoal[i] == INT32_MIN) {
				curRow.append("*,");
			} else {
				curRow.append(std::to_string(data.healthAtGoal[i]) + ",");
			}

			curRow.append(processDoubleValue(data.percentDamage[i]) + ",");
			curRow.append(processDoubleValue(data.givenPoints[i].x) + ",");
			curRow.append(processDoubleValue(data.givenPoints[i].y) + ",");
			curRow.append(processDoubleValue(data.expectedPoints[i].x) + ",");
			curRow.append(processDoubleValue(data.expectedPoints[i].y) + ",");
			curRow.append(processDoubleValue(data.disparity[i].z) + ",");
			curRow.append(processDoubleValue(data.goalArrivalDelay[i]) + ",");
			curRow.append(processDoubleValue(data.trustScores[i]) + "\n");

			csvToExport.append(curRow);
		}

		outFile << csvToExport;

		ROS_INFO("Finished export for %s!", robotName.c_str());
	}

	//Clear the runData we just exported?

	return true;
}

void GameModel::publishMazeMarkers() {

	this->marker_pub.publish(this->maze_points);
}
