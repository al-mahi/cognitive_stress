#include "Robot.h"
#include "GameModel.h"
#include "franticness.h"

Robot::Robot(std::string robotName, ros::NodeHandle *n, GameModel *modelPtr) {

    //Configure ROS Node
    this->public_nh = ros::NodeHandle(robotName);

    //Perform additional setup
    this->name = robotName;
    this->model = modelPtr;

    ROS_INFO("subscribing to /%s/move_base/current_goal\n", name.c_str());
    this->sub = public_nh.subscribe<geometry_msgs::PoseStamped>("/" + name + "/move_base/current_goal", 10, &Robot::newGoalCallback, this);

    this->curGoalIndex = 0;
    this->health = 1000;

    //Distance from centerline (walls) variables
    this->distance = 0,
            this->oldDistance = 0;

    this->curGoal = model->getExpPointAt(0);
    this->curState = RobotState::IDLE;
    this->started = false;
    this->lastGoalGivenAt = ros::Time(0);
//	this->goalArivalTimestamps.push_back(0.0);

    this->submitScoresService = this->public_nh.advertiseService("/maze_game/submit_scores_" + robotName,
                                                                 &Robot::submitScores, this);

    //Check for parameters
    if (this->public_nh.hasParam(robotName + "/goal_tolerance")) {
        ROS_INFO("%s had parameter 'goal_tolerance' in public node handle", robotName.c_str());
        this->public_nh.getParam(robotName + "/goal_tolerance", this->goal_tolerance);
    } else if (this->private_nh.hasParam("goal_tolerance")) {
        ROS_INFO("%s had parameter 'goal_tolerance' in private handle", robotName.c_str());
        this->private_nh.getParam("goal_tolerance", this->goal_tolerance);
    } else {
        ROS_INFO("%s did not have a parameter name goal_tolerance! Defaulting to .07!", robotName.c_str());
        this->goal_tolerance = .05;
    }

    //Set up goalFrequency map
    for (int i = 0; i < this->model->expectedPoints.size(); i++) {
        this->goalFrequencyForPoints.insert(std::make_pair(i, std::vector<double>()));
    }

    // Publish time remaining
    ROS_INFO("Trying to publish time!");
    this->timePublisher = this->public_nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    this->timeRemaining = 120;
    this->timeRemainingUpdateTimer = this->public_nh.createTimer(ros::Duration(1.0), &Robot::updateTimeRemaining, this);
    this->timeRemainingUpdateTimer.stop();

    // Set up the time to display

    geometry_msgs::Point displayLocation;
    displayLocation.x = 1.65;
    displayLocation.y = 2.0;
    displayLocation.z = 0.5;

    this->displayTime.header.frame_id = "/map";
    this->displayTime.header.stamp = ros::Time().now();
    this->displayTime.ns = "timer";
    this->displayTime.id = 0;
    this->displayTime.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    this->displayTime.action = visualization_msgs::Marker::ADD;
    this->displayTime.text = "Time Remaining:" + std::to_string(this->timeRemaining);
    this->displayTime.pose.position = displayLocation;
    this->displayTime.pose.orientation.x = 0.0;
    this->displayTime.pose.orientation.y = 0.0;
    this->displayTime.pose.orientation.z = 0.0;
    this->displayTime.pose.orientation.w = 1.0;
    this->displayTime.scale.x = 1.0;
    this->displayTime.scale.y = 1.0;
    this->displayTime.scale.z = 0.3;
    this->displayTime.color.a = 1.0;
    this->displayTime.color.r = 1.0;
    this->displayTime.color.g = 0.0;
    this->displayTime.color.b = 0.0;
    this->timePublisher.publish(this->displayTime);
}

void Robot::updateTimeRemaining(const ros::TimerEvent &event) {

    Robot *ptr = this->model->getRobot(this->name);

    ptr->displayTime.action = visualization_msgs::Marker::MODIFY;
    ptr->timeRemaining--;
    ptr->displayTime.text = "Time Remaining: " + std::to_string(ptr->timeRemaining);
    ptr->timePublisher.publish(ptr->displayTime);
}

bool Robot::submitScores(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

    this->callSubmitScoresService();
    return true;
}

void Robot::resetRobot() {
    this->started = false;
    this->curState = RobotState::IDLE;
    this->curGoalIndex = 0;
    this->health = 10000;
    this->distance = 0.0;
    this->oldDistance = 0.0;
    this->curGoal = this->model->expectedPoints.at(0);
    this->givenPoints.clear();
    this->expectedPoints.clear();
    this->goalArivalTimestamps.clear();
    this->healthAtGoal.clear();
    this->percentDamage.clear();
    this->goalArrivalDelay.clear();
    this->goalFrequencyForPoints.clear();
    //--------added by Mahi---------
    this->clicksAtGoals.clear();
    this->mouseDistAtGoal.clear();
    this->ignorance_time.clear();
    //------------------------------
}

void Robot::setState(RobotState::ENUM state) {
    this->curState = state;
}

RobotState::ENUM Robot::getCurState() {
    return this->curState;
}

bool Robot::isStarted() {
    return this->started;
}

void Robot::checkDistance() {

    //See if the Robot is in the IDLE state, and do nothing if it is
    if (this->curState == RobotState::IDLE) {
        return;
    }

    geometry_msgs::PoseStamped pBase, pMap;
    pBase.header.frame_id = this->name + "/base_link";

    pBase.pose.position.x = 0.0;
    pBase.pose.position.y = 0.0;
    pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    tf::TransformListener tfListener;
    ros::Time current_transform = ros::Time::now();

    try {
        tfListener.waitForTransform(pBase.header.frame_id, "/map", ros::Time(0), ros::Duration(10.0));
        tfListener.getLatestCommonTime(pBase.header.frame_id, "/map", current_transform, NULL);
        pBase.header.stamp = current_transform;

        //transform the robot's pose, pBase, into the map frame, pMap. pMap will contain these coordinates
        //according to the TF data available at time "current_transform"
        tfListener.transformPose("/map", pBase, pMap);
    } catch (tf::TransformException &e) {
        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
    }

    distance = getDistanceToMazeWalls(curGoalIndex, pMap);

    if (distance >= oldDistance) {
        oldDistance = distance;
    }

    // Only perform the distance penalty check if the robot is actually running the maze
    if (distance > this->goal_tolerance && this->curState == RobotState::RUNNING_MAZE) {
        health -= 10;
        printf("%s: Ouch! ---- health: %i----\n", this->name.c_str(), this->health);
    }

    // See if we have reached the next goal
// ROS_INFO("%s is %.6f from goal x: %.3f, y: %.3f, z: %.3f", name.c_str(), sqrt( pow(curGoal.x - pMap.pose.position.x, 2.0) + pow(curGoal.y - pMap.pose.position.y, 2.0) ), curGoal.x, curGoal.y, curGoal.z);

    if (sqrt(pow(curGoal.x - pMap.pose.position.x, 2.0) + pow(curGoal.y - pMap.pose.position.y, 2.0)) <= .2) {

        //Save the information for this goal
        if (this->curState == RobotState::RUNNING_MAZE && this->recievedNewGoal) {

            this->goalArivalTimestamps.push_back(ros::Time::now().toSec());


            ROS_INFO("Goal arrival time: %.6f", this->goalArivalTimestamps.back());

            this->healthAtGoal.push_back(this->health);
            //----------added by Mahi-----------
            time_arrival_waypoints[this->name].push_back(ros::Time::now().toSec());
            this->clicksAtGoals.push_back(map_robot_clicks[this->name]);
            map_robot_clicks[this->name] = map_robot_clicks[this->name] - clicks_when_new_goal[this->name];
            this->mouseDistAtGoal.push_back(map_robot_moves[this->name]);
            map_robot_moves[this->name] = map_robot_moves[this->name] - moves_when_new_goal[this->name];
            if(save_ignorance_time != -1)
                this->ignorance_time.push_back(save_ignorance_time);
            else
                save_ignorance_time = 0;
            std::cout<< "Debug[M]: ignorance_vect: "; for(auto elem : ignorance_time) std::cout<< elem << " "; std::cout<<std::endl;
            //----------------------------------
            this->timeRemainingAtGoal.push_back(this->timeRemaining);
            this->recievedNewGoal = false;

        } else if (this->curState == RobotState::MOVING_TO_START) {

            this->goalArivalTimestamps.push_back(ros::Time::now().toSec());

            ROS_INFO("%s has started the maze. Start time %.2f", this->name.c_str(), this->goalArivalTimestamps.back());

            this->curState = RobotState::RUNNING_MAZE;
            this->curGoalIndex = 0;
            this->curGoal = this->model->getExpPointAt(this->curGoalIndex);
            this->recievedNewGoal = true;
            this->timeRemainingAtGoal.push_back(this->timeRemaining);
            this->estimatedGoalIndexForGoal.push_back(this->curGoalIndex);
            this->healthAtGoal.push_back(this->health);
            this->timeRemainingUpdateTimer.start();

            //----------added by Mahi-----------
            map_robot_clicks[this->name] = clicks_when_new_goal[this->name] = 0;
            map_robot_moves[this->name] = moves_when_new_goal[this->name] = 0;
            //----------------------------------
        }

        //Increment the curGoalIndex
//		ROS_INFO("%s has reached goal at pointDex = %i", this->name.c_str(), this->curGoalIndex);
//		curGoalIndex++;
//		ROS_INFO("%s's pointDex is now: %i", this->name.c_str(), this->curGoalIndex);

//		printf("PointDex: %i\n", pointDex);

        //Have we finished the maze?
        if (curGoalIndex == this->model->expectedPoints.size() - 1) {
            ROS_INFO("%s completed the maze!", this->name.c_str());

            this->curState = RobotState::FINISHED_MAZE;
            this->started = false;

            // Stop the timer countdown
            this->timeRemainingUpdateTimer.stop();

            //Submit the scores
            callSubmitScoresService();

            this->curState = RobotState::IDLE;
            return;
        }

        //Get the next goal from the model
//		curGoal = model->getExpPointAt(curGoalIndex);
        oldDistance = 0;
    }
}

void Robot::setGameModel(GameModel *modelVar) {

    this->model = modelVar;
}

std::string Robot::getName() {

    return this->name;
}

void Robot::addDispPoint(geometry_msgs::Point point) {

    disparity.push_back(point);
}

float Robot::getAverageDisp(float *returned) {

    for (int i = 0; i < disparity.size(); i++) {
        *returned += disparity[i].z;
    }

    return *returned;
}

void Robot::newGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    //------------added by Mahi-----------------------
    clicks_when_new_goal[this->name] = map_robot_clicks[this->name];
    moves_when_new_goal[this->name] = map_robot_moves[this->name];
    std::cout << "arrival: " << time_arrival_waypoints[this->name].size() << " newgoal time: " << time_newgoal_after_arrival[this->name].size() << std::endl;
    if(time_arrival_waypoints[this->name].size() == time_newgoal_after_arrival[this->name].size() + 1)
    {
        time_newgoal_after_arrival[this->name].push_back(ros::Time::now().toSec());
        // todo: find work around for pushing to vector in callback
//        this->ignorance_time.push_back(time_newgoal_after_arrival[this->name].back() - time_arrival_waypoints[this->name].back());
        save_ignorance_time = time_newgoal_after_arrival[this->name].back() - time_arrival_waypoints[this->name].back();
        printf("ignorance time = %f\n", save_ignorance_time);
    }
    //------------------------------------------------
    // This is needed to overcome a bizarre limitation within boost when binding a Class function as a callback.
    // The bug makes a new instance of that class instead
    // of passing a pointer to the instance we actually care about.
    Robot *ptr = model->getRobot(this->name);

//	if (ptr->curGoalIndex == 0) {
//		ptr->goalArivalTimestamps.push_back(ros::Time::now().toSec());
//		ptr->curState = RobotState::RUNNING_MAZE;
    //Add missing values
//		ptr->healthAtGoal.push_back(10000);
//		ptr->expectedPoints.push_back(model->expectedPoints.front());
//		ptr->givenPoints.push_back(model->expectedPoints.front());
//
//		geometry_msgs::Point zeroDisparity;
//		zeroDisparity.x = 0.0;
//		zeroDisparity.y = 0.0;
//		zeroDisparity.z = 0.0;
//		ptr->disparity.push_back(zeroDisparity);
//	}

    //Are we running the maze? If not, we don't need to do anything.
    if (ptr->getCurState() != RobotState::RUNNING_MAZE) {
        return;
    }

    //Indicate that we have a new goal
    ptr->recievedNewGoal = true;

    //Compare this time as with the time the last goal was given
    ros::Time markTime;
    double dtGoal = markTime.toSec() - ptr->lastGoalGivenAt.toSec();
    ptr->lastGoalGivenAt = markTime;
    ptr->goalFrequencyForPoints[ptr->curGoalIndex].push_back(dtGoal);

    geometry_msgs::Point toAdd; //The disparity point to add to the list of disparity points in this->dispPoints
    GameModel::GoalAndIndexQuery *resp = model->getClosestGoalAndPathIndex(msg->pose.position, ptr->curGoalIndex);
    geometry_msgs::Point expectedPoint = resp->estimatedGoal;
    ptr->curGoal = resp->estimatedGoal;
    ptr->curGoalIndex = resp->index;
    ptr->estimatedGoalIndexForGoal.push_back(resp->index);

    // Add the health at this goal
    this->healthAtGoal.push_back(this->health);

    printf("Estimated goal for %s is %i\n", ptr->name.c_str(), ptr->curGoalIndex);

    // Add this point to givenPoints and add the expected point to expectePoints
    ptr->givenPoints.push_back(msg->pose.position);
    ptr->expectedPoints.push_back(expectedPoint);

    //Store the disparity data in toAdd
    toAdd.x = (expectedPoint.x - msg->pose.position.x);
    toAdd.y = (expectedPoint.y - msg->pose.position.y);
    toAdd.z = sqrt(pow(toAdd.x, 2.0) + pow(toAdd.y, 2.0));

//	ROS_INFO("Expected (%.6f, %.6f) got (%.6f, %.6f)", expectedPoint.x, expectedPoint.y, msg->pose.position.x, msg->pose.position.y);
//	printf("Calculated Disparity: x: %.6f, y: %.6f, total: %.6f\n", toAdd.x, toAdd.y, toAdd.z);
    ptr->addDispPoint(toAdd);
//	double avgDisp = 0;
//	for (int i = 0; i < ptr->disparity.size(); i++) {
//		avgDisp += ptr->disparity[i].z;
//	}

//	printf("Average disparity so far: %.6f\n", avgDisp);
    checkDistance();
}

double Robot::getStartTime() {
    if (this->curState == RobotState::RUNNING_MAZE) {
        return this->goalArivalTimestamps.front();
    } else {
        return -1.0;
    }
}

ros::Duration Robot::getRunDuration() {

    if (this->curState == RobotState::RUNNING_MAZE) {
        return ros::Duration(this->goalArivalTimestamps.back() - this->goalArivalTimestamps.front());
    } else {
        return ros::Duration(-1.0);
    }
}

double Robot::getDistanceToMazeWalls(int i, geometry_msgs::PoseStamped &p3transform) {

    geometry_msgs::Point p1, p2, p3;
    double distance, numerator, denominator;

    if (i <= 0) {
        i = 1;
    } else if (i >= model->expectedPoints.size()) {
        i = (model->expectedPoints.size() - 1);
    }

    p1 = model->getExpPointAt(i - 1);
    p2 = model->getExpPointAt(i);

    p3.x = p3transform.pose.position.x;
    p3.y = p3transform.pose.position.y;
    p3.z = p3transform.pose.position.z;

    //ensure that we are able to compute the distance to the line segment
    if (p1.x == p2.x) {
        if (p3.y < fmin(p1.y, p2.y) || p3.y > fmax(p1.y, p2.y)) {
            return -1.0;
        }
    } else {
        if (p3.x < fmin(p1.x, p2.x) || p3.x > fmax(p1.x, p2.x)) {
            return -1.0;
        }
    }

    //Formulas
    // distance = |(y2 - y1)p3.x - (x2 - x1)p3.y + x2*y1 - x1*y2| / sqrt( (y2 - y1)^2 + (x2 - y1)^2)
    //distance = ( fabs( (p2.y - p1.y) * p3.getOrigin().x() - (p2.x - p1.x) * p3.getOrigin().y() + p2.x*p1.y - p2.y*p1.x ) / sqrt( pow(p2.y - p1.y, 2.0) + pow(p2.x - p1.x, 2.0) ) );

    numerator = fabs((p2.y - p1.y) * p3.x - (p2.x - p1.x) * p3.y + p2.x * p1.y - p2.y * p1.x);
    denominator = sqrt(pow(p2.y - p1.y, 2.0) + pow(p2.x - p1.x, 2.0));

    distance = numerator / denominator;

    return distance;
}

void Robot::callSubmitScoresService() {

    Robot *robotPtr = model->getRobot(this->name);

    std::vector<uint64_t> lengths = {robotPtr->healthAtGoal.size(), robotPtr->givenPoints.size(),
                                     robotPtr->expectedPoints.size(), robotPtr->goalArrivalDelay.size(),
                                     robotPtr->trustScores.size(), robotPtr->timeRemainingAtGoal.size(),
                                     robotPtr->clicksAtGoals.size(), robotPtr->mouseDistAtGoal.size()};

    for (int i = 0; i < lengths.size(); i++) {
        printf("lengths @ %i: %lu\n", i, lengths[i]);
    }

    int maxLength = *std::max_element(lengths.begin(), lengths.end());
    printf("maxLength: %i\n", maxLength);

    //Make goal arrival time stamps relative to when the robot started the maze
//	for (int i = 0; i < this->goalArivalTimestamps.size(); i++) {
//		this->goalArivalTimestamps[i] -= this->goalArivalTimestamps[0];
//	}

    if (robotPtr->goalArivalTimestamps.size() < maxLength) {
        printf("goalArivalTimeStamps size: %lu. Resizing...\n", goalArivalTimestamps.size());
        for (int i = robotPtr->goalArivalTimestamps.size() - 1; i < maxLength; i++) {
            robotPtr->goalArivalTimestamps.push_back(-86401.0); //24 * 60 * 60 - 1;
        }
    }

    if (robotPtr->healthAtGoal.size() < maxLength) {
        printf("healthAtGoal size: %lu. Resizing...\n", healthAtGoal.size());
        for (int i = robotPtr->healthAtGoal.size() - 1; i < maxLength; i++) {
            robotPtr->healthAtGoal.push_back(INT32_MIN);
        }
    }

    if (robotPtr->percentDamage.size() < maxLength) {
        printf("percentDamage size: %lu. Resizing...\n", percentDamage.size());
        for (int i = robotPtr->percentDamage.size() - 1; i < maxLength; i++) {
            robotPtr->percentDamage.push_back(-86401.0);
        }
    }

    geometry_msgs::Point nullPoint;
    nullPoint.x = -86401.0;
    nullPoint.y = -86401.0;
    nullPoint.z = -86401.0;

    if (robotPtr->givenPoints.size() < maxLength) {
        printf("givenPoints size: %lu. Resizing...\n", givenPoints.size());
        for (int i = robotPtr->givenPoints.size() - 1; i < maxLength; i++) {
            robotPtr->givenPoints.push_back(nullPoint);
        }
    }

    if (robotPtr->expectedPoints.size() < maxLength) {
        printf("expectedPoints size: %lu. Resizing...\n", expectedPoints.size());
        for (int i = robotPtr->expectedPoints.size() - 1; i < maxLength; i++) {
            robotPtr->expectedPoints.push_back(nullPoint);
        }
    }

    if (robotPtr->goalArrivalDelay.size() < maxLength) {
        printf("inputDelay size: %lu. Resizing...\n", robotPtr->goalArrivalDelay.size());
        for (int i = robotPtr->goalArrivalDelay.size() - 1; i < maxLength; i++) {
            robotPtr->goalArrivalDelay.push_back(-86401.0);
        }
    }

    if (robotPtr->trustScores.size() < maxLength) {
        printf("trustScores size: %lu. Resizing...\n", robotPtr->trustScores.size());
        for (int i = robotPtr->trustScores.size() - 1; i < maxLength; i++) {
            robotPtr->trustScores.push_back(-86401.0);
        }
    }

    std::vector<double> nullVector = {-86401.0};
    if (robotPtr->goalFrequencyForPoints.size() < maxLength) {
        printf("goalFrequencyForPoints size: %lu. Resizing...\n", this->goalFrequencyForPoints.size());
        for (int i = robotPtr->goalFrequencyForPoints.size(); i < maxLength; i++) {
            robotPtr->goalFrequencyForPoints.insert(std::make_pair(i, nullVector));
        }
    }

    if (robotPtr->timeRemainingAtGoal.size() < maxLength) {
        printf("timeRemainingAtGoal size: %lu. Resizing...\n", robotPtr->timeRemainingAtGoal.size());
        for (int i = robotPtr->timeRemainingAtGoal.size() - 1; i < maxLength; i++) {
            robotPtr->timeRemainingAtGoal.push_back(INT32_MIN);
        }
    }

    if (robotPtr->estimatedGoalIndexForGoal.size() < maxLength) {
        printf("estimatedGoalIndexForGoal size: %lu. Resizing...\n", robotPtr->estimatedGoalIndexForGoal.size());
        for (int i = robotPtr->estimatedGoalIndexForGoal.size() - 1; i < maxLength; i++) {
            robotPtr->estimatedGoalIndexForGoal.push_back(INT32_MIN);
        }
    }

    robotPtr->model->submitAndUpdateAverageScores(robotPtr->name, robotPtr->goalArivalTimestamps,
                                                  robotPtr->healthAtGoal, robotPtr->percentDamage,
                                                  robotPtr->givenPoints, robotPtr->expectedPoints, robotPtr->disparity,
                                                  robotPtr->goalArrivalDelay, robotPtr->trustScores,
                                                  robotPtr->goalFrequencyForPoints, robotPtr->timeRemainingAtGoal,
                                                  robotPtr->estimatedGoalIndexForGoal,
                                                  //---------added by Mahi---------
                                                  robotPtr->clicksAtGoals, robotPtr->mouseDistAtGoal, robotPtr->ignorance_time);
                                                  //-------------------------------
}

ros::NodeHandle *Robot::getNodeHandlePtr() {
    return &this->public_nh;
}
