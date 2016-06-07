#ifndef MAZEGAME_H
#define MAZEGAME_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <stdio.h>
#include "maze_game/AddRobot.h"
#include "maze_game/RemoveRobot.h"
#include "maze_game/StartRobot.h"
#include "maze_game/RequestMazeMarkers.h"
#include "maze_game/RequestAverageScores.h"
#include "maze_game/SubmitScores.h"
#include "maze_game/ExportRunData.h"
#include "GameModel.h"
#include "Robot.h"

/**
 * \author Matthew Atkins
 *
 * This is main ROS node and performs the majority of the interactions between the MazeGame code and ROS itself.<br>
 * It provides the following services:<br>
 * <ul>
 * <li>add_robot: adds a robot with the name given in the service request.</li>
 * <li>remove_robot: removes the robot with the name given in the service request.</li>
 * <li>start_robot: sends the robot to the start of the maze and sets its state to MOVING_TO_START click \link RobotState here \endlink for more info.</li>
 * <li>restart_robot: resets the robot with the given name back to it's initialization values and set it's state to \link RobotState::IDLE IDLE\endlink </li>
 * <li>request_maze_markers: causes the maze's point to be published as visualization_msgs/Marker message on the /maze_game/markers topics</li>
 * <li>request_average_scores: </li>
 * <li>submit_scores: </li>
 * <li>export_run_data: exports the data for the robot with the given name or all available data if the name given is 'ALL'. The data is stored as a CSV file in the packages 'export' directory. Actual export is done \link GameModel::exportRunData here\link</li>
 * </ul>
 *
 * More info.
 */

class MazeGame {
    ros::NodeHandle nh; ///< Pointer that should be shared across all GameModels and Robot objects involved in this MazeGame

public:
    GameModel * model; ///< Pointer to the model used by this MazeGame

    /**
     * MazeGame constructor
     * @param n The ros::NodeHandle to give this MazeGame instance
     */
    MazeGame(ros::NodeHandle *n);

    void startNode(int argc, char** argv);

    /**
     * Service that is called to add to to the game, this creates a new Robot instance and adds it to the model. \see GameModel::addRobot
     * @param req ROS service request with the field 'robotName' that contains the name of the robot to add.
     * @param res ROS service response that contains a boolean field 'result' that indicates the success or failure of the operation.
     * @return
     */
    bool addRobot(maze_game::AddRobot::Request &req, maze_game::AddRobot::Response &res);

    /**
	 * Service that is called to remove a robot instance from the game, the robot is also remove from the model. \see GameModel::addRobot
	 * @param req ROS service request with the field 'robotName' that contains the name of the robot to remove.
	 * @param res ROS service response that contains a boolean field 'result' that indicates the success or failure of the operation.
	 * @return
    */
    bool removeRobot(maze_game::RemoveRobot::Request &req, maze_game::RemoveRobot::Response &res);

    /**
     * Service that is called to start a robot and send it to the start of the maze. \link GameModel::start_robot More info here.\endlink
     * @param req ROS service request with the field 'robotName' that contains the name of the robot to start.
     * @param res ROS service response that contains a boolean field 'result' that indicates the success or failure of the operation.
     * @return True if successful False otherwise
     */
    bool startRobot(maze_game::StartRobot::Request &req, maze_game::StartRobot::Response &res);

//    /**
//	 * Service that is called to restart a robot and send it to the start of the maze. \link GameModel::restart_robot More info here.\endlink
//	 * @param req ROS service request with the field 'robotName' that contains the name of the robot to restart.
//	 * @param res ROS service response that contains a boolean field 'result' that indicates the success or failure of the operation.
//	 * @return True if successful False otherwise
//	*/
    //bool restartRobot(std_msgs::String &req, std_msgs::String &res);

    /**
     * Service that is called to have the GameModel publish the maze markers. \see GameModel::publishMazeMarkers
     * @param req ROS service request with the field 'request' that can be any string, it does not affect the result.
     * @param res ROS service response that contains a boolean field 'result' that indicates the success or failure of the operation.
     * @return True if successful False otherwise
     */
    bool requestMazeMarkers(maze_game::RequestMazeMarkers::Request &req, maze_game::RequestMazeMarkers::Response &res);

    /**
     * Service that is called to retrieve the average run scores so far. \see GameModel::getAverageScores
     * @param req ROS service request with the field 'robotName' that can be any string, it does not affect the result.
     * @param res ROS service response with the fields: avgRuntime and avgDamage which contain the average runtime and damage values respectively for all runs through the maze to date.
     * @return True if successful False otherwise
     */
    bool requestAverageScores(maze_game::RequestAverageScores::Request &req, maze_game::RequestAverageScores::Response &res);

    /**
     * Service that is called to submit the data collected by a robot for a single run through the maze. \see GameModel::submitScores
     * @param req ROS service request with the fields:
     * <ul>
     * 		<li>robot_name: The name of the robot</li>
     * 		<li>goal_arival_times: A vector containing that contains the times the robot arrived at a particular goal.</li>
     * 		<li>damage_at_goal: A vector containing the 'health' of the robot at each goal.</li>
     * 		<li>given_points: A vector containing the points given by the user's navigation goals.</li>
     * 		<li>expected_points: A vector containing the points that the robot was expecting.</li>
     * </ul>
     * @param res ROS service response that contains a boolean field 'result' that indicates the success or failure of the operation.
     * @return True if successful False otherwise
     */
    bool submitScores(maze_game::SubmitScores::Request &req, maze_game::SubmitScores::Response &res);

    /**
     * Service that is called to export the run data accumulated so far for the robot with the given name. \see GameModel::exportRunData
     * @param req ROS service request with the field 'robotName' that contains the name of the robot to export data for. If this is set to 'ALL' then all available data will be exported.
     * @param res ROS service response that contains a boolean field 'result' that indicates the success or failure of the operation.
     * @return True if successful False otherwise
     */
    bool exportRunData(maze_game::ExportRunData::Request &req, maze_game::ExportRunData::Response &res);
};

#endif // MazeGame
