#ifndef GAMEMODEL_H
#define GAMEMODEL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <thread>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>

class Robot;

/**
 * \author Matthew Atkins
 *
 * This class represents the state of the "game". It should NOT handle any of the major logic apart from
 * what is needed to update the state of its components.
 */
class GameModel {

    ros::NodeHandle nh; ///< The node handle for this Maze Game
    std::map<std::string, Robot> robots; ///< Holds the Robot objects for the robots we are representing in the game keyed by their name
    std::vector<std::string> tempMap;
    ros::Publisher marker_pub; ///< Publishes the maze as a Visualization Marker message, used primarily by rosbridge
    visualization_msgs::Marker maze_points;

    //Data recording
    std::vector<ros::Duration> averageRunTimesVector; ///< Holds the average times
    ros::Duration averageRunTime;
    std::vector<int32_t> averageHealthVector;
    int32_t averageHealth;

    // Success score knobs
    const double kPercentDamage = 0.5,
            kDispairty = 0.75,
            kTimeRemaining = 0.01;


    //Action client
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    //Data Import/Export
    /**
     * Struct to represent an entry to export when a call to ExportRunData is made.
     */
    struct RunData {
        double timestamp; ///< Time stamp indicating when the data was saved by the robot
        std::string robotName; ///< Name of the robot that submitted the data
        bool runMazeBefore; ///< Flag indicating if the user has run the maze before
        std::vector<double> goalArivalTimes; ///< Time stamps for when the robot arrived a particular point, indexed by point
        std::vector<int32_t> healthAtGoal; ///< The health a robot had when it arrived a particular point in the maze, indexed by point
        std::vector<double> percentDamage;
        std::vector<geometry_msgs::Point> givenPoints; ///< Keeps track of the points that are given by the user
        std::vector<geometry_msgs::Point> expectedPoints; ///< Keeps track of the points that are expected by the robot
        std::vector<geometry_msgs::Point> disparity;
        std::vector<double> trustScores; ///< Vector of the unnormalized trust scores
        std::map<int, std::vector<double>> goalFrequencyForPoints;
        std::vector<int32_t> timeRemainingAtGoal;
        std::vector<int32_t> estimatedGoalIndexForGoal;
        //---------added by Mahi-----------------------
        std::vector<int32_t > clicksAtGoals;
        std::vector<double> mouseDistAtGoal;
        std::vector<double> ignorance_time;
        //---------------------------------------------

        /**
         * Constructs a new RunData object
         * @param robotName The name of the robot submitting the data
         * @param runMazeBefore A boolean to indicate if the user has run the maze before
         * @param goalArivalTimes The vector of goal arrival times
         * @param healthAtGoal The vector of health values at a particular goal
         */
        RunData(std::string robotName,
                bool runMazeBefore,
                std::vector<double> goalArivalTimes,
                std::vector<int32_t> healthAtGoal,
                std::vector<double> percentDamage,
                std::vector<geometry_msgs::Point> givenPoints,
                std::vector<geometry_msgs::Point> expectedPoints,
                std::vector<geometry_msgs::Point> disparity,
                std::vector<double> goalArrivalDelay,
                std::vector<double> trustScores,
                std::map<int, std::vector<double>> goalFrequencyForPoints,
                std::vector<int32_t> timeRemainingAtGoal,
                std::vector<int32_t> estimatedGoalIndexForGoal,
                //---------added by Mahi-----------------------
                std::vector<int32_t > clicksAtGoals,
                std::vector<double> mouseDistAtGoal,
                std::vector<double> ignorance_time
                //---------------------------------------------
        ) {

            this->timestamp = ros::Time().now().toSec();
            this->robotName = robotName;
            this->runMazeBefore = runMazeBefore;
            this->goalArivalTimes = goalArivalTimes;
            this->healthAtGoal = healthAtGoal;
            this->percentDamage = percentDamage;
            this->givenPoints = givenPoints;
            this->expectedPoints = expectedPoints;
            this->disparity = disparity;
            this->trustScores = trustScores;
            this->goalFrequencyForPoints = goalFrequencyForPoints;
            this->timeRemainingAtGoal = timeRemainingAtGoal;
            this->estimatedGoalIndexForGoal = estimatedGoalIndexForGoal;
            //-----------added by Mahi---------------------
            this->clicksAtGoals = clicksAtGoals;
            this->mouseDistAtGoal = mouseDistAtGoal;
            this->ignorance_time = ignorance_time;
            //---------------------------------------------
        }
    };

    std::vector<RunData> runDataVector; ///<Stores all the RunData objects since the last export.
    void readPoints();

    void readExpectedArrivalTimes();

public:

    struct GoalAndIndexQuery {

        int index;
        geometry_msgs::Point estimatedGoal;

        GoalAndIndexQuery(int index, geometry_msgs::Point estimatedGoal) {
            this->index = index;
            this->estimatedGoal = estimatedGoal;
        }
    };

    std::vector<geometry_msgs::Point> expectedPoints; ///< The points read using readPoints. They are used in the disparity calculation.
    std::vector<double> expectedArrivalTimes;

    /**
     * The default constructor.
     * @return An empty GameModel object with the given node handle
     */
    GameModel(ros::NodeHandle *n);

    /**
     * Creates and adds a new Robot object to the robots vector using the given string as its name
     * @param robotName The name for the new robot.
     */
    void addRobot(std::string robotName);

    /**
     * Removes the robot, if it exists, from the list of known robots.<br>
     * \note This does not remove the run data of the robot itself, just the object that represents it.
     *
     * @param robotName The name of the robot to remove
     */
    void removeRobot(std::string robotName);

    /**
     * Starts the robot with the given name and sends it to the start of the maze. This is the function that the service callback MazeGame::startRobot calls.
     * This also sets it's state to \link RobotState MOVEING_TO_START \endlink
     * When the checkDistance() function is called on this robot it will see if the robot has reached the start of the maze, if it has
     * the robot's state is update to \link RobotState RUNNING_MAZE \endlink .
     * @param robotName The name of the robot to start.
     */
    void startRobot(std::string robotName);

    /**
     * This will reset the robot with the given name to it's initialization values. \see Robot::restartRobot
     * @param robot_name
     */
    void restartRobot(std::string robot_name);

    /**
     * Get the Robot object with the given name or returns null if there is no such Robot
     * @param robotName The name of the Robot to retrieve
     * @return A pointer to the requested Robot object
     */
    Robot *getRobot(std::string robotName);

    /**
     * Iterates over all the robots currently in the 'robots' vector and gets their position in the maze.
     * It uses this to determine if the robot is close enough to a hazard (i.e. a wall) to penalize by determining the distance between the centerline
     * of the maze and the robots base. It also runs a check to see if the robot is close enough to it's next goal in the maze to give it the next goal.
     */
    void checkRobotsPosition();

    /**
     * Returns a pointer to the  point at the specified index as a geometry_msgs::Point object.
     * @param i The index of the point to retrieve
     */
    geometry_msgs::Point getExpPointAt(int i);

    /**
     * Computes and returns the average runtime across all runs and returns it as a ros::Duration instance
     * @return A pair containing the average runtime score first and the average health score second.
     */
    std::pair<ros::Duration, int32_t> getAverageScores();

    /**
     * Appends the given data to the list of entries for export.
     * @param name The name of the robot submitting the data
     * @param times The vector of goal arrival times
     * @param healthPonits The vector of health vales at a particular point
     * @param givenPoints The vector of points given by the user
     * @param expectedPoints The vector of points the robot was expecting
     */
    void submitAndUpdateAverageScores(std::string name, std::vector<double> times, std::vector<int32_t> healthPonits,
                                      std::vector<double> percentDamage, std::vector<geometry_msgs::Point> givenPoints,
                                      std::vector<geometry_msgs::Point> expectedPoints,
                                      std::vector<geometry_msgs::Point> disparity, std::vector<double> inputDelay,
                                      std::vector<double> trustScores,
                                      std::map<int, std::vector<double>> goalFrequencyForPoints,
                                      std::vector<int32_t> timeRemainingAtGoal,
                                      std::vector<int32_t> estimatedGoalIndexForGoal,
                                      //--------------added by Mahi------------------
                                      std::vector<int32_t> clicksAtGoals,
                                      std::vector<double> mouseDistAtGoal,
                                      std::vector<double> ignorance_time
                                      //---------------------------------------------
                                     );

    /**
     * Computes the raw distrust score for the given data sample. The result is stored inside the RunData object passed to it.
     * @param data The RunData object that contains the sample data.
     */
    void computeValuesForDataSample(RunData &data);

    /**
     * Exports the data currently stored in the runDataVector to individual CSV files to the 'export' folder in the package
     * @param robotName The name of the robot to export data for. If the name passed is 'ALL' then all available data will be exported.
     * @return True if the export was successful, false otherwise
     */
    bool exportRunData(std::string robotName);

    GoalAndIndexQuery *getClosestGoalAndPathIndex(geometry_msgs::Point goal, int curGoalIndex);

    /**
     * Publishes the maze as a visualization_msgs/Marker messages on the '' topic.
     */
    void publishMazeMarkers();
};

#endif
