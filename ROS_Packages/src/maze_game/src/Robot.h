#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseWithCovariance.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>
#include <maze_game/SubmitScores.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <visualization_msgs/Marker.h>

/*! \relates Robot
 * Enum Indicated the robots current state to help facilitate transitions from one operation to the next.
*/
namespace RobotState {
    enum ENUM : unsigned int {
        IDLE,           ///< Used to indicate that the robot is not doing anything and is ready to be re-tasked
        MOVING_TO_START, ///< Indicates that the robot is moving to the start of the maze
        RUNNING_MAZE,   ///< Indicates that the robot is running the maze and should not be interrupted
        FINISHED_MAZE   ///< Indicates that the robot has finished the maze. This is used primarily for score logging and a future re-queue mechanism
    };
}

class GameModel;

/**
 * \author Matthew Atkins
 *
 * This class is used to represent an individual robot in the current game.
 * The object interacts with ROS directly to update itself and should inform the MazeGame object
 * associated with this game about relevant events such as taking "damage". It does provide topics for
 * sending information, probably to rosbridge, about itself for things like updating the webUI.
 *
 * The class is designed to support multiple robots using multiple Robot object instances. This class IS NOT
 * designed to operate as an independent ROS node; a robot should be created with service call a running maze_game
 * node using the addRobot service. This service call requires a string robot_name that will be used to uniquely
 * identify a given robot. Each will be given their own ROS namespace to operate in to avoid accidentally updating
 * information incorrectly. The namespace is determine by the value of robot_name from the service call.
 *
 * For example, to use two robots at the same time, robot1 and robot2 respectively, then the following namespaces
 * will be created:
 *
 * robot1 -> robot1/move_base/goal<br>
 * robot2 -> robot2/move_base/goal
 *
 * If only one robot is to be used, a service call to "/maze_game/add_robot" with robot_name set to "" will result in no namespace being
 * prepended.
 */

class Robot {

private:
    std::string name; ///< The name of the robot (i.e. "miranda")
    ros::NodeHandle public_nh; ///< This Robot's public namespace NodeHandle
    ros::NodeHandle private_nh; ///< This Robot's private namespace NodeHandle
    ros::Subscriber sub; ///< Subscribes to '/ROBOT_NAME/move_base/current_goal' to receive new navigation goals

    RobotState::ENUM curState; ///< The robot's current state, initially set to IDLE.
    double distance; ///< The robot's distance from the center of the maze's path
    double oldDistance; ///< The previously observed distance from the center of the maze's path
    int health; ///< Used to determine how 'good' a run through the maze was and approximately how long a robot was in contact with an obstacle.
    geometry_msgs::Point curGoal; ///< The current goal used to calculate which path segment the distance calculation should be performed with respect to. Also used to compute the disparity between the robot's own notion of what is a 'good' goal and what the user actually give it.
    GameModel *model; ///< A pointer the GameModel to add this Robot instance to
    double goal_tolerance; ///< This is the value that check distance uses to see if the robot is within the target "zone" around it's current goal (stored in curGoal). It's important to know that this is the RADUIUS and not the diameter of the target zone.
    bool recievedNewGoal = false;

    //Score recording and submission
    std::vector <geometry_msgs::Point> givenPoints; ///< Keeps track of the points that are given by the user
    std::vector <geometry_msgs::Point> expectedPoints; ///< Keeps track of the points that are expected by the robot
    std::vector<double> goalArivalTimestamps; ///< Keeps track of when a Robot arrives at a certain goal/point in the maze. First point should be the time the maze was started, last should be the completion time.
    std::vector <int32_t> healthAtGoal; ///< Keeps track of what the robot's health was as it arrives at every goal.
    std::vector<double> percentDamage; ///< Keeps track of the amount of "damage" the robot has accumulated at each goal
    std::vector<double> goalArrivalDelay; ///< Stores the difference in seconds between the robots autonomous goal arrival times and the time the robot actually arrives at the goal
    std::vector<double> trustScores; ///< The UN-normalized trust scores for each input
    std::vector <int32_t> timeRemainingAtGoal;
    std::vector <int32_t> estimatedGoalIndexForGoal;

    //--------added by Mahi-----------
    std::vector<int32_t> clicksAtGoals; /// Keeps track of number of mouse clicks or what the user's franticness metric was as it arrives at every goal.
    std::vector<double> mouseDistAtGoal; /// Keeps track of distance movered by mouse cursor or what the user's franticness metric was as it arrives at every goal.
    std::vector<double> ignorance_time;  /// Keeps track of distance movered by mouse cursor or what the user's franticness metric was as it arrives at every goal.
    //--------------------------------

//	ros::Time arivedAtLastGoal;
    ros::Time lastGoalGivenAt;
    std::map<int, std::vector<double>> goalFrequencyForPoints;

    // Time remaining functions
    ros::Publisher timePublisher;
    ros::Timer timeRemainingUpdateTimer;
    int timeRemaining;
    visualization_msgs::Marker displayTime;

    void updateTimeRemaining(const ros::TimerEvent &event);

public:

    ros::ServiceServer submitScoresService;

    bool submitScores(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    //TODO: Remove started and related functions as it has been superseded by RobotState.
    bool started; ///< Has the robot started the maze?
    std::vector <geometry_msgs::Point> disparity; ///< The disparity between the expected point and the point given by the user
    int curGoalIndex; ///< Index for the expected point of the robots current goal

    /**
     * Constructs a new Robot object with the given name and returns it
     * @param robotName The name to assign to this robot. It must be unique to this game.
     */
    Robot(std::string robotName, ros::NodeHandle *n, GameModel *modelPtr);

    /**
     * Resets a robot to it's initialization conditions.
     */
    void resetRobot();

    /**
     * Sets the robot's current state (i.e. IDLE or MOVING_TO_START).
     * @param state The state to assign to the robot.
     */
    void setState(RobotState::ENUM state);

    /**
     * Returns the current state of the robot.
     * @return The robot's current state.
     */
    RobotState::ENUM getCurState();

    //TODO: Remove this function as it has been superseded by RobotState
    /**
     * \deprecated This will be removed in future updates, use \link RobotState::ENUM RobotState\endlink instead
     * A flag indicating the status of the maze run
     * @return True if the robot has started the maze
     */
    bool isStarted();

    /**
     * This function is called every time the robot receives a new nav-goal and on the interval set by MazeGame's spin thread sleep duration.
     * It does the bulk of the work for this class including performing the distance check that determines how far off the path the robots is.
     * It also checks the robot's current state to determine if the robot should be penalized for coming into contact with obstacles or straying to far from the path.
     * If the robot is in the MOVING_TO_START or RUNNING_MAZE states it also checks how close the robot is to the next goal in the maze (stored in curGoal). If below a certain threshold, it
     * will see if the robot has reached the end of the maze and either provide the next goal if it has not or stop the robot and submit the robot's score to GameModel with a call to callSubmitScoresService().
     */
    void checkDistance();

    /**
     * This function sets the pointer that contains this Robot object
     *
     * @param modelVar The pointer storing the location of this Robot's GameModel object
     */
    void setGameModel(GameModel *modelVar);


    /**
     * Returns the name given to this robot during it's initialization.
     * @return The name of this robot.
     */
    std::string getName();

    /**
     * Appends the disparity to the disparity vector.
     * @param dPoint The pointer to the Point to add
     */
    void addDispPoint(geometry_msgs::Point point);

    /**
     * Computes the average disparity (error) for all the points so far
     * @return The average error for all the points given by the user vs the expected points
     */
    float getAverageDisp(float *returned);

    /**
     * Callback that calculates the disparity between the expected point stored in the GameModel
     * against the point given by the user and stores it. It is called every time a new pose is sent
     * by the user.
     *
     * @param msg The pose of the robot from the listener
     */
    void newGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    /**
     * Get the time that robot started its current run through the maze as a ros::Time instance
     * @return Time the maze was started
     */
    double getStartTime();

    /**
     * Returns the time the robot has taken to get to where it is in the maze. If the robot has finished the maze then this is the runtime.
     * @return The time the current run has taken so far or the duration of the entire run if the maze has been completed.
     */
    ros::Duration getRunDuration();

    /**
     * This function calculates the normal distance from the robot to a maze wall given by two points. This two points
     * are determined by the given index, i.
     *
     * @param i The index of the point to look up in the expected points from GameModel
     * @param p3transform The transform frame given by transforming the robots pose in the map frame
     * @return The distance between the robot and the given line
     */
    double getDistanceToMazeWalls(int i, geometry_msgs::PoseStamped &p3);

    /**
     * Currently makes a direct call to the GameModel associated with this robot and submits the robot's data from this run.
     */
    void callSubmitScoresService();

    /**
     * Get's the ROS node handle used by this robot
     * @return The ROS node handle used by this robot.
     */
    ros::NodeHandle *getNodeHandlePtr();

};

#endif
