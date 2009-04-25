/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <ros/node.h>
#include <boost/thread/mutex.hpp>

#include <mechanism_model/controller.h>
#include <robot_mechanism_controllers/joint_pd_controller.h>

// Services
#include <robot_msgs/JointTraj.h>
#include <robot_msgs/JointTrajPoint.h>
#include <robot_msgs/DiagnosticMessage.h>

#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>
#include <pr2_mechanism_controllers/TrajectoryCancel.h>
#include <pr2_mechanism_controllers/base_controller.h>

//Kinematics
#include <trajectory/trajectory.h>
#include <robot_msgs/ControllerState.h>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_tools.h>
#include <std_msgs/String.h>

#include <angles/angles.h>

namespace controller
{

    const std::string JointTrajectoryStatusString[8] = {"0 - ACTIVE","1 - DONE","2 - QUEUED","3 - DELETED","4 - FAILED","5 - CANCELED","6 - DOES_NOT_EXIST","7 - NUM_STATUS"};

#define GOAL_REACHED_THRESHOLD 0.01
#define MAX_ALLOWABLE_JOINT_ERROR_THRESHOLD 0.2
  // comment this out if the controller is not supposed to publish its own max execution time
#define PUBLISH_MAX_TIME


/** @class JointTrajectoryController
 *  @brief ROS interface for a joint trajectory controller.
 *  @author Sachin Chitta <sachinc@willowgarage.com>
 *
 *  This class provides a ROS interface for controlling a set of joints by setting position configurations. If offers several ways to control the joints:
 *  - through listening to ROS messages: this is specified in the XML configuration file by the following parameters:
 *      <listen_topic name="the name of my message" />
 *      (only one topic can be specified)
 *  - through a non blocking service call: this service call can specify a single trajectory as a goal
 *
 */
  class JointTrajectoryController : public Controller
  {
    public:

    /**
     * @brief Default Constructor of the JointController class.
     *
     */
    JointTrajectoryController();

    /*!
     * \brief Destructor of the JointController class. Stops publishers and services.
     */
    virtual ~JointTrajectoryController();

    /*!
     * @brief Initialize the controller using an Xml configuration and a robot object.
     * @param robot pointer to a robot object passed in by all controllers.
     * @config TiXml configuration element
     */
    bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

    /*!
     * @brief Issues commands to the joints and is called at regular intervals in realtime. This function is required 
     * to be realtime safe.
     */
    virtual void update(void); 

    virtual bool starting(void);

    private:

    std::string prefix_; /**< prefix (initialized to controller name + "/") for all controller & ROS interaction */

    boost::mutex arm_controller_lock_; /**< Mutex lock for sharing information between services and realtime */

    double last_time_; /**< last time update was called */

    std::vector<control_toolbox::Pid> base_pid_controller_;      /**< Internal PID controllers for controlling the base. */

    std::vector<int> base_joint_index_; /**< index into the list of joints for the virtual joints corresponding to the base */

    std::vector<std::string> joint_name_; /**< names of joints controlled by this controller */

    std::vector<int> joint_type_; /**< joint types of the joints controlled by this controller, these are derived from the mechanism class */

    std::vector<double> current_joint_position_; /**< internal storage for the current joint positions */

    std::vector<double> current_joint_velocity_;  /**< internal storage for the current joint velocities */

    std::vector<double> joint_position_errors_;  /**< internal storage for the joint position errors (actual - command)*/

    std::vector<double> joint_velocity_errors_;  /**< internal storage for the joint velocity errors (actual - command)*/

    std::vector<double> max_allowable_joint_errors_;  /**< max allowable joint errors. These are loaded through the ROS Param server. e.g. <param name="r_shoulder_pan_joint/joint_error_threshold" type="double" value="0.1"/>*/

    controller::BaseControllerNode *base_controller_node_; /**< internal controller used to start up a controller for a robot base */

    std::vector<JointPDController *> joint_pv_controllers_; /**< internal set of controllers used to control individual joints */

    trajectory::Trajectory *joint_trajectory_; /**< internal representation of the trajectory specified for all the joints */

    std::vector<double> joint_cmd_; /**< internal representation of the instantaneous position commands for each joint */

    std::vector<double> joint_cmd_dot_; /**< internal representation of the instantaneous velocity commands for each joint */

    mechanism::Robot* robot_; /**< Pointer to a robot object */

    mechanism::RobotState* robot_state_; /**< Pointer to a robot state object */

    bool refresh_rt_vals_; /**< Indicates that a new trajectory has been received. */

    double trajectory_start_time_; /**< Start time for the current trajectory */

    double trajectory_end_time_;  /**< End time for the current trajectory */

    double current_time_; /**< Current time */

    bool current_trajectory_finished_; /**< Indicates that the current trajectory is done */

    bool at_rest_; /**< Indicates that all the desired joint positions are set to the current joint positions */

    int num_joints_; /**< Number of joints controlled by this controller */

    std::vector<double> joint_velocity_limits_; /**< Velocity limits for each joint. These are derived from the robot object created using a representation of the robot */

    std::string trajectory_type_; /**< The trajectory type used for control. It can be "linear", "cubic"*/

    double velocity_scaling_factor_; /**< Scaling factor for the velocity limits */

    double trajectory_wait_time_; /**< The amount of time that the controller waits after the expected completion time for a trajectory before declaring that a trajectory has failed */

    std::vector<double> goal_reached_threshold_; /**< Threshold within which the joints must be before goal is declared to have been reached */

    realtime_tools::RealtimePublisher <robot_msgs::ControllerState>* controller_state_publisher_ ;  /**< Publishes controller information */

    double max_update_time_; /**< maximum time (over the complete history of the run) taken for the update loop of this controller*/

    double last_traj_req_time_; /**< Last time a trajectory command was received on a topic or service */

    double max_allowed_update_time_; /**< Safety behavior: Max allowed time between commands */

    bool watch_dog_active_; 

    realtime_tools::RealtimePublisher <robot_msgs::DiagnosticMessage>* diagnostics_publisher_ ;  /**< Publishes controller information as a diagnostic message */

    /*!
     * \brief mutex lock for setting and getting ros messages
     */
    boost::mutex ros_lock_;

    robot_msgs::JointTraj traj_msg_; /**< The trajectory message received over ROS */

    /*!
     * \brief node name
     */
    std::string name_;

    /*
     * \brief Topic on which the controller listens for commands
     */
    std::string listen_topic_name_;

    /*
     * \brief pointer to ros node
     */
    ros::Node * const node_;

    std::vector<robot_msgs::JointTraj> joint_trajectory_vector_; /**< Vector of trajectory requests */

    std::vector<int> joint_trajectory_id_; /**< Vector of ids for trajectory requests */

    /**
     * @brief Get the parameters from the ROS parameter server 
     */
    void getParams(); 

    /**
     * @brief Initialize publishers (diagnostic, state) 
     */
    void initializePublishers();

    /**
     * @brief Stop publishers (diagnostic, state) 
     */
    void stopPublishers();

    /**
     * @brief Advertise all services 
     */
    void advertiseServices();

    /**
     * @brief Unadvertise all services 
     */
    void unadvertiseServices();

    /**
     * @brief Subscribe to topics
     */
    void subscribeTopics();

    /**
     * @brief Unsubscribe to topics
     */
    void unsubscribeTopics();

    /**
     * @brief Load parameters from an XML file 
     * @param robot pointer to a robot object passed in by all controllers.
     * @config TiXml configuration element
     */
    bool loadXmlFile(mechanism::RobotState * robot, TiXmlElement * config);

    /**
     * @brief Initialize the trajectory from a tiny XML element 
     */
    void initTrajectory(TiXmlElement * config);

    /**
     * @brief Set the trajectory command to the current values of each joint. This has the effect of stopping all joints
     * at their current positions
     */
    void setTrajectoryCmdToCurrentValues();

    /**
     * @brief Add a joint to the list of joints controlled by this controller.
     * @param name The name of the joint in the mechanism structure.
     */
    void addJoint(const std::string &name);

    /**
     * @brief Add a virtual joint for a robot base
     * @param elt A tinyXML element required to initialize the base controller
     */
    void addRobotBaseJoint(TiXmlElement *elt);

    /**
     * @brief Set the commanded trajectory for the controller
     * @param joint_trajectory A vector of trajectory points that this controller must follow 
     */
    bool setTrajectoryCmd(const std::vector<trajectory::Trajectory::TPoint>& joint_trajectory);

    /**
     * @brief Determine if the joint position errors are within the threshold specified by goal_reached_threshold. 
     * These thresholds are set using the ROS param server e.g. <param name="r_shoulder_pan_joint/goal_reached_threshold" type="double" value="0.1"/> 
     * @return true if absolute position errors are less than threshold values, false otherwise
     */
    bool errorsWithinThreshold();

    /**
     * @brief Determine if the joints are at rest. 
     * @return true if absolute velocities are less than zero
     */
    bool atRest();

    /**
     * @brief Compute the errors for each joint
     */
    void computeJointErrors();

    /**
     * @brief Check watchdog. If there has been no trajectory request externally for a while or if the position errors are too great, this will set the current position to the desired position, thus effectively stopping the robot.
     * The parameters that affect this function are 
     * (a) max_allowed_update_time_ which can be set using a ROS param call, e.g. <param name="max_allowed_update_time" type="double" value="0.1"/>
     * (b) max_allowable_joint_errors_ which can be set individually for each joint using a ROS param call, e.g. <param name="r_shoulder_pan_joint/joint_error_threshold" type="double" value="0.1"/>
     */
    bool checkWatchDog(double current_time);

    /**
     * @brief Stop the motion of all joints. In addition to setting all desired joint positions to the current position, it also sets velocities for the base to zero 
     */
    void stopMotion();

    /**
     * @brief Reset all the trajectory times on completion of an old trajectory or receipt of a new one that preempts the previous one 
     */
    void resetTrajectoryTimes();

    /**
     * @brief Internal function to get the actual set point values from the trajectory.
     */
    void getJointCommands();

    /**
     * @brief Check if the trajectory is done. If done, pop the next trajectory from the queue and use it. If no new trajectory is available, set the desired joint positions to the current joint positions
     */
    bool trajectoryDone();

    /** 
     * @brief Based on the current position of the base, update the velocity commands to be sent out to the base controller
     */
    void updateBaseCommand(double time);

    /**   
     * @brief Check if the goal position has been reached 
     */
    bool reachedGoalPosition(std::vector<double> joint_cmd);

    /**   
     * @brief Call update() on each joint controller. (This actually sends out the commands into hardware)
     */
    void updateJointControllers(void);

    /**   
     * @brief Update all joint values by reading from the robot structure
     */
    void updateJointValues();

    /**   
     * @brief Update the trajectory queue by removing trajectories that are done, set the current trajectory to be executed to the next trajectory on the queue.
     * If no new trajectory is available, set the desired joint positions to the current joint positions
     * @param id  The id of the last trajectory
     * @param finish_status The status of the last trajectory
     */
    void updateTrajectoryQueue(int id, int finish_status);

    /**   
     * @brief Get the next trajectory from the queue
     & @param index The index of the requested trajectory
     */
    bool getTrajectoryFromQueue(int &index);

    /**   
     * @brief Convert a trajectory command (from the queue) and set the current desired trajectory to it
     * @param traj_msg Trajectory message received on a topic or a service
     * @param id The associated id for the trajectory command
     */
    void setTrajectoryCmdFromMsg(robot_msgs::JointTraj traj_msg, int id);

    /**   
     * @brief Callback when a trajectory message is received on a topic
     */
    void TrajectoryReceivedOnTopic();

    /**   
     * @brief Service provided to set trajectories
     * @param req The request containing the trajectory to be queued up
     * @param resp The response contains the id assigned to the trajectory
     */
    bool setJointTrajSrv(pr2_mechanism_controllers::TrajectoryStart::Request &req,
                                                pr2_mechanism_controllers::TrajectoryStart::Response &resp);

    /**   
     * @brief Service provided to set trajectories
     * @param req The request contains the id of the trajectory about which you need information (Use the rosmsg tool to see the fields required for the request. e.g. rosmsg show TrajectoryQuery)
     * @param resp The response contains information about the trajectory in the following fields: 
     *             (a) done: 1 if trajectory is done, 0 if ongoing, 2 if queued, 3 if deleted, 4 if failed, 5 if canceled 
     *             (b) trajectorytime: If active, the current timestamp the trajectory is at, if done the total time taken for the trajectory, if queued 0
     *             (c) jointnames: the names of the joints controlled by this controller
     *             (d) jointpositions: the current joint positions 
     */
    bool queryJointTrajSrv(pr2_mechanism_controllers::TrajectoryQuery::Request &req,
                                                  pr2_mechanism_controllers::TrajectoryQuery::Response &resp);

    /**   
     * @brief Service provided to cancel trajectories
     * @param req The request contains the id of the trajectory which needs to be canceled (Use the rosmsg tool to see the fields required for the request. e.g. rosmsg show TrajectoryCancel)
     */
    bool cancelJointTrajSrv(pr2_mechanism_controllers::TrajectoryCancel::Request &req,
                                                   pr2_mechanism_controllers::TrajectoryCancel::Response &resp);


    /**   
     * @brief Create a vector of trajectory point(TPoint) objects from a trajectory message
     * @param new_traj The trajectory message that needs to be converted
     * @param tp The resultant vector of TPoints
     */
    bool createTrajectoryPointsVectorFromMsg(const robot_msgs::JointTraj &new_traj, std::vector<trajectory::Trajectory::TPoint> &tp);

    /**   
     * @brief Create a trajectory object from a trajectory message
     * @param new_traj The trajectory message that needs to be converted
     * @param return_trajectory The resultant trajectory object
     */
    bool createTrajectoryFromMsg(const robot_msgs::JointTraj &new_traj,trajectory::Trajectory &return_trajectory);

    /**   
     * @brief Add a new trajectory request to the queue of trajectories that need to be sent out
     * @param new_traj The trajectory message that needs to be queued
     * @param id The id of the trajectory to be queued
     */
    void addTrajectoryToQueue(robot_msgs::JointTraj new_traj, int id);

    /**   
     * @brief Preempt the current trajectory queue
     * @param new_traj The trajectory message that needs to executed
     * @param id The id of the trajectory
     */
    void preemptTrajectoryQueue(robot_msgs::JointTraj new_traj, int id);

    /**   
     * @brief Delete a trajectory from the queue of trajectories
     * @param new_traj The trajectory message that needs to be deleted
     */
    void deleteTrajectoryFromQueue(int id);

    /**   
     * @brief Publish diagnostic information
     */
    void publishDiagnostics();

    /**   
     * @brief Initialized to 1 (0 represents a trajectory that keeps the robot at its current position)
     */
    int request_trajectory_id_;

    /**   
     * @brief Initialized to 0 and then set to the requested trajectory id. Reverts to 0 when no trajectories are available on the queue
     */
    int current_trajectory_id_;

    /**   
     * @brief A map from trajectory ids to trajectory status
     */
//    std::map<int,int> joint_trajectory_status_;
    std::vector<int> joint_trajectory_status_;

    /**   
     * @brief A map from trajectory ids to trajectory times
     */
    std::vector<double> joint_trajectory_time_;
//    std::map<int,double>joint_trajectory_time_;

    /**   
     * @brief Enumeration of trajectory status
     */
    enum JointTrajectoryStatus{
      ACTIVE,/*!< The current trajectory being executed. */
      DONE,/*!< Trajectory execution was finished. */
      QUEUED, /*!< Trajectory is queued for execution. */
      DELETED, /*!< Trajectory has been deleted BEFORE execution. */
      FAILED, /*!< Trajectory execution has failed. This happens if the joints fail to get to the last desired position in the trajectory. */
      CANCELED, /*!< Trajectory has been canceled (preempted by other trajectory) while active */
      DOES_NOT_EXIST, /*!< This trajectory does not exist yet */
      NUM_STATUS
    };

    /**   
     * @brief The maximum amount of time that the controller waits after the expected completion time for a trajectory before declaring that a trajectory has failed 
     */
    double trajectory_wait_timeout_;

    double last_diagnostics_publish_time_; /**< Last time diagnostics infomation was published */

    double diagnostics_publish_delta_time_;  /**< Expected rate at which diagnostics information should be published. This can be published using a ROS param call  <param name="diagnostics_publish_delta_time" type="double" value="0.5"/> */

    double at_rest_velocity_threshold_; /**< Threshold for deciding if all the joints are at rest */

    std::vector<trajectory::Trajectory::TPoint> current_joint_position_vector_; /**< Pre-allocated to size 2 x num_joints in the constructor to have a realtime safe container for the current position of the joints */

    trajectory::Trajectory::TPoint trajectory_point_; /**< Pre-allocated to size num_joints in the constructor to have a realtime safe container for the current position of the joints */

    int base_theta_index_; /**< Index corresponding to the rotational degree of freedom of a robot base */

    int num_trajectory_available_;

    int next_free_index_;

    int max_trajectory_queue_size_;

    int current_trajectory_index_;

    bool trajectory_preempted_;

    boost::mutex trajectory_queue_;

  };
}
