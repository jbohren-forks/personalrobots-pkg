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
#include <robot_mechanism_controllers/joint_position_controller.h>
#include <robot_mechanism_controllers/joint_velocity_controller.h>
#include <robot_mechanism_controllers/joint_effort_controller.h>
#include <robot_mechanism_controllers/joint_pd_controller.h>

// Services
#include <pr2_mechanism_controllers/SetJointPosCmd.h>
#include <pr2_mechanism_controllers/GetJointPosCmd.h>

#include <pr2_mechanism_controllers/SetJointGains.h>
#include <pr2_mechanism_controllers/GetJointGains.h>

#include <pr2_mechanism_controllers/SetCartesianPosCmd.h>
#include <pr2_mechanism_controllers/GetCartesianPosCmd.h>

#include <pr2_mechanism_controllers/SetJointTarget.h>
#include <pr2_mechanism_controllers/JointPosCmd.h>

#include <pr2_mechanism_controllers/JointTraj.h>
#include <pr2_mechanism_controllers/JointTrajPoint.h>

#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>

//Kinematics
#include <robot_kinematics/robot_kinematics.h>
#include <trajectory/trajectory.h>

#include <robot_msgs/ControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_tools.h>
#include <std_msgs/String.h>

namespace controller
{

  #define GOAL_REACHED_THRESHOLD 0.01


    // comment this out if the controller is not supposed to publish its own max execution time
  #define PUBLISH_MAX_TIME

 
// The maximum number of joints expected in an arm.
  static const int MAX_ARM_JOINTS = 7;

  class ArmTrajectoryController : public Controller
  {
    public:

    /*!
     * \brief Default Constructor of the JointController class.
     *
     */
    ArmTrajectoryController();

    /*!
     * \brief Destructor of the JointController class.
     */
    virtual ~ArmTrajectoryController();

    /*!
     * \brief Functional way to initialize limits and gains.
     */
    bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

    /*!
     * \brief set the joint trajectory for the arm
     */
    void setTrajectoryCmd(const std::vector<trajectory::Trajectory::TPoint>& joint_trajectory);

    void getJointPosCmd(pr2_mechanism_controllers::JointPosCmd & cmd) const;

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update(void); // Real time safe.

    boost::mutex arm_controller_lock_;

    controller::JointPDController* getJointControllerByName(std::string name);

    private:

    std::vector<JointPDController *> joint_pd_controllers_;

    trajectory::Trajectory *joint_trajectory_;

    trajectory::Trajectory::TPoint trajectory_point_;

    std::vector<double> joint_cmd_rt_;

    std::vector<double> joint_cmd_dot_rt_;

    mechanism::Robot* robot_;

    void updateJointControllers(void);

    bool reachedGoalPosition(std::vector<double> joint_cmd);

    int getJointControllerPosByName(std::string name);

    // Indicates if goals_ and error_margins_ should be copied into goals_rt_ and error_margins_rt_
    bool refresh_rt_vals_;

    double trajectory_start_time_;

    double trajectory_end_time_;

    double current_time_;

    bool trajectory_done_;

    int dimension_;

    std::vector<double> joint_velocity_limits_;

    std::string trajectory_type_;

    double velocity_scaling_factor_;

    friend class ArmTrajectoryControllerNode;

    double trajectory_wait_time_;

    std::vector<double> goal_reached_threshold_;

    realtime_tools::RealtimePublisher <robot_msgs::ControllerState>* controller_state_publisher_ ;  //!< Publishes controller information

    double max_update_time_;
  };

/** @class ArmTrajectoryControllerNode
 *  @brief ROS interface for the arm controller.
 *  @author Sachin Chitta <sachinc@willowgarage.com>
 *
 *  This class provides a ROS interface for controlling the arm by setting position configurations. If offers several ways to control the arms:
 *  - through listening to ROS messages: this is specified in the XML configuration file by the following parameters:
 *      <listen_topic name="the name of my message" />
 *      (only one topic can be specified)
 *  - through a non blocking service call: this service call can specify a single configuration as a target (and maybe multiple configuration in the future)
 *  - through a blocking service call: this service can receive a list of position commands that will be followed one after the other
 *
 */
  class ArmTrajectoryControllerNode : public Controller
  {
    public:
    /*!
     * \brief Default Constructor
     *
     */
    ArmTrajectoryControllerNode();

    /*!
     * \brief Destructor
     */
    ~ArmTrajectoryControllerNode();

    void update();

    bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

    /** @brief service that returns the goal of the controller
     * @note if you know the goal has been reached and you do not want to subscribe to the /mechanism_state topic, you can use it as a hack to get the position of the arm
     * @param req
     * @param resp the response, contains a JointPosCmd message with the goal of the controller
     * @return
     */
    bool getJointPosCmd(pr2_mechanism_controllers::GetJointPosCmd::request &req,
                        pr2_mechanism_controllers::GetJointPosCmd::response &resp);

    bool setJointTrajSrv(pr2_mechanism_controllers::TrajectoryStart::request &req,
                         pr2_mechanism_controllers::TrajectoryStart::response &resp);

    bool queryJointTrajSrv(pr2_mechanism_controllers::TrajectoryQuery::request &req,
                           pr2_mechanism_controllers::TrajectoryQuery::response &resp);

    bool cancelJointTrajSrv(pr2_mechanism_controllers::TrajectoryQuery::request &req,
                            pr2_mechanism_controllers::TrajectoryQuery::response &resp);

    void deleteTrajectoryFromQueue(int id);

    void addTrajectoryToQueue(pr2_mechanism_controllers::JointTraj new_traj, int id);

    int createTrajectory(const pr2_mechanism_controllers::JointTraj &new_traj,trajectory::Trajectory &return_trajectory);

    void updateTrajectoryQueue(int last_trajectory_finish_status);

    void getJointTrajectoryThresholds();

    private:

    /*!
     * \brief mutex lock for setting and getting ros messages
     */
    boost::mutex ros_lock_;

    pr2_mechanism_controllers::JointTraj traj_msg_;

    pr2_mechanism_controllers::JointPosCmd msg_;   //The message used by the ROS callback
    ArmTrajectoryController *c_;

    /*!
     * \brief service prefix
     */
    std::string service_prefix_;

    /*
     * \brief save topic name for unsubscribe later
     */
    std::string topic_name_;

    /*!
     * \brief xml pointer to ros topic name
     */
    TiXmlElement * topic_name_ptr_;

    /*
     * \brief pointer to ros node
     */
    ros::Node * const node_;


    /*
     * \brief receive and set the trajectory in the controller
     */
    void CmdTrajectoryReceived();

    int trajectory_id_;

    std::vector<pr2_mechanism_controllers::JointTraj> joint_trajectory_vector_;

    std::vector<int> joint_trajectory_id_;

    void setTrajectoryCmdFromMsg(pr2_mechanism_controllers::JointTraj traj_msg);

    int request_trajectory_id_;

    int current_trajectory_id_;

    std::map<int,int> joint_trajectory_status_;

    std::map<int,double>joint_trajectory_time_;

    enum JointTrajectoryStatus{
      ACTIVE,
      DONE,
      QUEUED,
      DELETED,
      FAILED,
      CANCELED,
      NUM_STATUS
    };

    double trajectory_wait_timeout_;

  };

}
