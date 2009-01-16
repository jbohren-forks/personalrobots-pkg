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
#include <rosthread/mutex.h>

#include <mechanism_model/controller.h>
#include <robot_mechanism_controllers/joint_position_controller.h>
#include <robot_mechanism_controllers/joint_velocity_controller.h>
#include <robot_mechanism_controllers/joint_effort_controller.h>

// Services
#include <pr2_mechanism_controllers/SetJointPosCmd.h>
#include <pr2_mechanism_controllers/GetJointPosCmd.h>

#include <pr2_mechanism_controllers/SetJointGains.h>
#include <pr2_mechanism_controllers/GetJointGains.h>

#include <pr2_mechanism_controllers/SetCartesianPosCmd.h>
#include <pr2_mechanism_controllers/GetCartesianPosCmd.h>

#include <pr2_mechanism_controllers/SetJointTarget.h>
#include <pr2_mechanism_controllers/JointPosCmd.h>

//Kinematics
#include <robot_kinematics/robot_kinematics.h>

// #include <libTF/Pose3D.h>
// #include <urdf/URDF.h>

namespace controller
{

// The maximum number of joints expected in an arm.
static const int MAX_ARM_JOINTS = 7;

class ArmPositionController : public Controller
{
public:

  /*!
   * \brief Default Constructor of the JointController class.
   *
   */
  ArmPositionController();

  /*!
   * \brief Destructor of the JointController class.
   */
  virtual ~ArmPositionController();

  /*!
   * \brief Functional way to initialize limits and gains.
   *
  */
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param double pos Position command to issue
  */


  /*!
   * \brief Sets a goal for some joint, specified as a pair of (joint values, joint names)
   *
   * \param j_values the positions of the joints
   * \param j_names the names of the joints
  */
  void setJointPosCmd(const std::vector<double> &j_values, const std::vector<std::string> & j_names);
  void setJointPosCmd(const std::vector<double> &j_values);

//  /**
//   * @brief Overloaded method for convenience
//   * @param req
//   */
//  void setJointPosCmd(pr2_mechanism_controllers::SetJointPosCmd::request &req);

  void setJointPosCmd(const pr2_mechanism_controllers::JointPosCmd & cmd);

  void getJointPosCmd(pr2_mechanism_controllers::JointPosCmd & cmd) const;

  /*!
   * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
  */
//  void getJointPosCmd(pr2_mechanism_controllers::GetJointPosCmd::response &resp);

//  void getCurrentConfiguration(std::vector<double> &);

  /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
  */
  virtual void update(void); // Real time safe.

  ros::thread::mutex arm_controller_lock_;

//  void setJointGains(const pr2_mechanism_controllers::SetJointGains::request &req);

//  void getJointGains(pr2_mechanism_controllers::GetJointGains::response &resp);

  controller::JointPositionController* getJointControllerByName(std::string name);

  bool goalAchieved() const { return goal_achieved_; }

private:

  std::vector<JointPositionController *> joint_position_controllers_;

  // Goal of the joints
  std::vector<double> goals_;

  // Error margins of the goals. If set to <=0, they are not enforced
  std::vector<double> error_margins_;

  // Goal of the joints - used by the realtime code only
  std::vector<double> goals_rt_;

  // Error margins of the goals. If set to <=0, they are not enforced
  // Accessed by real-time code only
  std::vector<double> error_margins_rt_;

  mechanism::Robot* robot_;

  void updateJointControllers(void);

  int getJointControllerPosByName(std::string name);

  void checkForGoalAchieved_(void);

  bool goal_achieved_;

  // Indicates if goals_ and error_margins_ should be copied into goals_rt_ and error_margins_rt_
  bool refresh_rt_vals_;

};

/** @class ArmPositionControllerNode
 *  @brief ROS interface for the arm controller.
 *  @author Timothy Hunter <tjhunter@willowgarage.com>
 *
 *  This class provides a ROS interface for controlling the arm by setting position configurations. If offers several ways to control the arms:
 *  - through listening to ROS messages: this is specified in the XML configuration file by the following parameters:
 *      <listen_topic name="the name of my message" />
 *      (only one topic can be specified)
 *  - through a non blocking service call: this service call can specify a single configuration as a target (and maybe multiple configuration in the future)
 *  - through a blocking service call: this service can receive a list of position commands that will be followed one after the other
 *
 */
class ArmPositionControllerNode : public Controller
{
  public:
    /*!
    * \brief Default Constructor
    *
    */
    ArmPositionControllerNode();

    /*!
    * \brief Destructor
    */
    ~ArmPositionControllerNode();

    void update();

    bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

    /** @brief sets a target for some joints managed by the controller (blocking)
     *
     * Sets a target position for some joints managed by the controller and waits to the position to be reached.
     * Given an array of tuples (name, position, error margin), the controllers sets this as a new objective for the arm controller, for these elements only. This service returns when:
     * 1) all joints specified in the tuple have a position specified with the error margin, or
     * 2) the timeout value has been reached.
     * If you assign a new target for one joint only, this service will return as soon as this target is reached. The other joints will try to enforce the position command they had prior to this command, but this command may not be enforced when the service cal returns.
     * \note the blocking behavior is currently achieved by sleeping 1/100th of a second
     * @note if you want a non blocking version, use setJointPosSingleHeadless
     * @param cmd
     * @return
     */
    bool setJointPosSingle(const pr2_mechanism_controllers::JointPosCmd & cmd);

    /** @brief sets a target for some joints managed by the controller (non-blocking)
     * This is the non-blocking version of setJointPosSingle
     * @note the error margins and timeout values are discarded
     * @param cmd
     * @return
     */
    bool setJointPosSingleHeadless(pr2_mechanism_controllers::JointPosCmd & cmd);

    /** @brief sets a command for all the joints managed by the controller at once
     * This is a lightweight version of setJointPosHeadless when the caller knows the order of the joints.
     * @note this service should not be used. use setJointPosHeadless instead
     * This service returns immediately
     * @param req an array of position
     * @param resp The response is empty
     * @return
     */
    bool setJointPosSrv(pr2_mechanism_controllers::SetJointPosCmd::request &req,
                    pr2_mechanism_controllers::SetJointPosCmd::response &resp);

    /** @brief sets a command for all the joints managed by the controller at once, by specifying an array
     * This is a lightweight version of setJointPosHeadless when the caller knows the order of the joints.
     * @note this method should be used for debugging purpose only. You should first consider using the associated service setJointPosSrv
     * This function returns immediately
     * @param joint_pos an array of position
     */
    void setJointPosArray(const std::vector<double> & joint_pos);

    /** @brief blocking service to specify trajectory tracking
     * Given an array of JointPosCmd, the controller will try to visit each of these point one after the other
     * In case on of the points cannot be reached, the service will stop execution of the trajectory and return false
     * @note: this service should be used only in scripting applications
     * @param req an array of JointPosCmd elements
     * @param resp empty
     * @return true if the trajectory could be followed
     */
    bool setJointPosTarget(pr2_mechanism_controllers::SetJointTarget::request &req,
                     pr2_mechanism_controllers::SetJointTarget::response &resp);

    /** @brief non blocking service to specify a position target
     * Given an array of tuples (name, position, error margin), the controllers sets this as a new objective for the arm controller, for these elements only. This service retutrns immediately.
     * @note a request (('joint0',pos0),('joint1',pos1)) is equivalent to 2 simutaneous requests ('joint0',pos0) and ('joint1',pos1)
     * @note the service expects only one position to be given, and will return false if more the one position configuration are found in the SetJointTarget request
     * @note the error margins and timeout values are discarded when passing the message to the controller
     * @param req
     * @param resp
     * @return
     */
    bool setJointPosHeadless(pr2_mechanism_controllers::SetJointTarget::request &req,
                     pr2_mechanism_controllers::SetJointTarget::response &resp);

    /** @brief service that returns the goal of the controller
     * @note if you know the goal has been reached and you do not want to subscribe to the /mechanism_state topic, you can use it as a hack to get the position of the arm
     * @param req
     * @param resp the response, contains a JointPosCmd message with the goal of the controller
     * @return
     */
    bool getJointPosCmd(pr2_mechanism_controllers::GetJointPosCmd::request &req,
                        pr2_mechanism_controllers::GetJointPosCmd::response &resp);

    /** @brief ROS callback hook
     *  Provides a ROS callback to set a new goal. The topic the controller listens to is set in the xml init file.
     */
    void setJointPosSingleHeadless_cb();


  private:
    pr2_mechanism_controllers::JointPosCmd msg_;   //The message used by the ROS callback
    ArmPositionController *c_;

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
    TiXmlElement * ros_cb_;

    /*
     * \brief pointer to ros node
     */
    ros::Node * const node_;
};

}


