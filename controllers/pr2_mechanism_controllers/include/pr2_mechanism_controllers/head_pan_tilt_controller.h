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
#include <mechanism_model/joint.h>
#include <robot_mechanism_controllers/joint_position_controller.h>

// Services
#include <pr2_mechanism_controllers/SetJointCmd.h>
#include <pr2_mechanism_controllers/GetJointCmd.h>
#include <pr2_mechanism_controllers/TrackPoint.h>

#include <pr2_mechanism_controllers/SetJointGains.h>
#include <pr2_mechanism_controllers/GetJointGains.h>


// Math utils
#include <math_utils/angles.h>

#include <rosTF/rosTF.h>
// #include <urdf/URDF.h>

namespace controller
{

// The maximum number of joints expected in an arm.
static const int MAX_HEAD_JOINTS = 2;

class HeadPanTiltController : public Controller
{
public:

  /*!
   * \brief Default Constructor of the JointController class.
   *
   */
  HeadPanTiltController();

  /*!
   * \brief Destructor of the JointController class.
   */
  virtual ~HeadPanTiltController();

  /*!
   * \brief Functional way to initialize limits and gains.
   *
  */
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);


  /*!
   * \brief Sets a goal for some joint, specified as a pair of (joint values, joint names)
   *
   * \param j_values the positions of the joints
   * \param j_names the names of the joints
  */
  void setJointCmd(const std::vector<double> &j_values, const std::vector<std::string> & j_names);

//  /**
//   * @brief Overloaded method for convenience
//   * @param req
//   */
//  void setJointCmd(pr2_mechanism_controllers::SetJointPosCmd::request &req);

  void getJointCmd(pr2_mechanism_controllers::JointCmd & cmd) const;

  /*!
   * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
  */
//  void getJointPosCmd(pr2_mechanism_controllers::GetJointPosCmd::response &resp);

//  void getCurrentConfiguration(std::vector<double> &);

  /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
  */
  virtual void update(void); // Real time safe.

  controller::JointPositionController* getJointPositionControllerByName(std::string name);

  unsigned int num_joints_;
  std::vector<JointPositionController *> joint_position_controllers_;  

private:

  double last_time_;

  

  // Goal of the joints
  std::vector<double> set_pts_;

  mechanism::Robot* robot_;

  void updateJointControllers(void);

  int getJointControllerByName(std::string name);
  
  

};

/** @class HeadPanTiltControllerNode
 *  @brief ROS interface for the head controller.
 *
 *  This class provides a ROS interface for controlling the arm by setting position configurations. If offers several ways to control the head:
 *  - through listening to ROS messages: this is specified in the XML configuration file by the following parameters:
 *      <listen_topic name="the name of my message" />
 *      (only one topic can be specified)
 *  - through a non blocking service call: this service call can specify a single configuration as a target (and maybe multiple configuration in the future)
 *  - through a blocking service call: this service can receive a list of position commands that will be followed one after the other
 *
 */
class HeadPanTiltControllerNode : public Controller
{
  public:
    /*!
    * \brief Default Constructor
    *
    */
    HeadPanTiltControllerNode();

    /*!
    * \brief Destructor
    */
    ~HeadPanTiltControllerNode();

    void update();

    bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

    /** @brief sets a command for all the joints managed by the controller at once
     * This is a lightweight version of setJointPosHeadless when the caller knows the order of the joints.
     * @note this service should not be used. use setJointPosHeadless instead
     * This service returns immediately
     * @param req an array of position
     * @param resp The response is empty
     * @return
     */
    bool setJointSrv(pr2_mechanism_controllers::SetJointCmd::request &req,
                    pr2_mechanism_controllers::SetJointCmd::response &resp);

    /** @brief service that returns the goal of the controller
     * @note if you know the goal has been reached and you do not want to subscribe to the /mechanism_state topic, you can use it as a hack to get the position of the arm
     * @param req
     * @param resp the response, contains a JointPosCmd message with the goal of the controller
     * @return
     */
    bool getJointCmd(pr2_mechanism_controllers::GetJointCmd::request &req,
                        pr2_mechanism_controllers::GetJointCmd::response &resp);
    
    bool trackPoint(pr2_mechanism_controllers::TrackPoint::request &req,                                    
                      pr2_mechanism_controllers::TrackPoint::response &resp);
  private:
    pr2_mechanism_controllers::JointCmd msg_;   //The message used by the ROS callback
    HeadPanTiltController *c_;
    rosTFClient TF;

};

}


