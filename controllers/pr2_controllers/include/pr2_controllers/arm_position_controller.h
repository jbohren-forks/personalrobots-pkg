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

#include <generic_controllers/controller.h>
#include <generic_controllers/joint_position_controller.h>
#include <generic_controllers/joint_velocity_controller.h>
#include <generic_controllers/joint_effort_controller.h>

// Services
#include <pr2_controllers/SetJointPosCmd.h>
#include <pr2_controllers/GetJointPosCmd.h>

#include <pr2_controllers/SetJointGains.h>
#include <pr2_controllers/GetJointGains.h>

#include <pr2_controllers/SetCartesianPosCmd.h>
#include <pr2_controllers/GetCartesianPosCmd.h>

#include <pr2_controllers/SetJointTarget.h>
#include <pr2_controllers/JointPosCmd.h>

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
//  void setJointPosCmd(pr2_controllers::SetJointPosCmd::request &req);

  void setJointPosCmd(const pr2_controllers::JointPosCmd & cmd);

  /*!
   * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
  */
//  void getJointPosCmd(pr2_controllers::GetJointPosCmd::response &resp);

//  void getCurrentConfiguration(std::vector<double> &);

  /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
  */
  virtual void update(void); // Real time safe.

  ros::thread::mutex arm_controller_lock_;

//  void setJointGains(const pr2_controllers::SetJointGains::request &req);

//  void getJointGains(pr2_controllers::GetJointGains::response &resp);

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

//      void setJointPosCmd(std::vector<double> &req_goals_);

      bool setJointPosSingle(const pr2_controllers::JointPosCmd & cmd);

      // Services
      bool setJointPosSrv(pr2_controllers::SetJointPosCmd::request &req,
                      pr2_controllers::SetJointPosCmd::response &resp);

//      bool getJointPosCmd(pr2_controllers::GetJointPosCmd::request &req,
//                      pr2_controllers::GetJointPosCmd::response &resp);

      bool setJointPosTarget(pr2_controllers::SetJointTarget::request &req,
                       pr2_controllers::SetJointTarget::response &resp);

      bool setJointPosHeadless(pr2_controllers::SetJointTarget::request &req,
                       pr2_controllers::SetJointTarget::response &resp);

      //       bool setJointGains(pr2_controllers::SetJointGains::request &req,
//                                                     pr2_controllers::SetJointGains::response &resp);

//       bool getJointGains(pr2_controllers::GetJointGains::request &req,
//                                                     pr2_controllers::GetJointGains::response &resp);

    private:
      ArmPositionController *c_;

  };

}


