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
#include <robot_mechanism_controllers/joint_velocity_controller.h>

// Services
#include <pr2_mechanism_controllers/SetJointVelCmd.h>
#include <pr2_mechanism_controllers/GetJointVelCmd.h>

#include <pr2_mechanism_controllers/SetJointGains.h>
#include <pr2_mechanism_controllers/GetJointGains.h>

#include <pr2_mechanism_controllers/SetCartesianVelCmd.h>
#include <pr2_mechanism_controllers/GetCartesianVelCmd.h>

//Kinematics
#include <robot_kinematics/robot_kinematics.h>

namespace controller
{

  // The maximum number of joints expected in an arm.
  static const int MAX_ARM_JOINTS = 7;

  class ArmVelocityController : public Controller
  {
    public:

  /*!
     * \brief Default Constructor of the JointController class.
     *
   */
      ArmVelocityController();

  /*!
       * \brief Destructor of the JointController class.
   */
      virtual ~ArmVelocityController();

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
      void setJointVelCmd(pr2_mechanism_controllers::SetJointVelCmd::request &req);

  /*!
       * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
   */
          void getJointVelCmd(pr2_mechanism_controllers::GetJointVelCmd::response &resp);

          void getCurrentConfiguration(std::vector<double> &);
  /*!
       * \brief Issues commands to the joint. Should be called at regular intervals
   */
      virtual void update(void); // Real time safe.

      boost::mutex arm_controller_lock_;

      void setJointGains(const pr2_mechanism_controllers::SetJointGains::request &req);

      void getJointGains(pr2_mechanism_controllers::GetJointGains::response &resp);

      controller::JointVelocityController* getJointControllerByName(std::string name);

    private:

      std::vector<JointVelocityController *> joint_velocity_controllers_;

      // Goal of the joints
      std::vector<double> goals_;
      // Goal of the joints - used by the realtime code only
      std::vector<double> goals_rt_;

      mechanism::RobotState* robot_;

      void updateJointControllers(void);
  };

  class ArmVelocityControllerNode : public Controller
  {
    public:
  /*!
     * \brief Default Constructor
     *
   */
      ArmVelocityControllerNode();

  /*!
       * \brief Destructor
   */
      ~ArmVelocityControllerNode();

      void update();

      bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

      // Services
      bool setJointVelCmd(pr2_mechanism_controllers::SetJointVelCmd::request &req,
                      pr2_mechanism_controllers::SetJointVelCmd::response &resp);

      bool getJointVelCmd(pr2_mechanism_controllers::GetJointVelCmd::request &req,
                      pr2_mechanism_controllers::GetJointVelCmd::response &resp);

      bool setCartesianVelCmd(pr2_mechanism_controllers::SetCartesianVelCmd::request &req,pr2_mechanism_controllers::SetCartesianVelCmd::response &resp);

      bool getCartesianVelCmd(pr2_mechanism_controllers::GetCartesianVelCmd::request &req,pr2_mechanism_controllers::GetCartesianVelCmd::response &resp);

      bool setJointGains(pr2_mechanism_controllers::SetJointGains::request &req,
                                                    pr2_mechanism_controllers::SetJointGains::response &resp);

      bool getJointGains(pr2_mechanism_controllers::GetJointGains::request &req,
                                                    pr2_mechanism_controllers::GetJointGains::response &resp);

    private:
      ArmVelocityController *c_;
      robot_kinematics::RobotKinematics pr2_kin_;
      robot_kinematics::SerialChain * arm_chain_;

  };

}


