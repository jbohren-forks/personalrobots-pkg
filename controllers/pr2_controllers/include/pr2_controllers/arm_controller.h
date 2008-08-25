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
#include <pr2_controllers/SetArmCommand.h>
#include <pr2_controllers/SetArmCartesianPos.h>
#include <pr2_controllers/GetArmCartesianPos.h>
#include <pr2_controllers/GetArmCommand.h>

//Kinematics
#include <robot_kinematics/robot_kinematics.h>

// #include <libTF/Pose3D.h>
// #include <urdf/URDF.h>

namespace controller
{

  typedef struct
  {
    double p_gain;
    double i_gain; 
    double d_gain; 
    double windup;
    std::string joint_name;
    std::string control_type;
  } JointControlParam;

  // The maximum number of joints expected in an arm.
  static const int MAX_ARM_JOINTS = 7;
  
  class ArmDummyController : public Controller
  {
    public:
  
  /*!
     * \brief Default Constructor of the JointController class.
     *
   */
      ArmDummyController();

  /*!
       * \brief Destructor of the JointController class.
   */
      virtual ~ArmDummyController();

  /*!
       * \brief Functional way to initialize limits and gains.
       *
   */
      bool initXml(mechanism::Robot *robot, TiXmlElement *config);
  
  /*!
       * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
       *
       * \param double pos Position command to issue
   */
      void setCommand(pr2_controllers::SetArmCommand::request &req);

  /*!
       * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
   */
          void getCommand(pr2_controllers::GetArmCommand::response &resp);
          
          void getCurrentConfiguration(std::vector<double> &);
  /*!
       * \brief Issues commands to the joint. Should be called at regular intervals
   */
      virtual void update(void); // Real time safe.

      ros::thread::mutex arm_controller_lock_;

    private:

      std::vector<JointPositionController *> joint_position_controllers_;
      // Goal of the joints
      std::vector<double> goals_;
      // Goal of the joints - used by the realtime code only
      std::vector<double> goals_rt_; 

      mechanism::Robot* robot_;

      void updateJointControllers(void);

  };

  class ArmControllerNode : public Controller
  {
    public:
  /*!
     * \brief Default Constructor
     *
   */
      ArmControllerNode();

  /*!
       * \brief Destructor
   */
      ~ArmControllerNode();

      void update();

      bool initXml(mechanism::Robot *robot, TiXmlElement *config);

      // Services
      bool setCommand(pr2_controllers::SetArmCommand::request &req,
                      pr2_controllers::SetArmCommand::response &resp);

      bool getCommand(pr2_controllers::GetArmCommand::request &req,
                      pr2_controllers::GetArmCommand::response &resp);
      /** \brief sets the command to a cartesian position
       *  \return always true
      **/
      bool setCommandCP(pr2_controllers::SetArmCartesianPos::request &req,
                      pr2_controllers::SetArmCartesianPos::response &resp);

      /** \brief gets the cartesian positon of the gripper
       *  \return always true
       **/
      bool getCP(pr2_controllers::GetArmCartesianPos::request &req,
                        pr2_controllers::GetArmCartesianPos::response &resp);
    
    private:
      ArmDummyController *c_;
      robot_kinematics::RobotKinematics pr2_kin_;
      robot_kinematics::SerialChain * arm_chain_;


  };

}


