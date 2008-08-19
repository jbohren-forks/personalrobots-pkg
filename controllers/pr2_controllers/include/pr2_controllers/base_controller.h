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

#include <generic_controllers/controller.h>
#include <generic_controllers/joint_position_controller.h>
#include <generic_controllers/joint_velocity_controller.h>
#include <generic_controllers/joint_effort_controller.h>

// Services
#include <pr2_controllers/SetBaseCommand.h>
#include <pr2_controllers/GetBaseCommand.h>

#include <libTF/Pose3D.h>
#include <urdf/URDF.h>

#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <newmat10/newmatap.h>

#include <std_msgs/RobotBase2DOdom.h>

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
}JointControlParam;

typedef struct
{
    libTF::Pose3D::Vector pos;
    std::vector<libTF::Pose3D::Vector> wheel_pos;
}BaseCasterGeomParam;

class BaseController : public Controller
{
public:
  
  /*!
   * \brief Default Constructor of the JointController class.
   *
   */
  BaseController();

  /*!
   * \brief Destructor of the JointController class.
   */
  ~BaseController();

  /*!
   * \brief Functional way to initialize limits and gains.
   *
   */
  void init(std::vector<JointControlParam> jcp, mechanism::Robot *robot);
 void initXml(mechanism::Robot *robot, TiXmlElement *config);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param double pos Position command to issue
   */
  void setCommand(libTF::Pose3D::Vector cmd_vel);

  /*!
   * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
   */
  libTF::Pose3D::Vector getCommand();

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update();

  std::vector<BaseCasterGeomParam> base_casters_;

  std::vector<double> caster_steer_angles_;

  std::vector<double> wheel_speeds_;

  pthread_mutex_t base_controller_lock_;

  robot_desc::URDF urdf_model_;

  void setGeomParams(std:: string joint_name);

private:

  int num_wheels_;

  int num_casters_;

  std::vector<JointEffortController> joint_effort_controllers_;

  std::vector<JointPositionController> joint_position_controllers_;

  std::vector<JointVelocityController> joint_velocity_controllers_;

  mechanism::Robot* robot_;

  libTF::Pose3D::Vector computePointVelocity2D(const libTF::Pose3D::Vector& pos, const  libTF::Pose3D::Vector& vel);

  libTF::Pose3D::Vector rotate2D(const libTF::Pose3D::Vector& pos, double theta);

  void updateJointControllers();

  libTF::Pose3D::Vector cmd_vel_;  
  
  libTF::Pose3D::Vector cmd_vel_t_;  

  libTF::Pose3D::Vector base_odom_position_;  

  libTF::Pose3D::Vector base_odom_velocity_;  

  void setGeomParams();

  void computeAndSetCasterSteer();

  void computeAndSetWheelSpeeds();

  double wheel_radius_;

  double base_caster_x_offset_;

  double base_caster_y_offset_;

  double wheel_base_;

  std::vector<double> steer_angle_actual_;

  std::vector<double> wheel_speed_actual_;

  std::vector<libTF::Pose3D::Vector> base_wheels_position_;

  void computeBaseVelocity();

  void computeOdometry(double);

  NEWMAT::Matrix pseudoInverse(const NEWMAT::Matrix M);

  void computeWheelPositions();

  void getJointValues();

  std_msgs::RobotBase2DOdom odomMsg;

  double last_time_;

};

class BaseControllerNode : public Controller
{
public:
  /*!
   * \brief Default Constructor
   *
   */
  BaseControllerNode();

  /*!
   * \brief Destructor
   */
  ~BaseControllerNode();

  void update();

  void initXml(mechanism::Robot *robot, TiXmlElement *config);

  // Services
  bool setCommand(pr2_controllers::SetBaseCommand::request &req,
                  pr2_controllers::SetBaseCommand::response &resp);

  bool getCommand(pr2_controllers::GetBaseCommand::request &req,
                  pr2_controllers::GetBaseCommand::response &resp);

private:
  BaseController *c_;


};

}


