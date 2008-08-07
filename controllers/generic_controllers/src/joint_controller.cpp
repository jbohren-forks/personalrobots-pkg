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
#include <generic_controllers/joint_controller.h>
#define DEFAULTMAXACCEL 5
//#define DEBUG 1
//Todo:
//1. Get and set params via server
//2. Integrate Joint and robot objects
//3. Integrate Motor controller time

using namespace controller;

ROS_REGISTER_CONTROLLER(JointController)

//-------------------------------------------------------------------------//
//CONSTRUCTION/DESTRUCTION CALLS
//-------------------------------------------------------------------------//


JointController::JointController()
{
  //Instantiate PID class
  pid_controller_.initPid(0, 0, 0, 0, 0); //Constructor for pid controller

  //Set commands to zero
  cmd_torque_ = 0;
  cmd_pos_ = 0;
  cmd_vel_ = 0;

  control_mode_ = CONTROLLER_DISABLED;
}

JointController::~JointController()
{
}

void JointController::init()
{
}

void JointController::initXml(mechanism::Robot *robot, TiXmlElement *config)
{
}

void JointController::init(double p_gain, double i_gain, double d_gain, double windup_max, double windup_min,
                           ControllerControlMode mode, double time, double max_effort, double min_effort,
                           mechanism::Joint *joint)
{
  pid_controller_.initPid(p_gain, i_gain, d_gain, windup_max, windup_min); //Constructor for pid controller

  cmd_torque_ = 0;
  cmd_pos_ = 0;
  cmd_vel_ = 0;

  //Init time
  last_time_ = time;

  //Temporary: will transition to use param server
  p_gain_ = p_gain;
  i_gain_ = i_gain;
  d_gain_ = d_gain;
  windup_max_ = windup_max;
  windup_min_ = windup_min;

  min_effort_ = min_effort;
  max_effort_ = max_effort;
  joint_ = joint;

  control_mode_ = mode;
  enableController();
}

void JointController::init(PidControlParam pcp, ControllerControlMode mode, double time, double max_effort,
                           double min_effort, mechanism::Joint *joint)
{
  pid_controller_.initPid(pcp.p_gain_, pcp.i_gain_, pcp.d_gain_, pcp.windup_max_, pcp.windup_min_); //Constructor for pid controller

  cmd_torque_ = 0;
  cmd_pos_ = 0;
  cmd_vel_ = 0;

  //Init time
  last_time_ = time;

  //Temporary: will transition to use param server
  p_gain_ = pcp.p_gain_;
  i_gain_ = pcp.i_gain_;
  d_gain_ = pcp.d_gain_;
  windup_max_ = pcp.windup_max_;
  windup_min_ = pcp.windup_min_;

  min_effort_ = min_effort;
  max_effort_ = max_effort;
  joint_ = joint;

  control_mode_ = mode;
  enableController();
}

void JointController::init(double time, mechanism::Joint *joint)
{
  pid_controller_.initPid(p_gain_, i_gain_, d_gain_, windup_max_, windup_min_); //Constructor for pid controller

  cmd_torque_ = 0;
  cmd_pos_ = 0;
  cmd_vel_ = 0;

  last_time_ = time;
  joint_ = joint;
  enableController();
}

//-------------------------------------------------------------------------//
//MODE/ENABLE CALLS
//-------------------------------------------------------------------------//

//Set the controller control mode
ControllerControlMode JointController::setMode(ControllerControlMode mode)
{
  control_mode_ = mode;
  return CONTROLLER_MODE_SET;
}

//Getter for control mode
ControllerControlMode JointController::getMode()
{
  return control_mode_;
}

//Allow controller to function
ControllerControlMode JointController::enableController()
{
  enabled_ = true;
  return CONTROLLER_ENABLED;
}

//Disable functioning. Set joint torque to zero.
ControllerControlMode JointController::disableController()
{
  enabled_ = false;
  joint_->commanded_effort_ = 0; //Immediately set commanded Effort to 0
  control_mode_ = CONTROLLER_DISABLED;
  return CONTROLLER_DISABLED;
}

bool JointController::checkForSaturation(void)
{
  return saturation_flag_;
}

//-------------------------------------------------------------------------//
//TORQUE CALLS
//-------------------------------------------------------------------------//
ControllerErrorCode JointController::setTorqueCmd(double torque)
{
  // double max_effort = joint->effortLimit;

  if (control_mode_ != CONTROLLER_TORQUE) //Make sure we're in torque command mode
  {
    return CONTROLLER_MODE_ERROR;
  }

  cmd_torque_ = torque;

  if (cmd_torque_ >= max_effort_)
  { //Truncate to positive limit
    cmd_torque_ = max_effort_;
    return CONTROLLER_TORQUE_LIMIT;
  }
  else if (cmd_torque_ <= min_effort_)
  { //Truncate to negative limit
    cmd_torque_ = min_effort_;
    return CONTROLLER_TORQUE_LIMIT;
  }

  return CONTROLLER_CMD_SET;
}

ControllerErrorCode JointController::getTorqueCmd(double *torque)
{
  *torque = cmd_torque_;
  return CONTROLLER_ALL_OK;
}

ControllerErrorCode JointController::getTorqueAct(double *torque)
{
  *torque = joint_->applied_effort_; //Read torque from joint
  return CONTROLLER_ALL_OK;
}

//-------------------------------------------------------------------------//
//POSITION CALLS
//-------------------------------------------------------------------------//


//Query mode, then set desired position
ControllerErrorCode JointController::setPosCmd(double pos)
{
  if (control_mode_ != CONTROLLER_POSITION) //Make sure we're in position command mode
  {
    return CONTROLLER_MODE_ERROR;
  }

  cmd_pos_ = pos;
  if (cmd_pos_ >= joint_->joint_limit_max_ && joint_->type_ != mechanism::JOINT_CONTINUOUS)
  { //Truncate to positive limit
    cmd_pos_ = joint_->joint_limit_max_;
    return CONTROLLER_JOINT_LIMIT;
  }
  else if (cmd_pos_ <= joint_->joint_limit_min_ && joint_->type_ != mechanism::JOINT_CONTINUOUS)
  { //Truncate to negative limit
    cmd_pos_ = joint_->joint_limit_min_;
    return CONTROLLER_JOINT_LIMIT;
  }
  return CONTROLLER_CMD_SET;
}

//Return the current position command
controller::ControllerErrorCode JointController::getPosCmd(double *pos)
{
  *pos = cmd_pos_;
  return controller::CONTROLLER_ALL_OK;
}

//Query the joint for the actual position
controller::ControllerErrorCode JointController::getPosAct(double *pos)
{
  *pos = joint_->position_;
  return controller::CONTROLLER_ALL_OK;
}

//-------------------------------------------------------------------------//
//VELOCITY CALLS
//-------------------------------------------------------------------------//
//Check mode, then set the commanded velocity
ControllerErrorCode JointController::setVelCmd(double vel)
{
  if (control_mode_ == CONTROLLER_VELOCITY || control_mode_ == ETHERDRIVE_SPEED)
  { //Make sure we're in velocity command mode
    cmd_vel_ = vel;
    return CONTROLLER_CMD_SET;
  }
  else
    return CONTROLLER_MODE_ERROR;
}

//Return the internally stored commanded velocity
controller::ControllerErrorCode JointController::getVelCmd(double *vel)
{
  *vel = cmd_vel_;
  return controller::CONTROLLER_ALL_OK;
}

//Query our joint for velocity
controller::ControllerErrorCode JointController::getVelAct(double *vel)
{
  *vel = joint_->velocity_;
  return controller::CONTROLLER_ALL_OK;
}

double JointController::getMaxVelocity()
{
  double dis_to_min, dis_to_max, closest_limit;
  dis_to_min = fabs(math_utils::shortest_angular_distance(joint_->position_, joint_->joint_limit_min_));
  dis_to_max = fabs(math_utils::shortest_angular_distance(joint_->position_, joint_->joint_limit_max_));
  closest_limit = dis_to_min < dis_to_max ? dis_to_min : dis_to_max;
  return sqrt(fabs(closest_limit * max_accel_));
}

//-------------------------------------------------------------------------//
//UPDATE CALLS
//-------------------------------------------------------------------------//


void JointController::update(void)
{
  double error(0), time(0), current_torque_cmd(0);
  double max_velocity = cmd_vel_;
  double currentVoltageCmd, v_backemf, v_clamp_min, v_clamp_max, k;

  if (control_mode_ == controller::CONTROLLER_DISABLED)
  {
    printf("JointController.cpp: Error:: controller disabled\n");
    return; //If we're not initialized, don't try to interact
  }
  time = robot_->hw_->current_time_;
  switch (control_mode_)
  {
    case CONTROLLER_TORQUE: //Pass through torque command
      current_torque_cmd = cmd_torque_;
      break;

    case CONTROLLER_POSITION: //Close the loop around position
      if (joint_->type_ == mechanism::JOINT_ROTARY || joint_->type_ == mechanism::JOINT_CONTINUOUS)
        error = math_utils::shortest_angular_distance(cmd_pos_, joint_->position_);
      else
        error = joint_->position_ - cmd_pos_;
      current_torque_cmd = pid_controller_.updatePid(error, time - last_time_);
      break;

    case CONTROLLER_VELOCITY: //Close the loop around velocity
      if (cap_accel_)
      {
        max_velocity = getMaxVelocity(); //Check max velocity coming into wall
        if (fabs(cmd_vel_) > max_velocity)
        {
          cmd_vel_ = -max_velocity; //Truncate velocity smoothly
        }
      }
      error = joint_->velocity_ - cmd_vel_;

      current_torque_cmd = pid_controller_.updatePid(error, time - last_time_);
      break;
    case ETHERDRIVE_SPEED: // Use hack to contol speed in voltage control mode for the etherdrive
#ifdef DEBUG
      printf("JC:: %f\n",cmd_vel_);
#endif
      currentVoltageCmd = cmd_vel_ * 20 * 60 / (136 * 2 * M_PI);
      v_backemf = joint_->velocity_ * 20 * 60 / (136 * 2 * M_PI);

      v_clamp_min = v_backemf - 3;// 0.655*16.7;
      v_clamp_max = v_backemf + 3;//0.655*16.7;

      k = 1.0 / 36.0;
#ifdef DEBUG
      printf("JC::%f\t%f\t%f\n", v_clamp_min, currentVoltageCmd, v_clamp_max);
#endif
      if (currentVoltageCmd > v_clamp_max)
        currentVoltageCmd = v_clamp_max;

      if (currentVoltageCmd < v_clamp_min)
        currentVoltageCmd = v_clamp_min;
      current_torque_cmd = currentVoltageCmd * k; //Convert to match PWM conversion inside boards
      break;

    default:
      printf("JointController.cpp: Error:: invalid control_mode_\n");
      current_torque_cmd = 0;
  }
  last_time_ = time;

  setJointEffort(current_torque_cmd);
}

ControllerErrorCode JointController::setParam(std::string label, double value)
{
  return CONTROLLER_ALL_OK;
}

ControllerErrorCode JointController::getParam(std::string label, double* value)
{
  return CONTROLLER_ALL_OK;
}

//Truncates (if needed), then sets the effort
void JointController::setJointEffort(double effort)
{
  double new_effort;

  new_effort = effort;
  saturation_flag_ = false;

  if (effort >= max_effort_)
  {
    new_effort = max_effort_;
    saturation_flag_ = true;
  }
  else if (effort <= min_effort_)
  {
    new_effort = min_effort_;
    saturation_flag_ = true;
  }
  joint_->commanded_effort_ = new_effort;
}

ControllerErrorCode JointController::loadXML(std::string filename)
{
  robot_desc::URDF model;
  int exists = 0;

  if (!model.loadFile(filename.c_str()))
    return CONTROLLER_MODE_ERROR;

  const robot_desc::URDF::Data &data = model.getData();

  std::vector<std::string> types;
  std::vector<std::string> names;
  std::vector<std::string>::iterator iter;

  data.getDataTagTypes(types);

  for (iter = types.begin(); iter != types.end(); iter++)
  {
    if (*iter == "controller")
    {
      exists = 1;
      break;
    }
  }

  if (!exists)
    return CONTROLLER_MODE_ERROR;

  exists = 0;
  data.getDataTagNames("controller", names);

  for (iter = names.begin(); iter != names.end(); iter++)
  {
    if (*iter == name_)
    {
      exists = 1;
      break;
    }
  }

  if (!exists)
    return CONTROLLER_MODE_ERROR;

  param_map_ = data.getDataTagValues("controller", name_);

  loadParam("p_gain", p_gain_);
  loadParam("d_gain", d_gain_);
  loadParam("i_gain", i_gain_);
  loadParam("windup_max", windup_max_);
  loadParam("windup_min", windup_min_);
  loadParam("max_effort", max_effort_);
  loadParam("min_effort", min_effort_);

  return CONTROLLER_ALL_OK;
}

void JointController::loadParam(std::string label, double &param)
{
  if (param_map_.find(label) != param_map_.end()) // if parameter value has been initialized in the xml file, set internal parameter value
    param = atof(param_map_[label].c_str());
}

void JointController::loadParam(std::string label, int &param)
{
  if (param_map_.find(label) != param_map_.end())
    param = atoi(param_map_[label].c_str());
}

std::string JointController::getName()
{
  return name_;
}

