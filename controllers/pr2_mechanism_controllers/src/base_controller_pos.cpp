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

#include <pr2_mechanism_controllers/base_controller_pos.h>
#include <control_toolbox/filters.h>
#include <angles/angles.h>
#include "ros/node.h"

#define NUM_TRANSFORMS 2
#define EPS 1e-5
#define CMD_VEL_TRANS_EPS 1e-5
#define CMD_VEL_ROT_EPS 1e-5

using namespace ros;
using namespace std;
using namespace controller;
using namespace control_toolbox;
using namespace libTF;
using namespace NEWMAT;

ROS_REGISTER_CONTROLLER(BaseControllerPos)

BaseControllerPos::BaseControllerPos() : num_wheels_(0), num_casters_(0)
{
   MAX_DT_ = 0.01;

   max_accel_.x = 0.1;
   max_accel_.y = 0.1;
   max_accel_.z = 0.1;

   cmd_vel_.x = 0;
   cmd_vel_.y = 0;
   cmd_vel_.z = 0;

   desired_vel_.x = 0;
   desired_vel_.y = 0;
   desired_vel_.z = 0;

   cmd_vel_t_.x = 0;
   cmd_vel_t_.y = 0;
   cmd_vel_t_.z = 0;

   cmd_vel_direction_.x = 1.0;
   cmd_vel_direction_.y = 0.0;
   cmd_vel_direction_.z = 0.0;
   cmd_vel_magnitude_ = 0.0;

   cmd_vel_trajectory_ = new trajectory::Trajectory(4);

   cmd_vel_points_.resize(2);
   for(int i=0; i< 2; i++)
   {
      cmd_vel_points_[i].setDimension(4);
      cmd_vel_points_[i].q_[0] = 1.0;
      cmd_vel_points_[i].time_ = 0.0;
   }
   cmd_vel_trajectory_->setTrajectory(cmd_vel_points_);
   current_vel_point_.setDimension(4);

   kp_speed_ = KP_SPEED_DEFAULT;

   base_odom_position_.x = 0;
   base_odom_position_.y = 0;
   base_odom_position_.z = 0;

   base_odom_velocity_.x = 0;
   base_odom_velocity_.y = 0;
   base_odom_velocity_.z = 0;

   ils_weight_type_ = "Gaussian";
   ils_max_iterations_ = 3;
   caster_steer_vel_gain_ = 0;
   timeout_ = 0.1;

   odometer_distance_ = 0;
   odometer_angle_ = 0;

   new_cmd_available_ = false;

   pthread_mutex_init(&base_controller_lock_,NULL);
}

BaseControllerPos::~BaseControllerPos()
{
  delete cmd_vel_trajectory_;
}

// Set the base velocity command
void BaseControllerPos::setCommand(libTF::Vector cmd_vel)
{
  pthread_mutex_lock(&base_controller_lock_);
  cmd_vel_t_.x = filters::clamp(cmd_vel.x,-max_vel_.x, max_vel_.x);
  cmd_vel_t_.y = filters::clamp(cmd_vel.y,-max_vel_.y, max_vel_.y);
  cmd_vel_t_.z = filters::clamp(cmd_vel.z,-max_vel_.z, max_vel_.z);
  cmd_received_timestamp_ = robot_state_->hw_->current_time_;
#if 0
  ROS_INFO("BaseControllerPos:: command received: %f %f %f",cmd_vel.x,cmd_vel.y,cmd_vel.z);
  ROS_INFO("BaseControllerPos:: command current: %f %f %f", cmd_vel_.x,cmd_vel_.y,cmd_vel_.z);

  ROS_INFO("Base Odometry: Velocity ", base_odom_velocity_.x,base_odom_velocity_.y,base_odom_velocity_.z);
  ROS_INFO("Base Odometry: Position ", base_odom_position_.x,base_odom_position_.y,base_odom_position_.z);

  for(int i=0; i < (int) num_wheels_; i++)
  {
    ROS_INFO("BaseControllerPos:: wheel speed cmd:: %d %f",i,(base_wheels_[i].direction_multiplier_*wheel_speed_cmd_[i]));
  }
#endif
  new_cmd_available_ = true;
  pthread_mutex_unlock(&base_controller_lock_);
}

libTF::Vector BaseControllerPos::getCommand()// Return the current position command
{
  libTF::Vector cmd_vel;
  pthread_mutex_lock(&base_controller_lock_);
  cmd_vel.x = cmd_vel_.x;
  cmd_vel.y = cmd_vel_.y;
  cmd_vel.z = cmd_vel_.z;
  pthread_mutex_unlock(&base_controller_lock_);
  return cmd_vel;
}

void BaseControllerPos::init(std::vector<JointControlParam> jcp, mechanism::RobotState *robot_state)
{
   std::vector<JointControlParam>::iterator jcp_iter;
   robot_desc::URDF::Link *link;
   std::string joint_name;

   std::string xml_content;
   (ros::g_node)->get_param("robotdesc/pr2",xml_content);

   robot_state_ = robot_state;

   // wait for robotdesc/pr2 on param server
   while(!urdf_model_.loadString(xml_content.c_str()))
   {
      ROS_INFO("WARNING: base controller is waiting for robotdesc/pr2 in param server.  run roslaunch send.xml or similar.");
      (ros::g_node)->get_param("robotdesc/pr2",xml_content);
      usleep(100000);
   }

//   base_casters_.resize(4);
//   base_wheels_.resize(8);

   for(jcp_iter = jcp.begin(); jcp_iter != jcp.end(); jcp_iter++)
   {
      joint_name = jcp_iter->joint_name;
      link = urdf_model_.getJointLink(joint_name);

      if(joint_name.find("rotation") != string::npos)
      {
         base_casters_[num_casters_].pos_.x = link->xyz[0];

         base_casters_[num_casters_].pos_.y = link->xyz[1];
         base_casters_[num_casters_].pos_.z = link->xyz[2];
         base_casters_[num_casters_].parent_ = NULL;
         base_casters_[num_casters_].joint_state_ = robot_state_->getJointState(joint_name);
         base_casters_[num_casters_].local_id_ = num_casters_;
         steer_angle_actual_.push_back(0);
         steer_angle_stored_.push_back(0);
         steer_position_desired_.push_back(0);
         steer_velocity_desired_.push_back(0);
         steer_torque_desired_.push_back(0);

         base_casters_[num_casters_].position_controller_.init(robot_state_, joint_name, Pid(jcp_iter->p_gain,jcp_iter->i_gain,jcp_iter->d_gain,jcp_iter->windup));
         base_casters_[num_casters_].velocity_controller_.init(robot_state_, joint_name, Pid(jcp_iter->p_gain,jcp_iter->i_gain,jcp_iter->d_gain,jcp_iter->windup));

         base_casters_[num_casters_].name_ = std::string(" ");
         base_casters_[num_casters_].name_ = std::string(joint_name);

         num_casters_++;
      }
      if(joint_name.find("wheel") != string::npos)
      {
         base_wheels_[num_wheels_].pos_.x = link->xyz[0];
         base_wheels_[num_wheels_].pos_.y = link->xyz[1];
         base_wheels_[num_wheels_].pos_.z = link->xyz[2];
         base_wheels_[num_wheels_].name_ = joint_name;
         base_wheels_[num_wheels_].parent_ = NULL;
         base_wheels_[num_wheels_].joint_state_ = robot_state_->getJointState(joint_name);
         base_wheels_[num_wheels_].local_id_ = num_wheels_;
         if(joint_name.find("_r_") != string::npos)
            base_wheels_[num_wheels_].direction_multiplier_ = 1;
         else
            base_wheels_[num_wheels_].direction_multiplier_ = 1;

         wheel_speed_actual_.push_back(0);
         libTF::Vector *v=new libTF::Vector();
         base_wheels_position_.push_back(*v);
         wheel_speed_cmd_.push_back(0);

         base_wheels_[num_wheels_].velocity_controller_.init(robot_state_, joint_name, Pid(jcp_iter->p_gain,jcp_iter->i_gain,jcp_iter->d_gain,jcp_iter->windup));
         base_wheels_[num_wheels_].position_controller_.init(robot_state_, joint_name, Pid(jcp_iter->p_gain,jcp_iter->i_gain,jcp_iter->d_gain,jcp_iter->windup));

         num_wheels_++;
         delete v;
      }
   }

   for(int i =0; i < num_wheels_; i++)
   {
      link = urdf_model_.getJointLink(base_wheels_[i].name_);
      std::string parent_name = link->parent->joint->name;
      for(int j =0; j < num_casters_; j++)
      {
         if(parent_name == base_casters_[j].name_)
         {
            base_wheels_[i].parent_ = &base_casters_[j];
            break;
         }
      }
   }

   if(num_wheels_ > 0)
   {
      link = urdf_model_.getJointLink(base_wheels_[0].name_);
      robot_desc::URDF::Link::Geometry::Cylinder *wheel_geom = dynamic_cast<robot_desc::URDF::Link::Geometry::Cylinder*> (link->collision->geometry->shape);
      if (wheel_geom)
         wheel_radius_ = wheel_geom->radius;
      else
      {
         ROS_WARN("Wheel geom is not found in URDF, base controller will set wheel_radius_ for %s to DEFAULT_WHEEL_RADIUS %f\n.",base_wheels_[0].name_.c_str(),DEFAULT_WHEEL_RADIUS);
         wheel_radius_ = DEFAULT_WHEEL_RADIUS;
      }
   }
   else
   {
      wheel_radius_ = DEFAULT_WHEEL_RADIUS;
   }

   last_time_ = robot_state_->hw_->current_time_;

   sample_time_ = robot_state_->hw_->current_time_;

   cmd_received_timestamp_ = robot_state_->hw_->current_time_;
}

bool BaseControllerPos::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{

  ROS_INFO("BaseControllerPos:: name: %s",config->Attribute("name")); 
  TiXmlElement *elt = config->FirstChildElement("controller");
  std::vector<JointControlParam> jcp_vec;
  JointControlParam jcp;
  while (elt){
    TiXmlElement *jnt = elt->FirstChildElement("joint");
    jcp.p_gain = atof(jnt->FirstChildElement("pid")->Attribute("p"));
    jcp.i_gain = atof(jnt->FirstChildElement("pid")->Attribute("i"));
    jcp.d_gain = atof(jnt->FirstChildElement("pid")->Attribute("d"));
    jcp.windup = atof(jnt->FirstChildElement("pid")->Attribute("iClamp"));
    jcp.control_type = (std::string) elt->Attribute("type");
    jcp.joint_name = jnt->Attribute("name");
    jcp_vec.push_back(jcp);

    ROS_INFO("BaseControllerPos:: joint name: %s",jcp.joint_name.c_str()); 
    ROS_INFO("BaseControllerPos:: controller type: %s\n",jcp.control_type.c_str()); 

    elt = elt->NextSiblingElement("controller");
  }

  addParamToMap("kp_speed",&kp_speed_);
  addParamToMap("kp_caster_steer",&caster_steer_vel_gain_);
  addParamToMap("timeout",&timeout_);
  addParamToMap("max_x_dot",&(max_vel_.x));
  addParamToMap("max_y_dot",&(max_vel_.y));
  addParamToMap("max_yaw_dot",&(max_vel_.z));
  addParamToMap("max_x_accel",&(max_accel_.x));
  addParamToMap("max_y_accel",&(max_accel_.y));
  addParamToMap("max_yaw_accel",&(max_accel_.z));

  elt = config->FirstChildElement("map");

  while(elt)
  {
    if(elt->Attribute("name") == std::string("velocity_control"))
    {
      TiXmlElement *elt_key = elt->FirstChildElement("elem");
      while(elt_key)
      {
        *(param_map_[elt_key->Attribute("key")]) = atof(elt_key->GetText());
        elt_key = elt_key->NextSiblingElement("elem");
      }
    }
//    std::cout << "*************************************" << std::endl;
    elt = config->NextSiblingElement("map");
  }


  ROS_INFO("BaseControllerPos:: kp_speed %f",kp_speed_);
  ROS_INFO("BaseControllerPos:: kp_caster_steer  %f",caster_steer_vel_gain_);
  ROS_INFO("BaseControllerPos:: timeout %f",timeout_);
  ROS_INFO("BaseControllerPos:: max_x_dot %f",(max_vel_.x));
  ROS_INFO("BaseControllerPos:: max_y_dot %f",(max_vel_.y));
  ROS_INFO("BaseControllerPos:: max_yaw_dot %f",(max_vel_.z));
  ROS_INFO("BaseControllerPos:: max_x_accel %f",(max_accel_.x));
  ROS_INFO("BaseControllerPos:: max_y_accel %f",(max_accel_.y));
  ROS_INFO("BaseControllerPos:: max_yaw_accel %f",(max_accel_.z));

  init(jcp_vec,robot_state);

  ROS_INFO("BaseControllerPos:: Initialized");
  return true;
}

void BaseControllerPos::addParamToMap(std::string key, double *value)
{
  param_map_[key] = value;
}

void BaseControllerPos::getJointValues()
{
  bool speeds_are_valid = true;

  for(int i=0; i < num_casters_; i++)
    steer_angle_actual_[i] = base_casters_[i].joint_state_->position_;

  for(int i=0; i < num_wheels_; i++)
    wheel_speed_actual_[i] = base_wheels_[i].joint_state_->velocity_;

  for(int i=0; i < num_wheels_; i++)
    speeds_are_valid = speeds_are_valid && !isinf(wheel_speed_actual_[i]) && !isnan(wheel_speed_actual_[i]);

  if(!speeds_are_valid)
    ROS_WARN("BaseControllerPos:: input speed values are inf or nan");
}

void BaseControllerPos::computeWheelPositions()
{
  libTF::Vector res1;
  double steer_angle;
  for(int i=0; i < num_wheels_; i++)
  {
    steer_angle = base_wheels_[i].parent_->joint_state_->position_;
    res1 = base_wheels_[i].pos_.rot2D(steer_angle);
    res1 += base_wheels_[i].parent_->pos_;
    base_wheels_position_[i] = res1;
  }
}

void BaseControllerPos::update()
{
   for (int i = 0; i < num_casters_; ++i)
   {
      if(base_casters_[i].joint_state_ != NULL)
      {
         if (!base_casters_[i].joint_state_->calibrated_)
         {
            ROS_WARN("Casters are not calibrated");
         }
      }
      else
      {
         ROS_WARN("joint state not initialized: %s",base_casters_[i].name_.c_str());
      }
   }

  double current_time = robot_state_->hw_->current_time_;
  double dT = std::min<double>(current_time - last_time_,MAX_DT_);

  if((current_time - cmd_received_timestamp_) > timeout_)
  {
    cmd_vel_.x = 0;
    cmd_vel_.y = 0;
    cmd_vel_.z = 0;
    cmd_vel_magnitude_ = 0;
    setVelocityCmdTrajectory(cmd_vel_,max_accel_);
  }
  else
     cmd_vel_ = interpolateCommand(current_time-sample_time_);


  getJointValues();


  computeJointCommands(current_time-sample_time_,dT);

  setJointCommands();

  computeOdometry(current_time);

  updateJointControllers();

  last_time_ = current_time;

  if(new_cmd_available_)
  {
    if(pthread_mutex_trylock(&base_controller_lock_)==0)
    {
      desired_vel_.x = cmd_vel_t_.x;
      desired_vel_.y = cmd_vel_t_.y;
      desired_vel_.z = cmd_vel_t_.z;
      new_cmd_available_ = false;
      sample_time_ = current_time;
      setVelocityCmdTrajectory(desired_vel_,max_accel_);
      pthread_mutex_unlock(&base_controller_lock_);
    }
  }

}

libTF::Vector BaseControllerPos::interpolateCommand(double time)
{
   libTF::Vector result;

   cmd_vel_trajectory_->sample(current_vel_point_,time);

   cmd_vel_direction_.x = current_vel_point_.q_[0];
   cmd_vel_direction_.y = current_vel_point_.q_[1];
   cmd_vel_direction_.z = current_vel_point_.q_[2];
   cmd_vel_magnitude_ =  current_vel_point_.q_[3];

   result.x = current_vel_point_.q_[0] * current_vel_point_.q_[3];
   result.y = current_vel_point_.q_[1] * current_vel_point_.q_[3];
   result.z = current_vel_point_.q_[2] * current_vel_point_.q_[3];

//   ROS_INFO("Interpolate command: done sampling trajectory: %f %f %f",result.x, result.y, result.z);

   return result;
}

void BaseControllerPos::setVelocityCmdTrajectory(libTF::Vector new_cmd, libTF::Vector max_rate)
{
   double dt_min_x(0), dt_min_y(0), dt_min_t(0), dt(0);
   double new_cmd_mag = sqrt(new_cmd.x*new_cmd.x+new_cmd.y*new_cmd.y+new_cmd.z*new_cmd.z);
   libTF::Vector new_vel_direction;

   if(max_rate.x > EPS)
      dt_min_x = fabs(new_cmd.x-cmd_vel_.x)/max_rate.x;
   if(max_rate.y > EPS)
      dt_min_y = fabs(new_cmd.y-cmd_vel_.y)/max_rate.y;
   if(max_rate.z > EPS)
      dt_min_t = fabs(new_cmd.z-cmd_vel_.z)/max_rate.z;

   dt = std::max(std::max(dt_min_x,dt_min_y),dt_min_t);

//   ROS_INFO("Current command: %f %f %f",cmd_vel_.x,cmd_vel_.y,cmd_vel_.z);


   cmd_vel_points_[0].q_[0] = cmd_vel_direction_.x;
   cmd_vel_points_[0].q_[1] = cmd_vel_direction_.y;
   cmd_vel_points_[0].q_[2] = cmd_vel_direction_.z;
   cmd_vel_points_[0].q_[3] = cmd_vel_magnitude_;
   cmd_vel_points_[0].time_ = 0.0;

   if(new_cmd_mag > EPS)
   {
      new_vel_direction.x = new_cmd.x/new_cmd_mag;
      new_vel_direction.y = new_cmd.y/new_cmd_mag;
      new_vel_direction.z = new_cmd.z/new_cmd_mag;
   }
   else
   {
      new_vel_direction.x = cmd_vel_direction_.x;
      new_vel_direction.y = cmd_vel_direction_.y;
      new_vel_direction.z = cmd_vel_direction_.z; 
   }

   cmd_vel_points_[1].q_[0] = new_vel_direction.x;
   cmd_vel_points_[1].q_[1] = new_vel_direction.y;
   cmd_vel_points_[1].q_[2] = new_vel_direction.z;
   cmd_vel_points_[1].q_[3] = new_cmd_mag;
   cmd_vel_points_[1].time_ = dt;

   cmd_vel_trajectory_->setTrajectory(cmd_vel_points_);

//   ROS_INFO("New command: %f %f %f",new_cmd.x,new_cmd.y,new_cmd.z);
//   ROS_INFO("Max rate   : %f %f %f",max_rate.x,max_rate.y,max_rate.z);
//   ROS_INFO("New command issued: %f",dt);
}

void BaseControllerPos::computeJointCommands(double current_sample_time, double dT)
{
  computeWheelPositions();

  computeDesiredCasterSteer(current_sample_time, dT);

  computeDesiredWheelSpeeds();
}

void BaseControllerPos::setJointCommands()
{
  setDesiredCasterSteer();

  setDesiredWheelSpeeds();
}

void BaseControllerPos::computeDesiredCasterSteer(double current_sample_time, double dT)
{
   libTF::Vector result, result_dt;
   libTF::Vector cmd_vel_dt;

   cmd_vel_dt = interpolateCommand(current_sample_time+dT);

  double steer_angle_desired(0.0), steer_angle_desired_m_pi(0.0);
  double error_steer(0.0),error_steer_m_pi(0.0);

  double steer_angle_desired_dt(0.0), steer_angle_desired_m_pi_dt(0.0);
  double error_steer_dt(0.0),error_steer_m_pi_dt(0.0);
  double trans_vel = sqrt(cmd_vel_.x * cmd_vel_.x + cmd_vel_.y * cmd_vel_.y);

  for(int i=0; i < num_casters_; i++)
  {
    result    = computePointVelocity2D(base_casters_[i].pos_, cmd_vel_);
    result_dt = computePointVelocity2D(base_casters_[i].pos_, cmd_vel_dt);

    if( trans_vel < CMD_VEL_TRANS_EPS && fabs(cmd_vel_.z) < CMD_VEL_ROT_EPS)
    {
      steer_angle_desired = steer_angle_stored_[i];
      steer_angle_desired_dt = steer_angle_stored_[i];
    }
    else
    {
      steer_angle_desired = atan2(result.y,result.x);
      steer_angle_desired_dt = atan2(result_dt.y,result_dt.x);
      steer_angle_stored_[i] = steer_angle_desired;
    }
    steer_angle_desired_m_pi = angles::normalize_angle(steer_angle_desired+M_PI);
    steer_angle_desired_m_pi_dt = angles::normalize_angle(steer_angle_desired_dt+M_PI);

    error_steer = angles::shortest_angular_distance(steer_angle_desired, steer_angle_actual_[i]);
    error_steer_m_pi = angles::shortest_angular_distance(steer_angle_desired_m_pi, steer_angle_actual_[i]);

    error_steer_dt = angles::shortest_angular_distance(steer_angle_desired_dt, steer_angle_actual_[i]);
    error_steer_m_pi_dt = angles::shortest_angular_distance(steer_angle_desired_m_pi_dt, steer_angle_actual_[i]);

    if(fabs(error_steer_m_pi) < fabs(error_steer))
    {
      error_steer = error_steer_m_pi;
      steer_angle_desired = steer_angle_desired_m_pi;
    }

    if(fabs(error_steer_m_pi_dt) < fabs(error_steer_dt))
    {
      error_steer_dt = error_steer_m_pi_dt;
      steer_angle_desired_dt = steer_angle_desired_m_pi_dt;
    }

    steer_position_desired_[i] = steer_angle_desired;
    steer_velocity_desired_[i] = (steer_angle_desired_dt - steer_angle_desired)/dT - kp_speed_*error_steer;
//    ROS_INFO("Caster speed component: %d %f %f %f",i,steer_angle_desired,steer_angle_desired_dt,steer_velocity_desired_[i]);
  }
}

void BaseControllerPos::setDesiredCasterSteer()
{
  for(int i=0; i < num_casters_; i++)
     base_casters_[i].position_controller_.setCommand(steer_position_desired_[i]);
}

void BaseControllerPos::computeDesiredWheelSpeeds()
{
  libTF::Vector wheel_point_velocity;
  libTF::Vector wheel_point_velocity_projected;
  libTF::Vector wheel_caster_steer_component;
  libTF::Vector caster_2d_velocity;

  caster_2d_velocity.x = 0;
  caster_2d_velocity.y = 0;
  caster_2d_velocity.z = 0;

  double steer_angle_actual = 0;
  for(int i=0; i < (int) num_wheels_; i++)
  {
    caster_2d_velocity.z = caster_steer_vel_gain_*steer_velocity_desired_[base_wheels_[i].parent_->local_id_];
    steer_angle_actual = base_wheels_[i].parent_->joint_state_->position_;
    wheel_point_velocity = computePointVelocity2D(base_wheels_position_[i],cmd_vel_);

    wheel_caster_steer_component = computePointVelocity2D(base_wheels_[i].pos_,caster_2d_velocity);
    wheel_point_velocity_projected = wheel_point_velocity.rot2D(-steer_angle_actual);
    wheel_speed_cmd_[i] = (wheel_point_velocity_projected.x + wheel_caster_steer_component.x)/(wheel_radius_);
//    ROS_INFO("Wheel speed cmd/actual %d %f %f",i,wheel_speed_cmd_[i],wheel_speed_actual_[i]);
  }
}

void BaseControllerPos::setDesiredWheelSpeeds()
{
  for(int i=0; i < (int) num_wheels_; i++)
  {
     base_wheels_[i].velocity_controller_.setCommand(wheel_speed_cmd_[i]);
//     ROS_INFO("Wheel speed cmd/actual %d %f %f",i,wheel_speed_cmd_[i],wheel_speed_actual_[i]);
  }
}


void BaseControllerPos::updateJointControllers()
{
  for(int i=0; i < num_wheels_; i++)
    base_wheels_[i].velocity_controller_.update();
  for(int i=0; i < num_casters_; i++)
    base_casters_[i].position_controller_.update();
}

void BaseControllerPos::setOdomMessage(std_msgs::RobotBase2DOdom &odom_msg_)
{
  odom_msg_.header.frame_id   = "odom";
  odom_msg_.header.stamp.sec  = (unsigned long)floor(robot_state_->hw_->current_time_);
  odom_msg_.header.stamp.nsec = (unsigned long)floor(  1e9 * (  robot_state_->hw_->current_time_ - odom_msg_.header.stamp.sec) );

  odom_msg_.pos.x  = base_odom_position_.x;
  odom_msg_.pos.y  = base_odom_position_.y;
  odom_msg_.pos.th = angles::normalize_angle(base_odom_position_.z);

  odom_msg_.vel.x  = base_odom_velocity_.x;
  odom_msg_.vel.y  = base_odom_velocity_.y;
  odom_msg_.vel.th = base_odom_velocity_.z;
}


void BaseControllerPos::getOdometry(double &x, double &y, double &w, double &vx, double &vy, double &vw)
{
  x = base_odom_position_.x;
  y = base_odom_position_.y;
  w = base_odom_position_.z;

  vx = base_odom_velocity_.x;
  vy = base_odom_velocity_.y;
  vw = base_odom_velocity_.z;
}


Vector BaseControllerPos::computePointVelocity2D(const Vector& pos, const Vector& vel)
{
  Vector result;

  result.x = vel.x - pos.y * vel.z;
  result.y = vel.y + pos.x * vel.z;
  result.z = 0;

  return result;
}

void BaseControllerPos::computeOdometry(double time)
{
  double dt = time-last_time_;

  computeBaseVelocity();

  libTF::Vector base_odom_delta = (base_odom_velocity_*dt).rot2D(base_odom_position_.z);

  base_odom_delta.z = base_odom_velocity_.z * dt;
  base_odom_position_ += base_odom_delta;

  odometer_distance_ += sqrt(base_odom_delta.x*base_odom_delta.x + base_odom_delta.y*base_odom_delta.y);
  odometer_angle_ += base_odom_delta.z;
}

void BaseControllerPos::computeBaseVelocity()
{
  Matrix A(2*num_wheels_,1);
  Matrix C(2*num_wheels_,3);
  Matrix D(3,1);
  double steer_angle;
  double wheel_speed;
  libTF::Vector caster_2d_velocity;
  libTF::Vector wheel_caster_steer_component;

  caster_2d_velocity.x = 0;
  caster_2d_velocity.y = 0;

  for(int i = 0; i < num_wheels_; i++) {
    caster_2d_velocity.z = base_wheels_[i].parent_->joint_state_->velocity_;
    wheel_caster_steer_component = computePointVelocity2D(base_wheels_[i].pos_,caster_2d_velocity);
    wheel_speed = wheel_speed_actual_[i]-wheel_caster_steer_component.x/(wheel_radius_);

    steer_angle = base_wheels_[i].parent_->joint_state_->position_;

//    A.element(i*2,0)   = cos(steer_angle)*wheel_radius_*wheel_speed;
//    A.element(i*2+1,0) = sin(steer_angle)*wheel_radius_*wheel_speed;

    A.element(i*2,0)   = wheel_radius_*wheel_speed;
    A.element(i*2+1,0) = 0;


//  }
//  for(int i = 0; i < num_wheels_; i++) {

    C.element(i*2, 0)   = cos(steer_angle);
    C.element(i*2, 1)   = sin(steer_angle);
    C.element(i*2, 2)   = -cos(steer_angle) * base_wheels_position_[i].y + sin(steer_angle) *  base_wheels_position_[i].x;
    C.element(i*2+1, 0)   = -sin(steer_angle);
    C.element(i*2+1, 1)   = cos(steer_angle);
    C.element(i*2+1, 2)   = sin(steer_angle) * base_wheels_position_[i].y + cos(steer_angle) *  base_wheels_position_[i].x;
  }
  //   D = pseudoInverse(C)*A;
  D = iterativeLeastSquares(C,A,ils_weight_type_,ils_max_iterations_);
  base_odom_velocity_.x = (double)D.element(0,0);
  base_odom_velocity_.y = (double)D.element(1,0);
  base_odom_velocity_.z = (double)D.element(2,0);
}

Matrix BaseControllerPos::pseudoInverse(const Matrix M)
{
  Matrix result;
  //int rows = this->rows();
  //int cols = this->columns();
  // calculate SVD decomposition
  Matrix U,V;
  DiagonalMatrix D;
  NEWMAT::SVD(M,D,U,V, true, true);
  Matrix Dinv = D.i();
  result = V * Dinv * U.t();
  return result;
}

/*std::ostream & controller::operator<<(std::ostream& mystream, const controller::BaseParam &bp)
{
  mystream << "BaseParam" << bp.name_ << endl << "position " << bp.pos_ << "id " << bp.local_id_ << endl << "joint " << bp.joint_state_->joint_->name_ << endl << endl;
  return mystream;
  };*/

NEWMAT::Matrix BaseControllerPos::iterativeLeastSquares(NEWMAT::Matrix A, NEWMAT::Matrix b, std::string weight_type, int max_iter)
{
  NEWMAT::IdentityMatrix ident_matrix(2*num_wheels_);
  NEWMAT::Matrix weight_matrix(2*num_wheels_,2*num_wheels_);
  NEWMAT::Matrix a_fit(2*num_wheels_,3);
  NEWMAT::Matrix b_fit(2*num_wheels_,1);
  NEWMAT::Matrix residual(2*num_wheels_,1);
  NEWMAT::Matrix x_fit(3,1);

  weight_matrix = ident_matrix;

  for(int i=0; i < max_iter; i++)
  {
    a_fit = weight_matrix * A;
    b_fit = weight_matrix * b;
    x_fit = pseudoInverse(a_fit)*b_fit;
    residual = b - A * x_fit;

    for(int j=1; j <= num_wheels_; j++)
    {
      int fw = 2*j-1;
      int sw = fw+1;
      if(fabs(residual(fw,1)) > fabs(residual(sw,1)))
      {
        residual(fw,1) = fabs(residual(fw,1));
        residual(sw,1) = residual(fw,1);
      }
      else
      {
        residual(fw,1) = fabs(residual(sw,1));
        residual(sw,1) = residual(fw,1);
      }
    }
    weight_matrix = findWeightMatrix(residual,weight_type);
  }
  return x_fit;
};

NEWMAT::Matrix BaseControllerPos::findWeightMatrix(NEWMAT::Matrix residual, std::string weight_type)
{
  NEWMAT::Matrix w_fit(2*num_wheels_,2*num_wheels_);
  NEWMAT::IdentityMatrix ident_matrix(2*num_wheels_);

  w_fit = ident_matrix;
  double epsilon = 0;
  double g_sigma = 0.1;

  if(weight_type == std::string("BubeLagan"))
  {
    for(int i=1; i <= 2*num_wheels_; i++) //NEWMAT indexing starts from 1
    {
      if(fabs(residual(i,1) > epsilon))
        epsilon = fabs(residual(i,1));
    }
    epsilon = epsilon/100.0;
  }
  for(int i=1; i <= 2*num_wheels_; i++) //NEWMAT indexing starts from 1
  {
    if(weight_type == std::string("L1norm"))
    {
      w_fit(i,i) = 1.0/(1 + sqrt(fabs(residual(i,1))));
    }
    else if(weight_type == std::string("fair"))
    {
      w_fit(i,i) = 1.0/(1 + fabs(residual(i,1)));
    }
    else if(weight_type == std::string("Cauchy"))
    {
      w_fit(i,i) = 1.0/(1 + residual(i,1)*residual(i,1));
    }
    else if(weight_type == std::string("BubeLangan"))
    {
      w_fit(i,i) = 1.0/pow((1 + pow((residual(i,1)/epsilon),2)),0.25);
    }
    else if(weight_type == std::string("Gaussian"))
    {
      w_fit(i,i) = sqrt(exp(-pow(residual(i,1),2)/(2*g_sigma*g_sigma)));
    }
    else // default to fair
    {
      w_fit(i,i) = 1.0/(0.1 + sqrt(fabs(residual(i,1))));
    }
  }
  return w_fit;
};


ROS_REGISTER_CONTROLLER(BaseControllerPosNode)

  BaseControllerPosNode::BaseControllerPosNode()
{
  c_ = new BaseControllerPos();
  node = ros::node::instance();
  last_time_message_sent_ = 0.0;
  odom_publish_rate_ = 100.0;
  odom_publish_delta_t_ = 1.0/odom_publish_rate_;
  publisher_ = NULL;
  transform_publisher_ = NULL;
  odometer_publisher_ = NULL;
}

BaseControllerPosNode::~BaseControllerPosNode()
{
  node->unadvertise_service(service_prefix + "/set_command");
  node->unadvertise_service(service_prefix + "/get_actual");
  node->unadvertise_service(service_prefix + "/set_wheel_radius_multiplier");
  node->unadvertise_service(service_prefix + "/get_wheel_radius_multiplier");

  node->unsubscribe("cmd_vel");

  publisher_->stop();
  transform_publisher_->stop();
  delete publisher_;
  delete transform_publisher_;
  delete odometer_publisher_;
  delete c_;
}

void BaseControllerPosNode::update()
{
  double time = c_->robot_state_->hw_->current_time_;
//  ROS_INFO("In Base Controller Node");  
  c_->update();
  c_->setOdomMessage(odom_msg_);

  if (time-last_time_message_sent_ >= odom_publish_delta_t_) // send odom message
  {

    if (odometer_publisher_->trylock())
    {
      odometer_publisher_->msg_.distance = c_->odometer_distance_; 
      odometer_publisher_->msg_.angle = c_->odometer_angle_; 
      odometer_publisher_->unlockAndPublish();      
    }

    if (publisher_->trylock())
    {
      c_->setOdomMessage(publisher_->msg_);
      publisher_->unlockAndPublish() ;
      last_time_message_sent_ = time;
    }

    if (transform_publisher_->trylock())
    {
      double x=0,y=0,yaw=0,vx,vy,vyaw;
      this->getOdometry(x,y,yaw,vx,vy,vyaw);
      tf::TransformEuler &out = transform_publisher_->msg_.eulers[0];
      out.header.stamp.from_double(time);
      out.header.frame_id = "odom";
      out.parent = "base_footprint";
      out.x = -x*cos(yaw) - y*sin(yaw);
      out.y = +x*sin(yaw) - y*cos(yaw);
      out.z = 0;
      out.roll = 0;
      out.pitch = 0;
      out.yaw = angles::normalize_angle(-yaw);


      tf::TransformEuler &out2 = transform_publisher_->msg_.eulers[1];
      out2.header.stamp.from_double(time);
      out2.header.frame_id = "base_footprint";
      out2.parent = "base_link";
      out2.x = 0;
      out2.y = 0;
      out2.z = -c_->wheel_radius_;
      out2.roll = 0;
      out2.pitch = 0;
      out2.yaw = 0;

      transform_publisher_->unlockAndPublish() ;
    }
  }
}

bool BaseControllerPosNode::setCommand(
  pr2_mechanism_controllers::SetBaseCommand::request &req,
  pr2_mechanism_controllers::SetBaseCommand::response &resp)
{
  libTF::Vector command;
  command.x = req.vx;
  command.y = req.vy;
  command.z = req.vw;
  c_->setCommand(command);
  command = c_->getCommand();
  resp.vx = command.x;
  resp.vy = command.y;
  resp.vw = command.z;
  return true;
}

void BaseControllerPosNode::setCommand(double vx, double vy, double vw)
{
  libTF::Vector command;
  command.x = vx;
  command.y = vy;
  command.z = vw;
  c_->setCommand(command);
  //std::cout << "BaseControllerPos:: odom_publish_rate: " << odom_publish_rate_ << endl;
}


bool BaseControllerPosNode::getCommand(
  pr2_mechanism_controllers::GetBaseCommand::request &req,
  pr2_mechanism_controllers::GetBaseCommand::response &resp)
{
  libTF::Vector command;
  command = c_->getCommand();
  resp.vx = command.x;
  resp.vy = command.y;
  resp.vw = command.z;

  return true;
}

bool BaseControllerPosNode::getWheelRadiusMultiplier(
  pr2_mechanism_controllers::WheelRadiusMultiplier::request &req,
  pr2_mechanism_controllers::WheelRadiusMultiplier::response &resp)
{
  double param_multiplier;
  node->param<double>("base_controller/wheel_radius_multiplier",param_multiplier,1.0);
  resp.radius_multiplier = param_multiplier;

  return true;
}

bool BaseControllerPosNode::setWheelRadiusMultiplier(
  pr2_mechanism_controllers::WheelRadiusMultiplier::request &req,
  pr2_mechanism_controllers::WheelRadiusMultiplier::response &resp)
{
  double calibration_multiplier = req.radius_multiplier;
  ROS_INFO("Received radius multiplier %f ",calibration_multiplier); 
  c_->wheel_radius_ *= calibration_multiplier;

  double param_multiplier;
  node->param<double>("base_controller/wheel_radius_multiplier",param_multiplier,1.0);

  node->set_param("base_controller/wheel_radius_multiplier",param_multiplier*calibration_multiplier);

  return true;
}


bool BaseControllerPosNode::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{
  service_prefix = config->Attribute("name");

  ROS_INFO("Initializing base controller node");

  assert(robot_state); //this happens, see pr ticket 351,but not sure why yet

  if(!c_->initXml(robot_state, config))
    return false;

  ROS_INFO("Initialized base controller");

  node->advertise_service(service_prefix + "/set_wheel_radius_multiplier", &BaseControllerPosNode::setWheelRadiusMultiplier,this);
  node->advertise_service(service_prefix + "/get_wheel_radius_multiplier", &BaseControllerPosNode::getWheelRadiusMultiplier,this);


  node->advertise_service(service_prefix + "/set_command", &BaseControllerPosNode::setCommand, this);
  node->advertise_service(service_prefix + "/get_command", &BaseControllerPosNode::getCommand, this); //FIXME: this is actually get command, just returning command for testing.

  node->advertise_service(service_prefix + "/set_command", &BaseControllerPosNode::setCommand, this);
  node->subscribe("cmd_vel", baseVelMsg, &BaseControllerPosNode::CmdBaseVelReceived, this,1);


  if (publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete publisher_ ;
  publisher_ = new misc_utils::RealtimePublisher <std_msgs::RobotBase2DOdom> ("odom", 1) ;

  if (odometer_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete odometer_publisher_ ;
  odometer_publisher_ = new misc_utils::RealtimePublisher <pr2_msgs::Odometer> (service_prefix + "/odometer", 1) ;

  if (transform_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete transform_publisher_ ;
  transform_publisher_ = new misc_utils::RealtimePublisher <tf::TransformArray> ("TransformArray", 5) ;

  node->param<double>("base_controller/odom_publish_rate",odom_publish_rate_,100);

  double multiplier;
  node->param<double>("base_controller/wheel_radius_multiplier",multiplier,1.0);
  c_->wheel_radius_ = c_->wheel_radius_*multiplier;

  transform_publisher_->msg_.set_eulers_size(NUM_TRANSFORMS);

  if(odom_publish_rate_ > 1e-5)
    odom_publish_delta_t_ = 1.0/odom_publish_rate_;

  ROS_INFO("Initialized base controller node");

  return true;
}

void BaseControllerPosNode::CmdBaseVelReceived()
{
  this->ros_lock_.lock();
  this->setCommand(baseVelMsg.vx,baseVelMsg.vy,baseVelMsg.vw);
  this->ros_lock_.unlock();
}

void BaseControllerPosNode::getOdometry(double &x, double &y, double &w, double &vx, double &vy, double &vw)
{
  c_->getOdometry(x,y,w,vx,vy,vw);
}
