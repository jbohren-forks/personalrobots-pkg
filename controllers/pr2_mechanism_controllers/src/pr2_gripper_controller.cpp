/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 /*
 * Author: Sachin Chitta and Matthew Piccoli
 */

#include <pr2_mechanism_controllers/pr2_gripper_controller.h>
#include <fstream> //TODO:now that I have something better, should I delete this?
#define pi 3.1415926

using namespace controller;
ROS_REGISTER_CONTROLLER(Pr2GripperController)

Pr2GripperController::Pr2GripperController()
{
  state_publisher_ = NULL;
}

Pr2GripperController::~Pr2GripperController()
{
  //TODO::delete state_publisher
  service_thread_->join();
}
//TODO::switch to node handles
bool Pr2GripperController::init(mechanism::RobotState *robot_state, const ros::NodeHandle &node)
{
  double timeout_duration_double, timeout_duration_double_steady;
  double p, i, d, i1, i2;
  pthread_mutex_init(&pr2_gripper_controller_lock_,NULL);
  node_ = node;
  grasp_cmd_.cmd = "move";
  grasp_cmd_.val = 0.0;
  new_cmd_available_ = false;
  robot_state_ = robot_state;
  //name_ = config->Attribute("name"); //"l_gripper" or "r_gripper" expected
  name_ = node.getNamespace();
//  mechanism::Link *link = robot_state_->model_->getLink(name_ + "_link");
  joint_ = robot_state->getJointState(name_ + "_joint");
  joint_controller_.init(robot_state_, name_ + "_joint");
  node_.param<double>("default_effort",default_effort_,joint_->joint_->effort_limit_);
  node_.param<double>("timeout", timeout_, 0.0);
  node_.param<double>("break_stiction_amplitude", break_stiction_amplitude_, 0.0);
  node_.param<double>("break_stiction_period", break_stiction_period_, 0.1);
  node_.param<double>("break_stiction_velocity", break_stiction_velocity_, 0.005);
  node_.param<double>("proportional_offset", proportional_offset_, 0.01);
  node_.param<double>("stopped_threshold", stopped_threshold_, 0.001);
  node_.param<double>("timeout_duration_", timeout_duration_double, 10.0);
  node_.param<double>("timeout_duration_steady", timeout_duration_double_steady, 3.0);
  node_.param<double>("low_force", low_force_, 15.0);
  node_.param<double>("high_force", high_force_, 20.0);
  node_.param<double>("vel_p", p, 15000.0);
  node_.param<double>("vel_i", i, 25.0);
  node_.param<double>("vel_d", d, 0.0);
  node_.param<double>("vel_p", i1, 100.0);
  node_.param<double>("vel_p", i2, -100.0);
  node_.param<double>("default_low_speed", default_low_speed_, 0.01);
  node_.param<double>("default_high_speed", default_high_speed_, 0.015);
  node_.param<int>("contact_threshold", contact_threshold_, 1000);
  node_.param<int>("contact_threshold_individual", contact_threshold_individual_, 100);
  node_.param<int>("num_pressure_pads_front", num_pressure_pads_front_, 15);
  node_.param<int>("num_pressure_pads_side", num_pressure_pads_side_, 7);
  node_.param<std::string>("break_stiction_type", break_stiction_type_, "none");
  node_.param<std::string>("fingertip_sensor_topic", fingertip_sensor_topic_, "/pressure/" + name_ + "_motor");
  ros::AdvertiseServiceOptions ops = ros::AdvertiseServiceOptions::create<manipulation_srvs::GraspClosedLoop>("/grasp_closed_loop", boost::bind(&Pr2GripperController::grasp_cl_srv, this, _1, _2), ros::VoidPtr(), &service_queue_);
  grasp_service_ = node_.advertiseService(ops);
  service_thread_ = new boost::thread(boost::bind(&Pr2GripperController::callbackThread, this));
  //grasp_service_ = node_.advertiseService("/grasp_closed_loop", &Pr2GripperController::grasp_cl_srv, this);
  //TODO::find out why these can only be done with node_ and not node
  cmd_sub_ = node_.subscribe<pr2_mechanism_controllers::GripperControllerCmd>("/" + name_+"_cmd", 1, &Pr2GripperController::command_callback, this);
  pressure_sub_ = node_.subscribe<ethercat_hardware::PressureState>(fingertip_sensor_topic_, 1, &Pr2GripperController::pressure_state_callback, this);
  timeout_duration_ = ros::Duration().fromSec(timeout_duration_double);
  timeout_duration_steady_ = ros::Duration().fromSec(timeout_duration_double_steady);
  if (state_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete state_publisher_;
  state_publisher_ = new realtime_tools::RealtimePublisher <pr2_msgs::GripperControllerState> (name_ + "/state", 1);
  pressure_state_.data0.resize(num_pressure_pads_front_+num_pressure_pads_side_);
  pressure_state_.data1.resize(num_pressure_pads_front_+num_pressure_pads_side_);
  fingertip_sensor_start0_.resize(num_pressure_pads_front_);
  fingertip_sensor_start1_.resize(num_pressure_pads_front_);
  fingertip_sensor_first_peak0_.resize(num_pressure_pads_front_);
  fingertip_sensor_first_peak1_.resize(num_pressure_pads_front_);
  fingertip_sensor_first_steady0_.resize(num_pressure_pads_front_);
  fingertip_sensor_first_steady1_.resize(num_pressure_pads_front_);
  fingertip_sensor_second_peak0_.resize(num_pressure_pads_front_);
  fingertip_sensor_second_peak1_.resize(num_pressure_pads_front_);
  fingertip_sensor_second_steady0_.resize(num_pressure_pads_front_);
  fingertip_sensor_second_steady1_.resize(num_pressure_pads_front_);
  fingertip_sensor_sides_start0_.resize(num_pressure_pads_side_);
  fingertip_sensor_sides_start1_.resize(num_pressure_pads_side_);
  p_i_d_.initPid(p,i,d,i1,i2);
  return true;
}

bool Pr2GripperController::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{
  return init(robot_state,ros::NodeHandle(config->Attribute("name")));
}

void Pr2GripperController::update()
{
  //do nothing if the joint is not calibrated
  if (!joint_->calibrated_)
  {
    ROS_INFO("gripper not calibrated!");
    return;  // motor's not calibrated
  }

  double current_time = robot_state_->hw_->current_time_;
  if(new_cmd_available_)
  {
    if(pthread_mutex_trylock(&pr2_gripper_controller_lock_) == 0) //the callback is not writing to grasp_cmd_
    {
      if(grasp_cmd_desired_.cmd.compare("Event")) //If the incoming message is an event, don't copy the command
      {
        grasp_cmd_desired_.cmd = grasp_cmd_.cmd;
        grasp_cmd_desired_.start = grasp_cmd_.start;
        grasp_cmd_desired_.end = grasp_cmd_.end;
        grasp_cmd_desired_.time = grasp_cmd_.time;
        grasp_cmd_desired_.val = grasp_cmd_.val;
        closed_loop_grasp_state = unstarted;
        //ROS_INFO("Copying command, val is %f", grasp_cmd_desired_.val);
      }
      new_cmd_available_ = false;
      pthread_mutex_unlock(&pr2_gripper_controller_lock_);
      if(grasp_cmd_desired_.cmd.compare("step") == 0) //in here because it depends on former joint_controller_.command_, can't be changing with every update
      {
        velocity_mode_ = false;
        joint_controller_.command_ = effortLimit(stepMove(grasp_cmd_desired_.val));
        last_commanded_command = joint_controller_.command_;
      }
    }
  }

  //check for timeout
  if((current_time - cmd_received_timestamp_) <= timeout_ || timeout_ == 0.0) //continue with what you were doing
  {
    last_commanded_command = parseMessage(grasp_cmd_desired_);
    if(!velocity_mode_)
    {
      p_i_d_.reset();
      double direction = 1.0;
      last_commanded_command = effortLimit(last_commanded_command); //set value
      if(last_commanded_command < 0.0)
      {
        direction = -1.0;
      }
      if(break_stiction_type_.compare("sine") == 0 && fabs(joint_->velocity_) < break_stiction_velocity_ && last_commanded_command != 0.0)
      {
        joint_controller_.command_ = last_commanded_command + direction*(sin(2.0*pi*(current_time - cmd_received_timestamp_)/break_stiction_period_)*break_stiction_amplitude_ + break_stiction_amplitude_);
      }
      else if(break_stiction_type_.compare("ramp") == 0 && fabs(joint_->velocity_) < break_stiction_velocity_ && last_commanded_command != 0.0)
      {
        joint_controller_.command_ = last_commanded_command + direction*rampMove(0.0, break_stiction_amplitude_, break_stiction_period_, 0.0);
      }
      else
      {
        joint_controller_.command_ = last_commanded_command;
      }
      last_velocity_time_ = 0.0;
    }
    else
    {
      if(last_velocity_time_ == 0.0) //it was just in effort mode
      {
        if(last_commanded_command >= 0.0)
        {
          p_i_d_.setCurrentCmd(low_force_);
        }
        else
        {
          p_i_d_.setCurrentCmd(-1.0*low_force_);
        }
        joint_controller_.command_ = p_i_d_.updatePid(joint_->velocity_ - last_commanded_command, 0.0);
      }
      else
        joint_controller_.command_ = p_i_d_.updatePid(joint_->velocity_ - last_commanded_command, current_time - last_time_);
      last_velocity_time_ = current_time;
    }

    joint_controller_.update(); //update value
  }
  else //if timed out, don't do anything
  {
    //stop motor
    //set value
    joint_controller_.command_ = 0.0;
    last_commanded_command = 0.0;
    //update value
    joint_controller_.update();
  }

  //Publish state
  if(state_publisher_->trylock())
  {
    state_publisher_->msg_.joint_commanded_effort = joint_->commanded_effort_;
    state_publisher_->msg_.joint_applied_effort = joint_->applied_effort_;
    state_publisher_->msg_.joint_name = joint_->joint_->name_;
    state_publisher_->msg_.joint_velocity = joint_->velocity_;
    state_publisher_->msg_.joint_position = joint_->position_;
    state_publisher_->unlockAndPublish() ;
  }
  last_time_ = current_time;
}

bool Pr2GripperController::starting()
{
  last_time_ = robot_state_->hw_->current_time_;
  cmd_received_timestamp_ = robot_state_->hw_->current_time_;
  return true;
}

bool Pr2GripperController::stopping()
{
  if(state_publisher_)
  {
    state_publisher_->stop();
    delete state_publisher_;
  }
  return true;
}

double Pr2GripperController::rampMove(double start_force, double end_force, double time, double hold)
{
  double del_force = end_force - start_force;
  double del_time = robot_state_->hw_->current_time_ - cmd_received_timestamp_;
  if(del_time > time)
  	return hold;
  return start_force + del_time/time*del_force;
}

double Pr2GripperController::stepMove(double step_size)
{
  return last_commanded_command + step_size;
}

double Pr2GripperController::grasp(bool service_callback, bool closed_loop)
{
  //starting grasp
  if(closed_loop_grasp_state == unstarted)
  {
    //reset variables
    for(int i = 0; i < num_pressure_pads_front_; i++)
    {
      fingertip_sensor_start0_[i]=0;
      fingertip_sensor_start1_[i]=0;
      fingertip_sensor_first_peak0_[i]=0;
      fingertip_sensor_first_peak1_[i]=0;
      fingertip_sensor_first_steady0_[i]=0;
      fingertip_sensor_first_steady1_[i]=0;
      fingertip_sensor_second_peak0_[i]=0;
      fingertip_sensor_second_peak1_[i]=0;
      fingertip_sensor_second_steady0_[i]=0;
      fingertip_sensor_second_steady1_[i]=0;
    }
    //remember values from edges of fingertips
    for(int i = 0; i < num_pressure_pads_side_; i++)
    {
      fingertip_sensor_sides_start0_[i] = 0;
      fingertip_sensor_sides_start1_[i] = 0;
    }
    position_first_contact=-1;
    position_second_contact=-1;
    position_first_compression=-1;
    position_second_compression=-1;
    spring_const=-1;
    peak_force_first_grasp = 0;
    peak_force_second_grasp = 0;
    grasp_open_close_timestamp = ros::Time::now();
    closed_loop_grasp_state = open0;
    velocity_mode_ = false;
    return default_effort_;
  }

  //open
  else if(closed_loop_grasp_state == open0)
  {
    //check for any timeouts
    if(grasp_open_close_timestamp + timeout_duration_ < ros::Time::now())
    {
      ROS_WARN("grasp failed due to a timeout");
      closed_loop_grasp_state = failed;
      //report that there was a failure
      if(service_callback)
      {
        service_response_.result = 1;
        service_response_.distance = joint_->position_;
        service_response_.effort = default_effort_;
        service_response_.stiffness = -1.0;
        service_response_.force_peak0 = -1.0;
        service_response_.force_steady0 = -1.0;
        service_response_.force_peak1 = -1.0;
        service_response_.force_steady1 = -1.0;
        service_response_.distance_compressed_first = -1.0;
        service_response_.distance_compressed_second = -1.0;
      }
      velocity_mode_ = false;
      return default_effort_;
    }
    //if the gripper is done opening
    else if(joint_->velocity_ < stopped_threshold_ && grasp_open_close_timestamp.toSec() + .1 < ros::Time::now().toSec())
    {
      //read the gripper pads to zero them
      for(int i = 0; i < num_pressure_pads_front_; i++) //the front pads are 7-21
      {
        fingertip_sensor_start0_[i] = pressure_state_.data0[i+num_pressure_pads_side_];
        fingertip_sensor_start1_[i] = pressure_state_.data1[i+num_pressure_pads_side_];
      }
      for(int i = 0; i < num_pressure_pads_side_; i++)
      {
        fingertip_sensor_sides_start0_[i] = pressure_state_.data0[i];
        fingertip_sensor_sides_start1_[i] = pressure_state_.data1[i];
      }
      //chage state
      closed_loop_grasp_state = close0_closing;
      grasp_open_close_timestamp = ros::Time::now();
      velocity_mode_ = true;
      return -1.0*default_low_speed_;
    }
    else
    {
      velocity_mode_ = false;
      return default_effort_;
    }
  }

  //close with low force
  else if(closed_loop_grasp_state == close0_closing)
  {
    int starting_force_sum0 = 0;
    int current_force_sum0 = 0;
    int starting_force_sum1 = 0;
    int current_force_sum1 = 0;
    for(int i = 0; i < num_pressure_pads_front_; i++)
    {
      current_force_sum0 += pressure_state_.data0[i+num_pressure_pads_side_];
      current_force_sum1 += pressure_state_.data1[i+num_pressure_pads_side_];
      starting_force_sum0 += fingertip_sensor_start0_[i];
      starting_force_sum1 += fingertip_sensor_start1_[i];
    }
    //break if there is contact on edges of fingertips before first contact (cl only)
    if(closed_loop)
    {
      for(int i = 0; i < num_pressure_pads_side_; i++)
      {
        if(pressure_state_.data0[i] - fingertip_sensor_sides_start0_[i] > contact_threshold_individual_ || pressure_state_.data1[i] - fingertip_sensor_sides_start1_[i] > contact_threshold_individual_)
        {
          ROS_WARN("grasp failed due to impact on the fingertip sides");
          closed_loop_grasp_state = failed;
          //report that there was a failure
          if(service_callback)
          {
            service_response_.result = 2;
            service_response_.distance = joint_->position_;
            service_response_.effort = default_effort_;
            for(int j = 0; j < num_pressure_pads_side_; j++)
            {
              service_response_.fingertip_profile0[j] = pressure_state_.data0[j] - fingertip_sensor_sides_start0_[j];
              service_response_.fingertip_profile1[j] = pressure_state_.data1[j] - fingertip_sensor_sides_start1_[j];
            }
            service_response_.stiffness = -1.0;
            service_response_.force_peak0 = -1.0;
            service_response_.force_steady0 = -1.0;
            service_response_.force_peak1 = -1.0;
            service_response_.force_steady1 = -1.0;
            service_response_.distance_compressed_first = -1.0;
            service_response_.distance_compressed_second = -1.0;
          }
          velocity_mode_ = false;
          return default_effort_;
        }
      }
    }
    //break if we've closed too much for the expected object (cl only)
    if(closed_loop && service_callback && joint_->position_ < service_request_.distance - service_request_.distance_tolerance)
    {
      ROS_WARN("grasp failed due to closing too far without impacting object");
      closed_loop_grasp_state = failed;
      //report that there was a failure
      service_response_.result = 3;
      service_response_.distance = joint_->position_;
      service_response_.effort = default_effort_;
      service_response_.stiffness = -1.0;
      service_response_.force_peak0 = -1.0;
      service_response_.force_steady0 = -1.0;
      service_response_.force_peak1 = -1.0;
      service_response_.force_steady1 = -1.0;
      service_response_.distance_compressed_first = -1.0;
      service_response_.distance_compressed_second = -1.0;
      velocity_mode_ = false;
      return default_effort_;
    }
    //check for any timeouts
    if(grasp_open_close_timestamp + timeout_duration_ < ros::Time::now())
    {
      ROS_WARN("grasp failed due to a timeout");
      closed_loop_grasp_state = failed;
      //report that there was a failure
      if(service_callback)
      {
        service_response_.result = 1;
        service_response_.distance = joint_->position_;
        service_response_.effort = default_effort_;
        service_response_.stiffness = -1.0;
        service_response_.force_peak0 = -1.0;
        service_response_.force_steady0 = -1.0;
        service_response_.force_peak1 = -1.0;
        service_response_.force_steady1 = -1.0;
        service_response_.distance_compressed_first = -1.0;
        service_response_.distance_compressed_second = -1.0;
      }
      velocity_mode_ = false;
      return default_effort_;
    }
    //record first contact info
    else if(current_force_sum0 > starting_force_sum0 + contact_threshold_ && current_force_sum1 > starting_force_sum1 + contact_threshold_)
    {
      position_first_contact = joint_->position_;
      closed_loop_grasp_state = close0_contact;
      grasp_open_close_timestamp = ros::Time::now();
      if(closed_loop && service_callback && joint_->position_ > service_request_.distance + service_request_.distance_tolerance)
      {
        ROS_WARN("grasp failed due to closing too little before impacting object");
        closed_loop_grasp_state = failed;
        //report that there was a failure
        service_response_.result = 4;
        service_response_.distance = joint_->position_;
        service_response_.effort = default_effort_;
        service_response_.stiffness = -1.0;
        service_response_.force_peak0 = -1.0;
        service_response_.force_steady0 = -1.0;
        service_response_.force_peak1 = -1.0;
        service_response_.force_steady1 = -1.0;
        service_response_.distance_compressed_first = -1.0;
        service_response_.distance_compressed_second = -1.0;
        velocity_mode_ = false;
        return default_effort_;
      }
    }
    velocity_mode_ = true;
    return -1.0*default_low_speed_;
  }

  //continue closing
  else if(closed_loop_grasp_state == close0_contact)
  {
    double starting_force_sum = 0;
    double current_force_sum = 0;
    for(int i = 0; i < num_pressure_pads_front_; i++)
    {
      current_force_sum += pressure_state_.data0[i+num_pressure_pads_side_];
      current_force_sum += pressure_state_.data1[i+num_pressure_pads_side_];
      starting_force_sum += fingertip_sensor_start0_[i];
      starting_force_sum += fingertip_sensor_start1_[i];
    }
    //record peak contact info
    if(peak_force_first_grasp < current_force_sum - starting_force_sum)
    {
      peak_force_first_grasp = current_force_sum - starting_force_sum;
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {
        fingertip_sensor_first_peak0_[i] = pressure_state_.data0[i+num_pressure_pads_side_];
        fingertip_sensor_first_peak1_[i] = pressure_state_.data1[i+num_pressure_pads_side_];
      }
    }
    //record final contact info
    if(grasp_open_close_timestamp + timeout_duration_steady_ < ros::Time::now())
    {
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {
        fingertip_sensor_first_steady0_[i] = pressure_state_.data0[i+num_pressure_pads_side_];
        fingertip_sensor_first_steady1_[i] = pressure_state_.data1[i+num_pressure_pads_side_];
      }
      position_first_compression = joint_->position_;

      //if the grasp was of a known object, and want to compare for accuracy or unknown object, and want to store info
      if(service_callback)
      {
        //TODO::do we analyze the fingertip sensors here?
        service_response_.result = 0;
        service_response_.distance = position_first_contact;
        service_response_.effort = service_request_.effort;
        int force_peak0 = 0;
        int force_steady0 = 0;
        int force_peak1 = 0;
        int force_steady1 = 0;
        for(int i = 0; i < num_pressure_pads_front_; i++)
        {
          service_response_.fingertip_profile0[i+num_pressure_pads_side_] = fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
          service_response_.fingertip_profile1[i+num_pressure_pads_side_] = fingertip_sensor_first_steady1_[i] - fingertip_sensor_start1_[i];
          force_peak0 += fingertip_sensor_first_peak0_[i] - fingertip_sensor_start0_[i];
          force_steady0 += fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
          force_peak1 += fingertip_sensor_first_peak1_[i] - fingertip_sensor_start1_[i];
          force_steady1 += fingertip_sensor_first_steady1_[i] - fingertip_sensor_start1_[i];
        }
        for(int i = 0; i < num_pressure_pads_side_; i++)
        {
          service_response_.fingertip_profile0[i] = pressure_state_.data0[i]-fingertip_sensor_sides_start0_[i];
          service_response_.fingertip_profile1[i] = pressure_state_.data1[i]-fingertip_sensor_sides_start1_[i];
        }
        service_response_.stiffness = position_first_contact/joint_->position_;
        service_response_.force_peak0 = force_peak0;
        service_response_.force_steady0 = force_steady0;
        service_response_.force_peak1 = force_peak1;
        service_response_.force_steady1 = force_steady1;
        service_response_.distance_compressed_first = joint_->position_;
        service_response_.distance_compressed_second = -1.0;
        if(!closed_loop || service_response_.stiffness < service_request_.stiffness +service_request_.stiffness_threshold && service_response_.stiffness > service_request_.stiffness - service_request_.stiffness_threshold)
        {
          closed_loop_grasp_state = complete;
          service_success_ = true;
          velocity_mode_ = false;
          ROS_INFO("grasp succeeded");
          return service_request_.effort;
        }
        else
        {
          ROS_WARN("grasp failed due to the stiffness of the object being outside of tolerance");
          closed_loop_grasp_state = failed;
          //report that there was a failure
          service_response_.result = 5;
          velocity_mode_ = false;
          return default_effort_;
        }
      }
      else
      {
        //closed_loop_grasp_state = open1;
        //grasp_open_close_timestamp = ros::Time::now();
        closed_loop_grasp_state = complete;
      }
    }
    velocity_mode_ = false;
    return -1.0*low_force_;
  }
/*
  //open
  else if(closed_loop_grasp_state == open1)
  {
    //if I'm done opening
    if(ros::Time::now().toSec() > grasp_open_close_timestamp.toSec() + break_stiction_period_)
    {
      closed_loop_grasp_state = close1_closing;
      grasp_open_close_timestamp = ros::Time::now();
      velocity_mode_ = true;
      return -1.0*default_high_speed_;
    }
    //else open
    else
    {
      velocity_mode_ = false;
      return default_effort_;
    }
  }

  //close with higher force
  //TODO::is this necessary?
  else if(closed_loop_grasp_state == close1_closing)
  {
    double starting_force_sum = 0;
    double current_force_sum = 0;
    for(int i = 0; i < num_pressure_pads_front_; i++)
    {
      current_force_sum += pressure_state_.data0[i+num_pressure_pads_side_];
      current_force_sum += pressure_state_.data1[i+num_pressure_pads_side_];
      starting_force_sum += fingertip_sensor_start0_[i];
      starting_force_sum += fingertip_sensor_start1_[i];
    }
    //check for any timeouts
    if(grasp_open_close_timestamp + timeout_duration_ < ros::Time::now())
    {
      ROS_WARN("grasp failed due to a timeout");
      closed_loop_grasp_state = failed;
      //report faliure!
      if(closed_loop)
      {
        service_response_.distance = position_first_contact;
        service_response_.effort = default_effort_;
        int force_peak0 = 0;
        int force_steady0 = 0;
        int force_peak1 = 0;
        int force_steady1 = 0;
        for(int i = 0; i < num_pressure_pads_front_; i++)
        {
          service_response_.fingertip_profile0[i+num_pressure_pads_side_] = fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
          service_response_.fingertip_profile1[i+num_pressure_pads_side_] = fingertip_sensor_first_steady1_[i] - fingertip_sensor_start1_[i];
          force_peak0 += fingertip_sensor_first_peak0_[i] - fingertip_sensor_start0_[i];
          force_steady0 += fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
          force_peak1 += fingertip_sensor_first_peak1_[i] - fingertip_sensor_start1_[i];
          force_steady1 += fingertip_sensor_first_steady1_[i] - fingertip_sensor_start1_[i];
        }
        for(int i = 0; i < num_pressure_pads_side_; i++)
        {
          service_response_.fingertip_profile0[i] = pressure_state_.data0[i]-fingertip_sensor_sides_start0_[i];
          service_response_.fingertip_profile1[i] = pressure_state_.data1[i]-fingertip_sensor_sides_start1_[i];
        }
        service_response_.stiffness = 0.0;
        service_response_.force_peak0 = force_peak0;
        service_response_.force_steady0 = force_steady0;
        service_response_.force_peak1 = force_peak1;
        service_response_.force_steady1 = force_steady1;
        service_response_.distance_compressed_first = position_first_compression;
        service_response_.distance_compressed_second = -1.0;
      }
      velocity_mode_ = false;
      return default_effort_;
    }
    //record first contact info
    else if(current_force_sum > starting_force_sum + contact_threshold_)
    {
      position_second_contact = joint_->position_;
      closed_loop_grasp_state = close1_contact;
      grasp_open_close_timestamp = ros::Time::now();
    }
    velocity_mode_ = true;
    return -1.0*default_high_speed_;
  }

  //continue closing
  else if(closed_loop_grasp_state == close1_contact)
  {
    double starting_force_sum = 0;
    double current_force_sum = 0;
    for(int i = 0; i < num_pressure_pads_front_; i++)
    {
      current_force_sum += pressure_state_.data0[i+num_pressure_pads_side_];
      current_force_sum += pressure_state_.data1[i+num_pressure_pads_side_];
      starting_force_sum += fingertip_sensor_start0_[i];
      starting_force_sum += fingertip_sensor_start1_[i];
    }
    //record peak contact info
    if(peak_force_second_grasp < current_force_sum - starting_force_sum)
    {
      peak_force_second_grasp = current_force_sum - starting_force_sum;
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {
        fingertip_sensor_second_peak0_[i] = pressure_state_.data0[i+num_pressure_pads_side_];
        fingertip_sensor_second_peak1_[i] = pressure_state_.data1[i+num_pressure_pads_side_];
      }
    }
    //record final contact info
    if(grasp_open_close_timestamp + timeout_duration_steady_ < ros::Time::now())
    {
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {
        fingertip_sensor_second_steady0_[i] = pressure_state_.data0[i+num_pressure_pads_side_];
        fingertip_sensor_second_steady1_[i] = pressure_state_.data1[i+num_pressure_pads_side_];
      }
      position_second_compression = joint_->position_;
      closed_loop_grasp_state = complete;
      grasp_open_close_timestamp = ros::Time::now();
      //compute k
      spring_const = (high_force_-low_force_)/(position_second_compression - position_first_compression);

      //publish finding
      //if the grasp was of an unknown object, and want to identify it
      int peak_sum = 0;
      int steady_sum = 0;
      std::ofstream myfile;
      myfile.open("grasp_data.txt");
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {
        peak_sum += fingertip_sensor_first_peak0_[i] - fingertip_sensor_start0_[i];
        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_first_peak0_[i] - fingertip_sensor_start0_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_first_peak1_[i] - fingertip_sensor_start1_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {
        steady_sum += fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_first_steady1_[i] - fingertip_sensor_start0_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_second_peak0_[i] - fingertip_sensor_start0_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_second_peak1_[i] - fingertip_sensor_start1_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_second_steady0_[i] - fingertip_sensor_start0_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_second_steady1_[i] - fingertip_sensor_start0_[i] << " ";
      }
      myfile << "\n";
      myfile << "\n";
      myfile << spring_const << "\n" << position_first_contact << "\n" <<position_first_compression << "\n" << position_second_contact << "\n" << position_second_compression;
      myfile << "\n";
      myfile << "\n";
      myfile << peak_sum << "\n" << steady_sum;
      myfile.close();

      if(closed_loop)
      {
        service_response_.distance = position_first_contact;
        service_response_.effort = -1.0*high_force_;
        int force_peak0 = 0;
        int force_steady0 = 0;
        int force_peak1 = 0;
        int force_steady1 = 0;
        //TODO::some of this can be done above...
        for(int i = 0; i < num_pressure_pads_front_; i++)
        {
          service_response_.fingertip_profile0[i+num_pressure_pads_side_] = fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
          service_response_.fingertip_profile1[i+num_pressure_pads_side_] = fingertip_sensor_first_steady1_[i] - fingertip_sensor_start1_[i];
          force_peak0 += fingertip_sensor_first_peak0_[i] - fingertip_sensor_start0_[i];
          force_steady0 += fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
          force_peak1 += fingertip_sensor_first_peak1_[i] - fingertip_sensor_start1_[i];
          force_steady1 += fingertip_sensor_first_steady1_[i] - fingertip_sensor_start1_[i];
        }
        for(int i = 0; i < num_pressure_pads_side_; i++)
        {
          service_response_.fingertip_profile0[i] = pressure_state_.data0[i]-fingertip_sensor_sides_start0_[i];
          service_response_.fingertip_profile1[i] = pressure_state_.data1[i]-fingertip_sensor_sides_start1_[i];
        }
        service_response_.stiffness = spring_const;
        service_response_.force_peak0 = force_peak0;
        service_response_.force_steady0 = force_steady0;
        service_response_.force_peak1 = force_peak1;
        service_response_.force_steady1 = force_steady1;
        service_response_.distance_compressed_first = position_first_compression;
        service_response_.distance_compressed_second = position_second_compression;
        service_success_ = true;
      }
      ROS_INFO("Grasp complete!");
    }
    velocity_mode_ = false;
    return -1.0*high_force_;
  }
*/
  //finishing the grasp
  else if(closed_loop_grasp_state == complete)
  {
    velocity_mode_ = false;
    service_flag_ = true;
    if(service_callback && closed_loop)
    {
      return service_request_.effort;
    }
    //TODO::determine a good output force
    return -1.0*high_force_;  //because we're done
  }

  else if(closed_loop_grasp_state == failed)
  {
    velocity_mode_ = false;
    service_flag_ = true;
    return default_effort_;
  }
  velocity_mode_ = false;
  return 0.0;

}

double Pr2GripperController::parseMessage(pr2_mechanism_controllers::GripperControllerCmd desired_msg)
{
  if(desired_msg.cmd.compare("grasp_cl") == 0)
  {
    return grasp(service_callback_, true);
  }
  else if(desired_msg.cmd.compare("grasp") == 0)
  {
    return grasp(service_callback_, false);
  }
  else if(desired_msg.cmd.compare("move") == 0)
  {
    velocity_mode_ = false;
    return desired_msg.val;
  }
  else if(desired_msg.cmd.compare("moveVel") == 0)
  {
    velocity_mode_ = true;
    return desired_msg.val;
  }
  else if(desired_msg.cmd.compare("moveTo") == 0)
  {
    velocity_mode_ = false;
    double direction = 1.0;
    if(joint_->position_ > desired_msg.val)
      direction = -1.0;
    if(proportional_offset_ > fabs(joint_->position_-desired_msg.val))
      return default_effort_*direction*fabs(joint_->position_-desired_msg.val)/proportional_offset_;
    return default_effort_*direction;
  }
  //This is calculated elsewhere
  else if(desired_msg.cmd.compare("step") == 0)
  {
    velocity_mode_ = false;
    return last_commanded_command;
  }
  else if(desired_msg.cmd.compare("ramp") == 0)
  {
    velocity_mode_ = false;
    return rampMove(desired_msg.start, desired_msg.end, desired_msg.time, desired_msg.end);
  }
  else if(desired_msg.cmd.compare("open") == 0)
  {
    velocity_mode_ = false;
    return default_effort_;
  }
  else if(desired_msg.cmd.compare("close") == 0)
  {
    velocity_mode_ = false;
    return -1.0*default_effort_;
  }
  else
  {
    velocity_mode_ = false;
    return 0.0;
  }
}

double Pr2GripperController::effortLimit(double desiredEffort)
{
  if(desiredEffort > joint_->joint_->effort_limit_)
    return joint_->joint_->effort_limit_;
  if(desiredEffort < joint_->joint_->effort_limit_*-1.0)
    return joint_->joint_->effort_limit_*-1.0;
  return desiredEffort;
}

void Pr2GripperController::command_callback(const pr2_mechanism_controllers::GripperControllerCmdConstPtr& grasp_cmd)
{
  pthread_mutex_lock(&pr2_gripper_controller_lock_);
  grasp_cmd_.cmd = grasp_cmd->cmd;
  grasp_cmd_.start = grasp_cmd->start;
  grasp_cmd_.end = grasp_cmd->end;
  grasp_cmd_.time = grasp_cmd->time;
  grasp_cmd_.val = grasp_cmd->val;
  cmd_received_timestamp_ = robot_state_->hw_->current_time_;
  new_cmd_available_ = true;
  pthread_mutex_unlock(&pr2_gripper_controller_lock_);
}

void Pr2GripperController::pressure_state_callback(const ethercat_hardware::PressureStateConstPtr& pressure_state)
{
  if((int)pressure_state->data0.size() != num_pressure_pads_front_+num_pressure_pads_side_)
    ROS_WARN("fingertip pressure data is different size than expected");
  pthread_mutex_lock(&pr2_gripper_controller_lock_);
  for(int i = 0; i < num_pressure_pads_front_+num_pressure_pads_side_; i++)
  {
    pressure_state_.data0[i] = pressure_state->data0[i];
    pressure_state_.data1[i] = pressure_state->data1[i];
  }
  pthread_mutex_unlock(&pr2_gripper_controller_lock_);
}

bool Pr2GripperController::grasp_cl_srv(manipulation_srvs::GraspClosedLoop::Request &req, manipulation_srvs::GraspClosedLoop::Response &res)
{
  pthread_mutex_lock(&pr2_gripper_controller_lock_);
  grasp_cmd_.cmd = req.cmd;
  service_request_ = req;
  new_cmd_available_ = true;
  closed_loop_grasp_state = unstarted;
  cmd_received_timestamp_ = robot_state_->hw_->current_time_;
  service_response_.fingertip_profile0.resize(num_pressure_pads_front_+num_pressure_pads_side_);
  service_response_.fingertip_profile1.resize(num_pressure_pads_front_+num_pressure_pads_side_);
  pthread_mutex_unlock(&pr2_gripper_controller_lock_);
  service_flag_ = false;
  service_success_ = false;
  //TODO::check if this is thread safe
  service_callback_ = true;
  while(!service_flag_)
  {
    usleep(50000);
  }
  res = service_response_;
  //TODO::check if this is thread safe
  service_callback_ = false;
  return service_success_;
}

void Pr2GripperController::callbackThread()
{
  //ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  ros::NodeHandle n;
  while (n.ok())
  {
    service_queue_.callAvailable(ros::WallDuration(0.01));
  }
}
