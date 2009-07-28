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
//TODO::switch to node handles
bool Pr2GripperController::init(mechanism::RobotState *robot_state, const ros::NodeHandle &node)
{
  double timeout_duration_double;
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
  node_.param<double>("default_speed",default_speed_,joint_->joint_->effort_limit_);
  node_.param<double>("timeout", timeout_, 0.0);
  node_.param<double>("break_stiction_amplitude", break_stiction_amplitude_, 0.0);
  node_.param<double>("break_stiction_period", break_stiction_period_, 0.0);
  node_.param<double>("break_stiction_velocity", break_stiction_velocity_, 0.005);
  node_.param<double>("proportional_offset", proportional_offset_, 0.01);
  node_.param<double>("stopped_threshold", stopped_threshold_, 0.001);
  node_.param<double>("timeout_duration", timeout_duration_double, 3.0);
  node_.param<double>("low_force", low_force_, 1.0);
  node_.param<double>("high_force", high_force_, 2.0);
  node_.param<int>("contact_threshold", contact_threshold_, 1000);
  node_.param<int>("contact_threshold_individual", contact_threshold_individual_, 100);
  node_.param<std::string>("break_stiction_type", break_stiction_type_, "none");
  node_.param<std::string>("fingertip_sensor_topic", fingertip_sensor_topic_, "pressure/" + name_ + "_motor");
  grasp_service_ = node_.advertiseService("grasp_closed_loop", &Pr2GripperController::grasp_cl_srv, this);
  //TODO::find out why these can only be done with node_ and not node
  cmd_sub_ = node_.subscribe<pr2_mechanism_controllers::GripperControllerCmd>(name_+"_cmd", 1, &Pr2GripperController::command_callback, this);
  pressure_sub_ = node_.subscribe<ethercat_hardware::PressureState>(fingertip_sensor_topic_, 1, &Pr2GripperController::pressure_state_callback, this);
  timeout_duration = ros::Duration().fromSec(timeout_duration_double);
  if (state_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete state_publisher_;
  state_publisher_ = new realtime_tools::RealtimePublisher <pr2_msgs::GripperControllerState> (name_ + "/state", 1);
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
        joint_controller_.command_ = effortLimit(stepMove(grasp_cmd_desired_.val));
        last_commanded_command = joint_controller_.command_;
      }
    }
  }

  //check for timeout
  if((current_time - cmd_received_timestamp_) <= timeout_ || timeout_ == 0.0) //continue with what you were doing
  {
    double direction = 1.0;
    last_commanded_command = effortLimit(parseMessage(grasp_cmd_desired_)); //set value
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

double Pr2GripperController::grasp(bool closed_loop)
{
  //starting grasp
  if(closed_loop_grasp_state == unstarted)
  {
    //reset variables
    for(int i = 0; i < 15; i++)
    {
      fingertip_sensor_start0[i]=0;
      fingertip_sensor_start1[i]=0;
      fingertip_sensor_first_peak0[i]=0;
      fingertip_sensor_first_peak1[i]=0;
      fingertip_sensor_first_steady0[i]=0;
      fingertip_sensor_first_steady1[i]=0;
      fingertip_sensor_second_peak0[i]=0;
      fingertip_sensor_second_peak1[i]=0;
      fingertip_sensor_second_steady0[i]=0;
      fingertip_sensor_second_steady1[i]=0;
    }
    //remember values from edges of fingertips
    for(int i = 0; i < 7; i++)
    {
      fingertip_sensor_sides_start0[i] = 0;
      fingertip_sensor_sides_start1[i] = 0;
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
    return default_speed_;
  }

  //open
  else if(closed_loop_grasp_state == open0)
  {
    //check for any timeouts
    if(grasp_open_close_timestamp + timeout_duration < ros::Time::now())
    {
      ROS_WARN("grasp failed due to a timeout");
      closed_loop_grasp_state = failed;
      //report that there was a failure
      service_response_->distance = -1.0;
      service_response_->effort = default_speed_;
      service_response_->stiffness = -1.0;
      service_response_->force_peak0 = -1.0;
      service_response_->force_steady0 = -1.0;
      service_response_->force_peak1 = -1.0;
      service_response_->force_steady1 = -1.0;
      service_response_->distance_compressed_first = -1.0;
      service_response_->distance_compressed_second = -1.0;
      return default_speed_;
    }
    //if the gripper is done opening
    else if(joint_->velocity_ < stopped_threshold_ && grasp_open_close_timestamp.toSec() + .1 < ros::Time::now().toSec())
    {
      //read the gripper pads to zero them
      for(int i = 0; i < 15; i++) //the front pads are 7-21
      {
        fingertip_sensor_start0[i] = pressure_state_.data0[i+7];
        fingertip_sensor_start1[i] = pressure_state_.data1[i+7];
      }
      for(int i = 0; i < 7; i++)
      {
        fingertip_sensor_sides_start0[i] = pressure_state_.data0[i];
        fingertip_sensor_sides_start1[i] = pressure_state_.data1[i];
      }
      //chage state
      closed_loop_grasp_state = close0_closing;
      grasp_open_close_timestamp = ros::Time::now();
      return -1.0*low_force_;
    }
    else
    {
      return default_speed_;
    }
  }

  //close with low force
  else if(closed_loop_grasp_state == close0_closing)
  {
    double starting_force_sum = 0;
    double current_force_sum = 0;
    for(int i = 0; i < 15; i++)
    {
      current_force_sum += pressure_state_.data0[i+7];
      current_force_sum += pressure_state_.data1[i+7];
      starting_force_sum += fingertip_sensor_start0[i];
      starting_force_sum += fingertip_sensor_start1[i];
    }
    //break if there is contact on edges of fingertips before first contact (cl only)
    if(closed_loop)
    {
      for(int i = 0; i < 7; i++)
      {
        if(pressure_state_.data0[i] - fingertip_sensor_sides_start0[i] > contact_threshold_individual_ || pressure_state_.data1[i] - fingertip_sensor_sides_start1[i] > contact_threshold_individual_)
        {
          ROS_WARN("grasp failed due to impact on the fingertip sides");
          closed_loop_grasp_state = failed;
          //report that there was a failure
          service_response_->distance = -3.0;
          service_response_->effort = default_speed_;
          for(int j = 0; j < 7; j++)
          {
            service_response_->fingertip_profile0[j] = pressure_state_.data0[j] - fingertip_sensor_sides_start0[j];
            service_response_->fingertip_profile1[j] = pressure_state_.data1[j] - fingertip_sensor_sides_start1[j];
          }
          service_response_->stiffness = -1.0;
          service_response_->force_peak0 = -1.0;
          service_response_->force_steady0 = -1.0;
          service_response_->force_peak1 = -1.0;
          service_response_->force_steady1 = -1.0;
          service_response_->distance_compressed_first = -1.0;
          service_response_->distance_compressed_second = -1.0;
          return default_speed_;
        }
      }
    }
    //break if we've closed too much for the expected object (cl only)
    if(closed_loop && joint_->position_ < service_request_->distance - service_request_->distance_tolerance)
    {
      ROS_WARN("grasp failed due to closing too far without impacting object");
      closed_loop_grasp_state = failed;
      //report that there was a failure
      service_response_->distance = -2.0;
      service_response_->effort = default_speed_;
      service_response_->stiffness = -1.0;
      service_response_->force_peak0 = -1.0;
      service_response_->force_steady0 = -1.0;
      service_response_->force_peak1 = -1.0;
      service_response_->force_steady1 = -1.0;
      service_response_->distance_compressed_first = -1.0;
      service_response_->distance_compressed_second = -1.0;
      return default_speed_;
    }
    //check for any timeouts
    if(grasp_open_close_timestamp + timeout_duration < ros::Time::now())
    {
      ROS_WARN("grasp failed due to a timeout");
      closed_loop_grasp_state = failed;
      //report that there was a failure
      service_response_->distance = -1.0;
      service_response_->effort = default_speed_;
      service_response_->stiffness = -1.0;
      service_response_->force_peak0 = -1.0;
      service_response_->force_steady0 = -1.0;
      service_response_->force_peak1 = -1.0;
      service_response_->force_steady1 = -1.0;
      service_response_->distance_compressed_first = -1.0;
      service_response_->distance_compressed_second = -1.0;
      return default_speed_;
    }
    //record first contact info
    else if(current_force_sum > starting_force_sum + contact_threshold_)
    {
      position_first_contact = joint_->position_;
      closed_loop_grasp_state = close0_contact;
      grasp_open_close_timestamp = ros::Time::now();
    }
    return -1.0*low_force_;
  }

  //continue closing
  else if(closed_loop_grasp_state == close0_contact)
  {
    double starting_force_sum = 0;
    double current_force_sum = 0;
    for(int i = 0; i < 15; i++)
    {
      current_force_sum += pressure_state_.data0[i+7];
      current_force_sum += pressure_state_.data1[i+7];
      starting_force_sum += fingertip_sensor_start0[i];
      starting_force_sum += fingertip_sensor_start1[i];
    }
    //record peak contact info
    if(peak_force_first_grasp < current_force_sum - starting_force_sum)
    {
      peak_force_first_grasp = current_force_sum - starting_force_sum;
      for(int i = 0; i < 15; i++)
      {
        fingertip_sensor_first_peak0[i] = pressure_state_.data0[i+7];
        fingertip_sensor_first_peak1[i] = pressure_state_.data1[i+7];
      }
    }
    //record final contact info
    if(grasp_open_close_timestamp + timeout_duration < ros::Time::now())
    {
      for(int i = 0; i < 15; i++)
      {
        fingertip_sensor_first_steady0[i] = pressure_state_.data0[i+7];
        fingertip_sensor_first_steady1[i] = pressure_state_.data1[i+7];
      }
      position_first_compression = joint_->position_;
      //if the grasp was of a known object, and want to compare for accuracy
      if(closed_loop)
      {
        //TODO::do we analyze the fingertip sensors here?
        service_response_->distance = position_first_contact;
        service_response_->effort = service_request_->effort;
        int force_peak0 = 0;
        int force_steady0 = 0;
        int force_peak1 = 0;
        int force_steady1 = 0;
        for(int i = 0; i < 15; i++)
        {
          service_response_->fingertip_profile0[i+7] = fingertip_sensor_first_steady0[i] - fingertip_sensor_start0[i];
          service_response_->fingertip_profile1[i+7] = fingertip_sensor_first_steady1[i] - fingertip_sensor_start1[i];
          force_peak0 += fingertip_sensor_first_peak0[i] - fingertip_sensor_start0[i];
          force_steady0 += fingertip_sensor_first_steady0[i] - fingertip_sensor_start0[i];
          force_peak1 += fingertip_sensor_first_peak1[i] - fingertip_sensor_start1[i];
          force_steady1 += fingertip_sensor_first_steady1[i] - fingertip_sensor_start1[i];
        }
        for(int i = 0; i < 7; i++)
        {
          service_response_->fingertip_profile0[i] = pressure_state_.data0[i]-fingertip_sensor_sides_start0[i];
          service_response_->fingertip_profile1[i] = pressure_state_.data1[i]-fingertip_sensor_sides_start1[i];
        }
        service_response_->stiffness = 0.0;
        service_response_->force_peak0 = force_peak0;
        service_response_->force_steady0 = force_steady0;
        service_response_->force_peak1 = force_peak1;
        service_response_->force_steady1 = force_steady1;
        service_response_->distance_compressed_first = joint_->position_;
        service_response_->distance_compressed_second = -1.0;
        closed_loop_grasp_state = complete;
        service_success_ = true;
        return service_request_->effort;
      }
      else
      {
        closed_loop_grasp_state = open1;
        grasp_open_close_timestamp = ros::Time::now();
      }
    }
    return -1.0*low_force_;
  }

  //open
  else if(closed_loop_grasp_state == open1)
  {
    //if I'm done opening
    if(ros::Time::now().toSec() > grasp_open_close_timestamp.toSec() + break_stiction_period_)
    {
      closed_loop_grasp_state = close1_closing;
      grasp_open_close_timestamp = ros::Time::now();
      return -1.0*high_force_;
    }
    //else open
    else
    {
      return default_speed_;
    }
  }

  //close with higher force
  //TODO::is this necessary?
  else if(closed_loop_grasp_state == close1_closing)
  {
    double starting_force_sum = 0;
    double current_force_sum = 0;
    for(int i = 0; i < 15; i++)
    {
      current_force_sum += pressure_state_.data0[i+7];
      current_force_sum += pressure_state_.data1[i+7];
      starting_force_sum += fingertip_sensor_start0[i];
      starting_force_sum += fingertip_sensor_start1[i];
    }
    //check for any timeouts
    if(grasp_open_close_timestamp + timeout_duration < ros::Time::now())
    {
      ROS_WARN("grasp failed due to a timeout");
      closed_loop_grasp_state = failed;
      //report faliure!
      service_response_->distance = position_first_contact;
      service_response_->effort = default_speed_;
      int force_peak0 = 0;
      int force_steady0 = 0;
      int force_peak1 = 0;
      int force_steady1 = 0;
      for(int i = 0; i < 15; i++)
      {
        service_response_->fingertip_profile0[i+7] = fingertip_sensor_first_steady0[i] - fingertip_sensor_start0[i];
        service_response_->fingertip_profile1[i+7] = fingertip_sensor_first_steady1[i] - fingertip_sensor_start1[i];
        force_peak0 += fingertip_sensor_first_peak0[i] - fingertip_sensor_start0[i];
        force_steady0 += fingertip_sensor_first_steady0[i] - fingertip_sensor_start0[i];
        force_peak1 += fingertip_sensor_first_peak1[i] - fingertip_sensor_start1[i];
        force_steady1 += fingertip_sensor_first_steady1[i] - fingertip_sensor_start1[i];
      }
      for(int i = 0; i < 7; i++)
      {
        service_response_->fingertip_profile0[i] = pressure_state_.data0[i]-fingertip_sensor_sides_start0[i];
        service_response_->fingertip_profile1[i] = pressure_state_.data1[i]-fingertip_sensor_sides_start1[i];
      }
      service_response_->stiffness = 0.0;
      service_response_->force_peak0 = force_peak0;
      service_response_->force_steady0 = force_steady0;
      service_response_->force_peak1 = force_peak1;
      service_response_->force_steady1 = force_steady1;
      service_response_->distance_compressed_first = position_first_compression;
      service_response_->distance_compressed_second = -1.0;
      return default_speed_;
    }
    //record first contact info
    else if(current_force_sum > starting_force_sum + contact_threshold_)
    {
      position_second_contact = joint_->position_;
      closed_loop_grasp_state = close1_contact;
      grasp_open_close_timestamp = ros::Time::now();
    }
    return -1.0*high_force_;
  }

  //continue closing
  else if(closed_loop_grasp_state == close1_contact)
  {
    double starting_force_sum = 0;
    double current_force_sum = 0;
    for(int i = 0; i < 15; i++)
    {
      current_force_sum += pressure_state_.data0[i+7];
      current_force_sum += pressure_state_.data1[i+7];
      starting_force_sum += fingertip_sensor_start0[i];
      starting_force_sum += fingertip_sensor_start1[i];
    }
    //record peak contact info
    if(peak_force_second_grasp < current_force_sum - starting_force_sum)
    {
      peak_force_second_grasp = current_force_sum - starting_force_sum;
      for(int i = 0; i < 15; i++)
      {
        fingertip_sensor_second_peak0[i] = pressure_state_.data0[i+7];
        fingertip_sensor_second_peak1[i] = pressure_state_.data1[i+7];
      }
    }
    //record final contact info
    if(grasp_open_close_timestamp + timeout_duration < ros::Time::now())
    {
      for(int i = 0; i < 15; i++)
      {
        fingertip_sensor_second_steady0[i] = pressure_state_.data0[i+7];
        fingertip_sensor_second_steady1[i] = pressure_state_.data1[i+7];
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
      for(int i = 0; i < 15; i++)
      {
        peak_sum += fingertip_sensor_first_peak0[i] - fingertip_sensor_start0[i];
        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_first_peak0[i] - fingertip_sensor_start0[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < 15; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_first_peak1[i] - fingertip_sensor_start1[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < 15; i++)
      {
        steady_sum += fingertip_sensor_first_steady0[i] - fingertip_sensor_start0[i];
        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_first_steady0[i] - fingertip_sensor_start0[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < 15; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_first_steady1[i] - fingertip_sensor_start0[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < 15; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_second_peak0[i] - fingertip_sensor_start0[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < 15; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_second_peak1[i] - fingertip_sensor_start1[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < 15; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_second_steady0[i] - fingertip_sensor_start0[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < 15; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_second_steady1[i] - fingertip_sensor_start0[i] << " ";
      }
      myfile << "\n";
      myfile << "\n";
      myfile << spring_const << "\n" << position_first_contact << "\n" <<position_first_compression << "\n" << position_second_contact << "\n" << position_second_compression;
      myfile << "\n";
      myfile << "\n";
      myfile << peak_sum << "\n" << steady_sum;
      myfile.close();

      service_response_->distance = position_first_contact;
      service_response_->effort = -1.0*high_force_;
      int force_peak0 = 0;
      int force_steady0 = 0;
      int force_peak1 = 0;
      int force_steady1 = 0;
      //TODO::some of this can be done above...
      for(int i = 0; i < 15; i++)
      {
        service_response_->fingertip_profile0[i+7] = fingertip_sensor_first_steady0[i] - fingertip_sensor_start0[i];
        service_response_->fingertip_profile1[i+7] = fingertip_sensor_first_steady1[i] - fingertip_sensor_start1[i];
        force_peak0 += fingertip_sensor_first_peak0[i] - fingertip_sensor_start0[i];
        force_steady0 += fingertip_sensor_first_steady0[i] - fingertip_sensor_start0[i];
        force_peak1 += fingertip_sensor_first_peak1[i] - fingertip_sensor_start1[i];
        force_steady1 += fingertip_sensor_first_steady1[i] - fingertip_sensor_start1[i];
      }
      for(int i = 0; i < 7; i++)
      {
        service_response_->fingertip_profile0[i] = pressure_state_.data0[i]-fingertip_sensor_sides_start0[i];
        service_response_->fingertip_profile1[i] = pressure_state_.data1[i]-fingertip_sensor_sides_start1[i];
      }
      service_response_->stiffness = spring_const;
      service_response_->force_peak0 = force_peak0;
      service_response_->force_steady0 = force_steady0;
      service_response_->force_peak1 = force_peak1;
      service_response_->force_steady1 = force_steady1;
      service_response_->distance_compressed_first = position_first_compression;
      service_response_->distance_compressed_second = position_second_compression;
      service_success_ = true;
      ROS_INFO("Grasp complete!");
    }
    return -1.0*high_force_;
  }

  //finishing the grasp
  else if(closed_loop_grasp_state == complete)
  {
    service_flag_ = true;
    if(closed_loop)
    {
      return service_request_->effort;
    }
    //TODO::determine a good output force
    return -1.0*high_force_;  //because we're done
  }

  else if(closed_loop_grasp_state == failed)
  {
    service_flag_ = true;
    return default_speed_;
  }

  return 0.0;

}

double Pr2GripperController::parseMessage(pr2_mechanism_controllers::GripperControllerCmd desired_msg)
{
  if(desired_msg.cmd.compare("grasp_cl"))
  {
    return grasp(true);
  }
  else if(desired_msg.cmd.compare("grasp") == 0)
  {
    return grasp(false);
  }
  else if(desired_msg.cmd.compare("move") == 0)
  {
    return desired_msg.val;
  }
  else if(desired_msg.cmd.compare("moveTo") == 0)
  {
    double direction = 1.0;
    if(joint_->position_ > desired_msg.val)
      direction = -1.0;
    if(proportional_offset_ > fabs(joint_->position_-desired_msg.val))
      return default_speed_*direction*fabs(joint_->position_-desired_msg.val)/proportional_offset_;
    return default_speed_*direction;
  }
  //This is calculated elsewhere
  else if(desired_msg.cmd.compare("step") == 0)
  {
    return last_commanded_command;
  }
  else if(desired_msg.cmd.compare("ramp") == 0)
  {
    return rampMove(desired_msg.start, desired_msg.end, desired_msg.time, desired_msg.end);
  }
  else if(desired_msg.cmd.compare("open") == 0)
  {
    return default_speed_;
  }
  else if(desired_msg.cmd.compare("close") == 0)
  {
    return -1.0*default_speed_;
  }
  else
  {
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
  pthread_mutex_lock(&pr2_gripper_controller_lock_);
  for(int i = 0; i < 22; i++)
  {
    pressure_state_.data0[i] = pressure_state->data0[i];
    pressure_state_.data1[i] = pressure_state->data0[i];
  }
  pthread_mutex_unlock(&pr2_gripper_controller_lock_);
}

bool Pr2GripperController::grasp_cl_srv(manipulation_srvs::GraspClosedLoop::Request &req, manipulation_srvs::GraspClosedLoop::Response &res)
{
  pthread_mutex_lock(&pr2_gripper_controller_lock_);
  grasp_cmd_.cmd = req.cmd;
  service_request_ = &req;
  service_response_ = &res;
  new_cmd_available_ = true;
  closed_loop_grasp_state = unstarted;
  cmd_received_timestamp_ = robot_state_->hw_->current_time_;
  pthread_mutex_unlock(&pr2_gripper_controller_lock_);
  service_flag_ = false;
  service_success_ = false;
  while(!service_flag_)
  {
    usleep(50000);
  }
  return service_success_;
}
