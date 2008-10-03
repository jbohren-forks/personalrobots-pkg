/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gazebo_plugin/gazebo_battery.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <set>
#include <stl_utils/stl_utils.h>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Model.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <map>

namespace gazebo {

  GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_battery", GazeboBattery);

  GazeboBattery::GazeboBattery(Entity *parent): Controller(parent)
  {
     this->parent_model_ = dynamic_cast<Model*>(this->parent);

     if (!this->parent_model_)
        gzthrow("GazeboBattery controller requires a Model as its parent");

    rosnode_ = ros::g_node; // comes from where?
    int argc = 0;
    char** argv = NULL;
    if (rosnode_ == NULL)
    {
      // this only works for a single camera.
      ros::init(argc,argv);
      rosnode_ = new ros::node("ros_gazebo",ros::node::DONT_HANDLE_SIGINT);
      printf("-------------------- starting node in P3D \n");
    }
  }

  GazeboBattery::~GazeboBattery()
  {
  }

  void GazeboBattery::LoadChild(XMLConfigNode *node)
  {
    this->stateTopicName_ = node->GetString("stateTopicName","battery_state",0);
    rosnode_->advertise<robot_msgs::BatteryState>(this->stateTopicName_,10);
    //this->diagnosticMessageTopicName_ = node->GetString("diagnosticMessageTopicName","diagnostic",0);
    //rosnode_->advertise<robot_msgs::DiagnosticMessage>(this->diagnosticMessageTopicName_,10);

    /// faking the plug and unplug of robot
    rosnode_->subscribe("plugged_in",this->plug_msg_,&GazeboBattery::SetPlug,this,10);

    this->full_capacity_       = node->GetDouble("full_charge_energy",0.0,0);
    this->default_charge_rate_ = node->GetDouble("default_charge_rate",-2.0,0);

    /// @todo make below useful
    //this->diagnostic_rate_     = node->GetDouble("diagnostic_rate",1.0,0);
    /// @todo make below useful
    this->battery_state_rate_  = node->GetDouble("dbattery_state_rate_",1.0,0);
  }

  void GazeboBattery::SetPlug()
  {
    this->lock_.lock();
    if (this->plug_msg_.status == "the robot is very much plugged into the wall")
      this->consumption_rate_ = this->default_charge_rate_ + DISCHARGE_RATE;
    else
      this->consumption_rate_ = DISCHARGE_RATE;
    this->lock_.unlock();
  }

  void GazeboBattery::InitChild()
  {
    this->current_time_ = Simulator::Instance()->GetSimTime();
    this->last_time_    = this->current_time_;

    /// initialize battery
    this->charge_           = this->full_capacity_; /// our convention is joules
    this->consumption_rate_ = DISCHARGE_RATE; /// time based decay rate in watts
  }

  void GazeboBattery::UpdateChild()
  {
    this->current_time_ = Simulator::Instance()->GetSimTime();

    /**********************************************************/
    /*                                                        */
    /*   update battery                                       */
    /*                                                        */
    /**********************************************************/
    this->charge_ = this->charge_ - (this->current_time_ - this->last_time_)*this->consumption_rate_;
    if (this->charge_ < 0) this->charge_ = 0;
    if (this->charge_ > this->full_capacity_) this->charge_ = this->full_capacity_;
    //std::cout << " battery charge remaining: " << this->charge_ << " Joules " << std::endl;

    /**********************************************************/
    /*                                                        */
    /* publish battery state                                  */
    /*                                                        */
    /**********************************************************/
    //this->battery_state_.header.frame_id = ; // no frame id for battery
    this->battery_state_.header.stamp.sec = (unsigned long)floor(this->current_time_);
    this->battery_state_.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->current_time_ - this->battery_state_.header.stamp.sec) );
    this->battery_state_.energy_remaining = this->charge_;
    this->battery_state_.power_consumption = this->consumption_rate_;

    this->lock_.lock();
    this->rosnode_->publish(this->stateTopicName_,this->battery_state_);
    this->lock_.unlock();
    
    /**********************************************************/
    /*                                                        */
    /* publish diagnostic message                             */
    /*                                                        */
    /**********************************************************/
    //this->diagnostic_status_.level = 0;
    //this->diagnostic_status_.name = "battery diagnostic";
    //this->diagnostic_status_.message = "battery ok";
    //this->diagnostic_message_.header = this->battery_state_.header;
    //this->diagnostic_message_.set_status_size(1);
    //this->diagnostic_message_.status[0] = this->diagnostic_status_;
    //this->lock_.lock();
    //this->rosnode_->publish(this->diagnosticMessageTopicName_,diagnostic_message_);
    //this->lock_.unlock();

    this->last_time_    = this->current_time_;
  }

  void GazeboBattery::FiniChild()
  {
    std::cout << "--------------- calling FiniChild in GazeboBattery --------------------" << std::endl;

    rosnode_->unadvertise(this->stateTopicName_);
    //rosnode_->unadvertise(this->diagnosticMessageTopicName_);
    rosnode_->unsubscribe("plugged_in");

  }



} // namespace gazebo


