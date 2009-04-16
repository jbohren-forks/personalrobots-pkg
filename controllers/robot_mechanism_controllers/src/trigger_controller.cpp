
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

/* Example .xml configuration file
<controllers>
<controller name="cam_controller" type="TriggerControllerNode"> 
  <actuator name="fl_caster_l_wheel_motor" />
  <waveform rep_rate="10" active_low="0" phase="0" duty_cycle=".5" running="1" pulsed="1" />
</controller>    
<controller name="led_controller" type="TriggerControllerNode">
  <actuator name="bl_caster_l_wheel_motor" />
  <waveform rep_rate="10" active_low="0" phase="0" duty_cycle=".5" running="1" pulsed="1" />
</controller>    
</controllers>    

There are three operating modes:
Constant: (running = 0)
  active_low sets the constant output value
Pulsed: (running = 1, pulsed = 1)
  active_low sets resting output value
  With rate rep_rate (Hz), the output is inverted for one cycle. The pulse is delayed using 
  phase varying from 0 to 1.
Rectangular: (running = 1, pulsed = 0)
  Rectangular wave with duty cycle set by duty_cycle (0 to 1), rate rep_rate (Hz), goes to
  !active_low at instant determined by phase.
*/

#include <ros/console.h>
#include <robot_mechanism_controllers/trigger_controller.h>

using std::string;
using namespace controller;

ROS_REGISTER_CONTROLLER(TriggerController);

TriggerController::TriggerController()
{
  ROS_DEBUG("creating controller...");
}

TriggerController::~TriggerController()
{
}

double TriggerController::getTick()
{
  return getTick(robot_->hw_->current_time_, config_);
}

void TriggerController::update()
{
  double tick = getTick();
  bool active = false;
  
  if (config_.running)
  {
    if (config_.pulsed)
    {
      active = (floor(prev_tick_) != floor(tick));
      //if (active)
      //  ROS_INFO("Triggered (%s)", actuator_name_.c_str()); // KILLME
    }
    else
    {
      active = fmod(tick, 1) < config_.duty_cycle;
      //if (active != fmod(prev_tick_, 1) < config_.duty_cycle)
      //  ROS_INFO("Changed to: %i (%s)", active, actuator_name_.c_str()); // KILLME
    }
  }

  actuator_command_->digital_out_ = active ^ config_.active_low;

  // ROS_INFO("digital out: %i (%s)", actuator_command_->digital_out_, actuator_name_.c_str());
  
  prev_tick_ = tick;  
}

bool TriggerController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_=robot;
  
  ROS_DEBUG("TriggerController::initXml starting");
  
  // Get the actuator name.
  
  TiXmlElement *a = config->FirstChildElement("actuator");
  if (!a)
  {
    ROS_ERROR("TriggerController was not given an actuator.");
    return false;
  }

  actuator_name_ = a->Attribute("name") ? a->Attribute("name") : "";
  
  Actuator *actuator = robot_->model_->getActuator(actuator_name_);
  if (!actuator)
  {
    ROS_ERROR("TriggerController could not find actuator named \"%s\".", 
        actuator_name_.c_str());
    return false;
  }

  actuator_command_ = &actuator->command_;
  
  // Get the startup configuration (pulsed or constant)

  config_.rep_rate =  1;
  config_.phase = 0;
  config_.running = false;
  config_.active_low = false;
  config_.pulsed = true;
  config_.duty_cycle = .5;

  TiXmlElement *w = config->FirstChildElement("waveform");
  if (w)
  {
    ROS_DEBUG("TriggerController::initXml reading pulsed configuration.");
    if (w->Attribute("rep_rate"))   config_.rep_rate   = atof(w->Attribute("rep_rate"));
    if (w->Attribute("phase"))      config_.phase      = atof(w->Attribute("phase"));
    if (w->Attribute("duty_cycle")) config_.duty_cycle = atof(w->Attribute("duty_cycle"));
    if (w->Attribute("active_low")) config_.active_low = atoi(w->Attribute("active_low"));
    if (w->Attribute("running"))    config_.running    = atoi(w->Attribute("running"));
    if (w->Attribute("pulsed"))     config_.pulsed     = atoi(w->Attribute("pulsed"));
    // FIXME Could I handle the booleans better?
  }

  prev_tick_ = getTick();
  
  ROS_DEBUG("TriggerController::initXml completed successfully"
      " rr=%f ph=%f al=%i r=%i p=%i dc=%f.",
      config_.rep_rate, config_.phase, config_.active_low, config_.running, config_.pulsed, config_.duty_cycle);

  return true;
}

//------------------- NODE -------------

ROS_REGISTER_CONTROLLER(TriggerControllerNode);

TriggerControllerNode::TriggerControllerNode()
{
  ROS_DEBUG("creating controller node...");
  c_ = new TriggerController();
  ROS_DEBUG("done");
}

TriggerControllerNode::~TriggerControllerNode()
{
  delete c_;
}

void TriggerControllerNode::update()
{
  c_->update();
}

bool TriggerControllerNode::setWaveformSrv(
    trigger_configuration &req,
    robot_mechanism_controllers::SetWaveform::Response &resp)
{
  // FIXME This should be safe despite the asynchronous barrier. Should I
  // be doing anything special to ensure that things get written in order?
  c_->config_.running = false; // Turn off pulsing before we start.
  c_->config_.rep_rate = req.rep_rate;
  c_->config_.phase = req.phase;
  c_->config_.duty_cycle = req.duty_cycle;
  c_->config_.active_low = !!req.active_low;
  c_->config_.pulsed = !!req.pulsed;
  c_->config_.running = !!req.running;
  
  ROS_DEBUG("TriggerController::setWaveformSrv completed successfully"
      " rr=%f ph=%f al=%i r=%i p=%i dc=%f.", c_->config_.rep_rate, c_->config_.phase, 
      c_->config_.active_low, c_->config_.running, c_->config_.pulsed, c_->config_.duty_cycle);

  return true;
}

bool TriggerControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  //Init the model.

  ROS_DEBUG("LOADING TRIGGER CONTROLLER NODE");
  ros::Node * const node = ros::Node::instance();
  string prefix = config->Attribute("name");
  ROS_DEBUG_STREAM("the prefix is "<<prefix);


  // Parses subcontroller configuration
  if(c_->initXml(robot, config))
  {
    node->advertiseService(prefix + "/set_waveform", &TriggerControllerNode::setWaveformSrv, this);

    // Default parameters

    ROS_DEBUG("DONE LOADING TRIGGER CONTROLLER NODE");
    return true;
  }
  ROS_DEBUG("ERROR LOADING TRIGGER CONTROLLER NODE");
  return false;
}
