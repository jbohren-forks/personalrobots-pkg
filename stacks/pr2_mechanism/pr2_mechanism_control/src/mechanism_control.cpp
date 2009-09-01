////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Willow Garage, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////
/*
 * Author: Stuart Glaser, Wim Meeussen
 */

#include "pr2_mechanism_control/mechanism_control.h"
#include "pr2_mechanism_control/scheduler.h"
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <sstream>
#include "ros/console.h"
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

using namespace pr2_mechanism;
using namespace controller;
using namespace boost::accumulators;
using namespace ros;


MechanismControl::MechanismControl(HardwareInterface *hw) :
  model_(hw),
  state_(NULL), hw_(hw),
  controller_loader_("pr2_controller_interface", "controller::Controller"),
  start_request_(0),
  stop_request_(0),
  please_switch_(false),
  switch_success_(false),
  current_controllers_list_(0),
  used_by_realtime_(-1),
  pub_diagnostics_(node_, "/diagnostics", 1),
  pub_joint_state_(node_, "joint_states", 1),
  pub_mech_state_(node_, "mechanism_state", 1),
  last_published_state_(ros::Time::now().toSec()),
  last_published_diagnostics_(ros::Time::now().toSec())
{}

MechanismControl::~MechanismControl()
{
  if (state_)
    delete state_;
}


bool MechanismControl::initXml(TiXmlElement* config)
{
  model_.initXml(config);
  state_ = new RobotState(&model_, hw_);

  // pre-allocate for realtime publishing
  pub_mech_state_.msg_.set_actuator_states_size(hw_->actuators_.size());
  int joints_size = 0;
  for (unsigned int i = 0; i < model_.joints_.size(); ++i)
  {
    int type = state_->joint_states_[i].joint_->type_;
    if (type == urdf::Joint::REVOLUTE || type == urdf::Joint::CONTINUOUS || type == urdf::Joint::PRISMATIC)
      ++joints_size;
  }
  pub_mech_state_.msg_.set_joint_states_size(joints_size);
  pub_joint_state_.msg_.set_name_size(joints_size);
  pub_joint_state_.msg_.set_position_size(joints_size);
  pub_joint_state_.msg_.set_velocity_size(joints_size);
  pub_joint_state_.msg_.set_effort_size(joints_size);

  // Advertise services
  srv_list_controllers_ = node_.advertiseService("list_controllers", &MechanismControl::listControllersSrv, this);
  srv_list_controller_types_ = node_.advertiseService("list_controller_types", &MechanismControl::listControllerTypesSrv, this);
  srv_spawn_controller_ = node_.advertiseService("spawn_controller", &MechanismControl::spawnControllerSrv, this);
  srv_kill_controller_ = node_.advertiseService("kill_controller", &MechanismControl::killControllerSrv, this);
  srv_switch_controller_ = node_.advertiseService("switch_controller", &MechanismControl::switchControllerSrv, this);

  // get the publish rate for mechanism state and diagnostics
  double publish_rate_state, publish_rate_diagnostics;
  node_.param("~publish_rate_mechanism_state", publish_rate_state, 100.0);
  node_.param("~publish_rate_diagnostics", publish_rate_diagnostics, 1.0);
  publish_period_state_ = 1.0/fmax(0.000001, publish_rate_state);
  publish_period_diagnostics_ = 1.0/fmax(0.000001, publish_rate_diagnostics);

  return true;
}





// Must be realtime safe.
void MechanismControl::update()
{
  used_by_realtime_ = current_controllers_list_;
  std::vector<ControllerSpec> &controllers = controllers_lists_[used_by_realtime_];
  std::vector<size_t> &scheduling = controllers_scheduling_[used_by_realtime_];

  double start = ros::Time::now().toSec();
  state_->propagateState();
  state_->zeroCommands();
  double start_update = ros::Time::now().toSec();
  pre_update_stats_.acc(start_update - start);

  // Update all controllers in scheduling order
  for (size_t i=0; i<controllers.size(); i++){
    double start = ros::Time::now().toSec();
    controllers[scheduling[i]].c->updateRequest();
    double end = ros::Time::now().toSec();
    controllers[scheduling[i]].stats->acc(end - start);
  }
  double end_update = ros::Time::now().toSec();
  update_stats_.acc(end_update - start_update);

  state_->enforceSafety();
  state_->propagateEffort();
  double end = ros::Time::now().toSec();
  post_update_stats_.acc(end - end_update);

  // publish diagnostics and state
  publishDiagnostics();
  publishState();

  // there are controllers to atomically start/stop
  if (please_switch_)
  {
    // try to start controllers
    switch_success_ = true;
    int last_started = -1;
    for (unsigned int i=0; i<start_request_.size(); i++){
      if (!start_request_[i]->startRequest() &&
          switch_strictness_ == pr2_mechanism_msgs::SwitchController::Request::STRICT){
        switch_success_ = false;
        break;
      }
      last_started = i;
    }

    // if starting failed, stop them again
    if (!switch_success_){
      for (int i=0; i<=last_started; i++){
        start_request_[i]->stopRequest();
      }
    }
    // stop controllers
    else {
      for (unsigned int i=0; i<stop_request_.size(); i++)
        stop_request_[i]->stopRequest();
    }

    start_request_.clear();
    stop_request_.clear();
    please_switch_ = false;
  }
}

controller::Controller* MechanismControl::getControllerByName(const std::string& name)
{
  std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
  for (size_t i = 0; i < controllers.size(); ++i)
  {
    if (controllers[i].name == name)
      return controllers[i].c.get();
  }
  return NULL;
}

void MechanismControl::getControllerNames(std::vector<std::string> &names)
{
  boost::mutex::scoped_lock guard(controllers_lock_);
  std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
  for (size_t i = 0; i < controllers.size(); ++i)
  {
    names.push_back(controllers[i].name);
  }
}

void MechanismControl::getControllerSchedule(std::vector<size_t> &schedule)
{
  boost::mutex::scoped_lock guard(controllers_lock_);
  schedule = controllers_scheduling_[current_controllers_list_];
}


bool MechanismControl::spawnController(const std::string& name)
{
  ROS_DEBUG("Will spawn controller '%s'", name.c_str());

  // lock controllers
  boost::mutex::scoped_lock guard(controllers_lock_);

  // get reference to controller list
  int free_controllers_list = (current_controllers_list_ + 1) % 2;
  while (free_controllers_list == used_by_realtime_)
    usleep(200);
  std::vector<ControllerSpec>
    &from = controllers_lists_[current_controllers_list_],
    &to = controllers_lists_[free_controllers_list];
  to.clear();

  // Copy all controllers from the 'from' list to the 'to' list
  for (size_t i = 0; i < from.size(); ++i)
    to.push_back(from[i]);

  // Checks that we're not duplicating controllers
  for (size_t j = 0; j < to.size(); ++j)
  {
    if (to[j].name == name)
    {
      to.clear();
      ROS_ERROR("A controller named \"%s\" was already spawned inside mechanism control", name.c_str());
      return false;
    }
  }

  // Constructs the controller
  NodeHandle c_node(node_, name);
  controller::Controller *c = NULL;
  std::string type;
  if (c_node.getParam("type", type))
  {
    ROS_DEBUG("Constructing controller '%s' of type '%s'", name.c_str(), type.c_str());
    try {
      c = controller_loader_.createClassInstance(type, true);
    }
    catch (const std::runtime_error &ex)
    {
      ROS_ERROR("Could not load class %s: %s", type.c_str(), ex.what());
    }
  }

  // checks if controller was constructed
  if (c == NULL)
  {
    to.clear();
    if (type == "")
      ROS_ERROR("Could not spawn controller '%s' because the type was not specified. Did you load the controller configuration on the parameter server?",
              name.c_str());
    else
      ROS_ERROR("Could not spawn controller '%s' because controller type '%s' does not exist",
                name.c_str(), type.c_str());
    return false;
  }

  // Initializes the controller
  ROS_DEBUG("Initializing controller '%s'", name.c_str());
  bool initialized = c->initRequest(this, state_, c_node);
  if (!initialized)
  {
    to.clear();
    delete c;
    ROS_ERROR("Initializing controller '%s' failed", name.c_str());
    return false;
  }
  ROS_DEBUG("Initialized controller '%s' succesful", name.c_str());

  // Adds the controller to the new list
  to.resize(to.size() + 1);
  to[to.size()-1].name = name;
  to[to.size()-1].c.reset(c);

  //  Do the controller scheduling
  if (!scheduleControllers(to, controllers_scheduling_[free_controllers_list])){
    to.clear();
    delete c;
    ROS_ERROR("Scheduling controller '%s' failed", name.c_str());
    return false;
  }

  // Success!  Swaps in the new set of controllers.
  int former_current_controllers_list_ = current_controllers_list_;
  current_controllers_list_ = free_controllers_list;

  // Destroys the old controllers list when the realtime thread is finished with it.
  while (used_by_realtime_ == former_current_controllers_list_)
    usleep(200);
  from.clear();

  ROS_DEBUG("Successfully spawned controller '%s'", name.c_str());
  return true;
}




bool MechanismControl::killController(const std::string &name)
{
  ROS_DEBUG("Will kill controller '%s'", name.c_str());

  // lock the controllers
  boost::mutex::scoped_lock guard(controllers_lock_);

  // get reference to controller list
  int free_controllers_list = (current_controllers_list_ + 1) % 2;
  while (free_controllers_list == used_by_realtime_)
    usleep(200);
  std::vector<ControllerSpec>
    &from = controllers_lists_[current_controllers_list_],
    &to = controllers_lists_[free_controllers_list];
  to.clear();

  // check if no other controller depends on this controller
  for (size_t i = 0; i < from.size(); ++i){
    for (size_t b=0; b<from[i].c->before_list_.size(); b++){
      if (name == from[i].c->before_list_[b]){
        ROS_ERROR("Cannot kill controller %s because controller %s still depends on it",
                  name.c_str(), from[i].name.c_str());
        return false;
      }
    }
    for (size_t a=0; a<from[i].c->after_list_.size(); a++){
      if (name == from[i].c->after_list_[a]){
        ROS_ERROR("Cannot kill controller %s because controller %s still depends on it",
                  name.c_str(), from[i].name.c_str());
        return false;
      }
    }
  }

  // Transfers the running controllers over, skipping the one to be removed.
  bool removed = false;
  for (size_t i = 0; i < from.size(); ++i)
  {
    if (from[i].name == name)
      removed = true;
    else
      to.push_back(from[i]);
  }

  // Fails if we could not remove the controllers
  if (!removed)
  {
    to.clear();
    ROS_ERROR("Could not kill controller with name %s because no controller with this name exists",
              name.c_str());
    return false;
  }

  //  Do the controller scheduling
  if (!scheduleControllers(to, controllers_scheduling_[free_controllers_list])){
    to.clear();
    ROS_ERROR("Scheduling controllers failed when removing controller '%s' failed", name.c_str());
    return false;
  }

  // Success!  Swaps in the new set of controllers.
  int former_current_controllers_list_ = current_controllers_list_;
  current_controllers_list_ = free_controllers_list;

  // Destroys the old controllers list when the realtime thread is finished with it.
  while (used_by_realtime_ == former_current_controllers_list_)
    usleep(200);
  from.clear();

  ROS_DEBUG("Successfully killed controller '%s'", name.c_str());
  return true;
}



bool MechanismControl::switchController(const std::vector<std::string>& start_controllers,
                                        const std::vector<std::string>& stop_controllers,
                                        int strictness)
{
  ROS_DEBUG("switching controllers:");
  for (unsigned int i=0; i<start_controllers.size(); i++)
    ROS_DEBUG(" - starting controller %s", start_controllers[i].c_str());
  for (unsigned int i=0; i<stop_controllers.size(); i++)
    ROS_DEBUG(" - stopping controller %s", stop_controllers[i].c_str());

  // lock controllers
  boost::mutex::scoped_lock guard(controllers_lock_);

  controller::Controller* ct;
  // list all controllers to stop
  for (unsigned int i=0; i<stop_controllers.size(); i++)
  {
    ct = getControllerByName(stop_controllers[i]);
    if (ct == NULL){
      if (strictness ==  pr2_mechanism_msgs::SwitchController::Request::STRICT){
        ROS_ERROR("Could not stop controller with name %s because no controller with this name exists",
                  stop_controllers[i].c_str());
        return false;
      }
    }
    else
      stop_request_.push_back(ct);
  }

  // list all controllers to start
  for (unsigned int i=0; i<start_controllers.size(); i++)
  {
    ct = getControllerByName(start_controllers[i]);
    if (ct == NULL){
      if (strictness ==  pr2_mechanism_msgs::SwitchController::Request::STRICT){
        ROS_ERROR("Could not start controller with name %s because no controller with this name exists",
                  start_controllers[i].c_str());
        return false;
      }
    }
    else
      start_request_.push_back(ct);
  }

  // start the atomic controller switching
  switch_strictness_ = strictness;
  please_switch_ = true;

  // wait until switch is finished
  while (please_switch_)
    usleep(100);

  ROS_DEBUG("Successfully switched controllers");
  return switch_success_;
}



void MechanismControl::publishDiagnostics()
{
  if (round ((ros::Time::now().toSec() - last_published_diagnostics_ - publish_period_diagnostics_)/ (0.000001)) >= 0)
  {
    last_published_diagnostics_ = ros::Time::now().toSec();
    if (pub_diagnostics_.trylock())
    {
      std::vector<ControllerSpec> &controllers = controllers_lists_[used_by_realtime_];
      int active = 0;
      TimeStatistics blank_statistics;

      std::vector<diagnostic_msgs::DiagnosticStatus> statuses;
      diagnostic_updater::DiagnosticStatusWrapper status;

      status.name = "Mechanism Control";
      status.summary(0, "OK");

      for (size_t i = 0; i < controllers.size(); ++i)
      {
        ++active;
        double m = extract_result<tag::max>(controllers[i].stats->acc);
        controllers[i].stats->max1.push_back(m);
        controllers[i].stats->max = std::max(m, controllers[i].stats->max);
        std::string state;
        if (controllers[i].c->isRunning())
          state = "Running";
        else
          state = "Stopped";
        status.addf(controllers[i].name,
                       "%.4f (avg) %.4f (stdev) %.4f (max) %.4f (1-min max) %.4f (life max) %s (state)",
                       mean(controllers[i].stats->acc)*1e6,
                       sqrt(variance(controllers[i].stats->acc))*1e6,
                       m*1e6,
                       *std::max_element(controllers[i].stats->max1.begin(), controllers[i].stats->max1.end())*1e6,
                       controllers[i].stats->max*1e6,
                       state.c_str());

        controllers[i].stats->acc = blank_statistics;  // clear
      }

#define REPORT_STATS(stats_, label)             \
      {                                              \
        double m = extract_result<tag::max>(stats_.acc);        \
        stats_.max1.push_back(m);                               \
        stats_.max = std::max(m, stats_.max);                           \
        status.addf(label, "%.4f (avg) %.4f (stdev) %.4f (max) %.4f (1-min max) %.4f (life max)", \
                       mean(stats_.acc)*1e6,                            \
                       sqrt(variance(stats_.acc))*1e6,                  \
                       m*1e6,                                           \
                       *std::max_element(stats_.max1.begin(), stats_.max1.end())*1e6, \
                       stats_.max*1e6);                                 \
        stats_.acc = blank_statistics;                                  \
      }

      REPORT_STATS(pre_update_stats_, "Before Update");
      REPORT_STATS(update_stats_, "Update");
      REPORT_STATS(post_update_stats_, "After Update");

      status.addf("Active controllers", "%d", active);

      statuses.push_back(status);
      pub_diagnostics_.msg_.set_status_vec(statuses);
      pub_diagnostics_.unlockAndPublish();
    }
  }
}


void MechanismControl::publishState()
{
  if (round ((ros::Time::now().toSec() - last_published_state_ - publish_period_state_)/ (0.000001)) >= 0)
  {
    last_published_state_ = ros::Time::now().toSec();
    if (pub_mech_state_.trylock())
    {
      unsigned int j = 0;
      for (unsigned int i = 0; i < model_.joints_.size(); ++i)
      {
        int type = state_->joint_states_[i].joint_->type_;
        if (type == urdf::Joint::REVOLUTE || type == urdf::Joint::CONTINUOUS || type == urdf::Joint::PRISMATIC)
        {
          assert(j < pub_mech_state_.msg_.get_joint_states_size());
          pr2_mechanism::JointState *in = &state_->joint_states_[i];
          pr2_mechanism_msgs::JointState *out = &pub_mech_state_.msg_.joint_states[j];
          out->name = model_.joints_[i]->name_;
          out->position = in->position_;
          out->velocity = in->velocity_;
          out->applied_effort = in->applied_effort_;
          out->commanded_effort = in->commanded_effort_;
          out->is_calibrated = in->calibrated_;
          j++;
        }
      }

      for (unsigned int i = 0; i < hw_->actuators_.size(); ++i)
      {
        pr2_mechanism_msgs::ActuatorState *out = &pub_mech_state_.msg_.actuator_states[i];
        ActuatorState *in = &hw_->actuators_[i]->state_;
        out->name = hw_->actuators_[i]->name_;
        out->encoder_count = in->encoder_count_;
        out->position = in->position_;
        out->timestamp = in->timestamp_;
        out->device_id = in->device_id_;
        out->encoder_velocity = in->encoder_velocity_;
        out->velocity = in->velocity_;
        out->calibration_reading = in->calibration_reading_;
        out->calibration_rising_edge_valid = in->calibration_rising_edge_valid_;
        out->calibration_falling_edge_valid = in->calibration_falling_edge_valid_;
        out->last_calibration_rising_edge = in->last_calibration_rising_edge_;
        out->last_calibration_falling_edge = in->last_calibration_falling_edge_;
        out->is_enabled = in->is_enabled_;
        out->run_stop_hit = in->run_stop_hit_;
        out->last_requested_current = in->last_requested_current_;
        out->last_commanded_current = in->last_commanded_current_;
        out->last_measured_current = in->last_measured_current_;
        out->last_requested_effort = in->last_requested_effort_;
        out->last_commanded_effort = in->last_commanded_effort_;
        out->last_measured_effort = in->last_measured_effort_;
        out->motor_voltage = in->motor_voltage_;
        out->num_encoder_errors = in->num_encoder_errors_;
      }
      pub_mech_state_.msg_.header.stamp = ros::Time::now();
      pub_mech_state_.msg_.time = ros::Time::now().toSec();

      pub_mech_state_.unlockAndPublish();
    }

    if (pub_joint_state_.trylock())
    {
      unsigned int j = 0;
      for (unsigned int i = 0; i < model_.joints_.size(); ++i)
      {
        int type = state_->joint_states_[i].joint_->type_;
        if (type == urdf::Joint::REVOLUTE || type == urdf::Joint::CONTINUOUS || type == urdf::Joint::PRISMATIC)
        {
          assert(j < pub_joint_state_.msg_.get_name_size());
          assert(j < pub_joint_state_.msg_.get_position_size());
          assert(j < pub_joint_state_.msg_.get_velocity_size());
          assert(j < pub_joint_state_.msg_.get_effort_size());
          pr2_mechanism::JointState *in = &state_->joint_states_[i];
          pub_joint_state_.msg_.name[j] = model_.joints_[i]->name_;
          pub_joint_state_.msg_.position[j] = in->position_;
          pub_joint_state_.msg_.velocity[j] = in->velocity_;
          pub_joint_state_.msg_.effort[j] = in->applied_effort_;

          j++;
        }
      }

      pub_joint_state_.msg_.header.stamp = ros::Time::now();

      pub_joint_state_.unlockAndPublish();
    }
  }
}

bool MechanismControl::listControllerTypesSrv(
  pr2_mechanism_msgs::ListControllerTypes::Request &req,
  pr2_mechanism_msgs::ListControllerTypes::Response &resp)
{
  (void) req;
  //std::vector<std::string> types = controller::ControllerHandleFactory::Instance().RegisteredIds();
  std::vector<std::string> types = controller_loader_.getDeclaredClasses();

  resp.set_types_vec(types);
  return true;
}


bool MechanismControl::listControllersSrv(
  pr2_mechanism_msgs::ListControllers::Request &req,
  pr2_mechanism_msgs::ListControllers::Response &resp)
{
  // lock services
  boost::mutex::scoped_lock guard(services_lock_);
  std::vector<std::string> controllers;
  std::vector<size_t> schedule;

  (void) req;
  getControllerNames(controllers);
  getControllerSchedule(schedule);
  assert(controllers.size() == schedule.size());

  for (size_t i=0; i<controllers.size(); i++){
    // add controller state
    Controller* c = getControllerByName(controllers[schedule[i]]);
    assert(c);
    if (c->isRunning())
      controllers[schedule[i]] += " (Running)";
    else
      controllers[schedule[i]] += "  (Stopped)";
  }
  resp.set_controllers_vec(controllers);
  return true;
}


bool MechanismControl::spawnControllerSrv(
  pr2_mechanism_msgs::SpawnController::Request &req,
  pr2_mechanism_msgs::SpawnController::Response &resp)
{
  ROS_DEBUG("spawning service called for controller %s ",req.name.c_str());

  // lock services
  boost::mutex::scoped_lock guard(services_lock_);

  resp.ok = spawnController(req.name);

  ROS_DEBUG("spawning service finished for controller %s ",req.name.c_str());
  return true;
}


bool MechanismControl::killControllerSrv(
  pr2_mechanism_msgs::KillController::Request &req,
  pr2_mechanism_msgs::KillController::Response &resp)
{
  ROS_DEBUG("killing service called for controller %s ",req.name.c_str());

  // lock services
  boost::mutex::scoped_lock guard(services_lock_);
  resp.ok = killController(req.name);

  ROS_DEBUG("killing service finished for controller %s ",req.name.c_str());
  return true;
}


bool MechanismControl::switchControllerSrv(
  pr2_mechanism_msgs::SwitchController::Request &req,
  pr2_mechanism_msgs::SwitchController::Response &resp)
{
  ROS_DEBUG("switching service called");

  // lock services
  boost::mutex::scoped_lock guard(services_lock_);

  resp.ok = switchController(req.start_controllers, req.stop_controllers, req.strictness);

  ROS_DEBUG("switching service finished");
  return true;
}

