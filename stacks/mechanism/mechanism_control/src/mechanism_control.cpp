///////////////////////////////////////////////////////////////////////////////
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

#include "mechanism_control/mechanism_control.h"
#include <algorithm>
#include <boost/thread/thread.hpp>
#include "ros/console.h"

using namespace mechanism;
using namespace boost::accumulators;


MechanismControl* MechanismControl::mechanism_control_ = NULL;

namespace controller {
Loki::Factory< controller::Controller, std::string >& getControllerFactoryInstance()
{
  return controller::ControllerFactory::Instance();
}
}

MechanismControl::MechanismControl(HardwareInterface *hw) :
  state_(NULL), hw_(hw), initialized_(0),
  switch_success_(false),
  current_controllers_list_(0), used_by_realtime_(-1), publisher_("/diagnostics", 1),
  start_request_(0),
  stop_request_(0),
  please_switch_(false),
  last_published_(realtime_gettime())
{
  model_.hw_ = hw;
  mechanism_control_ = this;
}

MechanismControl::~MechanismControl()
{
  if (state_)
    delete state_;
}

bool MechanismControl::Instance(MechanismControl*& mech)
{
  if (mechanism_control_ == NULL) return false;

  mech = mechanism_control_;
  return true;
}

bool MechanismControl::initXml(TiXmlElement* config)
{
  model_.initXml(config);
  state_ = new RobotState(&model_, hw_);

  initialized_ = true;
  return true;
}

#define ADD_STRING_FMT(lab, fmt, ...) \
  s.label = (lab); \
  { char buf[1024]; \
    snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
    s.value = buf; \
  } \
  strings.push_back(s)

void MechanismControl::publishDiagnostics()
{
  if (publisher_.trylock())
  {
    std::vector<ControllerSpec> &controllers = controllers_lists_[used_by_realtime_];
    int active = 0;
    TimeStatistics blank_statistics;

    std::vector<diagnostic_msgs::DiagnosticStatus> statuses;
    std::vector<diagnostic_msgs::DiagnosticValue> values;
    std::vector<diagnostic_msgs::DiagnosticString> strings;
    diagnostic_msgs::DiagnosticStatus status;
    diagnostic_msgs::DiagnosticValue v;
    diagnostic_msgs::DiagnosticString s;

    status.name = "Mechanism Control";
    status.level = 0;
    status.message = "OK";

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
      ADD_STRING_FMT(controllers[i].name,
                     "%.4f (avg) %.4f (stdev) %.4f (max) %.4f (1-min max) %.4f (life max) %s (state)",
                     mean(controllers[i].stats->acc)*1e6,
                     sqrt(variance(controllers[i].stats->acc))*1e6,
                     m*1e6,
                     *std::max_element(controllers[i].stats->max1.begin(), controllers[i].stats->max1.end())*1e6,
                     controllers[i].stats->max*1e6,
                     state.c_str());

      controllers[i].stats->acc = blank_statistics;  // clear
    }

#define REPORT_STATS(stats_, label) \
    { \
    double m = extract_result<tag::max>(stats_.acc); \
    stats_.max1.push_back(m); \
    stats_.max = std::max(m, stats_.max); \
    ADD_STRING_FMT(label, "%.4f (avg) %.4f (stdev) %.4f (max) %.4f (1-min max) %.4f (life max)", \
                   mean(stats_.acc)*1e6, \
                   sqrt(variance(stats_.acc))*1e6, \
                   m*1e6, \
                   *std::max_element(stats_.max1.begin(), stats_.max1.end())*1e6, \
                   stats_.max*1e6); \
    stats_.acc = blank_statistics;  \
    }

    REPORT_STATS(pre_update_stats_, "Before Update");
    REPORT_STATS(update_stats_, "Update");
    REPORT_STATS(post_update_stats_, "After Update");

    ADD_STRING_FMT("Active controllers", "%d", active);

    status.set_values_vec(values);
    status.set_strings_vec(strings);
    statuses.push_back(status);
    publisher_.msg_.set_status_vec(statuses);
    publisher_.unlockAndPublish();

  }
}

// Must be realtime safe.
void MechanismControl::update()
{
  used_by_realtime_ = current_controllers_list_;
  std::vector<ControllerSpec> &controllers = controllers_lists_[used_by_realtime_];

  double start = realtime_gettime();
  state_->propagateState();
  state_->zeroCommands();
  double start_update = realtime_gettime();
  pre_update_stats_.acc(start_update - start);

  // Update all controllers
  // Start with controller that was last added
  for (int i=controllers.size()-1; i>=0; i--)
  {
    double start = realtime_gettime();
    controllers[i].c->updateRequest();
    double end = realtime_gettime();
    controllers[i].stats->acc(end - start);
  }
  double end_update = realtime_gettime();
  update_stats_.acc(end_update - start_update);

  state_->enforceSafety();
  state_->propagateEffort();
  double end = realtime_gettime();
  post_update_stats_.acc(end - end_update);

  if ((end - last_published_) > 1.0)
  {
    publishDiagnostics();
    last_published_ = end;
  }

  // there are controllers to start/stop
  if (please_switch_)
  {
    // try to start controllers
    switch_success_ = true;
    int last_started = -1;
    for (unsigned int i=0; i<start_request_.size(); i++){
      if (!start_request_[i]->startRequest()){
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

bool MechanismControl::spawnController(const std::string& xml_string,
                                       std::vector<int8_t>& ok,
                                       std::vector<std::string>& name)
{
  TiXmlDocument doc;
  doc.Parse(xml_string.c_str());

  std::vector<int8_t> oks;
  std::vector<std::string> names;

  TiXmlElement *config = doc.RootElement();
  if (!config)
  {
    ROS_ERROR("The XML given to SpawnController could not be parsed");
    return false;
  }
  if (config->ValueStr() != "controllers" &&
      config->ValueStr() != "controller")
  {
    ROS_ERROR("The XML given to SpawnController must have either \"controller\" or \
\"controllers\" as the root tag");
    return false;
  }

  if (config->ValueStr() == "controllers")
  {
    config = config->FirstChildElement("controller");
  }

  size_t last = 0;
  for (; config; config = config->NextSiblingElement("controller"))
  {
    ok.resize(last+1);
    name.resize(last+1);
    ok[last] = spawnController(config, name[last]);
    last++;
  }

  return true;
}


bool MechanismControl::spawnController(TiXmlElement *config, std::string& name)
{
  // lock controllers
  boost::mutex::scoped_lock guard(controllers_lock_);

  // parse name from xml
  if (!config->Attribute("name"))
  {
    ROS_ERROR("Could not spawn a controller because no name was given");
    name = "Could not parse name";
    return false;
  }
  name = config->Attribute("name");

  // get reference to controller list
  int free_controllers_list = (current_controllers_list_ + 1) % 2;
  while (free_controllers_list == used_by_realtime_)
    usleep(200);
  std::vector<ControllerSpec>
    &from = controllers_lists_[current_controllers_list_],
    &to = controllers_lists_[free_controllers_list];
  to.clear();

  // Copy the running controllers from the 'from' list to the 'to' list
  for (size_t i = 0; i < from.size(); ++i)
    to.push_back(from[i]);

  // Checks that we're not duplicating controllers
  for (size_t j = 0; j < to.size(); ++j)
  {
    if (to[j].name == name)
    {
      to.clear();
      ROS_ERROR("A controller named \"%s\" already exists", name.c_str());
      return false;
    }
  }

  // Constructs the controller
  controller::Controller *c = NULL;
  bool initialized = false;

  // Tries initializing the controller in the new way (with NodeHandles)
  std::string type;
  ros::NodeHandle c_node(ros::NodeHandle(), name);
  if (c_node.getParam("type", type))
  {
    try {
      c = controller::ControllerFactory::Instance().CreateObject(type);

      initialized = c->initRequest(state_, c_node);
      if (!initialized)
        delete c;
    } catch(Loki::DefaultFactoryError<std::string, controller::Controller>::Exception)
    {}
  }

  if (!initialized)
  {
    // parse type from xml
    if (!config->Attribute("type"))
    {
      ROS_ERROR("Could not spawn controller named \"%s\" because no type was given", name.c_str());
      return false;
    }
    type = config->Attribute("type");

    try {
      c = controller::ControllerFactory::Instance().CreateObject(type);
    }
    catch (Loki::DefaultFactoryError<std::string, controller::Controller>::Exception)
    {}
    if (c == NULL)
    {
      to.clear();
      ROS_ERROR("Could not spawn controller '%s' because controller type '%s' does not exist",
                name.c_str(), type.c_str());
      return false;
    }

    // Initializes the controller
    initialized = c->initXmlRequest(state_, config, name);
    if (!initialized)
    {
      to.clear();
      delete c;
      ROS_ERROR("Initializing controller \"%s\" failed", name.c_str());
      return false;
    }
  }

  // Adds the controller to the new list
  to.resize(to.size() + 1);
  to[to.size()-1].name = name;
  to[to.size()-1].c.reset(c);

  // Success!  Swaps in the new set of controllers.
  int former_current_controllers_list_ = current_controllers_list_;
  current_controllers_list_ = free_controllers_list;

  // Destroys the old controllers list when the realtime thread is finished with it.
  while (used_by_realtime_ == former_current_controllers_list_)
    usleep(200);
  from.clear();

  return true;
}



bool MechanismControl::killController(const std::string &name)
{
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
    ROS_ERROR("Could not find controller named \"%s\" to remove", name.c_str());
    return false;
  }

  // Success!  Swaps in the new set of controllers.
  int former_current_controllers_list_ = current_controllers_list_;
  current_controllers_list_ = free_controllers_list;

  // Destroys the old controllers list when the realtime thread is finished with it.
  while (used_by_realtime_ == former_current_controllers_list_)
    usleep(200);
  from.clear();

  return true;
}



bool MechanismControl::switchController(const std::vector<std::string>& start_controllers,
                                        const std::vector<std::string>& stop_controllers)
{
  // lock controllers
  boost::mutex::scoped_lock guard(controllers_lock_);

  controller::Controller* ct;
  // list all controllers to stop
  for (unsigned int i=0; i<stop_controllers.size(); i++)
  {
    ct = getControllerByName(stop_controllers[i]);
    if (ct == NULL){
      ROS_ERROR("Could not stop controller with name %s because no controller with this name exists",
                stop_controllers[i].c_str());
      return false;
    }
    stop_request_.push_back(ct);
  }

  // list all controllers to start
  for (unsigned int i=0; i<start_controllers.size(); i++)
  {
    ct = getControllerByName(start_controllers[i]);
    if (ct == NULL){
      ROS_ERROR("Could not start controller with name %s because no controller with this name exists",
                start_controllers[i].c_str());
      return false;
    }
    start_request_.push_back(ct);
  }

  // start the atomic controller switching
  please_switch_ = true;

  // wait until switch is finished
  while (please_switch_)
    usleep(100);

  //controllers_lock_.unlock();
  guard.unlock();

  return switch_success_;
}










// -----------------------------
// -------- NODE ---------------
// -----------------------------

MechanismControlNode::MechanismControlNode(MechanismControl *mc)
  : mc_(mc),
    mechanism_state_topic_("mechanism_state"),
    publisher_(mechanism_state_topic_, 1),
    pub_joints_("joint_states", 1)
{
  assert(mc != NULL);
  assert(mechanism_state_topic_);
  if ((node_ = ros::Node::instance()) == NULL) {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv);
    node_ = new ros::Node("mechanism_control", ros::Node::DONT_HANDLE_SIGINT);
  }
}

MechanismControlNode::~MechanismControlNode()
{
}

bool MechanismControlNode::initXml(TiXmlElement *config)
{
  if (!mc_->initXml(config))
    return false;
  publisher_.msg_.set_actuator_states_size(mc_->hw_->actuators_.size());
  int joints_size = 0;
  for (unsigned int i = 0; i < mc_->model_.joints_.size(); ++i)
  {
    int type = mc_->state_->joint_states_[i].joint_->type_;
    if (type == JOINT_ROTARY || type == JOINT_CONTINUOUS || type == JOINT_PRISMATIC)
      ++joints_size;
  }
  pub_joints_.msg_.set_joints_size(joints_size);
  publisher_.msg_.set_joint_states_size(joints_size);

  // Advertise services
  node_->advertiseService("list_controllers", &MechanismControlNode::listControllers, this);
  list_controllers_guard_.set("list_controllers");
  node_->advertiseService("list_controller_types", &MechanismControlNode::listControllerTypes, this);
  list_controller_types_guard_.set("list_controller_types");
  node_->advertiseService("spawn_controller", &MechanismControlNode::spawnController, this);
  spawn_controller_guard_.set("spawn_controller");
  node_->advertiseService("kill_controller", &MechanismControlNode::killController, this);
  kill_controller_guard_.set("kill_controller");
  node_->advertiseService("switch_controller", &MechanismControlNode::switchController, this);
  switch_controller_guard_.set("switch_controller");
  node_->advertiseService("kill_and_spawn_controllers", &MechanismControlNode::killAndSpawnControllers, this);
  kill_and_spawn_controllers_guard_.set("kill_and_spawn_controllers");


  // get the publish rate for mechanism state
  double publish_rate;
  node_->param("~publish_rate_mechanism_state", publish_rate, 100.0);
  publish_period_ = 1.0/fmax(0.000001,publish_rate);
  last_publish_ = realtime_gettime();
  return true;
}

void MechanismControlNode::update()
{
  mc_->update();

  if (round ((realtime_gettime() - last_publish_ - publish_period_)/ (0.000001)) >= 0)
  {
    last_publish_ = realtime_gettime();
    if (publisher_.trylock())
    {
      unsigned int j = 0;
      for (unsigned int i = 0; i < mc_->model_.joints_.size(); ++i)
      {
        int type = mc_->state_->joint_states_[i].joint_->type_;
        if (type == JOINT_ROTARY || type == JOINT_CONTINUOUS || type == JOINT_PRISMATIC)
        {
          assert(j < publisher_.msg_.get_joint_states_size());
          mechanism_msgs::JointState *out = &publisher_.msg_.joint_states[j++];
          mechanism::JointState *in = &mc_->state_->joint_states_[i];
          out->name = mc_->model_.joints_[i]->name_;
          out->position = in->position_;
          out->velocity = in->velocity_;
          out->applied_effort = in->applied_effort_;
          out->commanded_effort = in->commanded_effort_;
          out->is_calibrated = in->calibrated_;
        }
      }

      for (unsigned int i = 0; i < mc_->hw_->actuators_.size(); ++i)
      {
        mechanism_msgs::ActuatorState *out = &publisher_.msg_.actuator_states[i];
        ActuatorState *in = &mc_->hw_->actuators_[i]->state_;
        out->name = mc_->hw_->actuators_[i]->name_;
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
      publisher_.msg_.header.stamp.fromSec(realtime_gettime());
      publisher_.msg_.time = realtime_gettime();

      publisher_.unlockAndPublish();
    }

    if (pub_joints_.trylock())
    {
      unsigned int j = 0;
      for (unsigned int i = 0; i < mc_->model_.joints_.size(); ++i)
      {
        int type = mc_->state_->joint_states_[i].joint_->type_;
        if (type == JOINT_ROTARY || type == JOINT_CONTINUOUS || type == JOINT_PRISMATIC)
        {
          assert(j < pub_joints_.msg_.get_joints_size());
          mechanism_msgs::JointState *out = &pub_joints_.msg_.joints[j++];
          mechanism::JointState *in = &mc_->state_->joint_states_[i];
          out->name = mc_->model_.joints_[i]->name_;
          out->position = in->position_;
          out->velocity = in->velocity_;
          out->applied_effort = in->applied_effort_;
          out->commanded_effort = in->commanded_effort_;
          out->is_calibrated = in->calibrated_;
        }
      }

      pub_joints_.msg_.header.stamp.fromSec(realtime_gettime());

      pub_joints_.unlockAndPublish();
    }
  }
}

bool MechanismControlNode::listControllerTypes(
  mechanism_msgs::ListControllerTypes::Request &req,
  mechanism_msgs::ListControllerTypes::Response &resp)
{
  (void) req;
  std::vector<std::string> types = controller::ControllerFactory::Instance().RegisteredIds();
  resp.set_types_vec(types);
  return true;
}


bool MechanismControlNode::listControllers(
  mechanism_msgs::ListControllers::Request &req,
  mechanism_msgs::ListControllers::Response &resp)
{
  // lock services
  boost::mutex::scoped_lock guard(services_lock_);
  std::vector<std::string> controllers;

  (void) req;
  mc_->getControllerNames(controllers);

  for (size_t i=0; i<controllers.size(); i++){
    if (mc_->getControllerByName(controllers[i])->isRunning())
      controllers[i] += "  --> Running";
    else
      controllers[i] += "  --> Stopped";
  }
  resp.set_controllers_vec(controllers);
  return true;
}


bool MechanismControlNode::spawnController(
  mechanism_msgs::SpawnController::Request &req,
  mechanism_msgs::SpawnController::Response &resp)
{
  // lock services
  boost::mutex::scoped_lock guard(services_lock_);

  // spawn controllers
  if (!mc_->spawnController(req.xml_config, resp.ok, resp.name))
    return false;

  // start controllers if autostart true
  if (req.autostart){
    std::vector<std::string> start_list, stop_list;
    for (size_t i=0; i<resp.ok.size(); i++)
      if (resp.ok[i]) start_list.push_back(resp.name[i]);
    if (!start_list.empty())
      return mc_->switchController(start_list, stop_list);
  }
  return true;
}

bool MechanismControlNode::killController(
  mechanism_msgs::KillController::Request &req,
  mechanism_msgs::KillController::Response &resp)
{
  // lock services
  boost::mutex::scoped_lock guard(services_lock_);
  resp.ok = mc_->killController(req.name);
  return true;
}

bool MechanismControlNode::switchController(
  mechanism_msgs::SwitchController::Request &req,
  mechanism_msgs::SwitchController::Response &resp)
{
  // lock services
  boost::mutex::scoped_lock guard(services_lock_);

  resp.ok = mc_->switchController(req.start_controllers, req.stop_controllers);
  return true;
}

bool MechanismControlNode::killAndSpawnControllers(mechanism_msgs::KillAndSpawnControllers::Request &req,
                                                   mechanism_msgs::KillAndSpawnControllers::Response &res)
{
  // lock services
  boost::mutex::scoped_lock guard(services_lock_);

  // spawn controllers
  std::vector<std::string> spawn_name;
  std::vector<int8_t> spawn_ok;
  if (!mc_->spawnController(req.add_config, spawn_ok, spawn_name))
    return false;
  bool spawn_success = true;
  for (size_t i=0; i<spawn_ok.size(); i++){
    if (!spawn_ok[i]){
      spawn_success = false;
      mc_->killController(spawn_name[i]);
    }
  }
  if (!spawn_success) return false;

  // switch controllers
  if (!mc_->switchController(spawn_name, req.kill_name)){
    // kill controllers that were spawned
    for (size_t i=0; i<spawn_name.size(); i++)
      mc_->killController(spawn_name[i]);
    return false;
  }

  // kill controllers
  for (size_t i=0; i<req.kill_name.size(); i++)
    mc_->killController(req.kill_name[i]);

  return true;
}
