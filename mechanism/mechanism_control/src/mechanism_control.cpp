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


MechanismControl::MechanismControl(HardwareInterface *hw) :
  state_(NULL), hw_(hw), initialized_(0),
  current_controllers_list_(0), used_by_realtime_(-1), publisher_("/diagnostics", 1),
  loop_count_(0)
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

    std::vector<robot_msgs::DiagnosticStatus> statuses;
    std::vector<robot_msgs::DiagnosticValue> values;
    std::vector<robot_msgs::DiagnosticString> strings;
    robot_msgs::DiagnosticStatus status;
    robot_msgs::DiagnosticValue v;
    robot_msgs::DiagnosticString s;

    status.name = "Mechanism Control";
    status.level = 0;
    status.message = "OK";

    for (size_t i = 0; i < controllers.size(); ++i)
    {
      if (controllers[i].state != EMPTY)
      {
        ++active;
        double m = extract_result<tag::max>(controllers[i].diagnostics->acc);
        controllers[i].diagnostics->max1.push_back(m);
        controllers[i].diagnostics->max = std::max(m, controllers[i].diagnostics->max);
        ADD_STRING_FMT(controllers[i].name,
                       "%.4f (avg) %.4f (stdev) %.4f (max) %.4f (1-min max) %.4f (life max)",
                       mean(controllers[i].diagnostics->acc)*1e6,
                       sqrt(variance(controllers[i].diagnostics->acc))*1e6,
                       m*1e6,
                       *std::max_element(controllers[i].diagnostics->max1.begin(), controllers[i].diagnostics->max1.end())*1e6,
                       controllers[i].diagnostics->max*1e6);

        controllers[i].diagnostics->acc = blank_statistics;  // clear
      }
    }

    double m = extract_result<tag::max>(diagnostics_.acc);
    diagnostics_.max1.push_back(m);
    diagnostics_.max = std::max(m, diagnostics_.max);
    ADD_STRING_FMT("Overall", "%.4f (avg) %.4f (stdev) %.4f (max) %.4f (1-min max) %.4f (life max)",
                   mean(diagnostics_.acc)*1e6,
                   sqrt(variance(diagnostics_.acc))*1e6,
                   m*1e6,
                   *std::max_element(diagnostics_.max1.begin(), diagnostics_.max1.end())*1e6,
                   diagnostics_.max*1e6);
    ADD_STRING_FMT("Active controllers", "%d", active);

    status.set_values_vec(values);
    status.set_strings_vec(strings);
    statuses.push_back(status);
    publisher_.msg_.set_status_vec(statuses);
    publisher_.unlockAndPublish();

    diagnostics_.acc = blank_statistics;  // clear
  }
}

// Must be realtime safe.
void MechanismControl::update()
{
  used_by_realtime_ = current_controllers_list_;
  std::vector<ControllerSpec> &controllers = controllers_lists_[used_by_realtime_];

  state_->propagateState();
  state_->zeroCommands();

  // Update all controllers
  // Start with controller that was last added
  double start_update = realtime_gettime();
  for (int i=controllers.size()-1; i>=0; i--)
  {
    if (controllers[i].state != EMPTY)
    {
      if (controllers[i].state == INITIALIZED)
      {
        controllers[i].state = RUNNING;
        controllers[i].c->starting();
      }
      double start = realtime_gettime();
      controllers[i].c->updateRequest();
      double end = realtime_gettime();
      controllers[i].diagnostics->acc(end - start);
    }
  }
  double end_update = realtime_gettime();
  diagnostics_.acc(end_update - start_update);

  state_->enforceSafety();
  state_->propagateEffort();

  if (++loop_count_ >= 1000)
  {
    loop_count_ = 0;
    publishDiagnostics();
  }

  // there are controllers to start/stop
  if (please_switch_)
  {
    // try to start controllers
    switch_success_ = true;
    for (unsigned int i=0; i<start_request_.size(); i++){
      if (!start_request_[i]->startRequest()){
        switch_success_ = false;
        break;
      }
    }

    // if starting failed, stop them again
    if (!switch_success_){
      for (unsigned int i=0; i<start_request_.size(); i++){
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
    if (controllers[i].state != EMPTY)
    {
      names.push_back(controllers[i].name);
    }
  }
}

bool MechanismControl::spawnController(const std::string &type,
                                       const std::string &name,
                                       TiXmlElement *config)
{
  std::vector<AddReq> add_reqs;
  std::vector<RemoveReq> remove_reqs;

  add_reqs.resize(1);
  add_reqs[0].name = name;
  add_reqs[0].type = type;
  add_reqs[0].config = config;

  changeControllers(remove_reqs, add_reqs);
  if (!add_reqs[0].success)
    return false;
  return true;
}

bool MechanismControl::killController(const std::string &name)
{
  std::vector<AddReq> add_reqs;
  std::vector<RemoveReq> remove_reqs;

  remove_reqs.resize(1);
  remove_reqs[0].name = name;

  changeControllers(remove_reqs, add_reqs);
  if (!remove_reqs[0].success)
    return false;
  return true;
}


bool MechanismControl::switchController(const std::vector<std::string>& start_controllers,
                                        const std::vector<std::string>& stop_controllers)
{
  controllers_lock_.lock();

  controller::Controller* ct;
  // list all controllers to stop
  for (unsigned int i=0; i<stop_controllers.size(); i++)
  {
    ct = getControllerByName(stop_controllers[i]);
    if (ct != NULL)
      stop_request_.push_back(ct);
  }

  // list all controllers to start
  for (unsigned int i=0; i<start_controllers.size(); i++)
  {
    ct = getControllerByName(start_controllers[i]);
    if (ct != NULL)
      start_request_.push_back(ct);
  }

  // start the atomic controller switching
  please_switch_ = true;

  // wait until switch is finished
  while (please_switch_)
    usleep(100);

  controllers_lock_.unlock();
  return switch_success_;
}


void MechanismControl::changeControllers(std::vector<RemoveReq> &remove_reqs,
                                         std::vector<AddReq> &add_reqs,
                                         const int strictness)
{
  timespec start_time, end_time;
  clock_gettime(CLOCK_REALTIME, &start_time);


  for (size_t i = 0; i < remove_reqs.size(); ++i)
    remove_reqs[i].success = false;
  for (size_t i = 0; i < add_reqs.size(); ++i)
    add_reqs[i].success = false;

  boost::mutex::scoped_lock guard(controllers_lock_);

  int free_controllers_list = (current_controllers_list_ + 1) % 2;
  while (free_controllers_list == used_by_realtime_)
    usleep(200);
  std::vector<ControllerSpec>
    &from = controllers_lists_[current_controllers_list_],
    &to = controllers_lists_[free_controllers_list];
  to.clear();

  // Transfers the running controllers over, skipping the ones to be removed.
  for (size_t i = 0; i < from.size(); ++i)
  {
    bool found = false;
    for (size_t j = 0; j < remove_reqs.size(); ++j)
    {
      if (from[i].name == remove_reqs[j].name)
      {
        remove_reqs[j].success = true;
        found = true;
        break;
      }
    }

    if (!found)
      to.push_back(from[i]);
  }

  // Fails if we could not remove all the controllers
  if (strictness >= 2)
  {
    for (size_t i = 0; i < remove_reqs.size(); ++i)
    {
      if (!remove_reqs[i].success)
        return;
    }
  }

  // Adds the new controllers
  std::vector<controller::Controller*> just_added;
  for (size_t i = 0; i < add_reqs.size(); ++i)
  {
    // Checks that we're not duplicating controllers
    bool exists = false;
    for (size_t j = 0; j < to.size(); ++j)
    {
      if (to[j].name == add_reqs[i].name)
      {
        exists = true;
      }
    }
    if (exists)
    {
      ROS_ERROR("A controller named \"%s\" already exists", add_reqs[i].name.c_str());
      continue;
    }

    // Constructs the controller
    controller::Controller *c = NULL;
    try {
      c = controller::ControllerFactory::Instance().CreateObject(add_reqs[i].type);
    } catch (Loki::DefaultFactoryError<std::string, controller::Controller>::Exception)
    {
      // Do nothing, c is already NULL
    }
    if (c == NULL)
    {
      ROS_ERROR("Could not spawn controller '%s' because controller type '%s' does not exist",
                add_reqs[i].name.c_str(), add_reqs[i].type.c_str());
      continue;
    }
    timespec init_start, init_end;
    clock_gettime(CLOCK_REALTIME, &init_start);
    bool initialized = c->initXmlRequest(state_, add_reqs[i].config, add_reqs[i].name);
    clock_gettime(CLOCK_REALTIME, &init_end);
    double duration = 1.0e3 * (init_end.tv_sec - init_start.tv_sec) +
      double(init_end.tv_nsec)/1.0e6 - double(init_start.tv_nsec)/1.0e6;
    ROS_DEBUG("  Initialized %s in %.3lf ms", add_reqs[i].name.c_str(), duration);
    if (!initialized)
    {
      delete c;
      continue;
    }

    // Adds the controller to the new list
    to.resize(to.size() + 1);
    to[to.size()-1].name = add_reqs[i].name;
    to[to.size()-1].c.reset(c);
    to[to.size()-1].state = INITIALIZED;
    add_reqs[i].success = true;
    just_added.push_back(c);
  }

  // Fails if we could not add all the controllers
  if (strictness >= 1)
  {
    for (size_t i = 0; i < add_reqs.size(); ++i)
    {
      if (!add_reqs[i].success)
      {
        // FAIL
        to.clear();
        return;
      }
    }
  }

  // Success!  Swaps in the new set of controllers.
  int former_current_controllers_list_ = current_controllers_list_;
  current_controllers_list_ = free_controllers_list;
  clock_gettime(CLOCK_REALTIME, &end_time);

  // Destroys the old controllers list when the realtime thread is finished with it.
  while (used_by_realtime_ == former_current_controllers_list_)
    usleep(200);
  from.clear();

  double duration = 1.0e3 * (end_time.tv_sec - start_time.tv_sec) +
    double(end_time.tv_nsec)/1.0e6 - double(start_time.tv_nsec)/1.0e6;
  ROS_DEBUG("Controller replacement took %.3lf ms", duration);
}


MechanismControlNode::MechanismControlNode(MechanismControl *mc)
  : mc_(mc), cycles_since_publish_(0),
    mechanism_state_topic_("mechanism_state"),
    publisher_(mechanism_state_topic_, 1),
    transform_publisher_("tf_message", 5)
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
  publisher_.msg_.set_joint_states_size(mc_->model_.joints_.size());
  publisher_.msg_.set_actuator_states_size(mc_->hw_->actuators_.size());

  // Counts the number of transforms
  int num_transforms = 0;
  for (unsigned int i = 0; i < mc_->model_.links_.size(); ++i)
  {
    if (mc_->model_.links_[i]->parent_name_ != std::string("world"))
      ++num_transforms;
  }
  transform_publisher_.msg_.set_transforms_size(num_transforms);


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

  return true;
}

void MechanismControlNode::update()
{
  mc_->update();

  if (++cycles_since_publish_ >= CYCLES_PER_STATE_PUBLISH)
  {
    cycles_since_publish_ = 0;
    if (publisher_.trylock())
    {
      assert(mc_->model_.joints_.size() == publisher_.msg_.get_joint_states_size());
      for (unsigned int i = 0; i < mc_->model_.joints_.size(); ++i)
      {
        robot_msgs::JointState *out = &publisher_.msg_.joint_states[i];
        mechanism::JointState *in = &mc_->state_->joint_states_[i];
        out->name = mc_->model_.joints_[i]->name_;
        out->position = in->position_;
        out->velocity = in->velocity_;
        out->applied_effort = in->applied_effort_;
        out->commanded_effort = in->commanded_effort_;
        out->is_calibrated = in->calibrated_;
      }

      for (unsigned int i = 0; i < mc_->hw_->actuators_.size(); ++i)
      {
        robot_msgs::ActuatorState *out = &publisher_.msg_.actuator_states[i];
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
      publisher_.msg_.time = mc_->hw_->current_time_;

      publisher_.unlockAndPublish();
    }


    // Frame transforms
    if (transform_publisher_.trylock())
    {
      //assert(mc_->model_.links_.size() == transform_publisher_.msg_.get_quaternions_size());
      int ti = 0;
      for (unsigned int i = 0; i < mc_->model_.links_.size(); ++i)
      {
        if (mc_->model_.links_[i]->parent_name_ == std::string("world"))
          continue;

        tf::Vector3 pos = mc_->state_->link_states_[i].rel_frame_.getOrigin();
        tf::Quaternion quat = mc_->state_->link_states_[i].rel_frame_.getRotation();
        robot_msgs::TransformStamped &out = transform_publisher_.msg_.transforms[ti++];

        out.header.stamp.fromSec(mc_->hw_->current_time_);
        out.header.frame_id = mc_->model_.links_[i]->name_;
        out.parent_id = mc_->model_.links_[i]->parent_name_;
        out.transform.translation.x = pos.x();
        out.transform.translation.y = pos.y();
        out.transform.translation.z = pos.z();
        out.transform.rotation.w = quat.w();
        out.transform.rotation.x = quat.x();
        out.transform.rotation.y = quat.y();
        out.transform.rotation.z = quat.z();
      }

      transform_publisher_.unlockAndPublish();
    }
  }
}

bool MechanismControlNode::listControllerTypes(
  robot_srvs::ListControllerTypes::Request &req,
  robot_srvs::ListControllerTypes::Response &resp)
{
  (void) req;
  std::vector<std::string> types = controller::ControllerFactory::Instance().RegisteredIds();
  resp.set_types_vec(types);
  return true;
}

bool MechanismControlNode::spawnController(
  robot_srvs::SpawnController::Request &req,
  robot_srvs::SpawnController::Response &resp)
{
  TiXmlDocument doc;
  doc.Parse(req.xml_config.c_str());

  std::vector<uint8_t> oks;
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

  std::vector<MechanismControl::AddReq> add_reqs;
  std::vector<MechanismControl::RemoveReq> remove_reqs;

  for (; config; config = config->NextSiblingElement("controller"))
  {
    bool ok = true;

    if (!config->Attribute("type"))
    {
      ROS_ERROR("Could not spawn a controller because no type was given");
      ok = false;
    }
    else if (!config->Attribute("name"))
    {
      ROS_ERROR("Could not spawn a controller because no name was given");
      ok = false;
    }
    else
    {
      int last = add_reqs.size();
      add_reqs.resize(last + 1);
      add_reqs[last].type = config->Attribute("type");
      add_reqs[last].name = config->Attribute("name");
      add_reqs[last].config = config;
    }
  }

  mc_->changeControllers(remove_reqs, add_reqs);

  resp.ok.resize(add_reqs.size());
  resp.name.resize(add_reqs.size());
  for (size_t i = 0; i < add_reqs.size(); ++i)
  {
    resp.name[i] = add_reqs[i].name;
    resp.ok[i] = add_reqs[i].success;
  }

  return true;
}

bool MechanismControlNode::listControllers(
  robot_srvs::ListControllers::Request &req,
  robot_srvs::ListControllers::Response &resp)
{
  std::vector<std::string> controllers;

  (void) req;
  mc_->getControllerNames(controllers);
  resp.set_controllers_vec(controllers);
  return true;
}

bool MechanismControlNode::killController(
  robot_srvs::KillController::Request &req,
  robot_srvs::KillController::Response &resp)
{
  resp.ok = mc_->killController(req.name);
  return true;
}

bool MechanismControlNode::killAndSpawnControllers(
  robot_srvs::KillAndSpawnControllers::Request &req,
  robot_srvs::KillAndSpawnControllers::Response &resp)
{
  std::vector<MechanismControl::AddReq> add_reqs;
  std::vector<MechanismControl::RemoveReq> remove_reqs;

  TiXmlDocument doc;
  doc.Parse(req.add_config.c_str());

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

  for (; config; config = config->NextSiblingElement("controller"))
  {
    bool ok = true;

    if (!config->Attribute("type"))
    {
      ROS_ERROR("Could not spawn a controller because no type was given");
      ok = false;
    }
    else if (!config->Attribute("name"))
    {
      ROS_ERROR("Could not spawn a controller because no name was given");
      ok = false;
    }
    else
    {
      int last = add_reqs.size();
      add_reqs.resize(last + 1);
      add_reqs[last].type = config->Attribute("type");
      add_reqs[last].name = config->Attribute("name");
      add_reqs[last].config = config;
    }
  }

  remove_reqs.resize(req.kill_name.size());
  for (size_t i = 0; i < req.kill_name.size(); ++i)
    remove_reqs[i].name = req.kill_name[i];

  mc_->changeControllers(remove_reqs, add_reqs);

  resp.add_ok.resize(add_reqs.size());
  resp.add_name.resize(add_reqs.size());
  for (size_t i = 0; i < add_reqs.size(); ++i)
  {
    resp.add_name[i] = add_reqs[i].name;
    resp.add_ok[i] = add_reqs[i].success;
  }

  resp.kill_ok.resize(remove_reqs.size());
  for (size_t i = 0; i < remove_reqs.size(); ++i)
    resp.kill_ok[i] = remove_reqs[i].success;

  return true;
}

bool MechanismControlNode::switchController(
  robot_srvs::SwitchController::Request &req,
  robot_srvs::SwitchController::Response &resp)
{
  resp.ok = mc_->switchController(req.start_controllers, req.stop_controllers);
  return true;
}
