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
 * Author: Stuart Glaser
 */

#include "mechanism_control/mechanism_control.h"
#include "rosthread/member_thread.h"
#include "misc_utils/mutex_guard.h"

using namespace mechanism;

MechanismControl::MechanismControl(HardwareInterface *hw) :
  state_(NULL), hw_(hw), initialized_(0), please_remove_(-1), removed_(NULL)
{
  memset(controllers_, 0, MAX_NUM_CONTROLLERS * sizeof(void*));
  model_.hw_ = hw;
}

MechanismControl::~MechanismControl()
{
  // Destroy all controllers
  for (int i = 0; i < MAX_NUM_CONTROLLERS; ++i)
  {
    if (controllers_[i] != NULL)
      delete controllers_[i];
  }

  if (state_)
    delete state_;
}

bool MechanismControl::initXml(TiXmlElement* config)
{
  model_.initXml(config);
  state_ = new RobotState(&model_, hw_);

  initialized_ = true;
  return true;
}

controller::Controller* MechanismControl::getControllerByName(std::string name)
{
  for (int i = 0; i < MAX_NUM_CONTROLLERS; ++i)
    if (controller_names_[i] == name)
     return controllers_[i];

  return NULL;
}


// Must be realtime safe.
void MechanismControl::update()
{
  state_->propagateState();
  state_->zeroCommands();

  // Update all controllers
  for (int i = 0; i < MAX_NUM_CONTROLLERS; ++i)
  {
    if (controllers_[i] != NULL)
      controllers_[i]->update();
  }

  state_->enforceSafety();
  state_->propagateEffort();

  // If there's a controller to remove, we take it out of the controllers array.
  if (please_remove_ >= 0)
  {
    removed_ = controllers_[please_remove_];
    controllers_[please_remove_] = NULL;
    please_remove_ = -1;
  }
}

void MechanismControl::getControllerNames(std::vector<std::string> &controllers)
{
  controllers_lock_.lock();
  for (int i = 0; i < MAX_NUM_CONTROLLERS; i++)
  {
    if (controllers_[i] != NULL)
    {
      controllers.push_back(controller_names_[i]);
    }
  }
  controllers_lock_.unlock();
}

bool MechanismControl::addController(controller::Controller *c, const std::string &name)
{
  misc_utils::MutexGuard guard(&controllers_lock_);

  if (getControllerByName(name))
    return false;

  for (int i = 0; i < MAX_NUM_CONTROLLERS; i++)
  {
    if (controllers_[i] == NULL)
    {
      controllers_[i] = c;
      controller_names_[i] = name;
      return true;
    }
  }

  return false;
}


bool MechanismControl::spawnController(const std::string &type,
                                       const std::string &name,
                                       TiXmlElement *config)
{
  controller::Controller *c = controller::ControllerFactory::Instance().CreateObject(type);
  if (c == NULL)
    return false;
  printf("Spawning %s: %p\n", name.c_str(), &model_);


  if (!c->initXml(state_, config) ||
      !addController(c, name))
  {
    delete c;
    return false;
  }

  return true;
}

bool MechanismControl::killController(const std::string &name)
{
  bool found = false;
  controllers_lock_.lock();
  removed_ = NULL;
  for (int i = 0; i < MAX_NUM_CONTROLLERS; ++i)
  {
    if (controllers_[i] && controller_names_[i] == name)
    {
      found = true;
      please_remove_ = i;
      break;
    }
  }

  if (found)
  {
    while (removed_ == NULL)
      usleep(10000);

    delete removed_;
    removed_ = NULL;
  }

  controllers_lock_.unlock();
  return found;
}


MechanismControlNode::MechanismControlNode(MechanismControl *mc)
  : mc_(mc), cycles_since_publish_(0),
    mechanism_state_topic_("mechanism_state"),
    publisher_(mechanism_state_topic_, 1),
    transform_publisher_("TransformArray", 5)
{
  assert(mc != NULL);
  assert(mechanism_state_topic_);
  if ((node_ = ros::node::instance()) == NULL) {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv);
    node_ = new ros::node("mechanism_control", ros::node::DONT_HANDLE_SIGINT);
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
  transform_publisher_.msg_.set_quaternions_size(num_transforms);


  // Advertise services
  node_->advertise_service("list_controllers", &MechanismControlNode::listControllers, this);
  list_controllers_guard_.set("list_controllers");
  node_->advertise_service("list_controller_types", &MechanismControlNode::listControllerTypes, this);
  list_controller_types_guard_.set("list_controller_types");
  node_->advertise_service("spawn_controller", &MechanismControlNode::spawnController, this);
  spawn_controller_guard_.set("spawn_controller");
  node_->advertise_service("kill_controller", &MechanismControlNode::killController, this);
  kill_controller_guard_.set("kill_controller");

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
      transform_publisher_.msg_.header.stamp.fromSec(mc_->hw_->current_time_);
      int ti = 0;
      for (unsigned int i = 0; i < mc_->model_.links_.size(); ++i)
      {
        if (mc_->model_.links_[i]->parent_name_ == std::string("world"))
          continue;

        tf::Vector3 pos = mc_->state_->link_states_[i].rel_frame_.getOrigin();
        tf::Quaternion quat = mc_->state_->link_states_[i].rel_frame_.getRotation();
        rosTF::TransformQuaternion &out = transform_publisher_.msg_.quaternions[ti++];

        out.header.stamp.fromSec(mc_->hw_->current_time_);
        out.header.frame_id = mc_->model_.links_[i]->name_;
        out.parent = mc_->model_.links_[i]->parent_name_;
        out.xt = pos.x();
        out.yt = pos.y();
        out.zt = pos.z();
        out.w = quat.w();
        out.xr = quat.x();
        out.yr = quat.y();
        out.zr = quat.z();
      }

      transform_publisher_.unlockAndPublish();
    }
  }
}

bool MechanismControlNode::listControllerTypes(
  robot_srvs::ListControllerTypes::request &req,
  robot_srvs::ListControllerTypes::response &resp)
{
  (void) req;
  std::vector<std::string> types = controller::ControllerFactory::Instance().RegisteredIds();
  resp.set_types_vec(types);
  return true;
}

bool MechanismControlNode::spawnController(
  robot_srvs::SpawnController::request &req,
  robot_srvs::SpawnController::response &resp)
{
  TiXmlDocument doc;
  doc.Parse(req.xml_config.c_str());

  std::vector<uint8_t> oks;
  std::vector<std::string> names;

  TiXmlElement *config = doc.RootElement();
  if (!config)
    return false;
  if (config->ValueStr() != "controllers" &&
      config->ValueStr() != "controller")
    return false;

  if (config->ValueStr() == "controllers")
  {
    config = config->FirstChildElement("controller");
  }

  for (; config; config = config->NextSiblingElement("controller"))
  {
    bool ok = true;

    if (!config->Attribute("type"))
      ok = false;
    else if (!config->Attribute("name"))
      ok = false;
    else
    {
      ok = mc_->spawnController(config->Attribute("type"), config->Attribute("name"), config);
    }

    oks.push_back(ok);
    names.push_back(config->Attribute("name") ? config->Attribute("name") : "");
  }

  resp.set_ok_vec(oks);
  resp.set_name_vec(names);

  return true;
}

bool MechanismControlNode::listControllers(
  robot_srvs::ListControllers::request &req,
  robot_srvs::ListControllers::response &resp)
{
  std::vector<std::string> controllers;

  (void) req;
  mc_->getControllerNames(controllers);
  resp.set_controllers_vec(controllers);
  return true;
}

bool MechanismControlNode::killController(
  robot_srvs::KillController::request &req,
  robot_srvs::KillController::response &resp)
{
  resp.ok = mc_->killController(req.name);
  return true;
}
