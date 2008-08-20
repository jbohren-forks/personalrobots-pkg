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

#include "mechanism_control/mechanism_control.h"
#include "generic_controllers/controller.h"
#include "rosthread/member_thread.h"

using namespace mechanism;

MechanismControl::MechanismControl(HardwareInterface *hw) :
  model_((char*)"robot"), hw_(hw), initialized_(0)
{
  memset(controllers_, 0, MAX_NUM_CONTROLLERS * sizeof(void*));
  memset(garbage_, 0, GARBAGE_SIZE * sizeof(void*));
  model_.hw_ = hw;
}

MechanismControl::~MechanismControl()
{
}

bool MechanismControl::initXml(TiXmlElement* config)
{
  model_.initXml(config);

  initialized_ = true;
  return true;
}

bool MechanismControl::addJoint(Joint* j)
{
  model_.joints_.push_back(j);
  return true;
}

bool MechanismControl::addSimpleTransmission(SimpleTransmission *st)
{
  bool successful = true;
  ros::node *node = ros::node::instance();

  // Construct the transmissions
  const char *type = "SimpleTransmission";
  Transmission *t = TransmissionFactory::instance().create(type);
  if (t == NULL)
    node->log(ros::FATAL, "Unknown transmission type: %s\n", type);

  t->initTransmission(st->name_,st->joint_name_,st->actuator_name_,st->mechanical_reduction_,st->motor_torque_constant_,st->pulses_per_revolution_, &model_);

  model_.transmissions_.push_back(t);

  return successful;
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
  // Propagates through the robot model.
  for (unsigned int i = 0; i < model_.transmissions_.size(); ++i)
    model_.transmissions_[i]->propagatePosition();

  //zero all commands
  for (unsigned int i = 0; i < model_.joints_.size(); ++i)
    model_.joints_[i]->commanded_effort_= 0.0;

  // TODO: update KDL model with new joint position/velocities

  // Update all controllers
  for (int i = 0; i < MAX_NUM_CONTROLLERS; ++i)
  {
    if (controllers_[i] != NULL)
      controllers_[i]->update();
  }

  // Performs safety checks on the commands.
  for (unsigned int i = 0; i < model_.joints_.size(); ++i)
    model_.joints_[i]->enforceLimits();

  // Propagates commands back into the actuators.
  for (unsigned int i = 0; i < model_.transmissions_.size(); ++i)
    model_.transmissions_[i]->propagateEffort();

  // Cleanup.  Deletes any controllers in the garbage.
  for (int i = 0; i < GARBAGE_SIZE; ++i)
  {
    if (garbage_[i])
    {
      delete garbage_[i];
      garbage_[i] = NULL;
    }
  }
}

void MechanismControl::registerControllerType(const std::string& type, ControllerAllocator f)
{
  controller::ControllerFactory::instance().registerType(type, f);
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
  //Add controller to list of controllers in realtime-safe manner;
  controllers_lock_.lock(); //This lock is only to prevent us from other non-realtime threads.  The realtime thread may be spinning through the list of controllers while we are in here, so we need to keep that list always in a valid state.  This is why we fully allocate and set up the controller before adding it into the list of active controllers.

  bool spot_found = false;
  for (int i = 0; i < MAX_NUM_CONTROLLERS; i++)
  {
    if (controllers_[i] == NULL)
    {
      spot_found = true;
      controllers_[i] = c;
      controller_names_[i] = name;
      break;
    }
  }
  controllers_lock_.unlock();

  if (!spot_found)
  {
    return false;
  }

  return true;
}


bool MechanismControl::spawnController(const std::string &type,
                                       const std::string &name,
                                       TiXmlElement *config)
{
  controller::Controller *c = controller::ControllerFactory::instance().create(type);
  if (c == NULL)
    return false;
  printf("Spawning %s: %08x\n", name.c_str(), (int)&model_);


  if (!c->initXml(&model_, config) ||
      !addController(c, name))
  {
    delete c;
    return false;
  }

  return true;
}

bool MechanismControl::killController(const std::string &name)
{
  bool success = false;
  controllers_lock_.lock();
  for (int i = 0; i < MAX_NUM_CONTROLLERS; ++i)
  {
    if (controllers_[i] && controller_names_[i] == name)
    {
      // Moves the controller into the garbage.
      for (int j = 0; j < GARBAGE_SIZE; ++j)
      {
        if (NULL == garbage_[j])
        {
          garbage_[j] = controllers_[i];
          controllers_[i] = NULL;
          success = true;
          break;
        }
      }

      break;
    }
  }
  controllers_lock_.unlock();
  return success;
}


MechanismControlNode::MechanismControlNode(MechanismControl *mc)
  : mc_(mc), mechanism_state_topic_("mechanism_state")
{
  assert(mc != NULL);
  assert(mechanism_state_topic_);
  if ((node_ = ros::node::instance()) == NULL) {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv);
    node_ = new ros::node("mechanism_control", ros::node::DONT_HANDLE_SIGINT);
  }

  // Advertise services
  node_->advertise_service("list_controllers", &MechanismControlNode::listControllers, this);
  node_->advertise_service("list_controller_types", &MechanismControlNode::listControllerTypes, this);
  node_->advertise_service("spawn_controller", &MechanismControlNode::spawnController, this);
  node_->advertise_service("kill_controller", &MechanismControlNode::killController, this);

  // Advertise topics
  node_->advertise<mechanism_control::MechanismState>(mechanism_state_topic_, 1000);

  // Launches the worker state_publishing_loop_keep_running_
  pthread_cond_init (&mechanism_state_updated_cond_, NULL);
  pthread_mutex_init(&mechanism_state_lock_,NULL);
  state_publishing_loop_keep_running_ = true;
  state_publishing_thread_ = ros::thread::member_thread::startMemberFunctionThread<MechanismControlNode>(this, &MechanismControlNode::statePublishingLoop);
}

MechanismControlNode::~MechanismControlNode()
{
  state_publishing_loop_keep_running_ = false;
}

bool MechanismControlNode::initXml(TiXmlElement *config)
{
  if (!mc_->initXml(config))
    return false;
  mechanism_state_.set_joint_states_size(mc_->model_.joints_.size());
  mechanism_state_.set_actuator_states_size(mc_->hw_->actuators_.size());
  return true;
}

void MechanismControlNode::update()
{
  mc_->update();

  // Only update on every 100th call to update()
  static int count = 0;
  if (count++ % 100 != 0) return;

  // Attempts to lock the transfer structure
  // If we get it, update it
  if (!pthread_mutex_trylock(&mechanism_state_lock_))
  {
    // We should not have to resize
    assert(mc_->model_.joints_.size() == mechanism_state_.get_joint_states_size());

    for (unsigned int i = 0; i < mc_->model_.joints_.size(); ++i)
    {
      mechanism_control::JointState *out = mechanism_state_.joint_states + i;
      Joint *in = mc_->model_.joints_[i];
      out->name = in->name_;
      out->position = in->position_;
      out->velocity = in->velocity_;
      out->applied_effort = in->applied_effort_;
      out->commanded_effort = in->commanded_effort_;
    }

    for (unsigned int i = 0; i < mc_->hw_->actuators_.size(); ++i)
    {
      mechanism_control::ActuatorState *out = mechanism_state_.actuator_states + i;
      ActuatorState *in = &mc_->hw_->actuators_[i]->state_;
      out->name = mc_->hw_->actuators_[i]->name_;
      out->encoder_count = in->encoder_count_;
      out->timestamp = in->timestamp_;
      out->encoder_velocity = in->encoder_velocity_;
      out->calibration_reading = in->calibration_reading_;
      out->last_calibration_high_transition = in->last_calibration_high_transition_;
      out->last_calibration_low_transition = in->last_calibration_low_transition_;
      out->is_enabled = in->is_enabled_;
      out->run_stop_hit = in->run_stop_hit_;
      out->last_requested_current = in->last_requested_current_;
      out->last_commanded_current = in->last_commanded_current_;
      out->last_measured_current = in->last_measured_current_;
      out->motor_voltage = in->motor_voltage_;
      out->num_encoder_errors = in->num_encoder_errors_;
    }
    mechanism_state_.time = mc_->hw_->current_time_;
    pthread_cond_signal(&mechanism_state_updated_cond_);
    pthread_mutex_unlock(&mechanism_state_lock_);
  }
}

bool MechanismControlNode::listControllerTypes(
  mechanism_control::ListControllerTypes::request &req,
  mechanism_control::ListControllerTypes::response &resp)
{
  std::vector<std::string> types;

  (void) req;
  controller::ControllerFactory::instance().getTypes(&types);
  resp.set_types_vec(types);
  return true;
}

bool MechanismControlNode::spawnController(
  mechanism_control::SpawnController::request &req,
  mechanism_control::SpawnController::response &resp)
{
  TiXmlDocument doc;
  doc.Parse(req.xml_config.c_str());

  TiXmlElement *config = doc.RootElement();
  if (!config->Attribute("type"))
    resp.ok = false;
  else if (!config->Attribute("name"))
    resp.ok = false;
  else
    resp.ok = mc_->spawnController(config->Attribute("type"), config->Attribute("name"), config);
  return true;
}

bool MechanismControlNode::listControllers(
  mechanism_control::ListControllers::request &req,
  mechanism_control::ListControllers::response &resp)
{
  std::vector<std::string> controllers;

  (void) req;
  mc_->getControllerNames(controllers);
  resp.set_controllers_vec(controllers);
  return true;
}

void MechanismControlNode::publishMechanismState()
{
  assert(this->mechanism_state_topic_);
  // Waits for RT thread to release the mutex, signaling new data has arrived.
  pthread_mutex_lock(&mechanism_state_lock_);
  pthread_cond_wait(&mechanism_state_updated_cond_, &mechanism_state_lock_);
  assert(this->mechanism_state_topic_);
  node_->publish(mechanism_state_topic_, mechanism_state_);
  pthread_mutex_unlock(&mechanism_state_lock_);
}

void MechanismControlNode::statePublishingLoop()
{
  while(state_publishing_loop_keep_running_)
  {
    assert(mechanism_state_topic_);
    publishMechanismState();
  }
}


bool MechanismControlNode::killController(
  mechanism_control::KillController::request &req,
  mechanism_control::KillController::response &resp)
{
  resp.ok = mc_->killController(req.name);
  return true;
}
