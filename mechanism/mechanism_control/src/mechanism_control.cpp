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

using namespace mechanism;

MechanismControl::MechanismControl(HardwareInterface *hw) :
  model_((char*)"robot"), initialized_(0), hw_(hw)
{
  memset(controllers_, 0, MAX_NUM_CONTROLLERS * sizeof(void*));
  model_.hw_ = hw;
}

MechanismControl::~MechanismControl()
{
}

bool MechanismControl::registerActuator(const std::string &name, int index)
{
  if (initialized_)
    return false;

  model_.actuators_lookup_.insert(Robot::IndexMap::value_type(name, index));

  return true;
}

bool MechanismControl::initXml(TiXmlElement* config)
{
  bool successful = true;
  ros::node *node = ros::node::instance();

  TiXmlElement *elt;

  // Construct the joints
  for (elt = config->FirstChildElement("joint"); elt; elt = elt->NextSiblingElement("joint"))
  {
    Joint *j = new Joint;
    model_.joints_.push_back(j);
    model_.joints_lookup_.insert(Robot::IndexMap::value_type(elt->Attribute("name"), model_.joints_.size() - 1));
    j->initXml(elt);
  }

  // Construct the transmissions
  elt = config->FirstChildElement("transmission");
  for (; elt; elt = elt->NextSiblingElement("transmission"))
  {
    const char *type = elt->Attribute("type");
    Transmission *t = TransmissionFactory::instance().create(type);
    if (t == NULL)
      node->log(ros::FATAL, "Unknown transmission type: %s\n", type);
    t->initXml(elt, &model_);
    model_.transmissions_.push_back(t);
  }

  initialized_ = true;
  return successful;
}

// Must be realtime safe.
void MechanismControl::update()
{
  // Propagates through the robot model.
  for (unsigned int i = 0; i < model_.transmissions_.size(); ++i)
    model_.transmissions_[i]->propagatePosition();

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
}

void MechanismControl::registerControllerType(const std::string& type, ControllerAllocator f)
{
  controller::ControllerFactory::instance().registerType(type, f);
}

bool MechanismControl::addController(controller::Controller *c, const std::string &name)
{
  //Add controller to list of controllers in realtime-safe manner;
  controllers_mutex_.lock(); //This lock is only to prevent us from other non-realtime threads.  The realtime thread may be spinning through the list of controllers while we are in here, so we need to keep that list always in a valid state.  This is why we fully allocate and set up the controller before adding it into the list of active controllers.
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
  controllers_mutex_.unlock();

  if (!spot_found)
  {
    delete c;
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
  c->initXml(&model_, config);

  return addController(c, name);
}



MechanismControlNode::MechanismControlNode(MechanismControl *mc)
  : ros::node("MechanismControl"), mc_(mc)
{
  assert(mc != NULL);
  advertise_service("list_controller_types", &MechanismControlNode::listControllerTypes);
  advertise_service("spawn_controller", &MechanismControlNode::spawnController);
}


bool MechanismControlNode::listControllerTypes(
  mechanism_control::ListControllerTypes::request &req,
  mechanism_control::ListControllerTypes::response &resp)
{
  std::vector<std::string> types;
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
  resp.ok = mc_->spawnController(req.type, req.name, doc.RootElement());
  return true;
}
