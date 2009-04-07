///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008-2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
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
#ifndef MECHANISM_CONTROL_H
#define MECHANISM_CONTROL_H

#include <pthread.h>
#include <map>
#include <string>
#include <vector>
#include "ros/node.h"
#include "roslib/Time.h"
#include <tinyxml/tinyxml.h>
#include <hardware_interface/hardware_interface.h>
#include <mechanism_model/robot.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <mechanism_model/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <misc_utils/advertised_service_guard.h>

#include <robot_srvs/ListControllerTypes.h>
#include <robot_srvs/ListControllers.h>
#include <robot_srvs/SpawnController.h>
#include <robot_srvs/KillController.h>
#include <robot_srvs/KillAndSpawnControllers.h>
#include <robot_srvs/SwitchController.h>
#include <robot_msgs/MechanismState.h>
#include <robot_msgs/DiagnosticMessage.h>
#include "tf/tfMessage.h"

typedef controller::Controller* (*ControllerAllocator)();


class MechanismControl {

public:
  // Give access to mechanism control
  static bool Instance(MechanismControl*& mech);

  MechanismControl(HardwareInterface *hw);
  virtual ~MechanismControl();

  // Real-time functions
  void update();

  // Non real-time functions
  bool initXml(TiXmlElement* config);
  void getControllerNames(std::vector<std::string> &v);
  bool spawnController(const std::string &type, const std::string &name, TiXmlElement *config);
  bool killController(const std::string &name);
  bool switchController(const std::vector<std::string>& start_controllers,
                        const std::vector<std::string>& stop_controllers);

  // controllers_lock_ must be locked before calling
  controller::Controller* getControllerByName(const std::string& name);
  template<class ControllerType> bool getControllerByName(const std::string& name, ControllerType*& c)
  {
    // get controller
    controller::Controller* controller = getControllerByName(name);
    if (controller == NULL) return false;
    
    // cast controller to ControllerType
    ControllerType* controller_type = dynamic_cast< ControllerType* >(controller);
    if (controller_type == NULL)  return false;
  
    // copy result
    c = controller_type;
    return true;
  };

  struct AddReq
  {
    std::string name;
    std::string type;
    TiXmlElement *config;
    bool success;  // Response
  };
  struct RemoveReq
  {
    std::string name;
    bool success;  // Response
  };
  void changeControllers(std::vector<RemoveReq> &remove_reqs,
                         std::vector<AddReq> &add_reqs,
                         const int strictness = 0);

  mechanism::Robot model_;
  mechanism::RobotState *state_;
  HardwareInterface *hw_;

private:
  static MechanismControl* mechanism_control_;

  bool initialized_, switch_success_;

  typedef boost::accumulators::accumulator_set<
    double, boost::accumulators::stats<boost::accumulators::tag::max,
                                       boost::accumulators::tag::mean,
                                       boost::accumulators::tag::variance> > TimeStatistics;
  struct Diagnostics {
    TimeStatistics acc;
    double max;
    boost::circular_buffer<double> max1;

    Diagnostics() : max(0), max1(60) {}
  };

  enum {EMPTY, INITIALIZED = 1, RUNNING};
  struct ControllerSpec {
    std::string name;
    boost::shared_ptr<controller::Controller> c;
    int state;
    boost::shared_ptr<Diagnostics> diagnostics;

    ControllerSpec() : state(EMPTY), diagnostics(new Diagnostics) {}
    ControllerSpec(const ControllerSpec &spec)
      : name(spec.name), c(spec.c), state(spec.state), diagnostics(spec.diagnostics) {}
  };

  boost::mutex controllers_lock_;
  std::vector<ControllerSpec> controllers_lists_[2];
  int current_controllers_list_, used_by_realtime_;

  Diagnostics diagnostics_;
  realtime_tools::RealtimePublisher<robot_msgs::DiagnosticMessage> publisher_;
  void publishDiagnostics();
  std::vector<controller::Controller*> start_request_, stop_request_;
  bool please_switch_;

  int loop_count_;
};

/*
 * Exposes MechanismControl's interface over ROS
 */
class MechanismControlNode
{
public:
  MechanismControlNode(MechanismControl *mc);
  virtual ~MechanismControlNode();

  bool initXml(TiXmlElement *config);

  void update();  // Must be realtime safe

  bool listControllerTypes(robot_srvs::ListControllerTypes::Request &req,
                           robot_srvs::ListControllerTypes::Response &resp);
  bool listControllers(robot_srvs::ListControllers::Request &req,
                       robot_srvs::ListControllers::Response &resp);
  bool spawnController(robot_srvs::SpawnController::Request &req,
                       robot_srvs::SpawnController::Response &resp);
  bool killAndSpawnControllers(robot_srvs::KillAndSpawnControllers::Request &req,
                               robot_srvs::KillAndSpawnControllers::Response &resp);
  bool switchController(robot_srvs::SwitchController::Request &req,
                        robot_srvs::SwitchController::Response &resp);

private:
  ros::Node *node_;

  bool killController(robot_srvs::KillController::Request &req,
                      robot_srvs::KillController::Response &resp);

  bool getController(const std::string& name, int& id);

  MechanismControl *mc_;

  //static const double STATE_PUBLISHING_PERIOD = 0.01;  // this translates to about 100Hz
  static const int CYCLES_PER_STATE_PUBLISH = 10;  // 100 Hz
  int cycles_since_publish_;

  const char* const mechanism_state_topic_;
  realtime_tools::RealtimePublisher<robot_msgs::MechanismState> publisher_;

  realtime_tools::RealtimePublisher<tf::tfMessage> transform_publisher_;

  AdvertisedServiceGuard list_controllers_guard_, list_controller_types_guard_,
    spawn_controller_guard_, kill_controller_guard_, switch_controller_guard_,
    kill_and_spawn_controllers_guard_;
};

#endif /* MECHANISM_CONTROL_H */
