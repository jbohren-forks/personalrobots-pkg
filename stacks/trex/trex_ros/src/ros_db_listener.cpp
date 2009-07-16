
#include "trex_ros/ros_db_listener.h"

#include "Timeline.hh"
#include "Token.hh"
#include "Domains.hh"

#include <string>

using namespace EUROPA;

namespace trex_ros {
  ROSDbListener::ROSDbListener() {

   // plan_pub_ = node_handle_.advertise<trex_ros::PlanDescription>(TeleoReactor::getName().toString()+"/plan",10);
  }

  ROSDbListener::~ROSDbListener() {
  }

  void ROSDbListener::notifyAdded(const ObjectId& object, const TokenId& token) {
    ROS_INFO("ADDED TOKEN (%d) \"%s\" TO OBJECT \"%s\"",
	token->getKey(),
	token->getName().c_str(),
	object->getName().c_str());
  }

  void ROSDbListener::notifyMerged(const TokenId& token) {
  }

  void ROSDbListener::notifyActivated(const TokenId& token) {

  }

  void ROSDbListener::notifyRemoved(const TokenId& token) {
    ROS_INFO("REMOVED TOKEN (%d) \"%s\"",token->getKey(),token->getName().c_str());
  }

  void ROSDbListener::notifyCommitted(const TokenId& token) {
  }

  void ROSDbListener::notifyDeactivated(const TokenId& token) {
  }

  void ROSDbListener::notifySplit(const TokenId& token) {
  }

  void ROSDbListener::notifyRejected(const TokenId& token) {
  }

  void ROSDbListener::notifyTerminated(const TokenId& token) {
  }
}
