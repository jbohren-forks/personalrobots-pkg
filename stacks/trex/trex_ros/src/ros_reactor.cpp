#include "trex_ros/ros_reactor.h"
#include "trex_ros/PlanDescription.h"

#include "Agent.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Observer.hh"
#include "Token.hh"
#include "Domains.hh"
#include "ConstrainedVariable.hh"

using namespace TREX;
using namespace EUROPA;

namespace trex_ros{ 	

  /**
   * ROS Reactors will always log, to support playback. This is achieved by setting parameters in the
   * base class constructor
   */
  ROSReactor::ROSReactor(const LabelStr& agentName, const TiXmlElement& configData) :
    DbCore(agentName, configData)
  {
    // Advertise publisher to broadcast plan 
    // We want to use a publisher instead of a service call so that the plans are all sent out on the same tick.
    plan_pub_ = node_handle_.advertise<trex_ros::PlanDescription>(TeleoReactor::getName().toString()+"/plan",10);
  }

  ROSReactor::~ROSReactor() {}

  void ROSReactor::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
    lock();
    DbCore::handleInit(initialTick, serversByTimeline, observer);
    unlock();
  }

  void ROSReactor::handleTickStart(){
    lock();
    DbCore::handleTickStart();
    publishPlan();
    unlock();
  }

  bool ROSReactor::synchronize(){
    bool result(false);
    lock();
    result = DbCore::synchronize();
    unlock();
    return result;
  }

  bool ROSReactor::hasWork(){
    bool result(false);
    lock();
    result = DbCore::hasWork();
    unlock();
    return result;
  }

  void ROSReactor::resume(){
    lock();
    DbCore::resume();
    unlock();
  }

  void ROSReactor::lock() {
    lock_.lock();
  }

  void ROSReactor::unlock() {
    lock_.unlock();
  }

  void ROSReactor::fillTimelineDescriptionMsg(const DbCore::PlanDescription::TimelineDescription tlDesc, trex_ros::TimelineDescription &tlDescMsg) {
    // Get the timeline name
    tlDescMsg.name = tlDesc.name.toString();

    // Iterate over tokens
    std::vector<DbCore::PlanDescription::TokenDescription>::const_iterator
      tokit = tlDesc.tokens.begin();
    std::vector<DbCore::PlanDescription::TokenDescription>::const_iterator const
      endtok = tlDesc.tokens.end();

    for( ;endtok!=tokit; ++tokit ) {
      DbCore::PlanDescription::TokenDescription const &tokDesc = *tokit;
      trex_ros::TokenDescription tokDescMsg;

      tokDescMsg.key = tokDesc.key;

      tokDescMsg.name = tokDesc.name.toString();

      tokDescMsg.start[0] = tokDesc.start[0];
      tokDescMsg.start[1] = tokDesc.start[1];
      tokDescMsg.end[0] = tokDesc.end[0];
      tokDescMsg.end[1] = tokDesc.end[1];

      // Add token to timeline description
      tlDescMsg.tokens.push_back(tokDescMsg);
    }
  }

  void ROSReactor::publishPlan(){
    DbCore::PlanDescription planDesc;
    trex_ros::PlanDescription planDescMsg;

    // Get a TREX::DbCore::PlanDescription
    DbCore::getPlanDescription(planDesc);

    // Copy name, tick
    planDescMsg.tick = planDesc.m_tick;
    planDescMsg.name = planDesc.m_reactorName.toString();

    // Internals 
    std::vector<DbCore::PlanDescription::TimelineDescription>::const_iterator
      intit = planDesc.m_internalTimelines.begin();
    std::vector<DbCore::PlanDescription::TimelineDescription>::const_iterator const
      endint = planDesc.m_internalTimelines.end();
    
    // Add timeline description to plan description
    for( ; endint!=intit; ++intit ) {
      trex_ros::TimelineDescription tlDescMsg;
      fillTimelineDescriptionMsg(*intit, tlDescMsg);
      planDescMsg.internal.push_back(tlDescMsg);
    }
    
    // Externals 
    std::vector<DbCore::PlanDescription::TimelineDescription>::const_iterator
      extit = planDesc.m_externalTimelines.begin();
    std::vector<DbCore::PlanDescription::TimelineDescription>::const_iterator const
      endext = planDesc.m_externalTimelines.end();
    
    // Add timeline description to plan description
    for( ; endext!=extit; ++extit ) {
      trex_ros::TimelineDescription tlDescMsg;
      fillTimelineDescriptionMsg(*extit, tlDescMsg);
      planDescMsg.external.push_back(tlDescMsg);
    }

    // Publish PlanDescription
    plan_pub_.publish(planDescMsg);
  }

}
