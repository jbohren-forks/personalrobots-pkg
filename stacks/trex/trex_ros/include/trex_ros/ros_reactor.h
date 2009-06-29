/** ros_reactor.h
 * ROSReactor is a TREX DeliberaiveReactor that can broadcast it's plan over the
 * ROS messaging system.
 */

#ifndef H_ROSReactor
#define H_ROSReactor

#include "ros/ros.h"
#include "ros/node.h"

#include "trex_ros/executive.h"
#include "trex_ros/PlanDescription.h"

#include "DbCore.hh"

namespace trex_ros {

  /**
   * @brief Extends a deliberative reactor to enable it to be taskable over ROS
   */
  class ROSReactor: public TREX::DbCore {
  public:
    ROSReactor(const EUROPA::LabelStr& agentName, const TiXmlElement& configData);

    virtual ~ROSReactor();

  protected:

    /**
     * @brief Used to hook up observer for dispatch of observations and servers for dispatch of goals
     */
    virtual void handleInit(TREX::TICK initialTick, const std::map<double, TREX::ServerId>& serversByTimeline, const TREX::ObserverId& observer);

    /**
     * @brief Must adress new tick and conduct appropriate propagation and dispatch
     */
    virtual void handleTickStart();

    /**
     * @brief Handle synchronization. If it fails it is a system error.
     */
    virtual bool synchronize();

    /**
     * @brief Return true if there are flaws to resolve
     */
    virtual bool hasWork();

    /**
     * @brief Step the reactor to resolve flaws
     */
    virtual void resume();

  protected:

    /**
     * @brief Allows derived classes to lock the mutex during callbacks
     */
    void lock();

    /**
     * @brief Allows derived classes to unlock the mutex during callbacks
     */
    void unlock();

  private:
    ros::NodeHandle node_handle_;
    ros::Publisher plan_pub_;
    boost::recursive_mutex lock_;

    /**
     * @brief Populate a TimelineDescription message
     */
    void fillTimelineDescriptionMsg(
	const DbCore::PlanDescription::TimelineDescription tlDesc,
	trex_ros::TimelineDescription &tlDescMsg);

    /**
     * @brief Plublish the plan over ROS
     */
    void publishPlan();
  };
}
#endif
