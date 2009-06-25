#ifndef H_ROSReactor
#define H_ROSReactor

#include "ros/ros.h"
#include "ros/node.h"
#include "executive_trex_pr2/executive.h"
#include "trex_ros/ExecuteGoals.h"

#include "DbCore.hh"

using namespace TREX;
using namespace EUROPA;
using namespace trex_ros;
namespace executive_trex_pr2 {

  /**
   * @brief Extends a deliberative reactor to enable it to be taskable over ROS
   */
  class ROSReactor: public DbCore {
  public:
    ROSReactor(const LabelStr& agentName, const TiXmlElement& configData);

    virtual ~ROSReactor();

    bool executeGoals(ExecuteGoals::Request &req, ExecuteGoals::Response &resp);

  protected:

    /**
     * @brief Used to hook up observer for dispatch of observations and servers for dispatch of goals
     */
    virtual void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer);

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

  private:
    ros::NodeHandle node_handle_;
    ros::ServiceServer service_server_;
    boost::recursive_mutex lock_;
  };
}
#endif
