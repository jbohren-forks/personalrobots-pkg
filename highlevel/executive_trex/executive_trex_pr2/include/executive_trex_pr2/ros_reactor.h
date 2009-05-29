#ifndef H_ROSReactor
#define H_ROSReactor

#include "ros/ros.h"
#include "ros/node.h"
#include "executive_trex_pr2/executive.h"
#include "executive_trex_pr2/ExecuteGoals.h"

#include "DbCore.hh"

using namespace TREX;
using namespace EUROPA;
namespace executive_trex_pr2 {

  /**
   * @brief Extends a deliberative reactor to enable it to be taskable over ROS
   */
  class ROSReactor: public DbCore {
  public:
    ROSReactor(const LabelStr& agentName, const TiXmlElement& configData);

    virtual ~ROSReactor();

    bool executeGoals(ExecuteGoals::Request &req, ExecuteGoals::Response &resp);

  private:
    ros::NodeHandle node_handle_;
    ros::ServiceServer service_server_;
  };
}
#endif
