#ifndef H_MasterReactor
#define H_MasterReactor

#include "trex_ros/ros_reactor.h"
#include "trex_temp_proj/ExecuteGoals.h"

namespace trex_temp_proj {

  class MasterReactor : public trex_ros::ROSReactor {
  public:
    MasterReactor(const EUROPA::LabelStr& agentName, const TiXmlElement& configData);

    virtual ~MasterReactor();

    bool executeGoals(ExecuteGoals::Request &req, ExecuteGoals::Response &resp);

  private:
    ros::NodeHandle node_handle_;
    ros::ServiceServer service_server_;
  };
}

#endif // ifndef H_MasterReactor
