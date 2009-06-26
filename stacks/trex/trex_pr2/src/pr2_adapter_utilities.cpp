#include <trex_pr2/pr2_adapter_utilities.h>
#include <trex_ros/adapter_utilities.h>
#include <tf/transform_listener.h>

using namespace trex_ros;

namespace trex_pr2 {
  void Pr2AdapterUtilities::read(ObservationByValue& obs, const plugs_msgs::PlugStow& msg){
    setHeader(msg, obs);
    AdapterUtilities::read<bool>("stowed", obs, msg.stowed);
    AdapterUtilities::read<double>("x", obs, msg.plug_centroid.x);
    AdapterUtilities::read<double>("y", obs, msg.plug_centroid.y);
    AdapterUtilities::read<double>("z", obs, msg.plug_centroid.z);
  }

  void Pr2AdapterUtilities::write(const TokenId& token, plugs_msgs::PlugStow& msg){
    getHeader(msg, token);

    AdapterUtilities::write<int8_t>("stowed", token, msg.stowed);
    AdapterUtilities::write<double>("x", token, msg.plug_centroid.x);
    AdapterUtilities::write<double>("y", token, msg.plug_centroid.y);
    AdapterUtilities::write<double>("z", token, msg.plug_centroid.z);
  }
  
}
