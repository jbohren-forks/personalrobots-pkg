#include <trex_pr2/pr2_adapter_utilities.h>
#include <trex_ros/adapter_utilities.h>
#include <tf/transform_listener.h>

using namespace trex_ros;

namespace trex_pr2 {
  /**
   * Write token to the message
   */
  void Pr2AdapterUtilities::write(const TokenId& token, door_msgs::Door& msg){
    getHeader(msg, token);

    // Latch state
    AdapterUtilities::write<int32_t>("latch_state", token, msg.latch_state);

    // Frame Data
    AdapterUtilities::write<float>("frame_p1_x", token, msg.frame_p1.x);
    AdapterUtilities::write<float>("frame_p1_y", token, msg.frame_p1.y);
    AdapterUtilities::write<float>("frame_p1_z", token, msg.frame_p1.z);
    AdapterUtilities::write<float>("frame_p2_x", token, msg.frame_p2.x);
    AdapterUtilities::write<float>("frame_p2_y", token, msg.frame_p2.y);
    AdapterUtilities::write<float>("frame_p2_z", token, msg.frame_p2.z);
    AdapterUtilities::write<float>("height", token, msg.height);
    AdapterUtilities::write<int32_t>("hinge", token, msg.hinge);
    AdapterUtilities::write<int32_t>("rot_dir", token, msg.rot_dir);

    // Door Data
    AdapterUtilities::write<float>("door_p1_x", token, msg.door_p1.x);
    AdapterUtilities::write<float>("door_p1_y", token, msg.door_p1.y);
    AdapterUtilities::write<float>("door_p1_z", token, msg.door_p1.z);
    AdapterUtilities::write<float>("door_p2_x", token, msg.door_p2.x);
    AdapterUtilities::write<float>("door_p2_y", token, msg.door_p2.y);
    AdapterUtilities::write<float>("door_p2_z", token, msg.door_p2.z);

    // Handle Data
    AdapterUtilities::write<float>("handle_x", token, msg.handle.x);
    AdapterUtilities::write<float>("handle_y", token, msg.handle.y);
    AdapterUtilities::write<float>("handle_z", token, msg.handle.z);

    // Travel Dir
    AdapterUtilities::write<double>("travel_dir_x", token, msg.travel_dir.x);
    AdapterUtilities::write<double>("travel_dir_y", token, msg.travel_dir.y);
    AdapterUtilities::write<double>("travel_dir_z", token, msg.travel_dir.z);
  }

  // Read Observation from Door Message
  void Pr2AdapterUtilities::read(ObservationByValue& obs, const door_msgs::Door& msg){
    setHeader(msg, obs);

    // Latch state
    AdapterUtilities::read<int32_t>("latch_state", obs, msg.latch_state);

    // Frame Data
    AdapterUtilities::read<float>("frame_p1_x", obs, msg.frame_p1.x);
    AdapterUtilities::read<float>("frame_p1_y", obs, msg.frame_p1.y);
    AdapterUtilities::read<float>("frame_p1_z", obs, msg.frame_p1.z);
    AdapterUtilities::read<float>("frame_p2_x", obs, msg.frame_p2.x);
    AdapterUtilities::read<float>("frame_p2_y", obs, msg.frame_p2.y);
    AdapterUtilities::read<float>("frame_p2_z", obs, msg.frame_p2.z);
    AdapterUtilities::read<float>("height", obs, msg.height);
    AdapterUtilities::read<int32_t>("hinge", obs, msg.hinge);
    AdapterUtilities::read<int32_t>("rot_dir", obs, msg.rot_dir);

    // Door Data
    debugMsg("ros:debug:synchronization", 
	      "door_p1 = <" << msg.door_p1.x << ", " << msg.door_p1.y << ", " << msg.door_p1.z << ">");
    debugMsg("ros:debug:synchronization", 
	      "door_p2 = <" << msg.door_p2.x << ", " << msg.door_p2.y << ", " << msg.door_p2.z << ">");

    AdapterUtilities::read<float>("door_p1_x", obs, msg.door_p1.x);
    AdapterUtilities::read<float>("door_p1_y", obs, msg.door_p1.y);
    AdapterUtilities::read<float>("door_p1_z", obs, msg.door_p1.z);
    AdapterUtilities::read<float>("door_p2_x", obs, msg.door_p2.x);
    AdapterUtilities::read<float>("door_p2_y", obs, msg.door_p2.y);
    AdapterUtilities::read<float>("door_p2_z", obs, msg.door_p2.z);

    // Handle Data
    AdapterUtilities::read<float>("handle_x", obs, msg.handle.x);
    AdapterUtilities::read<float>("handle_y", obs, msg.handle.y);
    AdapterUtilities::read<float>("handle_z", obs, msg.handle.z);

    // Travel dir Data
    AdapterUtilities::read<double>("travel_dir_x", obs, msg.travel_dir.x);
    AdapterUtilities::read<double>("travel_dir_y", obs, msg.travel_dir.y);
    AdapterUtilities::read<double>("travel_dir_z", obs, msg.travel_dir.z);
  }

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
