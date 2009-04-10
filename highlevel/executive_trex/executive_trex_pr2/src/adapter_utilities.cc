#include <executive_trex_pr2/adapter_utilities.h>
#include <tf/transform_listener.h>

namespace executive_trex_pr2 {
			     


  /**
   * Write token to the message
   */
  void AdapterUtilities::write(const TokenId& token, robot_msgs::Door& msg){
    // Set the frame we are in
    msg.header.frame_id = getFrame(token);

    // Extract the stamp
    double time_stamp_double;
    write<double>("time_stamp", token, time_stamp_double);
    msg.header.stamp.fromSec(time_stamp_double);

    // Frame Data
    write<float>("frame_p1_x", token, msg.frame_p1.x);
    write<float>("frame_p1_y", token, msg.frame_p1.y);
    write<float>("frame_p1_z", token, msg.frame_p1.z);
    write<float>("frame_p2_x", token, msg.frame_p2.x);
    write<float>("frame_p2_y", token, msg.frame_p2.y);
    write<float>("frame_p2_z", token, msg.frame_p2.z);
    write<float>("height", token, msg.height);
    write<int32_t>("hinge", token, msg.hinge);
    write<int32_t>("rot_dir", token, msg.rot_dir);

    // Door Data
    write<float>("door_p1_x", token, msg.door_p1.x);
    write<float>("door_p1_y", token, msg.door_p1.y);
    write<float>("door_p1_z", token, msg.door_p1.z);
    write<float>("door_p2_x", token, msg.door_p2.x);
    write<float>("door_p2_y", token, msg.door_p2.y);
    write<float>("door_p2_z", token, msg.door_p2.z);

    // Handle Data
    write<float>("handle_x", token, msg.handle.x);
    write<float>("handle_y", token, msg.handle.y);
    write<float>("handle_z", token, msg.handle.z);
  }

  // Read Observation from Door Message
  void AdapterUtilities::read(ObservationByValue& obs, const robot_msgs::Door& msg){
    setFrame(msg.header.frame_id, obs);

    // Frame Data
    read<float>("frame_p1_x", obs, msg.frame_p1.x);
    read<float>("frame_p1_y", obs, msg.frame_p1.y);
    read<float>("frame_p1_z", obs, msg.frame_p1.z);
    read<float>("frame_p2_x", obs, msg.frame_p2.x);
    read<float>("frame_p2_y", obs, msg.frame_p2.y);
    read<float>("frame_p2_z", obs, msg.frame_p2.z);
    read<float>("height", obs, msg.height);
    read<int32_t>("hinge", obs, msg.hinge);
    read<int32_t>("rot_dir", obs, msg.rot_dir);

    // Door Data
    debugMsg("ros:debug:synchronization", 
	      "door_p1 = <" << msg.door_p1.x << ", " << msg.door_p1.y << ", " << msg.door_p1.z << ">");
    debugMsg("ros:debug:synchronization", 
	      "door_p2 = <" << msg.door_p2.x << ", " << msg.door_p2.y << ", " << msg.door_p2.z << ">");

    read<float>("door_p1_x", obs, msg.door_p1.x);
    read<float>("door_p1_y", obs, msg.door_p1.y);
    read<float>("door_p1_z", obs, msg.door_p1.z);
    read<float>("door_p2_x", obs, msg.door_p2.x);
    read<float>("door_p2_y", obs, msg.door_p2.y);
    read<float>("door_p2_z", obs, msg.door_p2.z);

    // Handle Data
    read<float>("handle_x", obs, msg.handle.x);
    read<float>("handle_y", obs, msg.handle.y);
    read<float>("handle_z", obs, msg.handle.z);
  }

  void AdapterUtilities::read(ObservationByValue& obs, const robot_msgs::PlugStow& msg){
    setFrame(msg.header.frame_id, obs);
    read<bool>("stowed", obs, msg.stowed);
    read<double>("x", obs, msg.plug_centroid.x);
    read<double>("y", obs, msg.plug_centroid.y);
    read<double>("z", obs, msg.plug_centroid.z);
  }

  void AdapterUtilities::read(ObservationByValue& obs, const robot_msgs::PointStamped& msg){
    setFrame(msg.header.frame_id, obs);
    readPoint(obs, msg.point.x, msg.point.y, msg.point.z);
  }

  void AdapterUtilities::write(const TokenId& token, robot_msgs::PointStamped& msg){
    // Set the frame we are in
    msg.header.frame_id = getFrame(token);

    // Extract the stamp
    double time_stamp_double;
    write<double>("time_stamp", token, time_stamp_double);
    msg.header.stamp.fromSec(time_stamp_double);

    write<double>("x", token, msg.point.x);
    write<double>("y", token, msg.point.y);
    write<double>("z", token, msg.point.z);
  }

  void AdapterUtilities::read(ObservationByValue& obs, const robot_msgs::PoseStamped& msg){
    setFrame(msg.header.frame_id, obs);
    readPoint(obs, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    read<double>("dx", obs, msg.pose.orientation.x);
    read<double>("dy", obs, msg.pose.orientation.y);
    read<double>("dz", obs, msg.pose.orientation.z);
    read<double>("dw", obs, msg.pose.orientation.w);
  }

  void AdapterUtilities::write(const TokenId& token, robot_msgs::PlugStow& msg){
    // Set the frame we are in
    msg.header.frame_id = getFrame(token);

    // Extract the stamp
    double time_stamp_double;
    write<double>("time_stamp", token, time_stamp_double);
    msg.header.stamp.fromSec(time_stamp_double);

    write<int8_t>("stowed", token, msg.stowed);
    write<double>("x", token, msg.plug_centroid.x);
    write<double>("y", token, msg.plug_centroid.y);
    write<double>("z", token, msg.plug_centroid.z);
  }


  void AdapterUtilities::write(const TokenId& token, robot_actions::ServoToOutlet& msg) {
    //Set frame
    msg.header.frame_id = getFrame(token);
    
    // Extract the stamp
    double time_stamp_double;
    write<double>("time_stamp", token, time_stamp_double);
    msg.header.stamp.fromSec(time_stamp_double);

    write<float>("x", token, msg.x);
    write<float>("y", token, msg.y);
    write<float>("z", token, msg.z);

    write<float>("dx", token, msg.dx);
    write<float>("dy", token, msg.dy);
    write<float>("dz", token, msg.dz);
    write<float>("dw", token, msg.dw);
  }



  void AdapterUtilities::readPose(ObservationByValue& obs, double x, double y, double th){
    read("x", obs, x);
    read("y", obs, y);
    read("th", obs, th);
  }

  void AdapterUtilities::writePose(const TokenId& token, float& x, float& y, float& th){
    write("x", token, x);
    write("y", token, y);
    write("th", token, th);
  }

  void AdapterUtilities::readPoint(ObservationByValue& obs, double x, double y, double z){
    read("x", obs, x);
    read("y", obs, y);
    read("z", obs, z);
  }

  void AdapterUtilities::writePoint(const TokenId& token, float& x, float& y, float& z){
    write("x", token, x);
    write("y", token, y);
    write("z", token, z);
  }

  StringDomain* AdapterUtilities::toStringDomain(const std_msgs::String& msg){
    return new StringDomain(LabelStr(msg.data), "string");
  }

  // bind a string
  void AdapterUtilities::write(const StringDomain& dom, std_msgs::String& msg){
    msg.data = (dom.isSingleton() ? LabelStr(dom.getSingletonValue()).toString() : "");
    condDebugMsg(!dom.isSingleton(), "trex:warning:dispatching", "Reducing unbound paramater " << dom.toString() << " to ''");
  }

  void AdapterUtilities::get2DPose(const robot_msgs::Pose& pose, double& x, double& y, double& th){
    tf::Stamped<tf::Pose> tf_pose;
    tf::PoseMsgToTF(pose, tf_pose);
    x = tf_pose.getOrigin().x();
    y = tf_pose.getOrigin().y();
    double useless_pitch, useless_roll;
    tf_pose.getBasis().getEulerZYX(th, useless_pitch, useless_roll);
    debugMsg("ros:debug:synchronization:get2DPose", 
	     "Extracted base pose to (x=" << x << ", y=" << y << ", th=" << th << ")");
  }

  std::string AdapterUtilities::getFrame(const TokenId& token){
    ConstrainedVariableId frame_var = token->getVariable("frame_id");
    // if no such parameter, or it is not a singleton, then return the empty string
    if(frame_var.isNoId() || !frame_var->lastDomain().isSingleton())
      return "map";

    LabelStr lblStr = frame_var->lastDomain().getSingletonValue();
    return lblStr.toString();
  }

  void AdapterUtilities::setFrame(const std::string& frame_id, ObservationByValue& obs){
    obs.push_back("frame_id", new StringDomain(frame_id, "string"));
  }
}
