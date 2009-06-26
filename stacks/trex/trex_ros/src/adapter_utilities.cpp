#include <trex_ros/adapter_utilities.h>
#include <tf/transform_listener.h>

namespace trex_ros {
			     


  void AdapterUtilities::read(ObservationByValue& obs, const robot_msgs::PointStamped& msg){
    setHeader(msg, obs);
    readPoint(obs, msg.point.x, msg.point.y, msg.point.z);
  }

  void AdapterUtilities::write(const TokenId& token, robot_msgs::PointStamped& msg){
    getHeader(msg, token);

    write<double>("x", token, msg.point.x);
    write<double>("y", token, msg.point.y);
    write<double>("z", token, msg.point.z);
  }

  void AdapterUtilities::read(ObservationByValue& obs, const robot_msgs::PoseStamped& msg){
    setHeader(msg, obs);

    readPoint(obs, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    read<double>("qx", obs, msg.pose.orientation.x);
    read<double>("qy", obs, msg.pose.orientation.y);
    read<double>("qz", obs, msg.pose.orientation.z);
    read<double>("qw", obs, msg.pose.orientation.w);
  }


  void AdapterUtilities::write(const TokenId& token, robot_msgs::PoseStamped& msg) {
    getHeader(msg, token);

    writePoint(token, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

    write<double>("qx", token, msg.pose.orientation.x);
    write<double>("qy", token, msg.pose.orientation.y);
    write<double>("qz", token, msg.pose.orientation.z);
    write<double>("qw", token, msg.pose.orientation.w);
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

  void AdapterUtilities::writePoint(const TokenId& token, double& x, double& y, double& z){
    write("x", token, x);
    write("y", token, y);
    write("z", token, z);
  }

  StringDomain* AdapterUtilities::toStringDomain(const std_msgs::String& msg){
    return new StringDomain(LabelStr(msg.data), StringDT::instance());
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
}
