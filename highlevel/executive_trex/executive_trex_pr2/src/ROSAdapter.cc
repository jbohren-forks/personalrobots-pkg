#include "ROSAdapter.hh"
#include "Agent.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Observer.hh"
#include <tf/transform_listener.h>

namespace TREX { 				     


  /**
   * Write token to the message
   */
  void ROSAdapter::writeTokenToDoorMessage(const TokenId& token, robot_msgs::Door& msg){
    // Set the frame we are in
    msg.header.frame_id = ROSAdapter::getFrame(token);

    // Extract the stamp
    double time_stamp_double;
    ROSAdapter::write<double>("time_stamp", token, time_stamp_double);
    msg.header.stamp.fromSec(time_stamp_double);

    // Frame Data
    ROSAdapter::write<float>("frame_p1_x", token, msg.frame_p1.x);
    ROSAdapter::write<float>("frame_p1_y", token, msg.frame_p1.y);
    ROSAdapter::write<float>("frame_p1_z", token, msg.frame_p1.z);
    ROSAdapter::write<float>("frame_p2_x", token, msg.frame_p2.x);
    ROSAdapter::write<float>("frame_p2_y", token, msg.frame_p2.y);
    ROSAdapter::write<float>("frame_p2_z", token, msg.frame_p2.z);
    ROSAdapter::write<float>("height", token, msg.height);
    ROSAdapter::write<int32_t>("hinge", token, msg.hinge);
    ROSAdapter::write<int32_t>("rot_dir", token, msg.rot_dir);

    // Door Data
    ROSAdapter::write<float>("door_p1_x", token, msg.door_p1.x);
    ROSAdapter::write<float>("door_p1_y", token, msg.door_p1.y);
    ROSAdapter::write<float>("door_p1_z", token, msg.door_p1.z);
    ROSAdapter::write<float>("door_p2_x", token, msg.door_p2.x);
    ROSAdapter::write<float>("door_p2_y", token, msg.door_p2.y);
    ROSAdapter::write<float>("door_p2_z", token, msg.door_p2.z);

    // Handle Data
    ROSAdapter::write<float>("handle_x", token, msg.handle.x);
    ROSAdapter::write<float>("handle_y", token, msg.handle.y);
    ROSAdapter::write<float>("handle_z", token, msg.handle.z);
  }

  /**
   * ROS Adapters will always log, to support playback. This is achieved by setting parameters in the
   * base class constructor
   */
  ROSAdapter::ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead)
    : Adapter(agentName, configData, lookAhead, 0, 1), m_initialized(false),
      timelineName(extractData(configData, "timelineName").toString()),
      timelineType(extractData(configData, "timelineType").toString()), 
      stateTopic(extractData(configData, "stateTopic").toString()){
    commonInit(configData);
  }

  ROSAdapter::ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead, const std::string& state_topic)
    : Adapter(agentName, configData, lookAhead, 0, 1), m_initialized(false),
      timelineName(extractData(configData, "timelineName").toString()),
      timelineType(extractData(configData, "timelineType").toString()), 
      stateTopic(timelineName + state_topic){
    commonInit(configData);
  }

  void ROSAdapter::commonInit(const TiXmlElement& configData){
    TREX_INFO("ros:info", nameString() << "Adapter constructed for " << timelineName.c_str());

    m_node = Executive::request();

    // Iterate over child xml nodes and look for nodes of type Param to populate the nddl to ros mappings
    // Iterate over internal and external configuration specifications
    for (TiXmlElement * child = configData.FirstChildElement();
           child != NULL;
           child = child->NextSiblingElement()) {

        if(strcmp(child->Value(), "Param") == 0) {
	  LabelStr nddl = extractData(*child, "nddl");
	  LabelStr ros = extractData(*child, "ros");
	  nddlNames_.push_back(nddl.toString());
	  rosNames_.push_back(ros.toString());
	}
    }
  }

  void ROSAdapter::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
    TREX_INFO("ros:info", "Trying to initialize " << timelineName.c_str());

    TREX::Adapter::handleInit(initialTick, serversByTimeline, observer);

    registerPublishers();

    registerSubscribers();

    // Wait till we are initialized before moving ahead
    while(!isInitialized() && ros::Node::instance()->ok()){
      TREX_INFO("ros:info", "Waiting to connect for " << timelineName.c_str() << 
		". If this is taking too long then the expected message is not being published. Check rostopic hz <topic_name>");
      sleep(1);
    }

    TREX_INFO("ros:info", "Connection established for " << timelineName.c_str());
  }

  ROSAdapter::~ROSAdapter() {
  }

  bool ROSAdapter::synchronize(){
    TREX_INFO("ros:debug:synchronization", nameString() << "Synchronizing");
    // Derived class will populate actual observations
    Observation* obs = NULL;
    obs = getObservation();

    if(obs != NULL){
      TREX_INFO("ros:info", nameString() << "Found observation:" << obs->toString());
      sendNotify(*obs);
      delete obs;
    }

    return true;
  }

  void ROSAdapter::handleNextTick(){}

  void ROSAdapter::handleCallback(){
    m_initialized = true;
  }

  bool ROSAdapter::isInitialized() const {
    return m_initialized;
  }

  bool ROSAdapter::handleRequest(const TokenId& goal){
    return dispatchRequest(goal, true);
  }

  /**
   * @brief Recalls will always be processed immediately
   */
  void ROSAdapter::handleRecall(const TokenId& goal){
    dispatchRequest(goal, false);
  }

  bool ROSAdapter::rosIndex(const std::string& rosName, unsigned int& ind) const{
    for(unsigned int i = 0; i < rosNames().size(); i++){
      if(rosNames()[i] == rosName){
	ind = i;
	return true;
      }
    }
    return false;
  }

  void ROSAdapter::readPose(ObservationByValue& obs, double x, double y, double th){
    read("x", obs, x);
    read("y", obs, y);
    read("th", obs, th);
  }

  void ROSAdapter::writePose(const TokenId& token, float& x, float& y, float& th){
    write("x", token, x);
    write("y", token, y);
    write("th", token, th);
  }

  void ROSAdapter::readPoint(ObservationByValue& obs, double x, double y, double z){
    read("x", obs, x);
    read("y", obs, y);
    read("z", obs, z);
  }

  void ROSAdapter::writePoint(const TokenId& token, float& x, float& y, float& z){
    write("x", token, x);
    write("y", token, y);
    write("z", token, z);
  }

  StringDomain* ROSAdapter::toStringDomain(const std_msgs::String& msg){
    return new StringDomain(LabelStr(msg.data), "string");
  }

  // bind a string
  void ROSAdapter::write(const StringDomain& dom, std_msgs::String& msg){
    msg.data = (dom.isSingleton() ? LabelStr(dom.getSingletonValue()).toString() : "");
    condDebugMsg(!dom.isSingleton(), "trex:warning:dispatching", "Reducing unbound paramater " << dom.toString() << " to ''");
  }

  void ROSAdapter::get2DPose(const robot_msgs::Pose& pose, double& x, double& y, double& th){
    tf::Stamped<tf::Pose> tf_pose;
    tf::PoseMsgToTF(pose, tf_pose);
    x = tf_pose.getOrigin().x();
    y = tf_pose.getOrigin().y();
    double useless_pitch, useless_roll;
    tf_pose.getBasis().getEulerZYX(th, useless_pitch, useless_roll);
    debugMsg("ros:debug:synchronization:get2DPose", 
	     "Extracted base pose to (x=" << x << ", y=" << y << ", th=" << th << ")");
  }

  std::string ROSAdapter::getFrame(const TokenId& token){
    ConstrainedVariableId frame_var = token->getVariable("frame_id");
    // if no such parameter, or it is not a singleton, then return the empty string
    if(frame_var.isNoId() || !frame_var->lastDomain().isSingleton())
      return "map";

    LabelStr lblStr = frame_var->lastDomain().getSingletonValue();
    return lblStr.toString();
  }

  void ROSAdapter::setFrame(const std::string frame_id, ObservationByValue* obs){
    obs->push_back("frame_id", new StringDomain(frame_id, "string"));
  }
}
