#include "ROSAdapter.hh"
#include "Agent.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Observer.hh"

namespace TREX { 				     

  /**
   * ROS Adapters will always log, to support playback. This is achieved by setting parameters in the
   * base class constructor
   */
  ROSAdapter::ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead)
    : Adapter(agentName, configData, lookAhead, 0, 1), m_initialized(false),
      timelineName(extractData(configData, "timelineName").toString()),
      timelineType(extractData(configData, "timelineType").toString()), 
      stateTopic(extractData(configData, "stateTopic").toString()),
      tf_enabled(hasFrameParam(configData)),
      tf(*ros::Node::instance(), true, ros::Duration(10)){
    commonInit(configData);
  }

  ROSAdapter::ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead, const std::string& state_topic)
    : Adapter(agentName, configData, lookAhead, 0, 1), m_initialized(false),
      timelineName(extractData(configData, "timelineName").toString()),
      timelineType(extractData(configData, "timelineType").toString()), 
      stateTopic(timelineName + state_topic), 
      tf_enabled(hasFrameParam(configData)),
      tf(*ros::Node::instance(), true, ros::Duration(10)){
    commonInit(configData);
  }

  void ROSAdapter::commonInit(const TiXmlElement& configData){
    TREX_INFO("ros:info", nameString() << "Adapter constructed for " << timelineName.c_str());

    m_node = Executive::request();

    // If there is a frame specified, then we will use its value and mark the adapter as transform enabled
    const char* frame_param = configData.Attribute("frame_id");
    if(frame_param == NULL){
      frame_id = "NOT APPLICABLE";
    }
    else {
      frame_id = frame_param;
    }

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

  bool ROSAdapter::hasFrameParam(const TiXmlElement& configData){
    return configData.Attribute("frame_id") != NULL;
  }

  void ROSAdapter::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
    ROS_INFO("Trying to initialize %s.", timelineName.c_str());

    TREX::Adapter::handleInit(initialTick, serversByTimeline, observer);

    registerPublishers();

    registerSubscribers();

    // Wait till we are initialized before moving ahead
    while(!isInitialized() && ros::Node::instance()->ok()){
      ROS_INFO("Waiting to connect for %s. If this is taking too long then the expected message is not being published.", timelineName.c_str());
      sleep(1);
    }

    ROS_INFO("Connection established for %s", timelineName.c_str());
  }

  ROSAdapter::~ROSAdapter() {
  }

  bool ROSAdapter::synchronize(){
    TREX_INFO("ros:synchronization", nameString() << "Checking..");

    // Derived class will populate actual observations
    Observation* obs = NULL;
    obs = getObservation();

    TREX_INFO("ros:synchronization", nameString() << obs->toString());

    if(obs != NULL){
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

  void ROSAdapter::get2DPose(double& x, double& y, double& th){
    checkTFEnabled();

    tf::Stamped<tf::Pose> source_pose, target_pose;
    source_pose.setIdentity();
    source_pose.frame_id_ = "base_footprint";
    source_pose.stamp_ = ros::Time();

    // Should we be waiting here?
    try{
      tf.transformPose(frame_id, source_pose, target_pose);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    }

    x = target_pose.getOrigin().x();
    y = target_pose.getOrigin().y();
    double useless_pitch, useless_roll;
    target_pose.getBasis().getEulerZYX(th, useless_pitch, useless_roll);

    ROS_DEBUG("Transformed pose to (x=%f, y=%f, th=%f)", x, y, th);
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

  // Transform one point to another. The output is in the frame id of the adapter
  void ROSAdapter::transformPoint(const std::string& source_frame_id, robot_msgs::Point32& out, const robot_msgs::Point32& in){
    checkTFEnabled();

    tf::Stamped<tf::Point> tmp;
    tmp.stamp_ = ros::Time();
    tmp.frame_id_ = source_frame_id;
    tmp[0] = in.x;
    tmp[1] = in.y;
    tmp[2] = in.z;

    // Should we be waiting here?
    try{
      tf.transformPoint(frame_id, tmp, tmp);
      out.x = tmp[0];
      out.y = tmp[1];
      out.z = tmp[2];
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    }

    ROS_DEBUG("Transformed pose to (x=%f, y=%f, th=%f)", out.x, out.y, out.y);
  }

  void ROSAdapter::checkTFEnabled(){
    if(!tf_enabled){
      ROS_ERROR("Attempted to call a transform for an adapater that is not transform enabled. Check your TREX input configuration for the adapter %s and ensure the 'frame_id' parameter is set",
		getName().c_str());
    }
  }
}
