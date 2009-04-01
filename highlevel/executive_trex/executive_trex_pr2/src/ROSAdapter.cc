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
      TREX_INFO("ros:info", "Storing data for " << timelineName.c_str() << " in the " << frame_id << " frame.");
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

    // Derived class will populate actual observations
    Observation* obs = NULL;
    obs = getObservation();

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

    // The source we are looking for is the origin in the base footprint frame
    source_pose.setIdentity();
    source_pose.frame_id_ = "base_footprint";
    source_pose.stamp_ = ros::Time();

    // Should we be waiting here?
    try{
      tf.transformPose(frame_id, source_pose, target_pose);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available getting 2D pose from base_footprint to %s. Error: %s\n", target_pose.frame_id_.c_str(), ex.what());
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

    debugMsg("ros:debug:synchronization:get2DPose", 
	     "Transformed base pose to (x=" << x << ", y=" << y << ", th=" << th << ")");
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

  void ROSAdapter::checkTFEnabled(){
    if(!tf_enabled){
      ROS_ERROR("Attempted to call a transform for an adapater that is not transform enabled. Check your TREX input configuration for the adapter %s and ensure the 'frame_id' parameter is set",
		getName().c_str());
    }
  }

  bool ROSAdapter::canTransform(const std::string& source_frame_id){
    static const ros::Duration timeout(ros::Duration().fromSec(1.0));

      // The message must have a frame id set
    if(!tf.canTransform(frame_id, source_frame_id, ros::Time::now(), timeout)){
      debugMsg("ros:error:synchronization", nameString() << "No transfrom available from " << source_frame_id << " to " << frame_id); 
      return false;
    }

    return true;
  }
}
