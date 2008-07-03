#include <signal.h>

//external ros includes for messages
#include <std_srvs/StaticMap.h>
#include <std_msgs/BaseVel.h>

//NDDL includes
#include "Nddl.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Utilities.hh"


#include "ROSNode.hh"
#include "WavefrontPlanner.hh"

//TiXmlElement* root = NULL;
//Clock* agentClock = NULL;
//std::ofstream dbgFile("Debug.log");

using namespace std_msgs;
static const double AllowableArmError = .12;

namespace TREX {
  ROSNodeId ROSNode::s_id;

  /**
   * ROSNode class implementation.
   */


  /**
   * @brief Singleton accessor
   */
  ROSNodeId ROSNode::request(){
    if(s_id == ROSNodeId::noId()){
      int argc = 0;
      ros::init(argc, NULL);
      new ROSNode();
    }
    s_id->addRef();
    return s_id;
  }

  /**
   * @brief Sets up publish and subscribe topics for this node to integrate it
   * at the top to publish Execution Status updates and at the bottom to dispatch 
   * commands to the RCS and receive updates
   */
  ROSNode::ROSNode() : ros::node("trex"), m_id(this),
			   m_initialized(1), 
			   m_state(UNDEFINED),
			   tf(*this, true, 200000000ULL), //nanoseconds
			   laser_maxrange(4.0),
			   laser_buffer_time(3.0) {

    

    debugMsg("ROSNode:Create", "ROSNode created.");
    s_id = m_id; m_refCount = 0;
    pthread_mutex_init(&m_lock, NULL);
    //  m_rcs_obs.pos.x = 0.0;
    //     m_rcs_obs.pos.y = 0.0;
    //     m_rcs_obs.pos.th = 0.0;
    //     m_rcs_obs.done = 1;
    //     m_rcs_obs.valid = 1;

    //copied from wavefront_planner.cc
    this->tf.setWithEulers(FRAMEID_LASER,
			   FRAMEID_ROBOT,
			   0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
    
    this->laser_hitpts_size = this->laser_hitpts_len = 0;
    this->laser_hitpts = NULL;

     std_srvs::StaticMap::request  req;
     std_srvs::StaticMap::response resp;
     debugMsg("ROSNode:Create", "Requesting the map...");
     while(!ros::service::call("static_map", req, resp)) {
       debugMsg("ROSNode:Create", "request failed; trying again...");
       usleep(1000000);
     }
     debugMsg("ROSNode:Create", "Received a " << resp.map.width << " X "
	      << resp.map.height << " map @ " << resp.map.resolution
	      << " m/pix");
     char* mapdata;
     int sx, sy;
     sx = resp.map.width;
     sy = resp.map.height;
     // Convert to player format
     mapdata = new char[sx*sy];
     for(int i=0;i<sx*sy;i++) {
       if(resp.map.data[i] == 0)
	 mapdata[i] = -1;
       else if(resp.map.data[i] == 100)
	 mapdata[i] = +1;
       else
	 mapdata[i] = 0;
     }
     

     //so we don't take up a lot of time during the cycle for map initialization
     WavefrontPlanner::Instance(mapdata, sx, sy);
     
     delete[] mapdata;


    //advertise<MsgToken>("navigator");
    advertise<Planner2DState>("state");
    advertise<Polyline2D>("gui_path");
    advertise<Polyline2D>("gui_laser");
    advertise<Planner2DGoal>("goal");
    advertise<BaseVel>("cmd_vel");
    advertise<PR2Arm>("right_pr2arm_set_position");
    advertise<PR2Arm>("left_pr2arm_set_position");
    //subscribe("state", m_rcs_obs, &ROSNode::rcs_cb);
    subscribe("scan", laserMsg, &ROSNode::laserReceived);
    subscribe("left_pr2arm_pos", leftArmPosMsg, &ROSNode::leftArmPosReceived);
    subscribe("right_pr2arm_pos", rightArmPosMsg, &ROSNode::rightArmPosReceived);
    //subscribe("localizedpose", m_localizedOdomMsg, &ROSNode::localizedOdomReceived);
    m_initialized = false;
    
    _leftArmActive = false;
    _rightArmActive = false;
    _generateFirstObservation = true;
    _leftArmInit = false;
    _rightArmInit = false;
    
  }


  void ROSNode::release() {
    if (ROSNode::s_id->decRef()) {
      ROSNode::s_id->shutdown();
      delete (ROSNode*)s_id;
      s_id = ROSNodeId::noId();
    }
  }

  ROSNode::~ROSNode() {
    m_id.remove();
  }

  void ROSNode::addRef() {
    m_refCount++;
  }

  bool ROSNode::decRef() {
    m_refCount--;
    return m_refCount == 0;
  }


  /**
   * @brief Nothing to do here. The msg object will already have been updated and that is what we use
   * directly in synchronization
   */
  void ROSNode::rcs_cb(){
    m_initialized = true;
    //debugMsg("ROSNode:Callback", "Received Update:" << toString(m_rcs_obs));
  }

  void ROSNode::leftArmPosReceived() {
    _leftArmInit = true;
    //std::cout << "Got left arm message " 
    //      << " turretAngle " << leftArmPosMsg.turretAngle << std::endl;
  }

  void ROSNode::rightArmPosReceived() {
    _rightArmInit = true;
    //std::cout << "Got right arm message " 
    //	      << " turretAngle " << leftArmPosMsg.turretAngle << std::endl;
  }

  /**
   * @brief Called when an update is received in the Monitor during synchronization
   */
  void ROSNode::my_publish(const Observation& obs)
  {
    debugMsg("ROSNode:publish", obs.toString());
    //MsgToken msg;
    //msg.predicate = obs.toString();
    //ros::node::publish("navigator", msg);
  }

  /**
   * @brief Dispatch a token
   */
  void ROSNode::dispatchWaypoint(const TokenId& goal){
    static const LabelStr ACTIVE("WaypointController.Active");
    static const LabelStr X2("x");
    static const LabelStr Y2("y");

    if (goal->getPredicateName() == ACTIVE){
      Planner2DGoal msg;
      const IntervalDomain& x = goal->getVariable(X2)->lastDomain();
      const IntervalDomain& y = goal->getVariable(Y2)->lastDomain();

      checkError(x.isSingleton(), x.toString());
      checkError(y.isSingleton(), y.toString());

      // Fill outbound msg. Don't care about orientation
      msg.goal.x = x.getSingletonValue();
      msg.goal.y = y.getSingletonValue();
      msg.enable = true;
      debugMsg("ROSNode::waypoint", "Dispatching Goal: " << 
	       goal->toString() << " AS " << toString(msg));
      ros::node::publish("goal", msg);
    }
  }

  void ROSNode::dispatchVel(const TokenId& cmd_vel){
    static const LabelStr HOLDS("VelCommander.Holds");
    static const LabelStr CMD_X("cmd_x");
    static const LabelStr CMD_TH("cmd_th");

    if(cmd_vel->getPredicateName() == HOLDS){
      BaseVel mbv;
      const IntervalDomain& cmd_x = cmd_vel->getVariable(CMD_X)->lastDomain();
      const IntervalDomain& cmd_th = cmd_vel->getVariable(CMD_TH)->lastDomain();
      
      checkError(cmd_x.isSingleton(), cmd_x.toString());
      checkError(cmd_th.isSingleton(), cmd_th.toString());

      mbv.vx = cmd_x.getSingletonValue();
      mbv.vw = cmd_th.getSingletonValue();
      //debugMsg("ROSNode:dispatchVel", "Sending vel x " << mbv.vx << " y " << mbv.vw);
      ros::node::publish("cmd_vel",mbv);
    }

    //going to do global path stuff here too
    plan_t* plan = WavefrontPlanner::Instance()->GetActivePlan();

    this->polylineMsg.set_points_size(plan->path_count);
    this->polylineMsg.color.r = 0;
    this->polylineMsg.color.g = 1.0;
    this->polylineMsg.color.b = 0;
    this->polylineMsg.color.a = 0;
    for(int i=0;i<plan->path_count;i++)
      {
	this->polylineMsg.points[i].x = 
	  PLAN_WXGX(plan,plan->path[i]->ci);
	this->polylineMsg.points[i].y = 
	  PLAN_WYGY(plan,plan->path[i]->cj);
      }
    publish("gui_path", polylineMsg);
  }

  void ROSNode::dispatchArm(const TokenId& cmd_arm) {
    
    std::cout << "Trying to dispatch arm.\n";
    

    if(cmd_arm->getPredicateName() == LabelStr("ArmController.Active")) {
      //figure out if it's left or right
      PR2Arm armGoal;
      armGoal.turretAngle = cmd_arm->getVariable("turretAngle")->lastDomain().getSingletonValue();
      armGoal.shoulderLiftAngle = cmd_arm->getVariable("shoulderLiftAngle")->lastDomain().getSingletonValue();
      armGoal.upperarmRollAngle = cmd_arm->getVariable("upperarmRollAngle")->lastDomain().getSingletonValue();
      armGoal.elbowAngle = cmd_arm->getVariable("elbowAngle")->lastDomain().getSingletonValue();
      armGoal.forearmRollAngle = cmd_arm->getVariable("forearmRollAngle")->lastDomain().getSingletonValue();
      armGoal.wristPitchAngle = cmd_arm->getVariable("wristPitchAngle")->lastDomain().getSingletonValue();
      armGoal.wristRollAngle = cmd_arm->getVariable("wristRollAngle")->lastDomain().getSingletonValue();
      armGoal.gripperForceCmd = cmd_arm->getVariable("gripperForceCmd")->lastDomain().getSingletonValue();
      armGoal.gripperGapCmd = cmd_arm->getVariable("gripperGapCmd")->lastDomain().getSingletonValue();

      //std::cout << "WTF: " << getObjectName(cmd_arm).toString() << std::endl;
      
      if(getObjectName(cmd_arm)==LabelStr("rac")) {
	debugMsg("ROSNode::Arm", "dispatching command for right arm.");
	publish("right_pr2arm_set_position",armGoal);
	_rightArmActive = true;
	_lastRightArmGoal = armGoal;
      } else {
	debugMsg("ROSNode::Arm", "dispatching command for left arm.");
	publish("left_pr2arm_set_position",armGoal);
	_leftArmActive = true;
	_lastLeftArmGoal = armGoal;
      }
    }
  }

  void ROSNode::get_obs(std::vector<Observation*>& buff){
    Observation* vs = get_vs_obs();
    if(vs != NULL)
      buff.push_back(vs);

    get_laser_obs();

    //Observation* wpc = get_wpc_obs();
    //if(wpc != NULL)
    //  buff.push_back(wpc);

    //Observation* velc = get_vc_obs();
    //if(velc != NULL) 
      //  buff.push_back(velc);
  }

  Observation* ROSNode::get_vs_obs(){
    lock();


    libTF::TFPose2D robotPose,global_pose;
    robotPose.x = 0;
    robotPose.y = 0;
    robotPose.yaw = 0;
    robotPose.frame = FRAMEID_ROBOT;
    robotPose.time = laserMsg.header.stamp.sec * 1000000000ULL + 
      laserMsg.header.stamp.nsec; ///HACKE FIXME we should be able to get time somewhere else
    try {
      global_pose = this->tf.transformPose2D(FRAMEID_MAP, robotPose);

      //debugMsg("ROSNode::VS", ""New global pose x " << global_pose.x << " y " << global_pose.y);

    } catch(libTF::TransformReference::LookupException& ex) {
      debugMsg("ROSNode::VS", "no global->local Tx yet");
      return NULL;
    } catch(libTF::Pose3DCache::ExtrapolateException& ex) {
      debugMsg("ROSNode::VS", 
	       "libTF::Pose3DCache::ExtrapolateException occured");
    }

    

    this->m_rcs_state.active = 1;
    //(this->enable &&
    //		   (this->planner_state == PURSUING_GOAL)) ? 1 : 0;
    this->m_rcs_state.valid = 1; //(this->plan->path_count > 0) ? 1 : 0;
    //this->pstate.done = (this->planner_state == REACHED_GOAL ||
    //	       this->planner_state == NO_GOAL) ? 1 : 0;
    this->m_rcs_state.done = 0; //(this->planner_state == REACHED_GOAL) ? 1 : 0;
    this->m_rcs_state.pos.x = global_pose.x;
    this->m_rcs_state.pos.y = global_pose.y;
    this->m_rcs_state.pos.th = global_pose.yaw;
    //this->m_rcs_state.goal.x = this->goal[0];
    //this->m_rcs_state.goal.y = this->goal[1];
    //this->m_rcs_state.goal.th = this->goal[2];
    this->m_rcs_state.waypoint.x = 0.0;
    this->m_rcs_state.waypoint.y = 0.0;
    this->m_rcs_state.waypoint.th = 0.0;
    this->m_rcs_state.set_waypoints_size(0);
    this->m_rcs_state.waypoint_idx = -1;
    this->ros::node::publish("state",this->m_rcs_state);

    ObservationByValue* obs = NULL;
    obs = new ObservationByValue("vs", "VehicleState.Holds");
    obs->push_back("x", new IntervalDomain(m_rcs_state.pos.x));
    obs->push_back("y", new IntervalDomain(m_rcs_state.pos.y));
    obs->push_back("th", new IntervalDomain(m_rcs_state.pos.th));
    unlock();
    return obs;
  }

  Observation* ROSNode::get_vc_obs(){
    lock();
    //ObservationByValue* obs = NULL;
    //obs = new ObservationByValue("vcom", "VelCommander.Holds");
    //obs->push_back("cmd_x", new IntervalDomain(m_localizedOdomMsg.vel.x));
    //obs->push_back("cmd_th", new IntervalDomain(m_localizedOdomMsg.vel.th));
    unlock();
    //debugMsg("ROSNode:VC", "Received Observation:" << obs->toString() << std::endl;
    return NULL;
  }
  

  Observation* ROSNode::get_wpc_obs(){
    return NULL;
    //lock();
   //  ObservationByValue* obs = NULL;
//     // Determine current obseved state
//     PlannerState observedState = INACTIVE;

//     if(m_rcs_obs.active == 1)
//       observedState = ACTIVE;

//     // If there is a state change we have work to do
//     if(m_state != observedState){

//       m_state = observedState;
//       if(m_state == INACTIVE){
// 	obs = new ObservationByValue("wpc", "WaypointController.Inactive");
//       }
//       else {
// 	obs = new ObservationByValue("wpc", "WaypointController.Active");
// 	obs->push_back("x", new IntervalDomain(m_rcs_obs.goal.x));
// 	obs->push_back("y", new IntervalDomain(m_rcs_obs.goal.y));
//       }

//       debugMsg("ROSNode:WPC", "Received Observation:" << obs->toString());
//     }

//     unlock();
    // return obs;
  }

  //this doesn't actually generate observations for now
  void ROSNode::get_laser_obs() {
    lock();
    unlock();  
  }

  void ROSNode::get_arm_obs(std::vector<Observation*>& buff){
   
    if(_rightArmInit && _leftArmInit) {

      Observation* larm = get_left_arm_obs();
      if(larm != NULL)
	buff.push_back(larm);
      
      Observation* rarm = get_right_arm_obs();
      if(rarm != NULL)
	buff.push_back(rarm);
      
      _generateFirstObservation = false;
    }
  }

  Observation* ROSNode::get_left_arm_obs(){
    lock();
    ObservationByValue* obs = NULL;
    
    if(_generateFirstObservation || 
       (_leftArmActive &&
	fabs(leftArmPosMsg.turretAngle-_lastLeftArmGoal.turretAngle) < AllowableArmError &&
	fabs(leftArmPosMsg.shoulderLiftAngle-_lastLeftArmGoal.shoulderLiftAngle) < AllowableArmError &&
	fabs(leftArmPosMsg.upperarmRollAngle-_lastLeftArmGoal.upperarmRollAngle) < AllowableArmError &&
	fabs(leftArmPosMsg.elbowAngle-_lastLeftArmGoal.elbowAngle) < AllowableArmError &&
	fabs(leftArmPosMsg.forearmRollAngle-_lastLeftArmGoal.forearmRollAngle) < AllowableArmError &&
	fabs(leftArmPosMsg.wristPitchAngle-_lastLeftArmGoal.wristPitchAngle) < AllowableArmError &&
	fabs(leftArmPosMsg.wristRollAngle-_lastLeftArmGoal.wristRollAngle) < AllowableArmError)) {
      
      std::cout << "Pushing left arm inactive observation.\n";

      obs = new ObservationByValue("lac", "ArmController.Inactive");
      obs->push_back("acTurretAngle", new IntervalDomain(leftArmPosMsg.turretAngle));
      obs->push_back("acShoulderLiftAngle", new IntervalDomain(leftArmPosMsg.shoulderLiftAngle));
      obs->push_back("acUpperarmRollAngle", new IntervalDomain(leftArmPosMsg.upperarmRollAngle));
      obs->push_back("acElbowAngle", new IntervalDomain(leftArmPosMsg.elbowAngle));
      obs->push_back("acForearmRollAngle", new IntervalDomain(leftArmPosMsg.forearmRollAngle));
      obs->push_back("acWristPitchAngle", new IntervalDomain(leftArmPosMsg.wristPitchAngle));
      obs->push_back("acWristRollAngle", new IntervalDomain(leftArmPosMsg.wristRollAngle));
      obs->push_back("acGripperForceCmd", new IntervalDomain(leftArmPosMsg.gripperForceCmd));
      obs->push_back("acGripperGapCmd", new IntervalDomain(leftArmPosMsg.gripperGapCmd));
      _leftArmActive = false;
   }
    unlock();
   return obs;
  }
    
  Observation* ROSNode::get_right_arm_obs(){
    lock();
    ObservationByValue* obs = NULL;
    
    if(_generateFirstObservation ||
       (_rightArmActive &&
	fabs(rightArmPosMsg.turretAngle-_lastRightArmGoal.turretAngle) < AllowableArmError &&
	fabs(rightArmPosMsg.shoulderLiftAngle-_lastRightArmGoal.shoulderLiftAngle) < AllowableArmError &&
	fabs(rightArmPosMsg.upperarmRollAngle-_lastRightArmGoal.upperarmRollAngle) < AllowableArmError &&
	fabs(rightArmPosMsg.elbowAngle-_lastRightArmGoal.elbowAngle) < AllowableArmError &&
	fabs(rightArmPosMsg.forearmRollAngle-_lastRightArmGoal.forearmRollAngle) < AllowableArmError &&
	fabs(rightArmPosMsg.wristPitchAngle-_lastRightArmGoal.wristPitchAngle) < AllowableArmError &&
	fabs(rightArmPosMsg.wristRollAngle-_lastRightArmGoal.wristRollAngle) < AllowableArmError)) {
	 
      std::cout << "Pushing right arm inactive observation.\n";

      obs = new ObservationByValue("rac", "ArmController.Inactive");
      obs->push_back("acTurretAngle", new IntervalDomain(rightArmPosMsg.turretAngle));
      obs->push_back("acShoulderLiftAngle", new IntervalDomain(rightArmPosMsg.shoulderLiftAngle));
      obs->push_back("acUpperarmRollAngle", new IntervalDomain(rightArmPosMsg.upperarmRollAngle));
      obs->push_back("acElbowAngle", new IntervalDomain(rightArmPosMsg.elbowAngle));
      obs->push_back("acForearmRollAngle", new IntervalDomain(rightArmPosMsg.forearmRollAngle));
      obs->push_back("acWristPitchAngle", new IntervalDomain(rightArmPosMsg.wristPitchAngle));
      obs->push_back("acWristRollAngle", new IntervalDomain(rightArmPosMsg.wristRollAngle));
      obs->push_back("acGripperForceCmd", new IntervalDomain(rightArmPosMsg.gripperForceCmd));
      obs->push_back("acGripperGapCmd", new IntervalDomain(rightArmPosMsg.gripperGapCmd));
      _rightArmActive = false;
    }
    unlock();
    return obs;
  }

  bool ROSNode::isInitialized() const {
    return m_initialized;
  }

  void ROSNode::lock(){
    pthread_mutex_lock(&m_lock);
  }

  void ROSNode::unlock(){
    pthread_mutex_unlock(&m_lock);
  }


 //  void ROSNode::localizedOdomReceived() {
    
//     debugMsg("ROSNode:Odom", "Got localized odom vel x " << this->m_localizedOdomMsg.vel.x
// 	      << " y " << this->m_localizedOdomMsg.vel.y 
// 	      << " th " << this->m_localizedOdomMsg.vel.th << endl; 
    
//   }

  void ROSNode::laserReceived() {


    m_initialized = true;

    // Copy and push this scan into the list of buffered scans
    LaserScan newscan(laserMsg);
    // Do a deep copy
    /*
    newscan.header.stamp.sec = laserMsg.header.stamp.sec;
    newscan.header.stamp.nsec = laserMsg.header.stamp.nsec;
    newscan.header.frame_id = laserMsg.header.frame_id;
    newscan.range_max = laserMsg.range_max;
    newscan.angle_min = laserMsg.angle_min;
    newscan.angle_max = laserMsg.angle_max;
    newscan.angle_increment = laserMsg.angle_increment;
    newscan.set_ranges_size(laserMsg.get_ranges_size());
    memcpy(newscan.ranges,laserMsg.ranges,laserMsg.get_ranges_size()*sizeof(float));
    */
    this->buffered_laser_scans.push_back(newscan);
    
    // Iterate through the buffered scans, trying to interpolate each one
    for(std::list<LaserScan>::iterator it = this->buffered_laser_scans.begin();
	it != this->buffered_laser_scans.end();
	it++)
      {
	// For each beam, convert to cartesian in the laser's frame, then convert
	// to the map frame and store the result in the the laser_scans list
	
	laser_pts_t pts;
	pts.pts_num = it->get_ranges_size();
	pts.pts = new double[pts.pts_num*2];
	assert(pts.pts);
	pts.ts = it->header.stamp;
	
	libTF::TFPose2D local,global;
	local.frame = it->header.frame_id;
	local.time = it->header.stamp.to_ull();
	//local.time = it->header.stamp.sec * 1000000000ULL + 
	//  it->header.stamp.nsec;
	float b=it->angle_min;
	float* r=it->ranges;
	unsigned int i;
	unsigned int cnt=0;
	for(i=0;i<it->get_ranges_size();
	    i++,r++,b+=it->angle_increment)
	  {
	    // TODO: take out the bogus epsilon range_max check, after the
	    // hokuyourg_player node is fixed
	    if(((*r) >= this->laser_maxrange) || 
	       ((it->range_max > 0.1) && ((*r) >= it->range_max)) ||
	       ((*r) <= 0.01))
	      continue;
	    
	    local.x = (*r)*cos(b);
	    local.y = (*r)*sin(b);
	    local.yaw = 0;
	    try
	      {
		global = this->tf.transformPose2D(FRAMEID_MAP, local);
	      }
	    catch(libTF::TransformReference::LookupException& ex)
	      {
		puts("no global->local Tx yet");
		delete[] pts.pts;
		return;
	      }
	    catch(libTF::Pose3DCache::ExtrapolateException& ex)
	      {
		puts("extrapolation required");
		delete[] pts.pts;
		break;
	      }
	    
	    // Copy in the result
	    pts.pts[2*cnt] = global.x;
	    pts.pts[2*cnt+1] = global.y;
	    cnt++;
	  }
	// Did we break early?
	if(i < it->get_ranges_size())
	  continue;
	else
	  {
	    pts.pts_num = cnt;
	    this->laser_scans.push_back(pts);
	    it = this->buffered_laser_scans.erase(it);
	    it--;
	  }
      }
    
    // Remove anything that's too old from the laser_scans list
    // Also count how many points we have
    unsigned int hitpt_cnt=0;
    for(std::list<laser_pts_t>::iterator it = this->laser_scans.begin();
	it != this->laser_scans.end();
	it++)
      {
	if((laserMsg.header.stamp - it->ts) > this->laser_buffer_time)
	  {
	    delete[] it->pts;
	    it = this->laser_scans.erase(it);
	    it--;
	  }
	else
	  hitpt_cnt += it->pts_num;
      }
    
    // Lock here because we're operating on the laser_hitpts array, which is
    // used in another thread.
    lock();
    // allocate more space as necessary
    if(this->laser_hitpts_size < hitpt_cnt)
      {
	this->laser_hitpts_size = hitpt_cnt;
	this->laser_hitpts = 
	  (double*)realloc(this->laser_hitpts,
			   2*this->laser_hitpts_size*sizeof(double));
	assert(this->laser_hitpts);
      }
    
    // Copy all of the current hitpts into the laser_hitpts array, from where
    // they will be copied into the planner, via plan_set_obstacles(), in
    // doOneCycle()
    this->laser_hitpts_len = 0;
    for(std::list<laser_pts_t>::iterator it = this->laser_scans.begin();
	it != this->laser_scans.end();
	it++)
      {
	memcpy(this->laser_hitpts + this->laser_hitpts_len * 2,
	       it->pts, it->pts_num * 2 * sizeof(double));
	this->laser_hitpts_len += it->pts_num;
      }

    // Draw the points

    this->pointcloudMsg.set_points_size(this->laser_hitpts_len);
    this->pointcloudMsg.color.a = 0.0;
    this->pointcloudMsg.color.r = 0.0;
    this->pointcloudMsg.color.b = 1.0;
    this->pointcloudMsg.color.g = 0.0;
    for(unsigned int i=0;i<this->laser_hitpts_len;i++)
      {
	this->pointcloudMsg.points[i].x = this->laser_hitpts[2*i];
	this->pointcloudMsg.points[i].y = this->laser_hitpts[2*i+1];
      }
    this->ros::node::publish("gui_laser",this->pointcloudMsg);

    
    WavefrontPlanner::Instance()->SetObstacles(this->laser_hitpts, 
					       this->laser_hitpts_len);  

    unlock(); 
  }

}
