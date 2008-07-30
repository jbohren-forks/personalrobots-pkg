#include <signal.h>

//external ros includes for messages
#include <std_srvs/StaticMap.h>
#include <std_msgs/BaseVel.h>
#include <pr2_msgs/EndEffectorState.h>
#include "WavefrontPlanner.hh"

//NDDL includes
#include "Nddl.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Utilities.hh"

// TREX Includes 
#include "Agent.hh"
#include "Debug.hh"

#include "ROSNode.hh"
#include "LogManager.hh"

using namespace std_msgs;
using namespace KDL;
static const double AllowableArmError = .05;
static const double AllowableGraspError = .011;

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
  ROSNode::ROSNode() : ros::node("trex"), 
		       tf(*this, true), //nanoseconds
		       m_id(this),
		       m_initialized(1), 
		       m_state(UNDEFINED),
		       laser_maxrange(4.0),
		       laser_buffer_time(3.0)
  {

    debugMsg("ROSNode:Create", "ROSNode created.");
    s_id = m_id; m_refCount = 0;
    pthread_mutex_init(&m_lock, NULL);
    //  m_rcs_obs.pos.x = 0.0;
    //     m_rcs_obs.pos.y = 0.0;
    //     m_rcs_obs.pos.th = 0.0;
    //     m_rcs_obs.done = 1;
    //     m_rcs_obs.valid = 1;

    //copied from wavefront_planner.cc
    //TODO change this to broadcast.
    this->tf.setWithEulers(tf.lookup("FRAMEID_LASER"),
			   tf.lookup("FRAMEID_ROBOT"),
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
    advertise<pr2_msgs::EndEffectorState>("cmd_leftarm_cartesian");
    advertise<pr2_msgs::EndEffectorState>("cmd_rightarm_cartesian");
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
    _lastActiveLeftArmDispatch = 10000;
    _lastActiveRightArmDispatch = 10000;
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
    static const LabelStr ACTIVE("MoveBase.Active");
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
    static const LabelStr HOLDS("BaseGoal.Holds");
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

  void ROSNode::dispatchArm(const TokenId& cmd_arm, TICK currentTick) {
    
    std::cout << "Trying to dispatch arm.\n";
    std::cout << "Pred name: " << cmd_arm->getPredicateName().toString() << std::endl;
    if(cmd_arm->getPredicateName() == LabelStr("MoveArm.Active")) {
      PR2Arm armGoal;
      //figure out if it's left or right
      armGoal.turretAngle = cmd_arm->getVariable("turretAngle")->lastDomain().getSingletonValue();
      armGoal.shoulderLiftAngle = cmd_arm->getVariable("shoulderLiftAngle")->lastDomain().getSingletonValue();
      armGoal.upperarmRollAngle = cmd_arm->getVariable("upperarmRollAngle")->lastDomain().getSingletonValue();
      armGoal.elbowAngle = cmd_arm->getVariable("elbowAngle")->lastDomain().getSingletonValue();
      armGoal.forearmRollAngle = cmd_arm->getVariable("forearmRollAngle")->lastDomain().getSingletonValue();
      armGoal.wristPitchAngle = cmd_arm->getVariable("wristPitchAngle")->lastDomain().getSingletonValue();
      armGoal.wristRollAngle = cmd_arm->getVariable("wristRollAngle")->lastDomain().getSingletonValue();
      armGoal.gripperForceCmd = cmd_arm->getVariable("gripperForceCmd")->lastDomain().getSingletonValue();
      armGoal.gripperGapCmd = cmd_arm->getVariable("gripperGapCmd")->lastDomain().getSingletonValue();

      if(getObjectName(cmd_arm)==LabelStr("moveRightArm")) {
	debugMsg("ROSNode::Arm", "dispatching command for right arm. " <<
		 armGoal.turretAngle << " " <<
		 armGoal.shoulderLiftAngle << " " <<
		 armGoal.upperarmRollAngle << " " <<
		 armGoal.elbowAngle << " " <<
		 armGoal.forearmRollAngle << " " <<
		 armGoal.wristPitchAngle << " " <<
		 armGoal.wristRollAngle << " " <<
		 armGoal.gripperForceCmd << " " <<
		 armGoal.gripperGapCmd);
	publish("right_pr2arm_set_position",armGoal);
	
	_lastActiveRightArmDispatch = currentTick;
	_rightArmActive = true;
	_lastRightArmGoal = armGoal;
      } else {
	debugMsg("ROSNode::Arm", "dispatching command for left arm." <<
		 armGoal.turretAngle << " " <<
		 armGoal.shoulderLiftAngle << " " <<
		 armGoal.upperarmRollAngle << " " <<
		 armGoal.elbowAngle << " " <<
		 armGoal.forearmRollAngle << " " <<
		 armGoal.wristPitchAngle << " " <<
		 armGoal.wristRollAngle << " " <<
		 armGoal.gripperForceCmd << " " <<
		 armGoal.gripperGapCmd);
	publish("left_pr2arm_set_position",armGoal);
	_leftArmActive = true;
	_lastActiveLeftArmDispatch = currentTick;
	_lastLeftArmGoal = armGoal;
      }
    }
  }

  void ROSNode::dispatchEndEffector(const TokenId& cmd_end, TICK currentTick) {
    if(cmd_end->getPredicateName() == LabelStr("EndEffectorGoal.Holds")) {
      pr2_msgs::EndEffectorState endGoal;
      endGoal.set_rot_size(9);
      endGoal.set_trans_size(3);
      endGoal.rot[0] = cmd_end->getVariable("cmd_rot1_1")->lastDomain().getSingletonValue();
      endGoal.rot[1] = cmd_end->getVariable("cmd_rot1_2")->lastDomain().getSingletonValue();
      endGoal.rot[2] = cmd_end->getVariable("cmd_rot1_3")->lastDomain().getSingletonValue();
      endGoal.rot[3] = cmd_end->getVariable("cmd_rot2_1")->lastDomain().getSingletonValue();
      endGoal.rot[4] = cmd_end->getVariable("cmd_rot2_2")->lastDomain().getSingletonValue();
      endGoal.rot[5] = cmd_end->getVariable("cmd_rot2_3")->lastDomain().getSingletonValue();
      endGoal.rot[6] = cmd_end->getVariable("cmd_rot3_1")->lastDomain().getSingletonValue();
      endGoal.rot[7] = cmd_end->getVariable("cmd_rot3_2")->lastDomain().getSingletonValue();
      endGoal.rot[8] = cmd_end->getVariable("cmd_rot3_3")->lastDomain().getSingletonValue();
      endGoal.trans[0] = cmd_end->getVariable("cmd_x")->lastDomain().getSingletonValue();
      endGoal.trans[1] = cmd_end->getVariable("cmd_y")->lastDomain().getSingletonValue();
      endGoal.trans[2] = cmd_end->getVariable("cmd_z")->lastDomain().getSingletonValue();
      debugMsg("ROSNode::EndEffector", "Dispatching end frame " 
	       << endGoal.rot[0] << " " 
	       << endGoal.rot[1] << " " 
	       << endGoal.rot[2] << " " 
	       << endGoal.rot[3] << " " 
	       << endGoal.rot[4] << " " 
	       << endGoal.rot[5] << " " 
	       << endGoal.rot[6] << " " 
	       << endGoal.rot[7] << " " 
	       << endGoal.rot[8] << " " 
	       << endGoal.trans[0] << " " 
	       << endGoal.trans[1] << " " 
	       << endGoal.trans[2]); 
      /*
	std::cout << "ROSNode::EndEffector:: Dispatching end frame " 
	<< endGoal.rot[0] << " " 
	<< endGoal.rot[1] << " " 
	<< endGoal.rot[2] << " " 
	<< endGoal.rot[3] << " " 
	<< endGoal.rot[4] << " " 
	<< endGoal.rot[5] << " " 
	<< endGoal.rot[6] << " " 
	<< endGoal.rot[7] << " " 
	<< endGoal.rot[8] << " " 
	<< endGoal.trans[0] << " " 
	<< endGoal.trans[1] << " " 
	<< endGoal.trans[2] << std::endl; 
      */
	
      if(getObjectName(cmd_end) == LabelStr("rightEndEffectorGoal")) {
	debugMsg("ROSNode::EndEffector", "dispatching command for right end effector");
	publish("cmd_rightarm_cartesian", endGoal);
      } else {
	debugMsg("ROSNode::EndEffector", "dispatching command for left end effector");
	publish("cmd_leftarm_cartesian", endGoal);
      }
    }
  }

  void ROSNode::get_obs(std::vector<Observation*>& buff, TICK currentTick){
    Observation* vs = get_vs_obs();
    if(vs != NULL)
      buff.push_back(vs);

    //get_laser_obs doesn't do anything
    //get_laser_obs();

    //arm observations
    get_arm_obs(buff, currentTick);

    get_end_effector_obs(buff, currentTick);

    //Observation* wpc = get_wpc_obs();
    //if(wpc != NULL)
    //  buff.push_back(wpc);

    //Observation* velc = get_vc_obs();
    //if(velc != NULL) 
      //  buff.push_back(velc);
  }

  //this doesn't actually generate observations for now
  void ROSNode::get_laser_obs() {
    lock();
    unlock();
  }

  Observation* ROSNode::get_vs_obs(){
    lock();


    libTF::TFPose2D robotPose,global_pose;
    robotPose.x = 0;
    robotPose.y = 0;
    robotPose.yaw = 0;
    robotPose.frame = tf.lookup("FRAMEID_ROBOT");
    robotPose.time = laserMsg.header.stamp.sec * 1000000000ULL + 
      laserMsg.header.stamp.nsec; ///HACKE FIXME we should be able to get time somewhere else
    try {
      global_pose = this->tf.transformPose2D("FRAMEID_MAP", robotPose);

      debugMsg("ROSNode::VS", "New global pose x " << global_pose.x << " y " << global_pose.y);

    } catch(libTF::TransformReference::LookupException& ex) {
      debugMsg("ROSNode::VS", "no global->local Tx yet");
      return NULL;
    } catch(libTF::Pose3DCache::ExtrapolateException& ex) {
      debugMsg("ROSNode::VS", 
	       "libTF::Quaternion3D::ExtrapolateException occured");
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
    obs = new ObservationByValue("baseState", "BaseState.Holds");
    obs->push_back("x", new IntervalDomain(m_rcs_state.pos.x));
    obs->push_back("y", new IntervalDomain(m_rcs_state.pos.y));
    obs->push_back("th", new IntervalDomain(m_rcs_state.pos.th));
    unlock();
    return obs;
  }

  void ROSNode::get_arm_obs(std::vector<Observation*>& buff, TICK currentTick){
   
    if(_rightArmInit && _leftArmInit) {
      get_left_arm_obs(buff, currentTick);
      get_right_arm_obs(buff, currentTick);
      _generateFirstObservation = false;
    }
  }

  void ROSNode::get_left_arm_obs(std::vector<Observation*>& buff, TICK currentTick){
    lock();

    //do this no matter what
    ObservationByValue* obs1 = new ObservationByValue("leftArmState", "ArmState.Holds");
    obs1->push_back("acTurretAngle", new IntervalDomain(leftArmPosMsg.turretAngle));
    obs1->push_back("acShoulderLiftAngle", new IntervalDomain(leftArmPosMsg.shoulderLiftAngle));
    obs1->push_back("acUpperarmRollAngle", new IntervalDomain(leftArmPosMsg.upperarmRollAngle));
    obs1->push_back("acElbowAngle", new IntervalDomain(leftArmPosMsg.elbowAngle));
    obs1->push_back("acForearmRollAngle", new IntervalDomain(leftArmPosMsg.forearmRollAngle));
    obs1->push_back("acWristPitchAngle", new IntervalDomain(leftArmPosMsg.wristPitchAngle));
    obs1->push_back("acWristRollAngle", new IntervalDomain(leftArmPosMsg.wristRollAngle));
    obs1->push_back("acGripperForceCmd", new IntervalDomain(leftArmPosMsg.gripperForceCmd));
    obs1->push_back("acGripperGapCmd", new IntervalDomain(leftArmPosMsg.gripperGapCmd));
    buff.push_back(obs1);
    if(currentTick != _lastActiveLeftArmDispatch) {
      if(_generateFirstObservation || 
	 (_leftArmActive &&
	  fabs(leftArmPosMsg.turretAngle-_lastLeftArmGoal.turretAngle) < AllowableArmError &&
	  fabs(leftArmPosMsg.shoulderLiftAngle-_lastLeftArmGoal.shoulderLiftAngle) < AllowableArmError &&
	  fabs(leftArmPosMsg.upperarmRollAngle-_lastLeftArmGoal.upperarmRollAngle) < AllowableArmError &&
	  fabs(leftArmPosMsg.elbowAngle-_lastLeftArmGoal.elbowAngle) < AllowableArmError &&
	  fabs(leftArmPosMsg.forearmRollAngle-_lastLeftArmGoal.forearmRollAngle) < AllowableArmError &&
	  fabs(leftArmPosMsg.wristPitchAngle-_lastLeftArmGoal.wristPitchAngle) < AllowableArmError &&
	  fabs(leftArmPosMsg.wristRollAngle-_lastLeftArmGoal.wristRollAngle) < AllowableArmError &&
	  fabs(leftArmPosMsg.gripperGapCmd-_lastLeftArmGoal.gripperGapCmd) < AllowableGraspError)) {
	
	std::cout << "Pushing left arm inactive observation.\n";
	
	ObservationByValue* obs2 = new ObservationByValue("moveLeftArm", "MoveArm.Inactive");
	obs2->push_back("acTurretAngle", new IntervalDomain(leftArmPosMsg.turretAngle));
	obs2->push_back("acShoulderLiftAngle", new IntervalDomain(leftArmPosMsg.shoulderLiftAngle));
	obs2->push_back("acUpperarmRollAngle", new IntervalDomain(leftArmPosMsg.upperarmRollAngle));
	obs2->push_back("acElbowAngle", new IntervalDomain(leftArmPosMsg.elbowAngle));
	obs2->push_back("acForearmRollAngle", new IntervalDomain(leftArmPosMsg.forearmRollAngle));
	obs2->push_back("acWristPitchAngle", new IntervalDomain(leftArmPosMsg.wristPitchAngle));
	obs2->push_back("acWristRollAngle", new IntervalDomain(leftArmPosMsg.wristRollAngle));
	obs2->push_back("acGripperForceCmd", new IntervalDomain(leftArmPosMsg.gripperForceCmd));
	obs2->push_back("acGripperGapCmd", new IntervalDomain(leftArmPosMsg.gripperGapCmd));
	buff.push_back(obs2);
	
	_leftArmActive = false;
      }
    }
    unlock();
  }
    
  void ROSNode::get_right_arm_obs(std::vector<Observation*>& buff, TICK currentTick){
    lock();

    //do this no matter what
    ObservationByValue* obs1 = new ObservationByValue("rightArmState", "ArmState.Holds");
    obs1->push_back("acTurretAngle", new IntervalDomain(rightArmPosMsg.turretAngle));
    obs1->push_back("acShoulderLiftAngle", new IntervalDomain(rightArmPosMsg.shoulderLiftAngle));
    obs1->push_back("acUpperarmRollAngle", new IntervalDomain(rightArmPosMsg.upperarmRollAngle));
    obs1->push_back("acElbowAngle", new IntervalDomain(rightArmPosMsg.elbowAngle));
    obs1->push_back("acForearmRollAngle", new IntervalDomain(rightArmPosMsg.forearmRollAngle));
    obs1->push_back("acWristPitchAngle", new IntervalDomain(rightArmPosMsg.wristPitchAngle));
    obs1->push_back("acWristRollAngle", new IntervalDomain(rightArmPosMsg.wristRollAngle));
    obs1->push_back("acGripperForceCmd", new IntervalDomain(rightArmPosMsg.gripperForceCmd));
    obs1->push_back("acGripperGapCmd", new IntervalDomain(rightArmPosMsg.gripperGapCmd));
    buff.push_back(obs1);

    if(currentTick != _lastActiveRightArmDispatch) {
      if(_generateFirstObservation ||
	 (_rightArmActive &&
	  fabs(rightArmPosMsg.turretAngle-_lastRightArmGoal.turretAngle) < AllowableArmError &&
	  fabs(rightArmPosMsg.shoulderLiftAngle-_lastRightArmGoal.shoulderLiftAngle) < AllowableArmError &&
	  fabs(rightArmPosMsg.upperarmRollAngle-_lastRightArmGoal.upperarmRollAngle) < AllowableArmError &&
	  fabs(rightArmPosMsg.elbowAngle-_lastRightArmGoal.elbowAngle) < AllowableArmError &&
	  fabs(rightArmPosMsg.forearmRollAngle-_lastRightArmGoal.forearmRollAngle) < AllowableArmError &&
	  fabs(rightArmPosMsg.wristPitchAngle-_lastRightArmGoal.wristPitchAngle) < AllowableArmError &&
	  fabs(rightArmPosMsg.wristRollAngle-_lastRightArmGoal.wristRollAngle) < AllowableArmError &&
	  fabs(rightArmPosMsg.gripperGapCmd-_lastRightArmGoal.gripperGapCmd) < AllowableGraspError)) {
	std::cout << "Pushing right arm inactive observation.\n";       	
	
	ObservationByValue* obs2 = new ObservationByValue("moveRightArm", "MoveArm.Inactive");
	obs2->push_back("acTurretAngle", new IntervalDomain(rightArmPosMsg.turretAngle));
	obs2->push_back("acShoulderLiftAngle", new IntervalDomain(rightArmPosMsg.shoulderLiftAngle));
	obs2->push_back("acUpperarmRollAngle", new IntervalDomain(rightArmPosMsg.upperarmRollAngle));
	obs2->push_back("acElbowAngle", new IntervalDomain(rightArmPosMsg.elbowAngle));
	obs2->push_back("acForearmRollAngle", new IntervalDomain(rightArmPosMsg.forearmRollAngle));
	obs2->push_back("acWristPitchAngle", new IntervalDomain(rightArmPosMsg.wristPitchAngle));
	obs2->push_back("acWristRollAngle", new IntervalDomain(rightArmPosMsg.wristRollAngle));
	obs2->push_back("acGripperForceCmd", new IntervalDomain(rightArmPosMsg.gripperForceCmd));
	obs2->push_back("acGripperGapCmd", new IntervalDomain(rightArmPosMsg.gripperGapCmd));
	buff.push_back(obs2);
	_rightArmActive = false;
      }
    }
    unlock();
  }

  void ROSNode::get_end_effector_obs(std::vector<Observation*>& buff, TICK currentTick){
   
    if(_rightArmInit && _leftArmInit) {
      Observation* larm = get_left_end_effector_obs();
      if(larm != NULL) {
	buff.push_back(larm);
      }
      Observation* rarm = get_right_end_effector_obs();
      if(rarm != NULL) {
	buff.push_back(rarm);
      }
    }
  }

  Observation* ROSNode::get_right_end_effector_obs(){
    lock();
    ObservationByValue* obs = NULL;
    Frame f;
    ConvertArmToEndEffectorFrame(rightArmPosMsg,
				 tf.lookup("FRAMEID_ARM_R_SHOULDER"),
				 f);
    
    obs = new ObservationByValue("rightEndEffectorState", "EndEffectorState.Holds");
    obs->push_back("rot1_1", new IntervalDomain(f.M.data[0]));
    obs->push_back("rot1_2", new IntervalDomain(f.M.data[1]));
    obs->push_back("rot1_3", new IntervalDomain(f.M.data[2]));
    obs->push_back("rot2_1", new IntervalDomain(f.M.data[3]));
    obs->push_back("rot2_2", new IntervalDomain(f.M.data[4]));
    obs->push_back("rot2_3", new IntervalDomain(f.M.data[5]));
    obs->push_back("rot3_1", new IntervalDomain(f.M.data[6]));
    obs->push_back("rot3_2", new IntervalDomain(f.M.data[7]));
    obs->push_back("rot3_3", new IntervalDomain(f.M.data[8]));
    obs->push_back("x", new IntervalDomain(f.p.data[0]));
    obs->push_back("y", new IntervalDomain(f.p.data[1]));
    obs->push_back("z", new IntervalDomain(f.p.data[2]));
    unlock();
    return obs;
  }

 Observation* ROSNode::get_left_end_effector_obs(){
    lock();
    ObservationByValue* obs = NULL;
    Frame f;
    ConvertArmToEndEffectorFrame(leftArmPosMsg,
				 tf.lookup("FRAMEID_ARM_L_SHOULDER"),
				 f);
    
    obs = new ObservationByValue("leftEndEffectorState", "EndEffectorState.Holds");
    obs->push_back("rot1_1", new IntervalDomain(f.M.data[0]));
    obs->push_back("rot1_2", new IntervalDomain(f.M.data[1]));
    obs->push_back("rot1_3", new IntervalDomain(f.M.data[2]));
    obs->push_back("rot2_1", new IntervalDomain(f.M.data[3]));
    obs->push_back("rot2_2", new IntervalDomain(f.M.data[4]));
    obs->push_back("rot2_3", new IntervalDomain(f.M.data[5]));
    obs->push_back("rot3_1", new IntervalDomain(f.M.data[6]));
    obs->push_back("rot3_2", new IntervalDomain(f.M.data[7]));
    obs->push_back("rot3_3", new IntervalDomain(f.M.data[8]));
    obs->push_back("x", new IntervalDomain(f.p.data[0]));
    obs->push_back("y", new IntervalDomain(f.p.data[1]));
    obs->push_back("z", new IntervalDomain(f.p.data[2]));
    unlock();
    return obs;
  }

  void ROSNode::ConvertArmToEndEffectorFrame(const PR2Arm arm,
					     const unsigned int target_frame,
					     Frame& f) {
    //frame comes from forward kinematics
    PR2_kinematics* pr2_kin = new PR2_kinematics();
    JntArray q = JntArray(pr2_kin->nJnts);
    
    q(0) = arm.turretAngle;
    q(1) = arm.shoulderLiftAngle;
    q(2) = arm.upperarmRollAngle;
    q(3) = arm.elbowAngle;
    q(4) = arm.forearmRollAngle;
    q(5) = arm.wristPitchAngle;
    q(6) = arm.wristRollAngle;

    pr2_kin->FK(q,f);

    delete pr2_kin;

    //for now need to convert to funky shoulder frame
  //   libTF::TFPose aPose;
//     aPose.x = 0;
//     aPose.y = 0;
//     aPose.z = 0;
//     aPose.roll = 0;
//     aPose.pitch = 0;
//     aPose.yaw = 0;
//     aPose.time = 0;
//     aPose.frame = target_frame;

//     libTF::TFPose inShoulderFrame = tf.transformPose(FRAMEID_ODOM, aPose);

//     f.p.data[0] -= inShoulderFrame.x;
//     f.p.data[1] -= inShoulderFrame.y;
//     f.p.data[2] -= inShoulderFrame.z;
  }
  

  /*
  Observation* ROSNode::get_end_effector_obs() {
    lock();
    ObservationByValue* obs = NULL;
    
    //take current arm position
    //do transform to determine current end effector position in bizarre shoulder coordinates
    //see if we're inactive using the same trick above


  }
  */

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
    
    unsigned long long newScanTime = newscan.header.stamp.to_ull();

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
		global = this->tf.transformPose2D("FRAMEID_MAP", local);
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

    
    WavefrontPlanner::Instance()->SetObstacles(newScanTime,
					       this->laser_hitpts, 
					       this->laser_hitpts_len);  

    unlock(); 
  }

  /* Deprecated
  Observation* ROSNode::get_vc_obs(){
    lock();
    //ObservationByValue* obs = NULL;
    //obs = new ObservationByValue("baseGoal", "BaseGoal.Holds");
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
// 	obs = new ObservationByValue("wpc", "MoveBase.Inactive");
//       }
//       else {
// 	obs = new ObservationByValue("wpc", "MoveBase.Active");
// 	obs->push_back("x", new IntervalDomain(m_rcs_obs.goal.x));
// 	obs->push_back("y", new IntervalDomain(m_rcs_obs.goal.y));
//       }

//       debugMsg("ROSNode:WPC", "Received Observation:" << obs->toString());
//     }

//     unlock();
    // return obs;
  }
  */
}
