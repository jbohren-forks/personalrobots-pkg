/*
 * wavefront_player
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**

@mainpage

@htmlinclude manifest.html

@b exec_trex is a bare bones demonstration of using the TREX hybrid executive
for planning and control. It uses the EUROPA-2 constraint-based temporal planning library
to represent plan state, do planning, write models etc. Right now this is
a crude integration which will evolve to include more generalized integration patterns to
ROS nodes. Also, the build structure uses jam, and that allows builds for a variety
of versions. For now we will use the default development build.

<hr>

@section usage Usage
@verbatim
$ exec_trex_g_rt cfgFile
@endverbatim

@param cfgFile Is the TREX configuration file which defines initial inputs and goals.

@todo 

@par Example

@verbatim
$ exec_trex_g_rt exec.cfg
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "state"/Planner2DState : current planner state (e.g., goal reached, no
path)

Publishes to (name/type):
- @b "goal"/Planner2DGoal : goal for the robot.

@todo

<hr>

@section parameters ROS parameters

- None

 **/

#include "Nddl.hh"
#include "Executive.hh"
#include "RCSAdapter.hh"
#include "Token.hh"
#include <signal.h>
#include "WavefrontPlanner.hh"
#include <std_srvs/StaticMap.h>

#include <std_msgs/BaseVel.h>

TiXmlElement* root = NULL;
Clock* agentClock = NULL;
std::ofstream dbgFile("Debug.log");

using namespace std_msgs;


namespace TREX {

  /**
   * Executive class implementation.
   */

  /**
   * @brief Singleton accessor
   */
  ExecutiveId Executive::instance(){
    return s_id;
  }

  /**
   * @brief Sets up publish and subscribe topics for this node to integrate it
   * at the top to publish Execution Status updates and at the bottom to dispatch 
   * commands to the RCS and receive updates
   */
  Executive::Executive() : ros::node("trex"), 
			   m_id(this), 
			   m_initialized(1), 
			   m_state(UNDEFINED),
			   tf(*this, true, 200000000ULL), //nanoseconds
			   laser_maxrange(4.0),
			   laser_buffer_time(3.0)
  {
    s_id = m_id;
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
  puts("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    puts("request failed; trying again...");
    usleep(1000000);
  }
  printf("Received a %d X %d map @ %.3f m/pix\n",
         resp.map.width,
         resp.map.height,
         resp.map.resolution);
  char* mapdata;
  int sx, sy;
  sx = resp.map.width;
  sy = resp.map.height;
  // Convert to player format
  mapdata = new char[sx*sy];
  for(int i=0;i<sx*sy;i++)
  {
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
    

  }

  Executive::~Executive(){
    m_id.remove();
    s_id = ExecutiveId::noId();
  }

  /**
   * @brief Nothing to do here. The msg object will already have been updated and that is what we use
   * directly in synchronization
   */
  void Executive::rcs_cb(){
    m_initialized = true;
    //std::cout << "Received Update:" << toString(m_rcs_obs) << std::endl;
  }

  /**
   * @brief Called when an update is received in the Monitor during synchronization
   */
  void Executive::my_publish(const Observation& obs)
  {
    debugMsg("Executive:publish", obs.toString());
    //MsgToken msg;
    //msg.predicate = obs.toString();
    //ros::node::publish("navigator", msg);
  }

  /**
   * @brief Dispatch a token
   */
  void Executive::dispatchWaypoint(const TokenId& goal){
    static const LabelStr ACTIVE("WaypointController.Active");
    static const LabelStr X2("x");
    static const LabelStr Y2("y");

    if(goal->getPredicateName() == ACTIVE){
      Planner2DGoal msg;
      const IntervalDomain& x = goal->getVariable(X2)->lastDomain();
      const IntervalDomain& y = goal->getVariable(Y2)->lastDomain();

      checkError(x.isSingleton(), x.toString());
      checkError(y.isSingleton(), y.toString());

      // Fill outbound msg. Don't care about orientation
      msg.goal.x = x.getSingletonValue();
      msg.goal.y = y.getSingletonValue();
      msg.enable = true;
      std::cout << "Dispatching Goal: " <<  goal->toString() << " AS " << toString(msg) << std::endl;
      ros::node::publish("goal", msg);
    }
  }

  void Executive::dispatchVel(const TokenId& cmd_vel){
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
      //std::cout << "Sending vel x " << mbv.vx << " y " << mbv.vw << std::endl;
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

  void Executive::get_obs(std::vector<Observation*>& buff){
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

  Observation* Executive::get_vs_obs(){
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

      //std::cout << "New global pose x " << global_pose.x
      //		<< " y " << global_pose.y << endl;

    } catch(libTF::TransformReference::LookupException& ex) {
      puts("no global->local Tx yet");
      return NULL;
    } catch(libTF::Quaternion3D::ExtrapolateException& ex) {
      cout << "libTF::Quaternion3D::ExtrapolateException occured\n";
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

  Observation* Executive::get_vc_obs(){
    lock();
    //ObservationByValue* obs = NULL;
    //obs = new ObservationByValue("vcom", "VelCommander.Holds");
    //obs->push_back("cmd_x", new IntervalDomain(m_localizedOdomMsg.vel.x));
    //obs->push_back("cmd_th", new IntervalDomain(m_localizedOdomMsg.vel.th));
    unlock();
    //std::cout << "Received Observation:" << obs->toString() << std::endl;
    return NULL;
  }
  

  Observation* Executive::get_wpc_obs(){
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

//       std::cout << "Received Observation:" << obs->toString() << std::endl;
//     }

//     unlock();
    // return obs;
  }

  //this doesn't actually generate observations for now
  void Executive::get_laser_obs() {
    lock();
    unlock();  
  }

  bool Executive::isInitialized() const {
    return m_initialized;
  }

  void Executive::lock(){
    pthread_mutex_lock(&m_lock);
  }

  void Executive::unlock(){
    pthread_mutex_unlock(&m_lock);
  }

  void Executive::requireROS(){
    //advertise<MsgToken>("navigator");
    advertise<Planner2DState>("state");
    advertise<Polyline2D>("gui_path");
    advertise<Polyline2D>("gui_laser");
    advertise<Planner2DGoal>("goal");
    advertise<BaseVel>("cmd_vel");
    //subscribe("state", m_rcs_obs, &Executive::rcs_cb);
    subscribe("scan", laserMsg, &Executive::laserReceived);
    //subscribe("localizedpose", m_localizedOdomMsg, &Executive::localizedOdomReceived);
    m_initialized = false;
  }

 //  void Executive::localizedOdomReceived() {
    
//     std::cout << "Got localized odom vel x " << this->m_localizedOdomMsg.vel.x
// 	      << " y " << this->m_localizedOdomMsg.vel.y 
// 	      << " th " << this->m_localizedOdomMsg.vel.th << endl; 
    
//   }

  void Executive::laserReceived() {


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
	    catch(libTF::Quaternion3D::ExtrapolateException& ex)
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



/**
 * @brief Handle cleanup on process termination signals.
 */
void signalHandler(int signalNo){
  std::cout << "Handling signal..." << std::endl;
  exit(0);
}

/**
 * @brief Handle cleanup on exit
 */
void cleanup(){
  std::cout << "Cleaning up..." << std::endl;

  // Terminate the agent
  Agent::terminate();

  Clock::sleep(3);

  Agent::cleanupLog();
}

ExecutiveId Executive::s_id;

int main(int argc, char **argv)
{
  signal(SIGINT,  &signalHandler);
  signal(SIGTERM, &signalHandler);
  signal(SIGQUIT, &signalHandler);
  signal(SIGKILL, &signalHandler);
  atexit(&cleanup);

  if (argc != 2) {
    std::cerr << "Invalid argument list: Usage: ros_exec configFile" << std::endl;
    return -1;
  }

  ros::init(argc, argv);

  LogManager::instance();

  initROSExecutive();

  Executive t;

  NDDL::loadSchema();

  char * configFile = argv[1];
  root = LogManager::initXml( configFile );
  DebugMessage::setStream(dbgFile);

  // Allocate a real time clock with 1 second per tick
  agentClock = new RealTimeClock(0.25);

  // Allocate the agent
  debugMsg("Executive", "Initializing the agent");
  Agent::initialize(*root, *agentClock);

  // Wait till we get a message before starting the agent
  while(!t.isInitialized() && t.ok()){
    std::cout << "Waiting.." << std::endl;
    sleep(1);
  }

  std::cout << "Starting TREX" << std::endl;
  try{
    debugMsg("ALWAYS", "Executing the agent");
    Agent::instance()->run();
  }
  catch(void*){
    debugMsg("Executive", "Caught unexpected exception.");
  }

  t.shutdown();
  
  //delete WavefrontPlanner::Instance();

  return 0;
}

