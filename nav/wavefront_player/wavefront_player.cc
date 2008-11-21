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

@b wavefront_player is a path-planning system for a robot moving in 2D.  It
implements the wavefront algorithm (described in various places; Latombe's
book is a good reference), which uses dynamic programming over an occupancy
grid map to find the lowest-cost path from the robot's pose to the goal.

This node uses part of the Player @b wavefront driver.  For detailed
documentation, consult <a
href="http://playerstage.sourceforge.net/doc/Player-cvs/player/group__driver__wavefront.html">Player
wavefront documentation</a>.  Note that this node does not actually wrap the @b
wavefront driver, but rather calls into the underlying library, @b
libwavefront_standalone.

The planning algorithm assumes that the robot is circular and holonomic.
Under these assumptions, it efficiently dilates obstacles and then plans
over the map as if the robot occupied a single grid cell.

If provided, laser scans are used to temporarily modify the map.  In this
way, the planner can avoid obstacles that are not in the static map.

The node also contains a controller that generates velocities for a
differential drive robot.  The intended usage is to run the node at a
modest rate, e.g., 20Hz, allowing it to replan and generate new controls
every cycle.  The controller ignores dynamics (i.e., assumes infinite
acceleration); this becomes a problem with robots that take non-trivial
time to slow down.

<hr>

@section usage Usage
@verbatim
$ wavefront_player
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "localizedpose"/RobotBase2DOdom : robot's map pose.  Only the position information is used (velocity is ignored).
- @b "goal"/Planner2DGoal : goal for the robot.
- @b "scan"/LaserScan : laser scans.  Used to temporarily modify the map for dynamic obstacles.

Publishes to (name / type):
- @b "cmd_vel"/BaseVel : velocity commands to robot
- @b "state"/Planner2DState : current planner state (e.g., goal reached, no
path)
- @b "gui_path"/Polyline2D : current global path (for visualization)
- @b "gui_laser"/Polyline2D : re-projected laser scans (for visualization)

@todo Start using libTF for transform management:
  - subscribe to odometry and use transform to recover map pose.

<hr>

@section parameters ROS parameters

- None

@todo There are an enormous number of parameters, values for which are all
currently hardcoded. These should be exposed as ROS parameters.  In
particular, robot radius and safety distance must be changed for each
robot.

 **/
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>

#include <list>

// The Player lib we're using
#include <libstandalone_drivers/plan.h>

// roscpp
#include <ros/node.h>

// The messages that we'll use
#include <std_msgs/Planner2DState.h>
#include <std_msgs/Planner2DGoal.h>
#include <std_msgs/BaseVel.h>
#include <std_msgs/PointCloud.h>
#include <std_msgs/LaserScan.h>
#include <std_srvs/StaticMap.h>

// For GUI debug
#include <std_msgs/Polyline2D.h>

// For transform support
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

//Laser projection
#include "laser_scan_utils/laser_scan.h"

// For time support
#include <ros/time.h>

#define ANG_NORM(a) atan2(sin((a)),cos((a)))
#define DTOR(a) ((a)*M_PI/180.0)
#define RTOD(a) ((a)*180.0/M_PI)
#define SIGN(x) (((x) < 0.0) ? -1 : 1)

// A bunch of x,y points, with a timestamp
typedef struct
{
  double* pts;
  size_t pts_num;
  ros::Time ts;
} laser_pts_t;

class WavefrontNode: public ros::node
{
  private:
    // Plan object
    plan_t* plan;
    // Our state
    enum
    {
      NO_GOAL,
      NEW_GOAL,
      PURSUING_GOAL,
      //STUCK,
      REACHED_GOAL
    } planner_state;

  tf::Stamped<btTransform> global_pose;

    // If we can't reach the goal, note when it happened and keep trying a bit
    //ros::Time stuck_time;
    // Are we enabled?
    bool enable;
    // Current goal
    double goal[3];
    // Direction that we're rotating in order to assume the goal
    // orientation
    int rotate_dir;

    bool printed_warning;

    // Have we already stopped the robot?
    bool stopped;

    // Planning parameters
    double robot_radius;
    double safety_dist;
    double max_radius;
    double dist_penalty;
    double plan_halfwidth;
    double dist_eps;
    double ang_eps;
    double cycletime;

    // Map update paramters (for adding obstacles)
    double laser_maxrange;
    ros::Duration laser_buffer_time;
    std::list<laser_pts_t> laser_scans;
    double* laser_hitpts;
    size_t laser_hitpts_len, laser_hitpts_size;

    // Controller paramters
    double lookahead_maxdist;
    double lookahead_distweight;
    double tvmin, tvmax, avmin, avmax, amin, amax;

    // incoming/outgoing messages
    std_msgs::Planner2DGoal goalMsg;
    //MsgRobotBase2DOdom odomMsg;
    std_msgs::Polyline2D polylineMsg;
    std_msgs::Polyline2D pointcloudMsg;
    std_msgs::Planner2DState pstate;
    //MsgRobotBase2DOdom prevOdom;
    bool firstodom;

    tf::MessageNotifier<std_msgs::LaserScan>* scan_notifier;

    // Lock for access to class members in callbacks
    ros::thread::mutex lock;

    // Message callbacks
    void goalReceived();
    void laserReceived(const tf::MessageNotifier<std_msgs::LaserScan>::MessagePtr& message);

    // Internal helpers
    void sendVelCmd(double vx, double vy, double vth);

    laser_scan::LaserProjection projector_;
  public:
    // Transform listener
    tf::TransformListener tf;

    WavefrontNode();
    virtual ~WavefrontNode();

    // Stop the robot
    void stopRobot();
    // Execute a planning cycle
    void doOneCycle();
    // Sleep for the remaining time of the cycle
    void sleep(double loopstart);
    // Compare two poses, tell whether they are close enough to be
    // considered the same, with tolerance
    bool comparePoses(double x1, double y1, double a1,
                      double x2, double y2, double a2);
};

#define _xy_tolerance 1e-3
#define _th_tolerance 1e-3
#define USAGE "USAGE: wavefront_player"

int
main(int argc, char** argv)
{
  ros::init(argc,argv);

  WavefrontNode wn;

  struct timeval curr;
  while(wn.ok())
  {
    gettimeofday(&curr,NULL);
    wn.doOneCycle();
    wn.sleep(curr.tv_sec+curr.tv_usec/1e6);
  }
  ros::fini();
  return(0);
}

WavefrontNode::WavefrontNode() :
        ros::node("wavfront_player"),
        planner_state(NO_GOAL),
        enable(true),
        rotate_dir(0),
        printed_warning(false),
        stopped(false),
        robot_radius(0.175), // overridden by param retrieval below!
        safety_dist(0.05),
        max_radius(2.0),
        dist_penalty(2.0),   // overridden by param retrieval below!
        plan_halfwidth(5.0),
        dist_eps(1.0),       // overridden by param retrieval below!
        ang_eps(DTOR(4.0)),
        cycletime(0.1),
        laser_maxrange(4.0),
        laser_buffer_time(3.0),
        lookahead_maxdist(2.0),
        lookahead_distweight(5.0),
        tvmin(0.2),
        tvmax(0.75),
        avmin(DTOR(10.0)),
        avmax(DTOR(80.0)),
        amin(DTOR(10.0)),
        amax(DTOR(40.0)),
        tf(*this, true, (uint64_t)10000000000ULL)// cache for 10 sec, no extrapolation
        //tf(*this, true, (uint64_t)200000000ULL, (uint64_t)200000000ULL) //nanoseconds
{
  // Initialize global pose. Will be set in control loop based on actual data.
  ///\todo does this need to be initialized?  global_pose.setIdentity();

  // set a few parameters. leave defaults just as in the ctor initializer list
  param("dist_eps", dist_eps, 1.0);
  param("robot_radius", robot_radius, 0.175);
  param("dist_penalty", dist_penalty, 2.0);

  // get map via RPC
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

  assert((this->plan = plan_alloc(this->robot_radius+this->safety_dist,
                                  this->robot_radius+this->safety_dist,
                                  this->max_radius,
                                  this->dist_penalty,0.5)));

  // allocate space for map cells
  assert(this->plan->cells == NULL);
  assert((this->plan->cells =
          (plan_cell_t*)realloc(this->plan->cells,
                                (sx * sy * sizeof(plan_cell_t)))));

  // Copy over obstacle information from the image data that we read
  for(int j=0;j<sy;j++)
  {
    for(int i=0;i<sx;i++)
      this->plan->cells[i+j*sx].occ_state = mapdata[i+j*sx];
  }
  delete[] mapdata;

  this->plan->scale = resp.map.resolution;
  this->plan->size_x = sx;
  this->plan->size_y = sy;
  this->plan->origin_x = 0.0;
  this->plan->origin_y = 0.0;

  // Do initialization
  plan_init(this->plan);

  // Compute cspace over static map
  plan_compute_cspace(this->plan);
  printf("done computing c-space\n");

  this->laser_hitpts_size = this->laser_hitpts_len = 0;
  this->laser_hitpts = NULL;

  this->firstodom = true;

  advertise<std_msgs::Planner2DState>("state",1);
  advertise<std_msgs::Polyline2D>("gui_path",1);
  advertise<std_msgs::Polyline2D>("gui_laser",1);
  advertise<std_msgs::BaseVel>("cmd_vel",1);
  subscribe("goal", goalMsg, &WavefrontNode::goalReceived,1);

  scan_notifier = new tf::MessageNotifier<std_msgs::LaserScan>(&tf, this, boost::bind(&WavefrontNode::laserReceived, this, _1), "scan", "map", 1);
}

WavefrontNode::~WavefrontNode()
{
  plan_free(this->plan);

  delete scan_notifier;
}

void
WavefrontNode::goalReceived()
{
  this->lock.lock();
  // Got a new goal message; handle it
  this->enable = goalMsg.enable;

  if(this->enable){
    printf("got new goal: %.3f %.3f %.3f\n",
	   goalMsg.goal.x,
	   goalMsg.goal.y,
	   RTOD(goalMsg.goal.th));

    // Populate goal data
    this->goal[0] = goalMsg.goal.x;
    this->goal[1] = goalMsg.goal.y;
    this->goal[2] = goalMsg.goal.th;
    this->planner_state = NEW_GOAL;

    // Set state for actively pursuing a goal
    this->pstate.active = 1;
    this->pstate.valid = 0;
    this->pstate.done = 0;
  }
  else {
    // Clear goal data
    this->planner_state = NO_GOAL;

    // Set state inactive
    this->pstate.active = 0;
  }

  double yaw,pitch,roll;
  btMatrix3x3 mat =  global_pose.getBasis();
  mat.getEulerZYX(yaw, pitch, roll);

  // Fill out and publish response
  this->pstate.pos.x = global_pose.getOrigin().x();
  this->pstate.pos.y = global_pose.getOrigin().y();
  this->pstate.pos.th = yaw;
  this->pstate.goal.x = this->goal[0];
  this->pstate.goal.y = this->goal[1];
  this->pstate.goal.th = this->goal[2];
  this->pstate.waypoint.x = 0.0;
  this->pstate.waypoint.y = 0.0;
  this->pstate.waypoint.th = 0.0;
  this->pstate.set_waypoints_size(0);
  this->pstate.waypoint_idx = -1;
  publish("state",this->pstate);
  this->lock.unlock();
}

void
WavefrontNode::laserReceived(const tf::MessageNotifier<std_msgs::LaserScan>::MessagePtr& message)
{
	// Assemble a point cloud, in the laser's frame
	std_msgs::PointCloud local_cloud;
	projector_.projectLaser(*message, local_cloud, laser_maxrange);

	// Convert to a point cloud in the map frame
	std_msgs::PointCloud global_cloud;

	try
	{
		this->tf.transformPointCloud("map", local_cloud, global_cloud);
	}
	catch(tf::LookupException& ex)
	{
		puts("no global->local Tx yet (point cloud)");
		printf("%s\n", ex.what());
		return;
	}
	catch(tf::ConnectivityException& ex)
	{
		puts("no global->local Tx yet (point cloud)");
		printf("%s\n", ex.what());
		return;
	}

	// Convert from point cloud to array of doubles formatted XYXYXY...
	// TODO: increase efficiency by reducing number of data transformations
	laser_pts_t pts;
	pts.pts_num = global_cloud.get_pts_size();
	pts.pts = new double[pts.pts_num*2];
	assert(pts.pts);
	pts.ts = global_cloud.header.stamp;
	for(unsigned int i=0;i<global_cloud.get_pts_size();i++)
	{
		pts.pts[2*i] = global_cloud.pts[i].x;
		pts.pts[2*i+1] = global_cloud.pts[i].y;
	}

	// Add the new point set to our list
	this->laser_scans.push_back(pts);

	// Remove anything that's too old from the laser_scans list
	// Also count how many points we have
	unsigned int hitpt_cnt=0;
	for(std::list<laser_pts_t>::iterator it = this->laser_scans.begin();
			it != this->laser_scans.end();
			it++)
	{
		if((message->header.stamp - it->ts) > this->laser_buffer_time)
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
  this->lock.lock();
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
  publish("gui_laser",this->pointcloudMsg);
  this->lock.unlock();
}

void
WavefrontNode::stopRobot()
{
  if(!this->stopped)
  {
    // TODO: should we send more than once, or perhaps use RPC for this?
    this->sendVelCmd(0.0,0.0,0.0);
    this->stopped = true;
  }
}

// Declare this globally, so that it never gets desctructed (message
// desctruction causes master disconnect)
std_msgs::BaseVel* cmdvel;

void
WavefrontNode::sendVelCmd(double vx, double vy, double vth)
{
  if(!cmdvel)
    cmdvel = new std_msgs::BaseVel();
  cmdvel->vx = vx;
  cmdvel->vw = vth;
  this->ros::node::publish("cmd_vel", *cmdvel);
  if(vx || vy || vth)
    this->stopped = false;
}

// Execute a planning cycle
void
WavefrontNode::doOneCycle()
{
  // Get the current robot pose in the map frame
  //convert!

  tf::Stamped<tf::Pose> robotPose;
  robotPose.setIdentity();
  robotPose.frame_id_ = "base_link";
  robotPose.stamp_ = ros::Time((uint64_t)0ULL); // request most recent pose
  //robotPose.time = laserMsg.header.stamp.sec * (uint64_t)1000000000ULL +
  //        laserMsg.header.stamp.nsec; ///HACKE FIXME we should be able to get time somewhere else
  try
  {
    this->tf.transformPose("map", robotPose, global_pose);
  }
  catch(tf::LookupException& ex)
  {
    puts("L: no global->local Tx yet");
    this->stopRobot();
    return;
  }
  catch(tf::ConnectivityException& ex)
    {
      puts("C: no global->local Tx yet");
      printf("%s\n", ex.what());
      this->stopRobot();
      return;
    }
  catch(tf::ExtrapolationException& ex)
  {
    // this should never happen
    puts("WARNING: extrapolation failed!");
    printf("%s",ex.what());
    this->stopRobot();
    return;
  }

  if(!this->enable)
  {
    this->stopRobot();
    return;
  }

  this->lock.lock();
  switch(this->planner_state)
  {
    case NO_GOAL:
      //puts("no goal");
      this->stopRobot();
      break;
    case REACHED_GOAL:
      //puts("still done");
      this->stopRobot();
      break;
    case NEW_GOAL:
    case PURSUING_GOAL:
    //case STUCK:
      {

        double yaw,pitch,roll; //fixme make cleaner namespace
        btMatrix3x3 mat =  global_pose.getBasis();
        mat.getEulerZYX(yaw, pitch, roll);

        // Are we done?
        if(plan_check_done(this->plan,
                           global_pose.getOrigin().x(), global_pose.getOrigin().y(), yaw,
                           this->goal[0], this->goal[1], this->goal[2],
                           this->dist_eps, this->ang_eps))
        {
          puts("done");
          this->rotate_dir = 0;
          this->stopRobot();
          this->planner_state = REACHED_GOAL;
          break;
        }

        //printf("setting %d hit points\n", this->laser_hitpts_len);
        plan_set_obstacles(this->plan,
                           this->laser_hitpts,
                           this->laser_hitpts_len);

        bool plan_valid = true;

        // Try a local plan
        if((this->planner_state == NEW_GOAL) ||
           (plan_do_local(this->plan, global_pose.getOrigin().x(), global_pose.getOrigin().y(),
                         this->plan_halfwidth) < 0))
        {
          // Fallback on global plan
          if(plan_do_global(this->plan, global_pose.getOrigin().x(), global_pose.getOrigin().y(),
                            this->goal[0], this->goal[1]) < 0)
          {
            // no global plan
            plan_valid = false;
            this->stopRobot();

            /*
            if (this->planner_state != STUCK)
            {
              printf("we're stuck. let's let the laser buffer empty "
                     "and see if that will free us.\n");
              this->planner_state = STUCK;
              this->stuck_time = ros::Time::now();
            }
            else
            {
              if (ros::Time::now() >
                  this->stuck_time + 1.5 * this->laser_buffer_time.to_double())
              {
                puts("global plan failed");
                this->planner_state = NO_GOAL; //NO_GOAL;
              }
            }
            */
          }
          else
          {
            // global plan succeeded; now try the local plan again
            this->printed_warning = false;
            if(plan_do_local(this->plan, global_pose.getOrigin().x(), global_pose.getOrigin().y(),
                             this->plan_halfwidth) < 0)
            {
              // no local plan; better luck next time through
              this->stopRobot();
              plan_valid = false;
            }
          }
        }

        if(this->planner_state == NEW_GOAL) // || this->planner_state == STUCK)
          this->planner_state = PURSUING_GOAL;

        if(!plan_valid)
          break;

        // We have a valid local plan.  Now compute controls
        double vx, va;

        //    double yaw,pitch,roll; //used temporarily earlier fixme make cleaner
        //btMatrix3x3
        mat =  global_pose.getBasis();
        mat.getEulerZYX(yaw, pitch, roll);

        if(plan_compute_diffdrive_cmds(this->plan, &vx, &va,
                                       &this->rotate_dir,
                                       global_pose.getOrigin().x(), global_pose.getOrigin().y(),
                                       yaw,
                                       this->goal[0], this->goal[1],
                                       this->goal[2],
                                       this->dist_eps, this->ang_eps,
                                       this->lookahead_maxdist,
                                       this->lookahead_distweight,
                                       this->tvmin, this->tvmax,
                                       this->avmin, this->avmax,
                                       this->amin, this->amax) < 0)
        {
          this->stopRobot();
          puts("Failed to find a carrot");
          break;
        }

        assert(this->plan->path_count);

        this->polylineMsg.set_points_size(this->plan->path_count);
        this->polylineMsg.color.r = 0;
        this->polylineMsg.color.g = 1.0;
        this->polylineMsg.color.b = 0;
        this->polylineMsg.color.a = 0;
        for(int i=0;i<this->plan->path_count;i++)
        {
          this->polylineMsg.points[i].x =
                  PLAN_WXGX(this->plan,this->plan->path[i]->ci);
          this->polylineMsg.points[i].y =
                  PLAN_WYGY(this->plan,this->plan->path[i]->cj);
        }
        publish("gui_path", polylineMsg);

        //printf("computed velocities: %.3f %.3f\n", vx, RTOD(va));
        this->sendVelCmd(vx, 0.0, va);

        break;
      }
    default:
      assert(0);
  }
  double yaw,pitch,roll;
  btMatrix3x3 mat =  global_pose.getBasis();
  mat.getEulerZYX(yaw, pitch, roll);

  this->pstate.active = (this->enable &&
			 (this->planner_state == PURSUING_GOAL)) ? 1 : 0;
  this->pstate.valid = (this->plan->path_count > 0) ? 1 : 0;
  this->pstate.done = (this->planner_state == REACHED_GOAL) ? 1 : 0;
  this->pstate.pos.x = global_pose.getOrigin().x();
  this->pstate.pos.y = global_pose.getOrigin().y();
  this->pstate.pos.th = yaw;
  this->pstate.goal.x = this->goal[0];
  this->pstate.goal.y = this->goal[1];
  this->pstate.goal.th = this->goal[2];
  this->pstate.waypoint.x = 0.0;
  this->pstate.waypoint.y = 0.0;
  this->pstate.waypoint.th = 0.0;
  this->pstate.set_waypoints_size(0);
  this->pstate.waypoint_idx = -1;

  publish("state",this->pstate);

  this->lock.unlock();
}

// Sleep for the remaining time of the cycle
void
WavefrontNode::sleep(double loopstart)
{
  struct timeval curr;
  double currt,tdiff;
  gettimeofday(&curr,NULL);
  currt = curr.tv_sec + curr.tv_usec/1e6;
  tdiff = this->cycletime - (currt-loopstart);
  if(tdiff <= 0.0)
    puts("Wavefront missed deadline and not sleeping; check machine load");
  else
    usleep((unsigned int)rint(tdiff*1e6));
}

bool
WavefrontNode::comparePoses(double x1, double y1, double a1,
                            double x2, double y2, double a2)
{
  bool res;
  if((fabs(x2-x1) <= _xy_tolerance) &&
     (fabs(y2-y1) <= _xy_tolerance) &&
     (fabs(ANG_NORM(ANG_NORM(a2)-ANG_NORM(a1))) <= _th_tolerance))
    res = true;
  else
    res = false;
  return(res);
}
