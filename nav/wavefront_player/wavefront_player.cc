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
$ wavefront_player <map> <res> [standard ROS args]
@endverbatim

@param map An image file to load as an occupancy grid map.  The robot will be localized against this map using laser scans.  The lower-left pixel of the map is assigned the pose (0,0,0).
@param res The resolution of the map, in meters, pixel.

@todo Remove the map and res arguments in favor map retrieval via ROSRPC.

@par Example

@verbatim
$ wavefront_player mymap.png 0.1
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
#include <std_msgs/MsgPlanner2DState.h>
#include <std_msgs/MsgPlanner2DGoal.h>
#include <std_msgs/MsgBaseVel.h>
//#include <std_msgs/MsgRobotBase2DOdom.h>
#include <std_msgs/MsgLaserScan.h>
#include <std_msgs/MsgPose2DFloat32.h>

// For GUI debug
#include <std_msgs/MsgPolyline2D.h>

// For transform support
#include <rosTF/rosTF.h>

// For time support
#include <ros/time.h>

#define ANG_NORM(a) atan2(sin((a)),cos((a)))
#define DTOR(a) ((a)*M_PI/180.0)
#define RTOD(a) ((a)*180.0/M_PI)
#define SIGN(x) (((x) < 0.0) ? -1 : 1)

//void draw_cspace(plan_t* plan, const char* fname);
//void draw_path(plan_t* plan, double lx, double ly, const char* fname);
#include <gdk-pixbuf/gdk-pixbuf.h>
// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
// computes the signed minimum difference between the two angles.
int read_map_from_image(int* size_x, int* size_y, char** mapdata, 
       			const char* fname, int negate);

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
      REACHED_GOAL
    } planner_state;
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
    std::list<MsgLaserScan> buffered_laser_scans;
    std::list<laser_pts_t> laser_scans;
    double* laser_hitpts;
    size_t laser_hitpts_len, laser_hitpts_size;

    // Controller paramters
    double lookahead_maxdist;
    double lookahead_distweight;
    double tvmin, tvmax, avmin, avmax, amin, amax;

    // incoming/outgoing messages
    MsgPlanner2DGoal goalMsg;
    //MsgRobotBase2DOdom odomMsg;
    MsgLaserScan laserMsg;
    MsgPolyline2D polylineMsg;
    MsgPolyline2D pointcloudMsg;
    MsgPlanner2DState pstate;
    std::vector<MsgLaserScan> laserScans;
    //MsgRobotBase2DOdom prevOdom;
    bool firstodom;

    // Lock for access to class members in callbacks
    ros::thread::mutex lock;

    // Message callbacks
    void goalReceived();
    void laserReceived();

    // Internal helpers
    void sendVelCmd(double vx, double vy, double vth);

  public:
    // Transform client
    rosTFClient tf;

    WavefrontNode(char* fname, double res);
    ~WavefrontNode();
    
    // Stop the robot
    void stopRobot();
    // Execute a planning cycle
    void doOneCycle();
    // Sleep for the remaining time of the cycle
    void sleep(double loopstart);
};

#define USAGE "USAGE: wavefront_player <map.png> <res>"

int
main(int argc, char** argv)
{
  if(argc < 3)
  {
    puts(USAGE);
    exit(-1);
  }

  ros::init(argc,argv);

  WavefrontNode wn(argv[1],atof(argv[2]));

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

WavefrontNode::WavefrontNode(char* fname, double res) : 
        ros::node("wavfront_player"),
        planner_state(NO_GOAL),
        enable(true),
        rotate_dir(0),
        printed_warning(false),
        stopped(false),
        robot_radius(0.175),
        safety_dist(0.05),
        max_radius(2.0),
        dist_penalty(2.0),
        plan_halfwidth(5.0),
        dist_eps(1.0),
        ang_eps(DTOR(10.0)),
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
        tf(*this, true)
{
  // TODO: get map via RPC
  char* mapdata;
  int sx, sy;
  assert(read_map_from_image(&sx, &sy, &mapdata, fname, 0) == 0);

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
  free(mapdata);

  this->plan->scale = res;
  this->plan->size_x = sx;
  this->plan->size_y = sy;
  this->plan->origin_x = 0.0;
  this->plan->origin_y = 0.0;

  // Do initialization
  plan_init(this->plan);

  // Compute cspace over static map
  plan_compute_cspace(this->plan);

  this->laser_hitpts_size = this->laser_hitpts_len = 0;
  this->laser_hitpts = NULL;

  this->firstodom = true;

  // Static robot->laser transform
  // TODO: get this info via ROS somehow
  this->tf.setWithEulers(FRAMEID_LASER,
                         FRAMEID_ROBOT,
                         0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

  advertise<MsgPlanner2DState>("state");
  advertise<MsgPolyline2D>("gui_path");
  advertise<MsgPolyline2D>("gui_laser");
  advertise<MsgBaseVel>("cmd_vel");
  subscribe("goal", goalMsg, &WavefrontNode::goalReceived);
  subscribe("scan", laserMsg, &WavefrontNode::laserReceived);
}

WavefrontNode::~WavefrontNode()
{
  plan_free(this->plan);
}

void 
WavefrontNode::goalReceived()
{
  this->lock.lock();
  // Got a new goal message; handle it
  this->enable = goalMsg.enable;
  printf("got new goal: %.3f %.3f %.3f\n", 
         goalMsg.goal.x,
         goalMsg.goal.y,
         RTOD(goalMsg.goal.th));

  if(this->enable)
  {
    this->goal[0] = goalMsg.goal.x;
    this->goal[1] = goalMsg.goal.y;
    this->goal[2] = goalMsg.goal.th;
    this->planner_state = NEW_GOAL;
  }
  this->lock.unlock();
}

void
WavefrontNode::laserReceived()
{
  // Copy and push this scan into the list of buffered scans
  MsgLaserScan newscan;
  // Do a deep copy
  newscan.header.stamp.sec = laserMsg.header.stamp.sec;
  newscan.header.stamp.nsec = laserMsg.header.stamp.nsec;
  newscan.header.frame_id = laserMsg.header.frame_id;
  newscan.range_max = laserMsg.range_max;
  newscan.angle_min = laserMsg.angle_min;
  newscan.angle_max = laserMsg.angle_max;
  newscan.angle_increment = laserMsg.angle_increment;
  newscan.set_ranges_size(laserMsg.get_ranges_size());
  memcpy(newscan.ranges,laserMsg.ranges,laserMsg.get_ranges_size()*sizeof(float));
  this->buffered_laser_scans.push_back(newscan);

  // Iterate through the buffered scans, trying to interpolate each one
  for(std::list<MsgLaserScan>::iterator it = this->buffered_laser_scans.begin();
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
    local.time = it->header.stamp.sec * 1000000000ULL + 
            it->header.stamp.nsec;
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
        //puts("extrapolation required");
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
      delete[] it->ranges;
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
MsgBaseVel* cmdvel;

void
WavefrontNode::sendVelCmd(double vx, double vy, double vth)
{
  if(!cmdvel)
    cmdvel = new MsgBaseVel();
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
  if(!this->enable)
  {
    this->stopRobot();
    return;
  }
  
  // Get the current robot pose in the map frame
  libTF::TFPose2D robotPose, global_pose;
  robotPose.x = 0;
  robotPose.y = 0;
  robotPose.yaw = 0;
  robotPose.frame = FRAMEID_ROBOT;
  //robotPose.time = 0; // request most recent pose
  robotPose.time = laserMsg.header.stamp.sec * 1000000000ULL + 
          laserMsg.header.stamp.nsec; ///HACKE FIXME we should be able to get time somewhere else
  try
  {
    global_pose = this->tf.transformPose2D(FRAMEID_MAP, robotPose);
  }
  catch(libTF::TransformReference::LookupException& ex)
  {
    puts("no global->local Tx yet");
    this->stopRobot();
    return;
  }
  catch(libTF::Quaternion3D::ExtrapolateException& ex)
  {
    // this should never happen
    puts("WARNING: extrapolation failed!");
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
      {
        // Are we done?
        if(plan_check_done(this->plan,
                           global_pose.x, global_pose.y, global_pose.yaw,
                           this->goal[0], this->goal[1], this->goal[2],
                           this->dist_eps, this->ang_eps))
        {
          puts("done");
          this->stopRobot();
          this->planner_state = REACHED_GOAL;
          break;
        }

        //printf("setting %d hit points\n", this->laser_hitpts_len);
        plan_set_obstacles(this->plan, 
                           this->laser_hitpts, 
                           this->laser_hitpts_len);

        // Try a local plan
        if((this->planner_state == NEW_GOAL) ||
           (plan_do_local(this->plan, global_pose.x, global_pose.y,
                         this->plan_halfwidth) < 0))
        {
          // Fallback on global plan
          if(plan_do_global(this->plan, global_pose.x, global_pose.y,
                            this->goal[0], this->goal[1]) < 0)
          {
            // no global plan
            this->stopRobot();

            //if(!this->printed_warning)
            //{
              puts("global plan failed");
              //this->printed_warning = true;
            //}
            break;
          }
          else
          {
            // global plan succeeded; now try the local plan again
            this->printed_warning = false;
            if(plan_do_local(this->plan, global_pose.x, global_pose.y, 
                             this->plan_halfwidth) < 0)
            {
              // no local plan; better luck next time through
              this->stopRobot();
              break;
            }
          }
        }

        if(this->planner_state == NEW_GOAL)
          this->planner_state = PURSUING_GOAL;

        // We have a valid local plan.  Now compute controls
        double vx, va;
        if(plan_compute_diffdrive_cmds(this->plan, &vx, &va,
                                       &this->rotate_dir,
                                       global_pose.x, global_pose.y,
                                       global_pose.yaw,
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

  this->pstate.active = (this->enable &&
			 (this->planner_state == PURSUING_GOAL)) ? 1 : 0;
  this->pstate.valid = (this->plan->path_count > 0) ? 1 : 0;
  this->pstate.done = (this->planner_state == REACHED_GOAL) ? 1 : 0;
  this->pstate.pos.x = global_pose.x;
  this->pstate.pos.y = global_pose.y;
  this->pstate.pos.th = global_pose.yaw;
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

int
read_map_from_image(int* size_x, int* size_y, char** mapdata, 
                    const char* fname, int negate)
{
  GdkPixbuf* pixbuf;
  guchar* pixels;
  guchar* p;
  int rowstride, n_channels, bps;
  GError* error = NULL;
  int i,j,k;
  double occ;
  int color_sum;
  double color_avg;

  // Initialize glib
  g_type_init();

  printf("MapFile loading image file: %s...", fname);
  fflush(stdout);

  // Read the image
  if(!(pixbuf = gdk_pixbuf_new_from_file(fname, &error)))
  {
    printf("failed to open image file %s", fname);
    return(-1);
  }

  *size_x = gdk_pixbuf_get_width(pixbuf);
  *size_y = gdk_pixbuf_get_height(pixbuf);

  assert(*mapdata = (char*)malloc(sizeof(char) * (*size_x) * (*size_y)));

  rowstride = gdk_pixbuf_get_rowstride(pixbuf);
  bps = gdk_pixbuf_get_bits_per_sample(pixbuf)/8;
  n_channels = gdk_pixbuf_get_n_channels(pixbuf);
  //if(gdk_pixbuf_get_has_alpha(pixbuf))
    //n_channels++;

  // Read data
  pixels = gdk_pixbuf_get_pixels(pixbuf);
  for(j = 0; j < *size_y; j++)
  {
    for (i = 0; i < *size_x; i++)
    {
      p = pixels + j*rowstride + i*n_channels*bps;
      color_sum = 0;
      for(k=0;k<n_channels;k++)
        color_sum += *(p + (k * bps));
      color_avg = color_sum / (double)n_channels;

      if(negate)
        occ = color_avg / 255.0;
      else
        occ = (255 - color_avg) / 255.0;
      if(occ > 0.5)
        (*mapdata)[MAP_IDX(*size_x,i,*size_y - j - 1)] = +1;
      else if(occ < 0.1)
        (*mapdata)[MAP_IDX(*size_x,i,*size_y - j - 1)] = -1;
      else
        (*mapdata)[MAP_IDX(*size_x,i,*size_y - j - 1)] = 0;
    }
  }

  gdk_pixbuf_unref(pixbuf);

  puts("Done.");
  printf("MapFile read a %d X %d map\n", *size_x, *size_y);
  return(0);
}

