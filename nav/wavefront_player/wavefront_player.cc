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
#include <std_msgs/MsgRobotBase2DOdom.h>
#include <std_msgs/MsgLaserScan.h>
#include <std_msgs/MsgPose2DFloat32.h>

// For GUI debug
#include <std_msgs/MsgPolyline2D.h>

// For transform support
#include <rosTF/rosTF.h>

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
static double angle_diff(double a, double b);
int read_map_from_image(int* size_x, int* size_y, char** mapdata, 
       			const char* fname, int negate);

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
    // Current pose
    double pose[3];
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
    double laser_buffer_time;
    std::list<MsgLaserScan*> buffered_laser_scans;
    std::list<std::pair<MsgPose2DFloat32*, MsgLaserScan*> > laser_scans;
    double* laser_hitpts;
    size_t laser_hitpts_len, laser_hitpts_size;

    // Controller paramters
    double lookahead_maxdist;
    double lookahead_distweight;
    double tvmin, tvmax, avmin, avmax, amin, amax;

    // incoming messages
    MsgPlanner2DGoal goalMsg;
    MsgRobotBase2DOdom odomMsg;
    MsgLaserScan laserMsg;
    MsgPolyline2D polylineMsg;
    MsgPolyline2D pointcloudMsg;
    std::vector<MsgLaserScan> laserScans;
    MsgRobotBase2DOdom prevOdom;
    bool firstodom;

    // Lock for access to class members in callbacks
    ros::thread::mutex lock;

    // Message callbacks
    void goalReceived();
    void odomReceived();
    void laserReceived();

    // Internal helpers
    void sendVelCmd(double vx, double vy, double vth);

  public:
    // Transform client
    rosTFClient* tf;

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
        robot_radius(0.2),
        safety_dist(0.1),
        max_radius(1.0),
        dist_penalty(1.0),
        plan_halfwidth(5.0),
        dist_eps(1.0),
        ang_eps(DTOR(10.0)),
        cycletime(0.1),
        laser_maxrange(4.0),
        laser_buffer_time(0.5),
        lookahead_maxdist(2.0),
        lookahead_distweight(10.0),
        tvmin(0.1),
        tvmax(0.35),
        avmin(DTOR(10.0)),
        avmax(DTOR(60.0)),
        amin(DTOR(5.0)),
        amax(DTOR(20.0)),
        tf(NULL)
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
    {
      this->plan->cells[i+j*sx].occ_state = mapdata[i+j*sx];
        /*
      if(mapdata[i+j*sx] < 0.1*255)
        this->plan->cells[i+j*sx].occ_state = -1;
      else if(mapdata[i+j*sx] > 0.5*255)
        this->plan->cells[i+j*sx].occ_state = 1;
      else
        this->plan->cells[i+j*sx].occ_state = 0;
        */
    }
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

  //this->tf = new rosTFClient(*this);

  this->laser_hitpts_size = this->laser_hitpts_len = 0;
  this->laser_hitpts = NULL;

  this->firstodom = true;

  advertise<MsgPlanner2DState>("state");
  advertise<MsgPolyline2D>("gui_path");
  advertise<MsgPolyline2D>("gui_laser");
  advertise<MsgBaseVel>("cmd_vel");
  subscribe("goal", goalMsg, &WavefrontNode::goalReceived);
  //subscribe("odom", odomMsg, &WavefrontNode::odomReceived);
  subscribe("localizedpose", odomMsg, &WavefrontNode::odomReceived);
  subscribe("scan", laserMsg, &WavefrontNode::laserReceived);
}

WavefrontNode::~WavefrontNode()
{
  plan_free(this->plan);
  if(this->tf)
    delete this->tf;
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
WavefrontNode::odomReceived()
{
  this->lock.lock();
  /*
  libTF::TFPose2D odom_pose;
  odom_pose.x = odomMsg.pos.x;
  odom_pose.y = odomMsg.pos.y;
  odom_pose.yaw = odomMsg.pos.th;
  odom_pose.time = odomMsg.header.stamp_secs * 1000000000ULL + 
          odomMsg.header.stamp_nsecs;
  odom_pose.frame = odomMsg.header.frame_id;

  try
  {
    libTF::TFPose2D global_pose = 
            this->tf->transformPose2D(ROSTF_FRAME_MAP, odom_pose);
    this->pose[0] = global_pose.x;
    this->pose[1] = global_pose.y;
    this->pose[2] = global_pose.yaw;
    printf("lpose: %.3f %.3f %.3f\n",
           odomMsg.pos.x,
           odomMsg.pos.y,
           RTOD(odomMsg.pos.th));
    printf("gpose: %.3f %.3f %.3f @ (%u:%u)\n",
           this->pose[0],
           this->pose[1],
           RTOD(this->pose[2]),
           odomMsg.header.stamp_secs,
           odomMsg.header.stamp_nsecs);
  }
  catch(libTF::TransformReference::LookupException& ex)
  {
    puts("no global->local Tx yet");
  }
  */

  this->pose[0] = odomMsg.pos.x;
  this->pose[1] = odomMsg.pos.y;
  this->pose[2] = odomMsg.pos.th;
  printf("gpose: %.3f %.3f %.3f\n",
         this->pose[0],
         this->pose[1],
         RTOD(this->pose[2]));

  if(this->firstodom)
  {
    this->prevOdom = odomMsg;
    this->firstodom = false;
  }
  else
  {
    // Interpolate poses for all buffered scans
    // TODO: use libTF's interpolation
    for(std::list<MsgLaserScan*>::iterator it = 
        this->buffered_laser_scans.begin();
        it != this->buffered_laser_scans.end();
        it++)
    {
      double t0 = prevOdom.header.stamp_secs + 
              prevOdom.header.stamp_nsecs / 1e9;
      double t1 = odomMsg.header.stamp_secs + 
              odomMsg.header.stamp_nsecs / 1e9;
      double tl = (*it)->header.stamp_secs + 
              (*it)->header.stamp_nsecs / 1e9;

      if(tl < t0)
      {
        // Too late
        it = this->buffered_laser_scans.erase(it);
        it--;
        continue;
      }

      if(tl > t1)
      {
        // Too early
        continue;
      }

      double dt = t1 - t0;
      double dtl = tl - t0;

      double dx = odomMsg.pos.x - prevOdom.pos.x;
      double dy = odomMsg.pos.y - prevOdom.pos.y;
      double da = angle_diff(odomMsg.pos.th, prevOdom.pos.th);

      MsgPose2DFloat32* p = new MsgPose2DFloat32();
      p->x = prevOdom.pos.x + (dx / dt) * dtl;
      p->y = prevOdom.pos.y + (dy / dt) * dtl;
      p->th = ANG_NORM(prevOdom.pos.th + (da / dt) * dtl);

      printf("0: %.3f %.3f %.3f\t%.6f\n",
             odomMsg.pos.x,
             odomMsg.pos.y,
             odomMsg.pos.th,
             t0);
      printf("I: %.3f %.3f %.3f\t%.6f\n",
             p->x,p->y,p->th, tl);
      printf("1: %.3f %.3f %.3f\t%.6f\n",
             prevOdom.pos.x,
             prevOdom.pos.y,
             prevOdom.pos.th,
             t1);

      MsgLaserScan* scan = new MsgLaserScan(this->laserMsg);

      std::pair<MsgPose2DFloat32*,MsgLaserScan*> item(p,scan);

      this->laser_scans.push_back(item);
      it = this->buffered_laser_scans.erase(it);
      it--;
    }

    // Remove anything that's too old
    // Also count how many points we have
    double currtime = this->laserMsg.header.stamp_secs + 
            this->laserMsg.header.stamp_nsecs / 1e9;
    unsigned int hitpt_cnt=0;
    for(std::list<std::pair<MsgPose2DFloat32*,MsgLaserScan*> >::iterator it = this->laser_scans.begin();
        it != this->laser_scans.end();
        it++)
    {
      double msgtime = it->second->header.stamp_secs + it->second->header.stamp_nsecs / 1e9;
      if((currtime - msgtime) > this->laser_buffer_time)
      {
        delete it->first;
        delete it->second;
        it = this->laser_scans.erase(it);
        it--;
      }
      else
        hitpt_cnt += it->second->get_ranges_size()*2;
    }

    // allocate more space as necessary
    if(this->laser_hitpts_size < hitpt_cnt)
    {
      this->laser_hitpts_size = hitpt_cnt;
      this->laser_hitpts = 
              (double*)realloc(this->laser_hitpts,
                               this->laser_hitpts_size*sizeof(double));
      assert(this->laser_hitpts);
    }

    // project hit points from each scan
    double* pts = this->laser_hitpts;
    this->laser_hitpts_len = 0;
    for(std::list<std::pair<MsgPose2DFloat32*, MsgLaserScan*> >::iterator it = this->laser_scans.begin();
        it != this->laser_scans.end();
        it++)
    {
      float b=it->second->angle_min;
      float* r=it->second->ranges;
      for(unsigned int j=0;
          j<it->second->get_ranges_size();
          j++,r++,b+=it->second->angle_increment)
      {
        if(((*r) >= this->laser_maxrange) || ((*r) >= it->second->range_max))
          continue;

        double cs,sn;
        cs = cos(it->first->th+b);
        sn = sin(it->first->th+b);

        double lx,ly;
        lx = it->first->x + (*r)*cs;
        ly = it->first->y + (*r)*sn;

        assert(this->laser_hitpts_len*2 < this->laser_hitpts_size);
        *(pts++) = lx;
        *(pts++) = ly;
        this->laser_hitpts_len++;
      }
    }

    //printf("setting %d hit points\n", this->scan_points_count);
    plan_set_obstacles(this->plan, this->laser_hitpts, this->laser_hitpts_len);

    // Draw the points

    this->pointcloudMsg.set_points_size(hitpt_cnt/2);
    this->pointcloudMsg.color.a = 0.0;
    this->pointcloudMsg.color.r = 0.0;
    this->pointcloudMsg.color.b = 1.0;
    this->pointcloudMsg.color.g = 0.0;
    for(unsigned int i=0;i<hitpt_cnt/2;i++)
    {
      this->pointcloudMsg.points[i].x = this->laser_hitpts[2*i];
      this->pointcloudMsg.points[i].y = this->laser_hitpts[2*i+1];
    }
    publish("gui_laser",this->pointcloudMsg);

    prevOdom = odomMsg;
  }


  this->lock.unlock();
}

void
WavefrontNode::laserReceived()
{
  // Buffer the scan.  It will get processed in odomReceived()
  this->lock.lock();
  this->buffered_laser_scans.push_back(new MsgLaserScan(laserMsg));
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

  this->lock.lock();
  switch(this->planner_state)
  {
    case NO_GOAL:
      //puts("no goal");
      this->stopRobot();
      break;
    case REACHED_GOAL:
      puts("still done");
      this->stopRobot();
      break;
    case NEW_GOAL:
    case PURSUING_GOAL:
      {
        // Are we done?
        if(plan_check_done(this->plan,
                           this->pose[0], this->pose[1], this->pose[2],
                           this->goal[0], this->goal[1], this->goal[2],
                           this->dist_eps, this->ang_eps))
        {
          puts("done");
          this->stopRobot();
          this->planner_state = REACHED_GOAL;
          break;
        }

        // Try a local plan
        if((this->planner_state == NEW_GOAL) ||
           (plan_do_local(this->plan, this->pose[0], this->pose[1], 
                         this->plan_halfwidth) < 0))
        {
          // Fallback on global plan
          if(plan_do_global(this->plan, this->pose[0], this->pose[1], 
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
            if(plan_do_local(this->plan, this->pose[0], this->pose[1], 
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
                                       this->pose[0], this->pose[1],
                                       this->pose[2],
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

        printf("computed velocities: %.3f %.3f\n", vx, RTOD(va));
        this->sendVelCmd(vx, 0.0, va);

        break;
      }
    default:
      assert(0);
  }
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

// computes the signed minimum difference between the two angles.
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = ANG_NORM(a);
  b = ANG_NORM(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}
