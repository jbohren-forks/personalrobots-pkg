/*
 * sbpl_2dnav
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

//for timing
#include <sys/time.h>

//for map
#include <gdk-pixbuf/gdk-pixbuf.h>

// roscpp
#include <ros/node.h>

// For transform support
#include <tf/transform_listener.h>

#include <std_msgs/PointCloud.h>
#include <robot_msgs/Planner2DGoal.h>
#include <pr2_msgs/OccDiff.h>
#include <std_srvs/StaticMap.h>
#include <pr2_srvs/TransientObstacles.h>

//sbpl headers file
#include <headers.h>

static const int NEW_POSE_REPLAN_THRESHOLD=5;

class Sbpl2DNav : public ros::node {
public:
  Sbpl2DNav(void); 

  ~Sbpl2DNav(void);

  void occDiffCallback();
  
  void newGoalCallback();

  void DrawEnvMapWithPath(vector<int> path);

  void doOneCycle();

  tf::TransformListener tf_;
private:

  MDPConfig MDPCfg;
  EnvironmentNAV2D environment_nav2D;
  ARAPlanner* ara_planner_;

  bool replan_required_;
  ros::thread::mutex map_lock_;

  char* static_mapdata_;
  char* full_mapdata_;
  int size_x_;
  int size_y_;

  int last_pose_x_;
  int last_pose_y_;

  bool goal_set_;

  robot_msgs::Planner2DGoal goal_msg_;
  std_msgs::Polyline2D pointcloud_msg_;
 pr2_msgs::OccDiff occ_diff_;
};

Sbpl2DNav::Sbpl2DNav(void): ros::node("sbpl_2dnav"), 
                            tf_(*this, true) 
{
  last_pose_x_ = -1;
  last_pose_y_ = -1;
  replan_required_ = false;
  goal_set_ = false;
  
  std_srvs::StaticMap::request  static_req;
  std_srvs::StaticMap::response static_resp;
  puts("Requesting the map...");
  while(!ros::service::call("static_map", static_req, static_resp))
  {
    puts("request failed; trying again...");
    usleep(1000000);
  }
  printf("Received a %d X %d map @ %.3f m/pix\n",
         static_resp.map.width,
         static_resp.map.height,
         static_resp.map.resolution);
    
  size_x_ = static_resp.map.width;
  size_y_ = static_resp.map.height;
  // Convert to player format
  static_mapdata_ = new char[size_x_*size_y_];
  for(int i=0;i<size_x_*size_y_;i++) {
    if(static_resp.map.data[i] == 0)
      static_mapdata_[i] = 0;
    else if(static_resp.map.data[i] == 100)
      static_mapdata_[i] = 1;
    else
      static_mapdata_[i] = 1;
  }

  full_mapdata_ = new char[size_x_*size_y_];
  memcpy(full_mapdata_,static_mapdata_,size_x_*size_y_);
  int g = 2;
  //stupid 4-connected obstacle growth
  for(int i = g; i < size_x_-g; i++) {
    for(int j = g; j < size_y_-g; j++) {
      if(static_mapdata_[i+j*size_x_] == 1) {
        for(int k = i-g; k <= i+g; k++) {
          for(int l = j-g; l <= j+g; l++) {
            full_mapdata_[k+l*size_x_] = 1;
          }
        }
      }
    }
  }

  //now getting full transient data
  pr2_srvs::TransientObstacles::request transient_req;
  pr2_srvs::TransientObstacles::response transient_res;
    
  puts("Calling full transient obstacle service");
  while(!ros::service::call("transient_obstacles_full", transient_req, transient_res)) {
    puts("request failed; trying again...");
    usleep(1000000);
  }
  puts("Got full transient obstacles");

  for(size_t i = 0; i < transient_res.obs.get_points_size(); i++) 
  {
    int mx = (int) transient_res.obs.points[i].x/.1;
    int my = (int) transient_res.obs.points[i].y/.1;
    full_mapdata_[mx+my*size_x_] = 1;
  }
  
  environment_nav2D.SetConfiguration(size_x_,
                                     size_y_,
                                     full_mapdata_,
                                     0,0,
                                     539,586);
  environment_nav2D.InitGeneral();
  
  //Initialize MDP Info
  if(!environment_nav2D.InitializeMDPCfg(&MDPCfg)) 
  {
    printf("ERROR: InitializeMDPCfg failed\n");
    exit(1);
  }

  ara_planner_ = new ARAPlanner(&environment_nav2D);

  /*
    ara_planner_->set_start(MDPCfg.startstateid);
    ara_planner_->set_goal(MDPCfg.goalstateid);
  */    

  advertise<std_msgs::Polyline2D>("gui_path");
  subscribe("goal", goal_msg_, &Sbpl2DNav::newGoalCallback);
  subscribe("transient_obstacles_diff", occ_diff_, &Sbpl2DNav::occDiffCallback);
  
  /*
    double allocated_time_secs = 60*10; //in seconds
    
    vector<int> solution_stateIDs_V;
    
    int bRet = ara_planner_->replan(allocated_time_secs, &solution_stateIDs_V);
    
    map_lock_.unlock();
    
    pointcloud_msg_.set_points_size(solution_stateIDs_V.size());
    pointcloud_msg_.color.a = 0.0;
    pointcloud_msg_.color.r = 0.0;
    pointcloud_msg_.color.b = 1.0;
    pointcloud_msg_.color.g = 0.0;
    for(unsigned int i=0;i<solution_stateIDs_V.size();i++)
    {
    int px, py;
    environment_nav2D.GetCoordFromState(solution_stateIDs_V[i], px, py);
    
    pointcloud_msg_.points[i].x = px*.1;
    pointcloud_msg_.points[i].y = py*.1;
    }
    publish("gui_path", pointcloud_msg_);
  */
}

Sbpl2DNav::~Sbpl2DNav() {
  delete static_mapdata_;
  delete full_mapdata_;
  delete ara_planner_;
}

void Sbpl2DNav::newGoalCallback() {
  map_lock_.lock();

  int mx = (int)goal_msg_.goal.x/.1;
  int my = (int)goal_msg_.goal.y/.1;
  
  environment_nav2D.SetGoal(mx, my);
  ara_planner_->set_goal(environment_nav2D.GetStateFromCoord(mx, my));

  //std::cout << "Setting new goal " << mx << " " << my << std::endl;

  replan_required_ = true;
  goal_set_ = true;
  
  map_lock_.unlock();
}

void Sbpl2DNav::occDiffCallback() {
  map_lock_.lock();

  //first take out points
  for(size_t i = 0; i < occ_diff_.get_unocc_points_size(); i++) 
  {
    int mx = (int) occ_diff_.unocc_points[i].x/.1;
    int my = (int) occ_diff_.unocc_points[i].y/.1;
    if(full_mapdata_[mx+my*size_x_] != 1) {
      //std::cout << "Sbpl2DNav::occDiffCallback() unocc_point cell " << mx << " " << my << " not occupied\n";
    } else if(static_mapdata_[mx+my*size_x_] != 1) {
      //only unset if not in static map for now
      full_mapdata_[mx+my*size_x_] = 0;
      environment_nav2D.UpdateCost(mx, my, 0);
      //not replanning on this for now
    }   
  }

  //adding new points
  for(size_t i = 0; i < occ_diff_.get_occ_points_size(); i++)
  {
    int mx = (int) occ_diff_.occ_points[i].x/.1;
    int my = (int) occ_diff_.occ_points[i].y/.1;
    if(full_mapdata_[mx+my*size_x_] == 0) {
      full_mapdata_[mx+my*size_x_] = 1;
      environment_nav2D.UpdateCost(mx, my, 1);
      replan_required_ = true;
    }
  } 
  map_lock_.unlock();
}

void Sbpl2DNav::doOneCycle() {

  //first update pose
  
  // Get the current robot pose in the map frame
  tf::Stamped<tf::Pose> robot_pose(tf::Pose(tf::Quaternion(0,0,0), tf::Point(0,0,0)), ros::Time(), "base_link");
  tf::Stamped<tf::Pose> global_pose;
  //robot_pose_.time = laserMsg.header.stamp.sec * 1000000000ULL + 
  //        laserMsg.header.stamp.nsec; ///HACKE FIXME we should be able to get time somewhere else
  try
  {
    tf_.transformPose("map", robot_pose, global_pose);
  }
  catch(tf::LookupException& ex)
  {
    puts("no global->local Tx yet");
    return;
  }
  catch(tf::ExtrapolationException& ex)
  {
    // this should never happen
    puts("WARNING: extrapolation failed!");
    return;
  }

  int mx = global_pose.getOrigin().x()/0.1;
  int my = global_pose.getOrigin().y()/0.1;

  if(abs(last_pose_x_-mx) >= NEW_POSE_REPLAN_THRESHOLD ||
     abs(last_pose_y_-my) >= NEW_POSE_REPLAN_THRESHOLD) {
    replan_required_ = true;
  }

  if(replan_required_ && goal_set_) {
    environment_nav2D.SetStart(mx, my);
    std::cout << "Setting start to " << mx << " " << my << std::endl;
    ara_planner_->set_start(environment_nav2D.GetStateFromCoord(mx, my));

    last_pose_x_ = mx;
    last_pose_y_ = my;

    map_lock_.lock();

    double allocated_time_secs = 2; //in seconds

    vector<int> solution_stateIDs_V;

    int bRet = ara_planner_->replan(allocated_time_secs, &solution_stateIDs_V);

    std::cout << "Number of states " << solution_stateIDs_V.size() << std::endl;

    map_lock_.unlock();

    pointcloud_msg_.set_points_size(solution_stateIDs_V.size());
    pointcloud_msg_.color.a = 0.0;
    pointcloud_msg_.color.r = 1.0;
    pointcloud_msg_.color.b = 0.0;
    pointcloud_msg_.color.g = 0.0;
    for(unsigned int i=0;i<solution_stateIDs_V.size();i++)
    {
      int px, py;
      environment_nav2D.GetCoordFromState(solution_stateIDs_V[i], px, py);
      
      pointcloud_msg_.points[i].x = px*.1;
      pointcloud_msg_.points[i].y = py*.1;
    }
    publish("gui_path", pointcloud_msg_);
    replan_required_=false;
  }
}


void Sbpl2DNav::DrawEnvMapWithPath(vector<int> path) {
  GdkPixbuf* pixbuf;
  GError* error = NULL;
  guchar* pixels;
  int p;
  int paddr;
  int i, j;
  
  // Initialize glib
  g_type_init();

  const EnvNAV2DConfig_t* env = environment_nav2D.GetEnvNavConfig();

  pixels = (guchar*)malloc(sizeof(guchar)*env->EnvWidth_c*env->EnvHeight_c*3);


  //this draws the obstacles
  p=0;
  for(j=env->EnvHeight_c-1;j>=0;j--) {
    for(i=0; i<env->EnvWidth_c; i++,p++) {
      paddr = p * 3;
	
      /*
        if(i > 190 && i < 210 && j > 290 && j < 310) {
        pixels[paddr] = 0;
        pixels[paddr+1] = 0;
        pixels[paddr+2] = 255;
	} else 
      */
      if(env->Grid2D[i][j] == 1) {
        pixels[paddr] = 0;
        pixels[paddr+1] = 0;
        pixels[paddr+2] = 0;
      } else {
        pixels[paddr] = 255;
        pixels[paddr+1] = 255;
        pixels[paddr+2] = 255;
      }
    }
  }
    
  //now we draw the path
  for(unsigned int i = 0; i < path.size(); i++) {
    int px, py;
    environment_nav2D.GetCoordFromState(path[i], px, py);
    //std::cout << px << " " << py << std::endl;
    paddr = 3*(px+env->EnvWidth_c*(env->EnvHeight_c-py-1));
    pixels[paddr] = 0;
    pixels[paddr+1] = 255;
    pixels[paddr+2] = 0;
  }

  pixbuf = gdk_pixbuf_new_from_data(pixels, 
                                    GDK_COLORSPACE_RGB,
                                    0,8,
                                    env->EnvWidth_c,
                                    env->EnvHeight_c,
                                    env->EnvWidth_c * 3,
                                    NULL, NULL);
    
  gdk_pixbuf_save(pixbuf,"path.png","png",&error,NULL);
  gdk_pixbuf_unref(pixbuf);
  free(pixels);
}

int main(int argc, char **argv) {

  ros::init(argc, argv);
  
  Sbpl2DNav snav;
  
  sleep(1);

  while(1) {
    snav.doOneCycle();
    usleep(100000);
  }
  snav.shutdown();
}
