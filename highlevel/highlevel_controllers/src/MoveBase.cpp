/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 *
 * @b move_base is...
 *
 * <hr>
 *
 *  @section usage Usage
 *  @verbatim
 *  $ move_base
 *  @endverbatim
 *
 * <hr>
 *
 * @section topic ROS topics
 *
 * Subscribes to (name/type):
 * - @b 
 *
 * Publishes to (name / type):
 * - @b 
 *
 *  <hr>
 *
 * @section parameters ROS parameters
 *
 * - None
 **/

#include <MoveBase.hh>

MoveBase::MoveBase(double windowLength, unsigned char lethalObstacleThreshold, unsigned char noInformation, double maxZ)
  : HighlevelController<std_msgs::Planner2DState, std_msgs::Planner2DGoal>("mobe_base", "state", "goal"),
    tf_(*this, true, 10000000000ULL), // cache for 10 sec, no extrapolation
    costMap_(NULL),
    laserMaxRange_(4.0){

  // Initialize global pose. Will be set in control loop based on actual data.
  global_pose_.x = 0;
  global_pose_.y = 0;
  global_pose_.yaw = 0;

  // Set up transforms
  double laser_x_offset(0.05);
  //param("laser_x_offset", laser_x_offset, 0.05);
  tf_.setWithEulers("FRAMEID_LASER", "FRAMEID_ROBOT", laser_x_offset, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

  // get map via RPC
  std_srvs::StaticMap::request  req;
  std_srvs::StaticMap::response resp;
  std::cout << "Requesting the map..." << std::endl;
  while(!ros::service::call("static_map", req, resp))
  {
    std::cout << "Request failed; trying again..." << std::endl;
    usleep(1000000);
  }

  std::cout << "Received a " << resp.map.width << " X " << 
    resp.map.height << " map at " << 
    resp.map.resolution << "m/pix" << std::endl;

  // Convert to cost map
  unsigned char* mapdata;
  unsigned int numCells = resp.map.width*resp.map.height;
  mapdata = new unsigned char[numCells];
  for(unsigned int i=0;i<numCells;i++)
  {
    if(resp.map.data[i] == noInformation){
      // Handle a translation for no information
      mapdata[i] = CostMap2D::NO_INFORMATION;
    }
    else{
      // Simply copy the actual cost value
      mapdata[i] = resp.map.data[i];
    }
  }

  // Now allocate the cost map
  costMap_ = new CostMap2D(resp.map.width, resp.map.height, mapdata, resp.map.resolution, 
			   windowLength, lethalObstacleThreshold, maxZ);

  // Finish by deleting the mapdata
  delete mapdata;

  // Subscribe to laser scan messages
  subscribe("scan", laserScanMsg_, &MoveBase::laserScanCallback);

  // Advertize messages to publish path updates
  advertise<std_msgs::Polyline2D>("gui_laser");

  // Now initialize
  initialize();
}

MoveBase::~MoveBase(){
  if(costMap_ != NULL)
    delete costMap_;
}

void MoveBase::updateGlobalPose(){
  libTF::TFPose2D robotPose;
  robotPose.x = 0;
  robotPose.y = 0;
  robotPose.yaw = 0;
  robotPose.frame = "FRAMEID_ROBOT";
  robotPose.time = 0; 

  try{
    global_pose_ = this->tf_.transformPose2D("FRAMEID_MAP", robotPose);
  }
  catch(libTF::TransformReference::LookupException& ex){
    std::cout << "No Transform available Error\n";
  }
  catch(libTF::TransformReference::ConnectivityException& ex){
    std::cout << "Connectivity Error\n";
  }
  catch(libTF::TransformReference::ExtrapolateException& ex){
    std::cout << "Extrapolation Error\n";
  }

  //std::cout << "Robot at (" << global_pose_.x << ", " << global_pose_.y << ", " << global_pose_.yaw << ")\n";
}

void MoveBase::updateStateMsg(){
  // Get the current robot pose in the map frame
  updateGlobalPose();
}

/**
 * The laserScanMsg_ member will have been updated. It is locked already too.
 */
void MoveBase::laserScanCallback(){

  // Assemble a point cloud, in the laser's frame
  std_msgs::PointCloudFloat32 local_cloud;
  projector_.projectLaser(laserScanMsg_, local_cloud, laserMaxRange_);
    
  // Convert to a point cloud in the map frame
  std_msgs::PointCloudFloat32 global_cloud;

  try
    {
      global_cloud = this->tf_.transformPointCloud("FRAMEID_MAP", local_cloud);
    }
  catch(libTF::TransformReference::LookupException& ex)
    {
      puts("no global->local Tx yet");
      printf("%s\n", ex.what());
      return;
    }
  catch(libTF::TransformReference::ConnectivityException& ex)
    {
      puts("no global->local Tx yet");
      printf("%s\n", ex.what());
      return;
    }
  catch(libTF::TransformReference::ExtrapolateException& ex)
    {
      puts("Extrapolation exception");
    }

  /*std::cout << "Laser scan received (Laser Scan:" << laserScanMsg_.get_ranges_size() << 
    ", localCloud:" << local_cloud.get_pts_size() << 
    ", globalCloud:" << global_cloud.get_pts_size() << ")\n"; */

  // Update the cost map
  const double ts = laserScanMsg_.header.stamp.to_double();
  costMap_->updateDynamicObstacles(ts, global_cloud);

  // Pubish projected laser scan for rendering in map co-ordinates.
  std_msgs::Polyline2D pointCloudMsg;
  unsigned int pointCount = global_cloud.get_pts_size();
  pointCloudMsg.set_points_size(pointCount);
  pointCloudMsg.color.a = 0.0;
  pointCloudMsg.color.r = 0.0;
  pointCloudMsg.color.b = 1.0;
  pointCloudMsg.color.g = 0.0;

  for(unsigned int i=0;i<pointCount;i++){
    pointCloudMsg.points[i].x = global_cloud.pts[i].x;
    pointCloudMsg.points[i].y = global_cloud.pts[i].y;
  }

  publish("gui_laser",pointCloudMsg);
}
