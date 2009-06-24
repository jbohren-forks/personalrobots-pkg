/*********************************************************************
*
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <ros/node.h>
#include <ros/console.h>
#include <new_costmap/costmap_2d.h>
#include <new_costmap/observation_buffer.h>
#include <robot_srvs/StaticMap.h>
#include <visualization_msgs/Polyline.h>
#include <map>
#include <vector>

#include <tf/transform_datatypes.h>
#include <tf/message_notifier.h>
#include <tf/transform_listener.h>

#include <laser_scan/LaserScan.h>
#include <laser_scan/laser_scan.h>

#include <robot_msgs/PointCloud.h>

// Thread suppport
#include <boost/thread.hpp>

using namespace costmap_2d;
using namespace tf;
using namespace robot_msgs;

class CostmapTester {
  public:
    CostmapTester(ros::Node& ros_node) : ros_node_(ros_node), base_scan_notifier_(NULL), tf_(ros_node, true, ros::Duration(10)), global_frame_("map"), freq_(5), base_scan_buffer_(NULL) {
      ros_node.advertise<visualization_msgs::Polyline>("raw_obstacles", 1);
      ros_node.advertise<visualization_msgs::Polyline>("inflated_obstacles", 1);
      
      base_scan_buffer_ = new ObservationBuffer(0.0, 0.2, tf_, "map", "base_laser");

      base_scan_notifier_ = new MessageNotifier<laser_scan::LaserScan>(&tf_, &ros_node,
          boost::bind(&CostmapTester::baseScanCallback, this, _1, (int) 1),
          "base_scan", global_frame_, 50);

      robot_srvs::StaticMap::Request map_req;
      robot_srvs::StaticMap::Response map_resp;
      ROS_INFO("Requesting the map...\n");
      while(!ros::service::call("static_map", map_req, map_resp))
      {
        ROS_INFO("Request failed; trying again...\n");
        usleep(1000000);
      }
      ROS_INFO("Received a %d X %d map at %f m/pix\n",
          map_resp.map.info.width, map_resp.map.info.height, map_resp.map.info.resolution);

      // We are treating cells with no information as lethal obstacles based on the input data. This is not ideal but
      // our planner and controller do not reason about the no obstacle case
      std::vector<unsigned char> input_data;
      unsigned int numCells = map_resp.map.info.width * map_resp.map.info.height;
      for(unsigned int i = 0; i < numCells; i++){
        input_data.push_back((unsigned char) map_resp.map.data[i]);
      }

      struct timeval start, end;
      double start_t, end_t, t_diff;
      gettimeofday(&start, NULL);
      new_costmap_ = new Costmap2D((unsigned int)map_resp.map.info.width, (unsigned int)map_resp.map.info.height,
          map_resp.map.info.resolution, 0.0, 0.0, 0.325, 0.46, 0.55, 2.5, 2.0, 3.0, 1.0, input_data, 100);
      gettimeofday(&end, NULL);
      start_t = start.tv_sec + double(start.tv_usec) / 1e6;
      end_t = end.tv_sec + double(end.tv_usec) / 1e6;
      t_diff = end_t - start_t;
      ROS_INFO("New map construction time: %.9f", t_diff);


      //create a separate thread to publish cost data to the visualizer
      visualizer_thread_ = new boost::thread(boost::bind(&CostmapTester::publishCostmap, this));
      window_reset_thread_ = new boost::thread(boost::bind(&CostmapTester::resetWindow, this));

    }

    ~CostmapTester(){
      delete new_costmap_;
      delete base_scan_notifier_;
      delete visualizer_thread_;
      delete window_reset_thread_;
      delete base_scan_buffer_;
    }

    void baseScanCallback(const tf::MessageNotifier<laser_scan::LaserScan>::MessagePtr& message, int i){
      //project the laser into a point cloud
      PointCloud base_cloud;
      base_cloud.header = message->header;
      //we want all values... even those out of range
      projector_.projectLaser(*message, base_cloud, -1.0, true);

      //buffer the point cloud
      lock_.lock();
      base_scan_buffer_->bufferCloud(base_cloud);
      lock_.unlock();
    }

    void spin(){
      while(ros_node_.ok()){
        updateMap();
        usleep(1e6/freq_);
      }
    }

    void updateMap(){
      tf::Stamped<tf::Pose> robot_pose, global_pose;
      global_pose.setIdentity();
      robot_pose.setIdentity();
      robot_pose.frame_id_ = "base_link";
      robot_pose.stamp_ = ros::Time();
      try{
        tf_.transformPose(global_frame_, robot_pose, global_pose);
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

      double wx = global_pose.getOrigin().x();
      double wy = global_pose.getOrigin().y();

      //in the real world... make a deep copy... but for testing I'm lazy so I'll lock around the updates
      lock_.lock();
      struct timeval start, end;
      double start_t, end_t, t_diff;
      gettimeofday(&start, NULL);
      std::vector<Observation> observations;
      base_scan_buffer_->getObservations(observations);
      new_costmap_->updateWorld(wx, wy, observations, observations);
      gettimeofday(&end, NULL);
      start_t = start.tv_sec + double(start.tv_usec) / 1e6;
      end_t = end.tv_sec + double(end.tv_usec) / 1e6;
      t_diff = end_t - start_t;
      ROS_INFO("Map update time: %.9f", t_diff);
      lock_.unlock();
    }

    void resetWindow(){
      while(ros_node_.ok()){
        tf::Stamped<tf::Pose> robot_pose, global_pose;
        global_pose.setIdentity();
        robot_pose.setIdentity();
        robot_pose.frame_id_ = "base_link";
        robot_pose.stamp_ = ros::Time();
        try{
          tf_.transformPose(global_frame_, robot_pose, global_pose);
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

        double wx = global_pose.getOrigin().x();
        double wy = global_pose.getOrigin().y();
        lock_.lock();
        ROS_INFO("Resetting map outside window");
        new_costmap_->resetMapOutsideWindow(wx, wy, 5.0, 5.0);
        lock_.unlock();

        usleep(1e6/0.2);
      }
    }

    void publishCostmap(){
      while(ros_node_.ok()){
        ROS_INFO("publishing map");
        lock_.lock();
        std::vector< std::pair<double, double> > raw_obstacles, inflated_obstacles;
        for(unsigned int i = 0; i<new_costmap_->cellSizeX(); i++){
          for(unsigned int j = 0; j<new_costmap_->cellSizeY();j++){
            double wx, wy;
            new_costmap_->mapToWorld(i, j, wx, wy);
            std::pair<double, double> p(wx, wy);

            if(new_costmap_->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
              raw_obstacles.push_back(p);
            else if(new_costmap_->getCost(i, j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
              inflated_obstacles.push_back(p);
          }
        }
        lock_.unlock();

        // First publish raw obstacles in red
        visualization_msgs::Polyline obstacle_msg;
        obstacle_msg.header.frame_id = global_frame_;
        unsigned int pointCount = raw_obstacles.size();
        obstacle_msg.set_points_size(pointCount);
        obstacle_msg.color.a = 0.0;
        obstacle_msg.color.r = 1.0;
        obstacle_msg.color.b = 0.0;
        obstacle_msg.color.g = 0.0;

        for(unsigned int i=0;i<pointCount;i++){
          obstacle_msg.points[i].x = raw_obstacles[i].first;
          obstacle_msg.points[i].y = raw_obstacles[i].second;
          obstacle_msg.points[i].z = 0;
        }

        ros::Node::instance()->publish("raw_obstacles", obstacle_msg);

        // Now do inflated obstacles in blue
        pointCount = inflated_obstacles.size();
        obstacle_msg.set_points_size(pointCount);
        obstacle_msg.color.a = 0.0;
        obstacle_msg.color.r = 0.0;
        obstacle_msg.color.b = 1.0;
        obstacle_msg.color.g = 0.0;

        for(unsigned int i=0;i<pointCount;i++){
          obstacle_msg.points[i].x = inflated_obstacles[i].first;
          obstacle_msg.points[i].y = inflated_obstacles[i].second;
          obstacle_msg.points[i].z = 0;
        }

        ros::Node::instance()->publish("inflated_obstacles", obstacle_msg);
        usleep(1e6/2.0);
      }
    }

    ros::Node& ros_node_;
    tf::MessageNotifier<laser_scan::LaserScan>* base_scan_notifier_; ///< @brief Used to guarantee that a transform is available for base scans
    tf::TransformListener tf_; ///< @brief Used for transforming point clouds
    laser_scan::LaserProjection projector_; ///< @brief Used to project laser scans into point clouds
    boost::recursive_mutex lock_; ///< @brief A lock for accessing data in callbacks safely
    Costmap2D* new_costmap_;
    std::string global_frame_;
    double freq_;
    boost::thread* visualizer_thread_;
    boost::thread* window_reset_thread_;
    ObservationBuffer* base_scan_buffer_;

};

int main(int argc, char** argv){
  ros::init(argc, argv);
  ros::Node ros_node("new_costmap_tester");
  CostmapTester tester(ros_node);
  tester.spin();

  return(0);

}

