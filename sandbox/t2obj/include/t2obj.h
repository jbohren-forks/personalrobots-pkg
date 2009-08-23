/*********************************************************************
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

// modified by Min Sun from Recognition_lambertian
#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv/cvaux.h"
#include "opencv_latest/CvBridge.h"
#include "sensor_msgs/Image.h"
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_oriented_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/geometry/projections.h>

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"

// transform library
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace std;
#define CV_PIXEL(type,img,x,y) (((type*)(img->imageData+y*img->widthStep))+x*img->nChannels)
//function lists
void filterByZBounds( const sensor_msgs::PointCloud& pc, double zmin, double zmax, sensor_msgs::PointCloud& filtered_pc, sensor_msgs::PointCloud& filtered_outside);

bool fitSACPlane(const sensor_msgs::PointCloud& points, const vector<int> &indices,  // input
			vector<int> &inliers, vector<double> &coeff,  // output
			double dist_thresh, int min_points_per_model, const geometry_msgs::Point32& orientation, double eps_angle);


void filterTablePlane(const sensor_msgs::PointCloud& cloud, vector<double>& coefficients, sensor_msgs::PointCloud& object_cloud, sensor_msgs::PointCloud& plane_cloud,
    sensor_msgs::PointCloud& filtered_outside, const geometry_msgs::Point32& orientation, double eps_angle, double min_depth, double max_depth);

void addTableFrame( geometry_msgs::PointStamped origin, const vector<double>& plane, tf::TransformListener& tf_, tf::TransformBroadcaster& broadcaster_);

float GetCameraHeight(const sensor_msgs::PointCloud& pc, tf::TransformListener& tf_);

float CalHorizontalLine( const sensor_msgs::PointCloud& cloud, vector<double> plane_coeff, const sensor_msgs::CameraInfo& lcinfo, tf::TransformListener& tf_);

void filterClusters( sensor_msgs::PointCloud& cloud, vector<geometry_msgs::Point32>& centers, vector<geometry_msgs::Point32>& clusters, double min_height, double cluster_radius);
void findTabletopClusters(const sensor_msgs::PointCloud& cloud, vector<geometry_msgs::Point32>& centers, vector<sensor_msgs::PointCloud>& clusters, double min_height, double cluster_radius);
//void findClusters2( sensor_msgs::PointCloud& cloud, vector<geometry_msgs::Point32>& clusters);

void findObjectPositionsFromStereo( const sensor_msgs::PointCloud& cloud, sensor_msgs::PointCloud& locations,
        vector<float>& scales, vector<geometry_msgs::Point> top, tf::TransformListener& tf_, tf::TransformBroadcaster& broadcaster_,
        const sensor_msgs::CameraInfo& lcinfo ,
        cv::Mat& right_objbbx, const geometry_msgs::Point32& orientation, double eps_angle, int indx, double max_height, double min_height,double min_depth, double max_depth, double cluster_radius);

geometry_msgs::Point project3DPointIntoImageNoTF(const sensor_msgs::CameraInfo& cam_info, geometry_msgs::PointStamped point);
geometry_msgs::Point project3DPointIntoImage(const sensor_msgs::CameraInfo& cam_info, geometry_msgs::PointStamped point, tf::TransformListener& tf_);

void GetPointCloud2pointNoTF( sensor_msgs::PointCloud& cloud, cv::Vector<geometry_msgs::Point>& point2d, const sensor_msgs::CameraInfo& lcinfo);
void GetPointCloud2point( sensor_msgs::PointCloud& cloud, cv::Vector<geometry_msgs::Point>& point2d, const sensor_msgs::CameraInfo& lcinfo, tf::TransformListener& tf_);

void Overlay2dpoint(cv::Mat& img, cv::Vector<geometry_msgs::Point>& pp, cv::Scalar Color);
