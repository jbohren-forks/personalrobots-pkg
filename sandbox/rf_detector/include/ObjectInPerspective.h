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
#include "topic_synchronizer2/topic_synchronizer.h"
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
//#include <point_cloud_mapping/geometry/statistics.h>
#include <point_cloud_mapping/geometry/projections.h>

#include "sensor_msgs/CamInfo.h"
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
			double dist_thresh, int min_points_per_model);


void filterTablePlane(const sensor_msgs::PointCloud& cloud, vector<double>& coefficients, sensor_msgs::PointCloud& object_cloud, sensor_msgs::PointCloud& plane_cloud);

void addTableFrame( geometry_msgs::PointStamped origin, const vector<double>& plane, tf::TransformListener& tf_, tf::TransformBroadcaster& broadcaster_);

float GetCameraHeight(const sensor_msgs::PointCloud& pc, tf::TransformListener& tf_);

float CalHorizontalLine( const sensor_msgs::PointCloud& cloud, vector<double> plane_coeff, const sensor_msgs::CamInfo& lcinfo, tf::TransformListener& tf_);

void filterClusters(const sensor_msgs::PointCloud& cloud, vector<geometry_msgs::Point32>& centers, vector<geometry_msgs::Point32>& clusters);

void findClusters2(const sensor_msgs::PointCloud& cloud, vector<geometry_msgs::Point32>& clusters);

//double meanShiftIteration(const PointCloud& pc, NNGridIndexer& index, vector<Point32>& centers, vector<Point32>& means, float step);

//void clearFromImage(IplImage* disp_img, const sensor_msgs::PointCloud& pc);

float findObjectPositionsFromStereo(const sensor_msgs::PointCloud& cloud, vector<CvPoint>& locations, vector<CvPoint>& obj_bottom,
        vector<float>& scales, vector<float>& scales_msun, tf::TransformListener& tf_, tf::TransformBroadcaster& broadcaster_,
        const sensor_msgs::CamInfo& lcinfo);

geometry_msgs::Point project3DPointIntoImage(const sensor_msgs::CamInfo& cam_info, geometry_msgs::PointStamped point, tf::TransformListener& tf_);
