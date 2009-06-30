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

//! \author Vijay Pradeep

#include "ros/node.h"
#include "dense_laser_assembler/Float32MultiArrayStamped.h"
#include "robot_msgs/PointCloud.h"
#include "robot_msgs/Point32.h"
#include "sensor_msgs/LaserScan.h"

#include "topic_synchronizer/topic_synchronizer.h"

#include "opencv/cv.h"

//using namespace laser_cb_processing ;
using namespace std;

#define CHECK_LAYOUT(ma) \
   if (ma.layout.dim[1].size * ma.layout.dim[0].size != ma.data.size()) \
     ROS_ERROR(#ma ": Inconsistent sizes") ; \
   else \
     ROS_INFO(#ma ": Correct size") ;


/**
 * Takes in dense_laser_scan data, and then extract corners' pixel locations. Publishes
 * the interpolated tilt joint position, laser ray angle, and range.
 */
class LaserCBProcessing
{
public:
  LaserCBProcessing(ros::Node* node) :
    node_(node), sync_(node_, this, &LaserCBProcessing::msgCallback, ros::Duration().fromSec(2.5),
                       &LaserCBProcessing::msgTimeout)
  {
    // Corners in pixel coordinates, according to openCV checkerboard detector
    sync_.subscribe("dense_tilt_scan/pixel_corners", pixel_corners_, 1);

    // Multiarray with range data
    sync_.subscribe("dense_tilt_scan/range", dense_range_, 1);

    // Multiarray with joint info
    sync_.subscribe("dense_tilt_scan/joint_info", joint_info_, 1);

    // Laser params
    sync_.subscribe("/dense_tilt_scan/scan_info", scan_info_, 1);

    // Publish corners as a (tilt angle, pointing angle, depth) tuple
    node_->advertise<robot_msgs::PointCloud> ("dense_tilt_scan/measured_corners", 1);

    sync_.ready();
  }

  void msgCallback(ros::Time t)
  {
    ROS_INFO("%f - Callback", t.toSec());

    unsigned int C = pixel_corners_.pts.size() ;
    robot_msgs::PointCloud measured_corners;
    measured_corners.header.stamp = pixel_corners_.header.stamp ;
    measured_corners.header.frame_id = "NOT_APPLICABLE" ;
    measured_corners.pts.resize(C);

    if (C > 0)
    {
      CHECK_LAYOUT(dense_range_.data);
      CHECK_LAYOUT(joint_info_.data);

      // Shape range data into an openCV matrix
      CvMat cv_ranges = cvMat(dense_range_.data.layout.dim[0].size, dense_range_.data.layout.dim[1].size, CV_32FC1,
                              &dense_range_.data.data[0]);

      // Shape joint data into an openCV matrix
      CvMat cv_joints = cvMat(joint_info_.data.layout.dim[0].size, joint_info_.data.layout.dim[1].size, CV_32FC1,
                              &joint_info_.data.data[0]);

      vector<float> corners_pix_x(C);
      vector<float> corners_pix_x_normalized(C);
      vector<float> corners_pix_y(C);
      vector<float> corners_joint(C);
      vector<float> corners_range(C);

      printf("Allocated matricies with C=%u\n", C) ;
      CvMat cv_corners_pix_x = cvMat(C, 1, CV_32FC1, &corners_pix_x[0]);
      CvMat cv_corners_pix_x_normalized = cvMat(C, 1, CV_32FC1, &corners_pix_x_normalized[0]);
      CvMat cv_corners_pix_y = cvMat(C, 1, CV_32FC1, &corners_pix_y[0]);
      CvMat cv_corners_joint = cvMat(C, 1, CV_32FC1, &corners_joint[0]);
      CvMat cv_corners_range = cvMat(C, 1, CV_32FC1, &corners_range[0]);
      printf("done allocating!") ;

      for (unsigned int i = 0; i < C; i++)
      {
        corners_pix_x[i] = pixel_corners_.pts[i].x;
        corners_pix_y[i] = pixel_corners_.pts[i].y;
      }

      float x_scale = (float)(joint_info_.data.layout.dim[1].size - 1)
        / (float)(dense_range_.data.layout.dim[1].size - 1);
      // corners_pix_x_normalized = corners_pix_x *
      cvConvertScale(&cv_corners_pix_x, &cv_corners_pix_x_normalized, x_scale);

      for (unsigned int i = 0; i < 12; i++)
      {
        printf("%2u) x: %.2f    norm(x): %.2f\n", i, corners_pix_x[i], corners_pix_x_normalized[i]);
        printf("    y: %.2f\n", corners_pix_y[i]);
        printf("    Range=%.2f\n", *( (float*) CV_MAT_ELEM_PTR( cv_ranges,
                                                                (int) corners_pix_y[i],
                                                                (int) corners_pix_x[i])) ) ;
      }

      cvRemap(&cv_joints, &cv_corners_joint, &cv_corners_pix_x_normalized, &cv_corners_pix_y);//, CV_INTER_LINEAR);
      cvRemap(&cv_ranges, &cv_corners_range, &cv_corners_pix_x, &cv_corners_pix_y);//, CV_INTER_LINEAR);

      printf("Range=\n") ;
      for (unsigned int i=0; i<6; i++)
        printf("%2u) joint: %.2f   range: %.2f\n", i, corners_joint[i], corners_range[i]) ;

      for (unsigned int i = 0; i < C; i++)
      {
        // Tilt angle
        measured_corners.pts[i].x = corners_joint[i];
        // Pointing angle
        measured_corners.pts[i].y = pointingAngle(scan_info_, pixel_corners_.pts[i].x);
        // Range
        measured_corners.pts[i].z = corners_range[i];
      }
    }

    node_->publish("dense_tilt_scan/measured_corners", measured_corners);

  }

  inline float pointingAngle(const sensor_msgs::LaserScan& info, const float& ray)
  {
    return info.angle_min + info.angle_increment * ray;
  }

  void msgTimeout(ros::Time t)
  {
    ROS_WARN("%f - Timeout", t.toSec());
    if (pixel_corners_.header.stamp != t)
      printf("- pixel_corners %f\n", pixel_corners_.header.stamp.toSec());
    else
      printf("+ pixel_corners %f\n", pixel_corners_.header.stamp.toSec());

    if (dense_range_.header.stamp != t)
      printf("- range         %f\n", dense_range_.header.stamp.toSec());
    else
      printf("+ range         %f\n", dense_range_.header.stamp.toSec());

    if (joint_info_.header.stamp != t)
      printf("- joint_info    %f\n", joint_info_.header.stamp.toSec());
    else
      printf("+ joint_info    %f\n", joint_info_.header.stamp.toSec());

    if (scan_info_.header.stamp != t)
      printf("- scan_info     %f\n", scan_info_.header.stamp.toSec());
    else
      printf("+ scan_info     %f\n", scan_info_.header.stamp.toSec());
  }

private:
  ros::Node* node_;
  TopicSynchronizer<LaserCBProcessing> sync_;

  robot_msgs::PointCloud pixel_corners_;
  dense_laser_assembler::Float32MultiArrayStamped dense_range_;
  dense_laser_assembler::Float32MultiArrayStamped joint_info_;
  sensor_msgs::LaserScan scan_info_;

};

int main(int argc, char** argv)
{
  float src[9] = {0, 1, 2,
                  3, 4, 5,
                  6, 7, 8};

  float dst[3] = {-1, -1, -1};

  float map_x[3] = {.5, 1.0, 1.5};
  float map_y[3] = {1.9, 1.0, 2.0};

  CvMat cv_src   = cvMat(3,3, CV_32FC1, src) ;
  CvMat cv_dst   = cvMat(1,3, CV_32FC1, dst) ;
  CvMat cv_map_x = cvMat(1,3, CV_32FC1, map_x) ;
  CvMat cv_map_y = cvMat(1,3, CV_32FC1, map_y) ;

  cvRemap(&cv_src, &cv_dst, &cv_map_x, &cv_map_y, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS) ;
  printf("dst=\n") ;
  for (int i=0; i<3; i++)
    printf("%.2f\n", dst[i]) ;

  ros::init(argc, argv);

  ros::Node node("laser_cb_processing");

  LaserCBProcessing cb_processing(&node);

  node.spin();
}

