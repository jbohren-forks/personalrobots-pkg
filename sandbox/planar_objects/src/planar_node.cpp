/*
 * planar_node.cpp
 *
 *  Created on: Jul 7, 2009
 *      Author: sturm
 */

#include "planar_node.h"
#include "find_planes.h"
#include "vis_utils.h"

#include "opencv_latest/CvBridge.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace ros;
using namespace std;
using namespace robot_msgs;

// Constructor
PlanarNode::PlanarNode() :
  sync_(&PlanarNode::syncCallback, this)
{
  cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("plane", CV_WINDOW_AUTOSIZE);

  nh_.param("~n_planes_max", n_planes_max_, 4);
  nh_.param("~point_plane_distance", point_plane_distance_, 0.015);

  // subscribe to topics
  cloud_sub_ = nh_.subscribe("stereo/cloud", 1, sync_.synchronize(&PlanarNode::cloudCallback, this));
  disp_sub_ = nh_.subscribe("stereo/disparity", 1, sync_.synchronize(&PlanarNode::dispCallback, this) );
  dinfo_sub_ = nh_.subscribe("stereo/disparity_info", 1, sync_.synchronize(&PlanarNode::dinfoCallback, this) );

  // advertise topics
  cloud_planes_pub_ = nh_.advertise<PointCloud> ("~planes", 1);
  cloud_outliers_pub_ = nh_.advertise<PointCloud> ("~outliers", 1);
  visualization_pub_ = nh_.advertise<visualization_msgs::Marker> ("visualization_marker", 1);
}

void PlanarNode::cloudCallback(const robot_msgs::PointCloud::ConstPtr& point_cloud)
{
  cloud_ = point_cloud;
}

void PlanarNode::dinfoCallback(const sensor_msgs::DisparityInfo::ConstPtr& dinfo) {
  dinfo_ = dinfo;
}

void PlanarNode::dispCallback(const sensor_msgs::Image::ConstPtr& disp_img)
{
  dimage_ = disp_img;

  if (dbridge_.fromImage(*dimage_))
  {
    // Disparity has to be scaled to be be nicely displayable
    IplImage* disp = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
    cvCvtScale(dbridge_.toIpl(), disp, 4.0/dinfo_->dpp);
    cvShowImage("disparity", disp);
    cvReleaseImage(&disp);
  }
}

void PlanarNode::syncCallback()
{
  ROS_INFO("PlanarNode::syncCallback(), %d points in cloud",cloud_->get_pts_size());



  vector<PointCloud> plane_cloud;
  vector<vector<double> > plane_coeff;
  vector<vector<int> > plane_indices;
  PointCloud outside;

  find_planes::findPlanes(*cloud_, n_planes_max_, point_plane_distance_, plane_indices, plane_cloud, plane_coeff,outside);
  vis_utils::visualizePlanes(*cloud_,plane_indices,plane_cloud,plane_coeff,outside,cloud_planes_pub_,visualization_pub_);

  IplImage* planeImage = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  find_planes::createPlaneImage(*cloud_, plane_indices[1], plane_coeff[1], planeImage);
  cvShowImage("plane", planeImage);
  cvReleaseImage(&planeImage);
}

bool PlanarNode::spin()
{
  while (nh_.ok())
  {
    int key = cvWaitKey(100) & 0x00FF;
    if (key == 27) //ESC
      break;

    ros::spinOnce();
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planar_node");

  PlanarNode node;
  node.spin();

  return 0;
}
