/*
 * cornercandidate.h
 *
 *  Created on: Jul 28, 2009
 *      Author: sturm
 */

#ifndef CORNERCANDIDATE_H_
#define CORNERCANDIDATE_H_

#include "ros/ros.h"
#include "topic_synchronizer2/topic_synchronizer.h"

#include "robot_msgs/PointCloud.h"

#include "visualization_msgs/Marker.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/StereoInfo.h"
#include "sensor_msgs/DisparityInfo.h"
#include "sensor_msgs/CamInfo.h"

#include "opencv_latest/CvBridge.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

// transform library
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace planar_objects {

class CornerCandidate
{
public:
  double *P;
  double *RP;
  double rect_min_size;
  double rect_max_size;
  double rect_max_displace;

  btTransform tf;

  double x;
  double y;

  double angle;
  double dist1;
  double dist2;

  double w;
  double h;

  double precision;
  double recall;
  int plane_id;

  btVector3 points3d[4];
  CvPoint points2d[4];

  void updatePoints3d();
  void updatePoints2d();
  double computeDistance(IplImage* distImage);
  double computeSupport2d(IplImage* pixOccupied, IplImage* pixDebug = NULL);
  double computeSupport3d(const sensor_msgs::PointCloud& cloud,
                          std::vector<int> & plane_indices);
  void optimizeWidth(IplImage* distImage, double a = -0.5, double b = +0.5, int steps = 20);
  void optimizeHeight(IplImage* distImage, double a = -0.5, double b = +0.5, int steps = 20);
  void optimizePhi(IplImage* distImage, double a = -0.5, double b = +0.5, int steps = 20);
  void optimizeX(IplImage* distImage, double a = -0.5, double b = +0.5, int steps = 20);
  void optimizeY(IplImage* distImage, double a = -0.5, double b = +0.5, int steps = 20);
};

}

#endif /* CORNERCANDIDATE_H_ */
