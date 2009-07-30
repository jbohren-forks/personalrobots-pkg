/*
 * find_planes.h
 *
 *  Created on: Jul 7, 2009
 *      Author: sturm
 */

#ifndef FIND_PLANES_H_
#define FIND_PLANES_H_

#include "ros/ros.h"

#include "robot_msgs/PointCloud.h"
#include <robot_msgs/Polygon3D.h>
#include <visualization_msgs/Marker.h>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

namespace planar_objects
{

void filterByZBounds(const robot_msgs::PointCloud& pc, double zmin, double zmax, robot_msgs::PointCloud& filtered_pc,
                     robot_msgs::PointCloud& filtered_outside);
void getPointIndicesInZBounds(const robot_msgs::PointCloud &points, double z_min, double z_max,
                              std::vector<int> &indices);

bool fitSACPlanes(robot_msgs::PointCloud *points, std::vector<int> &indices, std::vector<std::vector<int> > &inliers,
                  std::vector<std::vector<double> > &coeff, const robot_msgs::Point32 &viewpoint_cloud,
                  double dist_thresh, int n_max, int min_points_per_model);
void segmentPlanes(const robot_msgs::PointCloud &points, double sac_distance_threshold, double z_min, double z_max,
                   double support, double min_area, int n_max, std::vector<std::vector<int> > &indices, std::vector<
                       std::vector<double> > &models, int number);

void findPlanes(const robot_msgs::PointCloud& cloud, int n_planes_max, double sac_distance_threshold, std::vector<
    std::vector<int> >& indices, std::vector<robot_msgs::PointCloud>& plane_cloud,
                std::vector<std::vector<double> >& plane_coeff, robot_msgs::PointCloud& outside);

void createPlaneImage(const robot_msgs::PointCloud& cloud, std::vector<int> &inliers, std::vector<double> &plane_coeff,
                      IplImage *pixOccupied,IplImage *pixFree,IplImage *pixUnknown );

}

#endif /* FIND_PLANES_H_ */
