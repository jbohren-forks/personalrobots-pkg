/*
 * find_planes.h
 *
 *  Created on: Jul 7, 2009
 *      Author: sturm
 */

#ifndef FIND_PLANES_H_
#define FIND_PLANES_H_

#include "ros/ros.h"

#include "sensor_msgs/PointCloud.h"
#include <robot_msgs/Polygon3D.h>
#include <visualization_msgs/Marker.h>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

namespace find_planes
{

void filterByZBounds(const sensor_msgs::PointCloud& pc, double zmin, double zmax, sensor_msgs::PointCloud& filtered_pc,
                     sensor_msgs::PointCloud& filtered_outside);
void getPointIndicesInZBounds(const sensor_msgs::PointCloud &points, double z_min, double z_max,
                              std::vector<int> &indices);

bool fitSACPlanes(sensor_msgs::PointCloud *points, std::vector<int> &indices, std::vector<std::vector<int> > &inliers,
                  std::vector<std::vector<double> > &coeff, const geometry_msgs::Point32 &viewpoint_cloud,
                  double dist_thresh, int n_max, int min_points_per_model);
void segmentPlanes(const sensor_msgs::PointCloud &points, double sac_distance_threshold, double z_min, double z_max,
                   double support, double min_area, int n_max, std::vector<std::vector<int> > &indices, std::vector<
                       std::vector<double> > &models, int number);

void findPlanes(const sensor_msgs::PointCloud& cloud, int n_planes_max, double sac_distance_threshold, std::vector<
    std::vector<int> >& indices, std::vector<sensor_msgs::PointCloud>& plane_cloud,
                std::vector<std::vector<double> >& plane_coeff, sensor_msgs::PointCloud& outside);

void createPlaneImage(const sensor_msgs::PointCloud& cloud, std::vector<int> &inliers, std::vector<double> &plane_coeff,
                      IplImage *pixOccupied,IplImage *pixFree,IplImage *pixUnknown );

}

#endif /* FIND_PLANES_H_ */
