/*
 * visualization_utils.h
 *
 *  Created on: Jul 8, 2009
 *      Author: sturm
 */

#ifndef VISUALIZATION_UTILS_H_
#define VISUALIZATION_UTILS_H_

#include "ros/ros.h"

#include "sensor_msgs/PointCloud.h"
#include <robot_msgs/Polygon3D.h>
#include <visualization_msgs/Marker.h>

namespace vis_utils {

int HSV_to_RGB( float h, float s, float v);

float HSV_to_RGBf( float h, float s, float v);

float mix_color( float mix, float a, float b );

void visualizePlanes(const sensor_msgs::PointCloud& cloud,
                     std::vector<std::vector<int> >& plane_indices,
                     std::vector<sensor_msgs::PointCloud>& plane_cloud,
                     std::vector<std::vector<double> >& plane_coeff,
                     std::vector<float>& plane_color,
                     sensor_msgs::PointCloud& outside,
                     ros::Publisher& cloud_planes_pub,ros::Publisher& visualization_pub_);

void visualizePolygon(const sensor_msgs::PointCloud& cloud,robot_msgs::Polygon3D &polygon, int rgb, int id, ros::Publisher& visualization_pub );

}

#endif /* VISUALIZATION_UTILS_H_ */
