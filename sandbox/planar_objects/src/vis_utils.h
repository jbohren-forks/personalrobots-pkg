/*
 * visualization_utils.h
 *
 *  Created on: Jul 8, 2009
 *      Author: sturm
 */

#ifndef VISUALIZATION_UTILS_H_
#define VISUALIZATION_UTILS_H_

#include "ros/ros.h"

#include "robot_msgs/PointCloud.h"
#include <robot_msgs/Polygon3D.h>
#include <visualization_msgs/Marker.h>
// transform library
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace planar_objects
{

typedef std::vector<std::pair<btVector3,btVector3> > LineVector;

int HSV_to_RGB(float h, float s, float v);

float HSV_to_RGBf(float h, float s, float v);

float mix_color(float mix, float a, float b);

void visualizePlanes2(const robot_msgs::PointCloud& cloud, std::vector<std::vector<int> >& plane_indices, std::vector<
    robot_msgs::PointCloud>& plane_cloud, std::vector<std::vector<double> >& plane_coeff,
                     std::vector<float>& plane_color, robot_msgs::PointCloud& outside,
                     ros::Publisher& cloud_planes_pub, ros::Publisher& visualization_pub_, bool convexHull = false);

void visualizePolygon(const robot_msgs::PointCloud& cloud, robot_msgs::Polygon3D &polygon, int rgb, int id,
                      ros::Publisher& visualization_pub);

void visualizeLines(ros::Publisher& visualization_pub_, std::string frame_id, std::vector<std::pair<btVector3,
    btVector3> > lines, int id, double r, double b , double g );

void visualizeLines(ros::Publisher& visualization_pub_, std::string frame_id, std::vector<std::pair<btVector3,
    btVector3> > lines, int id, int rgb);

}

#endif /* VISUALIZATION_UTILS_H_ */
