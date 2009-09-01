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
#include <geometry_msgs/Polygon.h>
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

extern ros::Publisher* vis_utils_vis_pub;
extern ros::Publisher* vis_utils_cloud_pub;
extern roslib::Header vis_utils_header;

void setVisualization(ros::Publisher *visualization_pub_, ros::Publisher *cloud_pub_, roslib::Header header);

void visualizePlanes2(const sensor_msgs::PointCloud& cloud,
                     std::vector<std::vector<int> >& plane_indices,
                     std::vector<sensor_msgs::PointCloud>& plane_cloud,
                     std::vector<std::vector<double> >& plane_coeff,
                     std::vector<float>& plane_color,
                     sensor_msgs::PointCloud& outside,
                     bool convexHull=false);

void visualizePolygon(const sensor_msgs::PointCloud& cloud, geometry_msgs::Polygon &polygon, int rgb, int id);

void visualizeLines(std::vector<std::pair<btVector3,
    btVector3> > lines, int id, double r, double b , double g,double scale=0.002 );

void visualizeLines(std::vector<std::pair<btVector3,
    btVector3> > lines, int id, int rgb,double scale=0.002);

}

#endif /* VISUALIZATION_UTILS_H_ */
