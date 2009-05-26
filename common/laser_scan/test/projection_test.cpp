/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <sys/time.h>

#include "laser_scan/laser_scan.h"
#include "robot_msgs/PointCloud.h"
#include <math.h>


#include "angles/angles.h"

#define PROJECTION_TEST_RANGE_MIN (0.23)
#define PROJECTION_TEST_RANGE_MAX (40.0) 

laser_scan::LaserScan build_constant_scan(double range, double intensity, 
                                          double ang_min, double ang_max, double ang_increment,
                                          ros::Duration time_inc, ros::Duration scan_time)
{
  laser_scan::LaserScan scan;
  scan.header.stamp = ros::Time::now();
  scan.header.frame_id = "laser_frame";
  scan.angle_min = ang_min;
  scan.angle_max = ang_max;
  scan.angle_increment = ang_increment;
  scan.time_increment = time_inc.toSec();
  scan.scan_time = scan_time.toSec();
  scan.range_min = PROJECTION_TEST_RANGE_MIN;
  scan.range_max = PROJECTION_TEST_RANGE_MAX;
  unsigned int i = 0;
  for(; ang_min + i * ang_increment < ang_max; i++)
  {
    scan.ranges.push_back(range);
    scan.intensities.push_back(intensity);
  }

  return scan;
};


void test_getUnitVectors (float angle_min, float angle_max, float angle_increment)
{
  double tolerance = 1e-6;
  laser_scan::LaserProjection projector;  
  
  const boost::numeric::ublas::matrix<double> & mat = projector.getUnitVectors(angle_min, angle_max, angle_increment);
  
  

  for (unsigned int i = 0; i < mat.size2(); i++)
  {
    EXPECT_NEAR(angles::normalize_angle(atan2(mat(1,i), mat(0,i))),
                angles::normalize_angle(angle_min + i * angle_increment),
                tolerance);
  }
}

TEST(laser_scan, getUnitVectors)
{
  test_getUnitVectors(0, M_PI, M_PI/2.0);
  test_getUnitVectors(-M_PI, M_PI, M_PI/100.0);
  test_getUnitVectors(M_PI, 2.0 * M_PI, M_PI/40.0);

  test_getUnitVectors(-M_PI * 3.0/4.0, 3.0/4.0*M_PI, M_PI/180.0); // 270 @ one degree 
  test_getUnitVectors(-M_PI * 3.0/4.0, 3.0/4.0*M_PI, M_PI/360.0); // 270 @ half degree
  test_getUnitVectors(-M_PI * 3.0/4.0, 3.0/4.0*M_PI, M_PI/720.0); // 270 @ quarter degree
}

TEST(laser_scan, projectLaser)
{
  double tolerance = 1e-6;
  laser_scan::LaserProjection projector;  
  //\todo add more permutations
  laser_scan::LaserScan scan = build_constant_scan(1.0, 1.0, -M_PI/2, M_PI/2, M_PI/180, ros::Duration(1/400), ros::Duration(1/40));

  robot_msgs::PointCloud cloud_out;
  projector.projectLaser(scan, cloud_out);

  EXPECT_EQ(scan.ranges.size(), cloud_out.get_pts_size());
  for (unsigned int i = 0; i < cloud_out.pts.size(); i++)
  {
    EXPECT_NEAR(cloud_out.pts[i].x , scan.ranges[i] * cos(scan.angle_min + i * scan.angle_increment), tolerance);
    EXPECT_NEAR(cloud_out.pts[i].y , scan.ranges[i] * sin(scan.angle_min + i * scan.angle_increment), tolerance);
    EXPECT_NEAR(cloud_out.pts[i].z, 0, tolerance);
  };
  
}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
