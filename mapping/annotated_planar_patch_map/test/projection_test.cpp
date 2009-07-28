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

#include "robot_msgs/Polygon3D.h"
#include "geometry_msgs/Point32.h"
#include <math.h>


#include "angles/angles.h"

#include "tf/option_permutations.h"
#include "annotated_planar_patch_map/projection.h"


TEST(annotated_map, projectPolygon)
{
  robot_msgs::Polygon3D input_polygon;  
  robot_msgs::Polygon3D projected_polygon;  

  sensor_msgs::CamInfo cam_info;

  geometry_msgs::Point32 pt;

  pt.x=0;pt.y=0;pt.z=1;
  input_polygon.points.push_back(pt);

  pt.x=0.1;pt.y=-0.23;pt.z=0.86;
  input_polygon.points.push_back(pt);

  pt.x=0.86;pt.y=-0.01;pt.z=0.23;
  input_polygon.points.push_back(pt);


  double Kvalue[]={725.06115999999997, 0.0, 283.48507999999998, 0.0, 725.06115999999997, 237.56627, 0.0, 0.0, 1.0};
    //{715.70483000000002, 0.0, 309.17935, 0.0, 0.0, 715.70483000000002, 240.26486, 0.0, 0.0, 0.0, 1.0, 0.0};
  memcpy(&(cam_info.K[0]),Kvalue,9*sizeof(Kvalue[0]));

  annotated_planar_patch_map::projection::projectAnyObject(cam_info,input_polygon,projected_polygon);

  EXPECT_EQ(input_polygon.points.size(), projected_polygon.points.size());
  //EXPECT_NEAR(projected_polygon.points[0].x , 309, 1);

  printf("%f %f %f\n",projected_polygon.points[1].x,
         projected_polygon.points[1].y,
         projected_polygon.points[1].z);
  printf("%f %f %f\n",projected_polygon.points[2].x,
         projected_polygon.points[2].y,
         projected_polygon.points[2].z);

}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
