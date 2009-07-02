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

#include "robot_msgs/PointCloud.h"
#include "robot_msgs/Point32.h"
#include <math.h>


#include "angles/angles.h"

#include "tf/option_permutations.h"

#include "pcd_misc.h"


using namespace robot_msgs;

Point32 mk_pt(float x,float y,float z)
{
  Point32 p;
  p.x=x; p.y=y; p.z=z;
  return p;
}

TEST(labeled_object_detector, pcdSegmentationUF)
{
  PointCloud cloud_in;  
  cloud_in.pts.resize(3);

  cloud_in.pts[0]=mk_pt(0.0,0.0,0.0); //cluster 0
  cloud_in.pts[1]=mk_pt(0.1,0.0,0.0); //cluster 0
  cloud_in.pts[2]=mk_pt(1.0,0.0,0.0); //cluster 1


  std::vector<int> cluster_ids;
  unsigned int num_clusters;
  pcd_misc::cluster_pcd_points(cloud_in,0.2,cluster_ids,num_clusters);


  EXPECT_EQ(num_clusters,2);
  EXPECT_EQ(cluster_ids.size(),3);

  EXPECT_EQ(cluster_ids[0],0);
  EXPECT_EQ(cluster_ids[1],0);
  EXPECT_EQ(cluster_ids[2],1);

  std::vector<std::vector<int> >clouds_by_indices_out;

  pcd_misc::cluster_ids_to_cluster_indices(cluster_ids, num_clusters, clouds_by_indices_out );

  EXPECT_EQ(clouds_by_indices_out[0][0],0);
  EXPECT_EQ(clouds_by_indices_out[0][1],1);
  EXPECT_EQ(clouds_by_indices_out[1][0],2);
}


TEST(labeled_object_detector, pcdVariationAlongLine)
{
  PointCloud cloud_in;  
  cloud_in.pts.resize(3);

  cloud_in.pts[0]=mk_pt(0.0,2.0,0.0); //cluster 0
  cloud_in.pts[1]=mk_pt(0.1,1.0,0.0); //cluster 0
  cloud_in.pts[2]=mk_pt(1.0,0.0,0.0); //cluster 1

  Point32 dir_line=mk_pt(1,0,0);
  Point32 offset=mk_pt(0,0,0);

  std::vector<int> indices;
  indices.resize(cloud_in.pts.size());
  for(unsigned int i=0;i<cloud_in.pts.size();i++)
    indices[i]=i;

  float min_v,max_v;
  pcd_misc::variationAlongLine(dir_line,offset,cloud_in,indices,min_v,max_v);

  EXPECT_FLOAT_EQ(min_v,0.0);
  EXPECT_FLOAT_EQ(max_v,1.0);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
