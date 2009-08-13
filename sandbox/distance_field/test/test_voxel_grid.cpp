/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include <distance_field/voxel_grid.h>
#include <ros/ros.h>

using namespace distance_field;

TEST(TestVoxelGrid, TestReadWrite)
{
  int def=-100;
  VoxelGrid<int> vg(0.02,0.02,0.02,0.01,0,0,0, def);

  int numX = vg.getNumCells(VoxelGrid<int>::DIM_X);
  int numY = vg.getNumCells(VoxelGrid<int>::DIM_Y);
  int numZ = vg.getNumCells(VoxelGrid<int>::DIM_Z);

  vg.reset(0);

  int i=0;
  for (int x=0; x<numX; x++)
    for (int y=0; y<numY; y++)
      for (int z=0; z<numZ; z++)
      {
        vg.getCell(x,y,z) = i;
        i++;
      }

  i=0;
  for (int x=0; x<numX; x++)
    for (int y=0; y<numY; y++)
      for (int z=0; z<numZ; z++)
      {
        EXPECT_EQ(i, vg.getCell(x,y,z));
        i++;
      }

}

TEST(TestVoxelGrid, TestTimingLinear)
{
  int def=-100;
  VoxelGrid<int> vg(2.0,2.0,2.0,0.01,-0.5,-0.5,-0.5, def);

  int numX = vg.getNumCells(VoxelGrid<int>::DIM_X);
  int numY = vg.getNumCells(VoxelGrid<int>::DIM_Y);
  int numZ = vg.getNumCells(VoxelGrid<int>::DIM_Z);

  vg.reset(0);

  ros::WallTime start = ros::WallTime::now();
  for (int i=0; i<10; i++)
  {
    for (int x=0; x<numX; x++)
      for (int y=0; y<numY; y++)
        for (int z=0; z<numZ; z++)
          vg.getCellThroughPtr(x,y,z) += x*y*z*i;
  }
  ros::WallDuration ptrDuration = ros::WallTime::now() - start;
  ROS_INFO("Timing through ptrs: %f", ptrDuration.toSec());

  vg.reset(0);

  start = ros::WallTime::now();
  for (int i=0; i<10; i++)
  {
    for (int x=0; x<numX; x++)
      for (int y=0; y<numY; y++)
        for (int z=0; z<numZ; z++)
          vg.getCell(x,y,z) += x*y*z*i;
  }
  ros::WallDuration indexDuration = ros::WallTime::now() - start;
  ROS_INFO("Timing through index calculation: %f", indexDuration.toSec());

}

TEST(TestVoxelGrid, TestTimingRandom)
{
  int def=-100;
  VoxelGrid<int> vg(2.0,1.91,2.14,0.01,-0.5,-0.5,-0.5, def);

  int numX = vg.getNumCells(VoxelGrid<int>::DIM_X);
  int numY = vg.getNumCells(VoxelGrid<int>::DIM_Y);
  int numZ = vg.getNumCells(VoxelGrid<int>::DIM_Z);

  vg.reset(0);

  for (int i=0; i<10; i++)
  {
    for (int x=0; x<numX; x++)
      for (int y=0; y<numY; y++)
        for (int z=0; z<numZ; z++)
          vg.getCell(x,y,z) += x*y*z*i;
  }


  int num_accesses=1000000, sum=0, x=0, y=0, z=0;

  ros::WallTime start = ros::WallTime::now();
  for (int a=0; a<10; ++a)
  {
    sum=1000;
    x=1;
    y=2;
    z=3;
    for (int i=0; i<num_accesses; ++i)
    {
        sum += vg.getCellThroughPtr(x,y,z);
        x = (x+sum)%numX;
        if (x<0)
          x=-x;
        y = (y+sum)%numY;
        if (y<0)
          y=-y;
        z = (z+sum)%numZ;
        if (z<0)
          z=-z;
    }
  }
  ros::WallDuration ptrDuration = ros::WallTime::now() - start;
  ROS_INFO("Random timing through ptrs: %f, sum=%d", ptrDuration.toSec(), sum);

  start = ros::WallTime::now();
  for (int a=0; a<10; ++a)
  {
    sum=1000;
    x=1;
    y=2;
    z=3;
    for (int i=0; i<num_accesses; ++i)
    {
        sum += vg.getCell(x,y,z);
        x = (x+sum)%numX;
        if (x<0)
          x=-x;
        y = (y+sum)%numY;
        if (y<0)
          y=-y;
        z = (z+sum)%numZ;
        if (z<0)
          z=-z;
    }
  }
  ros::WallDuration indexDuration = ros::WallTime::now() - start;
  ROS_INFO("Random timing through index calculation: %f, sum=%d", indexDuration.toSec(), sum);
}

