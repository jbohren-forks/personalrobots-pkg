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
#include <tf/tf.h>
#include <math_utils/angles.h>
#include <sys/time.h>
#include "LinearMath/btVector3.h"


void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};

using namespace tf;


TEST(TimeCache, Repeatability)
{
  unsigned int runs = 100;

  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> values(runs);

  TransformStorage stor;
  stor.data_.setIdentity();
  
  for ( unsigned int i = 1; i < runs ; i++ )
  {
    values[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    std::stringstream ss;
    ss << values[i];
    stor.frame_id_ = ss.str();
    stor.parent_frame_id = i;
    stor.stamp_ = i;
    
    cache.insertData(stor);
  }



  for ( unsigned int i = 1; i < runs ; i++ )

  {
    cache.getData(i, stor);
    EXPECT_EQ(stor.parent_frame_id, i);
    EXPECT_EQ(stor.stamp_, i);
    std::stringstream ss;
    ss << values[i];
    EXPECT_EQ(stor.frame_id_, ss.str());
  }
  
}

TEST(TimeCache, ZeroAtFront)
{
  unsigned int runs = 100;

  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> values(runs);

  TransformStorage stor;
  stor.data_.setIdentity();
  
  for ( unsigned int i = 1; i < runs ; i++ )
  {
    values[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    std::stringstream ss;
    ss << values[i];
    stor.frame_id_ = ss.str();
    stor.parent_frame_id = i;
    stor.stamp_ = i;
    
    cache.insertData(stor);
  }

  stor.frame_id_ = "HEAD";
  stor.parent_frame_id = runs;
  stor.stamp_ = runs;
  cache.insertData(stor);
  


  for ( unsigned int i = 1; i < runs ; i++ )

  {
    cache.getData(i, stor);
    EXPECT_EQ(stor.parent_frame_id, i);
    EXPECT_EQ(stor.stamp_, i);
    std::stringstream ss;
    ss << values[i];
    EXPECT_EQ(stor.frame_id_, ss.str());
  }

  cache.getData(0, stor);
  EXPECT_EQ(stor.parent_frame_id, runs);
  EXPECT_EQ(stor.stamp_, runs);
  EXPECT_EQ(stor.frame_id_, std::string("HEAD"));

  stor.frame_id_ = "NEW_HEAD";
  stor.parent_frame_id = runs;
  stor.stamp_ = runs+1;
  cache.insertData(stor);


  //Make sure we get a different value now that a new values is added at the front
  cache.getData(0, stor);
  EXPECT_EQ(stor.parent_frame_id, runs);
  EXPECT_EQ(stor.stamp_, runs+1);
  EXPECT_NE(stor.frame_id_, std::string("HEAD"));
  
}

TEST(TimeCache, CartesianInterpolation)
{
  unsigned int runs = 100;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  unsigned int offset = 200;

  TransformStorage stor;
  stor.data_.setIdentity();
  
  for ( unsigned int i = 1; i < runs ; i++ )
  {

    for (unsigned int step = 0; step < 2 ; step++)
    {
      xvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      yvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      zvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    
      stor.data_.setOrigin(btVector3(xvalues[step], yvalues[step], zvalues[step]));
      stor.frame_id_ = "NO_NEED";
      stor.parent_frame_id = 2;
      stor.stamp_ = step * 100 + offset;
      cache.insertData(stor);
    }
    
    for (int pos = 0; pos < 100 ; pos ++)
    {
      cache.getData(offset + pos, stor);
      double x_out = stor.data_.getOrigin().x();
      double y_out = stor.data_.getOrigin().y();
      double z_out = stor.data_.getOrigin().z();
      EXPECT_TRUE(fabs(xvalues[0] + (xvalues[1] - xvalues[0]) * (double)pos/100.0 - x_out) < epsilon);
      EXPECT_TRUE(fabs(yvalues[0] + (yvalues[1] - yvalues[0]) * (double)pos/100.0 - y_out) < epsilon);
      EXPECT_TRUE(fabs(zvalues[0] + (zvalues[1] - zvalues[0]) * (double)pos/100.0 - z_out) < epsilon);
    }
    
    /// \todo check for interpolation not crossing between reparenting

    cache.clearList();
  }

  
}


TEST(TimeCache, CartesianExtrapolation)
{
  unsigned int runs = 100;
  double epsilon = 1e-5;
  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  unsigned int offset = 555;

  TransformStorage stor;
  stor.data_.setIdentity();
  
  for ( unsigned int i = 1; i < runs ; i++ )
  {

    for (unsigned int step = 0; step < 2 ; step++)
    {
      xvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      yvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      zvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    
      stor.data_.setOrigin(btVector3(xvalues[step], yvalues[step], zvalues[step]));
      stor.frame_id_ = "NO_NEED";
      stor.parent_frame_id = 2;
      stor.stamp_ = step * 100 + offset;
      cache.insertData(stor);
    }
    
    for (int pos = -200; pos < 300 ; pos ++)
    {
      cache.getData(offset + pos, stor);
      double x_out = stor.data_.getOrigin().x();
      double y_out = stor.data_.getOrigin().y();
      double z_out = stor.data_.getOrigin().z();
      EXPECT_TRUE(fabs(xvalues[0] + (xvalues[1] - xvalues[0]) * (double)pos/100.0 - x_out) < epsilon);
      EXPECT_TRUE(fabs(yvalues[0] + (yvalues[1] - yvalues[0]) * (double)pos/100.0 - y_out) < epsilon);
      EXPECT_TRUE(fabs(zvalues[0] + (zvalues[1] - zvalues[0]) * (double)pos/100.0 - z_out) < epsilon);
    }
    
    cache.clearList();
  }

  
}

/*
TEST(TimeCache, AngularInterpolation)
{
  unsigned int runs = 100;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> yawvalues(2);
  std::vector<double> pitchvalues(2);
  std::vector<double> rollvalues(2);

  unsigned int offset = 200;

  TransformStorage stor;
  stor.data_.setIdentity();
  
  for ( unsigned int i = 1; i < runs ; i++ )
  {

    for (unsigned int step = 0; step < 2 ; step++)
    {
      yawvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX / 100.0;
      pitchvalues[step] = 0;//10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      rollvalues[step] = 0;//10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    
      stor.data_.setRotation(btQuaternion(yawvalues[step], pitchvalues[step], rollvalues[step]));
      stor.frame_id_ = "NO_NEED";
      stor.parent_frame_id = 3;
      stor.stamp_ = step * 100 + offset;
      cache.insertData(stor);
    }
    
    for (int pos = 0; pos < 100 ; pos ++)
    {
      cache.getData(offset + pos, stor);
      btQuaternion quat;
      btScalar yaw_out, pitch_out, roll_out;
      stor.data_.getBasis().getEuler(yaw_out, pitch_out, roll_out);
      
      EXPECT_TRUE(fabs(yawvalues[0] + (yawvalues[1] - yawvalues[0]) * (double)pos/100.0 - yaw_out) < epsilon);
      EXPECT_EQ(yawvalues[0] + (yawvalues[1] - yawvalues[0]) * (double)pos/100.0, (double)yaw_out);
      /// \todo switch to quaternion or matrix so as not to have to deal with degeneracy etc.  This test isn't working at all
      //      EXPECT_TRUE(fabs(pitchvalues[0] + (pitchvalues[1] - pitchvalues[0]) * (double)pos/100.0 - pitch_out) < epsilon);
      //EXPECT_TRUE(fabs(rollvalues[0] + (rollvalues[1] - rollvalues[0]) * (double)pos/100.0 - roll_out) < epsilon);
    }
    
    cache.clearList();
  }

  
}
*/

/** \todo test insert out of order */

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
