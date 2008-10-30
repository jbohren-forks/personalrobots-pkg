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

void generate_rand_vectors(double scale, uint64_t runs, std::vector<double>& xvalues, std::vector<double>& yvalues, std::vector<double>&zvalues)
{
  seed_rand();
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
  }
}


using namespace tf;



TEST(tf, TransformTransformsCartesian)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child", "my_parent");
    mTR.setTransform(tranStamped);

  }

  //std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "child");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("my_parent",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
  Stamped<Pose> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), runs, "child");
  Stamped<Pose> outpose;
  outpose.setIdentity(); //to make sure things are getting mutated
  mTR.transformPose("child",inpose, outpose);
  EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
  
  
}

/** Make sure that the edge cases of transform the top of the tree to the top of the tree and 
 * the leaf of the tree can transform to the leaf of the tree without a lookup exception and accurately */
TEST(tf, TransformTransformToOwnFrame)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs), yawvalues(runs),  pitchvalues(runs),  rollvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yawvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    pitchvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    rollvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(yawvalues[i],pitchvalues[i],rollvalues[i]), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child", "my_parent");
    mTR.setTransform(tranStamped);

  }

  //std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "child");
    Stamped<btTransform> inpose2 (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "parent");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("child",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
    EXPECT_NEAR(outpose.getRotation().w(), 1, epsilon); //Identity is 0,0,0,1


    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("parent",inpose2, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
    EXPECT_NEAR(outpose.getRotation().w(), 1, epsilon); //Identity is 0,0,0,1
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
  Stamped<Pose> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), runs, "child");
  Stamped<Pose> outpose;
  outpose.setIdentity(); //to make sure things are getting mutated
  mTR.transformPose("child",inpose, outpose);
  EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
  
  
}

TEST(tf, TransformPointCartesian)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child", "my_parent");
    mTR.setTransform(tranStamped);

  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    double x =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    double y =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    double z =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    Stamped<Point> invec (btVector3(x,y,z), 10 + i, "child");

    try{
    Stamped<Point> outvec(btVector3(0,0,0), 10 + i, "child");
    //    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPoint("my_parent",invec, outvec);
    EXPECT_NEAR(outvec.x(), xvalues[i]+x, epsilon);
    EXPECT_NEAR(outvec.y(), yvalues[i]+y, epsilon);
    EXPECT_NEAR(outvec.z(), zvalues[i]+z, epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, TransformVectorCartesian)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child", "my_parent");
    mTR.setTransform(tranStamped);

  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    double x =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    double y =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    double z =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    Stamped<Point> invec (btVector3(x,y,z), 10 + i, "child");

    try{
    Stamped<Vector3> outvec(btVector3(0,0,0), 10 + i, "child");
    //    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformVector("my_parent",invec, outvec);
    EXPECT_NEAR(outvec.x(), x, epsilon);
    EXPECT_NEAR(outvec.y(), y, epsilon);
    EXPECT_NEAR(outvec.z(), z, epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, TransformQuaternionCartesian)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;


    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child", "my_parent");
    mTR.setTransform(tranStamped);

  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<btQuaternion> invec (btQuaternion(xvalues[i],yvalues[i],zvalues[i]), 10 + i, "child");
    //    printf("%f, %f, %f\n", xvalues[i],yvalues[i], zvalues[i]);

    try{
    Stamped<btQuaternion> outvec(btQuaternion(xvalues[i],yvalues[i],zvalues[i]), 10 + i, "child");

    mTR.transformQuaternion("my_parent",invec, outvec);
    EXPECT_NEAR(outvec.angle(invec) , 0, epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(data, Vector3Conversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    btVector3 btv = btVector3(xvalues[i], yvalues[i], zvalues[i]);
    btVector3 btv_out = btVector3(0,0,0);
    std_msgs::Vector3 msgv;
    Vector3TFToMsg(btv, msgv);
    Vector3MsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
  } 
  
}

TEST(data, Vector3StampedConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    Stamped<btVector3> btv = Stamped<btVector3>(btVector3(xvalues[i], yvalues[i], zvalues[i]), 1000000000ULL, "no frame");
    Stamped<btVector3> btv_out;
    std_msgs::Vector3Stamped msgv;
    Vector3StampedTFToMsg(btv, msgv);
    Vector3StampedMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
    EXPECT_STREQ(btv.frame_id_.c_str(), btv_out.frame_id_.c_str());
    EXPECT_EQ(btv.stamp_, btv_out.stamp_);
  } 
}

TEST(data, QuaternionConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    btQuaternion btv = btQuaternion(xvalues[i], yvalues[i], zvalues[i]);
    btQuaternion btv_out = btQuaternion(0,0,0);
    std_msgs::Quaternion msgv;
    QuaternionTFToMsg(btv, msgv);
    QuaternionMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
    EXPECT_NEAR(btv.w(), btv_out.w(), epsilon);
  } 
  
}

TEST(data, QuaternionStampedConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    Stamped<btQuaternion> btv = Stamped<btQuaternion>(btQuaternion(xvalues[i], yvalues[i], zvalues[i]), 1000000000ULL, "no frame");
    Stamped<btQuaternion> btv_out;
    std_msgs::QuaternionStamped msgv;
    QuaternionStampedTFToMsg(btv, msgv);
    QuaternionStampedMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
    EXPECT_NEAR(btv.w(), btv_out.w(), epsilon);
    EXPECT_STREQ(btv.frame_id_.c_str(), btv_out.frame_id_.c_str());
    EXPECT_EQ(btv.stamp_, btv_out.stamp_);
  } 
}

TEST(data, TransformConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  std::vector<double> xvalues2(runs), yvalues2(runs), zvalues2(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    btTransform btv = btTransform(btQuaternion(xvalues2[i], yvalues2[i], zvalues2[i]), btVector3(xvalues[i], yvalues[i], zvalues[i]));
    btTransform btv_out;
    std_msgs::Transform msgv;
    TransformTFToMsg(btv, msgv);
    TransformMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.getOrigin().x(), btv_out.getOrigin().x(), epsilon);
    EXPECT_NEAR(btv.getOrigin().y(), btv_out.getOrigin().y(), epsilon);
    EXPECT_NEAR(btv.getOrigin().z(), btv_out.getOrigin().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().x(), btv_out.getRotation().x(), epsilon);
    EXPECT_NEAR(btv.getRotation().y(), btv_out.getRotation().y(), epsilon);
    EXPECT_NEAR(btv.getRotation().z(), btv_out.getRotation().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().w(), btv_out.getRotation().w(), epsilon);
  } 
  
}

TEST(data, TransformStampedConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  std::vector<double> xvalues2(runs), yvalues2(runs), zvalues2(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    Stamped<btTransform> btv = Stamped<btTransform>(btTransform(btQuaternion(xvalues2[i], yvalues2[i], zvalues2[i]), btVector3(xvalues[i], yvalues[i], zvalues[i])), 1000000000ULL, "no frame");
    Stamped<btTransform> btv_out;
    std_msgs::TransformStamped msgv;
    TransformStampedTFToMsg(btv, msgv);
    TransformStampedMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.getOrigin().x(), btv_out.getOrigin().x(), epsilon);
    EXPECT_NEAR(btv.getOrigin().y(), btv_out.getOrigin().y(), epsilon);
    EXPECT_NEAR(btv.getOrigin().z(), btv_out.getOrigin().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().x(), btv_out.getRotation().x(), epsilon);
    EXPECT_NEAR(btv.getRotation().y(), btv_out.getRotation().y(), epsilon);
    EXPECT_NEAR(btv.getRotation().z(), btv_out.getRotation().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().w(), btv_out.getRotation().w(), epsilon);
    EXPECT_STREQ(btv.frame_id_.c_str(), btv_out.frame_id_.c_str());
    EXPECT_EQ(btv.stamp_, btv_out.stamp_);
  } 
}

TEST(tf, ListOneForward)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child",  "my_parent");
    mTR.setTransform(tranStamped);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "child");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("my_parent",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}


TEST(tf, ListOneInverse)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child",  "my_parent");
    mTR.setTransform(tranStamped);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "my_parent");

    try{
    Stamped<btTransform> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("child",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), -xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), -yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), -zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}


TEST(tf, getParent)
{
  
  std::vector<std::string> children;
  std::vector<std::string> parents;

  children.push_back("a");
  parents.push_back("c");

  children.push_back("b");
  parents.push_back("c");

  children.push_back("c");
  parents.push_back("e");

  children.push_back("d");
  parents.push_back("e");

  children.push_back("e");
  parents.push_back("f");

  children.push_back("f");
  parents.push_back("j");

  tf::Transformer mTR(true);

  for (uint64_t i = 0; i <  children.size(); i++)
    {
      Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10ULL , children[i],  parents[i]);
      mTR.setTransform(tranStamped);
    }

  //std::cout << mTR.allFramesAsString() << std::endl;

  std::string output;
  for  (uint64_t i = 0; i <  children.size(); i++)
    {
      EXPECT_TRUE(mTR.getParent(children[i], 10ULL, output));
      EXPECT_STREQ(parents[i].c_str(), output.c_str());
    }
  
  EXPECT_FALSE(mTR.getParent("j", 10ULL, output));

  EXPECT_FALSE(mTR.getParent("no_value", 10ULL, output));
  
}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
