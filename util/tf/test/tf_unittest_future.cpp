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


TEST(tf, ListOneForward)
{
  unsigned int runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child",  "my_parent");
    mTR.setTransform(tranStamped);
  }

  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "child");

    try{
    Stamped<Pose> outpose;
    outpose.data_.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("my_parent",inpose, outpose);
    EXPECT_NEAR(outpose.data_.getOrigin().x(), xvalues[i], epsilon);
    EXPECT_NEAR(outpose.data_.getOrigin().y(), yvalues[i], epsilon);
    EXPECT_NEAR(outpose.data_.getOrigin().z(), zvalues[i], epsilon);
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
  unsigned int runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child",  "my_parent");
    mTR.setTransform(tranStamped);
  }

  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "my_parent");

    try{
    Stamped<btTransform> outpose;
    outpose.data_.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("child",inpose, outpose);
    EXPECT_NEAR(outpose.data_.getOrigin().x(), -xvalues[i], epsilon);
    EXPECT_NEAR(outpose.data_.getOrigin().y(), -yvalues[i], epsilon);
    EXPECT_NEAR(outpose.data_.getOrigin().z(), -zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}


/** @todo Make this actually Assert something */

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

