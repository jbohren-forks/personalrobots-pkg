#include <gtest/gtest.h>
#include <tf/tf.h>
#include <math_utils/angles.h>
#include <sys/time.h>

#include "LinearMath/btVector3.h"

using namespace tf;

void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};

TEST(tf, NoExtrapolationExceptionFromParent)
{
  tf::Transformer mTR(true, ros::Duration(1000000000000LL), ros::Duration(0LL));
  


  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(1.0), "a",  "parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(10.0), "a",  "parent"));


  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(1.0), "b",  "parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(10.0), "b",  "parent"));

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(1.0), "parent",  "parent's parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(1.0), "parent's parent",  "parent's parent's parent"));

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(10.0), "parent",  "parent's parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(10.0), "parent's parent",  "parent's parent's parent"));

  Stamped<Point> output;

  try
  {
    mTR.transformPoint( "b", Stamped<Point>(Point(1,1,1), ros::Time(20.0), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    EXPECT_FALSE("Shouldn't have gotten this exception");
  }



};




TEST(tf, ExtrapolationFromOneValue)
{
  tf::Transformer mTR(true, ros::Duration(1000000000000LL), ros::Duration(0LL));
  


  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(1.0), "a",  "parent"));

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(1.0), "parent",  "parent's parent"));


  Stamped<Point> output;

  bool excepted = false;
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time(10.0), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_TRUE(excepted);

  excepted = false;
  try
  {
    mTR.transformPoint( "parent's parent", Stamped<Point>(Point(1,1,1), ros::Time(10.0), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_TRUE(excepted);

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(20.0), "a",  "parent"));

  excepted = false;
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time(10.0), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_FALSE(excepted);

};




/** @todo Make this actually Assert something */

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

