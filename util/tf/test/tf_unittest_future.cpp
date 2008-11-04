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
  tf::Transformer mTR(true, ros::Duration((int64_t)1000000LL), ros::Duration((int64_t)0LL));
  


  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(1000ULL), "a",  "parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(10000ULL), "a",  "parent"));


  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(1000ULL), "b",  "parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(10000ULL), "b",  "parent"));

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(1000ULL), "parent",  "parent's parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(1000ULL), "parent's parent",  "parent's parent's parent"));

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(10000ULL), "parent",  "parent's parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(10000ULL), "parent's parent",  "parent's parent's parent"));

  Stamped<Point> output;

  try
  {
    mTR.transformPoint( "parent's parent", Stamped<Point>(Point(1,1,1), ros::Time(20000ULL), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    EXPECT_FALSE("Shouldn't have gotten this exception");
  }



};




TEST(tf, ExtrapolationFromOneValue)
{
  tf::Transformer mTR(true, ros::Duration((int64_t)1000000LL), ros::Duration((int64_t)0LL));
  


  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(1000ULL), "a",  "parent"));

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(1000ULL), "parent",  "parent's parent"));


  Stamped<Point> output;

  bool excepted = false;
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time(10000ULL), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_TRUE(excepted);

  excepted = false;
  try
  {
    mTR.transformPoint( "parent's parent", Stamped<Point>(Point(1,1,1), ros::Time(10000ULL), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_TRUE(excepted);

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(20000ULL), "a",  "parent"));

  excepted = false;
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time(10000ULL), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_FALSE(excepted);

};

TEST(tf, SignFlipExtrapolate)
{
  double epsilon = 1e-3;

  double truex, truey, trueyaw1, trueyaw2;

  truex = 5.220;
  truey = 1.193;
  trueyaw1 = 2.094;
  trueyaw2 = 2.199;
  ros::Time ts0;
  ts0.fromSec(46.6);
  ros::Time ts1;
  ts1.fromSec(46.7);
  ros::Time ts2;
  ts2.fromSec(46.8);
  
  TransformStorage tout;
  double yaw, pitch, roll;

  TransformStorage t0(Stamped<btTransform>
                      (btTransform(btQuaternion(0.000, 0.000,  -0.8386707128751809, 0.5446388118427071),
                                   btVector3(1.0330764266905630, 5.2545257423922198, -0.000)),
                       ts0, "odom"), 3);
  TransformStorage t1(Stamped<btTransform>
                      (btTransform(btQuaternion(0.000, 0.000,  0.8660255375641606, -0.4999997682866531),
                                   btVector3(1.5766646418987809, 5.1177550046707436, -0.000)),
                       ts1, "odom"), 3);
  TransformStorage t2(Stamped<btTransform>
                      (btTransform(btQuaternion(0.000, 0.000, 0.8910066733792211, -0.4539902069358919),
                                   btVector3(2.1029791754869160, 4.9249128183465967, -0.000)),
                       ts2, "odom"), 3);

  tf::TimeCache tc;
  btTransform res;

  tc.interpolate(t0, t1, ts1, tout);
  res = tout.inverse();
  res.getBasis().getEulerZYX(yaw,pitch,roll);

  EXPECT_NEAR(res.getOrigin().x(), truex, epsilon);
  EXPECT_NEAR(res.getOrigin().y(), truey, epsilon);
  EXPECT_NEAR(yaw, trueyaw1, epsilon);

  tc.interpolate(t0, t1, ts2, tout);
  res = tout.inverse();
  res.getBasis().getEulerZYX(yaw,pitch,roll);

  EXPECT_NEAR(res.getOrigin().x(), truex, epsilon);
  EXPECT_NEAR(res.getOrigin().y(), truey, epsilon);
  EXPECT_NEAR(yaw, trueyaw2, epsilon);

  tc.interpolate(t1, t2, ts2, tout);
  res = tout.inverse();
  res.getBasis().getEulerZYX(yaw,pitch,roll);

  EXPECT_NEAR(res.getOrigin().x(), truex, epsilon);
  EXPECT_NEAR(res.getOrigin().y(), truey, epsilon);
  EXPECT_NEAR(yaw, trueyaw2, epsilon);
}



/** @todo Make this actually Assert something */

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

