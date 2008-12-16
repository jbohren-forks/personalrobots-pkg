#include "trajectory/trajectory.h"
#include <gtest/gtest.h>

using namespace trajectory;

TEST(Trajectory, samplingAfterInstantiationWithoutTimeBlendedLinear){
  Trajectory t(2);
  Trajectory::TPoint b(2);
  std::vector<double> a;

  std::vector<double> d;

  d.resize(2);

  d[0] = 1;
  d[1] = 1;

  t.setMaxRates(d);
  t.setMaxAcc(d);
  t.setInterpolationMethod("blended_linear");

  a.resize(8);
  
  for(int i=0; i< 8; i++)
    a[i] = (double) i;

  t.setTrajectory(a,4);
  t.sample(b,1.5);
  t.write(std::string("junk.txt"),0.01);

  a[2] = -1; 
  a[4] = -2;
  a[6] = -3;

  t.setTrajectory(a,4);
  t.sample(b,1.5);
  t.write(std::string("junk2.txt"),0.01);

}


TEST(Trajectory, samplingWithZeroInput){

  Trajectory t(2);
  Trajectory::TPoint b(2);

  std::vector<double> time;

  std::vector<double> a;
  std::vector<double> d;

  d.resize(2);

  d[0] = 1;
  d[1] = 1;

  t.setMaxRates(d);
  t.setMaxAcc(d);
  t.setInterpolationMethod("blended_linear");

  a.resize(8);
  time.resize(4);
  
  for(int i=0; i< 8; i++)
    a[i] = 0.0;

  for(int j=0; j < 4; j++)
   time[j] = double(j);

  t.setTrajectory(a,time,4);
  ROS_INFO("Setting trajectory done");
  t.sample(b,1.5);
  t.write(std::string("junk_zero.txt"),0.01);

}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
