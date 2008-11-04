#include "trajectory/trajectory.h"
#include <gtest/gtest.h>

using namespace trajectory;

TEST(Trajectory, instantiation){
  Trajectory t(2);
  Trajectory::TPoint b(2);
  std::vector<double> a;
  std::vector<double> c;

  a.resize(8);
  c.resize(4);
  
  for(int i=0; i< 8; i++)
    a[i] = (double) i;

  for(int j=0; j<4; j++)
    c[j] = (double) j;

  ROS_INFO("Setting trajectory");
  t.setTrajectory(a,c,4);

  t.sample(b,1.5);

  ROS_INFO("Point: %f %f",b.q_[0],b.q_[1]);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
