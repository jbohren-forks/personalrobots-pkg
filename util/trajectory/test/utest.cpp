#include "trajectory/trajectory.h"
#include <gtest/gtest.h>

using namespace trajectory;

TEST(Trajectory, instantiation){
  Trajectory t(2);
  Trajectory::TPoint b(2);
  std::vector<double> a;

  a.resize(12);
  
  for(int i=0; i< 12; i++)
    a[i] = (double) i;

  ROS_INFO("Setting trajectory");
  t.setTrajectory(a,4);

  t.sample(b,1.5);

  ROS_INFO("Point: %f %f",b.q_[0],b.q_[1]);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
