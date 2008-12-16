#include "trajectory/trajectory.h"
#include <gtest/gtest.h>

using namespace trajectory;

TEST(Trajectory, samplingAfterCubic){
  Trajectory t(2);
  Trajectory::TPoint b(2);
  std::vector<double> a;

  std::vector<double> d;

  d.resize(2);

  d[0] = 1;
  d[1] = 1;

  t.setMaxRates(d);
  t.setMaxAcc(d);
  t.setInterpolationMethod("cubic");

  a.resize(8);
  
  for(int i=0; i< 8; i++)
    a[i] = (double) i;

  a[2] = -1; 
  a[4] = -2;
  a[6] = -3;

  t.setTrajectory(a,4);

  t.sample(b,1.5);

  t.write(std::string("junk.txt"),0.01);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
