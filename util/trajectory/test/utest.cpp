#include "trajectory/trajectory.h"
#include <gtest/gtest.h>

using namespace trajectory;

TEST(Trajectory, instantiationWithTime){
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

  t.setTrajectory(a,c,4);

  EXPECT_EQ(t.getNumberPoints(),4);
}

TEST(Trajectory, instantiationWithoutTime){
  Trajectory t(2);
  Trajectory::TPoint b(2);
  std::vector<double> a;

  std::vector<double> d;

  d.resize(2);

  d[0] = 1;
  d[1] = 1;

  t.setMaxRates(d);

  a.resize(8);
  
  for(int i=0; i< 8; i++)
    a[i] = (double) i;

  t.setTrajectory(a,4);
}

TEST(Trajectory, getDurationAfterInstantiationWithoutTime){
  Trajectory t(2);
  Trajectory::TPoint b(2);
  std::vector<double> a;
  std::vector<double> duration;
  std::vector<double> d;
  double durationScalar;

  duration.resize(3);
  d.resize(2);

  d[0] = 1;
  d[1] = 1;

  t.setMaxRates(d);

  a.resize(8);
  
  for(int i=0; i< 8; i++)
    a[i] = (double) i;

  t.setTrajectory(a,4);

  t.getDuration(duration);

  EXPECT_EQ(duration[0],2);
  EXPECT_EQ(duration[1],2);
  EXPECT_EQ(duration[2],2);

  t.getDuration(1,durationScalar);
  EXPECT_EQ(durationScalar,2);
}


TEST(Trajectory, sampling){
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

  t.setTrajectory(a,c,4);

  t.sample(b,1.5);

}

TEST(Trajectory, settingMaxRate){
  Trajectory t(2);
  std::vector<double> d;

  d.resize(2);

  d[0] = 1;
  d[1] = 1;

  t.setMaxRates(d);
}

TEST(Trajectory, samplingAfterInstantiationWithoutTime){
  Trajectory t(2);
  Trajectory::TPoint b(2);
  std::vector<double> a;

  std::vector<double> d;

  d.resize(2);

  d[0] = 1;
  d[1] = 1;

  t.setMaxRates(d);

  a.resize(8);
  
  for(int i=0; i< 8; i++)
    a[i] = (double) i;

  t.setTrajectory(a,4);

  t.sample(b,1.5);
  EXPECT_EQ(b.q_[0],1.5);
  EXPECT_EQ(b.q_[1],2.5);
}


TEST(Trajectory, minimumTimeLinearTrajectory){
  Trajectory t(2);
  Trajectory::TPoint b(2);
  std::vector<double> a;

  std::vector<double> d;

  d.resize(2);

  d[0] = 1;
  d[1] = 1;

  t.setMaxRates(d);

  a.resize(8);
  
  for(int i=0; i< 8; i++)
    a[i] = (double) i;

  t.setTrajectory(a,4);
  t.minimizeSegmentTimes();

  t.sample(b,1.5);
  EXPECT_EQ(b.q_[0],1.5);
  EXPECT_EQ(b.q_[1],2.5);
}

TEST(Trajectory, samplingAfterInstantiationWithoutTimeCubic){
  Trajectory t(2);
  Trajectory::TPoint b(2);
  std::vector<double> a;

  std::vector<double> d;

  d.resize(2);

  d[0] = 1;
  d[1] = 1;

  t.setMaxRates(d);
  t.setInterpolationMethod("cubic");
  a.resize(8);
  
  for(int i=0; i< 8; i++)
    a[i] = (double) i;

  t.setTrajectory(a,4);

  t.sample(b,1.5);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
