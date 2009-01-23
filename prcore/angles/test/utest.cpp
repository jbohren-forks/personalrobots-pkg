#include "angles/angles.h"
#include <gtest/gtest.h>

TEST(Angles, shortestDistanceWithLimits){
  double shortest_angle;
  bool result = angles::shortest_angular_distance_with_limits(-0.5, 0.5,-0.25,0.25,shortest_angle);
  EXPECT_TRUE(!result);

  result = angles::shortest_angular_distance_with_limits(-0.5, 0.5,0.25,0.25,shortest_angle);
  EXPECT_TRUE(!result);

  result = angles::shortest_angular_distance_with_limits(-0.5, 0.5,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(result);
  EXPECT_NEAR(shortest_angle, -2*M_PI+1.0,1e-6);

  result = angles::shortest_angular_distance_with_limits(0.5, 0.5,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(result);
  EXPECT_NEAR(shortest_angle, 0,1e-6);

  result = angles::shortest_angular_distance_with_limits(0.5, 0,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(!result);
  EXPECT_NEAR(shortest_angle, -0.5,1e-6);

  result = angles::shortest_angular_distance_with_limits(-0.5, 0,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(!result);
  EXPECT_NEAR(shortest_angle, 0.5,1e-6);

  result = angles::shortest_angular_distance_with_limits(-0.2,0.2,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(!result);
  EXPECT_NEAR(shortest_angle, -2*M_PI+0.4,1e-6);

  result = angles::shortest_angular_distance_with_limits(0.2,-0.2,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(!result);
  EXPECT_NEAR(shortest_angle,2*M_PI-0.4,1e-6);

  result = angles::shortest_angular_distance_with_limits(0.2,0,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(!result);
  EXPECT_NEAR(shortest_angle,2*M_PI-0.2,1e-6);

  result = angles::shortest_angular_distance_with_limits(-0.2,0,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(!result);
  EXPECT_NEAR(shortest_angle,-2*M_PI+0.2,1e-6);

  result = angles::shortest_angular_distance_with_limits(-0.25,-0.5,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(result);
  EXPECT_NEAR(shortest_angle,-0.25,1e-6);

  result = angles::shortest_angular_distance_with_limits(-0.25,0.5,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(result);
  EXPECT_NEAR(shortest_angle,-2*M_PI+0.75,1e-6);

  result = angles::shortest_angular_distance_with_limits(-0.2500001,0.5,0.25,-0.25,shortest_angle);
  EXPECT_TRUE(result);
  EXPECT_NEAR(shortest_angle,-2*M_PI+0.5+0.2500001,1e-6);

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
