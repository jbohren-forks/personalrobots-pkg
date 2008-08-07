#include <gtest/gtest.h>
#include <libTF/Pose3D.h>

using namespace libTF;

TEST(Pose3D, DefaultConstructor){
  Pose3D p;
//EXPECT_EQ(0, p.getMagnitude());
  Pose3D::Euler e = p.getEuler();
  EXPECT_EQ(0, e.yaw);
  EXPECT_EQ(0, e.pitch);
  EXPECT_EQ(0, e.roll);
  Pose3D::Quaternion q = p.getQuaternion();
  EXPECT_EQ(0, q.x);
  EXPECT_EQ(0, q.y);
  EXPECT_EQ(0, q.z);
  EXPECT_EQ(1, q.w);
  Pose3D::Position x = p.getPosition();
  EXPECT_EQ(0, x.x);
  EXPECT_EQ(0, x.y);
  EXPECT_EQ(0, x.z);
}

void testEulerConversion(double yaw, double pitch, double roll){
  Pose3D p;
  p.setFromEuler(0, 0, 0, yaw, pitch, roll);
  Pose3D::Euler e = p.getEuler();
  EXPECT_FLOAT_EQ(e.yaw, yaw);
  EXPECT_FLOAT_EQ(e.pitch, pitch);
  EXPECT_FLOAT_EQ(e.roll, roll);
}

TEST(Pose3D, EulerConversions){
  Pose3D p;
  testEulerConversion(0, 0, 0);
  testEulerConversion(1, 0, 0);
  testEulerConversion(0, 1, 0);
  testEulerConversion(0, 0, 1);
  testEulerConversion(1, 1, 0);
  testEulerConversion(0, 1, 1);
  testEulerConversion(1, 0, 1);
  testEulerConversion(1, 1, 1);
  testEulerConversion(-1, -1, -1);
  testEulerConversion(0, 0, 3.14);
  testEulerConversion(0, 0, -3.14);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
