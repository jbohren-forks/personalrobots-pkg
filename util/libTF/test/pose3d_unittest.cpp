#include <gtest/gtest.h>
#include <libTF/Pose3D.h>
#include <math_utils/angles.h>
#include <sys/time.h>
#include <cstdlib>

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

TEST(Pose3D, MatrixToQuaternionAndBack)
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);

  libTF::Pose3D aPose;

  int i_max = 100;

  //Test many different premutations
  for (int i = -i_max; i < i_max; i++)
  {

    double yaw, pitch, roll;
    yaw = 2* M_PI * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    pitch = 2 * M_PI * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    roll = 2 *M_PI * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    
      
    NEWMAT::Matrix m = libTF::Pose3D::matrixFromEuler(0.0,
                                                      0.0,
                                                      0.0,
                                                      yaw,
                                                      pitch,
                                                      roll);
      
    //          std::cout << m <<std::endl;
    aPose.setFromMatrix(m);
    m = aPose.asMatrix();
          
    libTF::Pose3D::Euler out = libTF::Pose3D::eulerFromMatrix(m);
    libTF::Pose3D::Euler out2 = libTF::Pose3D::eulerFromMatrix(m,2);

    //Test the difference between input and output accounting for 2Pi redundancy.  
    bool difference = ((fabs(math_utils::modNPiBy2(out.yaw) - math_utils::modNPiBy2(yaw)) > 0.001 || fabs(math_utils::modNPiBy2(out.pitch) - math_utils::modNPiBy2(pitch)) > 0.001 || fabs(math_utils::modNPiBy2(out.roll) -math_utils::modNPiBy2(roll)) > 0.0001) &&
                       (fabs(math_utils::modNPiBy2(out2.yaw) - math_utils::modNPiBy2(yaw)) > 0.001 || fabs(math_utils::modNPiBy2(out2.pitch) - math_utils::modNPiBy2(pitch)) > 0.001 || fabs(math_utils::modNPiBy2(out2.roll) -math_utils::modNPiBy2(roll)) > 0.0001));
          
    ASSERT_FALSE(difference);
  }

}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
