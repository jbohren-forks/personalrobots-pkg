#include <gtest/gtest.h>
#include <libTF/Pose3D.h>
#include <math_utils/angles.h>
#include <sys/time.h>
#include <cstdlib>

  //Seed random number generator with current microseond count
void seed_rand(){
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};

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
TEST(Vector, DefaultConstructor){
  Pose3D::Vector v;
  EXPECT_EQ(0, v.x);
  EXPECT_EQ(0, v.y);
  EXPECT_EQ(0, v.z);
}

TEST(Vector, Addition){
  seed_rand();
  Pose3D::Vector a,b,c;
for (unsigned int i = 0; i < 1000; i++)
 {  
   a.x = rand();
   a.y = rand();
   a.z = rand();
   b.x = rand();
   b.y = rand();
   b.z = rand();
   c = a + b;
   EXPECT_EQ(c.x, a.x + b.x);
   EXPECT_EQ(c.y, a.y + b.y);
   EXPECT_EQ(c.z, a.z + b.z);
 }
}

TEST(Vector, Subtraction){
  seed_rand();
  Pose3D::Vector a,b,c;
for (unsigned int i = 0; i < 1000; i++)
 {  
   a.x = rand();
   a.y = rand();
   a.z = rand();
   b.x = rand();
   b.y = rand();
   b.z = rand();
   c = a - b;
   EXPECT_EQ(c.x, a.x - b.x);
   EXPECT_EQ(c.y, a.y - b.y);
   EXPECT_EQ(c.z, a.z - b.z);
 }
}

TEST(Position, DefaultConstructor){
  Pose3D::Position po;
  EXPECT_EQ(0, po.x);
  EXPECT_EQ(0, po.y);
  EXPECT_EQ(0, po.z);
}

TEST(Position, Addition){
  seed_rand();
  Pose3D::Position a,b,c;
for (unsigned int i = 0; i < 1000; i++)
 {  
   a.x = rand();
   a.y = rand();
   a.z = rand();
   b.x = rand();
   b.y = rand();
   b.z = rand();
   c = a + b;
   EXPECT_EQ(c.x, a.x + b.x);
   EXPECT_EQ(c.y, a.y + b.y);
   EXPECT_EQ(c.z, a.z + b.z);
 }
}

TEST(Position, Subtraction){
  seed_rand();
  Pose3D::Position a,b,c;
for (unsigned int i = 0; i < 1000; i++)
 {  
   a.x = rand();
   a.y = rand();
   a.z = rand();
   b.x = rand();
   b.y = rand();
   b.z = rand();
   c = a - b;
   EXPECT_EQ(c.x, a.x - b.x);
   EXPECT_EQ(c.y, a.y - b.y);
   EXPECT_EQ(c.z, a.z - b.z);
 }
}

TEST(Quaterion, DefaultConstructor){
  Pose3D::Quaternion quat;
  EXPECT_EQ(0, quat.x);
  EXPECT_EQ(0, quat.y);
  EXPECT_EQ(0, quat.z);
}

void testEulerConversion(double yaw, double pitch, double roll){
  Pose3D p;
  p.setFromEuler(0, 0, 0, yaw, pitch, roll);
  Pose3D::Euler e = p.getEuler();
  EXPECT_FLOAT_EQ(e.yaw, yaw);
  EXPECT_FLOAT_EQ(e.pitch, pitch);
  EXPECT_FLOAT_EQ(e.roll, roll);
}

TEST(Pose3D, ToFromEulerConversions){
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
    if (difference){
      printf("in: %.3f %.3f %.3f\n",
             yaw, pitch, roll);

      std::cout << m;


      printf("out: %.3f %.3f %.3f\n",
             out.yaw, out.pitch, out.roll);
      printf("out2: %.3f %.3f %.3f\n\n",
             out2.yaw, out2.pitch, out2.roll);
	  
      printf("FAILURE!!!!!!!!!!\n\n");
    }
  }

}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
